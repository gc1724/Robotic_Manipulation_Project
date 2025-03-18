%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dynamixel Setup and Task Placeholders
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear all;
close all;
%% -- TASK 1.a --

% This script uses the following DH table:
%
%   Frame (i) | α_(i-1)      | a_(i-1) | θ_i                     | d_i
%   ---------------------------------------------------------------
%   Frame 1   | 0            | 0       | theta1                  | 0.077
%   Frame 2   | -π/2         | 0       | theta2 - 1.385448377    | 0
%   Frame 3   | 0            | 0.13    | theta3 + 1.385448377    | 0
%   Frame 4   | 0            | 0.124   | theta4                  | 0
%   Frame 5   | 0            | 0.126   | 0                       | 0

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% UNCOMMENT THIS BLOCK FOR ACTUAL HARDWARE SETUP AND LIBRARY LOADING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', ...
        'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end
%}

%% ---- Control Table Addresses ---- %%
ADDR_PRO_TORQUE_ENABLE       = 64;           
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;
ADDR_PRO_PROFILE_VELOCITY    = 112;

%% ---- Other Settings ---- %%
PROTOCOL_VERSION            = 2.0;   
DXL_ID1                     = 11;     
DXL_ID2                     = 12;    
BAUDRATE                    = 57600;
DEVICENAME                  = 'COM5'; 

TORQUE_ENABLE               = 1; 
TORQUE_DISABLE              = 0; 
DXL_MINIMUM_POSITION_VALUE  = 0; 
DXL_MAXIMUM_POSITION_VALUE  = 4095;
DXL_MOVING_STATUS_THRESHOLD = 50; 
ESC_CHARACTER               = 'e'; 
COMM_SUCCESS                = 0; 
COMM_TX_FAIL                = -1001; 

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% UNCOMMENT THIS BLOCK FOR ACTUAL HARDWARE INITIALIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize PortHandler
port_num = portHandler(DEVICENAME);
% Initialize PacketHandler
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];
dxl_error = 0;
dxl_present_position = 0;

% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end

% ----- SET MOTION LIMITS (Torque must be off) ----------- 
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
ADDR_TORQUE_ENABLE = 64;
MAX_POS = 3100;
MIN_POS = 1000;

% Example for two motors:
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_TORQUE_ENABLE, 0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MAX_POS, MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MAX_POS, MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MIN_POS, MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MIN_POS, MIN_POS);

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Put actuators into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_OPERATING_MODE, 3);

% Set Profile Velocity
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PROFILE_VELOCITY, 10);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_PROFILE_VELOCITY, 10);

% Disable torque so we can freely read positions
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, 0);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end
%} 
% END OF HARDWARE BLOCK


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Task 1.b: Graphical Simulation of the Robot (Updated DH Parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The transformation matrix used is:
%
%     T = [ cos(theta)             -sin(theta)             0           a;
%           sin(theta)*cos(alpha)  cos(theta)*cos(alpha)   -sin(alpha) -d*sin(alpha);
%           sin(theta)*sin(alpha)  cos(theta)*sin(alpha)    cos(alpha)  d*cos(alpha);
%           0                      0                       0           1];
%
% The drawFrame function draws coordinate frames using:
%   - green arrow for the x-axis,
%   - blue arrow for the y-axis,
%   - red arrow for the z-axis.
%
% In this simulation each joint is moved individually while the others
% remain fixed. To modify a joint’s motion, change its amplitude (amp)
% or other motion parameters in the corresponding section.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Setup the Figure and Axes
figure('Name','Robot Simulation','NumberTitle','off'); 
axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Robot Simulation Using Updated DH Parameters');
xlim([-0.5, 0.5]); ylim([-0.5, 0.5]); zlim([0, 0.5]);
view(45,30);

frameScale = 0.05;

% Predefine initial joint angles (in radians)
theta1 = 0; 
theta2 = 0; 
theta3 = 0; 
theta4 = 0;  

% Pre-create the robot link plot (initial dummy data)
hRobot = plot3(0,0,0, '-o','LineWidth',2,'MarkerSize',6);

%% Define the number of steps for each joint movement and time vector
nSteps = 100;
tVec   = linspace(0, 2*pi, nSteps);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Animate Joint 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% In this section, only theta1 moves while theta2, theta3, theta4 remain constant.
amp1 = 0.5;  % Modify this value to change the movement amplitude of Joint 1
disp('Animating Joint 1. Modify amp1 in the code to change its movement amplitude.');
for i = 1:nSteps
    theta1 = amp1 * sin(tVec(i));
    theta2 = 0;  
    theta3 = 0;
    theta4 = 0;
    
    T01 = dh_transform_updated(0,       0,           theta1,                0.077);
    T12 = dh_transform_updated(-pi/2,   0,           theta2 - 1.385448377,   0);
    T23 = dh_transform_updated(0,       0.13,        theta3 + 1.385448377,   0);
    T34 = dh_transform_updated(0,       0.124,       theta4,                0);
    T45 = dh_transform_updated(0,       0.126,       0,                     0);
    
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;
    
    p0 = [0; 0; 0];
    p1 = T01(1:3,4);
    p2 = T02(1:3,4);
    p3 = T03(1:3,4);
    p4 = T04(1:3,4);
    p5 = T05(1:3,4);
    
    X = [p0(1) p1(1) p2(1) p3(1) p4(1) p5(1)];
    Y = [p0(2) p1(2) p2(2) p3(2) p4(2) p5(2)];
    Z = [p0(3) p1(3) p2(3) p3(3) p4(3) p5(3)];
    set(hRobot, 'XData', X, 'YData', Y, 'ZData', Z);
    
    hFrames = [];
    hFrames = [hFrames, drawFrame(eye(4), frameScale)];  % Base frame (frame 0)
    hFrames = [hFrames, drawFrame(T01, frameScale)];       % Frame 1
    hFrames = [hFrames, drawFrame(T02, frameScale)];       % Frame 2
    hFrames = [hFrames, drawFrame(T03, frameScale)];       % Frame 3
    hFrames = [hFrames, drawFrame(T04, frameScale)];       % Frame 4
    hFrames = [hFrames, drawFrame(T05, frameScale)];       % Frame 5
    
    drawnow;
    pause(0.05);
    delete(hFrames);
end
disp('Joint 1 animation complete. Press any key to animate Joint 2.');
pause;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Animate Joint 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now theta2 moves (using the offset built into the DH parameters),
% while theta1 remains at its final value from Joint 1 and theta3/theta4 are fixed.
amp2 = 0.5;  % Modify this value to change the movement amplitude of Joint 2
disp('Animating Joint 2. Modify amp2 in the code to change its movement amplitude.');
for i = 1:nSteps
    theta2 = amp2 * sin(tVec(i));
   
    
    T01 = dh_transform_updated(0,       0,           theta1,                0.077);
    T12 = dh_transform_updated(-pi/2,   0,           theta2 - 1.385448377,   0);
    T23 = dh_transform_updated(0,       0.13,        theta3 + 1.385448377,   0);
    T34 = dh_transform_updated(0,       0.124,       theta4,                0);
    T45 = dh_transform_updated(0,       0.126,       0,                     0);
    
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;
    
    p0 = [0; 0; 0];
    p1 = T01(1:3,4);
    p2 = T02(1:3,4);
    p3 = T03(1:3,4);
    p4 = T04(1:3,4);
    p5 = T05(1:3,4);
    
    X = [p0(1) p1(1) p2(1) p3(1) p4(1) p5(1)];
    Y = [p0(2) p1(2) p2(2) p3(2) p4(2) p5(2)];
    Z = [p0(3) p1(3) p2(3) p3(3) p4(3) p5(3)];
    set(hRobot, 'XData', X, 'YData', Y, 'ZData', Z);
    
    hFrames = [];
    hFrames = [hFrames, drawFrame(eye(4), frameScale)];
    hFrames = [hFrames, drawFrame(T01, frameScale)];
    hFrames = [hFrames, drawFrame(T02, frameScale)];
    hFrames = [hFrames, drawFrame(T03, frameScale)];
    hFrames = [hFrames, drawFrame(T04, frameScale)];
    hFrames = [hFrames, drawFrame(T05, frameScale)];
    
    drawnow;
    pause(0.05);
    delete(hFrames);
end
disp('Joint 2 animation complete. Press any key to animate Joint 3.');
pause;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Animate Joint 3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now theta3 moves while theta1 and theta2 remain at their current values.
amp3 = 0.5;  % Modify this value to change the movement amplitude of Joint 3
disp('Animating Joint 3. Modify amp3 in the code to change its movement amplitude.');
for i = 1:nSteps
    theta3 = amp3 * sin(tVec(i));
    
    T01 = dh_transform_updated(0,       0,           theta1,                0.077);
    T12 = dh_transform_updated(-pi/2,   0,           theta2 - 1.385448377,   0);
    T23 = dh_transform_updated(0,       0.13,        theta3 + 1.385448377,   0);
    T34 = dh_transform_updated(0,       0.124,       theta4,                0);
    T45 = dh_transform_updated(0,       0.126,       0,                     0);
    
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;
    
    p0 = [0; 0; 0];
    p1 = T01(1:3,4);
    p2 = T02(1:3,4);
    p3 = T03(1:3,4);
    p4 = T04(1:3,4);
    p5 = T05(1:3,4);
    
    X = [p0(1) p1(1) p2(1) p3(1) p4(1) p5(1)];
    Y = [p0(2) p1(2) p2(2) p3(2) p4(2) p5(2)];
    Z = [p0(3) p1(3) p2(3) p3(3) p4(3) p5(3)];
    set(hRobot, 'XData', X, 'YData', Y, 'ZData', Z);
    
    hFrames = [];
    hFrames = [hFrames, drawFrame(eye(4), frameScale)];
    hFrames = [hFrames, drawFrame(T01, frameScale)];
    hFrames = [hFrames, drawFrame(T02, frameScale)];
    hFrames = [hFrames, drawFrame(T03, frameScale)];
    hFrames = [hFrames, drawFrame(T04, frameScale)];
    hFrames = [hFrames, drawFrame(T05, frameScale)];
    
    drawnow;
    pause(0.05);
    delete(hFrames);
end
disp('Joint 3 animation complete. Press any key to animate Joint 4.');
pause;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Animate Joint 4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Finally, animate theta4 while the other joints remain fixed.
amp4 = 0.5;  % Modify this value to change the movement amplitude of Joint 4
disp('Animating Joint 4. Modify amp4 in the code to change its movement amplitude.');
for i = 1:nSteps
    theta4 = amp4 * sin(tVec(i));
    
    T01 = dh_transform_updated(0,       0,           theta1,                0.077);
    T12 = dh_transform_updated(-pi/2,   0,           theta2 - 1.385448377,   0);
    T23 = dh_transform_updated(0,       0.13,        theta3 + 1.385448377,   0);
    T34 = dh_transform_updated(0,       0.124,       theta4,                0);
    T45 = dh_transform_updated(0,       0.126,       0,                     0);
    
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;
    
    p0 = [0; 0; 0];
    p1 = T01(1:3,4);
    p2 = T02(1:3,4);
    p3 = T03(1:3,4);
    p4 = T04(1:3,4);
    p5 = T05(1:3,4);
    
    X = [p0(1) p1(1) p2(1) p3(1) p4(1) p5(1)];
    Y = [p0(2) p1(2) p2(2) p3(2) p4(2) p5(2)];
    Z = [p0(3) p1(3) p2(3) p3(3) p4(3) p5(3)];
    set(hRobot, 'XData', X, 'YData', Y, 'ZData', Z);
    
    hFrames = [];
    hFrames = [hFrames, drawFrame(eye(4), frameScale)];
    hFrames = [hFrames, drawFrame(T01, frameScale)];
    hFrames = [hFrames, drawFrame(T02, frameScale)];
    hFrames = [hFrames, drawFrame(T03, frameScale)];
    hFrames = [hFrames, drawFrame(T04, frameScale)];
    hFrames = [hFrames, drawFrame(T05, frameScale)];
    
    drawnow;
    pause(0.05);
    delete(hFrames);
end
disp('Joint 4 animation complete. Animation finished.');



close all;
clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- dh_transform_updated ------------------------------------------------
function T = dh_transform_updated(alpha, a, theta, d)  
    T = [ cos(theta)             -sin(theta)              0           a;
          sin(theta)*cos(alpha)  cos(theta)*cos(alpha)    -sin(alpha) -d*sin(alpha);
          sin(theta)*sin(alpha)  cos(theta)*sin(alpha)    cos(alpha)  d*cos(alpha);
          0                      0                       0           1];
end

% --- drawFrame -----------------------------------------------------------
function h = drawFrame(T, scale)
    % Draws a coordinate frame given a homogeneous transformation matrix T.
    % The frame is drawn with three quiver3 arrows:
    %   green for the x-axis, blue for the y-axis, and red for the z-axis.
    %
    %   Input:
    %     T     : 4x4 homogeneous transformation matrix for the frame
    %     scale : (optional) scaling factor for the axis vectors (default = 0.05)
    %
    %   Output:
    %     h     : handles to the three quiver objects
    
    if nargin < 2
        scale = 0.05;
    end
    
    origin = T(1:3,4);
    x_axis = T(1:3,1);
    y_axis = T(1:3,2);
    z_axis = T(1:3,3);
    
    h(1) = quiver3(origin(1), origin(2), origin(3), scale*x_axis(1), scale*x_axis(2), scale*x_axis(3),...
        'g','LineWidth',2,'MaxHeadSize',0.5);
    h(2) = quiver3(origin(1), origin(2), origin(3), scale*y_axis(1), scale*y_axis(2), scale*y_axis(3),...
        'b','LineWidth',2,'MaxHeadSize',0.5);
    h(3) = quiver3(origin(1), origin(2), origin(3), scale*z_axis(1), scale*z_axis(2), scale*z_axis(3),...
        'r','LineWidth',2,'MaxHeadSize',0.5);
end



