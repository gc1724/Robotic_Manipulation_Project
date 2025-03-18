clc;
clear all;
close all;

%% -- TASK 1.a --

% This script uses the following DH table:
%
%   Frame (i) | α_(i-1)      | a_(i-1) | θ_i                     | d_i
%   ---------------------------------------------------------------
%   Frame 1   | 0            | 0       | theta1                  | 0.077
%   Frame 2   | -π/2         | 0       | theta2 - 1.385448377     | 0
%   Frame 3   | 0            | 0.13    | theta3 + 1.385448377     | 0
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
%}
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', ...
        'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% ---- Control Table Addresses ---- %%
ADDR_PRO_TORQUE_ENABLE       = 64;    % Address for enabling torque
ADDR_PRO_GOAL_POSITION       = 116;   % Address for goal position
ADDR_PRO_PRESENT_POSITION    = 132;   % Address for present position
ADDR_PRO_OPERATING_MODE      = 11;    % Address for operating mode
ADDR_PRO_PROFILE_VELOCITY    = 112;   % Address for profile velocity

%% ---- Other Settings ---- %%
PROTOCOL_VERSION            = 2.0;    
% Motor IDs for joints:
DXL_ID1                     = 11;     % Joint 1 (rotational): Allowed [630,3400]
DXL_ID2                     = 12;     % Joint 2: Allowed [683,3075]
DXL_ID3                     = 13;     % Joint 3: Allowed [694,3035]
DXL_ID4                     = 14;     % Joint 4: Allowed [854,3450]
DXL_ID5                     = 15;     % Grabber: Allowed [1024,2673]

BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM4'; 

TORQUE_ENABLE               = 1; 
TORQUE_DISABLE              = 0; 
DXL_MOVING_STATUS_THRESHOLD = 50; 
ESC_CHARACTER               = 'e'; 
COMM_SUCCESS                = 0; 
COMM_TX_FAIL                = -1001; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% HARDWARE INITIALIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize PortHandler
port_num = portHandler(DEVICENAME);
% Initialize PacketHandler
packetHandler();

dxl_comm_result = COMM_TX_FAIL;
dxl_error = 0;

% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SET MOTION LIMITS FOR EACH JOINT (Torque must be off)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set the control table addresses for limits
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;

% --- Joint 1 ---
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, 0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MAX_POS, 3400);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MIN_POS, 630);

% --- Joint 2 ---
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, 0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MAX_POS, 3075);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MIN_POS, 683);

% --- Joint 3 ---
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_TORQUE_ENABLE, 0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_MAX_POS, 3035);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_MIN_POS, 694);

% --- Joint 4 ---
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_TORQUE_ENABLE, 0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_MAX_POS, 3450);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_MIN_POS, 854);

% --- Grabber (Joint 5) ---
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_TORQUE_ENABLE, 0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_MAX_POS, 2673);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_MIN_POS, 1024);

% Put actuators into Position Control Mode for all joints
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_OPERATING_MODE, 3);

% Set Profile Velocity for all joints
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PROFILE_VELOCITY, 15);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_PROFILE_VELOCITY, 15);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_PROFILE_VELOCITY, 15);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_PROFILE_VELOCITY, 15);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_PROFILE_VELOCITY, 15);

% Enable Torque for all joints
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel successfully connected\n');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SET DEFAULT POSITIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Default positions (encoder ticks):
% For joints 1-4: 2047, for grabber (joint 5): 1024
default1 = 2047;
default2 = 2047;
default3 = 2047; 
default4 = 2047;
default5 = 1024;

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, uint32(default1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, uint32(default2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, uint32(default3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, uint32(default4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_GOAL_POSITION, uint32(default5));
pause(10);  % Allow time for the robot to move to its default positions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% -- TASK 1.d: Inverse Kinematics and Square Trajectory --
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Setup the Figure and Axes for Simulation
figure('Name','Robot Simulation','NumberTitle','off'); 
axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Robot Simulation Using Updated DH Parameters');
xlim([-0.5, 0.5]); ylim([-0.5, 0.5]); zlim([0, 0.5]);
view(45,30);

frameScale = 0.05;

% Pre-create the robot link plot (dummy initial data)
hRobot = plot3(0,0,0, '-o','LineWidth',2,'MarkerSize',6);

%% Define Conversion Parameters for Motor Commands
% a full revolution corresponds to 4096 ticks.
ticks_per_rad = 4096/(2*pi);  % ≈ 652 ticks per radian

% Default (center) encoder values for joints 1–4:
center1 = 2047;
center2 = 2047;
center3 = 2047;
center4 = 2047;

% Allowed encoder limits for each joint:
min1 = 630;   max1 = 3400;
min2 = 683;   max2 = 3075;
min3 = 694;   max3 = 3035;
min4 = 854;   max4 = 3450;

%% Define Square Trajectory Parameters and Link Dimensions
% Link parameters 
d1 = 0.077;   % offset from frame 0 to frame 1
L1 = 0.13;    % link length (a2)
L2 = 0.124;   % link length (a3)
L3 = 0.126;   % link length (a4)
phi = 0;      % Desired final gripper orientation (in radians)

side = 0.10;  % Square side length in meters (10 cm)

% 1. XY plane (z constant)
centerXY = [0.2, 0, 0.2]; 
cornersXY = [ centerXY(1)-side/2, centerXY(2)-side/2, centerXY(3);
              centerXY(1)+side/2, centerXY(2)-side/2, centerXY(3);
              centerXY(1)+side/2, centerXY(2)+side/2, centerXY(3);
              centerXY(1)-side/2, centerXY(2)+side/2, centerXY(3);
              centerXY(1)-side/2, centerXY(2)-side/2, centerXY(3) ];  

% 2. XZ plane (y constant)
centerXZ = [0.2, 0, 0.2]; 
cornersXZ = [ centerXZ(1)-side/2, centerXZ(2), centerXZ(3)-side/2;
              centerXZ(1)+side/2, centerXZ(2), centerXZ(3)-side/2;
              centerXZ(1)+side/2, centerXZ(2), centerXZ(3)+side/2;
              centerXZ(1)-side/2, centerXZ(2), centerXZ(3)+side/2;
              centerXZ(1)-side/2, centerXZ(2), centerXZ(3)-side/2 ];

% 3. YZ plane (x constant)
centerYZ = [0.2, 0, 0.2]; 
cornersYZ = [ centerYZ(1), centerYZ(2)-side/2, centerYZ(3)-side/2;
              centerYZ(1), centerYZ(2)+side/2, centerYZ(3)-side/2;
              centerYZ(1), centerYZ(2)+side/2, centerYZ(3)+side/2;
              centerYZ(1), centerYZ(2)-side/2, centerYZ(3)+side/2;
              centerYZ(1), centerYZ(2)-side/2, centerYZ(3)-side/2 ];

squareTrajs = {cornersXY, cornersXZ, cornersYZ};
squareColors = {'r', 'g', 'b'}; 

ptsPerEdge = 20;

%% Loop Over Each Square Trajectory
for idx = 1:length(squareTrajs)
    trajCorners = squareTrajs{idx};
    color = squareColors{idx};
    
    trajPoints = [];
    for j = 1:size(trajCorners, 1)-1
        p_start = trajCorners(j, :);
        p_end   = trajCorners(j+1, :);
        for t = linspace(0,1,ptsPerEdge)
            trajPoints = [trajPoints; (1-t)*p_start + t*p_end];
        end
    end
    
    hSquare = plot3(trajPoints(:,1), trajPoints(:,2), trajPoints(:,3), ...
                    'Color', color, 'LineWidth', 2);
    
    %% Move to the Starting Position of the Square
    % Use the first waypoint to compute joint angles and command the robot.
    start_pt = trajPoints(1, :);
    xd = start_pt(1); yd = start_pt(2); zd = start_pt(3);
    % Inverse kinematics for starting point:
    theta1 = atan2(yd, xd);
    r = sqrt(xd^2 + yd^2);
    x4_1 = r - L3*cos(phi);
    z4_1 = d1 - zd - L3*sin(phi);
    cos_theta3 = (x4_1^2 + z4_1^2 - L1^2 - L2^2) / (2*L1*L2);
    cos_theta3 = max(min(cos_theta3,1),-1);  % Clamp for safety
    theta3_prime = acos(cos_theta3);           % Choose elbow-up solution
    theta2_prime = atan2(z4_1, x4_1) - atan2(L2*sin(theta3_prime), L1 + L2*cos(theta3_prime));
    theta2 = theta2_prime + 1.385448377;
    theta3 = theta3_prime - 1.385448377;
    theta4 = phi - (theta2_prime + theta3_prime);
    
    ticks1 = round(center1 + theta1 * ticks_per_rad); ticks1 = max(min1, min(max1, ticks1));
    ticks2 = round(center2 + theta2 * ticks_per_rad); ticks2 = max(min2, min(max2, ticks2));
    ticks3 = round(center3 + theta3 * ticks_per_rad); ticks3 = max(min3, min(max3, ticks3));
    ticks4 = round(center4 + theta4 * ticks_per_rad); ticks4 = max(min4, min(max4, ticks4));
    
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, uint32(ticks1));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, uint32(ticks2));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, uint32(ticks3));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, uint32(ticks4));
    pause(2);  
    
    %% Animate the Trajectory: For Each Waypoint, Compute IK and Send Commands
    for k = 1:size(trajPoints,1)
        xd = trajPoints(k,1);
        yd = trajPoints(k,2);
        zd = trajPoints(k,3);
        
        % Inverse kinematics:
        theta1 = atan2(yd, xd);
        r = sqrt(xd^2 + yd^2);
        x4_1 = r - L3*cos(phi);
        z4_1 = d1 - zd - L3*sin(phi);
        cos_theta3 = (x4_1^2 + z4_1^2 - L1^2 - L2^2) / (2*L1*L2);
        cos_theta3 = max(min(cos_theta3,1),-1);
        theta3_prime = acos(cos_theta3);
        theta2_prime = atan2(z4_1, x4_1) - atan2(L2*sin(theta3_prime), L1 + L2*cos(theta3_prime));
        theta2 = theta2_prime + 1.38217994;
        theta3 = theta3_prime - 1.38217994;
        theta4 = phi - (theta2_prime + theta3_prime);
        
        T01 = dh_transform_updated(0,       0,           theta1,                d1);
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
        
        ticks1 = round(center1 + theta1 * ticks_per_rad); ticks1 = max(min1, min(max1, ticks1));
        ticks2 = round(center2 + theta2 * ticks_per_rad); ticks2 = max(min2, min(max2, ticks2));
        ticks3 = round(center3 + theta3 * ticks_per_rad); ticks3 = max(min3, min(max3, ticks3));
        ticks4 = round(center4 + theta4 * ticks_per_rad); ticks4 = max(min4, min(max4, ticks4));
        
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, uint32(ticks1));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, uint32(ticks2));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, uint32(ticks3));
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, uint32(ticks4));
        
        drawnow;
        pause(0.05);  % Adjust animation speed 
    end
    pause(4);  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% RETURN TO HOLD POSITION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
default1 = 2047;
default2 = 705;
default3 = 2970; 
default4 = 2480;
default5 = 2673;

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, uint32(default1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, uint32(default2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, uint32(default3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, uint32(default4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_GOAL_POSITION, uint32(default5));
pause(10);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SHUTDOWN: Disable Torque and Close the Port
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port and unload the library
closePort(port_num);
fprintf('Port Closed \n');
unloadlibrary(lib_name);

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
