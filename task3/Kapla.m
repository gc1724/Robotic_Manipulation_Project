%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dynamixel Setup and Task Placeholders for Omnimanipulator-X Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear all;
close all;


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
DEVICENAME                  = 'COM4'; % Adjust to COM port

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
% insure that the grabber is within the limits
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_OPERATING_MODE, 3);
%write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_PROFILE_VELOCITY, 30);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_GOAL_POSITION, uint32(1900));
pause(1);
%set limits
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
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PROFILE_VELOCITY, 50);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_PROFILE_VELOCITY, 50);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_PROFILE_VELOCITY, 50);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_PROFILE_VELOCITY, 50);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_PROFILE_VELOCITY, 50);

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
default5 = 1900; %1024, 2673

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, uint32(default1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, uint32(default2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, uint32(default3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, uint32(default4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_GOAL_POSITION, uint32(default5));
pause(5);  % Allow time for the robot to move to its default positions


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Build dynParams Structure (avoids globals)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dynParams.port_num = port_num;
dynParams.PROTOCOL_VERSION = PROTOCOL_VERSION;
dynParams.DXL_ID1 = DXL_ID1;
dynParams.DXL_ID2 = DXL_ID2;
dynParams.DXL_ID3 = DXL_ID3;
dynParams.DXL_ID4 = DXL_ID4;
dynParams.DXL_ID5 = DXL_ID5;
dynParams.ADDR_PRO_GOAL_POSITION = ADDR_PRO_GOAL_POSITION;
dynParams.ticks_per_rad = 4096/(2*pi);
dynParams.center1 = 2047;
dynParams.center2 = 2047;
dynParams.center3 = 2047;
dynParams.center4 = 2047;
dynParams.min1 = 630;   dynParams.max1 = 3400;
dynParams.min2 = 683;   dynParams.max2 = 3075;
dynParams.min3 = 694;   dynParams.max3 = 3035;
dynParams.min4 = 854;   dynParams.max4 = 3450;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% -- TASK 3: KAPLA -- %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Create 3D Simulation Figure and Workspace Grid
figure('Name','KAPLA','NumberTitle','off');
axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('KAPLA');
view(45,30); 

% Define grid limits in robot arm base coordinates:
rows = -8:8;    % columns in grid units
cols = -2:9;    % rows in grid units
holeSpacing = 0.025;  % 25 mm spacing in meters

[colGrid, rowGrid] = meshgrid(cols, rows);
Xgrid = colGrid * holeSpacing;
Ygrid = rowGrid * holeSpacing;
Zgrid = zeros(size(Xgrid)); 

scatter3(Xgrid(:), Ygrid(:), Zgrid(:), 36, 'k', 'filled');

scatter3(0, 0, 0, 100, 'r', 'filled'); 

%% Define Cube Positions (in grid units relative to robot base)
materialFlatGridPosition = [0, -6];
materialTallGridPosition = [0, 6];

% sign: 0 no kapla, + tall kapla, - flat kapla,
%number: 1 - built from ground level, 2 - built on 1 flat kapla, 3 - built
%on 2 flat kaplas ...
%{
finalForm = [-1, 0, -1;
             2, 0, 2;
             -2, 0, -2;
             0, -3, 0;
             0, 4, 0;
             0, -4, 0];
%}

%X, Y, type of building (0 - short rbidge, 1 - tall bridge, 2 - house, 3 -complex, 4 - big houes)
% X and Y are the coordinate of the start of the building - the point
% closest to the robot
finalForm = [7, 7, 0; 
            5, 0, 2;
            7, -7, 0];
%finalForm = [9, 0, 4];
numbuildings = size(finalForm, 1);
% sign: 0 no kapla, + tall kapla, - flat kapla,
%number: 1 - built from ground level, 2 - built on 1 flat kapla, 3 - built
%on 2 flat kaplas ...
shortBridge = [1, 0, 1;
               0, -1, 0];
tallBridge = [];

house = [0, -1, 0, 0, 0, -1, 0;
         0, 2, 0, 0, 0, 2, 0;
         0, -2, 0, 0, 0, -2, 0;
         0, 0, 0, -3, 0, 0, 0;
         0, 0, 0, 4, 0, 0, 0;
         0, 0, 0, -4, 0, 0, 0];

complex = [1, 0, 1, 0, 1, 0, 1;
           0, -1, 0, 0, 0, -1, 0;
           0, 0, 2, 0, 2, 0, 0;
           0, 0, 0, -2, 0, 0, 0];

big_house = [0, -1, 0, 0, 0, -1, 0,  0, 0, -1, 0, 0, 0, -1, 0;
             0,  2, 0, 0, 0,  2, 0,  0, 0,  2, 0, 0, 0,  2, 0;
             0, -2, 0, 0, 0, -2, 0,  0, 0, -2, 0, 0, 0, -2, 0;
             0, 0, 0, -3, 0, 0, 0,  -3, 0, 0, 0, -3, 0, 0, 0;
             0, 0, 0, 4, 0, 0, 0,  4, 0, 0, 0, 4, 0, 0, 0;
             0, 0, 0, -4, 0, 0, 0,  -4, 0, 0, 0, -4, 0, 0, 0;
             0, 0, 0, 0, 0, -5, 0,  0, 0, -5, 0, 0, 0, 0, 0;
             0, 0, 0, 0, 0, 0, 0,  -6, 0, 0, 0, 0, 0, 0, 0;
             0, 0, 0, 0, 0, 0, 0,  7, 0, 0, 0, 0, 0, 0, 0;
             0, 0, 0, 0, 0, 0, 0,  -7, 0, 0, 0, 0, 0, 0, 0];





% Convert material grid position to meters 
materialElevation = 0.003;
materialPosInitial = zeros(3);
materialPosInitial(1) = materialTallGridPosition(1) * holeSpacing; 
materialPosInitial(2) = materialTallGridPosition(2) * holeSpacing;
materialPosInitial(3) = materialElevation;


%% Robot Arm Parameters and Default Setup
% DH and link parameters (meters and radians)
d1 = 0.077;   % offset from frame 0 to frame 1
L1 = 0.13;    % link length a2
L2 = 0.124;   % link length a3
L3 = 0.126;   % link length a4
phi = 0;     

theta1_default = 0; 
theta2_default = 0;
theta3_default = 0;
theta4_default = 0;
[]


% Gripper positions (joint 5): 2673 is open; 1024 is closed
gripperOpen = 1900; % make it grip 28 mm closer to root of robot arm
gripperClosed = 2260; % 2300 fully closed, but a little stress is induced
gripperState = gripperOpen;  % initial state is open

[robotX, robotY, robotZ] = computeFK(theta1_default, theta2_default, theta3_default, theta4_default, d1, L1, L2, L3);
hRobot = plot3(robotX, robotY, robotZ, '-o', 'LineWidth', 2, 'MarkerSize', 6, 'Color', [0.5 0 0.5]);
hGripperState = text(robotX(end)+0.01, robotY(end), robotZ(end), 'Gripper: Open', 'FontSize', 12, 'Color', 'm');

currentTheta1 = theta1_default;
currentTheta2 = theta2_default;
currentTheta3 = theta3_default;
currentTheta4 = theta4_default;

% Initialize default corrdinates
defaultX = L2 + L3 + 0.024;
defaultY = 0;
defaultZ = d1 + 0.128;
defaultPhi = 0;

%% Motion Parameters for Interpolation and Pick-and-Place Sequence
numSteps = 20;         
transportOffset = 0.05;  

%% Loop Over All positions of the final formh
afterDropX = defaultX;
afterDropY = defaultY;
afterDropZ = defaultZ;
afterDropPhi = defaultPhi;
kaplaLength = 0.1174;
kaplaWidth = 0.02348;
kaplaHeight = 0.0078267;

preGraspZStart = defaultZ;
x_values = [];  % Stores the X-coordinate over time
y_values = [];  % Stores the Y-coordinate over time
z_values = [];  % Stores the Z-coordinate over time
time_values = []; % Stores the corresponding time stamps

for k = 1:numbuildings
    buildingType = finalForm(k,3);
    if buildingType == 0
        buildingType = shortBridge;
    elseif buildingType == 1
        buildingType = tallBridge;
    elseif buildingType == 2
        buildingType = house;
    elseif buildingType == 3
        buildingType = complex;
    elseif buildingType == 4
	    buildingType = big_house;
    else
        disp("no such building");
        continue;
    end
    numLayers = size(buildingType, 1);
    numColumns = size(buildingType, 2);
    
    for i = 1:numLayers
        for j = 1:numColumns

            kaplaType = buildingType(i,j);
            if kaplaType == 0
                continue
            end

            heightOffset = -0.015;
            materialX = materialPosInitial(1);
            materialY = (materialPosInitial(2) + 0.002) * sign(kaplaType);
            materialZ = materialPosInitial(3) + heightOffset;
            materialPhi = pi/2;
    
            
            transportHeight = 0.13 + heightOffset;
            
            initX = finalForm(k,1)*holeSpacing - (j-1) * kaplaHeight / sqrt(finalForm(k,1) ^ 2 + finalForm(k, 2)) * finalForm(k,1);
            initY = finalForm(k,2)*holeSpacing - (j-1) * kaplaHeight / sqrt(finalForm(k,1) ^ 2 + finalForm(k, 2)) * finalForm(k,2);
            initZ = kaplaHeight * (abs(kaplaType)-1) + kaplaWidth * (i-abs(kaplaType)) + heightOffset;

            initR = sqrt(initX ^2 + initY ^ 2);
            if sign(kaplaType) == 1
                initZ = initZ + kaplaWidth / 2;
                materialZ = materialZ + kaplaWidth / 2;
                r = sqrt(initX^2 + initY^2);
                initX = (initX/r) * (r+0.001);
                initY = (initY/r) * (r+0.001);

                %initX = initX + 0.01* initX / sqrt (initX^2 + initY^2);
		        %initY = initY + 0.01* initY / sqrt (initX^2 + initY^2);

            else 
                initZ = initZ + kaplaHeight / 2;
                materialZ = materialZ + kaplaHeight / 2;
            end
            if initR > 6 * holeSpacing
                initZ = initZ + (initR - 6*holeSpacing) ^ 2 * 1.5 + 0.005;
            end
            initPhi = pi/2;
    
            % --- 1. Move above the material to pre grasp pose ---
            [currentTheta1, currentTheta2, currentTheta3, currentTheta4, preGraspX, preGraspY, preGraspZ, preGraspPhi, x_values, y_values, z_values, time_values] = interpolateArm(afterDropX, afterDropY, afterDropZ, ...
                materialX, materialY, transportHeight, afterDropPhi, materialPhi, numSteps, hRobot, hGripperState, ...
                d1, L1, L2, L3, dynParams, x_values, y_values, z_values, time_values);
            pause(1);
    
            % --- 2. Lower to Grasp Pose at material Position ---
            [currentTheta1, currentTheta2, currentTheta3, currentTheta4, graspX, graspY, graspZ, graspPhi, x_values, y_values, z_values, time_values] = interpolateArm(preGraspX, preGraspY, preGraspZ, ...
                materialX, materialY, materialZ, preGraspPhi, materialPhi, numSteps, hRobot, hGripperState, ...
                d1, L1, L2, L3, dynParams, x_values, y_values, z_values, time_values);
            pause(0.2);
    
            % --- 3. Close Gripper to Grasp the material ---
            gripperState = gripperClosed;
            set(hGripperState, 'String', 'Gripper: Closed');
            write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), ...
                double(dynParams.DXL_ID5), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(gripperState));
            desiredPos = gripperState;
            threshold = 10; % Acceptable error in encoder ticks
            maxTime = 2; 
            tic;
            while toc < maxTime
                currentPos = read4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), ...
                    double(dynParams.DXL_ID5), double(ADDR_PRO_PRESENT_POSITION));
                if abs(double(currentPos) - double(desiredPos)) < threshold
                    break;
                end
                pause(0.05);
            end
    
            % --- 4. Lift Tool to Transport Pose Above Initial Position ---
            [currentTheta1, currentTheta2, currentTheta3, currentTheta4, postGraspX, postGraspY, postGraspZ, postGraspPhi, x_values, y_values, z_values, time_values] = interpolateArm(graspX, graspY, graspZ, ...
                materialX, materialY, transportHeight, graspPhi, materialPhi, numSteps, hRobot, hGripperState, ...
                d1, L1, L2, L3, dynParams, x_values, y_values, z_values, time_values);
            pause(0.5);
    
            % --- 5. Move above the final destination of the current kapla ---
            [currentTheta1, currentTheta2, currentTheta3, currentTheta4, beforeDropX, beforeDropY, beforeDropZ, beforeDropPhi, x_values, y_values, z_values, time_values] = interpolateArm(postGraspX, postGraspY, postGraspZ, ...
                initX, initY, transportHeight, postGraspPhi, initPhi, numSteps, hRobot, hGripperState, ...
                d1, L1, L2, L3, dynParams, x_values, y_values, z_values, time_values);
            pause(1);
    
            % --- 6. Lower to the final destination of the current kapla ---
            [currentTheta1, currentTheta2, currentTheta3, currentTheta4, atDropX, atDropY, atDropZ, atDropPhi, x_values, y_values, z_values, time_values] = interpolateArm(beforeDropX, beforeDropY, beforeDropZ, ...
                initX, initY, initZ, beforeDropPhi, initPhi, numSteps, hRobot, hGripperState, ...
                d1, L1, L2, L3, dynParams, x_values, y_values, z_values, time_values);
            pause(2);
    
            % --- 7. Drop the kapla ---
            gripperState = gripperOpen;
            set(hGripperState, 'String', 'Gripper: Open');
            write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), ...
                double(dynParams.DXL_ID5), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(gripperState));
            desiredPos = gripperState;
            threshold = 10; % Acceptable error in encoder ticks
            maxTime = 5; 
            tic;
            while toc < maxTime
                currentPos = read4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), ...
                    double(dynParams.DXL_ID5), double(ADDR_PRO_PRESENT_POSITION));
                if abs(double(currentPos) - double(desiredPos)) < threshold
                    break;
                end
                pause(0.05);
            end
    
            % --- 8. Move above the final destination of the current (dropped) kapla ---
            [currentTheta1, currentTheta2, currentTheta3, currentTheta4, afterDropX, afterDropY, afterDropZ, afterDropPhi, x_values, y_values, z_values, time_values] = interpolateArm(atDropX, atDropY, atDropZ, ...
                initX, initY, transportHeight, atDropPhi, initPhi, numSteps, hRobot, hGripperState, ...
                d1, L1, L2, L3, dynParams, x_values, y_values, z_values, time_values);
            pause(1);
        end
    end
end

figure;
plot(time_values, x_values, '-o', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('X Coordinate (m)');
title('End Effector X-Coordinate Over Time');
grid on;

figure;
plot(time_values, y_values, '-o', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Y Coordinate (m)');
title('End Effector Y-Coordinate Over Time');
grid on;

figure;
plot(time_values, z_values, '-o', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Z Coordinate (m)');
title('End Effector Z-Coordinate Over Time');
grid on;


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
pause(10);  % Allow time for the robot to move to its default positions

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

function T = dh_transform_updated(alpha, a, theta, d)
    T = [ cos(theta)             -sin(theta)              0           a;
          sin(theta)*cos(alpha)  cos(theta)*cos(alpha)    -sin(alpha) -d*sin(alpha);
          sin(theta)*sin(alpha)  cos(theta)*sin(alpha)    cos(alpha)  d*cos(alpha);
          0                      0                       0           1];
end

function [theta1, theta2, theta3, theta4] = computeIK(xd, yd, zd, d1, L1, L2, L3, phi)
    % Compute the base rotation
    theta1 = atan2(yd, xd);
    
    % Compute wrist center in base coordinates:
    x_c = xd - L3 * cos(phi) * cos(theta1);
    y_c = yd - L3 * cos(phi) * sin(theta1);
    z_c = d1 - (zd + L3 * sin(phi));
    
    % Compute the effective planar distance from the base to the wrist center.
    r_wc = sqrt(x_c^2 + y_c^2);
    
    % Use the law of cosines to compute theta3_prime
    cos_theta3 = (r_wc^2 + z_c^2 - L1^2 - L2^2) / (2 * L1 * L2);
    cos_theta3 = max(min(cos_theta3, 1), -1);
    theta3_prime = acos(cos_theta3);
    
    % Compute theta2_prime using the geometry of the triangle
    theta2_prime = atan2(z_c, r_wc) - atan2(L2 * sin(theta3_prime), L1 + L2 * cos(theta3_prime));
    
    % Apply offsets (from your DH parameter adjustments)
    theta2 = theta2_prime + 1.385448377;
    theta3 = theta3_prime - 1.385448377;
    
    % Compute the wrist (end-effector) angle
    theta4 = phi - (theta2_prime + theta3_prime);
end

function [robotX, robotY, robotZ] = computeFK(theta1, theta2, theta3, theta4, d1, L1, L2, L3)
    T01 = dh_transform_updated(0, 0, theta1, d1);
    T12 = dh_transform_updated(-pi/2, 0, theta2 - 1.385448377, 0);
    T23 = dh_transform_updated(0, L1, theta3 + 1.385448377, 0);
    T34 = dh_transform_updated(0, L2, theta4, 0);
    T45 = dh_transform_updated(0, L3, 0, 0);
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;
    p0 = [0;0;0];
    p1 = T01(1:3,4);
    p2 = T02(1:3,4);
    p3 = T03(1:3,4);
    p4 = T04(1:3,4);
    p5 = T05(1:3,4);
    robotX = [p0(1), p1(1), p2(1), p3(1), p4(1), p5(1)];
    robotY = [p0(2), p1(2), p2(2), p3(2), p4(2), p5(2)];
    robotZ = [p0(3), p1(3), p2(3), p3(3), p4(3), p5(3)];
end

function [theta1, theta2, theta3, theta4, finalX, finalY, finalZ, finalPhi, x_values, y_values, z_values, time_values] = interpolateArm(initX, initY, initZ, finalX, finalY, finalZ, ...
                            initPhi, finalPhi, numSteps, hRobot, hGripperState, d1, L1, L2, L3, dynParams, x_values, y_values, z_values, time_values)
    stepX = (finalX - initX) / numSteps;
    stepY = (finalY - initY) / numSteps;
    stepZ = (finalZ - initZ) / numSteps;
    stepPhi = (finalPhi - initPhi) / numSteps;
    currentX = initX;
    currentY = initY;
    currentZ = initZ;
    currentPhi = initPhi;
    for i = 1:numSteps+1
        
        %{
        % Update the robot display using forward kinematics.
        set(hRobot, 'XData', currentX, 'YData', currentY, 'ZData', currentZ);
        set(hGripperState, 'Position', [currentX+0.01, currentY, currentZ]);
        drawnow;
        %}
        % Append the Z-coordinate and timestamp
         % Measure elapsed time since start
        %pause(0.02);

        [theta1_interp, theta2_interp, theta3_interp, theta4_interp] = computeIK(currentX, currentY, currentZ, d1, L1, L2, L3, currentPhi);
        
        currentX = currentX + stepX;
        currentY = currentY + stepY;
        currentZ = currentZ + stepZ;
        currentPhi = currentPhi + stepPhi;
    
        ticks1 = round(dynParams.center1 + theta1_interp * dynParams.ticks_per_rad);
        ticks1 = max(dynParams.min1, min(dynParams.max1, ticks1));
        ticks2 = round(dynParams.center2 + theta2_interp * dynParams.ticks_per_rad);
        ticks2 = max(dynParams.min2, min(dynParams.max2, ticks2));
        ticks3 = round(dynParams.center3 + theta3_interp * dynParams.ticks_per_rad);
        ticks3 = max(dynParams.min3, min(dynParams.max3, ticks3));
        ticks4 = round(dynParams.center4 + theta4_interp * dynParams.ticks_per_rad);
        ticks4 = max(dynParams.min4, min(dynParams.max4, ticks4));

        x_values = [x_values, ticks2];
        y_values = [y_values, ticks3];
        z_values = [z_values, ticks4];
        time_values = [time_values, toc];
        
        write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), ...
            double(dynParams.DXL_ID1), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(ticks1));
        write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), ...
            double(dynParams.DXL_ID2), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(ticks2));
        write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), ...
            double(dynParams.DXL_ID3), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(ticks3));
        write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), ...
            double(dynParams.DXL_ID4), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(ticks4));
    
        
    end
    
    theta1 = theta1_interp;
    theta2 = theta2_interp;
    theta3 = theta3_interp;
    theta4 = theta4_interp;


end


function pos = getEndEffectorPos(theta1, theta2, theta3, theta4, d1, L1, L2, L3)
    T01 = dh_transform_updated(0, 0, theta1, d1);
    T12 = dh_transform_updated(-pi/2, 0, theta2 - 1.385448377, 0);
    T23 = dh_transform_updated(0, L1, theta3 + 1.385448377, 0);
    T34 = dh_transform_updated(0, L2, theta4, 0);
    T45 = dh_transform_updated(0, L3, 0, 0);
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;
    pos = T05(1:3,4);
end
