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

%% Choose Dynamixel Library
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

%% Control Table Addresses
ADDR_PRO_TORQUE_ENABLE    = 64;   % Enable torque
ADDR_PRO_GOAL_POSITION    = 116;  % Goal position
ADDR_PRO_PRESENT_POSITION = 132;  % Present position
ADDR_PRO_OPERATING_MODE   = 11;   % Operating mode
ADDR_PRO_PROFILE_VELOCITY = 112;  % Profile velocity

%% Other Settings
PROTOCOL_VERSION = 2.0;

% Motor IDs for joints
DXL_ID1 = 11;  
DXL_ID2 = 12;  
DXL_ID3 = 13;  
DXL_ID4 = 14;  
DXL_ID5 = 15;  

BAUDRATE   = 1000000;
DEVICENAME = 'COM4'; 

TORQUE_ENABLE  = 1; 
TORQUE_DISABLE = 0; 
COMM_SUCCESS   = 0; 
COMM_TX_FAIL   = -1001; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% HARDWARE INITIALIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
port_num = portHandler(DEVICENAME);
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

% Set baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SET MOTION LIMITS (Torque must be off)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;

% Joint 1
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, 0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MAX_POS, 3400);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MIN_POS, 630);

% Joint 2
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, 0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MAX_POS, 3075);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MIN_POS, 683);

% Joint 3
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_TORQUE_ENABLE, 0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_MAX_POS, 3035);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_MIN_POS, 694);

% Joint 4
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_TORQUE_ENABLE, 0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_MAX_POS, 3450);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_MIN_POS, 854);

% Grabber (Joint 5)
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_GOAL_POSITION, uint32(1900));
pause(1);

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_TORQUE_ENABLE, 0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_MAX_POS, 2673);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_MIN_POS, 1024);

% Put actuators in Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_OPERATING_MODE, 3);

% Set profile velocities
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PROFILE_VELOCITY, 80);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_PROFILE_VELOCITY, 80);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_PROFILE_VELOCITY, 80);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_PROFILE_VELOCITY, 80);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_PROFILE_VELOCITY, 60);

% Enable Torque
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
default1 = 2047;
default2 = 2047;
default3 = 2047;
default4 = 2047;
default5 = 1900;

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, uint32(default1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, uint32(default2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3, ADDR_PRO_GOAL_POSITION, uint32(default3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4, ADDR_PRO_GOAL_POSITION, uint32(default4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5, ADDR_PRO_GOAL_POSITION, uint32(default5));
pause(5);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Build dynParams Structure
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dynParams.port_num                  = port_num;
dynParams.PROTOCOL_VERSION          = PROTOCOL_VERSION;
dynParams.DXL_ID1                   = DXL_ID1;
dynParams.DXL_ID2                   = DXL_ID2;
dynParams.DXL_ID3                   = DXL_ID3;
dynParams.DXL_ID4                   = DXL_ID4;
dynParams.DXL_ID5                   = DXL_ID5;
dynParams.ADDR_PRO_GOAL_POSITION    = ADDR_PRO_GOAL_POSITION;
dynParams.ADDR_PRO_PRESENT_POSITION = ADDR_PRO_PRESENT_POSITION;
dynParams.ticks_per_rad             = 4096/(2*pi);
dynParams.center1                   = 2047;
dynParams.center2                   = 2047;
dynParams.center3                   = 2047;
dynParams.center4                   = 2047;
dynParams.min1 = 630;   dynParams.max1 = 3600; 
dynParams.min2 = 683;   dynParams.max2 = 3275; 
dynParams.min3 = 694;   dynParams.max3 = 3235; 
dynParams.min4 = 854;   dynParams.max4 = 3650; 
dynParams.isAppFront = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TASK 2.B: Stacking Cubes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Create 3D Simulation Figure and Workspace Grid
figure('Name','Task 2.b: Stacking Cubes','NumberTitle','off');
axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Task 2.b: Stack Cubes on a Single Holder, Red Faces Outward');
view(45,30);

rows = -8:8;    
cols = -2:9;    
holeSpacing = 0.025;  

[colGrid, rowGrid] = meshgrid(cols, rows);
Xgrid = colGrid * holeSpacing;
Ygrid = rowGrid * holeSpacing;
Zgrid = zeros(size(Xgrid));

scatter3(Xgrid(:), Ygrid(:), Zgrid(:), 36, 'k', 'filled');  % grid
scatter3(0, 0, 0, 100, 'r', 'filled'); % base

%% Define Cube Positions
cubeInitialGridPositions = [0, 8, pi/6, 2, 0;
                            5, 5, pi/6, -1, 0;
                            8, 3, pi/6, -1, 1];
numCubes = size(cubeInitialGridPositions,1);

IntermediateManipPosition = [5, -5];
stackRow         = 0;
stackCol         = -6;
stackOrientation = pi/2;

cubeFinalGridPositions = repmat([stackRow, stackCol, stackOrientation], numCubes, 1);

cubeElevation             = 0.0325;
offsetMagnitudeFrontApF   = -0.017;
offsetMagnitudeBotApF     = 0.02;
offsetMagnitudeFrontApT   = 0.015;
offsetMagnitudeBotApT     = 0.023;
offsetMagnitudeBot45App   = 0.033;
offsetMagnitudeFront45App = 0;
offsetMagnitudeBotPi6App  = 0.015;
offsetMagnitudeFrontPi6App = -0.02;
offsetMagnitudeBotPi8App = 0.03;
offsetMagnitudeFrontPi8App = 0.03;
X_int = IntermediateManipPosition(1) * holeSpacing; 
Y_int = IntermediateManipPosition(2) * holeSpacing;
r_int = sqrt(X_int^2 + Y_int^2);


% Compute actual (x,y,z) for initial/final cubes
cubePosInitial = zeros(numCubes,3);
cubePosFinal   = zeros(numCubes,3);
phi_initial = pi/2;
phi_final = pi/2;
for i = 1:numCubes
    x_initial = cubeInitialGridPositions(i,1) * holeSpacing;
    y_initial = cubeInitialGridPositions(i,2) * holeSpacing;
    r_initial = sqrt(x_initial^2 + y_initial^2);
    if r_initial ~= 0
        [r_new, z_init] = computeInitialOffsets(cubeInitialGridPositions(i,3), r_initial, ...
            cubeElevation, offsetMagnitudeFrontApT, offsetMagnitudeBotApT, ...
            offsetMagnitudeFrontApF, offsetMagnitudeBotApF, offsetMagnitudeFront45App, offsetMagnitudeBot45App, offsetMagnitudeBotPi6App, offsetMagnitudeFrontPi6App, holeSpacing);
        cubePosInitial(i,1) = (x_initial / r_initial) * r_new;
        cubePosInitial(i,2) = (y_initial / r_initial) * r_new;
        cubePosInitial(i,3) = z_init;
    else
        fprintf('Error in initial position (r_initial=0)\n');
    end
    
    x_final = cubeFinalGridPositions(i,1) * holeSpacing;
    y_final = cubeFinalGridPositions(i,2) * holeSpacing;
    r_final = sqrt(x_final^2 + y_final^2);
    if r_final ~= 0
        [r_new_final, z_final] = computeFinalOffsets(phi_initial, phi_final, ...
            r_final, cubeElevation, ...
            offsetMagnitudeFrontApT, offsetMagnitudeBotApT, ...
            offsetMagnitudeFrontApF, offsetMagnitudeBotApF, offsetMagnitudeFront45App, offsetMagnitudeBot45App, offsetMagnitudeBotPi6App, offsetMagnitudeFrontPi6App, holeSpacing);
        cubePosFinal(i,1) = (x_final / r_final) * r_new_final;
        cubePosFinal(i,2) = (y_final / r_final) * r_new_final;
        cubePosFinal(i,3) = z_final;
    else
        fprintf('Error in final position (r_final=0)\n');
    end
end

hCubeMarkers = gobjects(numCubes,1);
for i = 1:numCubes
    hCubeMarkers(i) = scatter3(cubePosInitial(i,1), cubePosInitial(i,2), cubePosInitial(i,3), ...
        100, 'g', 'filled');
end
cubeAttached = false(numCubes, 1);

%% Robot Arm Parameters
d1  = 0.077;
L1  = 0.13;
L2  = 0.124;
L3  = 0.126;
phi = 0;  

theta1_default = 0; 
theta2_default = 0;
theta3_default = 0;
theta4_default = 0;

gripperOpen   = 1900; 
gripperClosed = 2300;

[robotX, robotY, robotZ] = computeFK(theta1_default, theta2_default, ...
                                     theta3_default, theta4_default, ...
                                     d1, L1, L2, L3);
hRobot = plot3(robotX, robotY, robotZ, '-o', ...
    'LineWidth', 2, 'MarkerSize', 6, 'Color', [0.5 0 0.5]);
hGripperState = text(robotX(end)+0.01, robotY(end), robotZ(end), ...
    'Gripper: Open', 'FontSize', 12, 'Color', 'm');

currentTheta1 = theta1_default;
currentTheta2 = theta2_default;
currentTheta3 = theta3_default;
currentTheta4 = theta4_default;

%numSteps       = 30;
transportOffset= 0.15;  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MAIN LOOP: PICK, (ROTATE), PLACE FOR EACH CUBE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:numCubes
    [initX, initY, finalX, finalY, ...
     phi_start, phi_final, ...
     graspZ_start, graspZ_final, ...
     preGraspZ, rotation_count] = getCubeParams(i, ...
         cubePosInitial, cubePosFinal, ...
         cubeInitialGridPositions, cubeFinalGridPositions, ...
         cubeElevation, transportOffset);

    % 1) Grab the cube
    if cubeInitialGridPositions(i, 5) == 1
        [currentTheta1, currentTheta2, currentTheta3, currentTheta4, currX, currY, currZ] = approachFront( ...
            currentTheta1, currentTheta2, currentTheta3, currentTheta4, ...
            initX, initY, graspZ_start, preGraspZ, ...
            phi_start, ...
            hRobot, hGripperState, d1, L1, L2, L3, dynParams, gripperClosed, ADDR_PRO_PRESENT_POSITION);
    else
        [currentTheta1, currentTheta2, currentTheta3, currentTheta4, cubeAttached, currX, currY, currZ] = pickCube( ...
        currentTheta1, currentTheta2, currentTheta3, currentTheta4, ...
        i, initX, initY, graspZ_start, preGraspZ, phi, phi_start, ...
        hCubeMarkers, cubeAttached, gripperClosed, ...
        hRobot, hGripperState, d1, L1, L2, L3, dynParams );
    end

    % 2) Rotate if needed
    [currentTheta1, currentTheta2, currentTheta3, currentTheta4, current_phi] = rotateCubeIfNeeded( ...
        currentTheta1, currentTheta2, currentTheta3, currentTheta4, ...
        rotation_count, ...
        X_int, Y_int, r_int,...
        offsetMagnitudeFrontApT, offsetMagnitudeBotApT, ...
        offsetMagnitudeFrontApF, offsetMagnitudeBotApF, offsetMagnitudeFront45App, offsetMagnitudeBot45App, offsetMagnitudeBotPi6App, offsetMagnitudeFrontPi6App, ...
        holeSpacing, cubeElevation, phi_start, ...
        hRobot, hGripperState, d1, L1, L2, L3, dynParams, ...
        gripperOpen, gripperClosed, ADDR_PRO_PRESENT_POSITION);

    % 3) Place the cube (stackFlag = 1 for stacking)
    stackFlag = 1;
    [currentTheta1, currentTheta2, currentTheta3, currentTheta4, cubeAttached] = placeCube( ...
        currentTheta1, currentTheta2, currentTheta3, currentTheta4, ...
        i, finalX, finalY, graspZ_final, preGraspZ, stackOrientation, current_phi, ...
        hCubeMarkers, cubeAttached, gripperOpen, stackFlag, ...
        hRobot, hGripperState, d1, L1, L2, L3, dynParams );

    phi = stackOrientation;
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
%% SHUTDOWN
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

closePort(port_num);
fprintf('Port Closed \n');
unloadlibrary(lib_name);

%--------------------------------------------------------------------------
% LOCAL FUNCTIONS
%--------------------------------------------------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1) getCubeParams
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [initX, initY, finalX, finalY, phi_start, phi_final, ...
          graspZ_start, graspZ_final, preGraspZ, rotation_count] = getCubeParams(i, ...
    cubePosInitial, cubePosFinal, ...
    cubeInitialGridPositions, cubeFinalGridPositions, ...
    cubeElevation, transportOffset)

initX  = cubePosInitial(i,1);
initY  = cubePosInitial(i,2);
finalX = cubePosFinal(i,1);
finalY = cubePosFinal(i,2);

phi_start  = cubeInitialGridPositions(i,3);
phi_final  = cubeFinalGridPositions(i,3);

graspZ_start   = cubePosInitial(i,3);
graspZ_final   = cubePosFinal(i,3);
preGraspZ      = cubeElevation + transportOffset;
rotation_count = cubeInitialGridPositions(i,4);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) pickCube
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [th1, th2, th3, th4, cubeAttached, currX, currY, currZ] = pickCube( ...
    th1_init, th2_init, th3_init, th4_init, ...
    cubeIndex, initX, initY, graspZ, preGraspZ, ...
    phi_global, phi_start, ...
    hCubeMarkers, cubeAttached, gripperClosed, ...
    hRobot, hGripperState, ...
    d1, L1, L2, L3, dynParams)

% -- Move to pre-grasp pose above initial cube --
[th1, th2, th3, th4] = interpolateArm( ...
    th1_init, th2_init, th3_init, th4_init, ...
    initX, initY, preGraspZ, phi_global, phi_start, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams);

% -- Lower to grasp pose --
[th1, th2, th3, th4] = interpolateArm( ...
    th1, th2, th3, th4, ...
    initX, initY, graspZ, phi_start, phi_start, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams);

% -- Close gripper --
pause(0.5);
waitForGripper(dynParams, gripperClosed, hGripperState, dynParams.ADDR_PRO_GOAL_POSITION);
pause(0.5);


% -- Mark the cube as attached and update marker --
cubeAttached(cubeIndex) = true;
endEffectorPos = getEndEffectorPos(th1, th2, th3, th4, d1, L1, L2, L3);
set(hCubeMarkers(cubeIndex), 'XData', endEffectorPos(1), ...
                             'YData', endEffectorPos(2), ...
                             'ZData', endEffectorPos(3));

% -- Lift cube back to pre-grasp (transport) height --
[th1, th2, th3, th4, currX, currY, currZ] = interpolateArm( ...
    th1, th2, th3, th4, ...
    initX, initY, preGraspZ, phi_start, phi_start, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3) rotateCubeIfNeeded
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [th1, th2, th3, th4, current_phi] = rotateCubeIfNeeded( ...
    th1, th2, th3, th4, ...
    rotation_count, ...
    X_int, Y_int, r_int, ...
    offsetFrontApT, offsetBotApT, ...
    offsetFrontApF, offsetBotApF, offsetMagnitudeFront45App, offsetMagnitudeBot45App, offsetMagnitudeBotPi6App, offsetMagnitudeFrontPi6App, ...
    holeSpacing, cubeElevation, current_phi, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams, ...
    gripperOpen, gripperClosed, ADDR_PRO_PRESENT_POSITION)

if rotation_count == 0 && current_phi == pi/2
    return;
end

safeHeight = cubeElevation + 0.15;  
if rotation_count > 0
    extra_offset = 0.017;
else
    extra_offset = -0.007;
end

%[th1, th2, th3, th4] = computeIK(currX, currY, currZ, d1, L1, L2, L3, current_phi);

% --- Helper inline function for jiggle ---
    function [] = Jiggle(dynParams, iterations, offset)
        current_ticks1 = read4ByteTxRx(dynParams.port_num, dynParams.PROTOCOL_VERSION, dynParams.DXL_ID1, dynParams.ADDR_PRO_PRESENT_POSITION);

        for i=1:iterations
            write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), double(dynParams.DXL_ID1), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(current_ticks1 + offset));
            pause(0.5)
            write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), double(dynParams.DXL_ID1), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(current_ticks1 - offset));
            pause(0.5)
        end
        write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), double(dynParams.DXL_ID1), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(current_ticks1));
        pause(0.5)
    end

% --- Helper inline function for intermediate reorientation ---
    function [th1, th2, th3, th4, current_phi] = reorientViaHolder(th1, th2, th3, th4, current_phi, desired_phi)
        % Compute adjusted (X,Y) when placing the block.
        [r_place, z_place] = computeFinalOffsets(current_phi, current_phi, ...
            r_int, cubeElevation, offsetFrontApT, offsetBotApT, offsetFrontApF, offsetBotApF, offsetMagnitudeFront45App, offsetMagnitudeBot45App, offsetMagnitudeBotPi6App, offsetMagnitudeFrontPi6App, holeSpacing);
        X_place = (X_int / r_int) * (r_place); %- 0.0002
        Y_place = (Y_int / r_int) * (r_place);
        % Move to intermediate holder at safe height.
        [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
            X_place, Y_place, safeHeight, current_phi, current_phi, ...
            hRobot, hGripperState, d1, L1, L2, L3, dynParams);
        pause(0.5);
        % Lower block to table level.
        [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
            X_place, Y_place, z_place, current_phi, current_phi, ...
            hRobot, hGripperState, d1, L1, L2, L3, dynParams);
        % Open gripper to release block.
        pause(0.5);
        waitForGripper(dynParams, gripperOpen, hGripperState, ADDR_PRO_PRESENT_POSITION);
        pause(0.5);
        Jiggle(dynParams, 3, 18);
        % Move to intermediate holder at safe height
        [r_pick, z_pick] = computeInitialOffsets(desired_phi, r_int, cubeElevation, ...
            offsetFrontApT, offsetBotApT, offsetFrontApF, offsetBotApF, offsetMagnitudeFront45App, offsetMagnitudeBot45App, offsetMagnitudeBotPi6App, offsetMagnitudeFrontPi6App, holeSpacing);
        X_pick = (X_int / r_int) * r_pick;
        Y_pick = (Y_int / r_int) * r_pick;
        [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
            X_pick, Y_pick, safeHeight, current_phi, current_phi, ...
            hRobot, hGripperState, d1, L1, L2, L3, dynParams);
        [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
            X_pick, Y_pick, safeHeight, current_phi, desired_phi, ...
            hRobot, hGripperState, d1, L1, L2, L3, dynParams);
        pause(0.5);
        % Now, pick up the block with the desired orientation.
        [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
            X_pick, Y_pick, z_pick, desired_phi, desired_phi, ...
            hRobot, hGripperState, d1, L1, L2, L3, dynParams);
        % Close gripper to grasp the block.
        pause(0.5);
        waitForGripper(dynParams, gripperClosed, hGripperState, ADDR_PRO_PRESENT_POSITION);
        pause(0.5);
        % Lift the block to safe height.
        [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
            X_pick, Y_pick, safeHeight, desired_phi, desired_phi, ...
            hRobot, hGripperState, d1, L1, L2, L3, dynParams);
        current_phi = desired_phi;
    end

% --- Main Rotation Cycle ---
while rotation_count ~= 0
    % For a positive rotation, the gripper must start at π/2;
    % for a negative rotation, it must start at 0.
    if current_phi == 0
        
    end


    if rotation_count > 0
        required_start = pi/2;
    else
        required_start = pi/6;
    end

    % If block is not held in the required starting pose, reorient it.
    if abs(current_phi - required_start) > 0.003
        [th1, th2, th3, th4, current_phi] = reorientViaHolder(th1, th2, th3, th4, current_phi, required_start);
    % Move to starting position
    else
        [r_place, z_place] = computeFinalOffsets(current_phi, current_phi, ...
            r_int, cubeElevation, offsetFrontApT, offsetBotApT, offsetFrontApF, offsetBotApF, offsetMagnitudeFront45App, offsetMagnitudeBot45App, offsetMagnitudeBotPi6App, offsetMagnitudeFrontPi6App, holeSpacing);
            X_place = (X_int / r_int) * (r_place + extra_offset);
            Y_place = (Y_int / r_int) * (r_place + extra_offset);
            
        [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
                X_place, Y_place, safeHeight, current_phi, current_phi, ...
                hRobot, hGripperState, d1, L1, L2, L3, dynParams);
    end

    % Now the block is held with the proper starting pose.
    % To perform a rotation, the gripper must rotate, return block to holder, release the block, change its orientation,
    % then pick up the block again.

    % Rotate Block
    if required_start == pi/2
        rotate_target = pi/6;
    elseif required_start == pi/6
        rotate_target = pi/2;
    end
         
    [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
            X_place, Y_place, safeHeight, current_phi, current_phi, ...
            hRobot, hGripperState, d1, L1, L2, L3, dynParams);
    [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
            X_place, Y_place, safeHeight, current_phi, rotate_target, ...
            hRobot, hGripperState, d1, L1, L2, L3, dynParams);
    pause(0.5);

    % Place the block on the intermediate holder (with current orientation).
    [r_place, z_place] = computeFinalOffsets(rotate_target, rotate_target, ...
            r_int, cubeElevation, offsetFrontApT, offsetBotApT, offsetFrontApF, offsetBotApF, offsetMagnitudeFront45App, offsetMagnitudeBot45App, offsetMagnitudeBotPi6App, offsetMagnitudeFrontPi6App, holeSpacing);
    X_place = (X_int / r_int) * (r_place + extra_offset);
    Y_place = (Y_int / r_int) * (r_place + extra_offset);
    [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
        X_place, Y_place, z_place, rotate_target, rotate_target, ...
        hRobot, hGripperState, d1, L1, L2, L3, dynParams);
    pause(0.5);
    waitForGripper(dynParams, gripperOpen, hGripperState, ADDR_PRO_PRESENT_POSITION);
    pause(0.5);
    Jiggle(dynParams, 3, 18);

    current_phi = rotate_target;

    
    % With the block now released, the gripper (empty) rotates.
    if current_phi == pi/2
        new_phi = pi/6;
    elseif current_phi == pi/6
        new_phi = pi/2;
    end
    
    % Move the gripper (empty) to the new orientation.
    [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
        X_place, Y_place, safeHeight, current_phi, current_phi, ...
        hRobot, hGripperState, d1, L1, L2, L3, dynParams);
    [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
        X_place, Y_place, safeHeight, current_phi, new_phi, ...
        hRobot, hGripperState, d1, L1, L2, L3, dynParams);

    current_phi = new_phi;

    %{ 
    if abs(rotation_count) == 1
        final_phi = pi/2;
        [r_pick, z_pick] = computeInitialOffsets(final_phi, r_int, cubeElevation, ...
                offsetFrontApT, offsetBotApT, offsetFrontApF, offsetBotApF, offsetMagnitudeFront45App, offsetMagnitudeBot45App, offsetMagnitudeBotPi6App, offsetMagnitudeFrontPi6App, holeSpacing);
        X_pick = (X_int / r_int) * (r_pick);
        Y_pick = (Y_int / r_int) * (r_pick);
        [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
        X_pick, Y_pick, safeHeight, current_phi, final_phi, ...
        hRobot, hGripperState, d1, L1, L2, L3, dynParams);
        pause(0.5)
        interpolateArm(th1, th2, th3, th4, ...
            X_pick, Y_pick, z_pick, final_phi, final_phi, ...
            hRobot, hGripperState, d1, L1, L2, L3, dynParams);
        pause(0.5);
        waitForGripper(dynParams, gripperClosed, hGripperState, ADDR_PRO_PRESENT_POSITION);
        pause(0.5);
        dynParams.isAppFront = 1;
        [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
            X_pick, Y_pick, safeHeight, final_phi, final_phi, ...
            hRobot, hGripperState, d1, L1, L2, L3, dynParams);
        dynParams.isAppFront = 0;

        current_phi = final_phi;
        pause(2);
    %}
    %else
 
        % Now pick up the block with the new orientation.
        [r_pick, z_pick] = computeInitialOffsets(current_phi, r_int, cubeElevation, ...
                offsetFrontApT, offsetBotApT, offsetFrontApF, offsetBotApF, offsetMagnitudeFront45App, offsetMagnitudeBot45App, offsetMagnitudeBotPi6App, offsetMagnitudeFrontPi6App, holeSpacing);
        X_pick = (X_int / r_int) * r_pick;
        Y_pick = (Y_int / r_int) * r_pick;
        [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
            X_pick, Y_pick, z_pick, current_phi, current_phi, ...
            hRobot, hGripperState, d1, L1, L2, L3, dynParams);
        pause(0.5);
        waitForGripper(dynParams, gripperClosed, hGripperState, ADDR_PRO_PRESENT_POSITION);
        pause(0.5);
        [th1, th2, th3, th4] = interpolateArm(th1, th2, th3, th4, ...
            X_pick, Y_pick, safeHeight, current_phi, current_phi, ...
            hRobot, hGripperState, d1, L1, L2, L3, dynParams);
    %end
    
    rotation_count = rotation_count - sign(rotation_count);
end

% --- Final Adjustment ---
% The stacking pose needs the block to be held with phi = π/2.
if current_phi ~= pi/2
    [th1, th2, th3, th4, current_phi] = reorientViaHolder(th1, th2, th3, th4, current_phi, pi/2);
end



end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4) placeCube
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [th1, th2, th3, th4, cubeAttached, currX, currY, currZ] = placeCube( ...
    th1_init, th2_init, th3_init, th4_init, ...
    cubeIndex, finalX, finalY, graspZ_final, preGraspZ, ...
    stackOrientation, current_phi, ...
    hCubeMarkers, cubeAttached, gripperOpen, stackFlag, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams)


th1 = th1_init;
th2 = th2_init;
th3 = th3_init;
th4 = th4_init;

% --- Helper inline function for jiggle ---
    function [] = Jiggle(dynParams, iterations, offset)
        current_ticks1 = read4ByteTxRx(dynParams.port_num, dynParams.PROTOCOL_VERSION, dynParams.DXL_ID1, dynParams.ADDR_PRO_PRESENT_POSITION);

        for i=1:iterations
            write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), double(dynParams.DXL_ID1), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(current_ticks1 + offset));
            pause(0.5)
            write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), double(dynParams.DXL_ID1), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(current_ticks1 - offset));
            pause(0.5)
        end
        write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), double(dynParams.DXL_ID1), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(current_ticks1));
        pause(0.5)
    end

% Move to transport pose above final
[th1, th2, th3, th4] = interpolateArm( ...
    th1, th2, th3, th4, ...
    finalX, finalY, preGraspZ, ...
    current_phi, stackOrientation, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams);

% If stacking, add offset for each subsequent cube
if stackFlag == 1
    finalZ_with_stack = graspZ_final + (cubeIndex-1)*0.023;
else
    finalZ_with_stack = graspZ_final;
end

% Lower to final placement
[th1, th2, th3, th4] = interpolateArm( ...
    th1, th2, th3, th4, ...
    finalX, finalY, finalZ_with_stack, ...
    stackOrientation, stackOrientation, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams);
pause(0.5);
% Open gripper
waitForGripper(dynParams, gripperOpen, hGripperState, dynParams.ADDR_PRO_GOAL_POSITION);
pause(0.5);
if cubeIndex == 1
    Jiggle(dynParams, 3, 18);
end

cubeAttached(cubeIndex) = false;
delete(hCubeMarkers(cubeIndex));

% Lift arm back to transport pose
[th1, th2, th3, th4, currX, currY, currZ] = interpolateArm( ...
    th1, th2, th3, th4, ...
    finalX, finalY, preGraspZ, ...
    stackOrientation, stackOrientation, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 5) approachFront 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [theta1_final, theta2_final, theta3_final, theta4_final, currX, currY, currZ] = approachFront( ...
    theta1_current, theta2_current, theta3_current, theta4_current, ...
    targetX, targetY, targetZ, safeHeight, phi_current, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams, gripperClosed, ADDR_PRO_PRESENT_POSITION)

gripperOpen = 2100;
pause(0.5);
    waitForGripper(dynParams, gripperOpen, hGripperState, ADDR_PRO_PRESENT_POSITION);
pause(0.5);


% 1. Compute the Approach Point
r_target = sqrt(targetX^2 + targetY^2);
if r_target < 0.07
    error('Target is too close to the origin to subtract 0.07 m radially.');
end
r_approach = r_target - 0.07;
disp(r_target)
disp(r_approach)
approachX = (targetX / r_target) * r_approach;
approachY = (targetY / r_target) * r_approach;

targetZ = targetZ + 0.005;

desired_phi = pi/6;

% 2. Move to Safe Height Above the Approach Point
[theta1_seg, theta2_seg, theta3_seg, theta4_seg] = interpolateArm( ...
    theta1_current, theta2_current, theta3_current, theta4_current, ...
    approachX, approachY, safeHeight, ...
    phi_current, desired_phi, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams);


% 3. Lower the Arm to the Approach Point
[theta1_seg, theta2_seg, theta3_seg, theta4_seg] = interpolateArm( ...
    theta1_seg, theta2_seg, theta3_seg, theta4_seg, ...
    approachX, approachY, targetZ, ...
    desired_phi, desired_phi, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams);
pause(0.5);

dynParams.isAppFront = 1;
% 4. Extend Laterally from the Approach Point to the Target 
[theta1_seg, theta2_seg, theta3_seg, theta4_seg] = interpolateArm( ...
    theta1_seg, theta2_seg, theta3_seg, theta4_seg, ...
    targetX, targetY, targetZ, ...
    desired_phi, desired_phi, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams);
dynParams.isAppFront = 0;

% 4.5 Close Gripper
pause(0.5);
    waitForGripper(dynParams, gripperClosed, hGripperState, ADDR_PRO_PRESENT_POSITION);
pause(0.5);

% 4.6 Lift a little
[theta1_seg, theta2_seg, theta3_seg, theta4_seg] = interpolateArm( ...
    theta1_seg, theta2_seg, theta3_seg, theta4_seg, ...
    targetX, targetY, targetZ + 0.005, ...
    desired_phi, desired_phi, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams);
pause(0.5);

% 5. Retract Laterally Back to the Approach Point 
[theta1_seg, theta2_seg, theta3_seg, theta4_seg] = interpolateArm( ...
    theta1_seg, theta2_seg, theta3_seg, theta4_seg, ...
    approachX, approachY, targetZ, ...
    desired_phi, desired_phi, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams);

% 6. Raise the Arm Back to Safe Height Above the Approach Point
[theta1_seg, theta2_seg, theta3_seg, theta4_seg, currX, currY, currZ] = interpolateArm( ...
    theta1_seg, theta2_seg, theta3_seg, theta4_seg, ...
    approachX, approachY, safeHeight, ...
    desired_phi, desired_phi, ...
    hRobot, hGripperState, d1, L1, L2, L3, dynParams);

theta1_final = theta1_seg;
theta2_final = theta2_seg;
theta3_final = theta3_seg;
theta4_final = theta4_seg;


end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% interpolateArm 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [theta1, theta2, theta3, theta4, currX, currY, currZ] = interpolateArm( ...
    theta1_init, theta2_init, theta3_init, theta4_init, ...
    targetX, targetY, targetZ, ...
    phi_init, phi_final, ... 
    hRobot, hGripperState, d1, L1, L2, L3, dynParams)

desiredStepSize = 0.01; %0.015, 0.018 can also be good
desiredPhiStep    = 0.1; 

if dynParams.isAppFront == 1
    desiredStepSize = 0.001;
end

[robotX0, robotY0, robotZ0] = computeFK(theta1_init, theta2_init, theta3_init, theta4_init, d1, L1, L2, L3);
x0 = robotX0(end);
y0 = robotY0(end);
z0 = robotZ0(end);

% 1. Determine number of steps based on Cartesian displacement.
deltaX = targetX - x0;
deltaY = targetY - y0;
deltaZ = targetZ - z0;

cartesianDist = sqrt(deltaX^2 + deltaY^2 + deltaZ^2);
numSteps_cart = ceil(cartesianDist / desiredStepSize);

deltaPhi = abs(phi_final - phi_init);
numSteps_phi = ceil(deltaPhi / desiredPhiStep);

numSteps = max(numSteps_cart, numSteps_phi);
numSteps = max(numSteps, 1);

maxSteps = 1e5;  
numSteps = min(numSteps, maxSteps);

% 2. Compute key Cartesian waypoints.
midX = (x0 + targetX) / 2;
midY = (y0 + targetY) / 2;
midZ = (z0 + targetZ) / 2;
mid_phi = (phi_init + phi_final) / 2;

% 3. Compute key joint configurations via IK.

q0 = [theta1_init, theta2_init, theta3_init, theta4_init];
[q_mid1, q_mid2, q_mid3, q_mid4] = computeIK(midX, midY, midZ, d1, L1, L2, L3, mid_phi);
q_mid = [q_mid1, q_mid2, q_mid3, q_mid4];
[q_target1, q_target2, q_target3, q_target4] = computeIK(targetX, targetY, targetZ, d1, L1, L2, L3, phi_final);
q_target = [q_target1, q_target2, q_target3, q_target4];

% 4. Generate joint-space splines for each joint.
t_key = [0, 0.5, 1];

q1_key = [q0(1), q_mid(1), q_target(1)];
q2_key = [q0(2), q_mid(2), q_target(2)];
q3_key = [q0(3), q_mid(3), q_target(3)];
q4_key = [q0(4), q_mid(4), q_target(4)];

spline1 = spline(t_key, q1_key);
spline2 = spline(t_key, q2_key);
spline3 = spline(t_key, q3_key);
spline4 = spline(t_key, q4_key);

t_interp = linspace(0, 1, numSteps);

q1_interp = ppval(spline1, t_interp);
q2_interp = ppval(spline2, t_interp);
q3_interp = ppval(spline3, t_interp);
q4_interp = ppval(spline4, t_interp);

[~, idx0] = min(abs(t_interp - 0));
[~, idxMid] = min(abs(t_interp - 0.5));
[~, idx1] = min(abs(t_interp - 1));

q1_interp(idx0) = q0(1); q1_interp(idxMid) = q_mid(1); q1_interp(idx1) = q_target(1);
q2_interp(idx0) = q0(2); q2_interp(idxMid) = q_mid(2); q2_interp(idx1) = q_target(2);
q3_interp(idx0) = q0(3); q3_interp(idxMid) = q_mid(3); q3_interp(idx1) = q_target(3);
q4_interp(idx0) = q0(4); q4_interp(idxMid) = q_mid(4); q4_interp(idx1) = q_target(4);

% 5. Execute the trajectory.
for i = 1:numSteps
    theta1_val = q1_interp(i);
    theta2_val = q2_interp(i);
    theta3_val = q3_interp(i);
    theta4_val = q4_interp(i);

    %{
    % Update visualization.
    [robotX, robotY, robotZ] = computeFK(theta1_val, theta2_val, theta3_val, theta4_val, d1, L1, L2, L3);
    set(hRobot, 'XData', robotX, 'YData', robotY, 'ZData', robotZ);
    set(hGripperState, 'Position', [robotX(end)+0.01, robotY(end), robotZ(end)]);
    drawnow limitrate;
    pause(0.005);
    %}
    
    ticks1 = round(dynParams.center1 + theta1_val * dynParams.ticks_per_rad) + 1;
    ticks1 = max(dynParams.min1, min(dynParams.max1, ticks1));
    ticks2 = round(dynParams.center2 + theta2_val * dynParams.ticks_per_rad);
    ticks2 = max(dynParams.min2, min(dynParams.max2, ticks2));
    ticks3 = round(dynParams.center3 + theta3_val * dynParams.ticks_per_rad);
    ticks3 = max(dynParams.min3, min(dynParams.max3, ticks3));
    ticks4 = round(dynParams.center4 + theta4_val * dynParams.ticks_per_rad);
    ticks4 = max(dynParams.min4, min(dynParams.max4, ticks4));
    
    write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), ...
        double(dynParams.DXL_ID1), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(ticks1));
    write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), ...
        double(dynParams.DXL_ID2), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(ticks2));
    write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), ...
        double(dynParams.DXL_ID3), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(ticks3));
    write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), ...
        double(dynParams.DXL_ID4), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(ticks4));
end
pause(0.5)

current_ticks1 = read4ByteTxRx(dynParams.port_num, dynParams.PROTOCOL_VERSION, dynParams.DXL_ID1, dynParams.ADDR_PRO_PRESENT_POSITION);
current_ticks2 = read4ByteTxRx(dynParams.port_num, dynParams.PROTOCOL_VERSION, dynParams.DXL_ID2, dynParams.ADDR_PRO_PRESENT_POSITION);
current_ticks3 = read4ByteTxRx(dynParams.port_num, dynParams.PROTOCOL_VERSION, dynParams.DXL_ID3, dynParams.ADDR_PRO_PRESENT_POSITION);
current_ticks4 = read4ByteTxRx(dynParams.port_num, dynParams.PROTOCOL_VERSION, dynParams.DXL_ID4, dynParams.ADDR_PRO_PRESENT_POSITION);

theta1_current = (current_ticks1 - dynParams.center1) / dynParams.ticks_per_rad;
theta2_current = (current_ticks2 - dynParams.center2) / dynParams.ticks_per_rad;
theta3_current = (current_ticks3 - dynParams.center3) / dynParams.ticks_per_rad;
theta4_current = (current_ticks4 - dynParams.center4) / dynParams.ticks_per_rad;

[robotX, robotY, robotZ] = computeFK(theta1_current, theta2_current, theta3_current, theta4_current, d1, L1, L2, L3);
targetPos = [targetX, targetY, targetZ];
robotPos  = [robotX(end), robotY(end), robotZ(end)];
distance = norm(targetPos - robotPos);

if distance > 0.15
    disp('Re-Attempting Interpolation')
    interpolateArm( ...
        theta1_current, theta2_current, theta3_current, theta4_current, ...
        targetX, targetY, targetZ, ...
        phi_init, phi_final, ... 
        hRobot, hGripperState, d1, L1, L2, L3, dynParams)
end


% 6. Set final joint angles and pos
[theta1, theta2, theta3, theta4] = computeIK(targetX, targetY, targetZ, d1, L1, L2, L3, phi_final);
currX = targetX;
currY = targetY;
currZ = targetZ;

end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% getEndEffectorPos
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function pos = getEndEffectorPos(theta1, theta2, theta3, theta4, d1, L1, L2, L3)
T01 = dh_transform_updated(0, 0, theta1, d1);
T12 = dh_transform_updated(-pi/2, 0, theta2 - 1.38217994, 0);
T23 = dh_transform_updated(0, L1, theta3 + 1.38217994, 0);
T34 = dh_transform_updated(0, L2, theta4, 0);
T45 = dh_transform_updated(0, L3, 0, 0);

T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;

pos = T05(1:3,4);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% computeInitialOffsets
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [r_new, z_pos] = computeInitialOffsets(pickupPhi, r, cubeElevation, ...
    offsetFrontApT, offsetBotApT, offsetFrontApF, offsetBotApF, offsetMagnitudeFront45App, offsetMagnitudeBot45App, offsetMagnitudeBotPi6App, offsetMagnitudeFrontPi6App, holeSpacing)

    if abs(pickupPhi - pi/2) < 1e-3
        r_new = r + offsetFrontApT;
        z_pos = cubeElevation + offsetBotApT;
    elseif abs(pickupPhi - 0) < 1e-3
        r_new = r + offsetFrontApF;
        z_pos = cubeElevation + offsetBotApF;
    elseif abs(pickupPhi - pi/4) < 1e-3
        r_new = r + offsetMagnitudeFront45App;
        z_pos = cubeElevation + offsetMagnitudeBot45App;
    elseif abs(pickupPhi - pi/6) < 1e-3
        r_new = r + offsetMagnitudeFrontPi6App;
        z_pos = cubeElevation + offsetMagnitudeBotPi6App;
    else
        error('Unexpected pickup orientation.');
    end

    initR = r_new;
    heightOffset = 0.01 / ( sqrt((9 * holeSpacing)^2 + (8 * holeSpacing)^2 ) ...
                                  - 4 * holeSpacing );
    if initR > 6 * holeSpacing
        initOffset = 2 * (initR - 6 * holeSpacing)^2;
    else
        initOffset = 0;
    end

    z_pos = z_pos + initOffset * heightOffset * 2;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% computeFinalOffsets
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [r_new_final, z_pos_final] = computeFinalOffsets(pickupPhi, placementPhi, ...
    r, cubeElevation, offsetFrontApT, offsetBotApT, offsetFrontApF, offsetBotApF, offsetMagnitudeFront45App, offsetMagnitudeBot45App, offsetMagnitudeBotPi6App, offsetMagnitudeFrontPi6App,...
    holeSpacing)

    offsetPi6R = 0;
    offsetPi6Z = 0;
    offsetPi2R = 0;
    offsetPi2Z = 0;

    if abs(placementPhi - pi/2) < 1e-3
        if pickupPhi == pi/6
            r_new_final = r + offsetFrontApT + offsetPi6R;
            z_pos_final = cubeElevation + offsetBotApT + offsetPi6Z;
        elseif pickupPhi == pi/2
             r_new_final = r + offsetFrontApT;
             z_pos_final = cubeElevation + offsetBotApT;
        end

    elseif abs(placementPhi - 0) < 1e-3
        z_pos_final = cubeElevation + offsetBotApF;
        r_new_final = r + offsetFrontApF;

    elseif abs(placementPhi - pi/4) < 1e-3
        r_new_final = r + offsetMagnitudeFront45App;
        z_pos_final = cubeElevation + offsetMagnitudeBot45App;

    elseif abs(placementPhi - pi/6) < 1e-3
        if pickupPhi == pi/2
            r_new_final = r + offsetMagnitudeFrontPi6App + offsetPi2R;
            z_pos_final = cubeElevation + offsetMagnitudeBotPi6App + offsetPi2Z;
        elseif pickupPhi == pi/6
            r_new_final = r + offsetMagnitudeFrontPi6App;
            z_pos_final = cubeElevation + offsetMagnitudeBotPi6App;
        end
    else
        error('Unexpected placement orientation.');
    end

    finalR = r_new_final; 
    heightOffset = 0.01 / ( sqrt((9 * holeSpacing)^2 + (8 * holeSpacing)^2 ) ...
                                  - 4 * holeSpacing);

    if finalR > 6 * holeSpacing
        finalOffset = 2 * ( finalR - 6*holeSpacing )^2;
    else
        finalOffset = 0;
    end

    z_pos_final = z_pos_final + finalOffset * heightOffset * 2;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% computeFK
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% computeIK
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [theta1, theta2, theta3, theta4] = computeIK(xd, yd, zd, d1, L1, L2, L3, phi)
theta1 = atan2(yd, xd);

x_c = xd - L3*cos(phi)*cos(theta1);
y_c = yd - L3*cos(phi)*sin(theta1);
z_c = d1 - (zd + L3*sin(phi)); 

r_wc = sqrt(x_c^2 + y_c^2);

cos_theta3 = (r_wc^2 + z_c^2 - L1^2 - L2^2) / (2*L1*L2);
cos_theta3 = max(min(cos_theta3,1),-1);
theta3_prime = acos(cos_theta3);

theta2_prime = atan2(z_c, r_wc) - atan2(L2*sin(theta3_prime), L1 + L2*cos(theta3_prime));

theta2 = theta2_prime + 1.385448377;
theta3 = theta3_prime - 1.385448377;
theta4 = phi - (theta2_prime + theta3_prime);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% dh_transform_updated
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function T = dh_transform_updated(alpha, a, theta, d)
T = [ cos(theta),            -sin(theta),            0,           a;
      sin(theta)*cos(alpha), cos(theta)*cos(alpha),  -sin(alpha), -d*sin(alpha);
      sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  d*cos(alpha);
      0,                     0,                      0,           1];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% waitForGripper
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function waitForGripper(dynParams, desiredState, hGripperState, ADDR_PRO_PRESENT_POSITION)
write4ByteTxRx(double(dynParams.port_num), double(dynParams.PROTOCOL_VERSION), ...
    double(dynParams.DXL_ID5), double(dynParams.ADDR_PRO_GOAL_POSITION), uint32(desiredState));

if desiredState == 2300
    set(hGripperState, 'String', 'Gripper: Closed');
elseif desiredState == 1900
    set(hGripperState, 'String', 'Gripper: Open');
else
    set(hGripperState, 'String', 'Gripper: Unknown');
end

threshold = 10; 
maxTime   = 5;  
tic;
while toc < maxTime
    currentPos = read4ByteTxRx(double(dynParams.port_num), ...
        double(dynParams.PROTOCOL_VERSION), double(dynParams.DXL_ID5), ...
        double(ADDR_PRO_PRESENT_POSITION));
    if abs(double(currentPos) - double(desiredState)) < threshold
        break;
    end
    pause(0.05);
end
end
