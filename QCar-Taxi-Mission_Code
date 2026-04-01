function [targetSpeed, stage, driveEnable, pathCmd, ledStripCmd, ledsCmd] = ...
    TaxiMissionManager(currentPose, measuredSpeed, lidarOK, stopSignDetected)
%#codegen
%
% TaxiMissionManager - MULTI-RIDE QUEUE + HARDCODED PATH SELECTION
% Paths 1..9 exist, Path 8 is DUMMY (never command 8).
%
% Stages:
%  1 Drive to PICKUP
%  2 Stop at PICKUP
%  3 Drive to DROPOFF (or STOP then DROPOFF for Ride E)
%  4 Stop at DROPOFF (or STOP node)
%  5 Drive to HUB
%  6 HUB stop confirm
%  7 HUB idle (then next ride or disarm)

%% ===================== Point Table (0..25) =====================
xy = [ ...
    0.0, 0.269, 1.127, 1.127, 2.255, 1.984, 1.013, 1.235, -0.749, -0.749, -1.282, 0.0, 0.0, 0.269, 2.255, 1.984, 0.908, 1.466, 0.623, 0.792, 0.0, 0.0, -1.984, -1.716, -0.826, -0.857; ...
    0.0, -0.049, -1.075, -0.820, -0.049, -0.049, 1.081, 0.791, 1.077, 0.790, -0.590, 2.033, 1.720, 1.720, 2.837, 1.720, 3.580, 3.020, 2.937, 2.729, 4.421, 4.146, 2.837, 2.837, 3.645, 1.716];

%% ===================== User Settings =====================
rideQueue       = int32([2 1 1 4 5]);   % Ride order
hubID           = uint8(10);            % Hub node
hubIdleTime     = 5.0;                  % Hub idle between rides (s)

% Vision Stop Sign Settings
stopHoldTime        = 2.0;  % Hold stop duration (s)
detectConfirmTime   = 0.06; % Detector high duration to confirm (s)
detectReleaseTime   = 0.20; % Detector low duration to re-arm (s)
stopCooldownTime    = 1.50; % Ignore detector after completed stop (s)

% Driving / Path Tuning
arrivalRadius       = 0.30; % Final waypoint radius
switchRadius        = 0.35; % Intermediate waypoint radius
hubArrivalRadius    = 0.70; % Hub detection radius
vCruise             = 0.4;
aBrake              = 0.50;
dBuffer             = 0.05;
vStopThresh         = 0.20;
stopTimePickup      = 3.0;
stopTimeStop        = 3.0;  % RED stop at point 23 (Ride E)
stopTimeDropoff     = 3.0;
hubConfirmTime      = 0.5;
Ts                  = 0.002; % Loop timestep

% Speed limit helper
speedLimitFromDistance = @(d) sqrt(max(2*aBrake*max(d - dBuffer, 0.0), 0.0));

%% ===================== LED Definitions =====================
LED_MAX = 100;
MAGENTA = [LED_MAX; 0; LED_MAX];
GREEN   = [0; LED_MAX; 0];
BLUE    = [0; 0; LED_MAX];
ORANGE  = [LED_MAX; 0.5*LED_MAX; 0];
RED     = [LED_MAX; 0; 0];
ALL_ON  = true(16,1);
ALL_OFF = false(16,1);

%% ===================== Persistent State =====================
persistent s tStop hubStopped hasStarted wpIdx lastPathCmd rideIdx stopServed
persistent stopVisionActive stopVisionT stopVisionLatched
persistent stopDetHighT stopDetLowT stopCooldownT

if isempty(s)
    s = uint8(1); tStop = 0.0; hubStopped = false; hasStarted = false;
    wpIdx = uint8(1); lastPathCmd = int32(7); rideIdx = uint8(1);
    stopServed = false; stopVisionActive = false; stopVisionT = 0.0;
    stopVisionLatched = false; stopDetHighT = 0.0; stopDetLowT = 0.0;
    stopCooldownT = 0.0;
end

%% ===================== Defaults =====================
targetSpeed = 0.0;
driveEnable = true;
pathCmd     = int32(7);
ledStripCmd = MAGENTA;
ledsCmd     = ALL_OFF;

%% ===================== Safety: LiDAR Check =====================
if ~lidarOK
    targetSpeed = 0.0;
    driveEnable = false;
    stage = s;
    
    % Reset vision stop
    stopVisionActive = false; stopVisionT = 0.0; stopVisionLatched = false;
    stopDetHighT = 0.0; stopDetLowT = 0.0; stopCooldownT = 0.0;
    
    % Reset mission
    s = uint8(1); tStop = 0.0; hubStopped = false; hasStarted = false;
    wpIdx = uint8(1); lastPathCmd = int32(7); rideIdx = uint8(1);
    stopServed = false;
    return;
end

%% ===================== Prepare Variables =====================
pos = currentPose(1:2);
v   = abs(measuredSpeed);

% Clamp ride index
rideIdx = max(uint8(1), min(rideIdx, uint8(numel(rideQueue))));
ACTIVE_RIDE = rideQueue(double(rideIdx));
hubXY = xy(:, double(hubID)+1);

%% ===================== Build Hardcoded Routes =====================
[pickupID, dropoffID, stopID, ...
 ROUTE1_NODES, ROUTE1_PATHS, ...
 ROUTE3_NODES, ROUTE3_PATHS, ...
 ROUTE5_NODES, ROUTE5_PATHS] = buildRideHardcoded(ACTIVE_RIDE, hubID);

stopIdx = uint8(0);
if (ACTIVE_RIDE == 5) && (stopID ~= uint8(255))
    stopIdx = uint8(4); % Stop node index for Ride E
end

pickupXY  = xy(:, double(pickupID)+1); %#ok<NASGU>
dropoffXY = xy(:, double(dropoffID)+1); %#ok<NASGU>

%% ===================== Helper Function: driveRoute =====================
function [pcmd, done] = driveRoute(routeNodes, routePaths)
    done = false;
    nNodes = numel(routeNodes);

    % Guard: paths length mismatch
    if numel(routePaths) ~= (nNodes-1)
        pcmd = lastPathCmd; targetSpeed = 0.0; return;
    end
    if wpIdx >= nNodes
        done = true; pcmd = lastPathCmd; targetSpeed = 0.0; return;
    end

    % Current path
    pcmd = routePaths(double(wpIdx));
    lastPathCmd = pcmd;

    % Next waypoint distance
    nextID = routeNodes(double(wpIdx)+1);
    nextXY = xy(:, double(nextID)+1);
    dNext = norm(pos - nextXY);

    %% Vision Stop Logic (debounce + cooldown)
    if stopSignDetected ~= 0
        stopDetHighT = stopDetHighT + Ts; stopDetLowT = 0.0;
    else
        stopDetLowT = stopDetLowT + Ts; stopDetHighT = 0.0;
    end

    if stopCooldownT > 0.0
        stopCooldownT = max(stopCooldownT - Ts, 0.0);
    end
    if (~stopVisionActive) && (stopCooldownT <= 0.0) && (stopDetLowT >= detectReleaseTime)
        stopVisionLatched = false;
    end
    if (~stopVisionActive) && (~stopVisionLatched) && (stopCooldownT <= 0.0) && (stopDetHighT >= detectConfirmTime)
        stopVisionActive = true; stopVisionT = 0.0; stopVisionLatched = true;
    end

    if stopVisionActive
        targetSpeed = 0.0; pcmd = lastPathCmd; done = false;
        if v < vStopThresh
            stopVisionT = stopVisionT + Ts;
        end
        if (stopVisionT >= stopHoldTime) && (v < vStopThresh)
            stopVisionActive = false; stopVisionT = 0.0; stopCooldownT = stopCooldownTime;
        end
        return; % pause route progression
    end

    % Speed command
    targetSpeed = min(vCruise, speedLimitFromDistance(dNext));

    % Waypoint progression
    isFinalNext = (double(wpIdx) + 1) == nNodes;
    if isFinalNext
        if dNext < arrivalRadius
            wpIdx = uint8(nNodes); done = true;
        end
    else
        if dNext < switchRadius
            wpIdx = wpIdx + 1;
        end
    end
end

%% ===================== Main FSM =====================
switch s
    case 1  % Drive to PICKUP
        ledStripCmd = GREEN; ledsCmd = ALL_ON;
        if ~hasStarted && norm(pos - hubXY) < arrivalRadius && v < vStopThresh
            ledStripCmd = MAGENTA;
        end
        if ~hasStarted && (norm(pos - hubXY) >= arrivalRadius || v >= vStopThresh)
            hasStarted = true;
        end
        [pathCmd, arrived] = driveRoute(ROUTE1_NODES, ROUTE1_PATHS);
        if arrived, s = uint8(2); tStop = 0.0; wpIdx = uint8(1); end

    case 2  % STOP at PICKUP
        pathCmd = lastPathCmd; ledStripCmd = BLUE; ledsCmd = ALL_OFF; targetSpeed = 0.0;
        tStop = tStop + Ts;
        if (tStop >= stopTimePickup) && (v < vStopThresh)
            s = uint8(3); tStop = 0.0; wpIdx = uint8(1);
        end

    case 3  % Drive to DROPOFF (Ride E STOP logic included)
        ledStripCmd = BLUE; ledsCmd = ALL_OFF;
        [pathCmd, arrived] = driveRoute(ROUTE3_NODES, ROUTE3_PATHS);
        if (ACTIVE_RIDE == 5) && (stopIdx ~= 0) && ~stopServed
            dStop = norm(pos - xy(:, double(stopID)+1));
            if dStop < arrivalRadius
                wpIdx = stopIdx; s = uint8(4); tStop = 0.0;
            end
        end
        if arrived, s = uint8(4); tStop = 0.0; end

    case 4  % STOP at DROPOFF or STOP node (Ride E)
        pathCmd = lastPathCmd; targetSpeed = 0.0; ledsCmd = ALL_OFF;
        atStopNode = (ACTIVE_RIDE == 5) && (stopIdx ~= 0) && (wpIdx == stopIdx) && ~stopServed;
        dwellT = atStopNode * stopTimeStop + ~atStopNode * stopTimeDropoff;
        ledStripCmd = atStopNode * RED + ~atStopNode * ORANGE;
        tStop = tStop + Ts;
        if (tStop >= dwellT) && (v < vStopThresh)
            if atStopNode
                stopServed = true; s = uint8(3); tStop = 0.0;
            else
                s = uint8(5); tStop = 0.0; wpIdx = uint8(1);
            end
        end

    case 5  % Drive back to HUB
        ledStripCmd = ORANGE; ledsCmd = ALL_OFF;
        [pathCmd, arrived] = driveRoute(ROUTE5_NODES, ROUTE5_PATHS);
        dHub = norm(pos - hubXY);
        if arrived || (dHub < hubArrivalRadius)
            wpIdx = uint8(numel(ROUTE5_NODES)); s = uint8(6); tStop = 0.0; hubStopped = false;
        end

    case 6  % HUB stop confirm
        pathCmd = lastPathCmd; ledStripCmd = MAGENTA; ledsCmd = ALL_OFF; targetSpeed = 0.0;
        if ~hubStopped
            if v < vStopThresh, tStop = tStop + Ts; else, tStop = 0.0; end
            if tStop >= hubConfirmTime
                hubStopped = true; tStop = 0.0; s = uint8(7);
            end
        end

    otherwise  % HUB IDLE then next ride
        pathCmd = lastPathCmd; ledStripCmd = MAGENTA; ledsCmd = ALL_OFF; targetSpeed = 0.0;
        if v < vStopThresh, tStop = tStop + Ts; else, tStop = 0.0; end
        if tStop >= hubIdleTime
            if rideIdx < uint8(numel(rideQueue))
                % Next ride
                rideIdx = rideIdx + 1;
                s = uint8(1); tStop = 0.0; hubStopped = false; hasStarted = false;
                wpIdx = uint8(1); stopServed = false;
                stopVisionActive = false; stopVisionT = 0.0; stopVisionLatched = false;
                stopDetHighT = 0.0; stopDetLowT = 0.0; stopCooldownT = 0.0;
            else
                driveEnable = false; % Finished all rides
            end
        end
end

stage = s;
end

%% ===================== Local Function: buildRideHardcoded =====================
function [pickupID, dropoffID, stopID, R1N, R1P, R3N, R3P, R5N, R5P] = buildRideHardcoded(rideID, hubID)
stopID = uint8(255); % default

switch int32(rideID)
    case 1  % Ride A: 1 -> 8
        pickupID  = uint8(21); dropoffID = uint8(8); viaID = uint8(8);
        R1N = uint8([hubID, viaID, pickupID]); R1P = int32([6, 9]);
        R3N = uint8([pickupID, dropoffID]);   R3P = int32([9]);
        R5N = uint8([dropoffID, hubID]);      R5P = int32([6]);
    case 2  % Ride B: 2 -> 6
        pickupID  = uint8(2); dropoffID = uint8(6);
        R1N = uint8([hubID, pickupID]); R1P = int32([2]);
        R3N = uint8([pickupID, dropoffID]); R3P = int32([2]);
        R5N = uint8([dropoffID, hubID]); R5P = int32([2]);
    case 3  % Ride C: 4 -> 0
        pickupID  = uint8(4); dropoffID = uint8(0);
        R1N = uint8([hubID, pickupID]); R1P = int32([2]);
        R3N = uint8([pickupID, dropoffID]); R3P = int32([1]);
        R5N = uint8([dropoffID, hubID]); R5P = int32([2]);
    case 4  % Ride D: 8 -> 18
        pickupID  = uint8(8); dropoffID = uint8(18);
        R1N = uint8([hubID, pickupID]); R1P = int32([6]);
        R3N = uint8([pickupID, dropoffID]); R3P = int32([9]);
        R5N = uint8([dropoffID, uint8(8), hubID]); R5P = int32([9, 2]);
    otherwise % Ride E: 7 -> 23 -> 11
        pickupID  = uint8(7); stopID = uint8(23); dropoffID = uint8(11);
        R1N = uint8([hubID, pickupID]); R1P = int32([8]);
        R3N = uint8([pickupID, uint8(1), uint8(8), stopID, dropoffID]); R3P = int32([8, 6, 9, 9]);
        R5N = uint8([dropoffID, uint8(8), hubID]); R5P = int32([9, 2]);
end
end
