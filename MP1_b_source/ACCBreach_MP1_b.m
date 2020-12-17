clc;
clear S;
close all;

%%
% Specify Video File
videoFileName = '05_highway_lanechange_25s.mp4';

x0_host = 0;
v0_host = 0;
vref_host = 0;
Ts = 0.01;
endTime = 50;
dSafe = 5;

%%                              STEP 1
%% 
% Setup Video Reader and Player
videoReader = VideoReader(videoFileName);
videoPlayer = vision.DeployableVideoPlayer();

% Load the monoCamera object that contains the camera information.
d = load('FCWDemoMonoCameraSensor.mat', 'sensor');

% Load Pre-Trained RCNN model
fasterRCNN = vehicleDetectorFasterRCNN('full-view');

% Common Vehicle Width
vehicleWidth = [1.5, 2.5];

% Configure the detector using the monoCamera sensor and desired width.
detector = configureDetectorMonoCamera(fasterRCNN, d.sensor, vehicleWidth);


%% Run While Loop (PART 1)
currentStep = 0;
snapshot = [];
snapTimeStamp = 120;
cont = hasFrame(videoReader);
LeadPosition = zeros(500);
while cont
    cont = hasdata(imdsTrain);
    if(cont == 0)
        break
    end
    currentStep = currentStep + 1;

    % Read the next frame.
      frame = readFrame(videoReader);
    % Run the detector and package the returned results into an object
        % required by multiObjectTracker.  You can find the |detectObjects|
        % function at the end of this example.
    % detections = detectObjects(detector, frame, currentStep);

    bboxes = detect(detector, frame);

    % locations = computeVehicleLocations(bboxes, sensor);
    locations = zeros(size(bboxes,1),2);
    flag = 0;
    for i = 1:size(bboxes, 1)
        bbox  = bboxes(i, :);

        % Get [x,y] location of the center of the lower portion of the
        % detection bounding box in meters. bbox is [x, y, width, height] in
        % image coordinates, where [x,y] represents upper-left corner.
        yBottom = bbox(2) + bbox(4) - 1;
        xCenter = bbox(1) + (bbox(3)-1)/2; % approximate center

        locations(i,:) = imageToVehicle(d.sensor, [xCenter, yBottom]);
        if(~(locations(i,2)>0.8 || locations(i,2)<-0.8))
            LeadPosition(currentStep) = locations(i,1);
            flag = 1;
        end

    end
    if(flag == 0 && currentStep>1)
        LeadPosition(currentStep) = LeadPosition(currentStep-1);
    end
    if(flag == 0 && currentStep==1)
        LeadPosition(currentStep) = 100;
    end
end



%%                              STEP 2
%% Using a Pre-Trained Detector with KITTI Dataset (STEP 2)
% Import Data
% parse data for given pattern
data = parseKITTIBoxes('KITTI_MOD_fixed/training/boxes', "2011_09_26_drive_0057_sync*");
% Use imageDatastore and boxLabelDatastore to create datastores for loading the image and label data during training and evaluation.

imdsTrain = imageDatastore(data{:,'imageFilename'});
bldsTrain = boxLabelDatastore(data(:,'vehicle'));

% Combine image and box label datastores.
trainingData = combine(imdsTrain,bldsTrain);
 


%% Configure a monoCamera for KITTI Dataset (STEP 2)
focalLength = [7.215377000000000e+02 7.215377000000000e+02];
principalPoint = [610 173];
imageSize = [375 1242]; 
height = 1.65;
pitch = 0;

intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
monCam = monoCamera(intrinsics,height,'Pitch',pitch);

% Load Pre-Trained RCNN model
fasterRCNN = vehicleDetectorFasterRCNN('full-view');

vehicleWidth = [1.5 2.5];

kitti_detector = configureDetectorMonoCamera(fasterRCNN, monCam, vehicleWidth);


%% Run While Loop (PART 2)

currentStep = 0;
snapshot = [];
snapTimeStamp = 120;
cont = hasdata(imdsTrain);
LeadPosition = zeros(500);
while cont
    cont = hasdata(imdsTrain);
    if(cont == 0)
        break
    end
    currentStep = currentStep + 1;

    % Read the next frame.
%     frame = readFrame(videoReader);
      frame = read(imdsTrain);
    % Run the detector and package the returned results into an object
        % required by multiObjectTracker.  You can find the |detectObjects|
        % function at the end of this example.
    % detections = detectObjects(detector, frame, currentStep);

    bboxes = detect(kitti_detector, frame);

    % locations = computeVehicleLocations(bboxes, sensor);
    locations = zeros(size(bboxes,1),2);
    flag = 0;
    for i = 1:size(bboxes, 1)
        bbox  = bboxes(i, :);

        % Get [x,y] location of the center of the lower portion of the
        % detection bounding box in meters. bbox is [x, y, width, height] in
        % image coordinates, where [x,y] represents upper-left corner.
        yBottom = bbox(2) + bbox(4) - 1;
        xCenter = bbox(1) + (bbox(3)-1)/2; % approximate center

        locations(i,:) = imageToVehicle(monCam, [xCenter, yBottom]);
        if(~(locations(i,2)>0.8 || locations(i,2)<-0.8))
            LeadPosition(currentStep) = locations(i,1);
            flag = 1;
        end

    end
    if(flag == 0 && currentStep>1)
        LeadPosition(currentStep) = LeadPosition(currentStep-1);
    end
    if(flag == 0 && currentStep==1)
        LeadPosition(currentStep) = 100;
    end
end




%%
InitBreach;

S = BreachSimulinkSystem('ACC_MP1_b');
tspan = 0:Ts:endTime;
S.SetTime(tspan);
inputGen.type = 'UniStep';
inputGen.cp = 20;
S.SetInputGen(inputGen);
params = cell(1,20);
for jj=1:20
    params{1,jj} = sprintf('dist_u%d',jj-1);
end
%%
S.PrintParams;
S.SetParam(params, LeadPosition(1:20));
S.PrintParams;
%%
numSimulations = 20;
S.QuasiRandomSample(numSimulations);
S.Sim;
%%
% clc;
STL_ReadFile('./ACC_Requirements.stl');
fprintf('Checking safe following ..');
robs1 = S.CheckSpec(safeFollowing)

disp('Checking forward progress');
robs2 = S.CheckSpec(forwardProgress)

disp('Checking if cruising works');
robs3 = S.CheckSpec(cruiseWhenNotImpeded)

disp('Checking that the host stops only if required');
robs4 = S.CheckSpec(dontStopUnlessLeadStops)

%% Pretty-printing plots by getting signal values and grouping them 
% together. DO NOT USE this if numSimulations is a large number, because
% Matlab will open 'numSimulations' figure windows, and it could slow down
% your Matlab or even cause it to crash!
signalValues = S.GetSignalValues({'HostPosition','LeadPosition'});
close all;

% as a safeguard, you could use min(numSimulations,20) instead of
% numSimulations as the loop termination condition below:
figure;
trace = signalValues;
subplot(5,1,1);
plot(tspan,trace(1,:),'-r.', tspan, trace(2,:), '-bx');
title('Position');