clear;
clc;

%% Define ground truth geometry
% Define the shared variables
pixel_size = 10 * 1.3500e-01;
detector_width = 164; 
line_rate = 250;
speed = 270;
n_scans = 90; %200;
trigger_height = 20;
delay = 0;
placement_direction = -150;
direction = [0, 1, 0];
normal = [0, 0, 1];

% Define variables specific to ground truth (must be estimated)
trigger_pos = -70;
placement_tilt = -15;
euler_mesh = [30 20 10];

% % Define the true geometry
% transf = createRotationOx(deg2rad(25)); % source and detector will be at 25 degree angle from vertical
% % X-ray source
% source_origin_vector = transformPoint3d([0, 0, 471.8], transf);
% source_gt = Source(source_origin_vector);
% % X-ray detector
% detector_origin_vector = transformPoint3d([0, 0, -152.5], transf);
% detector_gt = Detector(detector_origin_vector,...
%     pixel_size,...
%     [1, detector_width],...
%     unit_vect(detector_origin_vector),...
%     line_rate);
% % Conveyor belt
% conveyor_belt_gt = ConveyorBelt(direction,...   % direction
%                              normal,...         % normal
%                              speed,...          % speed
%                              trigger_pos,...    % trigger position
%                              trigger_height,... % trigger height
%                              delay);            % delay
% % Create ground truth vector
% x_gt = NaN(1,11);
% x_gt(1:3) = source_origin_vector;
% x_gt(4:6) = detector_origin_vector;
% x_gt(7)  = trigger_pos;
% x_gt(8)  = placement_tilt;
% x_gt(9:11) = euler_mesh;
% 
% % Define the ground truth line scanner
% line_scanner_gt = LineScanner(source_gt, detector_gt, conveyor_belt_gt, 'NumberOfScans', n_scans);
% 
% Create a mesh of a cube
mesh = createCube();
mesh.vertices = mesh.vertices * 50;
mesh = rmfield(mesh,'edges');
mesh.vertices = (mesh.vertices - mean(mesh.vertices, 1)); % center around origin
% 
% % Move mesh to start position on scanner A
% transf = eulerAnglesToRotation3d(euler_mesh);
% mesh_gt = transformMesh(mesh, transf);
% mesh_gt = line_scanner_gt.conveyor_belt.place_on_belt(mesh_gt, placement_direction, placement_tilt, 0);
% mesh_gt = line_scanner_gt.conveyor_belt.calc_start(mesh_gt);
% 
% % Simulate ground truth line scan
% [scan_gt, ~] = line_scanner_gt.line_scan(mesh_gt);
% scan_gt = mat2gray(scan_gt);
% % scan_gt = 1-mat2gray(bwdist(~(mat2gray(scan_gt)<1)));


%% Ground truth image

flat_field = double(ImportSlices('J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\xray-machine\flat-fields'));
flat_field = mean(flat_field, 3);
dark_field = double(ImportSlices('J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\xray-machine\dark-fields'));
dark_field = mean(dark_field, 3);

input_path = 'J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\pear\2018-2019_conference_PhD\sample_holder_cube\object_g06s02r04_20190510_151439_xray.tif';
img = double(imread(input_path));
img = flat_field_correction(img, flat_field, dark_field);
img = imresize(img, 1/10);
img = mat2gray(img);
img = img(40:end-71,:);

bw = ~(img < 0.80);
% scan_gt = double(bw);
scan_gt = 1 - mat2gray(bwdist(bw));

%% Define x0
transf = createRotationOx(deg2rad(25)); % source and detector will be at 25 degree angle from vertical
% X-ray source
source_origin_vector = transformPoint3d([0, 0, 470], transf);
% X-ray detector
detector_origin_vector = transformPoint3d([0, 0, -150], transf);

% Conveyor belt direction and normal
trigger_pos = -10;
placement_tilt = 0;
euler_mesh = [0 0 0];

x0 = NaN(1,11);
x0(1:3) = source_origin_vector;
x0(4:6) = detector_origin_vector;
x0(7)  = trigger_pos;
x0(8)  = placement_tilt;
x0(9:11) = euler_mesh;

%% Boundary conditions
lb = [-300, -300, 200,... % source
    -100, -100, -300,...  % detector
    -110,...              % trigger pos
    -20,...               % tilt
    -180, -180, -180];    % euler

ub = [300, 300, 1000,...  % source
    100, 100, 0.0,...       % detector
    0,...                 % trigger pos
    20,...                % tilt
    180, 180, 180];       % euler

nonlcon = @src_det_constraint;

%% Show start
[scan_pred, line_scanner_pred] = simulate_scans(x0, mesh);
% loss = immse(scan_gt, scan_pred);
figure();
hold on
subplot(1,2,1); imshow(scan_gt); title('Scan A')
subplot(1,2,2); imshow(scan_pred); title('Scan B')
hold off

figure();
imshowpair(scan_gt, scan_pred);

%% Show initial scene
% mesh_pred = line_scanner_pred.conveyor_belt.place_on_belt(mesh, -150, 0, 0);
% line_scanner_gt.plot_geometry('Mesh', mesh_gt)
% line_scanner_pred.plot_geometry('Mesh', mesh_pred)

%% Optimize x
tic;
opts = optimoptions('patternsearch', ...
    'Display', 'iter',...
    'UseParallel', true,...
    'MeshTolerance', 1e-10);
x = patternsearch(@(x)obj_fun(x, mesh, scan_gt, @img2bwdist, @immse), x0, [], [], [], [], lb, ub, nonlcon, opts);
toc;
%% Show end result
[scan_pred, line_scanner_pred] = simulate_scans(x, mesh);
loss = immse(scan_gt, scan_pred);
figure;
subplot(1,2,1); imshow(scan_gt); title('Scan A')
subplot(1,2,2); imshow(scan_pred); title('Scan B')

figure;
imshowpair(scan_gt, scan_pred); title('Difference between Scan A and B')

%%
%line_scanner_gt.plot_geometry()
line_scanner_pred.plot_geometry()

%% Function definitions
function [scan, line_scanner] = simulate_scans(x, mesh)
    % Define the shared variables
    pixel_size = 10 * 1.3500e-01;
    detector_width = 164; 
    line_rate = 250;
    speed = 270;
    n_scans = 90; %200;
    trigger_height = 20;
    delay = 0;
    placement_direction = -150;
    direction = [0, 1, 0];
    normal = [0, 0, 1];
    
    % Define the predicted geometry
    source_origin_vector = x(1:3);
    detector_origin_vector = x(4:6);
    trigger_pos = x(7);
    placement_tilt = x(8);
    euler_mesh = x(9:11);
        
    % X-ray source
    source = Source(source_origin_vector);
    % X-ray detector
    detector = Detector(detector_origin_vector,...
        pixel_size,...
        [1, detector_width],...
        unit_vect(detector_origin_vector),...
        line_rate);
    % Conveyor belt
    conveyor_belt = ConveyorBelt(direction,...      % direction
                                 normal,...         % normal
                                 speed,...          % speed
                                 trigger_pos,...    % trigger position
                                 trigger_height,... % trigger height
                                 delay);            % delay
    line_scanner = LineScanner(source, detector, conveyor_belt, 'NumberOfScans', n_scans);
    
    % Move mesh to start position on scanner B
    transf = eulerAnglesToRotation3d(euler_mesh);
    mesh = transformMesh(mesh, transf);
    mesh = line_scanner.conveyor_belt.place_on_belt(mesh, placement_direction, placement_tilt, 0);
    mesh = line_scanner.conveyor_belt.calc_start(mesh);

    % Simulate the line scans
    [scan, ~] = line_scanner.line_scan(mesh);
    scan = mat2gray(scan);
end

function loss = obj_fun(x, mesh, scan_gt, process_func, error_func)
% Simulate scans
scan_pred = simulate_scans(x, mesh);
% Process the image
scan_pred = process_func(scan_pred);
% Calculate the error
loss = error_func(scan_gt, scan_pred);
end

%% Process functions
function dist_img = img2bwdist(img)
dist_img = 1 - mat2gray(bwdist(~(mat2gray(img)<1)));
end

function bw = img2binary(img)
bw = mat2gray(img) < 1;
end

%% Error functions
function iou_score = iou(A, B)
iou_score = sum(A & B, 'all')/sum(A | B, 'all');
end

%% Constraints
function [c, ceq] = src_det_constraint(x)
source = x(1:3);
detector = x(4:6);
D = distancePoints3d(source, detector);
c = D - 625;
ceq = [];
end