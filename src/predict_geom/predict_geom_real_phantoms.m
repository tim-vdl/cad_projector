clear;
clc;

%% Ground truth image
% flat_field = double(ImportSlices('J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\xray-machine\flat-fields'));
flat_field = double(ImportSlices('J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\xray-machine\2021-04-26_phantom_scans\calibration\flat-fields'));
flat_field = mean(flat_field, 3);
% dark_field = double(ImportSlices('J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\xray-machine\dark-fields'));
dark_field = double(ImportSlices('J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\xray-machine\2021-04-26_phantom_scans\calibration\dark-fields'));
dark_field = mean(dark_field, 3);

% input_path = 'J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\pear\2018-2019_conference_PhD\sample_holder_cube\object_g06s02r04_20190510_151439_xray.tif';
input_path = 'J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\xray-machine\2021-04-26_phantom_scans\phantom_plate\phantom_plate.tif';
img = double(imread(input_path));
img = flat_field_correction(img, flat_field, dark_field);
img = imresize(img, 1/10);
img = img(40:end-41,:);
scan_gt = img > 0.5;
figure; imshow(scan_gt)

%% Load a phantom mesh
triangulation = stlread('J:\SET-MEBIOS-POSTHARVEST-DI0414\TimVanDeLooverbosch_u0117721\steel_plate_phantom.stl');
mesh.vertices = triangulation.Points;
mesh.faces = triangulation.ConnectivityList;
mesh.vertices = (mesh.vertices - mean(mesh.vertices, 1))/2; % center around origin
mesh = ensureManifoldMesh(mesh);
figure; patch(mesh, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'None')
axis equal; rotate3d on; view(3)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Lego phantom')

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
[scan_pred, line_scanner_pred] = simulate_scan(x0, mesh);
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
% line_scanner_pred.plot_geometry('Mesh', mesh_pred)

%% Optimize x
% https://www.mathworks.com/help/gads/how-globalsearch-and-multistart-work.html
tic;
% opts = optimoptions('patternsearch', ...
%     'Display', 'iter',...
%     'UseParallel', true,...
%     'MeshTolerance', 1e-10,...
%     'StepTolerance', 1e-10,...
%     'ConstraintTolerance', 1e-10);
problem = createOptimProblem('fmincon',...
    'objective',@(x)obj_fun(x, mesh, scan_gt, @no_processing, @iou),...
    'x0',x0,...
    'lb', lb,...
    'ub', ub,...
    'nonlcon', nonlcon,...
    'options',...
    optimoptions(@fmincon,'UseParallel', true));
ms = MultiStart('Display','iter');
[x,fval] = run(ms, problem, 50);

% x = patternsearch(@(x)obj_fun(x, mesh, scan_gt, @no_processing, @iou), x0, [], [], [], [], lb, ub, nonlcon, opts);
toc;
%% Show end result
[scan_pred, line_scanner_pred] = simulate_scan(x, mesh);
% loss = immse(scan_gt, scan_pred);
figure;
subplot(1,2,1); imshow(scan_gt); title('Scan A')
subplot(1,2,2); imshow(scan_pred); title('Scan B')

figure;
imshowpair(scan_gt, scan_pred); title('Difference between Scan A and B')

%%
line_scanner_pred.plot_geometry()

%% Objective function
function loss = obj_fun(x, mesh, scan_gt, process_func, error_func)
% Simulate scans
scan_pred = simulate_scan(x, mesh);
% Process the images
scan_gt = process_func(scan_gt);
scan_pred = process_func(scan_pred);
% Calculate the error
loss = error_func(scan_gt, scan_pred);
end

%% Process functions
function dist_img = img2bwdist(img)
dist_img = 1 - mat2gray(bwdist(~img));
end

function img = no_processing(img)
end

%% Error functions
function iou_score = iou(A, B)
iou_score = 1 - sum(A & B, 'all')/sum(A | B, 'all');
end

%% Constraints
function [c, ceq] = src_det_constraint(x)
source = x(1:3);
detector = x(4:6);
D = distancePoints3d(source, detector);
c = D - 625;
ceq = [];
end