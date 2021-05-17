%% MEIGO Optimization
clear;
close all;
clc;
%% Define x0
transf = createRotationOx(deg2rad(25)); % source and detector will be at 25 degree angle from vertical
% X-ray source
source_origin_vector = transformPoint3d([0, 0, 319.3], transf);
% X-ray detector
detector_origin_vector = transformPoint3d([0, 0, -152.5], transf);

% Conveyor belt direction and normal
trigger_pos = -10;
placement_tilt = 0;
euler_mesh = 0;

x0 = NaN(1,11);
x0(1:3) = source_origin_vector;
x0(4:6) = detector_origin_vector;
x0(7)  = trigger_pos;
x0(8)  = placement_tilt;
x0(9) = euler_mesh;

%% Boundary conditions
lb = [-100, -200, 250,... % source
    -100, 0, -200,...     % detector
    -110,...              % trigger pos
    -20,...               % tilt
    -30];                % euler

ub = [100, 0, 450,...     % source
    100, 100, -50,...     % detector
    0,...                 % trigger pos
    20,...                % tilt
    30];                 % euler

%% Problem specification
problem.f = 'obj_fun';
problem.x_L = lb;
problem.x_U = ub;
problem.x0 = x0;

opts.maxeval = 2000;
opts.local.solver = 'fmincon';
opts.local.iterprint=1;

%% Ground truth image
flat_field = double(ImportSlices('J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\xray-machine\2021-04-26_phantom_scans\calibration\flat-fields'));
flat_field = mean(flat_field, 3);
dark_field = double(ImportSlices('J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\xray-machine\2021-04-26_phantom_scans\calibration\dark-fields'));
dark_field = mean(dark_field, 3);

input_path = 'J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\xray-machine\2021-04-26_phantom_scans\phantom_plate\phantom_plate.tif';
img = double(imread(input_path));
img = flat_field_correction(img, flat_field, dark_field);
img = imresize(img, 1/10);
img = img(40:end-41,:);
scan_gt = img < 0.5;
figure; imshow(scan_gt)

%% Load phantom mesh
% triangulation = stlread('J:\SET-MEBIOS-POSTHARVEST-DI0414\TimVanDeLooverbosch_u0117721\plate_phantom_robovision.stl');
triangulation = stlread('J:\SET-MEBIOS-POSTHARVEST-DI0414\TimVanDeLooverbosch_u0117721\full_plate_phantom.stl');
mesh.vertices = triangulation.Points;
mesh.faces = triangulation.ConnectivityList;
mesh.vertices = (mesh.vertices - mean(mesh.vertices, 1))/2; % center around origin
mesh = transformMesh(mesh, createRotationOz(-pi/2));
mesh = transformMesh(mesh, createRotationOy(pi));
mesh = ensureManifoldMesh(mesh);
figure; patch(mesh, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'None')
axis equal; rotate3d on; view(3)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Phantom')

%% Compare ground truth and x0
[scan, line_scanner] = simulate_scan(x0, mesh);
line_scanner.plot_geometry('Mesh', mesh)

figure;
subplot(1,2,1)
imshow(scan_gt)
subplot(1,2,2)
imshow(scan)

%% Optimization
tic;
Results = MEIGO(problem, opts, 'ESS', mesh, scan_gt, @no_processing, @iou);
toc
%% Show result
[scan, line_scanner] = simulate_scan(Results.xbest, mesh);

transf = eulerAnglesToRotation3d([Results.xbest(9),0,0]);
mesh_final = transformMesh(mesh, transf);
line_scanner.plot_geometry('Mesh', mesh_final)

figure;
subplot(1,2,1)
imshow(scan_gt)
subplot(1,2,2)
imshow(scan)


%% Functions
% Process functions
function dist_img = img2bwdist(img)
dist_img = 1 - mat2gray(bwdist(~img));
end

function img = no_processing(img)
end

% Error functions
function iou_score = iou(A, B)
iou_score = 1 - sum(A & B, 'all')/sum(A | B, 'all');
end