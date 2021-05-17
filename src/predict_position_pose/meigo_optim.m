%% MEIGO Optimization
clear;
close all;
clc;

%% Load estimated geometry
data = load('../predict_geom/geometry_estimation_plate_phantom.mat');
x_geom = data.Results.xbest;

% Define x0
x0 = [x_geom(7),... % trigger position
      x_geom(8),... % tilt
      0,0,0];       % euler

%% Boundary conditions
lb = [-110,...          % trigger pos
    -20,...             % tilt
    -180, -90, -180];   % euler

ub = [0,...             % trigger pos
    20,...              % tilt
    180, 90, 180];      % euler

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

%img_path = "J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\pear\2018-2019_conference_PhD\RGBD_X\G0\xray\object_g00s04r04_20190509_144040_xray.tif";
img_path = "J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\pear\2018-2019_conference_PhD\RGBD_X\G0\xray\object_g00s04r01_20190509_143940_xray.tif";
img = double(imread(img_path));
img = flat_field_correction(img, flat_field, dark_field);
img = imresize(img, 0.1);
img = img(25:end-56,:);
scan_gt = img < 0.75;
figure;imshow(scan_gt)

%% Load phantom mesh
%load("J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\pear\2018-2019_conference_PhD\CT\conference_ct_ghb_20190510\G0\FV\g00s04.mat")
%mesh = this_roi;
sample = ImportSlices("J:\SET-Mebios_CFD-VIS-DI0327\TimVDL_shared\00_DATA\pear\2018-2019_conference_PhD\CT\conference_ct_ghb_20190510\G0\ROI\g00s04");
bw = sample > multithresh(sample,1);
bw = imfill(bw, 'holes');
mesh = isosurface(bw, 0.5);
mesh = reducepatch(mesh, 0.05);
mesh = smooth_this_mesh(mesh,1);
mesh = ensureManifoldMesh(mesh);
mesh.vertices = mesh.vertices - mean(mesh.vertices, 1);
mesh = align_mesh2axis(mesh, [0,-1,0]);
mesh.vertices = mesh.vertices - mean(mesh.vertices, 1);
figure;
patch(mesh, 'FaceColor', 'g', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'EdgeAlpha', 0.3)
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal, rotate3d on, view(3)

%% Compare ground truth and x0
[scan, line_scanner, mesh] = simulate_scan(x_geom, x0, mesh);
line_scanner.plot_geometry('Mesh', mesh)

figure;
subplot(1,2,1)
imshow(scan_gt)
subplot(1,2,2)
imshow(scan)

%% Optimization
tic;
Results = MEIGO(problem, opts, 'ESS', x_geom, mesh, scan_gt, @no_processing, @iou);
toc
%% Show result
[scan, line_scanner] = simulate_scan(x_geom, Results.xbest, mesh);

transf = eulerAnglesToRotation3d(Results.xbest(3:5));
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