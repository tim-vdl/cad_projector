clear;
clc;

%% Define x0
trans = createRotationOx(deg2rad(25)); % source and detector will be at 25 degree angle from vertical
% X-ray source
source_origin_vector = transformPoint3d([0, 0, 470], trans);
source = Source(source_origin_vector);
% X-ray detector
detector_origin_vector = transformPoint3d([0, 0, -150], trans);

% Conveyor belt direction and normal
direction = unit_vect([0.03, 0.92, 0.05]);
normal = transformPoint3d(direction, createRotationOx(deg2rad(90)));

x0 = NaN(1,12);
x0(1:3) = source_origin_vector;
x0(4:6) = detector_origin_vector;
x0(7:9) = direction;
x0(10:12) = normal;

%% Optimize
opts = optimoptions('fmincon', ...
    'Display', 'iter',...
    'UseParallel', true);
x = fmincon(@obj_fun, x0, [], [], [], [], [], [], [], opts);

%%
[loss, scan_A, scan_B] = obj_fun(x);
figure;
subplot(1,2,1); imshow(scan_A); title('Scan A')
subplot(1,2,2); imshow(scan_B); title('Scan B')

figure;
imshowpair(scan_A, scan_B); title('Difference between Scan A and B')

%% Objective function
function loss = obj_fun(x)
%% Define the shared variables
pixel_size = 8 * 1.3500e-01;
detector_width = 120; 
line_rate = 250;
speed = 270;
n_scans = 150;

trigger_pos = -30;
trigger_height = 30;
delay = 0;

%% Define the true geometry
trans = createRotationOx(deg2rad(25)); % source and detector will be at 25 degree angle from vertical
% X-ray source
source_origin_vector = transformPoint3d([0, 0, 471.8], trans);
source = Source(source_origin_vector);
% X-ray detector
detector_origin_vector = transformPoint3d([0, 0, -152.5], trans);
detector = Detector(detector_origin_vector,...
    pixel_size,...
    [1, detector_width],...
    unit_vect(detector_origin_vector),...
    line_rate);
% Conveyor belt
conveyor_belt = ConveyorBelt([0, 1, 0],...  % direction
                         [0, 0, 1],...    % normal
                         speed,...            % speed
                         trigger_pos,...            % trigger position
                         trigger_height,...             % trigger height
                         delay);                % delay

%% Define the ground truth line scanner
line_scanner_A = LineScanner(source, detector, conveyor_belt, 'NumberOfScans', n_scans);

%% Define the predicted geometry
source_origin_vector = x(1:3);
detector_origin_vector = x(4:6);
direction = x(7:9);
normal = x(10:12);

source = Source(source_origin_vector);
% X-ray detector
detector = Detector(detector_origin_vector,...
    pixel_size,...
    [1, detector_width],...
    unit_vect(detector_origin_vector),...
    line_rate);
% Conveyor belt
conveyor_belt = ConveyorBelt(direction,...  % direction
                         normal,...    % normal
                         speed,...            % speed
                         trigger_pos,...            % trigger position
                         trigger_height,...             % trigger height
                         delay);                % delay
line_scanner_B = LineScanner(source, detector, conveyor_belt, 'NumberOfScans', n_scans);

%% Create a mesh of a cube
mesh = createCube();
mesh.vertices = mesh.vertices * 50;
mesh = rmfield(mesh,'edges');
mesh.vertices = (mesh.vertices - mean(mesh.vertices, 1)); % center around origin

%% Move meshes to start position
mesh_A = line_scanner_A.conveyor_belt.place_on_belt(mesh, -100, 0, 0);
mesh_A = line_scanner_A.conveyor_belt.calc_start(mesh_A);
mesh_B = line_scanner_B.conveyor_belt.place_on_belt(mesh, -100, 0, 0);
mesh_B = line_scanner_A.conveyor_belt.calc_start(mesh_B);

%% Simulate the line scans
[scan_A, ~] = line_scanner_A.line_scan(mesh_A);
[scan_B, ~] = line_scanner_B.line_scan(mesh_B);
scan_A = mat2gray(scan_A);
scan_B = mat2gray(scan_B);

%% Calculate the error
loss = immse(scan_A, scan_B);

end