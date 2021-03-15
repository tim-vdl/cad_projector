clear;
clc;

%% Define x0
transf = createRotationOx(deg2rad(25)); % source and detector will be at 25 degree angle from vertical
% X-ray source
source_origin_vector = transformPoint3d([0, 0, 470], transf);
% X-ray detector
detector_origin_vector = transformPoint3d([0, 0, -150], transf);

% Conveyor belt direction and normal
trigger_pos = -70;
placement_tilt = 0;
euler_mesh = [0, 0, 0];

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
    0.1, 1, 0.1,...       % direction
    0,...                 % trigger pos
    20,...                % tilt
    180, 180, 180];       % euler

nonlcon = @src_det_constraint;

%% Show start
[scan_A, scan_B, line_scanner_A, line_scanner_B] = simulate_scans(lb);
loss = immse(scan_A, scan_B);
figure();
hold on
subplot(1,2,1); imshow(scan_A); title('Scan A')
subplot(1,2,2); imshow(scan_B); title('Scan B')
hold off

figure();
imshowpair(scan_A, scan_B);

%% Show initial scene
% Create a mesh of a cube
mesh = createCube();
mesh.vertices = mesh.vertices * 50;
mesh = rmfield(mesh,'edges');
mesh.vertices = (mesh.vertices - mean(mesh.vertices, 1)); % center around origin

% Move meshes to start position
mesh_A = line_scanner_A.conveyor_belt.place_on_belt(mesh, -150, 0, 20);
mesh_A = line_scanner_A.conveyor_belt.calc_start(mesh_A);
mesh_B = line_scanner_B.conveyor_belt.place_on_belt(mesh, -150, 0, 0);
%mesh_B = line_scanner_B.conveyor_belt.calc_start(mesh_B);
line_scanner_A.plot_geometry('Mesh', mesh_A)
line_scanner_B.plot_geometry('Mesh', mesh_B)

%% Optimize x
tic;
opts = optimoptions('fmincon', ...
    'Display', 'iter',...
    'UseParallel', true);
x = fmincon(@obj_fun, x0, [], [], [], [], lb, ub, nonlcon, opts);
toc;
%% Show end result
[scan_A, scan_B, line_scanner_A, line_scanner_B] = simulate_scans(x);
loss = immse(scan_A, scan_B);
figure;
subplot(1,2,1); imshow(scan_A); title('Scan A')
subplot(1,2,2); imshow(scan_B); title('Scan B')

figure;
imshowpair(scan_A, scan_B); title('Difference between Scan A and B')

%%
line_scanner_A.plot_geometry()
line_scanner_B.plot_geometry()

%% Function definitions
function [scan_A, scan_B, line_scanner_A, line_scanner_B, x_gt] = simulate_scans(x)
    % Define the shared variables
    pixel_size = 8 * 1.3500e-01;
    detector_width = 120; 
    line_rate = 250;
    speed = 270;
    n_scans = 150;
    direction = [0, 1, 0];
    normal = [0, 0, 1];

    trigger_pos = -30;
    trigger_height = 20;
    delay = 0;
    placement_direction = -150;
    placement_tilt = -15;
    euler_mesh = [30 20 10];

    % Define the true geometry
    transf = createRotationOx(deg2rad(25)); % source and detector will be at 25 degree angle from vertical
    % X-ray source
    source_origin_vector = transformPoint3d([0, 0, 471.8], transf);
    source = Source(source_origin_vector);
    % X-ray detector
    detector_origin_vector = transformPoint3d([0, 0, -152.5], transf);
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
    % Create ground truth vector
    x_gt = NaN(1,11);
    x_gt(1:3) = source_origin_vector;
    x_gt(4:6) = detector_origin_vector;
    x_gt(7)  = trigger_pos;
    x_gt(8)  = placement_tilt;
    x_gt(9:11) = euler_mesh;
    
    
    % Define the ground truth line scanner
    line_scanner_A = LineScanner(source, detector, conveyor_belt, 'NumberOfScans', n_scans);

    % Create a mesh of a cube
    mesh = createCube();
    mesh.vertices = mesh.vertices * 50;
    mesh = rmfield(mesh,'edges');
    mesh.vertices = (mesh.vertices - mean(mesh.vertices, 1)); % center around origin

    % Move mesh to start position on scanner A
    trans = eulerAnglesToRotation3d(euler_mesh);
    mesh_A = transformMesh(mesh, trans);
    mesh_A = line_scanner_A.conveyor_belt.place_on_belt(mesh_A, placement_direction, placement_tilt, 0);
    mesh_A = line_scanner_A.conveyor_belt.calc_start(mesh_A);
    
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
    line_scanner_B = LineScanner(source, detector, conveyor_belt, 'NumberOfScans', n_scans);
    
    % Move mesh to start position on scanner B
    trans = eulerAnglesToRotation3d(euler_mesh);
    mesh_B = transformMesh(mesh, trans);
    mesh_B = line_scanner_B.conveyor_belt.place_on_belt(mesh_B, placement_direction, placement_tilt, 0);
    mesh_B = line_scanner_B.conveyor_belt.calc_start(mesh_B);

    % Simulate the line scans
    [scan_A, ~] = line_scanner_A.line_scan(mesh_A);
    [scan_B, ~] = line_scanner_B.line_scan(mesh_B);
    scan_A = mat2gray(scan_A);
    scan_B = mat2gray(scan_B);
end

function loss = obj_fun(x)
% Simulate scans
[scan_A, scan_B, ~, ~] = simulate_scans(x);

% Calculate the error
loss = immse(scan_A, scan_B);

end

%% Constraints
function [c, ceq] = src_det_constraint(x)
source = x(1:3);
detector = x(4:6);
D = distancePoints3d(source, detector);
c = D - 625;
ceq = [];
end