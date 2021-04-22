clear;
clc;

%% Define a geometry of the system
trans = createRotationOx(deg2rad(25)); % source and detector will be at 25 degree angle from vertical
% X-ray source
source_origin_vector = transformPoint3d([0, 0, 471.8], trans);
source = Source(source_origin_vector);
% X-ray detector
detector_origin_vector = transformPoint3d([0, 0, -152.5], trans);
detector = Detector(detector_origin_vector,...
    8 * 1.3500e-01,...
    [1, 204],...
    unit_vect(detector_origin_vector),...
    250);
% Conveyor belt
conveyor_belt = ConveyorBelt([0, 1, 0],...  % direction
                         [0, 0, 1],...    % normal
                         270,...            % speed
                         -75,...            % trigger position
                         30,...             % trigger height
                         0);                % delay

%% Define the line scanner
line_scanner = LineScanner(source, detector, conveyor_belt, 'NumberOfScans', 250);

%% Load a sample mesh
mesh = createCube();
mesh.vertices = mesh.vertices * 50;
mesh = rmfield(mesh,'edges');
mesh.vertices = (mesh.vertices - mean(mesh.vertices, 1)); % center around origin

% Position on belt
mesh = line_scanner.conveyor_belt.place_on_belt(mesh, -150, 0, 0);

%% Depth camera
cam_position = [0, 200, 200];
cam_ref_frame = transformPoint3d(eye(3),createRotationOx(-3*pi/4));

depth_cam = DepthCam('CameraPosition', cam_position,...
    'CameraRefFrame', cam_ref_frame);

%% Multisensor system
multi_sensor_system = MultiSensorSystem(line_scanner, depth_cam);

%% Move mesh to moment of trigger activation
mesh = multi_sensor_system.line_scanner.conveyor_belt.calc_start(mesh);

%% Visualize the initial scene
multi_sensor_system.plot_geometry('Mesh', mesh)

%% Simulate the projection using Lambert-Beer law
tic
[scan, mesh_measured, meshes, mesh_depth] = multi_sensor_system.multi_sensor_scan(mesh);
toc
figure;imshow(mat2gray(scan))

%% Visualize intermediate scene
% intermediate_mesh = meshes{round(end/2)};
% line_scanner.plot_geometry('Mesh', intermediate_mesh)

%% Visualize scene of depth measurement
multi_sensor_system.plot_geometry('Mesh', mesh_depth)

%% Show depth camera scene
multi_sensor_system.depth_cam.show_scene(mesh_depth, 0)
multi_sensor_system.depth_cam.show_scene(mesh_depth, 1)


