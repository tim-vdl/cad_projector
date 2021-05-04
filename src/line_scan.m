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
triangulation = stlread('apple.stl');
mesh.vertices = triangulation.Points;
mesh.faces = triangulation.ConnectivityList;
mesh = reducepatch(mesh, 0.1); % Reduce number of vertices
mesh = smooth_this_mesh(mesh, 1);
mesh.vertices = (mesh.vertices - mean(mesh.vertices, 1)); % Center

% Apply a random rotation
random_axis  = rand(1,3);
random_axis  = random_axis / norm(random_axis); 
random_angle = rand(1) * 2 * pi;
rotm = axang2rotm([random_axis, random_angle]);
mesh.vertices = mesh.vertices * rotm;

% Position on belt
mesh = line_scanner.conveyor_belt.place_on_belt(mesh, -150, 0, 0);

%% Visualize the initial scene
line_scanner.plot_geometry('Mesh', mesh)

%% Move mesh to moment of trigger activation
mesh = line_scanner.conveyor_belt.calc_start(mesh);

%% Visualize the moment of trigger activation
line_scanner.plot_geometry('Mesh', mesh)

%% Simulate the projection using Lambert-Beer law
tic
[scan, meshes] = line_scanner.line_scan(mesh);
toc
figure;imshow(mat2gray(scan))

%% Visualize intermediate scene
intermediate_mesh = meshes{round(end/2)};
line_scanner.plot_geometry('Mesh', intermediate_mesh)
