%% Define a geometry of the system
source = Source([0, 0, 500]);
detector = Detector([0, 0, -500], 1, [250, 250], [0, 0.5, 1]);

%% Create a cad_projector instance with the source and detector geometry
cad_projector = CADProjector(source, detector);

%% Load a sample mesh
mesh = stlread('apple.stl');
mesh = reducepatch(mesh, 0.1); % Reduce number of vertices
mesh.vertices = (mesh.vertices - mean(mesh.vertices, 1)); % Center

% Apply a random rotation
random_axis  = rand(1,3);
random_axis  = random_axis / norm(random_axis); 
random_angle = rand(1) * 2 * pi;
rotm = axang2rotm([random_axis, random_angle]);
mesh.vertices = mesh.vertices * rotm;

% Visualize the mesh
figure;
patch(mesh,'FaceColor', [0.3, 0.8, 0.3], 'FaceAlpha', 0.5, 'EdgeAlpha', 0.3)
axis equal; view(3); rotate3d on

%% Visualize the geometry with the sample
cad_projector.plot_geometry('Mesh', mesh)

%% Simulate the projection using Lambert-Beer law
tic
projection = cad_projector.get_projection(mesh,...
                                             'LambertBeer', true,...
                                             'LinearAttenuationCoeff', 0.0015);
toc
figure;imshow(mat2gray(projection))