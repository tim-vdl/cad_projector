%% Define a geometry of the system
geometry.source_origin_distance     = [0 0 500];
geometry.detector_origin_distance   = [0 0 -500];
geometry.pixel_size                 = 1;
geometry.detector_size_px           = [250, 250];
geometry.detector_normal            = [0 0.5 1];

%% Create a cad_projector instance with the geometry
my_cad_projector = cad_projector();
my_cad_projector = my_cad_projector.build(geometry);

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
figure;
patch(mesh,'FaceColor', [0.3, 0.8, 0.3], 'FaceAlpha', 0.5, 'EdgeAlpha', 0.3)
axis equal; view(3); rotate3d on
hold on
scatter3(my_cad_projector.detector_points(:,1),...
    my_cad_projector.detector_points(:,2),...
    my_cad_projector.detector_points(:,3));
scatter3(my_cad_projector.source_position(1),...
    my_cad_projector.source_position(2),...
    my_cad_projector.source_position(3), 20, 'filled');
axis equal; view(3); rotate3d on
xlabel('X')
ylabel('Y')
zlabel('Z')

%% Simulate the projection using Lambert-Beer law
projection = my_cad_projector.get_projection(mesh,...
                                             'LambertBeer', true,...
                                             'LinearAttenuationCoeff', 0.0015);
figure;imshow(mat2gray(projection))
