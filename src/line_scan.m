clear;
clc;
%% Define the conveyorbelt
conv_belt = ConveyorBelt([0, 1, 0],...      % direction
                         [0.1,0,0.9],...    % normal
                         270,...            % speed
                         -75,...            % trigger position
                         30,...             % trigger height
                         0);                % delay

%% Define a geometry of the system
geometry.source_origin_vector   = [0, 0, 471.8];
geometry.detector_origin_vector = [0, 0, -152.5];

trans = createRotationOx(deg2rad(26));
geometry.source_origin_vector = transformPoint3d(geometry.source_origin_vector, trans);
geometry.detector_origin_vector = transformPoint3d(geometry.detector_origin_vector, trans);

geometry.pixel_size             = 8 * 1.3500e-01;
geometry.detector_size_px       = [1, 204];
geometry.detector_normal        = unit_vect(geometry.detector_origin_vector);
geometry.n_scans                = 250;
geometry.line_rate              = 250;

%% Create a cad_projector instance with the geometry
my_cad_projector = cad_projector();
my_cad_projector = my_cad_projector.build(geometry);

%% Load a sample mesh
mesh = stlread('apple.stl');
mesh = reducepatch(mesh, 0.1); % Reduce number of vertices
mesh = smooth_this_mesh(mesh, 1);
mesh.vertices = (mesh.vertices - mean(mesh.vertices, 1)); % Center
% Apply a random rotation
random_axis  = rand(1,3);
random_axis  = random_axis / norm(random_axis); 
random_angle = rand(1) * 2 * pi;
rotm = axang2rotm([random_axis, random_angle]);
mesh.vertices = mesh.vertices * rotm;
% Find contact point with conveyor belt
pos = linePosition3d(mesh.vertices, [[0,0,0],conv_belt.normal]);
translation = min(pos);
mesh.vertices = mesh.vertices - translation * conv_belt.normal;
% Initial position of the mesh
start_point = -150 * conv_belt.direction;
moving_mesh = mesh;
moving_mesh.vertices = moving_mesh.vertices + start_point;

%% Visualize initial scene
ray.vertices = [geometry.source_origin_vector;...
    my_cad_projector.detector_points(1,:);...
    my_cad_projector.detector_points(end,:)];
ray.faces = [1, 2, 3];
box = [geometry.source_origin_vector, 30];
figure;
patch(moving_mesh,'FaceColor', [0.3, 0.8, 0.3], 'FaceAlpha', 0.5, 'EdgeAlpha', 0.3)
patch(ray, 'FaceColor', 'y', 'FaceAlpha', 0.3, 'EdgeColor', 'None')
axis equal; view(3); rotate3d on
hold on
scatter3(my_cad_projector.detector_points(:,1),...
    my_cad_projector.detector_points(:,2),...
    my_cad_projector.detector_points(:,3));
scatter3(my_cad_projector.source_origin_vector(1),...
    my_cad_projector.source_origin_vector(2),...
    my_cad_projector.source_origin_vector(3), 100, 'filled', 'k');
drawLine3d(conv_belt.trigger, 'Color', 'r')
drawPlane3d(conv_belt.plane, 'FaceColor', [0.9, 0.9, 0.9], 'FaceAlpha', 0.5)
drawCube(box, 'FaceColor', [0.9, 0.9, 0.9], 'FaceAlpha', 0.5)
axis equal; view(3); rotate3d on
xlabel('X')
ylabel('Y')
zlabel('Z')

%% Move mesh to moment of trigger activation
[translation, ~, closest_pt] = conv_belt.calc_start(moving_mesh);
moving_mesh.vertices         = moving_mesh.vertices + translation;
closest_pt = closest_pt + translation;

%% Visualize the moment of trigger activation
figure;
patch(moving_mesh,'FaceColor', [0.3, 0.8, 0.3], 'FaceAlpha', 0.5, 'EdgeAlpha', 0.3)
patch(ray, 'FaceColor', 'y', 'FaceAlpha', 0.3, 'EdgeColor', 'None')
axis equal; view(3); rotate3d on
hold on
scatter3(my_cad_projector.detector_points(:,1),...
    my_cad_projector.detector_points(:,2),...
    my_cad_projector.detector_points(:,3));
scatter3(my_cad_projector.source_origin_vector(1),...
    my_cad_projector.source_origin_vector(2),...
    my_cad_projector.source_origin_vector(3), 100, 'filled', 'k');
scatter3(closest_pt(1),closest_pt(2),closest_pt(3),'filled','r')
drawLine3d(conv_belt.trigger, 'Color', 'r')
drawPlane3d(conv_belt.plane, 'FaceColor', [0.9, 0.9, 0.9], 'FaceAlpha', 0.5)
drawCube(box, 'FaceColor', [0.9, 0.9, 0.9], 'FaceAlpha', 0.5)
axis equal; view(3); rotate3d on
xlabel('X')
ylabel('Y')
zlabel('Z')

%% Calculate the position of the mesh for every line scan
duration    = geometry.n_scans/geometry.line_rate;
start_point = start_point + translation; % start at trigger activation
end_point   = start_point + conv_belt.speed_vector * duration;
positions   = linspaceNDim(start_point, end_point, geometry.n_scans)';

%% Simulate the projection using Lambert-Beer law
tic
n_proj      = size(positions, 1);
projs = NaN(n_proj, geometry.detector_size_px(2));
vertices = NaN(size(moving_mesh.vertices,1), 3, n_proj);
for i = 1:n_proj
vertices(:,:,i) = mesh.vertices + positions(i,:);
moving_mesh.vertices = vertices(:,:,i);
projs(i,:) = my_cad_projector.get_projection(moving_mesh,...
                                             'LambertBeer', true,...
                                             'LinearAttenuationCoeff', 0.0015);
end
toc
figure;imshow(mat2gray(projs))

%% Visualize intermediate scene
intermediate_mesh = moving_mesh;
intermediate_mesh.vertices = vertices(:,:,round(end/2));
figure;
patch(intermediate_mesh,'FaceColor', [0.3, 0.8, 0.3], 'FaceAlpha', 0.5, 'EdgeAlpha', 0.3)
patch(ray, 'FaceColor', 'y', 'FaceAlpha', 0.3, 'EdgeColor', 'None')
axis equal; view(3); rotate3d on
hold on
scatter3(my_cad_projector.detector_points(:,1),...
    my_cad_projector.detector_points(:,2),...
    my_cad_projector.detector_points(:,3));
scatter3(my_cad_projector.source_origin_vector(1),...
    my_cad_projector.source_origin_vector(2),...
    my_cad_projector.source_origin_vector(3), 100, 'filled', 'k');
drawLine3d(conv_belt.trigger, 'Color', 'r')
drawPlane3d(conv_belt.plane, 'FaceColor', [0.9, 0.9, 0.9], 'FaceAlpha', 0.5)
drawCube(box, 'FaceColor', [0.9, 0.9, 0.9], 'FaceAlpha', 0.5)
axis equal; view(3); rotate3d on
xlabel('X')
ylabel('Y')
zlabel('Z')