classdef cad_projector < handle
    % cad_projector object for projecting a 3D mesh on a surface.
    %   The object can be used to calculate the projection image of a mesh
    %   from a source on to a detector, e.g., X-ray source and X-ray
    %   detector using a cone beam. It can also be used to calculate the
    %   shadow/silhouette of a shape onto a surface when seen from a 
    %   certain point.
    %
    %   my_cad_projector = cap_projector()  
    %       Initializes the object with empty properties.
    %
    %   Properties:
    %   source_origin_vector    - Position of the source in [X, Y, Z]
    %                             coordinates.
    %   detector_origin_vector  - Position of the detector center in 
    %                             [X, Y, Z] coordinates.
    %   detector_size_px        - Size of the detector in terms of 
    %                             pixels in each dimension.
    %   detector_size           - Number of pixels in each dimension
    %                             in real world units
    %   detector_points         - [X, Y, Z] coordinates of all individual
    %                             detector pixels
    %   pixel_size              - Size of an individual pixel in real
    %                             world units.
    %   detector_normal         - Normal of the detector surface
    %   
    %   Methods:
    %   my_cad_projector.build(geometry)  
    %       Processes a specified projection geometry and fills in the
    %       object's properties.
    %
    %       The geometry is a struct specifying the following properties:
    %           - source_origin_vector
    %           - detector_origin_vector
    %           - detector_size_px
    %           - pixel_size
    %
    %   projection = my_cad_projector.get_projection(input_mesh, varargin)
    %       Calcultes the projection of the input_mesh onto the
    %       detector surface using the specified geometry.
    %
    %       Optional Name-Value pairs:
    %           'LambertBeer'               Boolean indicating whether 
    %                                       Lambert-Beer law should be used
    %                                       or not. If used, the
    %                                       'LinearAttenuationCoeff' is
    %                                       required.
    %
    %                                       Default: false
    %
    %           'LinearAttenuationCoeff'    Linear attenuation coefficient
    %                                       (scaler) for when Lambert-Beer
    %                                       law is used.
    %
    %                                       Default: []
    %
    %           'CenterMeshAroundOrigin'    Boolean indicating whether mesh
    %                                       should be centered around the
    %                                       origin [0, 0, 0] or not.
    %
    %                                       Default: []
    
    properties
        % Source properties
        source_origin_vector
        
        % Detector properties
        detector_origin_vector
        detector_size_px
        detector_size
        detector_points
        pixel_size
        detector_normal 
    end
    
    methods
        function obj = cad_projector()
            %==============================================================
            % Constructor
            %==============================================================
            % Initialize the instance
            obj.source_origin_vector    = [];
            obj.detector_origin_vector  = [];
            obj.detector_size_px        = [];
            obj.detector_size           = [];
            obj.detector_points         = [];
            obj.pixel_size              = [];
            obj.detector_normal         = [];
        end
        
        function obj = build(obj, geometry)
            % Position the source
            obj.source_origin_vector = geometry.source_origin_vector;
            
            % Get the detector size in real world dimensions 
            obj.detector_origin_vector = geometry.detector_origin_vector;
            obj.detector_size_px       = geometry.detector_size_px;
            obj.pixel_size             = geometry.pixel_size;
            obj.detector_size          = obj.detector_size_px * obj.pixel_size;
            
            % Build detector
            x = linspace(0, obj.detector_size(2), obj.detector_size_px(2));
            x = x - mean(x);
            y = linspace(0, obj.detector_size(1), obj.detector_size_px(1));
            y = y - mean(y);
            z = obj.detector_origin_vector(3);
            [X, Y, Z] = meshgrid(x, y, z);
            points    = [X(:), Y(:), Z(:)];
            
            % Transform (rotate) points due to detector pose using the
            % detector normal.
            obj.detector_normal = normalizeVector3d(geometry.detector_normal);
            if ~isequal(obj.detector_normal, normalizeVector3d(-obj.detector_origin_vector))
                initial_normal = [0,0,1]; %- obj.detector_origin_vector;
                rotation_angle  = vectorAngle3d(obj.detector_normal,...
                                                initial_normal);
                rotation_axis   = cross(obj.detector_normal,...
                                        initial_normal);
                rotation_matrix = rotationmat3D(rotation_angle,...
                                                rotation_axis);
                detector_position_final = transformPoint3d(obj.detector_origin_vector,...
                                                           rotation_matrix);
                translation = detector_position_final - obj.detector_origin_vector;
                obj.detector_points = transformPoint3d(points,...
                                                       rotation_matrix) - translation;
            else
                obj.detector_points = points;
            end
        end
        
        function projection = get_projection(obj, input_mesh, varargin)
            % Parse inputs
            p = inputParser;
            addRequired(p, 'InputMesh');
            addParameter(p, 'LambertBeer', false);
            addParameter(p, 'LinearAttenuationCoeff', []);
            addParameter(p, 'CenterMeshAroundOrigin', false);
            parse(p, input_mesh, varargin{:})
            
            use_lambert_beer         = p.Results.LambertBeer;
            linear_att_coeff         = p.Results.LinearAttenuationCoeff;
            center_mesh              = p.Results.CenterMeshAroundOrigin;
            
            % Initialize an empty vector to store the pixel values of the 
            % final projection image.
            n_points   = size(obj.detector_points, 1);
            projection = NaN(n_points, 1);
            
            % Center the mesh around [0, 0, 0] if requested.
            if center_mesh
                mn = mean(input_mesh.vertices, 1);
                input_mesh.vertices = input_mesh.vertices - mn;
            end
            
            % Loop over all detector points and calculate the pixel value
            % based on the distance traveled through the mesh.
            mesh_vertices   = input_mesh.vertices;
            mesh_faces      = input_mesh.faces;
            points          = obj.detector_points;
            source_origin   = obj.source_origin_vector;
            
            parfor p = 1:n_points
                fprintf('... working on point %i of %i \n', p, n_points)
                
                detector_point = points(p,:);
                ray_path       = [source_origin, detector_point - source_origin];
                intersection_points = intersectLineMesh3d(ray_path,...
                                                          mesh_vertices,...
                                                          mesh_faces);
                number_of_intersects = size(intersection_points, 1);
                
                if number_of_intersects > 1
                % Ray passes through the mesh
                    if rem(number_of_intersects, 2) ~= 0
                        % Odd, remove last intersection point because after 
                        % that, ray is not passing the mesh anymore
                        intersection_points = intersection_points(1:end-1,:);
                    end
                    % Calculate distance passed through mesh
                    enter_pts   = intersection_points(1:2:end,:); % Odd
                    exit_pts    = intersection_points(2:2:end,:); % Even
                    distance_in_mesh = sum(distancePoints3d(enter_pts, exit_pts));
                    
                    if use_lambert_beer 
                        projection(p)  = exp(-linear_att_coeff * distance_in_mesh);
                    else
                        total_distance = distancePoints3d(source_origin, detector_point);
                        projection(p)  = 1 - distance_in_mesh/total_distance;
                    end
                else
                    projection(p) = 1; % Maximum intensity
                end
            end
            % Reshape the vector with pixel values to the final projection
            % image shape.
            projection = reshape(projection, obj.detector_size_px); 
        end
    end
end

