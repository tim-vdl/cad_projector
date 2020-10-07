classdef CADProjector < handle
%CAD_PROJECTOR class for projecting a 3D mesh on a surface.
% The object can be used to calculate the projection image of a mesh
% from a source on to a detector, e.g., X-ray source and X-ray
% detector using a cone beam. It can also be used to calculate the
% shadow/silhouette of a shape onto a surface when seen from a 
% certain point.
%
% my_cad_projector = cap_projector(source, origin)  
%
% Properties
% ----------
% source:     X-ray source, see Source class
% detector:   X-ray detector, see Detector class
%   
% Methods
% -------
% projection = my_cad_projector.get_projection(input_mesh, varargin)
%     Calcultes the projection of the input_mesh onto the
%     detector surface using the specified geometry.
%
%     Optional Name-Value pairs:
%         'LambertBeer'               Boolean indicating whether 
%                                     Lambert-Beer law should be used
%                                     or not. If used, the
%                                     'LinearAttenuationCoeff' is
%                                     required.
%
%                                     Default: false
%
%         'LinearAttenuationCoeff'    Linear attenuation coefficient
%                                     (scaler) for when Lambert-Beer
%                                     law is used.
%
%                                     Default: []
%
%         'CenterMeshAroundOrigin'    Boolean indicating whether mesh
%                                     should be centered around the
%                                     origin [0, 0, 0] or not.
%
%                                     Default: []
    
    properties
        source
        detector
    end
    
    methods
        function obj = CADProjector(source, detector)
            obj.source = source;
            obj.detector = detector;
        end
        
        function projection = get_projection(obj, input_mesh, varargin)
            % Parse inputs
            p = inputParser;
            addRequired(p,  'InputMesh');
            addParameter(p, 'LambertBeer', false);
            addParameter(p, 'LinearAttenuationCoeff', []);
            addParameter(p, 'CenterMeshAroundOrigin', false);
            parse(p, input_mesh, varargin{:})
            
            use_lambert_beer         = p.Results.LambertBeer;
            linear_att_coeff         = p.Results.LinearAttenuationCoeff;
            center_mesh              = p.Results.CenterMeshAroundOrigin;
            
            % Center the mesh around [0, 0, 0] if requested.
            if center_mesh
                mn = mean(input_mesh.vertices, 1);
                input_mesh.vertices = input_mesh.vertices - mn;
            end
            
            % Initialize an empty vector to store the pixel values of the 
            % final projection image.
            points     = obj.detector.detector_points;
            n_points   = size(points, 1);
            projection = NaN(n_points, 1);
            
            % Loop over all detector points and calculate the pixel value
            % based on the distance traveled through the mesh.
            mesh_vertices   = input_mesh.vertices;
            mesh_faces      = input_mesh.faces;
            source_origin   = obj.source.source_origin_vector;
            
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
                        % that, ray is not passing through the mesh anymore
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
            projection = reshape(projection, obj.detector.detector_size_px); 
        end
    end
end

