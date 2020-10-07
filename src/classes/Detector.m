classdef Detector
    %DETECTOR
    % Properties
    % ----------
    % detector_origin_vector:   Position of the origin (center) of the
    %                           detector
    %                               1x3 vector
    % pixel_size:               Size of the pixels (isotropic) of the
    %                           detector in world units
    %                               float
    % detector_size_px:         Size of the detector in number of pixels in
    %                           each direction
    %                               1x2 vector of integers
    % detector_size:            Size of the detector in word units
    %                               1x2 vector
    % detector_normal:          Direction of the normal of the detector's
    %                           plane
    %                               1x3 vector
    % detector_points:          Positions of the individual pixels (points)
    %                           in 3D space
    %                               Nx3 matrix, with N as the number of pixels
    % line_rate:                Number of scans per second (Hz) taken by
    %                           the detector
    %                               float
    
    properties
        detector_origin_vector
        pixel_size              
        detector_size_px
        detector_size
        detector_normal
        detector_points
        line_rate
    end
    
    methods
        function obj = Detector(detector_origin_vector,pixel_size, detector_size_px, detector_normal, line_rate)
            obj.detector_origin_vector = detector_origin_vector;
            obj.pixel_size = pixel_size;             
            obj.detector_size_px = detector_size_px;
            obj.detector_normal = detector_normal;
            obj.line_rate = line_rate;
            
            % Get the detector size in real world dimensions
            obj.detector_size          = obj.detector_size_px * obj.pixel_size;
            
            % Calculate the positions of the pixels
            x = linspace(0, obj.detector_size(2), obj.detector_size_px(2));
            x = x - mean(x);
            y = linspace(0, obj.detector_size(1), obj.detector_size_px(1));
            y = y - mean(y);
            z = obj.detector_origin_vector(3);
            [X, Y, Z] = meshgrid(x, y, z);
            points    = [X(:), Y(:), Z(:)];
            
            % Transform (rotate) points to the detector pose using the
            % detector normal
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
    end
end

