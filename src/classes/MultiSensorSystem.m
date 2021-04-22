classdef MultiSensorSystem
    %MULTISENSORSYSTEM 
    
    properties
        line_scanner
        depth_cam
    end
    
    methods
        function obj = MultiSensorSystem(line_scanner, depth_cam)
            %MULTISENSORSYSTEM
            check_line_scanner = @(x) isa(x, 'LineScanner');
            check_depth_cam = @(x) isa(x, 'DepthCam');
            p = inputParser();
            p.addRequired('line_scanner', check_line_scanner);
            p.addRequired('depth_cam', check_depth_cam);
            p.parse(line_scanner, depth_cam);
            obj.line_scanner = p.Results.line_scanner;
            obj.depth_cam = p.Results.depth_cam;
        end
        
        function mesh = move2depth_cam(obj, mesh)
            % MOVE2DEPTH_CAM calculates moves a mesh from its current position
            % on the conveyor belt to the position where the mesh is right in 
            % front depth camera. 
            % This position is determined by:
            %   1) Projecting the start position of the mesh (centroid) onto
            %       the direction vector of the conveyor belt (point A); 
            %   2) Finding the intersection between the Z-axis of the depth 
            %       camera and the plane of the conveyor belt; 
            %   3) Projecting this intersection point onto the direction 
            %       vector of the conveyor belt (point B); 
            %   4) Calculating the duration of the object to move from
            %       point A to point B using the speed of the conveyor belt
            %   5) Calculating the position of the mesh when the depth
            %       image will be taken
            
            % Start position on line of transport direction
            start_position = mean(mesh.vertices, 1);
            proj_start_on_direction = linePosition3d(start_position, ...
                [0,0,0, unit_vect(obj.line_scanner.conveyor_belt.speed_vector)]);
            % End position  on line of transport direction
            line = [obj.depth_cam.cam_position, obj.depth_cam.cam_ref_frame(:,3)'];
            plane = obj.line_scanner.conveyor_belt.plane;
            intersect_camZ_belt = intersectLinePlane(line, plane);
            proj_end_on_direction = linePosition3d(intersect_camZ_belt,...
                [0,0,0, unit_vect(obj.line_scanner.conveyor_belt.speed_vector)]);
            % Duration to move from start to end
            duration = (proj_end_on_direction - proj_start_on_direction)/...
                norm(obj.line_scanner.conveyor_belt.speed_vector);
            % Mesh moved to position for depth image
            translation   = obj.line_scanner.conveyor_belt.speed_vector * duration;
            mesh.vertices = mesh.vertices + translation;
        end
        
        function [scan, mesh_measured, meshes, mesh_depth] = multi_sensor_scan(obj, mesh, varargin)
            %MULTI_SENSOR_SCAN
            p = inputParser;
            p.StructExpand = false;
            addRequired(p, 'mesh');
            addParameter(p, 'LambertBeer', false);
            addParameter(p, 'LinearAttenuationCoeff', []);
            parse(p, mesh, varargin{:});
            
            lambert_beer = p.Results.LambertBeer;
            lin_att_coef = p.Results.LinearAttenuationCoeff;
            
            [scan, meshes] = obj.line_scanner.line_scan(mesh,...
                'LambertBeer', lambert_beer,...
                'LinearAttenuationCoeff', lin_att_coef);
            
            mesh_depth = obj.move2depth_cam(mesh);
            mesh_measured = obj.depth_cam.transform_mesh(mesh_depth);
        end
        
        function plot_geometry(obj, varargin)
            p = inputParser();
            p.KeepUnmatched=true; % Spelling mistakes will not me found
            % Conveyor belt
            addParameter(p, 'ConveyorBeltColor', [0.9, 0.9, 0.9]);
            addParameter(p, 'ConveyorBeltAlpha', 0.5);
            % Trigger
            addParameter(p, 'TriggerColor', 'r');
            parse(p, varargin{:})
            
            obj.line_scanner.plot_geometry(varargin{:});
            
            % Depth camera
            scale = 100;
            X = obj.depth_cam.cam_ref_frame(:,1);
            Y = obj.depth_cam.cam_ref_frame(:,2);
            Z = obj.depth_cam.cam_ref_frame(:,3);
            % Depth cam position
            scatter3(obj.depth_cam.cam_position(1), ...
                obj.depth_cam.cam_position(2),...
                obj.depth_cam.cam_position(3), 50)
            % X axis depth cam
            quiver3(obj.depth_cam.cam_position(1), obj.depth_cam.cam_position(2), obj.depth_cam.cam_position(3),...
                X(1),X(2),X(3), scale,'Color', 'r')
            % Y axis depth cam
            quiver3(obj.depth_cam.cam_position(1), obj.depth_cam.cam_position(2), obj.depth_cam.cam_position(3),...
                Y(1),Y(2),Y(3), scale,'Color', 'g')
            % Z-axis depth cam
            quiver3(obj.depth_cam.cam_position(1), obj.depth_cam.cam_position(2), obj.depth_cam.cam_position(3),...
                Z(1),Z(2),Z(3), scale,'Color', 'b')
        end
    end
end


