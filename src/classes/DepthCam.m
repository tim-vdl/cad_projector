classdef DepthCam < handle
    % DepthCam
    properties
        cam_position % X, Y, Z coordinates of depth camera in world reference frame
        cam_ref_frame % X, Y, Z vectors (columns) of the depth camera in world reference frame
    end
    
    methods
        function obj = DepthCam(varargin)          
            p = inputParser;
            addParameter(p,'CameraPosition',[]);
            addParameter(p,'CameraRefFrame',eye(3));
            parse(p,varargin{:});                         
            
            obj.cam_position = p.Results.CameraPosition;
            obj.cam_ref_frame = p.Results.CameraRefFrame;
        end
        
    end
    
    methods (Access = public)
        
        function coords_transformed = transform_coords(obj, coords)
            % TRANSFORM_COORDS transforms the coordinates from world
            % reference frame to reference frame of the depth camera
            coords_transformed = (coords - obj.cam_position)...
                * obj.cam_ref_frame;
        end
        
        function mesh = transform_mesh(obj, mesh)
            % TRANSFORM_MESH transforms the coordinates of a mesh from world
            % reference frame to reference frame of the depth camera
            mesh.vertices = obj.transform_coords(mesh.vertices);
        end
        
        function depth_image = simulate_depth_cam(obj, mesh)
            % SIMULATE_DEPTH_CAM simulates a depth images measured by a
            % depth camera
            return
        end

        function show_scene(obj, mesh, cam_perspective)
            if cam_perspective 
                mesh = obj.transform_mesh(mesh);
                origin = [0,0,0];
                X = [1;0;0];
                Y = [0,1,0];
                Z = [0,0,1];
            else
                origin = obj.cam_position;
                X = obj.cam_ref_frame(:,1);
                Y = obj.cam_ref_frame(:,2);
                Z = obj.cam_ref_frame(:,3);
            end
            scale = 30;
            figure;
            patch('Faces', mesh.faces, 'Vertices', mesh.vertices, 'FaceAlpha', 0.3)
            hold on
            scatter3(origin(1), origin(2), origin(3), 30)
            quiver3(origin(1), origin(2), origin(3),...
                X(1), X(2), X(3), scale, 'Color', 'r')
            quiver3(origin(1), origin(2), origin(3),...
                Y(1), Y(2), Y(3), scale, 'Color', 'g')
            quiver3(origin(1), origin(2), origin(3),...
                Z(1), Z(2), Z(3), scale, 'Color', 'b')
            axis equal; rotate3d on; view(3)
        end
        
    end
end

