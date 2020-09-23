classdef ConveyorBelt
    %CONVEYORBELT
    % Properties:
    % -----------
    % direction:    Direction in which the conveyor belt moves
    %                   1x3 vector
    % normal:       Normal of the plane in which conveyor belt operates
    %                   1x3 vector
    % speed:        Speed of the conveyor belt in the movement direction
    %                   double
    % speed_vector: Vector indicating speed in each direction
    % tilt:         Direction perpendicular to the movement direction and
    %               the normal
    %                   1x3 vector
    % plane         Plane in which conveyor belt operates
    %                   1x9 vector: [[0, 0, 0], direction, tilt] (see plane3d.m)
    % trigger       Line used as start signal for line scans if line is 
    %               crossed by the object on the conveyor belt
    %                   1x6 vector (see line3d.m)
    % trigger_height:   Height of the trigger line in conveyor belt's 
    %                   normal direction
    %                   double
    % trigger_position: Position of the trigger line along the movement
    %                   direction of the conveyor belt
    %                   double
    % delay:        Delay between trigger activation and signal to start
    %               the line scans
    %               double
    
    properties
        direction
        normal
        speed
        speed_vector
        tilt
        plane
        trigger
        trigger_height
        trigger_position
        delay
    end
    
    methods
        function obj = ConveyorBelt(direction, normal, speed, trigger_position, trigger_height, delay)
            obj.direction = unit_vect(direction);
            obj.normal = unit_vect(normal);
            obj.speed = speed;
            obj.speed_vector = speed * direction;
            obj.tilt = cross(obj.direction, obj.normal);
            obj.plane = [[0, 0, 0], obj.direction, obj.tilt];
            obj.trigger_height = trigger_height;
            obj.trigger_position = trigger_position;
            obj.trigger = [trigger_position * obj.direction + trigger_height * obj.normal,...
                                    obj.tilt];
            obj.delay = delay;
        end
        
        function [translation, time, closest_pt] = calc_start(obj, mesh)
        % CALC_START calculates when and where a mesh will intersect a trigger
        % Input
        % -----
        % mesh:         Struct with vertices and faces fields
        %
        % Output
        % ------
        % translation:  Translation (1x3 vector) to be done by the mesh to activate 
        %               the trigger
        % time:         Time (s) between start of the movement and trigger
        %               activation
        % closest_pt:   Coordinates of the point on the mesh that will activate the
        %               trigger
        
        % Find all points of the mesh that will intersect with the trigger
        trigger_plane = createPlane(obj.trigger(1:3), obj.normal);
        polys = intersectPlaneMesh(trigger_plane, mesh.vertices, mesh.faces);
        polys = rmoutliers(polys{1});
        n_points = size(polys, 1);
        % Find the closest point of the mesh to the trigger. This point
        % will activate (i.e. intersect with) the trigger first.
        smallest_dist = Inf;
        closest_pt = NaN(1,3);
        for i = 1:n_points
           point = polys(i,:);
           dist = distancePointLine3d(point, obj.trigger);
           if dist < smallest_dist
               smallest_dist = dist;
               closest_pt = point;
           end
        end
        % Calculate the time and translation of the mesh required for
        % activating the trigger
        time = smallest_dist/norm(obj.speed_vector) + obj.delay;
        translation = obj.speed_vector * time;
        end

    end
end

