function [translation, time, closest_pt] = calc_start(mesh, speed_vector, trigger, delay)
% CALC_START calculates if a mesh will intersect a trigger and when
% Input
% -----
% mesh:         Struct with vertices and faces fields
% speed_vector: 1x3 Vector indicating speed per second and direction of the 
%               mesh movement
% trigger:      3D line, defined by a point on the line and a direction 
%               vector
% delay:        Delay (s) between activation of the trigger and start of
%               the line scans 
%
% Output
% ------
% translation:  Translation (1x3 vector) to be done by the mesh to activate 
%               the trigger
% time:         Time (s) between start of the movement and trigger
%               activation
% closest_pt:   Coordinates of the point on the mesh that will activate the
%               trigger

normal = cross(trigger(4:end), speed_vector);
trigger_plane = createPlane(trigger(1:3), normal);

polys = intersectPlaneMesh(trigger_plane, mesh.vertices, mesh.faces);
polys = rmoutliers(polys{1});
n_points = size(polys, 1);

smallest_dist = Inf;
closest_pt = NaN(1,3);
for i = 1:n_points
   point = polys(i,:);
   dist = distancePointLine3d(point, trigger);
   if dist < smallest_dist
       smallest_dist = dist;
       closest_pt = point;
   end
end

time = smallest_dist/norm(speed_vector) + delay;
translation = speed_vector * time;

end

