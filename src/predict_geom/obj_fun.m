function [loss, g] = obj_fun(x, mesh, scan_gt, process_func, error_func)
% Simulate scans
scan_pred = simulate_scan(x, mesh);
% Process the images
scan_gt = process_func(scan_gt);
scan_pred = process_func(scan_pred);
% Calculate the error
loss = error_func(scan_gt, scan_pred);

% Equality constrains
source = x(1:3);
detector = x(4:6);
D = distancePoints3d(source, detector);
g(1) = D - 625;
end


