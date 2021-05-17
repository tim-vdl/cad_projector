function [loss] = obj_fun(x, x_geom, mesh, scan_gt, process_func, error_func)
% Simulate scans
scan_pred = simulate_scan(x_geom, x, mesh);
% Process the images
scan_gt = process_func(scan_gt);
scan_pred = process_func(scan_pred);
% Calculate the error
loss = error_func(scan_gt, scan_pred);
end


