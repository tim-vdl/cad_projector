function [scan, line_scanner] = simulate_scan(x, mesh)
    % Define the shared variables
    pixel_size = 10 * 1.3500e-01;
    detector_width = 164; 
    line_rate = 200;
    speed = 270;
    n_scans = 120; % 90; %200;
    trigger_height = 1;
    delay = 0;
    placement_direction = -150;
    direction = [0, 1, 0];
    normal = [0, 0, 1];
    
    % Define the predicted geometry
    source_origin_vector = x(1:3);
    detector_origin_vector = x(4:6);
    trigger_pos = x(7);
    placement_tilt = x(8);
    euler_mesh = x(9);
        
    % X-ray source
    source = Source(source_origin_vector);
    % X-ray detector
    detector = Detector(detector_origin_vector,...
        pixel_size,...
        [1, detector_width],...
        unit_vect(detector_origin_vector),...
        line_rate);
    % Conveyor belt
    conveyor_belt = ConveyorBelt(direction,...      % direction
                                 normal,...         % normal
                                 speed,...          % speed
                                 trigger_pos,...    % trigger position
                                 trigger_height,... % trigger height
                                 delay);            % delay
    line_scanner = LineScanner(source, detector, conveyor_belt, 'NumberOfScans', n_scans);
    
    % Move mesh to start position on scanner B
    transf = eulerAnglesToRotation3d([euler_mesh,0,0]);
    mesh = transformMesh(mesh, transf);
    mesh = line_scanner.conveyor_belt.place_on_belt(mesh, placement_direction, placement_tilt, 0);
    mesh = line_scanner.conveyor_belt.calc_start(mesh);

    % Simulate the line scans
    [scan, ~] = line_scanner.line_scan(mesh,...
        'LambertBeer', 0,...
        'LinearAttenuationCoeff', 0.02);
    scan = flip(scan,2);
    scan = mat2gray(scan) < 0.5;
end
