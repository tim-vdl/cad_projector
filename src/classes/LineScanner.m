classdef LineScanner < CADProjector
    %LINESCANNER 
    
    properties
        conveyor_belt
        duration
        n_scans
        scan_positions
    end
    
    methods
        function obj = LineScanner(source, detector, conveyor_belt, varargin)
            %LINESCANNER
            check_source = @(x) isa(x, 'Source');
            check_detector = @(x) isa(x, 'Detector');
            check_conveyor = @(x) isa(x, 'ConveyorBelt');
            check_single_value = @(x) sum(size(x))/ndims(x) == 1;
            check_positive_float = @(x) isnumeric(x) && check_single_value(x);
            check_integer = @(x) check_positive_float(x) && (mod(x,1) == 0);
            p = inputParser();
            p.addRequired(source, check_source);
            p.addRequired(detector, check_detector);
            p.addRequired(conveyor_belt, check_conveyor);
            p.addParameter('Duration', [], check_positive_float)
            p.addParameter('NumberOfScans', [], check_integer)
            p.parse(source, detector, conveyor_belt, varargin{:});
            
            obj@CADProjector(source, detector)
            obj.conveyor_belt = conveyor_belt;
            
            obj.duration = p.Results.Duration;
            obj.n_scans = p.Results.NumberOfScans;
            
            if isempty(obj.duration) && ~isempty(obj.n_scans)
                obj.calc_duration();
            elseif ~isempty(obj.duration) && isempty(obj.n_scans)
                obj.calc_n_scans();
            end
            
            assert(~isempty(obj.duration) && ~isempty(obj.n_scans),...
                "At least one of the Name-Value input pairs 'Duration' or 'NumberOfScans' must be defined")
            
        end
        
        function duration = calc_duration(obj)
            %CALC_DURATION
            duration = obj.n_scans/obj.detector.line_rate;
            obj.scan_duration = duration;
        end
        
        function n_scans = calc_n_scans(obj)
            %CALC_N_SCANS
            n_scans = obj.duration * obj.detector.line_rate;
            obj.n_scans = n_scans;
        end
        
        function calc_positions(obj, start_position)
            end_position   = start + obj.conveyor_belt.speed_vector * obj.duration;
            obj.scan_positions = linspaceNDim(start_position, end_position, obj.n_scans)';
        end
        
        function scan = line_scan(mesh)
            %LINESCAN
            n_proj = size(obj.scan_positions, 1);
            scan   = NaN(n_proj, obj.detector.detector_size_px(2));
            vertices = NaN(size(mesh.vertices,1), 3, n_proj);
            for i = 1:n_proj
                vertices(:,:,i) = mesh.vertices + obj.scan_positions(i,:);
                mesh.vertices = vertices(:,:,i);
                scan(i,:) = obj.get_projection(mesh,...
                    'LambertBeer', true,...
                    'LinearAttenuationCoeff', 0.0015);
            end
        end
    end
end

