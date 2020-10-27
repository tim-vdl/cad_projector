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
            p.StructExpand = false;
            p.addRequired('source', check_source);
            p.addRequired('detector', check_detector);
            p.addRequired('conveyor_belt', check_conveyor);
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
            obj.duration = duration;
        end
        
        function n_scans = calc_n_scans(obj)
            %CALC_N_SCANS
            n_scans = obj.duration * obj.detector.line_rate;
            obj.n_scans = n_scans;
        end
        
        function calc_positions(obj, start_position)
            end_position   = start_position + obj.conveyor_belt.speed_vector * obj.duration;
            obj.scan_positions = linspaceNDim(start_position, end_position, obj.n_scans)';
        end
        
        function [scan, meshes] = line_scan(obj, mesh, varargin)
            %LINESCAN
            p = inputParser;
            p.StructExpand = false;
            addRequired(p, 'mesh');
            addParameter(p, 'LambertBeer', false);
            addParameter(p, 'LinearAttenuationCoeff', []);
            parse(p, mesh, varargin{:});
            
            lambert_beer = p.Results.LambertBeer;
            lin_att_coef = p.Results.LinearAttenuationCoeff;
            
            start_position = mean(mesh.vertices,1);
            obj.calc_positions(start_position);
            n_proj = size(obj.scan_positions, 1);
            scan   = NaN(n_proj, obj.detector.detector_size_px(2));
            start_vertices = mesh.vertices;
            meshes = cell(n_proj,1);
            for i = 1:n_proj
                mesh.vertices = start_vertices + (obj.scan_positions(i,:) - start_position);
                meshes{i} = mesh;
                if lambert_beer
                    scan(i,:) = obj.get_projection(mesh,...
                        'LambertBeer', lambert_beer,...
                        'LinearAttenuationCoeff', lin_att_coef);
                else
                    scan(i,:) = obj.get_projection(mesh);
                end
            end
        end
        
        function plot_geometry(obj, varargin)
            plot_geometry@CADProjector(obj, varargin{:});
            p = inputParser();
            p.KeepUnmatched=true; % Spelling mistakes will not me found
            % Conveyor belt
            addParameter(p, 'ConveyorBeltColor', [0.9, 0.9, 0.9]);
            addParameter(p, 'ConveyorBeltAlpha', 0.5);
            % Trigger
            addParameter(p, 'TriggerColor', 'r');
            parse(p, varargin{:})
            
            drawLine3d(obj.conveyor_belt.trigger, 'Color', p.Results.TriggerColor)
            drawPlane3d(obj.conveyor_belt.plane,...
                'FaceColor', p.Results.ConveyorBeltColor,...
                'FaceAlpha', p.Results.ConveyorBeltAlpha)
        end
    end
end

