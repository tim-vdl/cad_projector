classdef Source
    %SOURCE
    % Properties
    % ----------
    % source_origin_vector:      Position of the origin (center) of the
    %                           source
    %                               1x3 vector
    
    properties
        source_origin_vector
    end
    
    methods
        function obj = Source(source_origin_vector)
            obj.source_origin_vector = source_origin_vector;
        end
    end
end

