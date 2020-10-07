classdef Source
    %SOURCE
    % Properties
    % ----------
    % source_orgin_postion:     Position of the origin (center) of the
    %                           source
    %                               1x3 vector
    
    properties
        source_orgin_postion
    end
    
    methods
        function obj = Source(source_orgin_postion)
            obj.source_orgin_postion = source_orgin_postion;
        end
    end
end

