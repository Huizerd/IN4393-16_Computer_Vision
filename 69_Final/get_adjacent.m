function [lower, upper] = get_adjacent(idx, direction, set_size, lower, upper)
% GET_ADJACENT Determines adjacent sets based on an index and direction,
%   while also considering sets that are already included.
%
% Inputs:
% - idx: index of set to add
% - direction: add at the top or bottom
% - set_size: number of sets
% - lower: bottom of the already done part
% - upper: top of the already done part
%
% Outputs:
% - lower: new bottom
% - upper: new top
%
% . & Michiel Mollema - 01.07.2018

% First time
if direction == 0
    
    % If not on edges of set
    if idx > 1 && idx < set_size
        lower = idx - 1;
        upper = idx + 1;
    elseif idx == 1
        lower = set_size;
        upper = idx + 1;
    elseif idx == set_size
        lower = idx - 1;
        upper = 1;
    end

% Going up
elseif direction > 0
    
    if upper < set_size
        upper = upper + 1;
    else
        upper = 1;
    end
    
% And going down   
elseif direction < 0
    
    if lower > 1
        lower = lower - 1;
    else
        lower = set_size;
    end
    
end
end
        