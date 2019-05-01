function [triple_order, quad_order] = create_order(quad_models)
% CREATE_ORDER Creates new stitching order for sets of 3 and 4 based on
%   size of the sets of 4.
%
% Inputs:
% - quad_models: sets of 4 frames
%
% Outputs:
% - triple_order: new stitching order for sets of 3
% - quad_order: new stitching order for sets of 4
%
% . & Michiel Mollema - 01.07.2018

% Find biggest set of 4 to start with
[~, idx_max_quad] = max(cellfun('size', quad_models(:, 1), 2));

% Find adjacent sets of 3 that can be stitched to this set, and set them as
%   bounds that will be grown later
[lower, upper] = get_adjacent(idx_max_quad, 0, size(quad_models, 1), 0, 0);

% Create order arrays
triple_order = [idx_max_quad upper];
quad_order = idx_max_quad;

% Go over all sets of 4 (except 1st)
for i = 2:size(quad_models, 1)
    
    % Determine on which side to stitch
    if size(quad_models{lower, 1}, 2) > size(quad_models{upper, 1}, 2)
        
        % Add
        quad_order = [quad_order lower];
        triple_order = [triple_order lower];
        
        % Get new lower
        [lower, ~] = get_adjacent(idx_max_quad, -1, size(quad_models, 1), lower, upper);
        
    else
        
        % Add to order of sets of 4
        quad_order = [quad_order upper];
        
        % Get new upper & add to order of sets of 3
        [~, upper] = get_adjacent(idx_max_quad, 1, size(quad_models, 1), lower, upper);
        triple_order = [triple_order upper];

    end
end

% Delete last element of order of sets of 3 (1 too many)
triple_order(end) = [];

end
    
    