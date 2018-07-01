function [complete_S, complete_colors, complete_M, complete_D] = stitching(triple_models, quad_models, do_incremental_BA)
% STITCHING Stitches models of 3 frames together by making use of the models of
%   4 frames, which have a more consistent 3D geometry. Also includes
%   option to perform incremental BA between the last set stitched to the
%   main model and the current set.
%
% Inputs:
% - triple_models: models of 3 frames
% - quad_models: models of 4 frames
% - do_incremental_BA: perform incremental BA or not
%
% Outputs:
% - complete_S: complete 3D point set
% - complete_colors: colors for each point in the complete set
% - complete_D: complete set of centered point coordinates in the image
%
% Jesse Hagenaars & Michiel Mollema - 01.07.2018

%% Reorder

% Create new stitching order based on size of 4-frame models
[triple_order, quad_order] = create_order(quad_models);
track = quad_order(1);

% Cell arrays for updated models & colors
new_triple_models = cell(size(triple_models, 1), 1);
new_quad_models = cell(size(quad_models, 1), 1);

%% 1st pair of models

% First 4-frame model is not transformed
new_quad_models{quad_order(1), 1} = quad_models{quad_order(1), 1};
direction = 1;  % first stitch on top

% Select current 3- and 4-frame model
triple = triple_models{triple_order(1), 1};
quad = quad_models{quad_order(1), 1};

% Find current matching indices between 3- and 4-frame model
match_idx_triple = triple_models{triple_order(1), 2};   
match_idx_quad = quad_models{quad_order(1), 2}(1:3, :);

% Get colors for current 3- and 4-frame model
color_triple = triple_models{triple_order(1),3};
color_quad = quad_models{quad_order(1),3};

% Find intersection between 3- and 4-frame model
[~, ~, ib] = intersect(match_idx_quad', match_idx_triple', 'rows', 'stable');

% Do procrustes
[~, ~, tform] = procrustes(quad', triple(:, ib)');

% Apply transform to entire 3-frame model and save
new_triple = tform.b * triple' * tform.T + tform.c(1, :);
new_triple_models{triple_order(1)} = new_triple';

% Initialize main model to stitch to
complete_S = new_triple';
complete_colors = color_triple;
complete_M = triple_models{triple_order(1), 4};
complete_D = triple_models{triple_order(1), 5};

% Create last for both directions (stitching on top or bottom)
% For incremental bundle adjustment, since it has to know what the last
%   model stitched to the main model was on each 'side'
if do_incremental_BA
    
    last_S_0 = complete_S;
    last_S_1 = complete_S;
    last_M_0 = complete_M;
    last_M_1 = complete_M;
    last_D_0 = complete_D;
    last_D_1 = complete_D;
    
end


%% Remaining models

% Go over the other models, each time stitching a 3-frame model to a
%   4-frame model, then stitching the next 4-frame model to that 3-frame
%   model, etc.
% The 4-frame models serve as the 'skeleton' for the 3-frame models
for i = 2:length(quad_order)
    
    % Select current 3- and 4-frame model
    triple = triple_models{triple_order(i), 1};
    quad = new_quad_models{quad_order(i-1), 1};
    
    % Current centered image coordinates
    points_temp = triple_models{triple_order(i), 5};
    
    % Find current matching indices between 3- and 4-frame model
    match_idx_triple = triple_models{triple_order(i), 2};
    
    % Depending on direction, one has to select the beginning or the end of
    %   the 4-frame model
    if direction == 1
        match_idx_quad = quad_models{quad_order(i-1),2}(2:4,:);
    else
        match_idx_quad = quad_models{quad_order(i-1),2}(1:3,:);
    end

    % Find intersection between 3- and 4-frame model
    [~, ~, ib] = intersect(match_idx_quad', match_idx_triple', 'rows', 'stable');

    % Do procrustes
    [~, ~, tform] = procrustes(quad', triple(:, ib)');

    % Apply transform to entire 3-frame model and save
    new_triple = tform.b * triple' * tform.T + tform.c(1, :);
    new_triple_models{triple_order(i)} = new_triple';

    % Stitch at beginning or end of main model
    if direction == 1
        
        % Stitch at end
        complete_S = [complete_S new_triple'];
        complete_colors = [complete_colors; triple_models{triple_order(i),3}];
        complete_M = [complete_M; triple_models{triple_order(i), 4}(5:6, :)];
        complete_D(end-3:end+2, end+1:end+size(points_temp, 2)) = points_temp;

        if do_incremental_BA
            
            % Set last
            last_S = last_S_1; last_M = last_M_1; last_D = last_D_1;
            
            % Make subset of last S, M, D and current
            sub_S = [last_S new_triple'];
            sub_M = [last_M; triple_models{triple_order(i), 4}(5:6, :)];
            sub_D = zeros(size(last_D, 1) + 2, size(last_D, 2) + size(points_temp, 2));
            sub_D(1:size(last_D, 1), 1:size(last_D, 2)) = last_D;
            sub_D(end-size(points_temp, 1)+1:end, size(last_D, 2)+1:end) = points_temp;
            
        end        
    else
        
        % Stitch at beginning
        complete_S = [new_triple' complete_S];
        complete_colors = [triple_models{triple_order(i),3}; complete_colors];
        complete_M = [triple_models{triple_order(i), 4}(1:2, :); complete_M];
        dummy = complete_D;
        complete_D = zeros(size(dummy, 1) + 2, size(dummy, 2) + size(points_temp, 2));
        complete_D(1:size(points_temp, 1), 1:size(points_temp, 2)) = points_temp;
        complete_D(end-size(dummy, 1)+1:end, size(points_temp, 2)+1:end) = dummy;

        if do_incremental_BA
            
            % Set last
            last_S = last_S_0; last_M = last_M_0; last_D = last_D_0;
            
            % Make subset of last S, M, D and current
            sub_S = [new_triple' last_S];
            sub_M = [triple_models{triple_order(i), 4}(1:2, :); last_M];
            sub_D = zeros(size(last_D, 1) + 2, size(last_D, 2) + size(points_temp, 2));
            sub_D(1:size(points_temp, 1), 1:size(points_temp, 2)) = points_temp;
            sub_D(end-size(last_D, 1)+1:end, size(points_temp, 2)+1:end) = last_D;
            
        end
    end

    if do_incremental_BA
        
        % Do incremental bundle adjustment
        N_cam = size(sub_D, 1) / 2; N_pt = size(sub_D, 2);
        MS_0 = [sub_S(:); sub_M(:)];
        options = optimoptions(@fminunc, 'StepTolerance', 1e-16, 'OptimalityTolerance', 1e-16, 'FunctionTolerance', 1e-16, 'Display', 'iter');
        MS = fminunc(@(x)bundle_adjustment(sub_D, x, N_cam, N_pt), MS_0, options);

        % Reshape back
        S_BA = reshape(MS(1:N_pt*3), [3 N_pt]);
        M_BA = reshape(MS(end-N_cam*6+1:end), [2*N_cam 3]);

        % Put in final array, again take direction into account
        if direction == 1
            
            % Stitch at end
            complete_S(:, end-size(S_BA, 2)+1:end) = S_BA;
            complete_M(end-size(M_BA, 1)+1:end, :) = M_BA;
            last_S_1 = S_BA(:, end-size(new_triple, 1)+1:end);
            last_M_1 = M_BA(end-size(last_M, 1)+1:end, :);
            last_D_1 = points_temp;
            
            if i == 2
                
                % For 1st time, last model is same from both sides
                last_S_0 = S_BA(:, 1:size(last_S, 2));
                last_M_0 = M_BA(1:size(last_M, 1), :);
                
            end
        else
            
            % Stitch at beginning
            dummy = complete_S;
            complete_S = [S_BA dummy(:, size(S_BA, 2)+1:end)];
            dummy = complete_M;
            complete_M = [M_BA; dummy(size(M_BA, 1)+1:end, :)];
            last_S_0 = S_BA(:, 1:size(new_triple, 1));
            last_M_0 = M_BA(1:size(last_M, 1), :);
            last_D_0 = points_temp;
            
        end
    end
    
    % Stitch 4-frame model to last 3-frame model, but only if it's of
    %   decent size!
    if size(quad_models{quad_order(i)}, 2) > 100
        if find(triple_order(1:i) == quad_order(i))
            
            % Set next direction
            direction = 1;
            
            % Keep track of the 3-frame index
            triple_idx = find(triple_order(1:i) == quad_order(i));
            
            % Select current 3- and 4-frame model
            triple = new_triple_models{triple_order(triple_idx)};
            quad = quad_models{quad_order(i), 1};
            
            % Find current matching indices between 3- and 4-frame model
            match_idx_quad = quad_models{quad_order(i), 2}(1:3, :);
            match_idx_triple = triple_models{triple_order(triple_idx), 2};
            
            % Find intersection of points between four and three view
            [~, ~, ib] = intersect(match_idx_quad', match_idx_triple', 'rows', 'stable');    

            % Procrustes with matching points between three and four view
            [~, new_quad, ~] = procrustes(triple(:, ib)', quad');

            % Apply transform to entire quad view model and save in cell array
            new_quad_models{quad_order(i)} = new_quad';

        elseif ((quad_order(i) + 1) == track) || (quad_order(i) == 19 && triple_order(i) == 1)
            
            % Set next direction
            direction = 0;
            
            % Keep track of the 3-frame index
            triple_idx = find(triple_order == track);
            track = quad_order(i);
            
            % Select current 3- and 4-frame model
            triple = new_triple_models{triple_order(triple_idx)};
            quad = quad_models{quad_order(i), 1};
            
            % Find current matching indices between 3- and 4-frame model
            match_idx_quad = quad_models{quad_order(i), 2}(2:4, :);
            match_idx_triple = triple_models{triple_order(triple_idx), 2};
            
            % Find intersection of points between four and three view
            [~, ~, ib] = intersect(match_idx_quad', match_idx_triple', 'rows', 'stable');    

            % Procrustes with matching points between three and four view
            [~, new_quad, ~] = procrustes(triple(:, ib)', quad');

            % Apply transform to entire quad view model and save in cell array
            new_quad_models{quad_order(i)} = new_quad';

        end
    else
        fprintf('4-frame model %d < 100 pts: discarding!\n', quad_order(i));
        break
    end
end
end





















