% Get largest set of common points to start
% sizes = cellfun('size', points_common(1, :), 1);
% [~, order] = sort(sizes, 'descend');
order = [3 2 1 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19];

% Stitch to this first set
S_final = S{1, order(1)};
colors_final = colors{1, order(1)};

% Also try with pointCloud object
inverted_scene = [S_final(1, :); -S_final(3, :); -S_final(2, :)]';
scene = pointCloud(inverted_scene, 'Color', uint8(colors_final'));
merge_size = 0.01;

% Go over the sets
for i = 1:length(order)
    if ~isempty(points_common{1, order(i)})       
        if i == 1
            
            main_idx = order(i);
            neighbour_idx = order(i) + 1;
            
            main_view = S{1, main_idx}(:, points_common{1, order(i)})';
            neighbour_view = S{1, neighbour_idx}(:, points_common{2, order(i)})';
            
        else
            if order(i) < order(i-1)

                main_idx = order(i) + 1;
                neighbour_idx = order(i);

                main_view = S{1, main_idx}(:, points_common{2, order(i)})';
                neighbour_view = S{1, neighbour_idx}(:, points_common{1, order(i)})';

            else

                main_idx = order(i);
                neighbour_idx = order(i) + 1;

                main_view = S{1, main_idx}(:, points_common{1, order(i)})';
                neighbour_view = S{1, neighbour_idx}(:, points_common{2, order(i)})';

            end
        end

       % Save colors
        colors_final = [colors_final colors{1, neighbour_idx}];

        % Get transformation between 3D point sets
        % Transpose since x, y, z have to be columns
        [d, K, tform] = procrustes(main_view, neighbour_view);

        % Do transform, see MATLAB's procrustes documentation
        Z = tform.b * S{1, neighbour_idx}' * tform.T + tform.c(1, :);

        % SAVE Z IN S FOR NEXT TRANSFORM
        S{1, neighbour_idx} = Z';

        if d < 0.1

            % Convert to pointCloud
            new_cloud = pointCloud([Z(:, 1) -Z(:, 3) -Z(:, 2)], 'Color', uint8(colors{1, neighbour_idx}'));

            % Extend final 3D point set --> check dimensions
            S_final = [S_final Z'];

            % Also merge clouds
            scene = pcmerge(scene, new_cloud, merge_size);

            figure
            subplot(1, 2, 1)
            scatter3(main_view(:, 1), main_view(:, 2), main_view(:, 3), 'b.')
            hold on
            scatter3(neighbour_view(:, 1), neighbour_view(:, 2), neighbour_view(:, 3), 'r.')
            scatter3(K(:, 1), K(:, 2), K(:, 3), 'g.')
            title(num2str(d))
            subplot(1, 2, 2)
            pcshow(scene, 'MarkerSize', 100)

        end    
    end
end
