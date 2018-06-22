% Sets used, for BA later
S_used = zeros(1, size(S, 2));

% First, stitch 1st set of 3 to 1st set of 4 (delete partial columns)
triple = point_view_matrix(1:3, :); triple(:, ~all(triple, 1)) = [];
quad = point_view_matrix(1:4, :); quad(:, ~all(quad, 1)) = [];

% Match triple to begin of quad
[~, ~, ib] = intersect(quad(1:3, :)', triple', 'rows', 'stable');
[d, ~, tform] = procrustes(S{2, 1}', S{1, 1}(:, ib)');

% Apply transform
triple_t = tform.b * S{1, 1}' * tform.T + tform.c(1, :);

% Store transformed triple
S{1, 1} = triple_t';

% Only when score is good
if d < 0.1
    S_final = triple_t';
    S_used(1) = 1;
    colors_final = colors{1, 1};
    d_sum = d;
else
    S_final = [];
    colors_final = [];
    d_sum = 0;
end

% Rotate for proper viewing
t1 = pi / 1.4; t2 = pi / 1.2; t3 = -pi / 25;

A = [1 0 0 0; 0 cos(t1) sin(t1) 0; 0 -sin(t1) cos(t1) 0; 0 0 0 1];
A2 = [cos(t2) sin(t2) 0 0; -sin(t2) cos(t2) 0 0; 0 0 1 0; 0 0 0 1];
A3 = [cos(t3) 0 sin(t3) 0; 0 1 0 0; -sin(t3) 0 cos(t3) 0; 0 0 0 1];

% Also try with pointCloud object
inverted_scene = [S_final(1, :); S_final(2, :); -S_final(3, :)]';
scene = pointCloud(inverted_scene, 'Color', colors_final');
scene = pctransform(scene, affine3d(A * A2 * A3));
merge_size = 0.01;

% Now, simply shift the point-view matrix..
for s = 1:size(S, 2) - 2
    
    % Shift it to next set (123.. --> 234..)
    % Old needed for quad
    pvm_circ_old = circshift(point_view_matrix, -s+1, 1);
    pvm_circ = circshift(point_view_matrix, -s, 1);
    S_circ = circshift(S, -s, 2);
    S_circ_old = circshift(S, -s+1, 2);
    
    % Get new triple and quad (delete partial columns)
    triple = pvm_circ(1:3, :); triple(:, ~all(triple, 1)) = [];
    quad = pvm_circ_old(1:4, :); quad(:, ~all(quad, 1)) = [];
    
    % Match triple to end of quad (unlike for 1st set)
    [~, ~, ib] = intersect(quad(2:end, :)', triple', 'rows', 'stable');
    [d, ~, tform] = procrustes(S_circ_old{2, 1}', S_circ{1, 1}(:, ib)');
    
    % Apply transform
    triple_t = tform.b * S_circ{1, 1}' * tform.T + tform.c(1, :);
    
    % Store transformed triple
    S{1, s+1} = triple_t';
    dummy = S_circ{1, 1};  % for plotting
    S_circ{1, 1} = triple_t';
    
    % Add when good score
    if d < 0.1
        
        d_sum = d_sum + d;
        
        S_final = [S_final triple_t'];
        colors_final = [colors_final colors{1, s+1}];
        S_used(s+1) = 1;
        
        % Convert to pointCloud
        inverted_cloud = [triple_t(:, 1) triple_t(:, 2) -triple_t(:, 3)];
        new_cloud = pointCloud(inverted_cloud, 'Color', colors{1, s+1}');
        new_cloud = pctransform(new_cloud, affine3d(A * A2 * A3));

        % Also merge clouds
        scene = pcmerge(scene, new_cloud, merge_size);

        figure
        subplot(1, 2, 1)
        scatter3(S_circ_old{2, 1}(1, :), S_circ_old{2, 1}(2, :), S_circ_old{2, 1}(3, :), 'b.')
        hold on
        scatter3(dummy(1, :), dummy(2, :), dummy(3, :), 'r.')
        scatter3(triple_t(:, 1), triple_t(:, 2), triple_t(:, 3), 'g.')
        title(num2str(d))
        subplot(1, 2, 2)
        pcshow(scene)
        
    end
    
    % Get next quad (to link to transformed triple)
    quad = pvm_circ(1:4, :); quad(:, ~all(quad, 1)) = [];
    
    % Match begin of next quad to transformed triple
    [~, ia, ~] = intersect(triple', quad(1:3, :)', 'rows', 'stable');
    [~, ~, tform] = procrustes(S_circ{1, 1}(:, ia)', S_circ{2, 1}');
    
    % Store transformed quad
    S{2, s+1} = (tform.b * S_circ{2, 1}' * tform.T + tform.c(1, :))';   
    
end

% Different for last, to prevent large drift
% Shift it 18 times (number of sets - 1)
pvm_circ = circshift(point_view_matrix, -18, 1);
S_circ = circshift(S, -18, 2);

% Matching last quad to first triple
triple = point_view_matrix(1:3, :); triple(:, ~all(triple, 1)) = [];
quad = pvm_circ(1:4, :); quad(:, ~all(quad, 1)) = [];

% So last 3 rows of quad
% Match triple to end of quad (unlike for 1st set)
[~, ia, ~] = intersect(triple', quad(2:end, :)', 'rows', 'stable');
[~, ~, tform] = procrustes(S{1, 1}(:, ia)', S_circ{2, 1}');

% Apply transform
quad_t = tform.b * S_circ{2, 1}' * tform.T + tform.c(1, :);

% Store transformed quad
S{2, 19} = quad_t';
S_circ{2, 1} = quad_t';

% Now, match last triple to last quad
triple = pvm_circ(1:3, :); triple(:, ~all(triple, 1)) = [];

% Match triple to begin of quad
[~, ~, ib] = intersect(quad(1:3, :)', triple', 'rows', 'stable');
[d, ~, tform] = procrustes(S_circ{2, 1}', S_circ{1, 1}(:, ib)');

% Apply transform
triple_t = tform.b * S_circ{1, 1}' * tform.T + tform.c(1, :);

% Store transformed triple
S{1, 19} = triple_t';
dummy = S_circ{1, 1};  % for plotting
S_circ{1, 1} = triple_t';

% Add when good score
if d < 0.1

    d_sum = d_sum + d;

    S_final = [S_final triple_t'];
    S_used(end) = 1;
    colors_final = [colors_final colors{1, 19}];

    % Convert to pointCloud
    inverted_cloud = [triple_t(:, 1) triple_t(:, 2) -triple_t(:, 3)];
    new_cloud = pointCloud(inverted_cloud, 'Color', colors{1, 19}');
    new_cloud = pctransform(new_cloud, affine3d(A * A2 * A3));

    % Also merge clouds
    scene = pcmerge(scene, new_cloud, merge_size);

    figure
    subplot(1, 2, 1)
    scatter3(S_circ{2, 1}(1, :), S_circ{2, 1}(2, :), S_circ{2, 1}(3, :), 'b.')
    hold on
    scatter3(dummy(1, :), dummy(2, :), dummy(3, :), 'r.')
    scatter3(triple_t(:, 1), triple_t(:, 2), triple_t(:, 3), 'g.')
    title(num2str(d))
    subplot(1, 2, 2)
    pcshow(scene)

end