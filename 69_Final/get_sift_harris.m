function [features, descriptors] = get_sift_harris(edges, image, sigma, threshold_R)
% GET_SIFT_HARRIS Obtains SIFT descriptors based on Harris corners.
%
% Inputs:
% - edges: either output of edge detector or grayscale image, input to
%   Harris
% - image: grayscale image, input to SIFT
% - sigma: scale-space
% - threshold_R: cornerness threshold
%
% Outputs:
% - features: SIFT feature keypoints
% - descriptors: SIFT feature descriptors
%
% Jesse Hagenaars & Michiel Mollema - 01.07.2018
    
% To store Laplacian
laplacian = zeros([size(edges) length(sigma)]);

for s = 1:length(sigma)

    % Get locations of Harris corners
    [r, c] = harris(edges, sigma(s), threshold_R);

    % Calculate Laplacian
    laplacian_i = sigma(s)^2 * (image_derivatives(edges, sigma(s), 'xx') + image_derivatives(edges, sigma(s), 'yy'));

    % Put in store
    laplacian(sub2ind(size(laplacian), r, c, ones(size(r)) * s)) = abs(laplacian_i(sub2ind(size(laplacian_i), r, c)));

end

% Get maximum response for each corner
max_laplacian = max(laplacian, [], 3);

% Select local maximum for close corners (3x3 block)
max_laplacian_block = imdilate(max_laplacian, strel('square', 3));
max_laplacian_clean = (max_laplacian_block == max_laplacian) & (max_laplacian > 0);

% Rows and cols of max responses
[r, c] = find(max_laplacian_clean);

% Store sigmas for max responses
sigma_max = zeros(size(edges));
 
for s = 1:length(sigma)
    sigma_max((max_laplacian_block == laplacian(:, :, s)) & (laplacian(:, :, s) ~= 0)) = sigma(s);
end

% Get max sigmas as list
sigma_list = sigma_max(sub2ind(size(sigma_max), r, c));
sigma_list = sigma_list * 2 + 1;

% Now do SIFT
[features, descriptors] = vl_sift(single(image), 'Frames', [c'; r'; sigma_list'; zeros(size(sigma_list))'], 'Orientations');

% Select only x, y from features
features = features(1:2, :);

end





