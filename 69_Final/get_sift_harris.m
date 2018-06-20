function [features, descriptors] = get_sift_harris(image, sigma, threshold_R)

% Derivative scale = gamma * integration scale
gamma = 0.7;
    
% To store Laplacian
laplacian = zeros([size(image) length(sigma)]);

for s = 1:length(sigma)

    % Get locations of Harris corners
    [r, c] = harris(image, sigma(s), threshold_R);

    % Calculate Laplacian
    laplacian_i = sigma(s)^2 * (imageDerivatives(image, sigma(s)*gamma, 'xx') + imageDerivatives(image, sigma(s)*gamma, 'yy'));

    % Put in store
    laplacian(sub2ind(size(laplacian), r, c, ones(size(r)) * s)) = abs(laplacian_i(sub2ind(size(laplacian_i), r, c)));

end

% Get maximum response for each corner
max_laplacian = max(laplacian, [], 3);

% Select local maximum for close corners (3x3 block)
max_laplacian_block = imdilate(max_laplacian, strel('square', 3));
max_laplacian_clean = (max_laplacian_block == max_laplacian) & (max_laplacian > 0);

% Final rows and cols
[r, c] = find(max_laplacian_clean);

% Store sigmas for max responses
sigma_max = zeros(size(image));
        
for s = 1:length(sigma)
    
    sigma_max((max_laplacian_block == laplacian(:, :, s)) & (laplacian(:, :, s) ~= 0)) = sigma(s);
    
end

% Get max sigmas as list
sigma_list = sigma_max(sub2ind(size(sigma_max), r, c));

% Now do SIFT
[features, descriptors] = vl_sift(single(image), 'Frames', [c'; r'; (2*sigma_list + 1)'; zeros(size(sigma_list))'], 'Orientations');

end





