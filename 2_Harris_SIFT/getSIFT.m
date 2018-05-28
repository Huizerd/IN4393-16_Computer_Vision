function results = getSIFT(image, sigma)

laplacian = zeros([size(image) length(sigma)]);

% Columns: sigma, keypoint location, max laplacian, sigma for
% max, f, d, matches, scores
results = cell(length(sigma), 6);

for i = 1:length(sigma)
    
    results{i, 1} = sigma(i);
    [results{i, 2}, laplacian(:,:,i)] = harris(image, sigma(i));
    
    keypoints = reshape(abs(laplacian(repmat(results{i, 2}, [1 1 size(laplacian, 3)]))), 1, [], size(laplacian, 3));
    [results{i, 3}, results{i, 4}] = max(keypoints, [], 3);
    
    % Convert index to sigma
    results{i, 4} = sigma(results{i, 4});
    
    % Get rows and columns of keypoints
    [r,c] = find(results{i, 2} == 1);
    
    % Use VLFeat to match features (from http://www.vlfeat.org/download.html)
    [results{i, 5}, results{i, 6}] = vl_sift(single(image), 'Frames', [c'; r'; results{i, 4}(:)'; zeros(size(results{i, 4}(:)))'], 'Orientations');
    
end

end