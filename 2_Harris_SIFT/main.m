% Assignment 2: Harris Corner Detector
% Jesse Hagenaars & Michiel Mollema - 05-03-2018

sigma = 1.2.^(0:12);

image = uint8(mean(imread('landscape-a.jpg'), 3));

laplacian = zeros([size(image) length(sigma)]);

r = {};
c = {};

for i = 1:length(sigma)
        
    [r{i}, c{i}, laplacian(:,:,i)] = harris(image, sigma(i));
    
end

[idx, ~] = cellfun('length', r);
r = r{idx};
c = c{idx};

