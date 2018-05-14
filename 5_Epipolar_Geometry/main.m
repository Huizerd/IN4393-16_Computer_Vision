% Assignment 5: Epipolar Geometry
% Jesse Hagenaars & Michiel Mollema - 07-05-2018

clear; clc

% run('C:\Users\jesse\Documents\MATLAB\vlfeat\toolbox\vl_setup')
run('/home/michiel/Programs/MATLAB/vlfeat/toolbox/vl_setup')

images = ["TeddyBearPNG/obj02_001.png", "TeddyBearPNG/obj02_002.png"];
descriptors = ["obj02_001.png.harhes.sift", "obj02_002.png.harhes.sift"];

for i = 2:length(images)
    
    % Image 1
    im1 = uint8(mean(imread(char(images(i-1))), 3));
    output1 = dlmread(descriptors(i-1), ' ', 2, 0);
    
    x1 = output1(:, 1)';
    y1 = output1(:, 2)';
    a1 = output1(:, 3)';
    b1 = output1(:, 4)';
    c1 = output1(:, 5)';
    desc1 = output1(:, 6:end)';
    
    % Image 2
    im2 = uint8(mean(imread(char(images(i))), 3));
    output2 = dlmread(descriptors(i), ' ', 2, 0);
    
    x2 = output2(:, 1)';
    y2 = output2(:, 2)';
    a2 = output2(:, 3)';
    b2 = output2(:, 4)';
    c2 = output2(:, 5)';
    desc2 = output2(:, 6:end)';
    
    % Image derivatives to detect foreground
%     sigma = 1.2.^(0:12);
%     
%     im1_gradientx = ImageDerivatives(im1, sigma(12), 'x');
%     im1_gradienty = ImageDerivatives(im1, sigma(12), 'y');
%     figure
%     imshow(im1_gradientx)
%     figure
%     imshow(im1_gradienty)
    
    % Matching
    [matches, scores] = vl_ubcmatch(desc1, desc2, 2.);
    
    % Get n best matches
    n = 20;
    best_matches = zeros(3, n);
    
    m = 1;
    while m <= n
        [best_matches(1,m), idx] = min(scores);
        
        scores(idx) = +Inf;
        
        if (x1(:,matches(1,idx)) > 750) && (x1(:,matches(1,idx)) < 1500) ...
           && (y1(:,matches(1,idx)) > 550) && (y1(:,matches(1,idx)) < 1100)
       
            best_matches(2:end,m) = matches(:,idx);
            m = m + 1;
        end
    end
    
    % Plot    
%     imtot = cat(2, im1,im2);
%     figure
%     imshow(imtot);
%     hold on;
    
%     scatter(x1(:,best_matches(2,:)), y1(:,best_matches(2,:)), 'y');
%     scatter(x2(:,best_matches(3,:)) + size(im1, 2), y2(:,best_matches(3,:)), 'y');
    
    % Create matrix A
    A = zeros(n, 9);
    
    for row = 1:size(A, 1)
        A(row,:) = [x1(:,best_matches(2,row))*x2(:,best_matches(3,row))
                    x1(:,best_matches(2,row))*y2(:,best_matches(3,row))
                    x1(:,best_matches(2,row))
                    y1(:,best_matches(2,row))*x2(:,best_matches(3,row))
                    y1(:,best_matches(2,row))*y2(:,best_matches(3,row))
                    y1(:,best_matches(2,row))
                    x2(:,best_matches(3,row))
                    y2(:,best_matches(3,row))
                    1];
    end
    
    [F, F_denorm, Fbest] = eightpoint(A);
    
end