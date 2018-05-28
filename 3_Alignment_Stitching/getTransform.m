function [xBest, tform, transformedImage1, transformedImage2] = getTransform(image1, image2, N, threshold)

%% Match Images using SIFT
[frames1, desc1] = vl_sift(single(image1));
[frames2, desc2] = vl_sift(single(image2));

matches = vl_ubcmatch(desc1, desc2);

%% RANSAC

inliersBest = 0;

for n = 1:N
    
    perm = randperm(length(matches));
    disp(perm)
    P    = 3;  % 3 matches needed for 6 unknowns
    disp(P)
    seed = perm(1:P);

    matchesUsed = matches(:, seed);

    % Build matrix A from P matches
    A = [[frames1(1,matchesUsed(1,1)) frames1(2,matchesUsed(1,1)) 0 0 1 0];
         [0 0 frames1(1,matchesUsed(1,1)) frames1(2,matchesUsed(1,1)) 0 1];
         [frames1(1,matchesUsed(1,2)) frames1(2,matchesUsed(1,2)) 0 0 1 0];
         [0 0 frames1(1,matchesUsed(1,2)) frames1(2,matchesUsed(1,2)) 0 1];
         [frames1(1,matchesUsed(1,3)) frames1(2,matchesUsed(1,3)) 0 0 1 0];
         [0 0 frames1(1,matchesUsed(1,3)) frames1(2,matchesUsed(1,3)) 0 1]];

    b = [frames2(1,matchesUsed(2,1)); 
         frames2(2,matchesUsed(2,1));
         frames2(1,matchesUsed(2,2));
         frames2(2,matchesUsed(2,2));
         frames2(1,matchesUsed(2,3));
         frames2(2,matchesUsed(2,3))];

    x = pinv(A) * b;

    frames2New = frames2;
    
    % Edit only matches
    for m = 1:length(matches)
        frames2New(1:2,matches(2,m)) = [[x(1) x(2)]; [x(3) x(4)]] * [frames1(1,matches(1,m)); frames1(2,matches(1,m))] + [x(5); x(6)];
    end

    inliers   = length(find(sqrt(sum((frames2New(1:2,:) - frames2(1:2,:)).^2)) < threshold));
    
    if inliers > inliersBest   
        inliersBest = inliers;
        xBest = x;  
    end
    
end

%% Transforming images
T = [xBest(1) xBest(2) xBest(5);
     xBest(3) xBest(4) xBest(6);
     0        0        1       ];

tform = affine2d(T');
transformedImage1 = imwarp(image1, tform, 'bicubic');

% Transform for image2 -> image1 is returned
tform = affine2d(inv(T)');
transformedImage2 = imwarp(image2, tform, 'bicubic');
