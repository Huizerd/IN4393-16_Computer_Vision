I1 = images_gray(:, :, 1);
I2 = images_gray(:, :, 2);

points1 = detectMinEigenFeatures(I1);
points2 = detectMinEigenFeatures(I2);

[features1, valid_points1] = extractFeatures(I1,points1);
[features2, valid_points2] = extractFeatures(I2,points2);

indexPairs = matchFeatures(features1,features2);

matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

figure; showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2);