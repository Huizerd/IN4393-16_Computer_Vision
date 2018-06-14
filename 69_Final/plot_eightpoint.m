function [] = plot_eightpoint(inliers_1, inliers_2, F_ransac_denorm)
% PLOT_EIGHTPOINT Plots either the matches and epipolar lines for two
% images (only first pair for checking) or the matches and corresponding 
% lines between matches.
% 
% Inputs:
% - inliers_1: coordinates of inliers for first image
% - inliers_2: coordinates of inliers for second image
% - F_ransac_denorm: denormalised fundamental matrix with highest number of
%   inliers found using RANSAC
% 
% Outputs: None (shows figures)

%% Load images and coordinates
I1=(imread('model_castle/8ADT8586.JPG'));
I2=(imread('model_castle/8ADT8587.JPG'));

p1 = inliers_1;
p2 = inliers_2;

%% Plotting of matching features
figure;
p3 = p2;
p3(:, 1) = p2(:, 1) + 4064;

imshow([I1 I2]);
hold on
h1 = vl_plotframe(p1');
h2 = vl_plotframe(p1');
set(h1,'color','k','linewidth',3);
set(h2,'color','y','linewidth',2);

h1 = vl_plotframe(p3');
h2 = vl_plotframe(p3');
set(h1,'color','k','linewidth',3);
set(h2,'color','y','linewidth',2);
hold on;

for idx = 1:numel(p1(:, 1))
    X_lines = [p1(idx, 1), p3(idx, 1)];
    Y_lines = [p1(idx, 2), p3(idx, 2)];
    plot(X_lines,Y_lines)
end

%% Plotting of epipolar lines
figure;
subplot(121)
imshow(I1)
title('Inliers and Epipolar Lines in First Image RANSAC'); hold on;
plot(p1(:, 1), p1(:, 2), 'go');

lines1 = epipolarLine(F_ransac_denorm', p2); %Ax+By+C
epipoint1 = lineToBorderPoints(lines1, size(I1));
line(epipoint1(:, [1, 3])', epipoint1(:, [2, 4])');

subplot(122); 
imshow(I2)
title('Epipolar lines in second image RANSAC'); hold on; 
plot(p2(:, 1), p2(:, 2), 'go');

lines2 = epipolarLine(F_ransac_denorm, p1);
epipoint2 = lineToBorderPoints(lines2, size(I2));
line(epipoint2(:, [1,3])', epipoint2(:, [2,4])');
truesize;

end