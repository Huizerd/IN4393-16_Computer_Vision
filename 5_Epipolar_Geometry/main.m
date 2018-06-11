% Assignment 5: Epipolar Geometry
% Jesse Hagenaars & Michiel Mollema - 07-05-2018

clear; clc
threshold = 1e-5;

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
    [matches, scores] = vl_ubcmatch(desc1, desc2, 5.);
    
    % Get n best matches
    n = 200;
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
    
% %     Plot    
%     imtot = cat(2, im1,im2);
%     figure
%     imshow(imtot);
%     hold on;
%     
%     scatter(x1(:,best_matches(2,:)), y1(:,best_matches(2,:)), 'y');
%     scatter(x2(:,best_matches(3,:)) + size(im1, 2), y2(:,best_matches(3,:)), 'y');
    

    
    [F_ransac, F_ransac_denorm, inliers] = eightpoint(x1, y1, x2, y2, best_matches, threshold);
    
end

% plotting, plot the F_ransac

I1=imread('TeddyBearPNG/obj02_001.png');
I2=imread('TeddyBearPNG/obj02_002.png');

p1 = [x1(matches(1, :)); y1(matches(1, :))]';
p2 = [x2(matches(2, :)); y2(matches(2, :))]';
% F_final = F_ransac;
F_final = F_ransac_denorm;
% F_final = [-4.6553926e-08 -4.716306e-07 0.00026527;...
%             2.46148e-07 -3.853155e-08 0.00407577;...
%             -0.00048518 -0.00379291 0.336424]

figure;
subplot(121)
imshow(I1)
title('Inliers and Epipolar Lines in First Image RANSAC'); hold on;
plot(p1(:,1),p2(:,2),'go');

lines1=epipolarLine(F_final',p2); %Ax+By+C
epipoint1=lineToBorderPoints(lines1,size(I1));
line(epipoint1(:,[1,3])',epipoint1(:,[2,4])');

subplot(122); 
imshow(I2)
title('Epipolar lines in second image RANSAC'); hold on; 
plot(p2(:,1),p2(:,2),'go');  

lines2=epipolarLine(F_final,p1);
epipoint2=lineToBorderPoints(lines2,size(I2));
line(epipoint2(:,[1,3])',epipoint2(:,[2,4])');
truesize;