function [] = plot_eightpoint(inliers_1, inliers_2, F_ransac_denorm)
% plotting, plot the F_ransac
I1=(imread('model_castle/8ADT8586.JPG'));
I2=(imread('model_castle/8ADT8587.JPG'));

p1 = inliers_1';
p2 = inliers_2';

p2(1, :) = p2(1, :) + 4064;

imshow([I1 I2]);
hold on

% F_final = F_ransac_denorm;
% 
% % figure;
% % subplot(121)
% % imshow(I1)
% % title('Inliers and Epipolar Lines in First Image RANSAC'); hold on;
% % plot(p1(:,1),p1(:,2),'go');
% 
% % % lines1=epipolarLine(F_final',p2); %Ax+By+C
% % % epipoint1=lineToBorderPoints(lines1,size(I1));
% % % line(epipoint1(:,[1,3])',epipoint1(:,[2,4])');
% 
% % subplot(122); 
% % imshow(I2)
% % title('Epipolar lines in second image RANSAC'); hold on; 
% % plot(p2(:,1),p2(:,2),'go');
% 
h1 = vl_plotframe(p1);
h2 = vl_plotframe(p1);
set(h1,'color','k','linewidth',3);
set(h2,'color','y','linewidth',2);

h1 = vl_plotframe(p2);
h2 = vl_plotframe(p2);
set(h1,'color','k','linewidth',3);
set(h2,'color','y','linewidth',2);
hold on;
for idx = 1:numel(p1(1, :))
    X_lines = [p1(1, idx),p2(1, idx)];
    Y_lines = [p1(2, idx),p2(2, idx)];
    plot(X_lines,Y_lines)
end

% % lines2=epipolarLine(F_final,p1);
% % epipoint2=lineToBorderPoints(lines2,size(I2));
% % line(epipoint2(:,[1,3])',epipoint2(:,[2,4])');
% truesize;


end