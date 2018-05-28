% the Lukas Kanade Tracker:
% the initial points in the first frams are tracked. In the video
% 'tracked.avi' this is shown, where yellow dots are the ground truth and
% pink dots are the tracked points
%%%You can also not follow this instrcution and implement the tracker
%%%according to your own interpretation!
function [pointsx, pointsy] = LKtracker(p,im,sigma)
[H,W,frames] = size(im);
patchSize = 15;
patchStep = floor(patchSize/2);

%pre-alocate point locations and image derivatives
pointsx = zeros(size(im,3),size(p,2));
pointsy = zeros(size(im,3),size(p,2));
pointsx(1,:) = p(1,:);
pointsy(1,:) = p(2,:);
%fill in starting points

It=zeros(size(im) - [0 0 1]);
Ix=zeros(size(im) - [0 0 1]);
Iy=zeros(size(im) - [0 0 1]);

%calculate the gaussian derivative
% G = 
% Gd = 
% 
%find x,y and t derivative
for i=1:size(im,3)-1
    Ix(:,:,i)= ImageDerivatives(im(:,:,i), sigma, 'x');
    Iy(:,:,i)= ImageDerivatives(im(:,:,i), sigma, 'y');
    It(:,:,i)= im(:,:,i+1) - im(:,:,i);
end

writerObj = VideoWriter('test.avi');
open(writerObj);

for num = 1:size(im,3)-1 % iterating through images
    for i = 1:size(p,2) % iterating throught points
        % make a matrix consisting of derivatives around the pixel location
        x = int16(pointsx(num, i));      %%%center of the patch
        y = int16(pointsy(num, i));      %%%center of the patch

        A1 = Ix(max(1,y-patchStep):min(H,y+patchStep),...
                max(1,x-patchStep):min(W,x+patchStep),...
                num);
        A2 = Iy(max(1,y-patchStep):min(H,y+patchStep),...
                max(1,x-patchStep):min(W,x+patchStep),...
                num);
        A = cat(2,...
               reshape(A1, [], 1),...
               reshape(A2, [], 1));
        % make b matrix consisting of derivatives in time
        b = -reshape(It(max(1,y-patchStep):min(H,y+patchStep),...
                max(1,x-patchStep):min(W,x+patchStep),...
                num),...
                [], 1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        v = pinv((A' * A)) * A' * b;
        pointsx(num+1,i) = pointsx(num,i) + v(1);
        pointsy(num+1,i) = pointsy(num,i) + v(2);
    end
%         figure(1)
%         imshow(im(:,:,num),[])
%         hold on
%         plot(pointsx(num,:),pointsy(num,:),'.y') %tracked points
%         plot(p(num*2-1,:),p(num*2,:),'.m')  %ground truth
%         frame = getframe;
%         writeVideo(writerObj,frame);
end
close(writerObj);


end
