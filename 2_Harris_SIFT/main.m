% Assignment 2: Harris Corner Detector
% Jesse Hagenaars & Michiel Mollema - 05-03-2018

sigma = 1.2.^(0:12);

ima = imread('landscape-a.jpg');
imb = imread('landscape-b.jpg');

ima_grey = uint8(mean(imread('landscape-a.jpg'), 3));
imb_grey = uint8(mean(imread('landscape-b.jpg'), 3));

results1 = getSIFT(ima_grey, sigma);
results2 = getSIFT(imb_grey, sigma);

%compute the matching scores with a treshold value
[matches, scores] = vl_ubcmatch(results1{1,6}, results2{1,6}, 2.);
imtot = cat(2, ima,imb);
imshow(imtot);

%plot the sift features of image a
fa1 = results1{1,5}(:,matches(1,:));
% perm = randperm(size(fa1,2));
h1 = vl_plotframe(fa1);
h2 = vl_plotframe(fa1);
set(h1,'color','k','linewidth',3);
set(h2,'color','y','linewidth',2);

%plot the sift features of image b
fb1 = results2{1,5}(:,matches(2,:));
% perm = randperm(size(fb1,2));
fb1(1,:) = fb1(1,:)+720;
h1 = vl_plotframe(fb1);
h2 = vl_plotframe(fb1);
set(h1,'color','k','linewidth',3);
set(h2,'color','y','linewidth',2);
hold on;
for idx = 1:numel(fa1(1,:))
    X_lines = [fa1(1,idx),fb1(1,idx)];
    Y_lines = [fa1(2,idx),fb1(2,idx)];
    plot(X_lines,Y_lines)
end