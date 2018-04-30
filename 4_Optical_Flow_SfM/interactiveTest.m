% Create random signals

time = 1:101;
signal_1 = rand(101, 1);
signal_2 = rand(101, 1);

% Code for index pair testing
x = randi([0 100], 100, 1);  %random demo data
y = randi([0 100], 100, 1);  
plot(x, y, '.');  %plot all points
distancethreshold = 5;
pairwisedist = hypot(x - x', y - y');  %or use pdist
[p1, p2] = find(tril(pairwisedist <= distancethreshold & pairwisedist > 0))
hold on;
indexpairs = [p1, p2]';
disp(indexpairs)
plot(x(indexpairs), y(indexpairs), '-r')
%

imageDir   = [pwd '\model_house'];
imageNames = dir(fullfile(imageDir, '*.jpg'));
imageNames = {imageNames.name}';
imageSize  = size(imread(fullfile(imageDir, imageNames{1})));

points = importdata([imageDir '\measurement_matrix.txt']);
frames = zeros([imageSize, length(imageNames)]);

pointsNew = points + 10;

for i = 1:length(imageNames)
   img = imread(fullfile(imageDir, imageNames{i}));
   frames(:, :, i) = img;
end

% For plotting correlation line..need all values first

plotFrames  = subplot(2, 2, 1);
plotSSE     = subplot(2, 2, 2);
plotMSE     = subplot(2, 2, 4);

framesPos = get(plotFrames, 'Position');
centerPos = mean([get(plotSSE, 'Position'); get(plotMSE, 'Position')], 1);
set(plotFrames, 'Position', [framesPos(1), centerPos(2), framesPos(3), framesPos(4)])

SSE = animatedline(plotSSE);
MSE = animatedline(plotMSE);

axis([plotSSE plotMSE], [-0.1 80 -1 1])

for start_point = 0:80
    sig1_cut = signal_1(start_point+1:start_point+1+19);
    sig2_cut = signal_2(start_point+1:start_point+1+19);
    corr_pre = corrcoef(sig1_cut,sig2_cut);
    corr_fin(start_point+1) = corr_pre(2,1);
end

x_plot = 0:80;
n = 1;
k = 1;
    
for start_point = 0:80  
    addpoints(SSE, x_plot(n), corr_fin(n))
    addpoints(MSE, x_plot(n), corr_fin(n))
%     image(plotFrames, frames(:, :, n))
    imshow(uint8(frames(:, :, n)), 'Parent', plotFrames)
    truesize([320 320])
    hold(plotFrames, 'on')
    scatter(plotFrames, points(k, :), points(k+1, :), 'r')
    scatter(plotFrames, pointsNew(k, :), pointsNew(k+1, :), 'g')
%     plot(plotFrames, points(k, indexpairs), points(k+1, indexpairs), 'r')
    plot(plotFrames, [points(k, :); pointsNew(k, :)], [points(k+1, :); pointsNew(k+1, :)], 'y')
    hold(plotFrames, 'off')
%     showMatchedFeatures(frames(:, :, n), frames(:, :, n), pointsNew(:, :, n), pointsNew(:, :, n), 'Parent', plotFrames)
    drawnow
    n = n+1;
    k = k+2;
%     pause(0.03)   
end