function [SSE] = benchmarkLK(points, pointsLK, frames)

SE  = (points - pointsLK).^2;
SSE = sum(SE, 2);
MSE = mean(SE, 2);

plotFrames = subplot(2, 2, 1);
plotSSE    = subplot(2, 2, 2);
plotMSE    = subplot(2, 2, 4);

framesPos = get(plotFrames, 'Position');
centerPos = mean([get(plotSSE, 'Position'); get(plotMSE, 'Position')], 1);
set(plotFrames, 'Position', [framesPos(1), centerPos(2), framesPos(3), framesPos(4)])

lineSSE = animatedline(plotSSE);
lineMSE = animatedline(plotMSE);

axis(plotSSE, [0 100 0 500])
axis(plotMSE, [0 100 0 10])

% Counter for point data
k = 1;

for f = 1:size(frames, 3)
    
    addpoints(lineSSE, f, SSE(f))
    addpoints(lineMSE, f, MSE(f))
    
    imshow(uint8(frames(:, :, f)), 'Parent', plotFrames)
    truesize([320 320])
    
    hold(plotFrames, 'on')
    scatter(plotFrames, points(k, :), points(k+1, :), 'g')
    scatter(plotFrames, pointsLK(k, :), points(k+1, :), 'r')
    plot(plotFrames, [points(k, :); pointsLK(k, :)], [points(k+1, :); pointsLK(k+1, :)], 'y')
    hold(plotFrames, 'off')
    
    drawnow

    k = k + 2; 
end