% Assignment 5: Epipolar Geometry
% Jesse Hagenaars & Michiel Mollema - 07-05-2018

images = ["TeddyBear/obj02_001.jpg", "TeddyBear/obj02_002.jpg"];
regions = ["TeddyBear/obj02_001.haraff", "TeddyBear/obj02_002.haraff"];

for i = 1:length(images)
    
    if ~exist(regions(i), 'file')
        edit(regions(i))
    end
    
    display_features(regions(i), images(i), 0, 0);
end