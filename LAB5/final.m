clc
clear all
close all
K=load('Exter_Param1.mat');
%K=load('Exter_Param.mat');
cameraParams=K.cameraParams1;


buildingDir = fullfile('./photo');
%buildingDir = fullfile('./fifteen_percent1');
%buildingDir = fullfile('./block_wall1');
buildingScene = imageDatastore(buildingDir);

montage(buildingScene.Files)


I = readimage(buildingScene,1);
I = undistortImage(I,cameraParams,'OutputView','valid');
I=imresize(I,0.5);

gray_need_to_process = rgb2gray(I);
[y,x,m] = harris(gray_need_to_process,4500,'tile',[2 2],'disp');
points=[x,y];
[features, points] = extractFeatures(gray_need_to_process, points);
numImages = numel(buildingScene.Files);
tforms(numImages) = projective2d(eye(3));

imageSize = zeros(numImages,2);

for n = 2:numImages
    

    pointsPrevious = points;
    featuresPrevious = features;
        

    I = readimage(buildingScene, n);
    I = undistortImage(I,cameraParams,'OutputView','valid');

    I=imresize(I,0.5);

    gray_need_to_process = rgb2gray(I);    
    imageSize(n,:) = size(gray_need_to_process);
    
    [y,x,m] = harris(gray_need_to_process,5000,'tile',[2 2],'disp');
    points=[x,y];
    [features, points] = extractFeatures(gray_need_to_process, points);
  
    indexPairs = matchFeatures(features, featuresPrevious,'Unique', true,'MatchThreshold',1);
       
    matchedPoints = points(indexPairs(:,1), :);
    matchedPointsPrev = pointsPrevious(indexPairs(:,2), :);        
    
    tforms(n) = estimateGeometricTransform(matchedPoints, matchedPointsPrev,...
        'projective', 'Confidence', 99.9, 'MaxNumTrials', 2000);
    
    tforms(n).T = tforms(n).T * tforms(n-1).T; 
end

for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);    
end
avgXLim = mean(xlim, 2);
[~, idx] = sort(avgXLim);
centerIdx = floor((numel(tforms)+1)/2);
centerImageIdx = idx(centerIdx);
Tinv = invert(tforms(centerImageIdx));
if true
for i = 1:numel(tforms)    
    tforms(i).T = tforms(i).T * Tinv.T;
end
end

for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);
end

maxImageSize = max(imageSize);
yMin = min([1; ylim(:)]);
yMax = max([maxImageSize(1); ylim(:)]);


xMin = min([1; xlim(:)]);
xMax = max([maxImageSize(2); xlim(:)]);

image_width  = round(xMax - xMin);
image_height = round(yMax - yMin);
panorama = zeros([image_height image_width 3], 'like', I);
blender = vision.AlphaBlender('Operation', 'Binary mask', 'MaskSource', 'Input port');  


limit_x = [xMin xMax];
limit_Y = [yMin yMax];
panoramaView = imref2d([image_height image_width], limit_x, limit_Y);


for i = 1:numImages   
    I = readimage(buildingScene, i);  
    I = undistortImage(I,cameraParams,'OutputView','valid');
    I=imresize(I,0.5);
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);                  
    mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);    
    panorama = step(blender, panorama, warpedImage, mask);
end
%for i = 1:numImages
    
%    I = readimage(buildingScene, i);   
   
    % Transform I into the panorama.
 %   warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
                  
    % Generate a binary mask.    
 %   mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);
    
    % Overlay the warpedImage onto the panorama.
 %   panorama = step(blender, panorama, warpedImage, mask);
%end



figure
imshow(panorama)