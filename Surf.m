
clear all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reading the Images from the Folder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

imPath = 'kebel'; imExt = 'jpg';

filearray = dir([imPath filesep '*.' imExt]); % get all files in the directory
NumImages = size(filearray,1); % get the number of images


for k = 1:NumImages
imgname = [imPath filesep filearray(k).name]; % get image name
I(:,:,:,k) = im2double(imread(imgname));
im(:,:,k) = rgb2gray(I(:,:,:,k));
end

imargb = I(:,:,:,1);
ima = im(:,:,1);


 for i=2:NumImages
    
imbrgb = I(:,:,:,i);
imb = im(:,:,i);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Detecting the Feature points using SURF 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

points1 = detectSURFFeatures(ima);
points2 = detectSURFFeatures(imb);


%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extracting the Features
%%%%%%%%%%%%%%%%%%%%%%%%%%%
[features1, valid_points1] = extractFeatures(ima, points1);
[features2, valid_points2] = extractFeatures(imb, points2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matching the Features
%%%%%%%%%%%%%%%%%%%%%%%%%%%
INDEX_PAIRS = matchFeatures(features1, features2);
         
matched_pts1 = valid_points1(INDEX_PAIRS(:, 1));
matched_pts2 = valid_points2(INDEX_PAIRS(:, 2));



xat = matched_pts1.Location(:,1);
 yat = matched_pts1.Location(:,2);
 
xbt = matched_pts2.Location(:,1);
 ybt = matched_pts2.Location(:,2);
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Estimate homography between images.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t              = 0.01; 
[Hab, inliers] = ransacfithomography([xbt'; ybt'],[xat'; yat'], t);
 
 bbox=[-50 600*i -200 1000];  % The Size of the Display Box
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Final warping and compositing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 iwa = vgg_warp_H(ima, eye(3), 'linear', bbox); iwa(iwa>1)=0;
 
 iwb = vgg_warp_H(imb, Hab, 'linear', bbox); iwb(iwb>1)=0; % warp image a to the mosaic image
ima = (double(max(iwa,iwb))); % combine images into a common mosaic (take maximum value of the two images)
 
 end
 figure; clf;
imshow(ima); axis image;
