function varargout = Gui(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Gui_OpeningFcn, ...
                   'gui_OutputFcn',  @Gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Gui is made visible.
function Gui_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Gui_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;


% --- Executes on button press in sk1_push.
function sk1_push_Callback(hObject, eventdata, handles)

[filename,pathname]=uigetfile('*','open');

% whether you open an image.
if isequal(filename,0) 
    disp('User selected Cancel.') 
else
    disp(['User selected ', fullfile(pathname, filename), '.'])
end

full_file = fullfile(pathname,filename);

handles.I = imread(full_file);
guidata(hObject, handles);
axes(handles.axes1); 
imshow(handles.I, 'DisplayRange', []);


% --- Executes on button press in sk2_push.
function sk2_push_Callback(hObject, eventdata, handles)

imargb = double(imread('keble_a.jpg'))/255;
imbrgb = double(imread('keble_b.jpg'))/255;
imcrgb = double(imread('keble_c.jpg'))/255;
ima = rgb2gray(imargb);
imb = rgb2gray(imbrgb);
imc = rgb2gray(imcrgb);
topn = 200; % how many Harris corners points?
[xa,ya,strengtha] = harris(ima,topn);
[xb,yb,strengthb] = harris(imb,topn);
[xc,yc,strengthc] = harris(imc,topn);
[Da,xa,ya] = ext_desc(ima,xa,ya);
[Db,xb,yb] = ext_desc(imb,xb,yb);
[Dc,xc,yc] = ext_desc(imc,xc,yc);

D2        = dist2(Da',Db'); % compute pair-wise distances between descriptors
[Y,I]     = sort(D2,2);     % sort distances
rr        = Y(:,1)./Y(:,2); % compute D. Lowes' 1nn/2nn ratio test
inD12     = find(rr<.8);    % take only points with a 1nn/2nn ratio below 0.8
I         = I(inD12);       % select matched points
xat       = xa(inD12);
yat       = ya(inD12);
xbt       = xb(I);
ybt       = yb(I);

t              = 0.01; 
[Hab, inliers] = ransacfithomography([xat; yat], [xbt; ybt], t);

vgg_gui_H(ima,imb,Hab);

bbox=[-400 1150 -200 700];  
iwb = vgg_warp_H(imbrgb, eye(3), 'linear', bbox);

iwa = vgg_warp_H(imargb, Hab, 'linear', bbox);  % warp image a to the mosaic image

D3        = dist2(Db',Dc'); % compute pair-wise distances between descriptors
[Y1,I1]     = sort(D3,2);     % sort distances
rr1        = Y1(:,1)./Y1(:,2); % compute D. Lowes' 1nn/2nn ratio test
inD121     = find(rr1<.8);    % take only points with a 1nn/2nn ratio below 0.8
I1         = I1(inD121);       % select matched points
xbt       = xb(inD121);
ybt       = yb(inD121);
xct       = xc(I1);
yct       = yc(I1);

[Hcb, inliers] = ransacfithomography( [xct; yct], [xbt; ybt], t);

iwc = vgg_warp_H(imcrgb, Hcb, 'linear', bbox); 
axes(handles.axes2); 
imagesc(max(iwc,double(max(iwb,iwa)))); % combine images into a common mosaic
axis image;  axis off;


% --- Executes on button press in sk3_push.
function sk3_push_Callback(hObject, eventdata, handles)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reading the Images from the Folder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


folder_name = uigetdir('*','open');

imPath = folder_name; imExt = 'jpg';

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
axes(handles.axes2); 
imshow(ima); axis image;

% --- Executes on button press in sk4_push.
function sk4_push_Callback(hObject, eventdata, handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reading the Images from the Folder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

folder_name = uigetdir('*','open');

imPath = folder_name; imExt = 'jpg';

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
% Detecting the Feature points using MSER 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

points1 = detectMSERFeatures(ima);
points2 = detectMSERFeatures(imb);


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
axes(handles.axes2); 
imshow(ima); axis image;


% --- Executes on button press in sk6_push.
function sk6_push_Callback(hObject, eventdata, handles)

% Open an image.
[filename,pathname]=uigetfile('*','open');

% whether you open an image.
if isequal(filename,0) 
    disp('User selected Cancel.') 
else
    disp(['User selected ', fullfile(pathname, filename), '.'])
end

full_file = fullfile(pathname,filename);

featWinLen   = 9;            % Length of feature window
maxNumPoints = int32(75);    % Maximum number of points
sizePano     = [400 680];
origPano     = [5 60];
classToUse   = 'single';

hsrc = vision.VideoFileReader(full_file, 'ImageColorSpace', ...
    'RGB', 'PlayCount', 1);

hcsc = vision.ColorSpaceConverter('Conversion', 'RGB to intensity');

 hcornerdet = vision.CornerDetector( ...
    'Method', 'Local intensity comparison (Rosen & Drummond)', ...
    'IntensityThreshold', 0.1, 'MaximumCornerCount', maxNumPoints, ...
    'CornerThreshold', 0.001, 'NeighborhoodSize', [21 21]);

hestgeotform = vision.GeometricTransformEstimator;

hgeotrans = vision.GeometricTransformer( ...
    'OutputImagePositionSource', 'Property', 'ROIInputPort', true, ...
    'OutputImagePosition', [-origPano fliplr(sizePano)]);

halphablender = vision.AlphaBlender( ...
    'Operation', 'Binary mask', 'MaskSource', 'Input port');

hdrawmarkers = vision.MarkerInserter('Shape', 'Circle', ...
    'BorderColor', 'Custom', 'CustomBorderColor', [1 0 0]);

hVideo1 = vision.VideoPlayer('Name', 'Corners');
hVideo1.Position(1) = hVideo1.Position(1) - 350;

hVideo2 = vision.VideoPlayer('Name', 'Mosaic');
hVideo2.Position(1) = hVideo1.Position(1) + 400;
hVideo2.Position([3 4]) = [750 500];

points   = zeros([0 2], classToUse);
features = zeros([0 featWinLen^2], classToUse);

while ~isDone(hsrc)
    % Save the points and features computed from the previous image
    pointsPrev   = points;
    featuresPrev = features;

    % To speed up mosaicking, select and process every 5th image
    for i = 1:5
        rgb = step(hsrc);
        if isDone(hsrc)
            break;
        end
    end
    I = step(hcsc, rgb);
    roi = int32([2 2 size(I, 2)-2 size(I, 1)-2]);

    % Detect corners in the image
    cornerPoints = step(hcornerdet, I);
    cornerPoints = cast(cornerPoints, classToUse);

    % Extract the features for the corners
    [features, points] = extractFeatures(I, ...
        cornerPoints, 'BlockSize', featWinLen);

    % Match features computed from the current and the previous images
    indexPairs = matchFeatures(features, featuresPrev);

    % Check if there are enough corresponding points in the current and the
    % previous images
    isMatching = false;
    if size(indexPairs, 1) > 2
        matchedPoints     = points(indexPairs(:, 1), :);
        matchedPointsPrev = pointsPrev(indexPairs(:, 2), :);

        % Find corresponding points in the current and the previous images,
        % and compute a geometric transformation from the corresponding
        % points
        [tform, inlier] = step(hestgeotform, matchedPoints, matchedPointsPrev);

        if sum(inlier) >= 4
            % If there are at least 4 corresponding points, we declare the
            % current and the previous images matching
            isMatching = true;
        end
    end

    if isMatching
        % If the current image matches with the previous one, compute the
        % transformation for mapping the current image onto the mosaic
        % image
        xtform = xtform * [tform, [0 0 1]'];
    else
        % If the current image does not match the previous one, reset the
        % transformation and the mosaic image
        xtform = eye(3, classToUse);
        mosaic = zeros([sizePano,3], classToUse);
    end

    % Display the current image and the corner points
    cornerImage = step(hdrawmarkers, rgb, cornerPoints);
    step(hVideo1, cornerImage);
    
    % Warp the current image onto the mosaic image
    transformedImage = step(hgeotrans, rgb, xtform, roi);
    mosaic = step(halphablender, mosaic, transformedImage, ...
        transformedImage(:,:,1)>0);
    step(hVideo2, mosaic);

end

release(hsrc);
