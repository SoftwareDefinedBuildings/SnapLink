% To match a new 2D image without depth information to existing 
% 3D model

% Author: Kaifei Chen <kaifei@cs.berkeley.edu>

% Copied from basicSetup.m
addpath(genpath('lib/awf'));
addpath(genpath('lib/nyu'));
addpath(genpath('lib/visualindex'));
addpath(genpath('lib/estimateRigidTransform'));
addpath(genpath('lib/peter'));
addpath(genpath('lib/kdtree'));
addpath(genpath('lib/sift'));
addpath(genpath('lib/arcball'));
run('lib/vlfeat/toolbox/vl_setup');


dataPath = '/home/kaifei/Data/sun3d/sfm/hotel_umd/maryland_hotel3/';
dataFile = 'data.mat';
BOWmodelFile = 'BOWmodel.mat';
cameraRtC2WFile = 'cameraRtC2W.mat';
plyFile = 'BA.ply';
newImageFile = '0000530-000017729964.jpg';

load(fullfile(dataPath, dataFile));
load(fullfile(dataPath, BOWmodelFile));
load(fullfile(dataPath, cameraRtC2WFile));


%% Get SIFTs and BOW histogram of new images
newImage = imread(fullfile(dataPath, newImageFile));

[newSIFTloc, newSIFTdes] = visualindex_get_features([], newImage); % first parameter not used
newWords = visualindex_get_words(BOWmodel, newSIFTdes);
newHistogram = sparse(double(newWords), 1, ones(length(newWords),1), BOWmodel.vocab.size, 1);

%% find most similar images
matchScores = newHistogram' * BOWmodel.index.histograms;
[maxScore, maxScoreIndex] = max(matchScores);

%% Match SIFTs
% We calculate the SIFTs again because this is what the SUN3DSfM does
[newSIFTloc, newSIFTdes] = vl_sift(single(rgb2gray(newImage)));
newSIFTloc = newSIFTloc([2,1], :);

[matchPointsID_new, matchPointsID_old] = matchSIFTdesImagesBidirectional(newSIFTdes, BOWmodel.index.descrs{maxScoreIndex});
newSIFTloc = newSIFTloc(:, matchPointsID_new);
oldSIFTloc = BOWmodel.index.frames{maxScoreIndex};
oldSIFTloc = oldSIFTloc([2,1], :);
oldSIFTloc = oldSIFTloc(:, matchPointsID_old);
%oldSIFTloc = double(oldSIFTloc);

% Get 3D locations of SIFT points in the old image
oldXYZcam = depth2XYZcamera(data.K, depthRead(data.depth{maxScoreIndex}));
oldXYZcamX = oldXYZcam(:, :, 1);
oldXYZcamY = oldXYZcam(:, :, 2);
oldXYZcamZ = oldXYZcam(:, :, 3);
ind = sub2ind([size(oldXYZcamX, 1) size(oldXYZcamX, 2)], round(oldSIFTloc(1,:)), round(oldSIFTloc(2,:)));
oldSIFTloc3D = [oldXYZcamX(ind); oldXYZcamY(ind); oldXYZcamZ(ind)];

% Prepare data to fit MATLAB input
newSIFTloc = newSIFTloc([2,1], :)'; % 2D coord needs to be [y,x]
oldSIFTloc3D = oldSIFTloc3D';
oldSIFTloc3D = double(oldSIFTloc3D);
newK = reshape(readValuesFromTxt(fullfile(dataPath, 'intrinsics.txt')), 3, 3)';
newCameraParams = cameraParameters('IntrinsicMatrix', newK'); % MATLAB uses the transpose of K

%% Calculate R, t of new image from camera to the world
[R, t] = extrinsics(newSIFTloc, oldSIFTloc3D, newCameraParams);
R = R'; % Because MATLAB use transpose of R, [x y z] = [X Y Z]*R+t
newRC2W = cameraRtC2W(:,1:3,maxScoreIndex) * inv(R);
newtC2W = - cameraRtC2W(:,1:3,maxScoreIndex) * inv(R) * t' + cameraRtC2W(:,4,maxScoreIndex);

%% Project 3D model to the new image
% http://www.mathworks.com/help/vision/ref/cameramatrix.html





