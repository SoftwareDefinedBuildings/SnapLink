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
BOWmodel = 'BOWmodel.mat';
cameraRtFile = 'cameraRt_RANSAC.mat';
plyFile = 'BA.ply';
newImageFile = '0000530-000017729964.jpg';

load(fullfile(dataPath, BOWmodel));
load(fullfile(dataPath, cameraRtFile));


%% Get SIFTs and BOW histogram of new images
newImage = imread(fullfile(dataPath, newImageFile));

[SIFTloc, SIFTdes] = vl_sift(single(rgb2gray(newImage)));
SIFTloc = SIFTloc([2,1], :);

[frames, descrs] = visualindex_get_features([], newImage); % first parameter not used
words = visualindex_get_words(BOWmodel, descrs);
histogram = sparse(double(words), 1, ones(length(words),1), BOWmodel.vocab.size, 1);

%% find most similar images
matchScores = histogram' * BOWmodel.index.histograms;


%% Match SIFTs
% matchSIFTdesImagesBidirectional



%% Eight Point Algorithm to Get Fundamental Matrix



%% Get Rt of new iamge in the New world


%% Project 3D model to the new image
% http://www.mathworks.com/help/vision/ref/cameramatrix.html
