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

load(fullfile(dataPath, dataFile));
load(fullfile(dataPath, BOWmodelFile));
load(fullfile(dataPath, cameraRtC2WFile));
