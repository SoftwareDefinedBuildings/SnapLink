%addpath(genpath('lib'));
%addpath(genpath('IO'));

addpath(genpath('lib/awf'));			
addpath(genpath('lib/nyu'));			
addpath(genpath('lib/visualindex'));
addpath(genpath('lib/estimateRigidTransform'));	
addpath(genpath('lib/peter'));
addpath(genpath('lib/kdtree'));			
addpath(genpath('lib/sift'));
addpath(genpath('lib/arcball'));


warning('off', 'images:imshow:magnificationMustBeFitForDockedFigure');

%fprintf('test\n');
dbstop if error
%clc;

run('lib/vlfeat/toolbox/vl_setup');

poolobj = gcp('nocreate');
if isempty(poolobj)
    try
        parpool;
    catch
    end
end