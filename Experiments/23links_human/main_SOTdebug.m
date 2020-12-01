%% Preliminaries
clc; close all; clear all;

rng(1); % Force the casual generator to be const
format long;

addpath(genpath('src'));
addpath(genpath('../../src'));
addpath(genpath('../../external'));
addpath(genpath('scripts'));
addpath(genpath('test_SOTdebug'));

opts.static = true;
if opts.static
    %% Static dataset
    % suit
    bucket.pathToWearableDataXsens = fullfile(pwd,('/test_SOTdebug/trial2-Npose_subj8/wearable/xsens'));
    % left ftShoe
    bucket.pathToWearableDataLeftFtShoe = fullfile(pwd,('/test_SOTdebug/trial2-Npose_subj8/wearable/FTshoes/left'));
    % right ftShoe
    bucket.pathToWearableDataRightFtShoe = fullfile(pwd,('/test_SOTdebug/trial2-Npose_subj8/wearable/FTshoes/right'));
    % processed
    bucket.pathToProcessedData = fullfile(pwd,('/test_SOTdebug/trial2-Npose_subj8/processed'));
else
    %% Walking dataset
    % suit
    bucket.pathToWearableDataXsens = fullfile(pwd,('/test_SOTdebug/trial2-walking_subj8/wearable/xsens'));
    % left ftShoe
    bucket.pathToWearableDataLeftFtShoe = fullfile(pwd,('/test_SOTdebug/trial2-walking_subj8/wearable/FTshoes/left'));
    % right ftShoe
    bucket.pathToWearableDataRightFtShoe = fullfile(pwd,('/test_SOTdebug/trial2-walking_subj8/wearable/FTshoes/right'));
    % processed
    bucket.pathToProcessedData = fullfile(pwd,('/test_SOTdebug/trial2-walking_subj8/processed'));
end
%% SUIT struct creation
if ~exist(fullfile(bucket.pathToProcessedData,'suit.mat'), 'file')
    disp('-------------------------------------------------------------------');
    disp('[Start] Suit extraction ...');
    % 1) ---extract data from suit as YARP-dumped IWear file
    extractWearableDataFromIWear_xsens;
    % left ftshoe
    opts.left = true;
    bucket.pathToWearableDataFtShoes = bucket.pathToWearableDataLeftFtShoe;
    extractWearableDataFromIWear_ftShoes;
    % right ftshoes
    opts.left = false;
    bucket.pathToWearableDataFtShoes = bucket.pathToWearableDataRightFtShoe;
    extractWearableDataFromIWear_ftShoes;
    
    % 2) ---compute sensor position w.r.t. the links
    disp('[Warning]: Check manually the length of the data for the sensor position computation!');
    disp('[Warning]: By default, the computation is done by considering all the samples. It may take time!');
    wearData.properties.lenData = wearData.nrOfFrames;
    suit = computeSuitSensorPosition(wearData);
    save(fullfile(bucket.pathToProcessedData,'suit.mat'),'suit');
    save(fullfile(bucket.pathToProcessedData,'wearData.mat'),'wearData');
    
    disp('[End] Suit extraction');
else
    load(fullfile(bucket.pathToProcessedData,'suit.mat'));
    load(fullfile(bucket.pathToProcessedData,'wearData.mat'));
end

%% Length equilizer (tappullo for the moment)
% Shoes
if length(wearData.ftShoes.Right) ~= length(wearData.ftShoes.Left)
    if length(wearData.ftShoes.Right) < length(wearData.ftShoes.Left)
        newRange = 1:length(wearData.ftShoes.Right);
        wearData.ftShoes.Left = wearData.ftShoes.Left(:,newRange);
    else
        newRange = 1:length(wearData.ftShoes.Left);
        wearData.ftShoes.Right = wearData.ftShoes.Right(:,newRange);
    end
end
% Equalizing (with a tappullo) the shoes and the suit
if length(newRange) ~= suit.nrOfFrames
    if length(newRange) < suit.nrOfFrames
        finalRange = 1:length(newRange);
        % TODO: cut all the suit variables
        % for the moment, let's compute only position and orientation
        for suitLinksIdx = 1 : size(suit.links,1)
            suit.links{suitLinksIdx, 1}.meas.position    = suit.links{suitLinksIdx, 1}.meas.position(:,finalRange);
            suit.links{suitLinksIdx, 1}.meas.orientation = suit.links{suitLinksIdx, 1}.meas.orientation(:,finalRange);
            
        end
    else
        finalRange = 1:suit.nrOfFrames;
        wearData.ftShoes.Right = wearData.ftShoes.Right(:,finalRange);
        wearData.ftShoes.Left  = wearData.ftShoes.Left(:,finalRange);
    end
end

%% Plot input feet wrenches
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');

title('wrench measurements coming from FTshoes');
ylabel_wrenches = {'$f_x$ [N]', '$f_y$ [N]','$f_z$ [N]' ...
    '$m_x$ [Nm]', '$m_y$ [Nm]','$m_z$ [Nm]'};
for picIdx = 1 : 6
    subplot (2,3,picIdx)
    plot1 = plot(wearData.ftShoes.Left(picIdx,:));
    axis tight;
    ax = gca;
    ax.FontSize = 20;
    hold on
    plot2 = plot(wearData.ftShoes.Right(picIdx,:));
    hold on
    ylabel(ylabel_wrenches{picIdx},'HorizontalAlignment','center',...
        'FontSize',40,'interpreter','latex');
    xlabel('samples','FontSize',25);
    leg = legend([plot1, plot2],{'Left foot', 'Right foot'});
end

%% Subject data
mass = 53.5; %kg
height_heelFoot = 0.08; %m

%% Suit base orientation and position w.r.t. G
currentBase = 'Pelvis';
for suitLinksIdx = 1 : size(suit.links,1)
    if strcmp(suit.links{suitLinksIdx, 1}.label, currentBase)
        basePos_wrtG  = suit.links{suitLinksIdx, 1}.meas.position;
        orientation.baseOrientation = suit.links{suitLinksIdx, 1}.meas.orientation;
    end
end

%% Feet orientation w.r.t. G
bucket.contactLink = cell(2,1);

% Define contacts configuration
bucket.contactLink{1} = 'RightFoot'; % human link in contact with RightShoe
bucket.contactLink{2} = 'LeftFoot';  % human link in contact with LeftShoe
% -----LEFT
for suitLinksIdx = 1 : size(suit.links,1)
    if strcmp(suit.links{suitLinksIdx, 1}.label, bucket.contactLink{2})
        orientation.leftFootOrientation = suit.links{suitLinksIdx, 1}.meas.orientation;
        break
    end
end
% -----RIGHT
for suitLinksIdx = 1 : size(suit.links,1)
    if strcmp(suit.links{suitLinksIdx, 1}.label, bucket.contactLink{1})
        orientation.rightFootOrientation = suit.links{suitLinksIdx, 1}.meas.orientation;
        break
    end
end

%% Transform shoes wrenches
shoes     = transformShoesWrenches_iWear(orientation, wearData, height_heelFoot, mass);
% shoes_pre = transformShoesWrenches_pre_iWear(orientation, wearData, height_heelFoot, mass);
% save(fullfile(bucket.pathToProcessedData,'shoes_iWear.mat'),'shoes');
% save(fullfile(bucket.pathToProcessedData,'shoesPre_iWear.mat'),'shoes_pre');

%% Plot beta angles
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');

title('time evolution of \beta');
plot_betaLF = plot(shoes.betaInDeg_LF ,'lineWidth',2);
ylabel('\beta_{LF} [deg]','FontSize',20);
hold on;
plot_betaRF = plot(shoes.betaInDeg_RF ,'g','lineWidth',2);
ylabel('\beta  [deg]','FontSize',20);
xlabel('sample','FontSize',20);
leg = legend([plot_betaLF, plot_betaRF],{'\beta_{left}', '\beta_{right}'});

%% Plot equation terms comparison
ylabel_wrenches = {'$f_x$ [N]', '$f_y$ [N]','$f_z$ [N]'};
fig = figure();
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
for axisIdx = 1 : 3
    subplot (3,1,axisIdx)
    % term_1
    plot_term1 = plot(shoes.term1(axisIdx,:),'lineWidth',2);
    hold on;
    % term 2
    plot_term2 = plot(shoes.term2(axisIdx,:),'lineWidth',2);
    ylabel(ylabel_wrenches(axisIdx),'HorizontalAlignment','center',...
        'FontSize',40,'interpreter','latex');
    % legend
    leg = legend([plot_term1, plot_term2], ...
        {'${}^{\mathcal{B}}R_G \sum {}^{\mathcal{B}}f^x$','${}^{\mathcal{B}}R_G mg$'});
    set(leg,'Interpreter','latex','Location','northeast');
    set(leg,'FontSize',25);
end
%%
disp('[End]');
