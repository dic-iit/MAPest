
% Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
% All rights reserved.
%
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.

%% Preliminaries
close all;

% NE color
orangeAnDycolor = [0.952941176470588   0.592156862745098   0.172549019607843];
% WE color
greenAnDycolor  = [0.282352941176471   0.486274509803922   0.427450980392157];

subjectID = 21;
mass = 90; % in kg
height = 1.80; % in m
% bucket.datasetRoot          = fullfile(pwd, 'dataLBP_SPEXOR');
% bucket.pathToSubject        = fullfile(bucket.datasetRoot, sprintf('S%03d',subjectID));
% bucket.pathToSubjectRawData = fullfile(bucket.pathToSubject,'data');

% New plot label for motion tasks
motionTasksPlotLabel     = {'squat'};
motionTasksPlotLabel_exo = {'squat_EXO'};

%% Extraction data squat
% NE
bucket.pathToTask   = fullfile(bucket.pathToSubjectRawData,'squat');
bucket.pathToProcessedData   = fullfile(bucket.pathToTask,'processed');
estimatedVariables_squat     = load(fullfile(bucket.pathToProcessedData,'processed_SOTtask2/estimatedVariables.mat'));
synchrokin_squat             = load(fullfile(bucket.pathToProcessedData,'synchrokin.mat'));

% WE
bucket.pathToTask   = fullfile(bucket.pathToSubjectRawData,'squat_EXO');
bucket.pathToProcessedData  = fullfile(bucket.pathToTask,'processed');
estimatedVariables_squatEXO = load(fullfile(bucket.pathToProcessedData,'processed_SOTtask2/estimatedVariables.mat'));
synchrokin_squatEXO         = load(fullfile(bucket.pathToProcessedData,'synchrokin.mat'));

selectedJoints           = load(fullfile(bucket.pathToProcessedData,'selectedJoints.mat'));

%% ========================================================================
%% =============================== L5S1 ===================================
%% ========================================================================
fig = figure('Name', 'intra-subject L5S1','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

%% ----------- rotx
for jIdx = 1 : length(selectedJoints.selectedJoints)
    if strcmp(selectedJoints.selectedJoints{jIdx}, 'jL5S1_rotx')
        jointIndex = jIdx;
    end
end
subplot (2,2,1) %-------------squat
% NE
plot1 = plot((estimatedVariables_squat.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot2 = plot((estimatedVariables_squatEXO.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',greenAnDycolor,'lineWidth',4);
hold on
title(sprintf('Task  < %s >', 'squat'),'FontSize',22);
ylabel('$\tau^{L5S1}_{x}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot1,plot2],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

subplot (2,2,3) %-------------squat kinematics
% NE
plot3 = plot((synchrokin_squat.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot4 = plot((synchrokin_squatEXO.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',greenAnDycolor,'lineWidth',4);
xlabel('samples','FontSize',25);
ylabel('$q^{L5S1}_{x}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot3,plot4],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

% subplot (3,1,3) %-------------exo forces w.r.t. T8 link
% plot5 = plot(EXOfext.T8(1,:),'k','lineWidth',2);
% axis tight;
% ax = gca;
% ax.FontSize = 15;
% hold on
% xlabel('samples','FontSize',25);
% ylabel('$f^{EXO,T8}_x$','HorizontalAlignment','center',...
%     'FontSize',40,'interpreter','latex');
grid on;

%% ----------- roty
for jIdx = 1 : length(selectedJoints.selectedJoints)
    if strcmp(selectedJoints.selectedJoints{jIdx}, 'jL5S1_roty')
        jointIndex = jIdx;
    end
end
subplot (2,2,2) %-------------squat
% NE
plot1 = plot((estimatedVariables_squat.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot2 = plot((estimatedVariables_squatEXO.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',greenAnDycolor,'lineWidth',4);
hold on
title(sprintf('Task  < %s >', 'squat'),'FontSize',22);
ylabel('$\tau^{L5S1}_{y}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot1,plot2],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

subplot (2,2,4) %-------------squat kinematics
% NE
plot3 = plot((synchrokin_squat.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot4 = plot((synchrokin_squatEXO.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',greenAnDycolor,'lineWidth',4);
xlabel('samples','FontSize',25);
ylabel('$q^{L5S1}_{y}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot3, plot4],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);
% 
% subplot (3,1,3) %-------------exo forces w.r.t. T8 link
% plot5 = plot(EXOfext.T8(2,:),'k','lineWidth',2);
% axis tight;
% ax = gca;
% ax.FontSize = 15;
% hold on
% xlabel('samples','FontSize',25);
% ylabel('$f^{EXO,T8}_{y}$','HorizontalAlignment','center',...
%     'FontSize',40,'interpreter','latex');
grid on;

%% ========================================================================
%% =============================== T9T8 ===================================
%% ========================================================================
fig = figure('Name', 'intra-subject T9T8','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

%% ----------- rotx
for jIdx = 1 : length(selectedJoints.selectedJoints)
    if strcmp(selectedJoints.selectedJoints{jIdx}, 'jT9T8_rotx')
        jointIndex = jIdx;
    end
end
subplot (2,3,1) %-------------squat
% NE
plot1 = plot((estimatedVariables_squat.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot2 = plot((estimatedVariables_squatEXO.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',greenAnDycolor,'lineWidth',4);
hold on
title(sprintf('Task  < %s >', 'squat'),'FontSize',22);
ylabel('$\tau^{T9T8}_{x}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot1,plot2],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

subplot (2,3,4) %-------------squat kinematics
% NE
plot3 = plot((synchrokin_squat.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot4 = plot((synchrokin_squatEXO.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',greenAnDycolor,'lineWidth',4);
xlabel('samples','FontSize',25);
ylabel('$q^{T9T8}_{x}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot3, plot4],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

%% ----------- roty
for jIdx = 1 : length(selectedJoints.selectedJoints)
    if strcmp(selectedJoints.selectedJoints{jIdx}, 'jT9T8_roty')
        jointIndex = jIdx;
    end
end

subplot (2,3,2) %-------------squat
% NE
plot1 = plot((estimatedVariables_squat.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot2 = plot((estimatedVariables_squatEXO.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',greenAnDycolor,'lineWidth',4);
hold on
title(sprintf('Task  < %s >', 'squat'),'FontSize',22);
ylabel('$\tau^{T9T8}_{y}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot1,plot2],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

subplot (2,3,5) %-------------squat kinematics
% NE
plot3 = plot((synchrokin_squat.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot4 = plot((synchrokin_squatEXO.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',greenAnDycolor,'lineWidth',4);
xlabel('samples','FontSize',25);
ylabel('$q^{T9T8}_{y}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot3, plot4],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

%% ----------- rotz
for jIdx = 1 : length(selectedJoints.selectedJoints)
    if strcmp(selectedJoints.selectedJoints{jIdx}, 'jT9T8_rotz')
        jointIndex = jIdx;
    end
end

subplot (2,3,3) %-------------squat
% NE
plot1 = plot((estimatedVariables_squat.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot2 = plot((estimatedVariables_squatEXO.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',greenAnDycolor,'lineWidth',4);
hold on
title(sprintf('Task  < %s >', 'squat'),'FontSize',22);
ylabel('$\tau^{T9T8}_{z}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot1,plot2],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

subplot (2,3,6) %-------------squat kinematics
% NE
plot3 = plot((synchrokin_squat.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot4 = plot((synchrokin_squatEXO.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',greenAnDycolor,'lineWidth',4);
xlabel('samples','FontSize',25);
ylabel('$q^{T9T8}_{z}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot3, plot4],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

%% ========================================================================
%% =========================== Right Knee =================================
%% ========================================================================
fig = figure('Name', 'intra-subject Right knee','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

%% ----------- roty
for jIdx = 1 : length(selectedJoints.selectedJoints)
    if strcmp(selectedJoints.selectedJoints{jIdx}, 'jRightKnee_roty')
        jointIndex = jIdx;
    end
end
subplot (2,2,1) %-------------squat
% NE
plot1 = plot((estimatedVariables_squat.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot2 = plot((estimatedVariables_squatEXO.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',greenAnDycolor,'lineWidth',4);
hold on
title(sprintf('Task  < %s >', 'squat'),'FontSize',22);
ylabel('$\tau^{rKnee}_{y}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot1,plot2],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

subplot (2,2,3) %-------------squat kinematics
% NE
plot3 = plot((synchrokin_squat.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot4 = plot((synchrokin_squatEXO.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',greenAnDycolor,'lineWidth',4);
xlabel('samples','FontSize',25);
ylabel('$q^{rKnee}_{y}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot3, plot4],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

% subplot (3,1,3) %-------------exo forces w.r.t. T8 link
% plot5 = plot(EXOfext.T8(1,:),'k','lineWidth',2);
% axis tight;
% ax = gca;
% ax.FontSize = 15;
% hold on
% xlabel('samples','FontSize',25);
% ylabel('$f^{EXO,T8}_x$','HorizontalAlignment','center',...
%     'FontSize',40,'interpreter','latex');
grid on;

%% ----------- rotz
for jIdx = 1 : length(selectedJoints.selectedJoints)
    if strcmp(selectedJoints.selectedJoints{jIdx}, 'jRightKnee_rotz')
        jointIndex = jIdx;
    end
end
subplot (2,2,2) %-------------squat
% NE
plot1 = plot((estimatedVariables_squat.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot2 = plot((estimatedVariables_squatEXO.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',greenAnDycolor,'lineWidth',4);
hold on
title(sprintf('Task  < %s >', 'squat'),'FontSize',22);
ylabel('$\tau^{rKnee}_{z}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot1,plot2],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

subplot (2,2,4) %-------------squat kinematics
% NE
plot3 = plot((synchrokin_squat.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot4 = plot((synchrokin_squatEXO.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',greenAnDycolor,'lineWidth',4);
xlabel('samples','FontSize',25);
ylabel('$q^{rKnee}_{z}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot3, plot4],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);
% 
% subplot (3,1,3) %-------------exo forces w.r.t. T8 link
% plot5 = plot(EXOfext.T8(2,:),'k','lineWidth',2);
% axis tight;
% ax = gca;
% ax.FontSize = 15;
% hold on
% xlabel('samples','FontSize',25);
% ylabel('$f^{EXO,T8}_{y}$','HorizontalAlignment','center',...
%     'FontSize',40,'interpreter','latex');
grid on;

%% ========================================================================
%% ============================ Right ankle ===============================
%% ========================================================================
fig = figure('Name', 'intra-subject Rigth Ankle','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

%% ----------- rotx
for jIdx = 1 : length(selectedJoints.selectedJoints)
    if strcmp(selectedJoints.selectedJoints{jIdx}, 'jRightAnkle_rotx')
        jointIndex = jIdx;
    end
end
subplot (2,3,1) %-------------squat
% NE
plot1 = plot((estimatedVariables_squat.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot2 = plot((estimatedVariables_squatEXO.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',greenAnDycolor,'lineWidth',4);
hold on
title(sprintf('Task  < %s >', 'squat'),'FontSize',22);
ylabel('$\tau^{rAnkle}_{x}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot1,plot2],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

subplot (2,3,4) %-------------squat kinematics
% NE
plot3 = plot((synchrokin_squat.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot4 = plot((synchrokin_squatEXO.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',greenAnDycolor,'lineWidth',4);
xlabel('samples','FontSize',25);
ylabel('$q^{T9T8}_{x}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot3, plot4],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

%% ----------- roty
for jIdx = 1 : length(selectedJoints.selectedJoints)
    if strcmp(selectedJoints.selectedJoints{jIdx}, 'jRightAnkle_roty')
        jointIndex = jIdx;
    end
end

subplot (2,3,2) %-------------squat
% NE
plot1 = plot((estimatedVariables_squat.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot2 = plot((estimatedVariables_squatEXO.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',greenAnDycolor,'lineWidth',4);
hold on
title(sprintf('Task  < %s >', 'squat'),'FontSize',22);
ylabel('$\tau^{rAnkle}_{y}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot1,plot2],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

subplot (2,3,5) %-------------squat kinematics
% NE
plot3 = plot((synchrokin_squat.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot4 = plot((synchrokin_squatEXO.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',greenAnDycolor,'lineWidth',4);
xlabel('samples','FontSize',25);
ylabel('$q^{rAnkle}_{y}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot3, plot4],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

%% ----------- rotz
for jIdx = 1 : length(selectedJoints.selectedJoints)
    if strcmp(selectedJoints.selectedJoints{jIdx}, 'jRightAnkle_rotz')
        jointIndex = jIdx;
    end
end

subplot (2,3,3) %-------------squat
% NE
plot1 = plot((estimatedVariables_squat.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot2 = plot((estimatedVariables_squatEXO.estimatedVariables.tau.values(jointIndex,:)), ...
    'color',greenAnDycolor,'lineWidth',4);
% hold on
title(sprintf('Task  < %s >', 'squat'),'FontSize',22);
ylabel('$\tau^{rAnkle}_{z}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot1,plot2],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);

subplot (2,3,6) %-------------squat kinematics
% NE
plot3 = plot((synchrokin_squat.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',orangeAnDycolor,'lineWidth',4);
axis tight;
ax = gca;
ax.FontSize = 15;
hold on
% WE
plot4 = plot((synchrokin_squatEXO.synchroKin.q(jointIndex,:)*180/pi), ...
    'color',greenAnDycolor,'lineWidth',4);
xlabel('samples','FontSize',25);
ylabel('$q^{rAnkle}_{z}$','HorizontalAlignment','center',...
    'FontSize',40,'interpreter','latex');
grid on;
%legend
leg = legend([plot3, plot4],{'NE', 'WE'},'Location','northeast');
set(leg,'Interpreter','latex','FontSize',25);