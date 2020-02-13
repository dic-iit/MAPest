
% -----------------------------------------------------------------------%
%  VARIABLES ESTIMATION --> ANALYSIS & PLOTS
% -----------------------------------------------------------------------%
close all;

%% WHY THIS ANALYSIS
% If MAP is able to estimate properly those variables that are also
% measured (acc, fext and ddq) --> then we can suppose that the estimation
% of fint, fnet and tau are reasonable, as well.

% variables in y simulated from d
load(fullfile(bucket.pathToProcessedData,'y_sim.mat'));

%% Set the data range indexes
startIndex = 1;
endIndex = size(y_sim,2);


%% task05
% startIndex = 610;
% endIndex = 780;

len = size(estimation.mu_dgiveny(:,startIndex:endIndex),2);
specific_vector_sigma = zeros(1,len);

% Plot folder
bucket.pathToPlots = fullfile(bucket.pathToTask,'plots');
if ~exist(bucket.pathToPlots)
    mkdir (bucket.pathToPlots)
end
saveON = true;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  Rate of Change of Linear Momentum
% -----------------------------------------------------------------------%

fig = figure('Name', 'Rate of Change of Linear Momentum Vs Hands Force Estimate','NumberTitle','off','units','normalized','outerposition',[0 0 1 1]);
axes1 = axes('Parent',fig,'FontSize',16,'Units', 'normalized');
box(axes1,'on');
hold(axes1,'on');
grid on;

subplot (3,3,1) % dotL_x
plot1 = plot(y1_sim(295,:),'b','lineWidth',1.5);
hold on
title('Rate of Change of Linear Momentum','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
ylabel ('$\dot{L}_x$','FontSize',18,'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (3,3,4) % dotL_y
plot1 = plot(y1_sim(296,:),'b','lineWidth',1.5);
ylabel ('$\dot{L}_y$','FontSize',18,'Interpreter','latex');
hold on
grid on;
axis tight;
xlim([0 len])

subplot (3,3,7) % dotL_z
plot1 = plot(y1_sim(297,:),'b','lineWidth',1.5);
xlabel('samples','FontSize',18);
ylabel ('$\dot{L}_z$','FontSize',18,'Interpreter','latex');
hold on;
xlabel('z');
grid on;
axis tight;
xlim([0 len])


% Right hand measurements
range_fextMEAS_rightHand = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'RightHand', true);
fext.measured.rightHand = y_sim((range_fextMEAS_rightHand:range_fextMEAS_rightHand+5),startIndex:endIndex);
fext.measured.rightHand_sigma = diag(Sigmay((range_fextMEAS_rightHand:range_fextMEAS_rightHand+5),(range_fextMEAS_rightHand:range_fextMEAS_rightHand+5)));

% % Right hand estimates
range_fextEST_rightHand = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'RightHand',opts.stackOfTaskMAP);
fext.estimated.rightHand = estimation.mu_dgiveny((range_fextEST_rightHand:range_fextEST_rightHand+5 ),startIndex:endIndex);

subplot (3,3,2) % Right hand fx component
plot1 = plot(fext.estimated.rightHand(1,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(1);
shad1 = shadedErrorBar([],fext.measured.rightHand(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title(' Right Hand Forces','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
ylabel('$f_x$','FontSize',18, 'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (3,3,5) % Right hand fy component
plot1 = plot(fext.estimated.rightHand(2,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(2);
shad2 = shadedErrorBar([],fext.measured.rightHand(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
ylabel('$f_y$','FontSize',18, 'Interpreter','latex');
xlim([0 len])

subplot (3,3,8) % Right hand fz component
plot1 = plot(fext.estimated.rightHand(3,:),'b','lineWidth',1.5);
xlabel('samples','FontSize',18);
hold on
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(3);
shad2 = shadedErrorBar([],fext.measured.rightHand(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
ylabel('$f_z$','FontSize',18, 'Interpreter','latex');
xlim([0 len])
 
% % Left hand measurements
range_fextMEAS_leftHand = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'LeftHand',true);
fext.measured.leftHand = y_sim((range_fextMEAS_leftHand:range_fextMEAS_leftHand+5),startIndex:endIndex);
fext.measured.leftHand_sigma = diag(Sigmay((range_fextMEAS_leftHand:range_fextMEAS_leftHand+5),(range_fextMEAS_leftHand:range_fextMEAS_leftHand+5)));

% % Left hand estimates
range_fextEST_leftHand = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'LeftHand',opts.stackOfTaskMAP);
fext.estimated.leftHand = estimation.mu_dgiveny((range_fextEST_leftHand:range_fextEST_leftHand+5 ),startIndex:endIndex);

subplot (3,3,3) % left hand fx component
plot1 = plot(fext.estimated.leftHand(1,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(1);
shad1 = shadedErrorBar([],fext.measured.leftHand(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title(' Left Hand Forces','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
ylabel('$f_x$','FontSize',18, 'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (3,3,6) % left hand fy component
plot1 = plot(fext.estimated.leftHand(2,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(2);
shad2 = shadedErrorBar([],fext.measured.leftHand(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
ylabel('$f_y$','FontSize',18, 'Interpreter','latex');
xlim([0 len])

subplot (3,3,9) % left hand fz component
plot1 = plot(fext.estimated.leftHand(3,:),'b','lineWidth',1.5);
xlabel('samples','FontSize',18);
hold on
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(3);
shad2 = shadedErrorBar([],fext.measured.leftHand(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
ylabel('$f_z$','FontSize',18, 'Interpreter','latex');
xlim([0 len])

tightfig();
% save
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('HDE_rate_of_change_of_linear_momentum_vs_hands_force_estimates')),fig,600);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  Rate of Change of Angular Momentum
% -----------------------------------------------------------------------%

fig = figure('Name', 'Rate of Change of Angular Momentum Vs Hands Moments Estimate','NumberTitle','off','units','normalized','outerposition',[0 0 1 1]);
axes1 = axes('Parent',fig,'FontSize',16,'Units', 'normalized');
box(axes1,'on');
hold(axes1,'on');
grid on;

subplot (3,3,1) % dotL_x
plot1 = plot(y1_sim(298,:),'b','lineWidth',1.5);
hold on
title('Rate of Change of Angular Momentum','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
ylabel ('$\dot{L}_x$','FontSize',18,'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (3,3,4) % dotL_y
plot1 = plot(y1_sim(299,:),'b','lineWidth',1.5);
ylabel ('$\dot{L}_y$','FontSize',18,'Interpreter','latex');
hold on
grid on;
axis tight;
xlim([0 len])

subplot (3,3,7) % dotL_z
plot1 = plot(y1_sim(300,:),'b','lineWidth',1.5);
xlabel('samples','FontSize',18);
ylabel ('$\dot{L}_z$','FontSize',18,'Interpreter','latex');
hold on;
xlabel('z');
grid on;
axis tight;
xlim([0 len])


% Right hand measurements
range_fextMEAS_rightHand = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'RightHand', true);
fext.measured.rightHand = y_sim((range_fextMEAS_rightHand:range_fextMEAS_rightHand+5),startIndex:endIndex);
fext.measured.rightHand_sigma = diag(Sigmay((range_fextMEAS_rightHand:range_fextMEAS_rightHand+5),(range_fextMEAS_rightHand:range_fextMEAS_rightHand+5)));

% % Right hand estimates
range_fextEST_rightHand = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'RightHand',opts.stackOfTaskMAP);
fext.estimated.rightHand = estimation.mu_dgiveny((range_fextEST_rightHand:range_fextEST_rightHand+5 ),startIndex:endIndex);

subplot (3,3,2) % Right hand fx component
plot1 = plot(fext.estimated.rightHand(4,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(4);
shad1 = shadedErrorBar([],fext.measured.rightHand(4,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title(' Right Hand Forces','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
ylabel('$m_x$','FontSize',18, 'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (3,3,5) % Right hand fy component
plot1 = plot(fext.estimated.rightHand(5,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(5);
shad2 = shadedErrorBar([],fext.measured.rightHand(5,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
ylabel('$m_y$','FontSize',18, 'Interpreter','latex');
xlim([0 len])

subplot (3,3,8) % Right hand fz component
plot1 = plot(fext.estimated.rightHand(6,:),'b','lineWidth',1.5);
xlabel('samples','FontSize',18);
hold on
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(6);
shad2 = shadedErrorBar([],fext.measured.rightHand(6,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
ylabel('$m_z$','FontSize',18, 'Interpreter','latex');
xlim([0 len])
 
% % Left hand measurements
range_fextMEAS_leftHand = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'LeftHand',true);
fext.measured.leftHand = y_sim((range_fextMEAS_leftHand:range_fextMEAS_leftHand+5),startIndex:endIndex);
fext.measured.leftHand_sigma = diag(Sigmay((range_fextMEAS_leftHand:range_fextMEAS_leftHand+5),(range_fextMEAS_leftHand:range_fextMEAS_leftHand+5)));

% % Left hand estimates
range_fextEST_leftHand = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'LeftHand',opts.stackOfTaskMAP);
fext.estimated.leftHand = estimation.mu_dgiveny((range_fextEST_leftHand:range_fextEST_leftHand+5 ),startIndex:endIndex);

subplot (3,3,3) % left hand fx component
plot1 = plot(fext.estimated.leftHand(4,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(4);
shad1 = shadedErrorBar([],fext.measured.leftHand(4,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title(' Left Hand Forces','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
ylabel('$m_x$','FontSize',18, 'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (3,3,6) % left hand fy component
plot1 = plot(fext.estimated.leftHand(5,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(5);
shad2 = shadedErrorBar([],fext.measured.leftHand(5,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
ylabel('$m_y$','FontSize',18, 'Interpreter','latex');
xlim([0 len])

subplot (3,3,9) % left hand fz component
plot1 = plot(fext.estimated.leftHand(6,:),'b','lineWidth',1.5);
xlabel('samples','FontSize',18);
hold on
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(6);
shad2 = shadedErrorBar([],fext.measured.leftHand(6,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
ylabel('$m_x$','FontSize',18, 'Interpreter','latex');
xlim([0 len])

tightfig();
% save
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('HDE_rate_of_change_of_angular_momentum_vs_hands_force_estimates')),fig,600);
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  EXTERNAL FORCES
% -----------------------------------------------------------------------%
% - other applied (exernal) forces are null

fig = figure('Name', 'Foot External forces','NumberTitle','off','units','normalized','outerposition',[0 0 1 1]);
axes1 = axes('Parent',fig,'FontSize',16,'Units', 'normalized');
box(axes1,'on');
hold(axes1,'on');
grid on;

% Right foot measurements
range_fextMEAS_rightFoot = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'RightFoot', opts.stackOfTaskMAP);
fext.measured.rightFoot = y_sim((range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5),startIndex:endIndex);
fext.measured.rightFoot_sigma = diag(Sigmay((range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5),(range_fextMEAS_rightFoot:range_fextMEAS_rightFoot+5)));

% % Right foot estimate
range_fextEST_rightFoot = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'RightFoot',opts.stackOfTaskMAP);
fext.estimated.rightFoot = estimation.mu_dgiveny((range_fextEST_rightFoot:range_fextEST_rightFoot+5 ),startIndex:endIndex);

subplot (6,4,1) % Right foot fx component
plot1 = plot(fext.estimated.rightFoot(1,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(1);
shad1 = shadedErrorBar([],fext.measured.rightFoot(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title(' Right Foot','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
ylabel ('$f_x^x$','FontSize',18,'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (6,4,5) % Right foot fy component
plot1 = plot(fext.estimated.rightFoot(2,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(2);
shad2 = shadedErrorBar([],fext.measured.rightFoot(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
ylabel ('$f_y^x$','FontSize',18,'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (6,4,9) % Right foot fz component
plot1 = plot(fext.estimated.rightFoot(3,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(3);
shad2 = shadedErrorBar([],fext.measured.rightFoot(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
ylabel ('$f_z^x$','FontSize',18,'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (6,4,13) % Right foot mx component
plot1 = plot(fext.estimated.rightFoot(4,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(4);
shad2 = shadedErrorBar([],fext.measured.rightFoot(4,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
ylabel ('$m_x^x$','FontSize',18,'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (6,4,17) % Right foot my component
plot1 = plot(fext.estimated.rightFoot(5,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(5);
shad2 = shadedErrorBar([],fext.measured.rightFoot(5,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
ylabel ('$m_y^x$','FontSize',18,'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (6,4,21) % Right foot mz component
plot1 = plot(fext.estimated.rightFoot(6,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightFoot_sigma(6);
shad2 = shadedErrorBar([],fext.measured.rightFoot(6,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
ylabel ('$m_z^x$','FontSize',18,'Interpreter','latex');
xlabel('samples','FontSize',18);
grid on;
axis tight;
xlim([0 len])

% % Left foot measurements
range_fextMEAS_leftFoot = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'LeftFoot',opts.stackOfTaskMAP);
fext.measured.leftFoot = y_sim((range_fextMEAS_leftFoot:range_fextMEAS_leftFoot+5),startIndex:endIndex);
fext.measured.leftFoot_sigma = diag(Sigmay((range_fextMEAS_leftFoot:range_fextMEAS_leftFoot+5),(range_fextMEAS_leftFoot:range_fextMEAS_leftFoot+5)));

% % Left foot estimates
range_fextEST_leftFoot = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'LeftFoot',opts.stackOfTaskMAP);
fext.estimated.leftFoot = estimation.mu_dgiveny((range_fextEST_leftFoot:range_fextEST_leftFoot+5 ),startIndex:endIndex);

subplot (6,4,2) % left foot fx component
plot1 = plot(fext.estimated.leftFoot(1,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftFoot_sigma(1);
shad1 = shadedErrorBar([],fext.measured.leftFoot(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title(' Left Foot','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (6,4,6) % left foot fy component
plot1 = plot(fext.estimated.leftFoot(2,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftFoot_sigma(2);
shad2 = shadedErrorBar([],fext.measured.leftFoot(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])

subplot (6,4,10) % left foot fz component
plot1 = plot(fext.estimated.leftFoot(3,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftFoot_sigma(3);
shad2 = shadedErrorBar([],fext.measured.leftFoot(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])

subplot (6,4,14) % left foot mx component
plot1 = plot(fext.estimated.leftFoot(4,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftFoot_sigma(4);
shad2 = shadedErrorBar([],fext.measured.leftFoot(4,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])

subplot (6,4,18) % left foot my component
plot1 = plot(fext.estimated.leftFoot(5,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftFoot_sigma(5);
shad2 = shadedErrorBar([],fext.measured.leftFoot(5,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])

subplot (6,4,22) % left foot mz component
plot1 = plot(fext.estimated.leftFoot(5,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftFoot_sigma(5);
shad2 = shadedErrorBar([],fext.measured.leftFoot(5,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])
xlabel('samples','FontSize',18);


% Right hand measurements
range_fextMEAS_rightHand = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'RightHand', true);
fext.measured.rightHand = y_sim((range_fextMEAS_rightHand:range_fextMEAS_rightHand+5),startIndex:endIndex);
fext.measured.rightHand_sigma = diag(Sigmay((range_fextMEAS_rightHand:range_fextMEAS_rightHand+5),(range_fextMEAS_rightHand:range_fextMEAS_rightHand+5)));

% % Right hand estimates
range_fextEST_rightHand = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'RightHand',opts.stackOfTaskMAP);
fext.estimated.rightHand = estimation.mu_dgiveny((range_fextEST_rightHand:range_fextEST_rightHand+5 ),startIndex:endIndex);

subplot (6,4,3) % Right hand fx component
plot1 = plot(fext.estimated.rightHand(1,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(1);
shad1 = shadedErrorBar([],fext.measured.rightHand(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title(' Right Hand','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (6,4,7) % Right hand fy component
plot1 = plot(fext.estimated.rightHand(2,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(2);
shad2 = shadedErrorBar([],fext.measured.rightHand(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])

subplot (6,4,11) % Right hand fz component
plot1 = plot(fext.estimated.rightHand(3,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(3);
shad2 = shadedErrorBar([],fext.measured.rightHand(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])

subplot (6,4,15) % Right hand mx component
plot1 = plot(fext.estimated.rightHand(4,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(4);
shad2 = shadedErrorBar([],fext.measured.rightHand(4,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])

subplot (6,4,19) % Right hand my component
plot1 = plot(fext.estimated.rightHand(5,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(5);
shad2 = shadedErrorBar([],fext.measured.rightHand(5,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])


subplot (6,4,23) % Right hand my component
plot1 = plot(fext.estimated.rightHand(6,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.rightHand_sigma(6);
shad2 = shadedErrorBar([],fext.measured.rightHand(6,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])
xlabel('samples','FontSize',18);

 
% % Left hand measurements
range_fextMEAS_leftHand = rangeOfSensorMeasurement(berdy, iDynTree.NET_EXT_WRENCH_SENSOR, 'LeftHand',true);
fext.measured.leftHand = y_sim((range_fextMEAS_leftHand:range_fextMEAS_leftHand+5),startIndex:endIndex);
fext.measured.leftHand_sigma = diag(Sigmay((range_fextMEAS_leftHand:range_fextMEAS_leftHand+5),(range_fextMEAS_leftHand:range_fextMEAS_leftHand+5)));

% % Left hand estimates
range_fextEST_leftHand = rangeOfDynamicVariable(berdy, iDynTree.NET_EXT_WRENCH, 'LeftHand',opts.stackOfTaskMAP);
fext.estimated.leftHand = estimation.mu_dgiveny((range_fextEST_leftHand:range_fextEST_leftHand+5 ),startIndex:endIndex);

subplot (6,4,4) % left hand fx component
plot1 = plot(fext.estimated.leftHand(1,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(1);
shad1 = shadedErrorBar([],fext.measured.leftHand(1,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
title(' Left Hand','HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (6,4,8) % left hand fy component
plot1 = plot(fext.estimated.leftHand(2,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(2);
shad2 = shadedErrorBar([],fext.measured.leftHand(2,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])

subplot (6,4,12) % left hand fz component
plot1 = plot(fext.estimated.leftHand(3,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(3);
shad2 = shadedErrorBar([],fext.measured.leftHand(3,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])

subplot (6,4,16) % left hand mx component
plot1 = plot(fext.estimated.leftHand(4,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(4);
shad2 = shadedErrorBar([],fext.measured.leftHand(4,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])

subplot (6,4,20) % left hand my component
plot1 = plot(fext.estimated.leftHand(5,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(5);
shad2 = shadedErrorBar([],fext.measured.leftHand(5,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])

subplot (6,4,24) % left hand mz component
plot1 = plot(fext.estimated.leftHand(6,:),'b','lineWidth',1.5);
hold on
specific_vector_sigma(1,:) = fext.measured.leftHand_sigma(6);
shad2 = shadedErrorBar([],fext.measured.leftHand(6,:),2.*sqrt(specific_vector_sigma(1,:)),'r',1.5);
grid on;
axis tight;
xlim([0 len])
xlabel('samples','FontSize',18);

leg = legend([plot1,shad2.mainLine,shad2.patch],{'estim','meas','2$\sigma_{meas}$'});
set(leg,'Interpreter','latex', ...
       'Location','best', ...
       'Orientation','horizontal');
set(leg,'FontSize',13);

tightfig();
% save
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('HDE_force_estimation_fext_comparison')),fig,600);
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------------------------------------------------------------%
%  JOINT TORQUE - VERSION WITHOUT PLOTTED SIGMA TAU
% -----------------------------------------------------------------------%
for tauIdx = 1 : nrDofs
    % (after MAP) right ankle
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightAnkle_rotx')
        tau.estimated.rightAnkle.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightAnkle_roty')
        tau.estimated.rightAnkle.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightAnkle_rotz')
        tau.estimated.rightAnkle.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    % (after MAP) right knee
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightKnee_rotx')
        tau.estimated.rightKnee.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightKnee_roty')
        tau.estimated.rightKnee.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightKnee_rotz')
        tau.estimated.rightKnee.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    % (after MAP) right hip
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightHip_rotx')
        tau.estimated.rightHip.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightHip_roty')
        tau.estimated.rightHip.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightHip_rotz')
        tau.estimated.rightHip.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    % (after MAP) right C7shoulder
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightC7Shoulder_rotx')
        tau.estimated.rightC7Shoulder.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightC7Shoulder_roty')
        tau.estimated.rightC7Shoulder.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightC7Shoulder_rotz')
        tau.estimated.rightC7Shoulder.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    % (after MAP) right shoulder
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightShoulder_rotx')
        tau.estimated.rightShoulder.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightShoulder_roty')
        tau.estimated.rightShoulder.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightShoulder_rotz')
        tau.estimated.rightShoulder.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    % (after MAP) right elbow
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightElbow_rotx')
        tau.estimated.rightElbow.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightElbow_roty')
        tau.estimated.rightElbow.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightElbow_rotz')
        tau.estimated.rightElbow.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    % (after MAP) right wrist
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightWrist_rotx')
        tau.estimated.rightWrist.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightWrist_roty')
        tau.estimated.rightWrist.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jRightWrist_rotz')
        tau.estimated.rightWrist.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    % ---------------------
    % (after MAP) left ankle
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftAnkle_rotx')
        tau.estimated.leftAnkle.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftAnkle_roty')
        tau.estimated.leftAnkle.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftAnkle_rotz')
        tau.estimated.leftAnkle.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    % (after MAP) left knee
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftKnee_rotx')
        tau.estimated.leftKnee.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftKnee_roty')
        tau.estimated.leftKnee.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftKnee_rotz')
        tau.estimated.leftKnee.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    % (after MAP) left hip
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftHip_rotx')
        tau.estimated.leftHip.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftHip_roty')
        tau.estimated.leftHip.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftHip_rotz')
        tau.estimated.leftHip.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    % (after MAP) left C7shoulder
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftC7Shoulder_rotx')
        tau.estimated.leftC7Shoulder.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLefttC7Shoulder_roty')
        tau.estimated.leftC7Shoulder.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftC7Shoulder_rotz')
        tau.estimated.leftC7Shoulder.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    % (after MAP) left shoulder
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftShoulder_rotx')
        tau.estimated.leftShoulder.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftShoulder_roty')
        tau.estimated.leftShoulder.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftShoulder_rotz')
        tau.estimated.leftShoulder.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    % (after MAP) left elbow
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftElbow_rotx')
        tau.estimated.leftElbow.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftElbow_roty')
        tau.estimated.leftElbow.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftElbow_rotz')
        tau.estimated.leftElbow.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    % (after MAP) left wrist
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftWrist_rotx')
        tau.estimated.leftWrist.rotx = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftWrist_roty')
        tau.estimated.leftWrist.roty = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
    if strcmp(estimatedVariables.tau.label{tauIdx},'jLeftWrist_rotz')
        tau.estimated.leftWrist.rotz = estimatedVariables.tau.values(tauIdx,startIndex:endIndex);
    end
end


%% RIGHT ANKLE-KNEE-HIP
fig = figure('Name', 'Right leg torques','NumberTitle','off','units','normalized','outerposition',[0 0 1 1]);
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;
% ----
subplot (331) % right ankle x component
plot1 = plot(tau.estimated.rightAnkle.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightAnkle.rotx,2.*sqrt(Sigma_tau.rightAnkle_rotx),'b',1.5);
hold on
ylabel({'jRightAnkle','$Nm$'},'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
title ('x','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (332) % right ankle y component
plot1 = plot(tau.estimated.rightAnkle.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightAnkle.roty,2.*sqrt(Sigma_tau.rightAnkle_roty),'b',1.5);
title ('y','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (333) % right ankle z component
plot1 = plot(tau.estimated.rightAnkle.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightAnkle.rotz,2.*sqrt(Sigma_tau.rightAnkle_rotz),'b',1.5);
title ('z','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (335) % right knee y component
plot1 = plot(tau.estimated.rightKnee.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightKnee.roty,2.*sqrt(Sigma_tau.rightAnkle_roty),'b',1.5);
ylabel({'jRightKnee','$Nm$'},'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (336) % right knee z component
plot1 = plot(tau.estimated.rightKnee.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightKnee.rotz,2.*sqrt(Sigma_tau.rightAnkle_rotz),'b',1.5);
grid on;
axis tight;
xlim([0 len])

subplot (337) % right hip x component
plot1 = plot(tau.estimated.rightHip.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightHip.rotx,2.*sqrt(Sigma_tau.rightHip_rotx),'b',1.5);
hold on
ylabel({'jRightHip','$Nm$'},'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('samples','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (338) % right hip y component
plot1 = plot(tau.estimated.rightHip.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightHip.roty,2.*sqrt(Sigma_tau.rightHip_roty),'b',1.5);
xlabel('samples','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (339) % right hip z component
plot1 = plot(tau.estimated.rightHip.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightHip.rotz,2.*sqrt(Sigma_tau.rightHip_rotz),'b',1.5);
% title ('z');
xlabel('samples','FontSize',18);
grid on;
axis tight;
xlim([0 len])

% % % tau plot legend
% % leg = legend([plot1,shad1.mainLine,shad1.patch],{'estim', 'meas','2$\sigma_{meas}$'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

% % put a title on the top of the subplots
% ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% text(0.5, 1,'\bf Joint torque [Nm]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);

tightfig();
% save
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('HDE_force_estimation_right_leg_joint_torque_estimates')),fig,600);
end

%% LEFT ANKLE-KNEE-HIP
fig = figure('Name', 'Left leg torques','NumberTitle','off','units','normalized','outerposition',[0 0 1 1]);
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

subplot (331) % left ankle x component
plot1 = plot(tau.estimated.leftAnkle.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftAnkle.rotx,2.*sqrt(Sigma_tau.leftAnkle_rotx),'b',1.5);
hold on
ylabel({'jLeftAnkle','$Nm$'},'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
title ('x','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (332) % left ankle y component
plot1 = plot(tau.estimated.leftAnkle.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftAnkle.roty,2.*sqrt(Sigma_tau.leftAnkle_roty),'b',1.5);
title ('y','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (333) % left ankle z component
plot1 = plot(tau.estimated.leftAnkle.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftAnkle.rotz,2.*sqrt(Sigma_tau.leftAnkle_rotz),'b',1.5);
title ('z','FontSize',18);
grid on;
axis tight;
xlim([0 len])

% % subplot (334) % left knee x component
% % plot1 = plot(tau.estimated.leftKnee.rotx,'b','lineWidth',1.5);
% % % shad2 = shadedErrorBar([],tau.estimated.leftKnee.roty,2.*sqrt(Sigma_tau.leftAnkle_roty),'b',1.5);
% % ylabel('leftKnee','HorizontalAlignment','center',...
% %     'FontWeight','bold',...
% %     'FontSize',18,...
% %     'Interpreter','latex');
% % xlabel('N samples');
% % grid on;
% % axis tight;
% % xlim([0 len])

subplot (335) % left knee y component
plot1 = plot(tau.estimated.leftKnee.roty,'b','lineWidth',1.5);
ylabel({'jLeftKnee','$Nm$'},'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (336) % left knee z component
plot1 = plot(tau.estimated.leftKnee.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftKnee.rotz,2.*sqrt(Sigma_tau.leftAnkle_rotz),'b',1.5);
grid on;
axis tight;
xlim([0 len])

% ----
subplot (337) % left hip x component
plot1 = plot(tau.estimated.leftHip.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftHip.rotx,2.*sqrt(Sigma_tau.leftHip_rotx),'b',1.5);
hold on
ylabel({'jLeftHip','$Nm$'},'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('samples','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (338) % left hip y component
plot1 = plot(tau.estimated.leftHip.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftHip.roty,2.*sqrt(Sigma_tau.leftHip_roty),'b',1.5);
xlabel('samples','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (339) % left hip z component
plot1 = plot(tau.estimated.leftHip.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftHip.rotz,2.*sqrt(Sigma_tau.leftHip_rotz),'b',1.5);
xlabel('samples','FontSize',18);
grid on;
axis tight;
xlim([0 len])

% align_Ylabels(fig) % if there are 9 subplots, align the ylabels

% % % tau plot legend
% % leg = legend([plot1],{'estim'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

% % put a title on the top of the subplots
% ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% text(0.5, 1,'\bf Joint torque [Nm]','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);

tightfig();
% save
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('HDE_force_estimation_left_leg_joint_torque_estimates')),fig,600);
end

%% RIGHT ARM
fig = figure('Name', 'Right arm torques','NumberTitle','off','units','normalized','outerposition',[0 0 1 1]);
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

subplot (331) % right shoulder x component
plot1 = plot(tau.estimated.rightShoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotx,2.*sqrt(Sigma_tau.rightShoulder_rotx),'b',1.5);
hold on
ylabel({'jRightShoulder','$Nm$'},'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
title ('x','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (332) % right shoulder y component
plot1 = plot(tau.estimated.rightShoulder.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.roty,2.*sqrt(Sigma_tau.rightShoulder_roty),'b',1.5);
title ('y');
grid on;
axis tight;
xlim([0 len])

subplot (333) % right shoulder z component
plot1 = plot(tau.estimated.rightShoulder.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
title ('z');
grid on;
axis tight;
xlim([0 len])

subplot (334) % right C7shoulder x component
plot1 = plot(tau.estimated.rightC7Shoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightC7Shoulder.rotx,2.*sqrt(Sigma_tau.rightC7Shoulder_rotx),'b',1.5);
hold on
ylabel({'jRightC7Shoulder','$Nm$'},'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (335) % right C7shoulder y component
plot1 = plot(tau.estimated.rightShoulder.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
grid on;
axis tight;
xlim([0 len])

subplot (336) % right C7shoulder z component
plot1 = plot(tau.estimated.rightShoulder.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
grid on;
axis tight;
xlim([0 len])

% subplot (437) % right elbow x component
% plot1 = plot(tau.estimated.rightElbow.rotx,'b','lineWidth',1.5);
% % shad2 = shadedErrorBar([],tau.estimated.rightC7Shoulder.rotx,2.*sqrt(Sigma_tau.rightC7Shoulder_rotx),'b',1.5);
% hold on
% ylabel('rightElbow','HorizontalAlignment','center',...
%     'FontWeight','bold',...
%     'FontSize',18,...
%     'Interpreter','latex');
% xlabel('N samples');
% grid on;
% axis tight;
% xlim([0 len])

subplot (338) % right elbow y component
plot1 = plot(tau.estimated.rightElbow.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
ylabel({'jRightElbow','$Nm$'},'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('samples','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (339) % right elbow z component
plot1 = plot(tau.estimated.rightElbow.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('samples','FontSize',18);
title ('z');
grid on;
axis tight;
xlim([0 len])


% % % tau plot legend
% % leg = legend([plot1],{'estim'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

% % put a title on the top of the subplots
% ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% text(0.5, 1,'','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);

tightfig();
% save
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('HDE_force_estimation_right_arm_joint_torque_estimates')),fig,600);
end

%% LEFT ARM
fig = figure('Name', 'Left arm torques','NumberTitle','off','units','normalized','outerposition',[0 0 1 1]);
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
grid on;

subplot (331) % left shoulder x component
plot1 = plot(tau.estimated.leftShoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.leftShoulder.rotx,2.*sqrt(Sigma_tau.rightShoulder_rotx),'b',1.5);
hold on
ylabel({'jLeftShoulder','$Nm$'},'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
title ('x','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (332) % left shoulder y component
plot1 = plot(tau.estimated.leftShoulder.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.roty,2.*sqrt(Sigma_tau.rightShoulder_roty),'b',1.5);
title ('y','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (333) % left shoulder z component
plot1 = plot(tau.estimated.leftShoulder.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
title ('z','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (334) % left C7shoulder x component
plot1 = plot(tau.estimated.leftC7Shoulder.rotx,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightC7Shoulder.rotx,2.*sqrt(Sigma_tau.rightC7Shoulder_rotx),'b',1.5);
hold on
ylabel({'jRightC7Shoulder','$Nm$'},'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
grid on;
axis tight;
xlim([0 len])

subplot (335) % left C7shoulder y component
plot1 = plot(tau.estimated.leftShoulder.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5)
grid on;
axis tight;
xlim([0 len])

subplot (336) % left C7shoulder z component
plot1 = plot(tau.estimated.leftShoulder.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
grid on;
axis tight;
xlim([0 len])

% % subplot (437) % left elbow x component
% % plot1 = plot(tau.estimated.leftElbow.rotx,'b','lineWidth',1.5);
% % % shad2 = shadedErrorBar([],tau.estimated.rightC7Shoulder.rotx,2.*sqrt(Sigma_tau.rightC7Shoulder_rotx),'b',1.5);
% % hold on
% % ylabel('rightElbow','HorizontalAlignment','center',...
% %     'FontWeight','bold',...
% %     'FontSize',18,...
% %     'Interpreter','latex');
% % xlabel('N samples');
% % grid on;
% % axis tight;
% % xlim([0 len])

subplot (338) % left elbow y component
plot1 = plot(tau.estimated.leftElbow.roty,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
ylabel({'jRightElbow','$Nm$'},'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',18,...
    'Interpreter','latex');
xlabel('samples','FontSize',18);
grid on;
axis tight;
xlim([0 len])

subplot (339) % left elbow z component
plot1 = plot(tau.estimated.leftElbow.rotz,'b','lineWidth',1.5);
% shad2 = shadedErrorBar([],tau.estimated.rightShoulder.rotz,2.*sqrt(Sigma_tau.rightShoulder_rotz),'b',1.5);
xlabel('samples','FontSize',18);
grid on;
axis tight;
xlim([0 len])

% % % tau plot legend
% % leg = legend([plot1],{'estim'},'Location','northeast');
% % set(leg,'Interpreter','latex', ...
% %        'Position',[0.436917552718887 0.0353846154974763 0.158803168001834 0.0237869821356598], ...
% %        'Orientation','horizontal');
% % set(leg,'FontSize',13);

% % put a title on the top of the subplots
% ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% text(0.5, 1,'','HorizontalAlignment','center','VerticalAlignment', 'top','FontSize',14);

tightfig();
% save
if saveON
    save2pdf(fullfile(bucket.pathToPlots, ('HDE_force_estimation_left_arm_joint_torque_estimates')),fig,600);
end
