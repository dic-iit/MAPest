
% =========================================================================
%                 NORM MEAN WHOLE-BODY EFFECTS
% =========================================================================
%% Intra-subject whole-body torque norm
for subjIdx = 1 : nrOfSubject
    for blockIdx = 1 : block.nrOfBlocks
        intraSubj(subjIdx).torqueNormNE(blockIdx).block = block.labels(blockIdx);
        intraSubj(subjIdx).torqueNormWE(blockIdx).block = block.labels(blockIdx);
        lenNE = length(intraSubj(subjIdx).NE.estimatedVariables.tau(blockIdx).values);
        lenWE = length(intraSubj(subjIdx).WE.estimatedVariables.tau(blockIdx).values);
        % ---- norm NE
        for i = 1 : lenNE
            intraSubj(subjIdx).torqueNormNE(blockIdx).torqueNorm(1,i) = ...
                norm(intraSubj(subjIdx).NE.estimatedVariables.tau(blockIdx).values(:,i));
        end
        % ---- norm WE
        for i = 1 : lenWE
            intraSubj(subjIdx).torqueNormWE(blockIdx).torqueNorm(1,i) = ...
                norm(intraSubj(subjIdx).WE.estimatedVariables.tau(blockIdx).values(:,i));
        end
    end
end

% Normalization
for subjIdx = 1 : nrOfSubject
    for blockIdx = 1 : block.nrOfBlocks
        % ---- normalized norm NE
        intraSubj(subjIdx).torqueNormNE(blockIdx).torqueNorm_normalized = ...
            (intraSubj(subjIdx).torqueNormNE(blockIdx).torqueNorm - ...
            min(intraSubj(subjIdx).torqueNormNE(blockIdx).torqueNorm))/ ...
            (max(intraSubj(subjIdx).torqueNormNE(blockIdx).torqueNorm) - ...
            min(intraSubj(subjIdx).torqueNormNE(blockIdx).torqueNorm));
        % ---- normalized norm WE
        intraSubj(subjIdx).torqueNormWE(blockIdx).torqueNorm_normalized = ...
            (intraSubj(subjIdx).torqueNormWE(blockIdx).torqueNorm - ...
            min(intraSubj(subjIdx).torqueNormWE(blockIdx).torqueNorm))/ ...
            (max(intraSubj(subjIdx).torqueNormWE(blockIdx).torqueNorm) - ...
            min(intraSubj(subjIdx).torqueNormWE(blockIdx).torqueNorm));
    end
end

%% Statistics
stats_vect = [];
tmp.tmp_vect = [];
for blockIdx = 1 : block.nrOfBlocks
    for subjIdx = 1 : nrOfSubject
        % NE
        tmp.tmp_vect(subjIdx,1) = ...
            mean(intraSubj(subjIdx).torqueNormNE(blockIdx).torqueNorm_normalized);
        % WE
        tmp.tmp_vect(subjIdx,2) = ...
            mean(intraSubj(subjIdx).torqueNormWE(blockIdx).torqueNorm_normalized);
    end
    stats_vect = [stats_vect; tmp.tmp_vect];
end
% ANOVA2 computation
% Computation two-way analysis of variance (ANOVA) with balanced designs.
% - columns --> conditions (NE or WE), nrOfGroups = 2
% - rows    --> block (per subject) --> block.nrOfBlocks*nrOfSubject
repetitions = block.nrOfBlocks;
[~,~,stats_anova2] = anova2(stats_vect,repetitions);
c = multcompare(stats_anova2);

%% ----- Box plot
% ============================= single block ==============================
fig = figure('Name', 'stats all joints','NumberTitle','off');
axes1 = axes('Parent',fig,'FontSize',16);
box(axes1,'on');
hold(axes1,'on');
for blockIdx = 1 : block.nrOfBlocks
    sub1 = subplot(1,5,blockIdx);
    tmp.range = nrOfSubject*(blockIdx-1)+1 : nrOfSubject*blockIdx;
    box1 = boxplot(stats_vect(tmp.range,:));
    ax = gca;
    ax.FontSize = 20;
    set(box1, 'Linewidth', 2.5);
    h = findobj(gca,'Tag','Box');
    % Group 1
    patch(get(h(1),'XData'),get(h(1),'YData'),orangeAnDycolor,'FaceAlpha',.8);
    % Group 2
    patch(get(h(2),'XData'),get(h(2),'YData'),greenAnDycolor,'FaceAlpha',.8);
    
    title(sprintf('Block %s', num2str(blockIdx)),'FontSize',20);
    if blockIdx == 1
        ylabel('$|\tau^{wb}|$', 'HorizontalAlignment','center',...
            'FontSize',25,'interpreter','latex');
    else
        set(gca,'YTickLabel',[]);
    end
    %     xlabel('Conditions','HorizontalAlignment','center',...
    %             'FontSize',25,'interpreter','latex');
    set(sub1,'TickLabelInterpreter','none','XTick',[1 2],...
        'XTickLabel',{'WE','NE'});
    ax=gca;
    ax.XAxis.FontSize = 20;
    ylim([0 1]);
    grid on;
end

% ================================ total ==================================
sub2 = subplot(1,6,6);
box1 = boxplot(stats_vect);
ax = gca;
ax.FontSize = 20;
set(box1, 'Linewidth', 2.5);
h = findobj(gca,'Tag','Box');
% Group 1
patch(get(h(1),'XData'),get(h(1),'YData'),orangeAnDycolor,'FaceAlpha',.8);
% Group 2
patch(get(h(2),'XData'),get(h(2),'YData'),greenAnDycolor,'FaceAlpha',.8);

% ----- Statistical significance among pair of groups
% Add stars and lines highlighting significant differences between pairs of groups.
% Stars are drawn according to:
%   * represents p<=0.05
%  ** represents p<=1E-2
% *** represents p<=1E-3
H = sigstar({[1,2]},[c(1,6)]);
title('Total','FontSize',20);
% % %     ylabel('Normalized $\tau_{NORM}$ mean', 'HorizontalAlignment','center',...
% % %         'FontSize',25,'interpreter','latex');
% % % % xlabel('Factor: subjects ','HorizontalAlignment','center',...
% % % %     'FontSize',25,'interpreter','latex');
set(sub2,'TickLabelInterpreter','none','XTick',[1 2],...
    'XTickLabel',{'WE','NE'});
ax=gca;
ax.XAxis.FontSize = 20;
ylim([0 1]);
grid on;
