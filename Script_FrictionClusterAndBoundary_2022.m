%%%%%%%%%%%%  Script .m   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Script Purpose: code for paper "Boxes-based Representation and Data Sharing of Road Surface Friction for CAVs"
% Matlab work Path: ~\Publications_Journals_2022RSS
% Author:       Liming
% Created Date: 2022-10-15
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% step 0: Prepare the workspace
% Clear workspace, figures, and console
clear all;  %#ok<CLALL> % Clears workspace to remove any old variables
close all;  % Close all open figures
clc;        % Clear console space

addpath('./Functions/');    % all the functions and wrapper class
addpath('./Data/');    % all the .mat data files
addpath(genpath('../Images/'));    % all the image files

dir.datafiles = ['.' filesep 'Data' filesep]; % all the .mat data files

% flag setup 
flag.doDebug = false; % set to 'true' to print trajectory information to command window
flag.friction_noise = true;
flag.xEast_noise = true;
flag.yNorth_noise = true;
flag.cluster_confidence_interval = false;

Axis_label_FontSize = 12;
index_ref = 39;
index_lane_center_lateral=[20 58];

%% Step1: load/generate road grid and true friction data
%NOTE: All the data already queried from database. Please download the data
%before running this data load code
listOfSections = 7002; % focus on the 256/7002 road segment
load(strcat(dir.datafiles,'sectionRef_table_section_',strjoin(cellstr(num2str(listOfSections)),'_'), '.mat'),'sectionRef_table')
load(strcat(dir.datafiles,'FrictionGrid_ref_section_',strjoin(cellstr(num2str(listOfSections)),'_'), '.mat'),'FrictionGrid_ref')

%% Step2: load friction data or run vehicle dynamic simualtion get the friction measurement data
% check runCoupledPathFollowingSim.m
Trip_id = 2022;
load(strcat(dir.datafiles,'trajectory_table_trips',strjoin(cellstr(num2str(Trip_id)),'_'),'.mat'),'trajectory_table')
trajectory_table = trajectory_table(trajectory_table.section_id == listOfSections,:);

%% Step3: load friction measurement

% load data friction true and measurement data which is queried using runCoupledPathFollowingSim.m
load(strcat(dir.datafiles,'frictionMeasure_table_processed_section_7002.mat'),'friction_measurement_processed')
% 

seed = 1;
rng(seed);

if flag.friction_noise
    friction_measurement_noisy_origin = friction_measurement_processed.friction_measurement_noisy;
    Signal_noise_ratio = 35;
    friction_measurement_processed.friction_measurement_noisy = ...
        awgn(friction_measurement_processed.friction_true,Signal_noise_ratio,'measured');
else
    friction_measurement_processed.friction_measurement_noisy = friction_measurement_processed.friction_true;
end

if flag.xEast_noise
    %     xEast_noisy_origin = friction_measurement_processed.contact_point_east;
    Location_signal_noise_ratio = 35;
    friction_measurement_processed.east_noisy = ...
        awgn(friction_measurement_processed.contact_point_east,Location_signal_noise_ratio);
else
    friction_measurement_processed.east_noisy = friction_measurement_processed.contact_point_east;
end

if flag.yNorth_noise
    Location_signal_noise_ratio = 35;
    friction_measurement_processed.north_noisy = ...
        awgn(friction_measurement_processed.contact_point_north,Location_signal_noise_ratio);
else
    friction_measurement_processed.north_noisy = friction_measurement_processed.contact_point_north;
end


if flag.doDebug
    %% plot noisy position 
    h_fig = figure(12048); % thesis
    set(h_fig,'Name','trajectory true and noisy');
    width=640;%
    height=450;%
    right=100;%
    bottom=400;%
    set(gcf,'position',[right,bottom,width,height])
    clf;
    rows = (friction_measurement_processed.vehicle_id==[2682]);
    plot(friction_measurement_processed.contact_point_east(rows),...
        friction_measurement_processed.contact_point_north(rows),'b')
    hold on
    plot(friction_measurement_processed.east_noisy(rows),...
        friction_measurement_processed.north_noisy(rows),'r')
    
    grid on
    
    h_fig = figure(12049); % thesis
    set(h_fig,'Name','trajectory true and noisy');
    width=640;%
    height=450;%
    right=100;%
    bottom=400;%
    set(gcf,'position',[right,bottom,width,height])
    clf;
    rows = (friction_measurement_processed.vehicle_id==[2682]);
    plot(friction_measurement_processed.contact_point_north(rows),...
      'b')
    hold on
    plot(friction_measurement_processed.north_noisy(rows),...
        'r.')
    
    grid on

end

%% plot true and measurement friction data
if flag.doDebug
    h_fig = figure(20945); % thesis
    set(h_fig,'Name','friction true and noisy measurement');
    width=640;%
    height=450;%
    right=100;%
    bottom=400;%
    set(gcf,'position',[right,bottom,width,height])
    clf;
    hold on
    
    % true friction
    offset_x = 100;  % 100
    offset_y = 0;  % 50
    scatter3(reshape(FrictionGrid_ref.position_x,[],1)-offset_x ,reshape(FrictionGrid_ref.position_y,[],1) + offset_y ,reshape(FrictionGrid_ref.friction,[],1),10,reshape(FrictionGrid_ref.friction,[],1),'.');
    hold on
    scatter3(FrictionGrid_ref.position_x(20,:)-offset_x ,FrictionGrid_ref.position_y(20,:) + offset_y ,FrictionGrid_ref.friction(20,:),10,FrictionGrid_ref.friction(20,:),'r.');
    scatter3(FrictionGrid_ref.position_x(10,:)-offset_x ,FrictionGrid_ref.position_y(10,:) + offset_y ,FrictionGrid_ref.friction(10,:),10,FrictionGrid_ref.friction(10,:),'r.');
    
    % measured friction
    
    scatter3(friction_measurement_processed.contact_point_east,friction_measurement_processed.contact_point_north,...
        friction_measurement_processed.friction_measurement_noisy,10, friction_measurement_processed.friction_measurement_noisy,'.');
    surf(FrictionGrid_ref.position_x,FrictionGrid_ref.position_y,FrictionGrid_ref.friction,'EdgeColor','r','FaceColor','none')
    %     scatter3(friction_measurement_processed.contact_point_east,friction_measurement_processed.contact_point_north,friction_measurement_processed.friction_true,10, friction_measurement_processed.friction_true,'g.');
    
    scatter3(FrictionGrid_ref.position_x(20,:) ,FrictionGrid_ref.position_y(20,:) ,FrictionGrid_ref.friction(20,:),10,FrictionGrid_ref.friction(20,:),'r.');
    scatter3(FrictionGrid_ref.position_x(58,:) ,FrictionGrid_ref.position_y(58,:) ,FrictionGrid_ref.friction(58,:),10,FrictionGrid_ref.friction(58,:),'r.');
    scatter3(FrictionGrid_ref.position_x(10,:) ,FrictionGrid_ref.position_y(10,:) ,FrictionGrid_ref.friction(10,:),10,FrictionGrid_ref.friction(10,:),'r.');
    scatter3(FrictionGrid_ref.position_x(6,:) ,FrictionGrid_ref.position_y(6,:) ,FrictionGrid_ref.friction(6,:),10,FrictionGrid_ref.friction(6,:),'r.');
    scatter3(FrictionGrid_ref.position_x(14,:) ,FrictionGrid_ref.position_y(14,:) ,FrictionGrid_ref.friction(14,:),10,FrictionGrid_ref.friction(14,:),'r.');
    % aimsun trajectory
    plot(trajectory_table.positioncg_x,trajectory_table.positioncg_y,'m.')
    annotation(h_fig,'textarrow',[0.401648016747403 0.359222022114258],...
        [0.861707744530094 0.808252469621471],...
        'String',{'Estimated Road Friction Distribution'},...
        'FontSize',11,...
        'FontName','Times New Roman');
    annotation(h_fig,'textarrow',[0.466367063159658 0.512486011057128],...
        [0.199340501792115 0.289905370246451],...
        'String',{'True Road Friction Distribution'},...
        'FontSize',11,...
        'FontName','Times New Roman');
    
    %legend('Ture Road Friction Distribution', 'Estimated Road Friction Distribution')
    grid on
    xlabel('xEast [m]')
    ylabel('yNorth [m]')
    zlabel('friction\_coeficient')
    %axis equal
    view(0,90)
    zlim([0 1])
    caxis([0.1,0.9])
    colormap(jet); % used for ACC paper
    colormap(parula);% used for IVSG friction reprsentation,  parula
    %     colormap(prism);%
    colorbar
    
    % axis equal
    axes_handle = gca;
    set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', 14);
    axes_handle.GridLineStyle = '-.';
    axes_handle.GridColor = 'k';
    axes_handle.GridAlpha = 0.2;
    %             ax.YMinorGrid = 'on';
    %             ax.MinorGridAlpha = 0.5
    box on
    %     saveas(h_fig,'Figures\friction_measurement_withSwerve.svg')
    %
    %     saveas(h_fig,'Figures\friction_measurement_withSwerve.pdf')
    %     savefig(h_fig,'Figures\friction_measurement_withSwerve.fig')
    %%
    h_fig = figure(20946); % thesis
    set(h_fig,'Name','friction true');
    width=640;%
    height=450;%
    right=100;%
    bottom=400;%
    set(gcf,'position',[right,bottom,width,height])
    clf;
    hold on
    % measured friction
    scatter3(friction_measurement_processed.contact_point_east,friction_measurement_processed.contact_point_north,friction_measurement_processed.friction_true,10, friction_measurement_processed.friction_true,'.');
    %     surf(FrictionGrid_ref.position_x,FrictionGrid_ref.position_y,FrictionGrid_ref.friction,'EdgeColor','r','FaceColor','none')
    scatter3(friction_measurement_processed.contact_point_east,friction_measurement_processed.contact_point_north,...
        friction_measurement_processed.friction_measurement_noisy,10, friction_measurement_processed.friction_measurement_noisy,'.');
    
    
    grid on
    xlabel('xEast [m]')
    ylabel('yNorth [m]')
    zlabel('friction\_coeficient')
    %axis equal
    view(0,90)
    zlim([0 1])
    colormap(jet); % used for ACC paper
    colormap(parula);% used for IVSG friction reprsentation,  parula
    %     colormap(prism);%
    colorbar
    
    % axis equal
    axes_handle = gca;
    set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', 14);
    axes_handle.GridLineStyle = '-.';
    axes_handle.GridColor = 'k';
    axes_handle.GridAlpha = 0.2;
    %             ax.YMinorGrid = 'on';
    %             ax.MinorGridAlpha = 0.5
    box on
    
    
    
    %% surf plot and image of true friction data
    
    h_fig = figure(4444);
    set(h_fig,'Name','True Friction ENU');
    width=540;%
    height=300;%
    right=100;%
    bottom=200;%
    set(gcf,'position',[right,bottom,width,height])
    clf
    hold on
    surf(FrictionGrid_ref.position_x + 7800,FrictionGrid_ref.position_y - 1500,FrictionGrid_ref.friction)
    % plot(sectionRef_table.east,sectionRef_table.north,'k.')
    % axis tight
    grid on
    colorbar
    % shading interp
    shading flat;lighting phong
    view(0,90)
    caxis([0.1 0.9])
    zlim([0 1])
    % xlim([-8300 -7850])
    % axis equal
    box on
    xlabel('xEast [m]')
    ylabel('yNorth [m]')
    annotation(h_fig,'arrow',[0.676 0.629777777777778],[0.2596 0.1892]);
    annotation(h_fig,'arrow',[0.678074074074073 0.630074074074074],...
        [0.258466666666668 0.365066666666668]);
    axes_handle = gca;
    set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize);
    axes_handle.GridLineStyle = '-.';
    axes_handle.GridColor = 'k';
    axes_handle.GridAlpha = 0.1;
    
    annotation(h_fig,'textbox',...
        [0.594444444444446 0.169999998283387 0.0688888900898121 0.105333335049947],...
        'String','T',...
        'LineStyle','none',...
        'FontSize',14,...
        'FontName','Times New Roman');
    annotation(h_fig,'textbox',...
        [0.590740740740743 0.293999998283387 0.0659259270562066 0.105333335049947],...
        'String','S',...
        'LineStyle','none',...
        'FontSize',14,...
        'FontName','Times New Roman');
    
    
    h_fig=figure(4445);
    set(h_fig,'Name','True Friction STH');
    width=540;%
    height=300;%
    right=100;%
    bottom=600;%
    set(gcf,'position',[right,bottom,width,height])
    clf
    hold on
    surf(FrictionGrid_ref.position_s,FrictionGrid_ref.position_t,FrictionGrid_ref.friction)
    plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
    for i= 1:length(index_lane_center_lateral)
        plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
    end
    % axis tight
    % grid on
    colorbar
    % shading interp
    shading flat;lighting phong
    view(0,90)
    caxis([0.1 0.9])
    zlim([0 1])
    xlim([-1 500])
    xlabel('Station [m]')
    ylabel('Transverse [m]')
    %     axis equal
    box on
    
    legend1 = legend('Road Surface Friction','Road Ref. Line', 'Lane Center Lines');
    set(legend1,...
        'Position',[0.54527677255157 0.929568589438862 0.161207974446963 0.11505586991763],...
        'FontName', 'Times New Roman', 'FontSize', 11);
    axes_handle = gca;
    set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize);
    axes_handle.GridLineStyle = '-.';
    axes_handle.GridColor = 'k';
    axes_handle.GridAlpha = 0.1;
    
    
    figure(4446)
    clf
    image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        FrictionGrid_ref.friction,'CDataMapping','scaled')
    xlabel('s[m]')
    ylabel('t[m]')
    %     axis equal
    box on
end
FrictionGrid_ref.friction_vector = reshape(FrictionGrid_ref.friction,[],1);

%% Step 4:  cluster the true data
if 1==1 %flag.doDebug
    % split friction data into groups
    nbs_friction_grid = 10;
    [counts,edges] = histcounts(FrictionGrid_ref.friction,nbs_friction_grid,'BinLimits',[0,1]); % Calculate a 101-bin histogram for the image.
    % execute the k-means cluster
    nbs_cluster = length(counts(counts>100));
    [cluster_idx,cluster_centroid_locations,sumDist] = kmeans(FrictionGrid_ref.friction_vector,nbs_cluster,'Replicates',3,'Display','final');
    
    % histtogram of friction data
    h_fig = figure(15565);
    set(h_fig,'Name','histogram of friction data');
    histogram(FrictionGrid_ref.friction_vector,edges)
    %stem(edges,counts)
    %% find boundary of each cluster
    method_boundary = 4;
    if method_boundary ==1 % cluster find the boundaries through nearest neighbor
        threshold = 0.15; %0.145; %meters, the Diagonal length of grid( 0.2 > threshold >0.141)
        nbs_neighbor = 8 ; % the number of neighbor nodes for each innner grid
        road_friction_cluster_boundary = [];
        boundary_index = [];
        for i_cluster=1:nbs_cluster
            
            %     cluster_friction_position = results_road_lane_grid(cluster_idx==i_cluster,:);
            cluster_index = find(cluster_idx == i_cluster);
            cluster_grid = [FrictionGrid_ref.position_x_vector(cluster_index) FrictionGrid_ref.position_y_vector(cluster_index)];
            cluster_friction = FrictionGrid_ref.friction_vector(cluster_index);
            Md_cluster_grid = KDTreeSearcher(cluster_grid); % Mdl is a KDTreeSearcher model. By default, the distance metric it uses to search for neighbors is Euclidean distance.
            [~,D] = knnsearch(Md_cluster_grid,cluster_grid,'k',nbs_neighbor+1);
            %boundary_index = Idx(D(:,nbs_neighbor+1)>threshold,1);
            cluster_boundary_index = find(D(:,nbs_neighbor+1)>threshold);
            road_friction_cluster_boundary = vertcat(road_friction_cluster_boundary,[cluster_grid(cluster_boundary_index,:) cluster_friction(cluster_boundary_index)] );
            boundary_index =  vertcat(boundary_index,cluster_index(cluster_boundary_index));
            clear D
            
            [row,col] = ind2sub(sz,ind)
        end
    elseif method_boundary ==2
        % step 1 : find the super pixel using the k-means cluster results
        Label_matrix = zeros(size(FrictionGrid_ref.friction)); % cluster matrix
        for i_cluster=1:nbs_cluster
            
            cluster_index = find(cluster_idx == i_cluster);
            %         [cluster_sub_row,cluster_sub_col] = ind2sub(size(FrictionGrid_ref.friction),cluster_index); % cluster matrix subscripts
            
            cluster_friction = mean(FrictionGrid_ref.friction(cluster_index),'all'); %value of each cluster
            
            Label_matrix(cluster_index) = cluster_friction; % assign values to each cluster
            
        end
        % step 2 : find the boundaries using the boundarymask functon
        
        Label_matrix_pad = padarray(Label_matrix,[1 1],0,'both');
        
        boundary_pad = boundarymask(Label_matrix_pad);
        boundary = boundary_pad(2:end-1,2:end-1);
        
        figure(4447)
        %     imshow(label2rgb(L))
        clf
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            Label_matrix,'CDataMapping','scaled')
        %     clf
        %     im = image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        %         [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        %         labeloverlay(FrictionGrid_ref.friction,boundary,'Transparency',0));
        
        figure(4448) % original and cluster data
        clf
        scatter3(reshape(FrictionGrid_ref.position_s,[],1),reshape(FrictionGrid_ref.position_t,[],1),reshape(FrictionGrid_ref.friction,[],1),10,reshape(FrictionGrid_ref.friction,[],1),'.');
        hold on
        scatter3(reshape(FrictionGrid_ref.position_s,[],1),reshape(FrictionGrid_ref.position_t,[],1),reshape(boundary,[],1),20,reshape(boundary,[],1),'r.');
        
        view(0,90)
        zlim([0 1])
        %     xlim([-5 500])
        xlabel('s[m]')
        ylabel('t[m]')
        
        
    elseif method_boundary ==3
        [L,N] = superpixels(FrictionGrid_ref.friction,10); % 2-D superpixel oversegmentation of images
        
        mask = boundarymask(L);
        
        figure(4447)
        %     imshow(label2rgb(L))
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            L,'CDataMapping','scaled')
        
        figure(4448)
        %     imshow(label2rgb(L))
        imshow(mask)
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            mask,'CDataMapping','scaled')
        figure(4449)
        %     im = image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        %         [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        %         labeloverlay(FrictionGrid_ref.friction,mask,'Transparency',0));
        %
        scatter3(FrictionGrid_ref.position_s,FrictionGrid_ref.position_t,mask,10, mask,'.');
        
        hold on
    else
        %     nbs_cluster = 3;
        %     nbs_friction_grid = 5;
        lower_friction_bound = min(FrictionGrid_ref.friction,[],'all');
        upper_friction_bound = max(FrictionGrid_ref.friction,[],'all');
        friction_interval = 0.1
        nbs_friction_grid = round((upper_friction_bound-lower_friction_bound)/friction_interval);
        %     [counts,edges] = histcounts(FrictionGrid_ref.friction,...
        %         nbs_friction_grid,'BinLimits',[lower_friction_bound,upper_friction_bound]); % Calculate a 101-bin histogram for the image.
        [counts,edges] = histcounts(FrictionGrid_ref.friction,...
            round(1/friction_interval),'BinLimits',[0,1.0],'Normalization', 'probability' ); % Calculate a 101-bin histogram for the image.
        
        % histtogram of friction data
        h_fig = figure(15567);
        set(h_fig,'Name','histogram of friction data');
        histogram(reshape(FrictionGrid_ref.friction,[],1),edges)
        %     stem(edges,counts)
        
        % execute the k-means cluster
        nbs_cluster = length(counts(counts>0.01))+1;
        
        data = [reshape(FrictionGrid_ref.position_s,[],1)/max(FrictionGrid_ref.position_s(1,:)),...
            0.5+reshape(FrictionGrid_ref.position_t,[],1)/(2*max(FrictionGrid_ref.position_t(1,:))),...
            30*reshape(FrictionGrid_ref.friction,[],1)];
        [cluster_idx,cluster_centroid_locations,sumDist] = kmeans(data,nbs_cluster,'Replicates',3,...
            'Distance','cityblock','Display','final'); % sqeuclidean,  cityblock
        figure(5559)
        clf
        hold on
        box on
        grid on
        view(0,90)
        zlim([0 1])
        %     xlim([-5 500])
        xlabel('s[m]')
        ylabel('t[m]')
        for i_cluster=1:nbs_cluster
            cluster_index = find(cluster_idx == i_cluster);
            scatter3(FrictionGrid_ref.position_s(cluster_index),FrictionGrid_ref.position_t(cluster_index),FrictionGrid_ref.friction(cluster_index),10,'.');
        end
        
        % step 1 : find the super pixel using the k-means cluster results
        Label_matrix = zeros(size(FrictionGrid_ref.friction)); % cluster matrix
        for i_cluster=1:nbs_cluster
            
            cluster_index = find(cluster_idx == i_cluster);
            %         [cluster_sub_row,cluster_sub_col] = ind2sub(size(FrictionGrid_ref.friction),cluster_index); % cluster matrix subscripts
            
            cluster_friction = mean(FrictionGrid_ref.friction(cluster_index),'all'); %value of each cluster
            
            Label_matrix(cluster_index) = cluster_friction; % assign values to each cluster
            
            sub_cluster = zeros(size(FrictionGrid_ref.friction));
            sub_cluster(cluster_index) = Label_matrix(cluster_index);
            [L,n] = bwlabel(sub_cluster,4);
            for i=1:n
                index_subcluster = find(L==i);
                if length(index_subcluster)<50 % find samll sub cluster
                    Label_matrix(index_subcluster) = NaN;
                end
            end
            
        end
        % fill the samll holes uing nearest data
        Label_matrix = fillmissing(Label_matrix,'nearest',2,'EndValues','nearest'); % by column
        
        
        %%step 2 : find the boundaries using the boundarymask functon
        Label_matrix(Label_matrix==0) =1;
        Label_matrix_pad = padarray(Label_matrix,[1 1],0,'both');
        
        boundary_pad = boundarymask(Label_matrix_pad);
        %     boundary_pad = edge(Label_matrix_pad,'Canny',0.002);
        
        FrictionGrid_ref.boundary = double(boundary_pad(2:end-1,2:end-1));
        FrictionGrid_ref.boundary(FrictionGrid_ref.boundary==0)= NaN;
        %%
        
        mid_results = diff(diff(FrictionGrid_ref.boundary,1,1),1,2);
        
        true_cluster_boundary = [[mid_results ones(size(mid_results,1),1)];ones(1,size(FrictionGrid_ref.boundary,2))] ;
        
        true_cluster_boundary(:,1)=1;
        true_cluster_boundary(1,:)=1;
        true_cluster_boundary(true_cluster_boundary==0)=1;
        
        
        %%  find critical_boundary = [ind_x,ind_y,]
        
        friction_diff_col = [zeros(1,size(FrictionGrid_ref.friction,2)) ; abs(diff(FrictionGrid_ref.friction,1,1))]; %columns 
        
        friction_diff_row = [zeros(size(FrictionGrid_ref.friction,1),1) abs(diff(FrictionGrid_ref.friction,1,2))];
        
        critical_boundary = friction_diff_col + friction_diff_row;
        critical_boundary(critical_boundary<0.1) = 0;
%         critical_boundary(critical_boundary>=0.1) = 1;
        
        
        critical_boundary_true_position(:,1) = FrictionGrid_ref.position_s(logical(critical_boundary));
        critical_boundary_true_position(:,2) = FrictionGrid_ref.position_t(logical(critical_boundary));
        
        critical_boundary(critical_boundary==0) = NaN;
        
        h_fig = figure(44774);
        set(h_fig,'Name','cluster results image');
        
        width=540;%
        height=300;%
        right=100;%
        bottom=400;%
        set(gcf,'position',[right,bottom,width,height])
        clf
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            FrictionGrid_ref.friction,'CDataMapping','scaled')
        hold on
        s = surf(FrictionGrid_ref.position_s,FrictionGrid_ref.position_t,critical_boundary,'EdgeColor','r','FaceColor','none'); %,'FaceAlpha',0.1
        
        index_ref = 39;
        plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
        index_lane_center_lateral=[20 58];
        for i= 1:length(index_lane_center_lateral)
            plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
        end
        
%         scatter3(critical_boundary_true_position(:,1),critical_boundary_true_position(:,2),ones(size(critical_boundary_true_position(:,2))),'go');
        
        set(gca,'YDir','normal')
        colormap parula
        colorbar
        % shading interp
        shading flat
        lighting phong
        view(0,90)
        zlim([0 1])
        caxis([0.1,0.9])
        s.EdgeColor = 'k';
        s.LineWidth = 3;
        % axis equal
        box on
        xlabel('Station[m]')
        ylabel('Transverse[m]')
        axes_handle = gca;
        set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+3);
        
        %%
        figure(44473)
        %     imshow(label2rgb(L))
        clf
        %     Label_matrix(Label_matrix==1) =NaN;
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            Label_matrix,'CDataMapping','scaled')
        %     clf
        %     im = image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        %         [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        %         labeloverlay(FrictionGrid_ref.friction,boundary,'Transparency',0));
        %
        figure(44474)
        %     imshow(label2rgb(L))
        clf
        surf(FrictionGrid_ref.position_s,FrictionGrid_ref.position_t,Label_matrix);
        hold on
        index_ref = 39;
        plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
        index_lane_center_lateral=[20 58];
        for i= 1:length(index_lane_center_lateral)
            plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
        end
        % axis tight
        % grid on
        colormap parula
        colorbar
        % shading interp
        shading flat
        lighting phong
        view(0,90)
        zlim([0 1])
        caxis([0.1,0.9])
        % axis equal
        box on
        xlabel('station [m]')
        ylabel('traversal [m]')
        zlabel('friction\_coeficient')
        
        figure(44482) % original and cluster data
        clf
        surf(FrictionGrid_ref.position_s,FrictionGrid_ref.position_t,Label_matrix);
        %     scatter3(reshape(FrictionGrid_measure.position_s,[],1),reshape(FrictionGrid_measure.position_t,[],1),reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),10,reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),'.');
        hold on
       
        scatter3(reshape(FrictionGrid_ref.position_s,[],1),reshape(FrictionGrid_ref.position_t,[],1),reshape(FrictionGrid_ref.boundary,[],1),20,reshape(FrictionGrid_ref.boundary,[],1),'r.');
        shading flat
        view(0,90)
        zlim([0 1])
        colorbar
        caxis([0.1,0.9])
        %     xlim([-5 500])
        xlabel('s[m]')
        ylabel('t[m]')
        box on
        nbs_boundariy_grid = length(find(FrictionGrid_ref.boundary ==1))
        
        sub_cluster_label = ones(size(FrictionGrid_ref.friction));
        sub_cluster_label(FrictionGrid_ref.boundary ==1) = 0;
        [~,nbs_subcluster] = bwlabel(sub_cluster_label,4)
        
        %% step 3: split each sub_cluster into regular rectangular
        
        sub_cluster_label = zeros(size(Label_matrix));
        friction_cluster = unique(Label_matrix);
        nbs_subcluster = length(friction_cluster);
        for i=1:nbs_subcluster
            sub_cluster_label(Label_matrix==friction_cluster(i))=i;
        end
        % sub_cluster_label(sub_cluster_label==3)=1;
        %
        rect_row = []; % create a recttangular list for each row
        
        for i_cluster=1:nbs_subcluster
            
            [cluster_row,cluster_col] = find(sub_cluster_label == i_cluster);
            
            ind_cluster_row = unique(cluster_row); % the number of rows in the cluster
            
            % find the connected rectangular block in each row
            for i_row = 1:length(ind_cluster_row)
                cluster_cols_irow = sort(cluster_col(cluster_row==ind_cluster_row(i_row))); % start from the first row
                ind_edge = find(diff(cluster_cols_irow)>1);
                ind_edges = [0 ind_edge length(cluster_cols_irow)];
                for i_edge= 1:length(ind_edges)-1
                    rect_row = [rect_row; [ind_cluster_row(i_row) cluster_cols_irow(ind_edges(i_edge)+1) ind_cluster_row(i_row) cluster_cols_irow(ind_edges(i_edge+1)) ...
                        friction_cluster(i_cluster) 1*(cluster_cols_irow(ind_edges(i_edge+1))-cluster_cols_irow(ind_edges(i_edge)+1)+1)]]; % rectangular of each row
                end
            end
            
            % merge the rectangular blocks in each row
            
            rect_current = rect_row(rect_row(:,1)==ind_cluster_row(1),:);
            %         rectCounter = size(rect_row,1);
            rect_merge=[];
            for i_row = 2:length(ind_cluster_row)
                
                rect_irow_next = rect_row(rect_row(:,1)==ind_cluster_row(i_row),:);
                
                for i = 1: size(rect_current,1)
                    for j = 1: size(rect_irow_next,1)
                        
                        if (rect_current(i,2)==rect_irow_next(j,2)) && (rect_current(i,4)==rect_irow_next(j,4))&& (rect_current(i,1)==rect_irow_next(j,3)-1)
                            
                            rect_merge = [rect_merge;[rect_irow_next(j,1) rect_irow_next(j,2) rect_current(i,3) rect_current(i,4) ...
                                rect_current(i,5) rect_current(i,6)+rect_irow_next(j,6)]];
                            rect_irow_next(j,:)= [NaN NaN NaN NaN NaN NaN]; % remove original
                            rect_current(i,:)= [NaN NaN NaN NaN NaN NaN]; % remove original
                        end
                        
                    end
                end
                rect_current = [rect_merge;rect_current ;rect_irow_next];
                rect_current(all(isnan(rect_current),2),:) = [];
                rect_merge = [];
                
            end
            
        end
        rect_list=rect_current;
        nbs_rect = length(rect_list)
        % lower left corner,width , height.
        rect_list(:,3) = rect_current(:,4)-rect_current(:,2);
        rect_list(:,4) = rect_current(:,1)-rect_current(:,3);
        %% Plot the rectangles
        figure(55582)
        
        clf
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            flipud(Label_matrix),'CDataMapping','scaled')
        set(gca,'YDir','normal')
        for i = 1:size(rect_list,1)
            rectangle('position',[FrictionGrid_ref.position_s(1,rect_list(i,2))-0.05 FrictionGrid_ref.position_t(rect_list(i,1),1)-0.05 0.1*rect_list(i,3:4)+[0.1 0.1]],'EdgeColor','r','LineWidth',2);
        end
        
        box on
        caxis([0.1,0.9])
        
        figure(55583)
        colors = parula(100);
        cmap = [(linspace(0.1,0.9,100))' colors];
        for i = 1:size(rect_list,1)
            p = rectangle('position',[FrictionGrid_ref.position_s(1,rect_list(i,2))-0.05 FrictionGrid_ref.position_t(rect_list(i,1),1)-0.05 0.1*rect_list(i,3:4)+[0.1 0.1]]);
            set(p,'edgecolor',[0.5 0.5 0.5],'linewidth',2,'facecolor',interp1(cmap(:,1),cmap(:,2:4),rect_list(i,5)));
        end
        
        colormap(colors);
        colorbar;
        
        caxis([0.1,0.9])
        
    end
    
    
    
    
    %%
    flag_cluster_check =0;
    if flag_cluster_check
        
        
        % elbow of cluster
        %     h_fig = figure(17265);
        %     set(h_fig,'Name','elbow');
        %     hold on
        %     Total_sumDist = zeros(10,1);
        %     for i=1:10
        %         [~,~,sumDist] = kmeans(rictionGrid_ref.friction_vector,i,'Replicates',1,'Display','final');
        %         Total_sumDist(i) = sum(sumDist);
        %     end
        %     plot((1:10),Total_sumDist,'o-','Color','b')
        %     grid on
        %     xlabel('Number of clusters')
        %     ylabel('Total sum of distance')
        
        %%original data and cluster data
        %     h_fig = figure(9849);
        %     set(h_fig,'Name','friction cluster');
        %     clf;
        %     hold on
        %     % true friction
        %      scatter3(FrictionGrid_ref.position_x_vector,FrictionGrid_ref.position_y_vector,FrictionGrid_ref.friction_vector,10, FrictionGrid_ref.friction_vector,'.');
        %     % plot(friction_grid(1:1:end,1),friction_grid(1:1:end,2),'g.','MarkerSize',10)
        %     for i_cluster =1:nbs_cluster
        %         %plot(friction_grid(idx==1,1),friction_grid(idx==1,2),'r.','MarkerSize',10)
        %         cluster_index = find(cluster_idx == i_cluster);
        %         cluster_grid = [FrictionGrid_ref.position_x_vector(cluster_index) FrictionGrid_ref.position_y_vector(cluster_index)];
        %         cluster_friction = FrictionGrid_ref.friction_vector(cluster_index);
        %         scatter3(cluster_grid(:,1),cluster_grid(:,2),cluster_friction,'r.');
        %     end
        %    %legend('Ture Road Friction Distribution', 'Estimated Road Friction Distribution')
        %     grid on
        %     xlabel('xEast [m]')
        %     ylabel('yNorth [m]')
        %     zlabel('friction\_coeficient')
        %     %axis equal
        %     view(0,90)
        %     zlim([0 1])
        %     colormap(jet); % used for ACC paper
        %     colormap(parula);% used for IVSG friction reprsentation,  parula
        %     %     colormap(prism);%
        %     colorbar
    end
    
end
%% step 5: aggregate the measurememt data to friction grid

% initialize aggregated data
FrictionGrid_measure.position_x = FrictionGrid_ref.position_x;
FrictionGrid_measure.position_y = FrictionGrid_ref.position_y;
FrictionGrid_measure.position_s = FrictionGrid_ref.position_s;
FrictionGrid_measure.position_t = FrictionGrid_ref.position_t;
FrictionGrid_measure.friction_true = FrictionGrid_ref.friction;
FrictionGrid_measure.friction_nbs = zeros(size(FrictionGrid_measure.position_x)); % number of measurement points at each grid
FrictionGrid_measure.friction_mean = NaN(size(FrictionGrid_measure.position_x)); % mean value of noisy measurement at each grid
FrictionGrid_measure.friction_mean_true= NaN(size(FrictionGrid_measure.position_x)); % mean value of no-noise measurement at each grid
FrictionGrid_measure.friction_std = NaN(size(FrictionGrid_measure.position_x)); % std value of noisy measurement at each grid
FrictionGrid_measure.friction_confidenceInterval_upper = NaN(size(FrictionGrid_measure.position_x)); % friction_confidenceInterval_upper of mean value of noisy measurement at each grid
FrictionGrid_measure.friction_confidenceInterval_lower = NaN(size(FrictionGrid_measure.position_x)); % friction_confidenceInterval_lower of mean value of noisy measurement at each grid

%% step 4.1 find the neasrest grid
road_grid = [reshape(FrictionGrid_ref.position_x,[],1), reshape(FrictionGrid_ref.position_y,[],1)];
% friction_coefficient_true = FrictionGrid_ref.friction_vector;
Md_road_grid = KDTreeSearcher(road_grid); % build KDTreeSearcher model for all lane grid data

friction_measurement_position = [friction_measurement_processed.east_noisy, friction_measurement_processed.north_noisy];
friction_measurement_value = friction_measurement_processed.friction_measurement_noisy;
%%find nearest grid for each measured data point
tic
% this is fast: 3.13 seconds
[Id_grid,~] = knnsearch(Md_road_grid,friction_measurement_position); % Id_grid contain the nearest grid id for each measuresmnt point
toc
%%step 4.2: find the id of measurement data which are located in the same grid

Id_grid_measure = [Id_grid,(1:size(Id_grid))']; % first is the id of grid, second column is the id of each measuresmnt point located in the corresponding grid
Id_grid_measure_sort = sortrows(Id_grid_measure); % sort rows by the first column

diff_Id_grid = diff(Id_grid_measure_sort(:,1));
bound_index = [0; find(diff_Id_grid >0);length(Id_grid_measure_sort)];

%%step 4.3: average the measured friction at each grid

tic %this taeks a while: 60 seconds
measurement_grid_id = unique(Id_grid_measure_sort(:,1),'stable'); % the grid which has data
measurement_grid_friction_mean = zeros(size(measurement_grid_id));
measurement_grid_nbs = zeros(size(measurement_grid_id)); % how many measurement data in the grid
measurement_grid_friction_std = zeros(size(measurement_grid_id)); % standard deviation of measurement data in the grid
friction_confidenceInterval_lower= zeros(size(measurement_grid_id));
friction_confidenceInterval_upper = zeros(size(measurement_grid_id));

for i_grid = 1:length(measurement_grid_id)
    id_measure=Id_grid_measure_sort(bound_index(i_grid)+1:bound_index(i_grid+1),2); % very slow, avoid to use it
    measurement_grid_friction_mean(i_grid) = mean(friction_measurement_value(id_measure));
    measurement_grid_nbs(i_grid) = length(id_measure);
    measurement_grid_friction_std(i_grid) = std(friction_measurement_value(id_measure));
    %     x_mean = mean(x)
    x_SEM = std(friction_measurement_value(id_measure))/sqrt(length(id_measure)); % ¡®Standard Error Of The Mean¡¯
    
    CI95 = tinv([0.025 0.975], length(id_measure)-1);% Calculate 95% Probability Intervals Of t-Distribution
    
    x_CI95 = mean(friction_measurement_value(id_measure)) + bsxfun(@times, x_SEM, CI95(:)); % slow            % Calculate 95% Confidence Intervals Of All Experiments At Each Value Of ¡®x¡¯
    friction_confidenceInterval_lower(i_grid) = x_CI95(1);
    friction_confidenceInterval_upper(i_grid) = x_CI95(2);
    
end
toc
%
Signal_noise_ratio = 10; %unit: dB
% measurement_grid_nbs(measurement_grid_nbs>500) = 500; % normalization
FrictionGrid_measure.friction_nbs(measurement_grid_id) = measurement_grid_nbs;
FrictionGrid_measure.friction_mean(measurement_grid_id) = measurement_grid_friction_mean;
FrictionGrid_measure.friction_std(measurement_grid_id) = measurement_grid_friction_std;
FrictionGrid_measure.friction_mean_true(measurement_grid_id) = FrictionGrid_measure.friction_true(measurement_grid_id);
FrictionGrid_measure.friction_mean_true_noise = awgn(FrictionGrid_measure.friction_mean_true,Signal_noise_ratio,'measured'); % Add white Gaussian noise to signal
FrictionGrid_measure.friction_confidenceInterval_lower(measurement_grid_id) = friction_confidenceInterval_lower;
FrictionGrid_measure.friction_confidenceInterval_upper(measurement_grid_id) = friction_confidenceInterval_upper;
FrictionGrid_measure.friction_confidenceInterval = FrictionGrid_measure.friction_confidenceInterval_upper -FrictionGrid_measure.friction_confidenceInterval_lower;
FrictionGrid_measure.friction_confidenceInterval(FrictionGrid_measure.friction_confidenceInterval>1) = 1;
% FrictionGrid_measure.friction_confidenceInterval = FrictionGrid_measure.friction_confidenceInterval/max(FrictionGrid_measure.friction_confidenceInterval,[],'all');
% FrictionGrid_measure.friction_confidenceInterval_fillmiss = fillmissing(FrictionGrid_measure.friction_confidenceInterval,'constant',max(FrictionGrid_measure.friction_confidenceInterval,[],'all'));
FrictionGrid_measure.friction_confidenceInterval_fillmiss = fillmissing(FrictionGrid_measure.friction_confidenceInterval,'constant',1);

% combine to get the grid which are measured,[id, coordinates, measured friction values]

% measurement_grid = [measurement_grid_id,road_grid(measurement_grid_id,:),measurement_grid_friction_mean,measurement_grid_nbs]; % road grid coordinate which are measaured by the vehicle
%%
figure(23)
clf
histogram(measurement_grid_nbs)

figure(24)
clf
plot(measurement_grid_nbs,'b.');

%% surf plot
h_fig = figure(25);
set(h_fig,'Name','the number of raw data in each grid cell');
width=540;%
height=300;%
right=100;%
bottom=400;%
set(gcf,'position',[right,bottom,width,height])
clf
surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,FrictionGrid_measure.friction_nbs,FrictionGrid_measure.friction_nbs)
hold on
% scatter3(FrictionGrid_ref.position_x_trans(boundary_index),FrictionGrid_ref.position_y_trans(boundary_index),FrictionGrid_ref.friction_trans(boundary_index),'r.');
index_ref = 39;
plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),...
    ones(size(FrictionGrid_ref.position_t(index_ref,:))),'w--','LineWidth',2)
index_lane_center_lateral=[20 58];
for i= 1:length(index_lane_center_lateral)
    plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),...
        ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'w-.','LineWidth',1.5)
end
% legend('Number of Measurement Points','True Friction Boundary')
% axis tight
% % grid on

cMap = parula(256);
dataMax = 500;
dataMin = 0;
centerPoint = 50;
scalingIntensity = 5;
% perform some operations to create your colormap
x = 1:length(cMap);
x = x - (centerPoint-dataMin)*length(x)/(dataMax-dataMin);
x = scalingIntensity * x/max(abs(x));

% x = sign(x).* exp(1.2*abs(x));
x = (x).* exp(1.5*abs(x));
x = x - min(x);
x = x*511/max(x)+1;
% figure(1111)
% plot(x)

newMap = interp1(x, cMap, 1:512);

% colormap hot
% colormap cool
% colormap parula
colormap(newMap)
caxis([0,500])
% colormap(mymap)
colorbar

% shading interp
shading flat
lighting phong
view(0,90)
% zlim([0 100])
xlim([-1,500])
xlabel('Station[m]')
ylabel('Transverse[m]')
zlabel('numbers of measurement','interpreter','latex','fontsize',14);
% legend('$steering$','location','best','interpreter','latex','fontsize',14);
box on
axes_handle = gca;
set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+2);

%
if flag.doDebug
    figure(251)
    clf
    surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,1000000*FrictionGrid_measure.friction_nbs/length(Id_grid_measure),1000000*FrictionGrid_measure.friction_nbs/length(Id_grid_measure))
    hold on
    % scatter3(FrictionGrid_ref.position_x_trans(boundary_index),FrictionGrid_ref.position_y_trans(boundary_index),FrictionGrid_ref.friction_trans(boundary_index),'r.');
    index_ref = 39;
    plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),...
        ones(size(FrictionGrid_ref.position_t(index_ref,:))),'w--','LineWidth',2)
    index_lane_center_lateral=[20 58];
    for i= 1:length(index_lane_center_lateral)
        plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),...
            ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'w-.','LineWidth',1.5)
    end
    % legend('Number of Measurement Points','True Friction Boundary')
    % axis tight
    % % grid on
    
    cMap = parula(256);
    dataMax = 200;
    dataMin = 0;
    centerPoint = 20;
    scalingIntensity = 5;
    % perform some operations to create your colormap
    x = 1:length(cMap);
    x = x - (centerPoint-dataMin)*length(x)/(dataMax-dataMin);
    x = scalingIntensity * x/max(abs(x));
    
    % x = sign(x).* exp(1.2*abs(x));
    x = (x).* exp(1.5*abs(x));
    x = x - min(x);
    x = x*511/max(x)+1;
    % figure(1111)
    % plot(x)
    
    newMap = interp1(x, cMap, 1:512);
    
    % colormap hot
    % colormap cool
    % colormap parula
    colormap(newMap)
    caxis([0,200])
    colorbar
    
    % shading interp
    shading flat
    lighting phong
    view(0,90)
    % zlim([0 100])
    xlim([-1,500])
    xlabel('station[m]','interpreter','latex','fontsize',14);
    ylabel('traversal[m]','interpreter','latex','fontsize',14);
    zlabel('millionth','interpreter','latex','fontsize',14);
    % legend('$steering$','location','best','interpreter','latex','fontsize',14);
    box on
end
%%

h_fig = figure(984727);
set(h_fig,'Name','the mean friction coefficient value in each grid cell');
width=540;%
height=300;%
right=100;%
bottom=400;%
set(gcf,'position',[right,bottom,width,height])
clf

hold on

surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,FrictionGrid_measure.friction_mean);

index_ref = 39;
plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
index_lane_center_lateral=[20 58];
for i= 1:length(index_lane_center_lateral)
    plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
end

pMap = parula;
myMap = [0.65 0.65 0.65; pMap]; % use gray color for empty value
colormap(myMap)
colorbar
% shading interp
shading flat
lighting phong
view(0,90)
zlim([0 1])
caxis([0.1,0.9])
% axis equal
box on
xlim([-1,500])
xlabel('Station[m]')
ylabel('Transverse[m]')
zlabel('numbers of measurement','interpreter','latex','fontsize',14);
% legend('$steering$','location','best','interpreter','latex','fontsize',14);
box on
axes_handle = gca;
set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+2);

zlabel('friction\_coeficient')

% % empty dark
%  FrictionGrid_measure.empty = ones(size(FrictionGrid_measure.friction_mean));
%  FrictionGrid_measure.empty(isnan(FrictionGrid_measure.friction_mean)) = 0;
% %  [row,col]= find(FrictionGrid_measure.empty == 0);
% %  FrictionGrid_measure.empty(row+1,col+1) =0; 
%  surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,FrictionGrid_measure.empty,...
%      'EdgeColor','none','FaceColor','k');

%%
h_fig = figure(984728);
set(h_fig,'Name','the mean friction coefficient value in each grid cell');
width=540;%
height=300;%
right=100;%
bottom=400;%
set(gcf,'position',[right,bottom,width,height])

clf
hold on
% image regard NaN as zero
image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
    [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
    FrictionGrid_measure.friction_mean,'CDataMapping','scaled')
%surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,FrictionGrid_measure.friction_mean);

index_ref = 39;
plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
index_lane_center_lateral=[20 58];
for i= 1:length(index_lane_center_lateral)
    plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
end
% axis tight
% grid on
pMap = parula;
myMap = [0.65 0.65 0.65; pMap]; % use gray color for empty value

colormap(myMap)
cb=colorbar;
cb.Label.String = 'fiction coeficient';
cb.FontName= 'Times New Roman';
cb.FontSize= Axis_label_FontSize+2;
cb.Limits= [0 1];
% shading interp
shading flat
lighting phong
view(0,90)
zlim([0 1])
caxis([0,1])
% axis equal
box on
xlim([-1,500])
xlabel('Station[m]')
ylabel('Transverse[m]')
zlabel('numbers of measurement','interpreter','latex','fontsize',14);
% legend('$steering$','location','best','interpreter','latex','fontsize',14);

axes_handle = gca;
set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+2);

zlabel('friction\_coeficient')

%  FrictionGrid_measure.empty = NaN(size(FrictionGrid_measure.friction_mean));
%  FrictionGrid_measure.empty(isnan(FrictionGrid_measure.friction_mean)) = 0;
%  find(FrictionGrid_measure.empty == 0);
%  surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,FrictionGrid_measure.empty,...
%      'EdgeColor','none','FaceColor',[0.5 0.5 0.5]);

% im= image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
%     [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
%     FrictionGrid_measure.empty,'CDataMapping','scaled');
% im.AlphaData = 0.1;
%%
h_fig = figure(26);
set(h_fig,'Name','the confidence interval value in each grid cell');
    clf
    surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,FrictionGrid_measure.friction_confidenceInterval,FrictionGrid_measure.friction_confidenceInterval)
    hold on
    % scatter3(FrictionGrid_ref.position_x_trans(boundary_index),FrictionGrid_ref.position_y_trans(boundary_index),FrictionGrid_ref.friction_trans(boundary_index),'r.');
    index_ref = 39;
    plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
    index_lane_center_lateral=[20 58];
    for i= 1:length(index_lane_center_lateral)
        plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
    end
    % legend('Magnitude of 95% confidence Interval','True Friction Boundary')
    % axis tight
    cMap = parula(256);
    % cMap = gray(256);
    dataMax = 0.5;
    dataMin = 0;
    centerPoint = 0.05;
    scalingIntensity = 0.7;
    % perform some operations to create your colormap
    x = 1:length(cMap);
    x = x - (centerPoint-dataMin)*length(x)/(dataMax-dataMin);
    x = scalingIntensity * x/max(abs(x));
    
    % x = sign(x).* exp(1.2*abs(x));
    x = (x).* exp(4*abs(x));
    x = x - min(x);
    x = x*511/max(x)+1;
    % figure(1111)
    % plot(x)
    
    newMap = interp1(x, cMap, 1:512);
    
    colormap(newMap)
    % caxis([0,200])
    colorbar
    
    % shading interp
    shading flat
    lighting phong
    view(0,90)
    % zlim([0 100])
    xlabel('station[m]','interpreter','latex','fontsize',14);
    ylabel('traversal[m]','interpreter','latex','fontsize',14);
    zlabel('Confidence Interval Magnitude','interpreter','latex','fontsize',14);
    % axis equal
    box on
    
%%
if flag.doDebug
    
    
    %%plot the true friction and aggregated measured results
    
    % scatter plot
    h_fig = figure(9845);
    set(h_fig,'Name','true friction and measurement average');
    clf
    hold on
    % true
    %  scatter3(frictionGrid_table_calibration.position_x -100,frictionGrid_table_calibration.position_y + 50,frictionGrid_table_calibration.friction_coefficient,10, frictionGrid_table_calibration.friction_coefficient,'.');
    % scatter3(frictionGrid_table_calibration.position_x(1:100:end)-offset_x,frictionGrid_table_calibration.position_y(1:100:end)+offset_y,frictionGrid_table_calibration.friction_coefficient(1:100:end),10, frictionGrid_table_calibration.friction_coefficient(1:100:end),'.');
    ss = scatter3(road_grid(1:1:end,1)-100,road_grid(1:1:end,2),FrictionGrid_ref.friction(1:1:end),10, 1*FrictionGrid_ref.friction(1:1:end),'.');
    ss.MarkerEdgeAlpha = 0.2;
    ss.MarkerFaceAlpha  = 0.2;
    % s = surf(FrictionGrid_ref.position_x,FrictionGrid_ref.position_y,FrictionGrid_ref.friction,'FaceColor' ,'none')
    % s.FaceColor = 'none';
    hold on
    
    % axis tight
    % grid on
    
    % shading flat
    % lighting phong
    
    % measurement
    scatter3(FrictionGrid_measure.position_x(1:end),FrictionGrid_measure.position_y(1:end),FrictionGrid_measure.friction_mean(1:end),10, FrictionGrid_measure.friction_mean(1:end),'.');
    
    % scatter3(measurement_grid(:,2),measurement_grid(:,3),measurement_grid(:,4),10, measurement_grid(:,4),'*' )
    colormap(jet);% used for IVSG friction reprsentation,  parula
    colormap(parula);% used for IVSG friction reprsentation,  parula
    colorbar
    %legend('Ture Road Friction Distribution', 'Estimated Road Friction Distribution')
    grid on
    xlabel('xEast [m]')
    ylabel('yNorth [m]')
    zlabel('friction\_coeficient')
    %axis equal
    view(0,90)
    zlim([0 1])
    % axis equal
    box on
    %%
    % surf plot
    h_fig = figure(9846);
    set(h_fig,'Name','true friction and measurement average');
    clf
    hold on
    % true
    surf(FrictionGrid_measure.position_x-100,FrictionGrid_measure.position_y+50,FrictionGrid_measure.friction_true);
    
    % measurement
    surf(FrictionGrid_measure.position_x,FrictionGrid_measure.position_y,FrictionGrid_measure.friction_mean);
    % % shading interp
    shading flat
    
    colormap(jet);% used for IVSG friction reprsentation,  parula
    colormap(parula);% used for IVSG friction reprsentation,  parula
    colorbar
    %legend('Ture Road Friction Distribution', 'Estimated Road Friction Distribution')
    grid on
    xlabel('xEast [m]')
    ylabel('yNorth [m]')
    zlabel('friction\_coeficient')
    %axis equal
    view(0,90)
    zlim([0 1])
    % axis equal
    box on
    
    %%
    figure(9847)
    clf
    subplot(2,1,1)
    surf(FrictionGrid_ref.position_s,FrictionGrid_ref.position_t,FrictionGrid_ref.friction)
    hold on
    index_ref = 39;
    plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
    index_lane_center_lateral=[20 58];
    for i= 1:length(index_lane_center_lateral)
        plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
    end
    % axis tight
    % grid on
    colormap parula
    colorbar
    % shading interp
    shading flat
    lighting phong
    view(0,90)
    zlim([0 1])
    caxis([0.1,0.9])
    % axis equal
    box on
    xlabel('station [m]')
    ylabel('traversal [m]')
    zlabel('friction\_coeficient')
    
    subplot(2,1,2)
    surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,FrictionGrid_measure.friction_mean);
    hold on
    index_ref = 39;
    plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
    index_lane_center_lateral=[20 58];
    for i= 1:length(index_lane_center_lateral)
        plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
    end
    % axis tight
    % grid on
    colormap parula
    colorbar
    % shading interp
    shading flat
    lighting phong
    view(0,90)
    zlim([0 1])
    caxis([0.1,0.9])
    % axis equal
    box on
    xlabel('station [m]')
    ylabel('traversal [m]')
    zlabel('friction\_coeficient')
end

if flag.doDebug
    %% STEP5: cluster the noisy friction measurement data
    
    % split friction data into groups
    nbs_friction_grid = 10;
    [counts,edges] = histcounts(FrictionGrid_measure.friction_mean,nbs_friction_grid,'BinLimits',[0,1]); % Calculate a 101-bin histogram for the image.
    % execute the k-means cluster
    nbs_cluster = length(counts(counts>1000));
    [cluster_idx,cluster_centroid_locations,sumDist] = kmeans(reshape(FrictionGrid_measure.friction_mean,[],1),nbs_cluster,'Replicates',1,'Display','final');
    
    % histtogram of friction data
    h_fig = figure(15566);
    set(h_fig,'Name','histogram of friction data');
    histogram(reshape(FrictionGrid_measure.friction_mean,[],1),edges)
    %stem(edges,counts)
    %% find boundary of each cluster
    method_boundary = 2;
    if method_boundary ==1 % cluster find the boundaries through nearest neighbor
        threshold = 0.15; %0.145; %meters, the Diagonal length of grid( 0.2 > threshold >0.141)
        nbs_neighbor = 8 ; % the number of neighbor nodes for each innner grid
        road_friction_cluster_boundary = [];
        boundary_index = [];
        for i_cluster=1:nbs_cluster
            cluster_index = find(cluster_idx == i_cluster);
            cluster_grid = [FrictionGrid_ref.position_x(cluster_index) FrictionGrid_ref.position_y(cluster_index)];
            cluster_friction = FrictionGrid_ref.friction(cluster_index);
            Md_cluster_grid = KDTreeSearcher(cluster_grid); % Mdl is a KDTreeSearcher model. By default, the distance metric it uses to search for neighbors is Euclidean distance.
            [~,D] = knnsearch(Md_cluster_grid,cluster_grid,'k',nbs_neighbor+1);
            %boundary_index = Idx(D(:,nbs_neighbor+1)>threshold,1);
            cluster_boundary_index = find(D(:,nbs_neighbor+1)>threshold);
            road_friction_cluster_boundary = vertcat(road_friction_cluster_boundary,[cluster_grid(cluster_boundary_index,:) cluster_friction(cluster_boundary_index)] );
            boundary_index =  vertcat(boundary_index,cluster_index(cluster_boundary_index));
            clear D
            
            [row,col] = ind2sub(sz,ind)
        end
    elseif method_boundary ==2
        % step 1 : find the super pixel using the k-means cluster results
        Label_matrix = zeros(size(FrictionGrid_measure.friction_mean)); % cluster matrix
        for i_cluster=1:nbs_cluster
            
            cluster_index = find(cluster_idx == i_cluster);
            %         [cluster_sub_row,cluster_sub_col] = ind2sub(size(FrictionGrid_ref.friction),cluster_index); % cluster matrix subscripts
            
            cluster_friction = mean(FrictionGrid_measure.friction_mean(cluster_index),'all'); %value of each cluster
            
            Label_matrix(cluster_index) = cluster_friction; % assign values to each cluster
            
        end
        % step 2 : find the boundaries using the boundarymask functon
        Label_matrix(Label_matrix==0) =1;
        Label_matrix_pad = padarray(Label_matrix,[1 1],0,'both');
        
        boundary_pad = boundarymask(Label_matrix_pad);
        boundary = boundary_pad(2:end-1,2:end-1);
        
        figure(44471)
        %     imshow(label2rgb(L))
        clf
        Label_matrix(Label_matrix==1) =NaN;
        image([FrictionGrid_measure.position_s(1,1) FrictionGrid_measure.position_s(1,end)],...
            [FrictionGrid_measure.position_t(1,1) FrictionGrid_measure.position_t(end,1)],...
            Label_matrix,'CDataMapping','scaled')
        %     clf
        %     im = image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        %         [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        %         labeloverlay(FrictionGrid_ref.friction,boundary,'Transparency',0));
        
        figure(44472)
        %     imshow(label2rgb(L))
        clf
        surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,Label_matrix);
        hold on
        index_ref = 39;
        plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
        index_lane_center_lateral=[20 58];
        for i= 1:length(index_lane_center_lateral)
            plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
        end
        % axis tight
        % grid on
        colormap parula
        colorbar
        % shading interp
        shading flat
        lighting phong
        view(0,90)
        zlim([0 1])
        caxis([0.1,0.9])
        % axis equal
        box on
        xlabel('station [m]')
        ylabel('traversal [m]')
        zlabel('friction\_coeficient')
        
        figure(44481) % original and cluster data
        clf
        scatter3(reshape(FrictionGrid_measure.position_s,[],1),reshape(FrictionGrid_measure.position_t,[],1),reshape(FrictionGrid_measure.friction_mean,[],1),10,reshape(FrictionGrid_measure.friction_mean,[],1),'.');
        hold on
        FrictionGrid_measure.boundary = double(boundary);
        FrictionGrid_measure.boundary(FrictionGrid_measure.boundary==0)= NaN;
        scatter3(reshape(FrictionGrid_measure.position_s,[],1),reshape(FrictionGrid_measure.position_t,[],1),reshape(FrictionGrid_measure.boundary,[],1),20,reshape(FrictionGrid_measure.boundary,[],1),'r.');
        
        view(0,90)
        zlim([0 1])
        caxis([0.1,0.9])
        %     xlim([-5 500])
        xlabel('s[m]')
        ylabel('t[m]')
        box on
        nbs_boundariy_grid = length(find(FrictionGrid_measure.boundary ==1))
        
    else
        [L,N] = superpixels(FrictionGrid_ref.friction,10);
        
        mask = boundarymask(L);
        
        figure(4447)
        %     imshow(label2rgb(L))
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            L,'CDataMapping','scaled')
        
        figure(4448)
        %     imshow(label2rgb(L))
        imshow(mask)
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            mask,'CDataMapping','scaled')
        figure(4449)
        %     im = image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        %         [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        %         labeloverlay(FrictionGrid_ref.friction,mask,'Transparency',0));
        %
        scatter3(FrictionGrid_ref.position_s,FrictionGrid_ref.position_t,mask,10, mask,'.');
        
        hold on
    end
    
    %% STEP5.5: cluster the non-noisy friction measurement data
    
    % split friction data into groups
    nbs_friction_grid = 5;
    [counts,edges] = histcounts(FrictionGrid_measure.friction_mean_true,nbs_friction_grid,'BinLimits',[0,1]); % Calculate a 101-bin histogram for the image.
    % execute the k-means cluster
    nbs_cluster = length(counts(counts>500));
    [cluster_idx,cluster_centroid_locations,sumDist] = kmeans(reshape(FrictionGrid_measure.friction_mean_true,[],1),nbs_cluster,'Replicates',1,'Display','final');
    
    % histtogram of friction data
    h_fig = figure(15569);
    set(h_fig,'Name','histogram of friction data');
    histogram(reshape(FrictionGrid_measure.friction_mean_true,[],1),edges)
    %stem(edges,counts)
    %% find boundary of each cluster
    method_boundary = 2;
    if method_boundary ==1 % cluster find the boundaries through nearest neighbor
        threshold = 0.15; %0.145; %meters, the Diagonal length of grid( 0.2 > threshold >0.141)
        nbs_neighbor = 8 ; % the number of neighbor nodes for each innner grid
        road_friction_cluster_boundary = [];
        boundary_index = [];
        for i_cluster=1:nbs_cluster
            cluster_index = find(cluster_idx == i_cluster);
            cluster_grid = [FrictionGrid_ref.position_x(cluster_index) FrictionGrid_ref.position_y(cluster_index)];
            cluster_friction = FrictionGrid_ref.friction(cluster_index);
            Md_cluster_grid = KDTreeSearcher(cluster_grid); % Mdl is a KDTreeSearcher model. By default, the distance metric it uses to search for neighbors is Euclidean distance.
            [~,D] = knnsearch(Md_cluster_grid,cluster_grid,'k',nbs_neighbor+1);
            %boundary_index = Idx(D(:,nbs_neighbor+1)>threshold,1);
            cluster_boundary_index = find(D(:,nbs_neighbor+1)>threshold);
            road_friction_cluster_boundary = vertcat(road_friction_cluster_boundary,[cluster_grid(cluster_boundary_index,:) cluster_friction(cluster_boundary_index)] );
            boundary_index =  vertcat(boundary_index,cluster_index(cluster_boundary_index));
            clear D
            
            [row,col] = ind2sub(sz,ind)
        end
    elseif method_boundary ==2
        % step 1 : find the super pixel using the k-means cluster results
        Label_matrix = zeros(size(FrictionGrid_measure.friction_mean_true)); % cluster matrix
        for i_cluster=1:nbs_cluster
            
            cluster_index = find(cluster_idx == i_cluster);
            %         [cluster_sub_row,cluster_sub_col] = ind2sub(size(FrictionGrid_ref.friction),cluster_index); % cluster matrix subscripts
            
            cluster_friction = mean(FrictionGrid_measure.friction_mean_true(cluster_index),'all'); %value of each cluster
            
            Label_matrix(cluster_index) = cluster_friction; % assign values to each cluster
            
        end
        % step 2 : find the boundaries using the boundarymask functon
        Label_matrix(Label_matrix==0) =1;
        Label_matrix_pad = padarray(Label_matrix,[1 1],0,'both');
        
        boundary_pad = boundarymask(Label_matrix_pad);
        boundary = boundary_pad(2:end-1,2:end-1);
        
        figure(44471)
        clf
        Label_matrix(Label_matrix==1) =NaN;
        image([FrictionGrid_measure.position_s(1,1) FrictionGrid_measure.position_s(1,end)],...
            [FrictionGrid_measure.position_t(1,1) FrictionGrid_measure.position_t(end,1)],...
            Label_matrix,'CDataMapping','scaled')
        %     clf
        %     im = image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        %         [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        %         labeloverlay(FrictionGrid_ref.friction,boundary,'Transparency',0));
        
        figure(44472)
        %     imshow(label2rgb(L))
        clf
        surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,Label_matrix);
        hold on
        index_ref = 39;
        plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
        index_lane_center_lateral=[20 58];
        for i= 1:length(index_lane_center_lateral)
            plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
        end
        % axis tight
        % grid on
        colormap parula
        colorbar
        % shading interp
        shading flat
        lighting phong
        view(0,90)
        zlim([0 1])
        caxis([0.1,0.9])
        % axis equal
        box on
        xlabel('station [m]')
        ylabel('traversal [m]')
        zlabel('friction\_coeficient')
        
        figure(44481) % original and cluster data
        clf
        scatter3(reshape(FrictionGrid_measure.position_s,[],1),reshape(FrictionGrid_measure.position_t,[],1),reshape(FrictionGrid_measure.friction_mean_true,[],1),10,reshape(FrictionGrid_measure.friction_mean_true,[],1),'.');
        hold on
        FrictionGrid_measure.boundary = double(boundary);
        FrictionGrid_measure.boundary(FrictionGrid_measure.boundary==0)= NaN;
        scatter3(reshape(FrictionGrid_measure.position_s,[],1),reshape(FrictionGrid_measure.position_t,[],1),reshape(FrictionGrid_measure.boundary,[],1),20,reshape(FrictionGrid_measure.boundary,[],1),'r.');
        
        view(0,90)
        zlim([0 1])
        caxis([0.1,0.9])
        %     xlim([-5 500])
        xlabel('s[m]')
        ylabel('t[m]')
        box on
        nbs_boundariy_grid = length(find(FrictionGrid_measure.boundary ==1))
        
    else
        [L,N] = superpixels(FrictionGrid_measure.friction_mean_true,10);
        
        mask = boundarymask(L);
        
        figure(4447)
        %     imshow(label2rgb(L))
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            L,'CDataMapping','scaled')
        
        figure(4448)
        %     imshow(label2rgb(L))
        imshow(mask)
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            mask,'CDataMapping','scaled')
        
    end
    
end
%% Step 6 fill missing
FrictionGrid_measure.friction_mean_fillmiss = fillmissing(FrictionGrid_measure.friction_mean,'nearest',1,'EndValues','nearest'); % by column
% FrictionGrid_measure.friction_mean_fillmiss =
% fillmissing(FrictionGrid_measure.friction_mean,'linear',2,'EndValues','nearest');% by row
FrictionGrid_measure.friction_mean_true_fillmiss = fillmissing(FrictionGrid_measure.friction_mean_true,'nearest',1,'EndValues','nearest');
% FrictionGrid_measure.friction_mean_true_noise_fillmiss = fillmissing(FrictionGrid_measure.friction_mean_true_noise,'nearest',1,'EndValues','nearest');
%
FrictionGrid_measure.friction_mean_true_noise_fillmiss = awgn(FrictionGrid_measure.friction_mean_true_fillmiss,40,'measured'); % Add white Gaussian noise to signal
%

%% surf plot of filling results
h_fig = figure(9847101);
set(h_fig,'Name','true friction and measurement average');

width=540;%
height=300;%
right=100;%
bottom=400;%
set(gcf,'position',[right,bottom,width,height])
clf
surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,FrictionGrid_measure.friction_mean_fillmiss);
hold on
index_ref = 39;
plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
index_lane_center_lateral=[20 58];
for i= 1:length(index_lane_center_lateral)
    plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
end
% axis tight
% grid on
colormap parula
colorbar
% shading interp
shading flat
lighting phong
view(0,90)
zlim([0 1])
xlim([-1 500])
caxis([0.1,0.9])
% axis equal
box on
xlabel('Station[m]')
ylabel('Transverse[m]')
% axis equal
axes_handle = gca;
set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+2);
zlabel('friction\_coeficient')
%%
h_fig = figure(261);
set(h_fig,'Name','confidence interval magnitude');

width=540;%
height=300;%
right=100;%
bottom=400;%
set(gcf,'position',[right,bottom,width,height])
clf
surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,FrictionGrid_measure.friction_confidenceInterval_fillmiss,FrictionGrid_measure.friction_confidenceInterval_fillmiss)
hold on
% scatter3(FrictionGrid_ref.position_x_trans(boundary_index),FrictionGrid_ref.position_y_trans(boundary_index),FrictionGrid_ref.friction_trans(boundary_index),'r.');
index_ref = 39;
plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
index_lane_center_lateral=[20 58];
for i= 1:length(index_lane_center_lateral)
    plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
end
% legend('Magnitude of 95% confidence Interval','True Friction Boundary')
% axis tight
cMap = parula(256);
% cMap = gray(256);
dataMax = 0.5;
dataMin = 0;
centerPoint = 0.05;
scalingIntensity = 0.7;
% perform some operations to create your colormap
x = 1:length(cMap);
x = x - (centerPoint-dataMin)*length(x)/(dataMax-dataMin);
x = scalingIntensity * x/max(abs(x));

% x = sign(x).* exp(1.2*abs(x));
x = (x).* exp(4*abs(x));
x = x - min(x);
x = x*511/max(x)+1;
% figure(1111)
% plot(x)

newMap = interp1(x, cMap, 1:512);

colormap(newMap)
% caxis([0,200])
colorbar

% shading interp
shading flat
lighting phong
view(0,90)
zlim([0 100])
xlim([-1 500])
% axis equal
box on
xlabel('Station[m]')
ylabel('Transverse[m]')
% axis equal
axes_handle = gca;
set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+2);
zlabel('friction\_coeficient')
% axis equal

%%
if flag.doDebug
    h_fig = figure(98471);
    set(h_fig,'Name','true friction and measurement average');
    clf
    subplot(2,1,1)
    surf(FrictionGrid_ref.position_s,FrictionGrid_ref.position_t,FrictionGrid_ref.friction)
    hold on
    index_ref = 39;
    plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
    index_lane_center_lateral=[20 58];
    for i= 1:length(index_lane_center_lateral)
        plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
    end
    % axis tight
    % grid on
    colormap parula
    colorbar
    % shading interp
    shading flat
    lighting phong
    view(0,90)
    zlim([0 1])
    caxis([0.1,0.9])
    % axis equal
    box on
    xlabel('station [m]')
    ylabel('traversal [m]')
    zlabel('friction\_coeficient')
    
    subplot(2,1,2)
    surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,FrictionGrid_measure.friction_mean_fillmiss);
    hold on
    index_ref = 39;
    plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
    index_lane_center_lateral=[20 58];
    for i= 1:length(index_lane_center_lateral)
        plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
    end
    % axis tight
    % grid on
    colormap parula
    colorbar
    % shading interp
    shading flat
    lighting phong
    view(0,90)
    zlim([0 1])
    caxis([0.1,0.9])
    % axis equal
    box on
    xlabel('station [m]')
    ylabel('traversal [m]')
    zlabel('friction\_coeficient')
end


%% Step 6: cluster the filling friction measurement data
method_boundary = 4; % 1: using neasrest searching to find the cluster boundaries of  K-means clutering results
% 2: using the boundarymask functon to find the cluster boundaries of  K-means clutering results
% 3: using the boundarymask functon to find the cluster boundaries of superpixels
% 4:(default method) the method used in the RSS paper. K-means to find
% the cluster, then partition each cluster into
% bounding box, also boundarymask is used find the
% cluater.
% 5: the method used by Prof. Beal

if method_boundary ==1 % cluster find the boundaries through nearest neighbor
    % split friction data into groups
    nbs_friction_grid = 10;
    [counts,edges] = histcounts(FrictionGrid_measure.friction_mean_fillmiss,nbs_friction_grid,'BinLimits',[0,1]); % Calculate a 101-bin histogram for the image.
    % execute the k-means cluster
    nbs_cluster = length(counts(counts>1000));
    [cluster_idx,cluster_centroid_locations,sumDist] = kmeans(reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),nbs_cluster,'Replicates',1,'Display','final');
    
    % histtogram of friction data
    h_fig = figure(15567);
    set(h_fig,'Name','histogram of friction data');
    histogram(reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),edges)
    %stem(edges,counts)
    threshold = 0.15; %0.145; %meters, the Diagonal length of grid( 0.2 > threshold >0.141)
    nbs_neighbor = 8 ; % the number of neighbor nodes for each innner grid
    road_friction_cluster_boundary = [];
    boundary_index = [];
    for i_cluster=1:nbs_cluster
        cluster_index = find(cluster_idx == i_cluster);
        cluster_grid = [FrictionGrid_ref.position_x(cluster_index) FrictionGrid_ref.position_y(cluster_index)];
        cluster_friction = FrictionGrid_ref.friction(cluster_index);
        Md_cluster_grid = KDTreeSearcher(cluster_grid); % Mdl is a KDTreeSearcher model. By default, the distance metric it uses to search for neighbors is Euclidean distance.
        [~,D] = knnsearch(Md_cluster_grid,cluster_grid,'k',nbs_neighbor+1);
        %boundary_index = Idx(D(:,nbs_neighbor+1)>threshold,1);
        cluster_boundary_index = find(D(:,nbs_neighbor+1)>threshold);
        road_friction_cluster_boundary = vertcat(road_friction_cluster_boundary,[cluster_grid(cluster_boundary_index,:) cluster_friction(cluster_boundary_index)] );
        boundary_index =  vertcat(boundary_index,cluster_index(cluster_boundary_index));
        clear D
        
        [row,col] = ind2sub(sz,ind)
    end
elseif method_boundary ==2
    % split friction data into groups
    nbs_friction_grid = 10;
    [counts,edges] = histcounts(FrictionGrid_measure.friction_mean_fillmiss,nbs_friction_grid,'BinLimits',[0,1]); % Calculate a 101-bin histogram for the image.
    % execute the k-means cluster
    nbs_cluster = length(counts(counts>1000));
    [cluster_idx,cluster_centroid_locations,sumDist] = kmeans(reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),nbs_cluster,'Replicates',1,'Display','final');
    
    % histtogram of friction data
    h_fig = figure(15567);
    set(h_fig,'Name','histogram of friction data');
    histogram(reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),edges)
    %stem(edges,counts)
    % step 1 : find the super pixel using the k-means cluster results
    Label_matrix = zeros(size(FrictionGrid_measure.friction_mean_fillmiss)); % cluster matrix
    for i_cluster=1:nbs_cluster
        
        cluster_index = find(cluster_idx == i_cluster);
        %         [cluster_sub_row,cluster_sub_col] = ind2sub(size(FrictionGrid_ref.friction),cluster_index); % cluster matrix subscripts
        
        cluster_friction = mean(FrictionGrid_measure.friction_mean_fillmiss(cluster_index),'all'); %value of each cluster
        
        Label_matrix(cluster_index) = cluster_friction; % assign values to each cluster
        
    end
    % step 2 : find the boundaries using the boundarymask functon
    Label_matrix(Label_matrix==0) =1;
    Label_matrix_pad = padarray(Label_matrix,[1 1],0,'both');
    
    boundary_pad = boundarymask(Label_matrix_pad);
    boundary = boundary_pad(2:end-1,2:end-1);
    
    figure(44473)
    %     imshow(label2rgb(L))
    clf
    Label_matrix(Label_matrix==1) =NaN;
    image([FrictionGrid_measure.position_s(1,1) FrictionGrid_measure.position_s(1,end)],...
        [FrictionGrid_measure.position_t(1,1) FrictionGrid_measure.position_t(end,1)],...
        Label_matrix,'CDataMapping','scaled')
    %     clf
    %     im = image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
    %         [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
    %         labeloverlay(FrictionGrid_ref.friction,boundary,'Transparency',0));
    
    figure(44474)
    %     imshow(label2rgb(L))
    clf
    surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,Label_matrix);
    hold on
    index_ref = 39;
    plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
    index_lane_center_lateral=[20 58];
    for i= 1:length(index_lane_center_lateral)
        plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
    end
    % axis tight
    % grid on
    colormap parula
    colorbar
    % shading interp
    shading flat
    lighting phong
    view(0,90)
    zlim([0 1])
    caxis([0.1,0.9])
    % axis equal
    box on
    xlabel('station [m]')
    ylabel('traversal [m]')
    zlabel('friction\_coeficient')
    
    figure(44482) % original and cluster data
    clf
    scatter3(reshape(FrictionGrid_measure.position_s,[],1),reshape(FrictionGrid_measure.position_t,[],1),reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),10,reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),'.');
    hold on
    FrictionGrid_measure.boundary = double(boundary);
    FrictionGrid_measure.boundary(FrictionGrid_measure.boundary==0)= NaN;
    scatter3(reshape(FrictionGrid_measure.position_s,[],1),reshape(FrictionGrid_measure.position_t,[],1),reshape(FrictionGrid_measure.boundary,[],1),20,reshape(FrictionGrid_measure.boundary,[],1),'r.');
    
    view(0,90)
    zlim([0 1])
    caxis([0.1,0.9])
    %     xlim([-5 500])
    xlabel('s[m]')
    ylabel('t[m]')
    box on
    nbs_boundariy_grid = length(find(FrictionGrid_measure.boundary ==1))
    
elseif method_boundary ==3
    [L,N] = superpixels(FrictionGrid_ref.friction,10);
    
    mask = boundarymask(L);
    
    figure(4447)
    %     imshow(label2rgb(L))
    image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        L,'CDataMapping','scaled')
    
    figure(4448)
    %     imshow(label2rgb(L))
    imshow(mask)
    image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        mask,'CDataMapping','scaled')
    figure(4449)
    %     im = image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
    %         [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
    %         labeloverlay(FrictionGrid_ref.friction,mask,'Transparency',0));
    %
    scatter3(FrictionGrid_ref.position_s,FrictionGrid_ref.position_t,mask,10, mask,'.');
    
    hold on
elseif method_boundary ==4
    %     nbs_cluster = 3;
    %     nbs_friction_grid = 5;
    lower_friction_bound = min(FrictionGrid_measure.friction_mean_fillmiss,[],'all');
    upper_friction_bound = max(FrictionGrid_measure.friction_mean_fillmiss,[],'all');
    friction_interval = 0.1
    nbs_friction_grid = round((upper_friction_bound-lower_friction_bound)/friction_interval);
    %     [counts,edges] = histcounts(FrictionGrid_measure.friction_mean_fillmiss,...
    %         nbs_friction_grid,'BinLimits',[lower_friction_bound,upper_friction_bound]); % Calculate a 101-bin histogram for the image.
    [counts,edges] = histcounts(FrictionGrid_measure.friction_mean_fillmiss,...
        round(1/friction_interval),'BinLimits',[0,1.0],'Normalization', 'probability' ); % Calculate a 101-bin histogram for the image.
    
    % histtogram of friction data
    h_fig = figure(15569);
    set(h_fig,'Name','histogram of friction data');
    width=540;%
    height=300;%
    right=100;%
    bottom=400;%
    set(gcf,'position',[right,bottom,width,height])
    histogram(reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),edges,'FaceColor',[0.3010 0.7450 0.9330],'Normalization', 'probability' )
    xlabel('Friction Coefficient')
    ylabel('Percentage')
    box on
    axes_handle = gca;
    set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+2);
    %     stem(edges,counts)
    ytix = get(axes_handle, 'YTick');
    set(axes_handle, 'YTick',ytix, 'YTickLabel',ytix*100)
    
    %%execute the k-means cluster
    nbs_cluster = length(counts(counts>0.01));
    friction_scale = 30;
    data = [reshape(FrictionGrid_measure.position_s,[],1)/max(FrictionGrid_measure.position_s(1,:)),...
        0.5+reshape(FrictionGrid_measure.position_t,[],1)/(2*max(FrictionGrid_measure.position_t(1,:))),...
        friction_scale*reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1)];
    [cluster_idx,cluster_centroid_locations,sumDist] = kmeans(data,nbs_cluster,'Replicates',3,...
        'Distance','sqeuclidean','Display','final'); % sqeuclidean,  cityblock
    figure(5559)
    clf
    hold on
    box on
    grid on
    view(0,90)
    zlim([0 1])
    %     xlim([-5 500])
    xlabel('s[m]')
    ylabel('t[m]')
    for i_cluster=1:nbs_cluster
        cluster_index = find(cluster_idx == i_cluster);
        scatter3(FrictionGrid_measure.position_s(cluster_index),FrictionGrid_measure.position_t(cluster_index),FrictionGrid_measure.friction_mean_fillmiss(cluster_index),10,'.');
    end
    %     nbs_cluster = 5
    %% step 1 : find the super pixel using the k-means cluster results
    Label_matrix = zeros(size(FrictionGrid_measure.friction_mean_fillmiss)); % cluster matrix
    cluster_mean_low_up_std = zeros(nbs_cluster,5);
    for i_cluster=1:nbs_cluster
        
        cluster_index = find(cluster_idx == i_cluster);
        Label_matrix(cluster_index) = cluster_centroid_locations(i_cluster,3)/friction_scale; % assign values to each cluster
        
        %statistics results of each cluster
        cluster_mean_low_up_std(i_cluster,1:4) = [mean(FrictionGrid_measure.friction_mean_fillmiss(cluster_index),'all') min(FrictionGrid_measure.friction_mean_fillmiss(cluster_index),[],'all') ...
            max(FrictionGrid_measure.friction_mean_fillmiss(cluster_index),[],'all') std(FrictionGrid_measure.friction_mean_fillmiss(cluster_index),0,'all')]; %value of each cluster
        
        sub_cluster = zeros(size(FrictionGrid_measure.friction_mean_fillmiss));
        sub_cluster(cluster_index) = Label_matrix(cluster_index);
        [L,n] = bwlabel(sub_cluster,4);
        for i=1:n
            index_subcluster = find(L==i);
            if length(index_subcluster)< 50 % find samll sub cluster where the grids number is less than 50
                Label_matrix(index_subcluster) = NaN;
            end
        end
        
    end
    cluster_mean_low_up_std(:,5)= cluster_mean_low_up_std(:,3)-cluster_mean_low_up_std(:,2);
    % fill the samll holes uing nearest data
    Label_matrix = fillmissing(Label_matrix,'nearest',2,'EndValues','nearest'); % by column
    
    
    %%step 2 : find the boundaries using the boundarymask functon
    
    % remove some noise before conducting boundary detection
    %     se = strel('disk',5);
    se = strel('square',7);
    %     Label_matrix_close = imclose(Label_matrix,se);
    Label_matrix_open = imopen(Label_matrix,se);
    
    % boundary detection
    Label_matrix_open(Label_matrix_open==0) =1;
    Label_matrix_pad = padarray(Label_matrix_open,[1 1],0,'both');
    
    boundary_pad = boundarymask(Label_matrix_pad);
    %     boundary_pad = edge(Label_matrix_pad,'Canny',0.002);
    
    boundary = boundary_pad(2:end-1,2:end-1);
    
    RMSE_true_grid = sqrt(mean((FrictionGrid_measure.friction_mean_fillmiss - FrictionGrid_ref.friction).^2,'all'))
    RMSE_true_cluster = sqrt(mean((Label_matrix_open - FrictionGrid_ref.friction).^2,'all'))
    
    max_abs_error_true_grid = maxk(abs(FrictionGrid_measure.friction_mean_fillmiss(:) - FrictionGrid_ref.friction(:)),2)
    max_abs_error_true_cluster = maxk(abs(Label_matrix_open(:)- FrictionGrid_ref.friction(:)),2)
    
    MAE_true_grid = mean(abs(FrictionGrid_measure.friction_mean_fillmiss - FrictionGrid_ref.friction),'all')
    MAE_true_cluster = mean(abs(Label_matrix_open- FrictionGrid_ref.friction),'all') % mean abs error
    
    RMSPE_true_grid = 100*sqrt(mean(((FrictionGrid_measure.friction_mean_fillmiss - FrictionGrid_ref.friction)./FrictionGrid_ref.friction).^2,'all'))
    RMSPE_true_cluster = 100*sqrt(mean(((Label_matrix_open - FrictionGrid_ref.friction)./FrictionGrid_ref.friction).^2,'all'))
    
    MAPE_true_grid = 100*mean(abs((FrictionGrid_measure.friction_mean_fillmiss - FrictionGrid_ref.friction)./FrictionGrid_ref.friction),'all')
    MAPE_true_cluster = 100*mean(abs((Label_matrix_open- FrictionGrid_ref.friction)./FrictionGrid_ref.friction),'all') %  mean absolute percentage error (MAPE)
    
    
    h_fig = figure(44474);
    set(h_fig,'Name','cluster results image');
    
    width=540;%
    height=300;%
    right=100;%
    bottom=400;%
    set(gcf,'position',[right,bottom,width,height])
    clf
    image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        Label_matrix_open,'CDataMapping','scaled')
    hold on
    
    index_ref = 39;
    plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
    index_lane_center_lateral=[20 58];
    for i= 1:length(index_lane_center_lateral)
        plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
    end
    set(gca,'YDir','normal')
    colormap parula
    colorbar
    % shading interp
    shading flat
    lighting phong
    view(0,90)
    zlim([0 1])
    caxis([0.1,0.9])
    % axis equal
    box on
    xlabel('Station[m]')
    ylabel('Transverse[m]')
    axes_handle = gca;
    set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+3);
    
    figure(444749)
    %     imshow(label2rgb(L))
    clf
    surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,Label_matrix);
    hold on
    index_ref = 39;
    plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
    index_lane_center_lateral=[20 58];
    for i= 1:length(index_lane_center_lateral)
        plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
    end
    % axis tight
    % grid on
    colormap parula
    colorbar
    % shading interp
    shading flat
    lighting phong
    view(0,90)
    zlim([0 1])
    caxis([0.1,0.9])
    % axis equal
    box on
    xlabel('station [m]')
    ylabel('traversal [m]')
    zlabel('friction\_coeficient')
    
    figure(444922) % original and cluster data
    clf
    surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,Label_matrix);
    %     scatter3(reshape(FrictionGrid_measure.position_s,[],1),reshape(FrictionGrid_measure.position_t,[],1),reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),10,reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),'.');
    hold on
    FrictionGrid_measure.boundary = double(boundary);
    FrictionGrid_measure.boundary(FrictionGrid_measure.boundary==0)= NaN;
    scatter3(reshape(FrictionGrid_measure.position_s,[],1),reshape(FrictionGrid_measure.position_t,[],1),reshape(FrictionGrid_measure.boundary,[],1),20,reshape(FrictionGrid_measure.boundary,[],1),'r.');
    shading flat
    view(0,90)
    zlim([0 1])
    colorbar
    caxis([0.1,0.9])
    %     xlim([-5 500])
    xlabel('s[m]')
    ylabel('t[m]')
    box on
    nbs_boundariy_grid = length(find(FrictionGrid_measure.boundary ==1))
    
    
    %% find sub cluster label using location connection
    friction_cluster = unique(Label_matrix_open);
    sub_cluster_label = zeros(size(Label_matrix));
    n=zeros(1,nbs_cluster+1);
    for i_cluster=1:nbs_cluster
        cluster_label = zeros(size(Label_matrix));
        
        cluster_label(Label_matrix_open==friction_cluster(i_cluster))=i_cluster;
        [L,n(i_cluster+1)] = bwlabel(cluster_label,4);
        
        
        sub_cluster_label(L>0) = L(L>0) + sum(n(1:i_cluster));
    end
    nbs_subcluster = sum(n)
    length(unique(sub_cluster_label))
    %     sub_cluster_label = ones(size(FrictionGrid_measure.friction_mean_fillmiss));
    %     sub_cluster_label(FrictionGrid_measure.boundary ==1) = 0;
    %     [~,nbs_subcluster] = bwlabel(sub_cluster_label,4)
    
    mid_results = diff(diff(FrictionGrid_measure.boundary,1,1),1,2);
    
    mean_cluster_boundary = [[mid_results ones(size(mid_results,1),1)];ones(1,size(FrictionGrid_measure.boundary,2))] ;
    
    mean_cluster_boundary(:,1)=1;
    mean_cluster_boundary(1,:)=1;
    mean_cluster_boundary(mean_cluster_boundary==0)=1;
    
    h_fig = figure(2612);
    set(h_fig,'Name','confidence interval magnitude');
    width=540;%
    height=300;%
    right=100;%
    bottom=400;%
    set(gcf,'position',[right,bottom,width,height])
    clf
    
    hold on
    surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,FrictionGrid_measure.friction_confidenceInterval_fillmiss,FrictionGrid_measure.friction_confidenceInterval_fillmiss)
    hold on
    %
    %     s = surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,FrictionGrid_measure.boundary,'EdgeColor','r','FaceColor','none'); %,'FaceAlpha',0.1
    s = surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,mean_cluster_boundary,'EdgeColor','r','FaceColor','none'); %,'FaceAlpha',0.1
    
    % scatter3(FrictionGrid_ref.position_x_trans(boundary_index),FrictionGrid_ref.position_y_trans(boundary_index),FrictionGrid_ref.friction_trans(boundary_index),'r.');
    index_ref = 39;
    
    plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
    index_lane_center_lateral=[20 58];
    for i= 1:length(index_lane_center_lateral)
        plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
    end
    %
    % legend('Magnitude of 95% confidence Interval','True Friction Boundary')
    % axis tight
    cMap = parula(256);
    % cMap = gray(256);
    dataMax = 1;
    dataMin = 0;
    centerPoint = 0.05;
    scalingIntensity = 0.7;
    % perform some operations to create your colormap
    x = 1:length(cMap);
    x = x - (centerPoint-dataMin)*length(x)/(dataMax-dataMin);
    x = scalingIntensity * x/max(abs(x));
    
    % x = sign(x).* exp(1.2*abs(x));
    x = (x).* exp(5*abs(x));
    x = x - min(x);
    x = x*511/max(x)+1;
    % figure(1111)
    % plot(x)
    
    newMap = interp1(x, cMap, 1:512);
    
    colormap(newMap)
    % caxis([0,200])
    cb=colorbar;
    cb.Label.String = 'confidence interval';
    cb.FontName= 'Times New Roman';
    cb.FontSize= Axis_label_FontSize+2;
    cb.Limits= [0 1];
    % cb.Limits= [0 1];
    cb.Ticks= [0 0.2 0.4 0.6 0.8 1.0];
    caxis([0 1])
    % shading interp
    shading flat
    lighting phong
    
    s.EdgeColor = 'r';
    s.LineWidth = 1.3;
    %
    view(0,90)
    zlim([0 100])
    xlim([-1 498])
    % axis equal
    box on
    xlabel('Station[m]')
    ylabel('Transverse[m]')
    % axis equal
    axes_handle = gca;
    set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+2);
    zlabel('friction\_coeficient')
    
    h_fig = figure(44475);
    set(h_fig,'Name','cluster results image');
    
    width=540;%
    height=300;%
    right=100;%
    bottom=400;%
    set(gcf,'position',[right,bottom,width,height])
    clf
    image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        Label_matrix_open,'CDataMapping','scaled')
    hold on
    s = surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,mean_cluster_boundary,'EdgeColor','r','FaceColor','none'); %,'FaceAlpha',0.1
    
    index_ref = 39;
    plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
    index_lane_center_lateral=[20 58];
    for i= 1:length(index_lane_center_lateral)
        plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
    end
    set(gca,'YDir','normal')
    colormap parula
    colorbar
    % shading interp
    shading flat
    lighting phong
    view(0,90)
    zlim([0 1])
    caxis([0.1,0.9])
    s.EdgeColor = 'r';
    s.LineWidth = 1.3;
    % axis equal
    box on
    xlabel('Station[m]')
    ylabel('Transverse[m]')
    axes_handle = gca;
    set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+3);
    
    
    
    %% cluster the confidence interval of each sub_cluster
    boundary_conf_int = NaN(size(FrictionGrid_measure.friction_confidenceInterval_fillmiss)); % cluster matrix
    Label_matrix_conf_int = boundary_conf_int;
    if flag.cluster_confidence_interval
        
        
        for i_subcluster = 1:nbs_subcluster
            
            ind_subcluster = find(sub_cluster_label==i_subcluster);
            
            %             conf_int_subcluster = zeros(size(Label_matrix));
            %
            %             conf_int_subcluster(sub_cluster_label==i_subcluster)=FrictionGrid_measure.friction_confidenceInterval_fillmiss(sub_cluster_label==i_subcluster);
            %
            %             subcluster.conf_int = reshape(conf_int_subcluster,[],1);
            %
            subcluster.conf_int = reshape(FrictionGrid_measure.friction_confidenceInterval_fillmiss(sub_cluster_label==i_subcluster),[],1);
            subcluster.position_s = reshape(FrictionGrid_measure.position_s(sub_cluster_label==i_subcluster),[],1);
            subcluster.position_t = reshape(FrictionGrid_measure.position_t(sub_cluster_label==i_subcluster),[],1);
            
            lower_conf_int_bound = min(subcluster.conf_int,[],'all');
            upper_conf_int_bound = max(subcluster.conf_int,[],'all');
            conf_int_interval = 0.1
            nbs_conf_int_grid = round((upper_conf_int_bound-lower_conf_int_bound)/conf_int_interval);
            %     [counts,edges] = histcounts(FrictionGrid_measure.friction_mean_fillmiss,...
            %         nbs_friction_grid,'BinLimits',[lower_friction_bound,upper_friction_bound]); % Calculate a 101-bin histogram for the image.
            [counts,edges] = histcounts(FrictionGrid_measure.friction_confidenceInterval_fillmiss,...
                [0,0.05,0.1:0.1:1],'Normalization', 'probability' ); % Calculate a 101-bin histogram for the image.
            
            %             [counts,edges] = histcounts(subcluster.conf_int,...
            %                 round(0.5/conf_int_interval),'BinLimits',[0,0.5],'Normalization', 'probability' ); % Calculate a 101-bin histogram for the image.
            %
            if flag.doDebug
                % histtogram of friction data
                h_fig = figure(15570);
                set(h_fig,'Name','histogram of conf_int data');
                width=540;%
                height=300;%
                right=100;%
                bottom=400;%
                set(gcf,'position',[right,bottom,width,height])
                histogram(subcluster.conf_int,edges,'FaceColor',[0.3010 0.7450 0.9330],'Normalization', 'probability' )
                xlabel('Confidence Interval')
                ylabel('Percentage')
                box on
                axes_handle = gca;
                set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+2);
                %     stem(edges,counts)
                ytix = get(axes_handle, 'YTick');
                set(axes_handle, 'YTick',ytix, 'YTickLabel',ytix*100)
            end
            %%execute the k-means cluster
            nbs_cluster = length(counts(counts>0.02));
            %             nbs_cluster = 4;
            conf_int_scale = 50;
            data = [subcluster.position_s/(max(subcluster.position_s) - min(subcluster.position_s)),...
                0.5+subcluster.position_t/(max(subcluster.position_t) - min(subcluster.position_t)),...
                conf_int_scale*subcluster.conf_int];
            [cluster_idx,cluster_centroid_locations,sumDist] = kmeans(data,nbs_cluster,'Replicates',3,...
                'Distance','sqeuclidean','Display','final'); % sqeuclidean,  cityblock
            if flag.doDebug
                figure(5560)
                clf
                hold on
                box on
                %             grid on
                view(0,90)
                zlim([0 1])
                %     xlim([-5 500])
                xlabel('s[m]')
                ylabel('t[m]')
                s = surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,mean_cluster_boundary,'EdgeColor','k','FaceColor','none'); %,'FaceAlpha',0.1
                
                for i_cluster=1:nbs_cluster
                    cluster_index = ind_subcluster(cluster_idx == i_cluster);
                    scatter3(FrictionGrid_measure.position_s(cluster_index),FrictionGrid_measure.position_t(cluster_index),FrictionGrid_measure.friction_confidenceInterval_fillmiss(cluster_index),10,'.');
                    %                 surf(FrictionGrid_measure.position_s(cluster_index),FrictionGrid_measure.position_t(cluster_index),FrictionGrid_measure.friction_confidenceInterval_fillmiss(cluster_index));
                end
            end
            %%step 1 : find the super pixel using the k-means cluster results
            Label_matrix = zeros(size(FrictionGrid_measure.friction_confidenceInterval_fillmiss)); % cluster matrix
            cluster_mean_low_up_std = zeros(nbs_cluster,5);
            for i_cluster=1:nbs_cluster
                
                cluster_index = ind_subcluster(cluster_idx == i_cluster);
                Label_matrix(cluster_index) = cluster_centroid_locations(i_cluster,3)/conf_int_scale; % assign values to each cluster
                
                %statistics results of each cluster
                cluster_mean_low_up_std(i_cluster,1:4) = [mean(FrictionGrid_measure.friction_confidenceInterval_fillmiss(cluster_index),'all') min(FrictionGrid_measure.friction_confidenceInterval_fillmiss(cluster_index),[],'all') ...
                    max(FrictionGrid_measure.friction_confidenceInterval_fillmiss(cluster_index),[],'all') std(FrictionGrid_measure.friction_confidenceInterval_fillmiss(cluster_index),0,'all')]; %value of each cluster
                
                sub_cluster = zeros(size(FrictionGrid_measure.friction_confidenceInterval_fillmiss));
                sub_cluster(cluster_index) = Label_matrix(cluster_index);
                [L,n] = bwlabel(sub_cluster,4);
                for i=1:n
                    index_subcluster = find(L==i);
                    if length(index_subcluster)< 20 %length(ind_subcluster)/1000 % find samll sub cluster where the grids number is less than 20
                        Label_matrix(index_subcluster) = NaN;
                    end
                end
                
            end
            cluster_mean_low_up_std(:,5)= cluster_mean_low_up_std(:,3)-cluster_mean_low_up_std(:,2);
            % fill the samll holes uing nearest data
            Label_matrix = fillmissing(Label_matrix,'nearest',2,'EndValues','nearest'); % by row
            
            
            %%step 2 : find the boundaries using the boundarymask functon
            
            % remove some noise before conducting boundary detection
            %     se = strel('disk',5);
            %             se = strel('square',5);
            se = strel('rectangle',[3 25]);
            se2 = strel('rectangle',[3 15]);
            %     Label_matrix_close = imclose(Label_matrix,se);
            Label_matrix_close = imclose(Label_matrix,se);
            Label_matrix_open = imopen(Label_matrix_close,se2);
            %             Label_matrix_open = imdilate(Label_matrix,se);
            %             Label_matrix_open = imopen(Label_matrix,se2);
            %             Label_matrix_open = imerode(originalBW,se);
            %             Label_matrix_open = imgaussfilt(Label_matrix,[4 4]);
            
            
            Label_matrix_conf_int(ind_subcluster) = Label_matrix_open(ind_subcluster); % local lable to global label
            
            % boundary detection
            Label_matrix_open(Label_matrix_open==0) =1;
            Label_matrix_pad = padarray(Label_matrix_open,[1 1],0,'both');
            
            boundary_pad = boundarymask(Label_matrix_pad);
            %     boundary_pad = edge(Label_matrix_pad,'Canny',0.002);
            
            boundary = double(boundary_pad(2:end-1,2:end-1));
            
            boundary(boundary==0)= NaN;
            
            mid_results = diff(diff(boundary,1,1),1,2);
            
            boundary_sub = [[mid_results ones(size(mid_results,1),1)];ones(1,size(boundary,2))] ;
            
            boundary_sub(:,1)=1;
            boundary_sub(1,:)=1;
            boundary_sub(boundary_sub==0)=1;
            
            boundary_conf_int(ind_subcluster) = boundary(ind_subcluster);
            
            
            h_fig = figure(444754);
            set(h_fig,'Name','cluster results image');
            clf
            image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
                [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
                Label_matrix_open,'CDataMapping','scaled')
            
            %             surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,boundary_sub,'EdgeColor','r','FaceColor','none'); %,'FaceAlpha',0.1
            
            hold on
            %             image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            %                 [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)]+0.05,...
            %                 boundary_sub,'CDataMapping','scaled')
            %
            %                          s = surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,mean_cluster_boundary,'EdgeColor','r','FaceColor','none'); %,'FaceAlpha',0.1
            s = surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,boundary,'EdgeColor','r','FaceColor','none'); %,'FaceAlpha',0.1
            
            set(gca,'YDir','normal')
            colormap parula
            colorbar
            % shading interp
            shading flat
            lighting phong
            view(0,90)
            zlim([0 1])
            xlim([min(subcluster.position_s)-0.1 max(subcluster.position_s)+0.1])
            ylim([min(subcluster.position_t)-0.1 max(subcluster.position_t)+0.1])
            caxis([0,1])
            % axis equal
            box on
            xlabel('Station[m]')
            ylabel('Transverse[m]')
            axes_handle = gca;
            set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+3);
            s.EdgeColor = 'r';
            s.LineWidth = 1.3;
            %
            
            
        end
        
        %%
        
        %% find sub cluster label using location connection
        conf_int_cluster = unique(Label_matrix_conf_int);
        conf_int_subcluster_label = zeros(size(Label_matrix_conf_int));
        n=zeros(1,length(conf_int_cluster)+1);
        for i_cluster=1:length(conf_int_cluster)
            cluster_label = zeros(size(Label_matrix));
            
            cluster_label(Label_matrix_conf_int==conf_int_cluster(i_cluster))=i_cluster;
            [L,n(i_cluster+1)] = bwlabel(cluster_label,4);
            
            conf_int_subcluster_label(L>0) = L(L>0) + sum(n(1:i_cluster));
        end
        nbs_conf_int_subcluster = sum(n)
        length(unique(conf_int_subcluster_label))
        
        
        
        %% boundary detection
        
        Label_matrix_conf_int_pad = padarray(Label_matrix_conf_int,[1 1],0,'both');
        
        boundary_pad = boundarymask(Label_matrix_conf_int_pad);
        %     boundary_pad = edge(Label_matrix_pad,'Canny',0.002);
        
        boundary_conf_int_2 = double(boundary_pad(2:end-1,2:end-1));
        boundary_conf_int_2(boundary_conf_int_2==0)= NaN;
        
        
        
        mid_results = diff(diff(boundary_conf_int_2,1,1),1,2);
        
        mid_mid_results = [[mid_results ones(size(mid_results,1),1)];ones(1,size(boundary_conf_int_2,2))] ;
        
        mid_mid_results(:,1)=1;
        mid_mid_results(1,:)=1;
        mid_mid_results(mid_mid_results==0)=1;
        
        try_num =2;
        if try_num==1
            conf_int_boundary = mid_mid_results;
        elseif try_num==2
            for i = 1:size(mid_mid_results,2)
                
                for j = 1:size(mid_mid_results,1)-3
                    %                     if ~all(mid_mid_results(j:min(j+3,size(mid_mid_results,1)),i)==1) && j
                    
                    if ((mid_mid_results(j,i) ==1) && (mid_mid_results(j+1,i) ==1 )...
                            && (mid_mid_results(j+2,i) ==1 ) && (isnan(mid_mid_results(j+3,i))) )
                        
                        mid_mid_results(j+1,i) = NaN;
                    end
                    %                     end
                end
            end
            for i = 1:size(mid_mid_results,2)
                
                for j = 1:size(mid_mid_results,1)-2
                    %                     if ~all(mid_mid_results(j:min(j+3,size(mid_mid_results,1)),i)==1) && j
                    
                    if ((mid_mid_results(j,i) ==1) && (mid_mid_results(j+1,i) ==1 )...
                            && (isnan(mid_mid_results(j+2,i))) )
                        
                        mid_mid_results(j+1,i) = NaN;
                    end
                    %                     end
                end
                
                
            end
            
            
            for i = 1:size(mid_mid_results,2)
                
                for j = [size(mid_mid_results,1)-2:size(mid_mid_results,1)-1]
                    %                     if ~all(mid_mid_results(j:min(j+3,size(mid_mid_results,1)),i)==1) && j
                    
                    if ((mid_mid_results(j,i) ==1) && (mid_mid_results(j+1,i) ==1 ))
                        
                        mid_mid_results(j+1,i) = NaN;
                    end
                    %                     end
                end
                
                
            end
            
            
            conf_int_boundary = mid_mid_results;
            
        else
            mid_results_2 = diff(mid_mid_results,1,1);
            mid_mid_results_2 = [mid_results_2 ;ones(1,size(boundary_conf_int_2,2))] ;
            
            mid_mid_results_2(:,1)=1;
            mid_mid_results_2(1,:)=1;
            mid_mid_results_2(mid_mid_results_2==0)=1;
            conf_int_boundary = mid_mid_results_2;
        end
        
        %
        h_fig = figure(2613);
        set(h_fig,'Name','confidence interval magnitude clustered');
        width=540;%
        height=300;%
        right=100;%
        bottom=400;%
        set(gcf,'position',[right,bottom,width,height])
        clf
        
        hold on
        %         surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,FrictionGrid_measure.friction_confidenceInterval_fillmiss,FrictionGrid_measure.friction_confidenceInterval_fillmiss)
        s1 = surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,mean_cluster_boundary,'EdgeColor','r','FaceColor','none'); %,'FaceAlpha',0.1
        %
        s2 = surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,conf_int_boundary,'EdgeColor','r','FaceColor','none'); %,'FaceAlpha',0.1
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)]+0.05,...
            Label_matrix_conf_int,'CDataMapping','scaled')
        % scatter3(FrictionGrid_ref.position_x_trans(boundary_index),FrictionGrid_ref.position_y_trans(boundary_index),FrictionGrid_ref.friction_trans(boundary_index),'r.');
        index_ref = 39;
        
        %         plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
        %         index_lane_center_lateral=[20 58];
        %         for i= 1:length(index_lane_center_lateral)
        %             plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
        %         end
        %
        % legend('Magnitude of 95% confidence Interval','True Friction Boundary')
        % axis tight
        cMap = parula(256);
        % cMap = gray(256);
        dataMax = 1;
        dataMin = 0;
        centerPoint = 0.05;
        scalingIntensity = 0.7;
        % perform some operations to create your colormap
        x = 1:length(cMap);
        x = x - (centerPoint-dataMin)*length(x)/(dataMax-dataMin);
        x = scalingIntensity * x/max(abs(x));
        
        % x = sign(x).* exp(1.2*abs(x));
        x = (x).* exp(5*abs(x));
        x = x - min(x);
        x = x*511/max(x)+1;
        % figure(1111)
        % plot(x)
        
        newMap = interp1(x, cMap, 1:512);
        
        colormap(newMap)
        % caxis([0,200])
        cb=colorbar;
        cb.Label.String = 'confidence interval';
        cb.FontName= 'Times New Roman';
        cb.FontSize= Axis_label_FontSize+2;
        cb.Limits= [0 1];
        % cb.Limits= [0 1];
        cb.Ticks= [0:0.2:1];
        caxis([0 1])
        % shading interp
        shading flat
        lighting phong
        
        s1.EdgeColor = 'r';
        s1.LineWidth = 4;
        
        s2.EdgeColor = 'k';
        s2.LineWidth = 1.5;
        %
        view(0,90)
        zlim([0 100])
        xlim([-1 498])
        % axis equal
        box on
        xlabel('Station[m]')
        ylabel('Transverse[m]')
        % axis equal
        axes_handle = gca;
        set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+2);
        zlabel('friction\_coeficient')
        
        
        %% step 3: split each sub_cluster into regular rectangular
        
        sub_cluster_label = zeros(size(Label_matrix_conf_int));
        friction_cluster = unique(Label_matrix_conf_int);
        nbs_subcluster = length(friction_cluster);
        for i=1:nbs_subcluster
            sub_cluster_label(Label_matrix_conf_int==friction_cluster(i))=i;
        end
        % sub_cluster_label(sub_cluster_label==3)=1;
        %
        rect_current_all= [];
        
        rect_row = [];
        rect_col = [];
        method = 2; % 1: rows at the first, 2: column at he first
        if method==1
            for i_cluster=1:nbs_subcluster
                
                [cluster_row,cluster_col] = find(sub_cluster_label == i_cluster);
                
                ind_cluster_row = unique(cluster_row); % the number of rows in the cluster
                
                % find the connected rectangular block in each row
                for i_row = 1:length(ind_cluster_row)
                    cluster_cols_irow = sort(cluster_col(cluster_row==ind_cluster_row(i_row))); % start from the first row
                    ind_edge = find(diff(cluster_cols_irow)>1);
                    ind_edges = [0 ind_edge length(cluster_cols_irow)];
                    for i_edge= 1:length(ind_edges)-1
                        rect_row = [rect_row; [ind_cluster_row(i_row) cluster_cols_irow(ind_edges(i_edge)+1) ind_cluster_row(i_row) cluster_cols_irow(ind_edges(i_edge+1)) ...
                            friction_cluster(i_cluster) 1*(cluster_cols_irow(ind_edges(i_edge+1))-cluster_cols_irow(ind_edges(i_edge)+1)+1)]]; % rectangular of each row
                    end
                    
                end
                
                % merge
                rect_current=[];
                rect_current = rect_row(rect_row(:,1)==ind_cluster_row(1),:);
                rectCounter = size(rect_row,1);
                rect_merge=[];
                for i_row = 2:length(ind_cluster_row)
                    
                    rect_irow_next = rect_row(rect_row(:,1)==ind_cluster_row(i_row),:);
                    
                    for i = 1: size(rect_current,1)
                        for j = 1: size(rect_irow_next,1)
                            
                            if (rect_current(i,2)==rect_irow_next(j,2)) && (rect_current(i,4)==rect_irow_next(j,4))&& (rect_current(i,1)==rect_irow_next(j,3)-1)
                                %                     rectCounter = rectCounter-1;
                                rect_merge = [rect_merge;[rect_irow_next(j,1) rect_irow_next(j,2) rect_current(i,3) rect_current(i,4) ...
                                    rect_current(i,5) rect_current(i,6)+rect_irow_next(j,6)]];
                                rect_irow_next(j,:)= [NaN NaN NaN NaN NaN NaN]; % remove original
                                rect_current(i,:)= [NaN NaN NaN NaN NaN NaN]; % remove original
                            end
                            
                        end
                    end
                    rect_current = [rect_merge;rect_current ;rect_irow_next];
                    rect_current(all(isnan(rect_current),2),:) = [];
                    rect_merge = [];
                    rect_current_all= [rect_current_all ;rect_current];
                end
                
            end
            rect_list=rect_current_all;
            % lower left corner,width , height.
            rect_list(:,3) = rect_current_all(:,4)-rect_current_all(:,2);
            rect_list(:,4) = rect_current_all(:,1)-rect_current_all(:,3);
            rect_list = unique(rect_list,'rows');
            rect_current_all= unique(rect_current_all,'rows');
        else
            for i_cluster=1:nbs_subcluster
                
                [cluster_row,cluster_col] = find(sub_cluster_label == i_cluster);
                
                ind_cluster_col = unique(cluster_col); % the number of columns in the cluster
                
                % find the connected rectangular block in each row
                for i_col = 1:length(ind_cluster_col)
                    cluster_rows_icol = sort(cluster_row(cluster_col==ind_cluster_col(i_col))); % start from the first row
                    ind_edge = find(diff(cluster_rows_icol)>1)';
                    ind_edges = [0 ind_edge length(cluster_rows_icol)];
                    for i_edge= 1:length(ind_edges)-1
                        rect_col = [rect_col; [ cluster_rows_icol(ind_edges(i_edge+1)) ind_cluster_col(i_col) cluster_rows_icol(ind_edges(i_edge)+1) ind_cluster_col(i_col)  ...
                            friction_cluster(i_cluster) 1*(cluster_rows_icol(ind_edges(i_edge+1))-cluster_rows_icol(ind_edges(i_edge)+1)+1)]]; % rectangular of each row
                    end
                    
                end
                
                % merge
                
                rect_current = rect_col(rect_col(:,2)==ind_cluster_col(1),:);
                rectCounter = size(rect_col,1);
                rect_merge=[];
                for i_col = 2:length(ind_cluster_col)
                    
                    rect_icol_next = rect_col(rect_col(:,4)==ind_cluster_col(i_col),:);
                    
                    for i = 1: size(rect_current,1)
                        for j = 1: size(rect_icol_next,1)
                            
                            if (rect_current(i,1)==rect_icol_next(j,1)) && (rect_current(i,3)==rect_icol_next(j,3))&& (rect_current(i,4)==rect_icol_next(j,2)-1)
                                %                     rectCounter = rectCounter-1;
                                rect_merge = [rect_merge;[ rect_current(i,1) rect_current(i,2) rect_icol_next(j,3) rect_icol_next(j,4) ...
                                    rect_current(i,5) rect_current(i,6)+rect_icol_next(j,6)]];
                                rect_icol_next(j,:)= [NaN NaN NaN NaN NaN NaN]; % remove original
                                rect_current(i,:)= [NaN NaN NaN NaN NaN NaN]; % remove original
                            end
                            
                        end
                    end
                    rect_current = [rect_merge;rect_current ;rect_icol_next];
                    rect_current(all(isnan(rect_current),2),:) = [];
                    rect_merge = [];
                    
                end
                rect_current_all= [rect_current_all ;rect_current];
            end
            
            % remove repeat blocks
            rect_current_all= unique(rect_current_all,'rows');
            
            
            rect_list=rect_current_all;
            % lower left corner,width , height.
            
            rect_list(:,3) = rect_current_all(:,4)-rect_current_all(:,2);
            rect_list(:,4) = rect_current_all(:,1)-rect_current_all(:,3);
            rect_list = unique(rect_list,'rows');
            
        end
        
        
        %% Plot the rectangles
        figure(555821)
        
        clf
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            Label_matrix_conf_int,'CDataMapping','scaled')
        set(gca,'YDir','normal')
        for i = 1:size(rect_list,1)
            rectangle('position',[FrictionGrid_ref.position_s(1,rect_list(i,2))-0.05 FrictionGrid_ref.position_t(rect_list(i,1),1)-0.05 0.1*rect_list(i,3:4)+[0.1 0.1]],'EdgeColor','r','LineWidth',2);
        end
        
        box on
        caxis([0.1,0.9])
        
        figure(555831)
        colors = parula(100);
        cmap = [(linspace(0,1,100))' colors];
        for i = 1:size(rect_list,1)
            p = rectangle('position',[FrictionGrid_ref.position_s(1,rect_list(i,2))-0.05 FrictionGrid_ref.position_t(rect_list(i,1),1)-0.05 0.1*rect_list(i,3:4)+[0.1 0.1]]);
            set(p,'edgecolor',[0.5 0.5 0.5],'linewidth',2,'facecolor',interp1(cmap(:,1),cmap(:,2:4),rect_list(i,5)));
        end
        
        colormap(colors);
        colorbar;
        
        caxis([0,1])
        nbs_bounding_box = size(rect_list,1)
        
        
        
        %% merge the small rects
        [areas,order] = sort(rect_current_all(:,6));
        rectList2 = rect_current_all(order,:);
        
        %     mergeTol = 0.2; % merge the rect whose area is smaller than or equal to this value
        mergeTol= 0.04;%friction_interval;
        
        for i = 1:size(rectList2,1)
            % If one of the rectangle dimensions is less than a threshold, combine
            % it with an adjacent one
            %rectList(i,:)
            
            if ((rectList2(i,1)-rectList2(i,3)) < 10) || ((rectList2(i,4)-rectList2(i,2)) < 20)  %height || width,
                % Find the adjacent rectangles
                Left  = find((rectList2(:,1) == rectList2(i,1)) & (rectList2(:,3) == rectList2(i,3)) & (rectList2(:,4)+1 == rectList2(i,2)));
                Right = find((rectList2(:,1) == rectList2(i,1)) & (rectList2(:,3) == rectList2(i,3)) & (rectList2(:,2)-1 == rectList2(i,4)));
                Down  = find((rectList2(:,2) == rectList2(i,2)) & (rectList2(:,4) == rectList2(i,4)) & (rectList2(:,3)-1 == rectList2(i,1)));
                Up    = find((rectList2(:,2) == rectList2(i,2)) & (rectList2(:,4) == rectList2(i,4)) & (rectList2(:,1)+1 == rectList2(i,3)));
                
                % Check to see which ones match dimensions, remove non-matches
                if isempty(Left)
                    Left = [];
                    matchList(1) = Inf;
                else
                    Left = Left(1);
                    %                     matchList(1) = abs(FrictionGrid_measure.friction_confidenceInterval_fillmiss(rectList2(Left,1),rectList2(Left,2)) - FrictionGrid_measure.friction_confidenceInterval_fillmiss(rectList2(i,1),rectList2(i,2)));
                    matchList(1) = abs(rectList2(Left,5) - rectList2(i,5));
                    
                end
                if isempty(Right)
                    Right = [];
                    matchList(2) = Inf;
                else
                    Right = Right(1);
                    %                     matchList(2) = abs(FrictionGrid_measure.friction_confidenceInterval_fillmiss(rectList2(Right,1),rectList2(Right,2)) - FrictionGrid_measure.friction_confidenceInterval_fillmiss(rectList2(i,1),rectList2(i,2)));
                    matchList(2) = abs(rectList2(Right,5) - rectList2(i,5));
                end
                if isempty(Up)
                    Up = [];
                    matchList(3) = Inf;
                else
                    Up = Up(1);
                    %                     matchList(3) = abs(FrictionGrid_measure.friction_confidenceInterval_fillmiss(rectList2(Up,1),rectList2(Up,2)) - FrictionGrid_measure.friction_confidenceInterval_fillmiss(rectList2(i,1),rectList2(i,2)));
                    matchList(3) = abs(rectList2(Up,5) - rectList2(i,5));
                end
                if isempty(Down)
                    Down = [];
                    matchList(4) = Inf;
                else
                    Down = Down(1);
                    %                     if length(Down)>1
                    %                         i
                    %                     else
                    %
                    %                         matchList(4) = abs(FrictionGrid_measure.friction_confidenceInterval_fillmiss(rectList2(Down,1),rectList2(Down,2)) - FrictionGrid_measure.friction_confidenceInterval_fillmiss(rectList2(i,1),rectList2(i,2)));
                    %                     end
                    %                     matchList(4) = abs(FrictionGrid_measure.friction_confidenceInterval_fillmiss(rectList2(Down(1),1),rectList2(Down(1),2)) - FrictionGrid_measure.friction_confidenceInterval_fillmiss(rectList2(i,1),rectList2(i,2)));
                    matchList(4) = abs(rectList2(Down,5) - rectList2(i,5));
                    
                end
                % Of the remaining matching rectangles, determine which have closer
                % means and combine
                [minVal,minInd] = min(matchList);
                
                % Update the rectangle list to reflect the merge. Make sure to
                % eliminate rectangle 'i' so that it doesn't get repeated and so
                % the combined rectangle gets checked later.
                if minVal < mergeTol
                    switch(minInd)
                        case 1
                            rectList2(Left,5) = (rectList2(Left,5)*rectList2(Left,6) + rectList2(i,5)*rectList2(i,6))/(rectList2(Left,6)+rectList2(i,6));
                            rectList2(Left,6) = rectList2(Left,6)+rectList2(i,6);
                            rectList2(Left,3) = rectList2(i,3);
                            rectList2(Left,4) = rectList2(i,4);
                        case 2
                            rectList2(Right,5) = (rectList2(Right,5)*rectList2(Right,6) + rectList2(i,5)*rectList2(i,6))/(rectList2(Right,6)+rectList2(i,6));
                            rectList2(Right,6) = rectList2(Right,6)+rectList2(i,6);
                            rectList2(Right,1) = rectList2(i,1);
                            rectList2(Right,2) = rectList2(i,2);
                        case 3
                            rectList2(Up(1),5) = (rectList2(Up(1),5)*rectList2(Up(1),6) + rectList2(i,5)*rectList2(i,6))/(rectList2(Up(1),6)+rectList2(i,6));
                            rectList2(Up(1),6) = rectList2(Up(1),6)+rectList2(i,6);
                            rectList2(Up(1),1) = rectList2(i,1);
                            rectList2(Up(1),2) = rectList2(i,2);
                        case 4
                            rectList2(Down(1),3) = rectList2(i,3);
                            rectList2(Down(1),4) = rectList2(i,4);
                            rectList2(Down(1),5) = (rectList2(Down(1),5)*rectList2(Down(1),6) + rectList2(i,5)*rectList2(i,6))/(rectList2(Down(1),6)+rectList2(i,6));
                            rectList2(Down(1),6) = rectList2(Down(1),6)+rectList2(i,6);
                            
                    end
                    % And eliminate the row that was merged
                    rectList2(i,:) = [NaN NaN NaN NaN NaN NaN];
                end
            end
        end
        
        % Collapse all of the NaN rows
        rectList2(all(isnan(rectList2),2),:) = [];
        rect_list2=rectList2;
        % lower left corner,width , height.
        rect_list2(:,3) = rectList2(:,4)-rectList2(:,2);
        rect_list2(:,4) = rectList2(:,1)-rectList2(:,3);
        rect_list_conf_int = rect_list2;
        save('rect_list_conf_int.mat','rect_list_conf_int')
        %%
        load('rect_list_mean.mat','rect_list_mean')
        
        h_fig = figure(555842);
        set(h_fig,'Name','Partition each cluster into ranctanguler');
        clf
        width=540;%
        height=300;%
        right=100;%
        bottom=400;%
        set(gcf,'position',[right,bottom,width,height])
        %     image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        %         [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        %         Label_matrix,'CDataMapping','scaled')
        %     set(gca,'YDir','normal')
        for i = 1:size(rect_list2,1)
            p = rectangle('position',[FrictionGrid_ref.position_s(1,rect_list2(i,2))-0.05 FrictionGrid_ref.position_t(rect_list2(i,1),1)-0.05 0.1*rect_list2(i,3:4)+[0.1 0.1]]);
            set(p,'edgecolor',[0 0 0],'linewidth',1,'facecolor',interp1(cmap(:,1),cmap(:,2:4),rect_list2(i,5)));
        end
        
        hold on
        
        for i = 1:size(rect_list_mean,1)
            p = rectangle('position',[FrictionGrid_ref.position_s(1,rect_list2(i,2))-0.05 FrictionGrid_ref.position_t(rect_list_mean(i,1),1)-0.05 0.1*rect_list_mean(i,3:4)+[0.1 0.1]]);
            set(p,'edgecolor',[0 0 0],'linewidth',1,'facecolor','none');
        end
        
        box on
        
        
        cMap = parula(256);
        % cMap = gray(256);
        dataMax = 1;
        dataMin = 0;
        centerPoint = 0.05;
        scalingIntensity = 0.7;
        % perform some operations to create your colormap
        x = 1:length(cMap);
        x = x - (centerPoint-dataMin)*length(x)/(dataMax-dataMin);
        x = scalingIntensity * x/max(abs(x));
        
        % x = sign(x).* exp(1.2*abs(x));
        x = (x).* exp(5*abs(x));
        x = x - min(x);
        x = x*511/max(x)+1;
        % figure(1111)
        % plot(x)
        
        newMap = interp1(x, cMap, 1:512);
        
        colormap(newMap)
        % caxis([0,200])
        cb=colorbar;
        cb.Label.String = 'confidence interval';
        cb.FontName= 'Times New Roman';
        cb.FontSize= Axis_label_FontSize+2;
        cb.Limits= [0 1];
        % cb.Limits= [0 1];
        cb.Ticks= [0:0.2:1];
        caxis([0 1])
        % shading interp
        shading flat
        lighting phong
        
        %
        view(0,90)
        zlim([0 100])
        xlim([-1 498])
        % axis equal
        box on
        xlabel('Station[m]')
        ylabel('Transverse[m]')
        % axis equal
        axes_handle = gca;
        set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+2);
        zlabel('friction\_coeficient')
        
        
        nbs_bounding_box2 = size(rect_list2,1)
        
        %%
        h_fig = figure(555841);
        set(h_fig,'Name','Partition each cluster into ranctanguler');
        
        width=540;%
        height=300;%
        right=100;%
        bottom=400;%
        set(gcf,'position',[right,bottom,width,height])
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            Label_matrix_conf_int,'CDataMapping','scaled')
        set(gca,'YDir','normal')
        colors = parula(100);
        cmap = [(linspace(0,1,100))' colors];
        for i = 1:size(rect_list2,1)
            p = rectangle('position',[FrictionGrid_ref.position_s(1,rect_list2(i,2))-0.05 FrictionGrid_ref.position_t(rect_list2(i,1),1)-0.05 0.1*rect_list2(i,3:4)+[0.1 0.1]]);
            set(p,'edgecolor','r','linewidth',2);
        end
        
        colormap(colors);
        colorbar;
        
        caxis([0,1])
        
        
        
        
    else
        
        
        %% step 3: split each sub_cluster into regular rectangular
        
        sub_cluster_label = zeros(size(Label_matrix));
        friction_cluster = unique(Label_matrix_open);
        nbs_subcluster = length(friction_cluster);
        for i=1:nbs_subcluster
            sub_cluster_label(Label_matrix_open==friction_cluster(i))=i;
        end
        % sub_cluster_label(sub_cluster_label==3)=1;
        %
        rect_current_all= [];
        
        rect_row = [];
        rect_col = [];
        method = 2; % 1: rows at the first, 2: column at he first
        if method==1
            for i_cluster=1:nbs_subcluster
                
                [cluster_row,cluster_col] = find(sub_cluster_label == i_cluster);
                
                ind_cluster_row = unique(cluster_row); % the number of rows in the cluster
                
                % find the connected rectangular block in each row
                for i_row = 1:length(ind_cluster_row)
                    cluster_cols_irow = sort(cluster_col(cluster_row==ind_cluster_row(i_row))); % start from the first row
                    ind_edge = find(diff(cluster_cols_irow)>1);
                    ind_edges = [0 ind_edge length(cluster_cols_irow)];
                    for i_edge= 1:length(ind_edges)-1
                        rect_row = [rect_row; [ind_cluster_row(i_row) cluster_cols_irow(ind_edges(i_edge)+1) ind_cluster_row(i_row) cluster_cols_irow(ind_edges(i_edge+1)) ...
                            friction_cluster(i_cluster) 1*(cluster_cols_irow(ind_edges(i_edge+1))-cluster_cols_irow(ind_edges(i_edge)+1)+1)]]; % rectangular of each row
                    end
                    
                end
                
                % merge
                rect_current=[];
                rect_current = rect_row(rect_row(:,1)==ind_cluster_row(1),:);
                rectCounter = size(rect_row,1);
                rect_merge=[];
                for i_row = 2:length(ind_cluster_row)
                    
                    rect_irow_next = rect_row(rect_row(:,1)==ind_cluster_row(i_row),:);
                    
                    for i = 1: size(rect_current,1)
                        for j = 1: size(rect_irow_next,1)
                            
                            if (rect_current(i,2)==rect_irow_next(j,2)) && (rect_current(i,4)==rect_irow_next(j,4))&& (rect_current(i,1)==rect_irow_next(j,3)-1)
                                %                     rectCounter = rectCounter-1;
                                rect_merge = [rect_merge;[rect_irow_next(j,1) rect_irow_next(j,2) rect_current(i,3) rect_current(i,4) ...
                                    rect_current(i,5) rect_current(i,6)+rect_irow_next(j,6)]];
                                rect_irow_next(j,:)= [NaN NaN NaN NaN NaN NaN]; % remove original
                                rect_current(i,:)= [NaN NaN NaN NaN NaN NaN]; % remove original
                            end
                            
                        end
                    end
                    rect_current = [rect_merge;rect_current ;rect_irow_next];
                    rect_current(all(isnan(rect_current),2),:) = [];
                    rect_merge = [];
                    rect_current_all= [rect_current_all ;rect_current];
                end
                
            end
            rect_list=rect_current_all;
            % lower left corner,width , height.
            rect_list(:,3) = rect_current_all(:,4)-rect_current_all(:,2);
            rect_list(:,4) = rect_current_all(:,1)-rect_current_all(:,3);
            rect_list = unique(rect_list,'rows');
            rect_current_all= unique(rect_current_all,'rows');
        else
            for i_cluster=1:nbs_subcluster
                
                [cluster_row,cluster_col] = find(sub_cluster_label == i_cluster);
                
                ind_cluster_col = unique(cluster_col); % the number of columns in the cluster
                
                % find the connected rectangular block in each row
                for i_col = 1:length(ind_cluster_col)
                    cluster_rows_icol = sort(cluster_row(cluster_col==ind_cluster_col(i_col))); % start from the first row
                    ind_edge = find(diff(cluster_rows_icol)>1)';
                    ind_edges = [0 ind_edge length(cluster_rows_icol)];
                    for i_edge= 1:length(ind_edges)-1
                        rect_col = [rect_col; [ cluster_rows_icol(ind_edges(i_edge+1)) ind_cluster_col(i_col) cluster_rows_icol(ind_edges(i_edge)+1) ind_cluster_col(i_col)  ...
                            friction_cluster(i_cluster) 1*(cluster_rows_icol(ind_edges(i_edge+1))-cluster_rows_icol(ind_edges(i_edge)+1)+1)]]; % rectangular of each row
                    end
                    
                end
                
                % merge
                
                rect_current = rect_col(rect_col(:,2)==ind_cluster_col(1),:);
                rectCounter = size(rect_col,1);
                rect_merge=[];
                for i_col = 2:length(ind_cluster_col)
                    
                    rect_icol_next = rect_col(rect_col(:,4)==ind_cluster_col(i_col),:);
                    
                    for i = 1: size(rect_current,1)
                        for j = 1: size(rect_icol_next,1)
                            
                            if (rect_current(i,1)==rect_icol_next(j,1)) && (rect_current(i,3)==rect_icol_next(j,3))&& (rect_current(i,4)==rect_icol_next(j,2)-1)
                                %                     rectCounter = rectCounter-1;
                                rect_merge = [rect_merge;[ rect_current(i,1) rect_current(i,2) rect_icol_next(j,3) rect_icol_next(j,4) ...
                                    rect_current(i,5) rect_current(i,6)+rect_icol_next(j,6)]];
                                rect_icol_next(j,:)= [NaN NaN NaN NaN NaN NaN]; % remove original
                                rect_current(i,:)= [NaN NaN NaN NaN NaN NaN]; % remove original
                            end
                            
                        end
                    end
                    rect_current = [rect_merge;rect_current ;rect_icol_next];
                    rect_current(all(isnan(rect_current),2),:) = [];
                    rect_merge = [];
                    
                end
                rect_current_all= [rect_current_all ;rect_current];
            end
            rect_list=rect_current_all;
            % lower left corner,width , height.
            rect_list(:,3) = rect_current_all(:,4)-rect_current_all(:,2);
            rect_list(:,4) = rect_current_all(:,1)-rect_current_all(:,3);
            rect_list = unique(rect_list,'rows');
            rect_current_all= unique(rect_current_all,'rows');
        end
        %% Plot the rectangles
        figure(555821)
        
        clf
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            Label_matrix,'CDataMapping','scaled')
        set(gca,'YDir','normal')
        for i = 1:size(rect_list,1)
            rectangle('position',[FrictionGrid_ref.position_s(1,rect_list(i,2))-0.05 FrictionGrid_ref.position_t(rect_list(i,1),1)-0.05 0.1*rect_list(i,3:4)+[0.1 0.1]],'EdgeColor','r','LineWidth',2);
        end
        
        box on
        caxis([0,1])
        
        figure(555831)
        colors = parula(100);
        cmap = [(linspace(0,1,100))' colors];
        for i = 1:size(rect_list,1)
            p = rectangle('position',[FrictionGrid_ref.position_s(1,rect_list(i,2))-0.05 FrictionGrid_ref.position_t(rect_list(i,1),1)-0.05 0.1*rect_list(i,3:4)+[0.1 0.1]]);
            set(p,'edgecolor',[0.5 0.5 0.5],'linewidth',2,'facecolor',interp1(cmap(:,1),cmap(:,2:4),rect_list(i,5)));
        end
        
        colormap(colors);
        colorbar;
        
        caxis([0,1])
        nbs_bounding_box = size(rect_list,1)
        %%
        [areas,order] = sort(rect_current_all(:,6));
        rectList2 = rect_current_all(order,:);
        
        %     mergeTol = 0.2; % merge the rect whose area is smaller than or equal to this value
        mergeTol= friction_interval;
        
        for i = 1:size(rectList2,1)
            % If one of the rectangle dimensions is less than a threshold, combine
            % it with an adjacent one
            %rectList(i,:)
            if ((rectList2(i,1)-rectList2(i,3)) < 10) || ((rectList2(i,4)-rectList2(i,2)) < 15)
                % Find the adjacent rectangles
                Left  = find((rectList2(:,1) == rectList2(i,1)) & (rectList2(:,3) == rectList2(i,3)) & (rectList2(:,4)+1 == rectList2(i,2)));
                Right = find((rectList2(:,1) == rectList2(i,1)) & (rectList2(:,3) == rectList2(i,3)) & (rectList2(:,2)-1 == rectList2(i,4)));
                Down  = find((rectList2(:,2) == rectList2(i,2)) & (rectList2(:,4) == rectList2(i,4)) & (rectList2(:,3)-1 == rectList2(i,1)));
                Up    = find((rectList2(:,2) == rectList2(i,2)) & (rectList2(:,4) == rectList2(i,4)) & (rectList2(:,1)+1 == rectList2(i,3)));
                % Check to see which ones match dimensions, remove non-matches
                if isempty(Left)
                    Left = [];
                    matchList(1) = Inf;
                else
                    Left = Left(1);
                    matchList(1) = abs(FrictionGrid_measure.friction_mean_fillmiss(rectList2(Left,1),rectList2(Left,2)) - FrictionGrid_measure.friction_mean_fillmiss(rectList2(i,1),rectList2(i,2)));
                end
                if isempty(Right)
                    Right = [];
                    matchList(2) = Inf;
                else
                    Right = Right(1);
                    matchList(2) = abs(FrictionGrid_measure.friction_mean_fillmiss(rectList2(Right,1),rectList2(Right,2)) - FrictionGrid_measure.friction_mean_fillmiss(rectList2(i,1),rectList2(i,2)));
                end
                if isempty(Up)
                    Up = [];
                    matchList(3) = Inf;
                else
                    matchList(3) = abs(FrictionGrid_measure.friction_mean_fillmiss(rectList2(Up,1),rectList2(Up,2)) - FrictionGrid_measure.friction_mean_fillmiss(rectList2(i,1),rectList2(i,2)));
                end
                if isempty(Down)
                    Down = [];
                    matchList(4) = Inf;
                else
                    matchList(4) = abs(FrictionGrid_measure.friction_mean_fillmiss(rectList2(Down,1),rectList2(Down,2)) - FrictionGrid_measure.friction_mean_fillmiss(rectList2(i,1),rectList2(i,2)));
                end
                % Of the remaining matching rectangles, determine which have closer
                % means and combine
                [minVal,minInd] = min(matchList);
                
                % Update the rectangle list to reflect the merge. Make sure to
                % eliminate rectangle 'i' so that it doesn't get repeated and so
                % the combined rectangle gets checked later.
                if minVal < mergeTol
                    switch(minInd)
                        case 1
                            rectList2(Left,5) = (rectList2(Left,5)*rectList2(Left,6) + rectList2(i,5)*rectList2(i,6))/(rectList2(Left,6)+rectList2(i,6));
                            rectList2(Left,6) = rectList2(Left,6)+rectList2(i,6);
                            rectList2(Left,3) = rectList2(i,3);
                            rectList2(Left,4) = rectList2(i,4);
                        case 2
                            rectList2(Right,5) = (rectList2(Right,5)*rectList2(Right,6) + rectList2(i,5)*rectList2(i,6))/(rectList2(Right,6)+rectList2(i,6));
                            rectList2(Right,6) = rectList2(Right,6)+rectList2(i,6);
                            rectList2(Right,1) = rectList2(i,1);
                            rectList2(Right,2) = rectList2(i,2);
                        case 3
                            rectList2(Up,5) = (rectList2(Up,5)*rectList2(Up,6) + rectList2(i,5)*rectList2(i,6))/(rectList2(Up,6)+rectList2(i,6));
                            rectList2(Up,6) = rectList2(Up,6)+rectList2(i,6);
                            rectList2(Up,1) = rectList2(i,1);
                            rectList2(Up,2) = rectList2(i,2);
                        case 4
                            rectList2(Down,3) = rectList2(i,3);
                            rectList2(Down,4) = rectList2(i,4);
                            rectList2(Down,5) = (rectList2(Down,5)*rectList2(Down,6) + rectList2(i,5)*rectList2(i,6))/(rectList2(Down,6)+rectList2(i,6));
                            rectList2(Down,6) = rectList2(Down,6)+rectList2(i,6);
                            
                    end
                    % And eliminate the row that was merged
                    rectList2(i,:) = [NaN NaN NaN NaN NaN NaN];
                end
            end
        end
        
        % Collapse all of the NaN rows
        rectList2(all(isnan(rectList2),2),:) = [];
        rect_list2=rectList2;
        % lower left corner,width , height.
        rect_list2(:,3) = rectList2(:,4)-rectList2(:,2);
        rect_list2(:,4) = rectList2(:,1)-rectList2(:,3);
        
        h_fig = figure(555842);
        set(h_fig,'Name','Partition each cluster into ranctanguler');
        clf
        width=540;%
        height=300;%
        right=100;%
        bottom=400;%
        set(gcf,'position',[right,bottom,width,height])
        %     image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        %         [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        %         Label_matrix,'CDataMapping','scaled')
        %     set(gca,'YDir','normal')
        for i = 1:size(rect_list2,1)
            p = rectangle('position',[FrictionGrid_ref.position_s(1,rect_list2(i,2))-0.05 FrictionGrid_ref.position_t(rect_list2(i,1),1)-0.05 0.1*rect_list2(i,3:4)+[0.1 0.1]]);
            set(p,'edgecolor',[0 0 0],'linewidth',1,'facecolor',interp1(cmap(:,1),cmap(:,2:4),rect_list2(i,5)));
        end
        
        box on
        caxis([0,1])
        xlabel('Station[m]')
        ylabel('Transverse[m]')
        axes_handle = gca;
        set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+2);
        
        colorbar;
        xlim([0,inf])
        %     legend('boxdd')
        
        nbs_bounding_box2 = size(rect_list2,1)
        
        %%
        h_fig = figure(555841);
        set(h_fig,'Name','Partition each cluster into ranctanguler');
        
        width=540;%
        height=300;%
        right=100;%
        bottom=400;%
        set(gcf,'position',[right,bottom,width,height])
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            Label_matrix_open,'CDataMapping','scaled')
        set(gca,'YDir','normal')
        colors = parula(100);
        cmap = [(linspace(0,1,100))' colors];
        for i = 1:size(rect_list2,1)
            p = rectangle('position',[FrictionGrid_ref.position_s(1,rect_list2(i,2))-0.05 FrictionGrid_ref.position_t(rect_list2(i,1),1)-0.05 0.1*rect_list2(i,3:4)+[0.1 0.1]]);
            set(p,'edgecolor','r','linewidth',2);
        end
        
        colormap(colors);
        colorbar;
        
        caxis([0,1])
        %%
        
    end
    
    
    
else
    %%
    if 1==1
        Ml = size(FrictionGrid_measure.friction_mean_fillmiss,2);
        Mw = size(FrictionGrid_measure.friction_mean_fillmiss,1);
        binned = FrictionGrid_measure.friction_mean_fillmiss';
        %     binned = Label_matrix';
        rects = zeros(Ml,Mw);
        % Set a count for the rectangles
        rectCounter = 0;
        % Set the tolerance for equivalence
        tol = 0.1;
        % Create a list for the rectangles and their properties
        rectList = [];
        
        for i = 1:Ml
            for j = 1:Mw
                % Check to make sure the grid has not already been labeled. If so,
                % no need to do anything
                if rects(i,j) == 0
                    % Starting a new rectangle, so increment the counter
                    rectCounter = rectCounter + 1;
                    % Assign this corner to the next rectangle value and increment
                    rects(i,j) = rectCounter;
                    % Set an expansion flag
                    maxExpandFlag = 0;
                    % Set the next corner to test expansion of the rectangle
                    testGridi = i+1;
                    testGridj = j+1;
                    
                    % Expand the rectangle as much as possible in the i
                    % direction, taking into account previously identified
                    % rectangles, and the threshold for averaging across rectangles
                    while(testGridi <= Ml && (all(rects(testGridi,j) == 0)) && (all(abs(binned(i,j) - binned(testGridi-1:testGridi,j)) < tol)))
                        testGridi = testGridi + 1;
                    end
                    testGridi = testGridi - 1;
                    % Expand the rectangle as much as possible in the j
                    % direction, taking into account the new extent of the
                    % rectangle in the i direction, previously identified
                    % rectangles, and the threshold for averaging across rectangles
                    while(testGridj <= Mw && (all(rects(i:testGridi,testGridj) == 0)) && (all(abs(binned(testGridi,j) - binned(testGridi,testGridj-1:testGridj)) < tol)))
                        testGridj = testGridj + 1;
                    end
                    testGridj = testGridj - 1;
                    
                    % Store the rectangle in the rectangle listing with the i, j
                    % coordinates of the corner, the height and width, the mean
                    % friction value, and the area
                    rectList = [rectList; [i j testGridi-i+1 testGridj-j+1 mean(binned(i:testGridi,j:testGridj),'all') (testGridi-i+1).*(testGridj-j+1)]];
                    
                    % Set the identified grid spaces to the current rectangle
                    % number so that they don't get identified again
                    rects(i:testGridi,j:testGridj) = rectCounter;
                end
            end
        end
        
        box_mean_low_up_std = zeros(rectCounter,5);
        for i_cluster=1:rectCounter
            
            cluster_index = find(rects' == i_cluster);
            box_mean_low_up_std(i_cluster,1:4) = [cluster_friction min(FrictionGrid_measure.friction_mean_fillmiss(cluster_index),[],'all') ...
                max(FrictionGrid_measure.friction_mean_fillmiss(cluster_index),[],'all') std(FrictionGrid_measure.friction_mean_fillmiss(cluster_index),0,'all')]; %value of each cluster
            
        end
        box_mean_low_up_std(:,5) =box_mean_low_up_std(:,3) -box_mean_low_up_std(:,2);
        %%Plot the rectangles
        colors = parula(100);
        figure(3)
        %     cla
        clf
        %     cmap = [(linspace(min(binned,[],'all'),max(binned,[],'all'),100))' colors];
        cmap = [(linspace(0.1,0.9,100))' colors];
        for i = 1:rectCounter
            p = rectangle('position',[rectList(i,1:2) rectList(i,3:4)]);
            set(p,'edgecolor',[0.5 0.5 0.5],'linewidth',2,'facecolor',interp1(cmap(:,1),cmap(:,2:4),rectList(i,5)));
        end
        %     colormap(colors);
        colormap parula
        colorbar;
        %     caxis([min(binned,[],'all') max(binned,[],'all')])
        caxis([0.1,0.9])
        box on
        %%
        figure(55582)
        clf
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            (FrictionGrid_measure.friction_mean_fillmiss),'CDataMapping','scaled')  % Label_matrix
        set(gca,'YDir','normal')
        t_rev = flip(FrictionGrid_ref.position_t);
        for i = 1:size(rectList,1)
            rectangle('position',[FrictionGrid_ref.position_s(1,rectList(i,1)) t_rev(rectList(i,2),1)-0.05 0.1*rectList(i,3:4)],'EdgeColor','r','LineWidth',2);
        end
        
        box on
        caxis([0.1,0.9])
        %%
        figure(55583)
        colors = parula(100);
        cmap = [(linspace(0.1,0.9,100))' colors];
        t_rev = flip(FrictionGrid_ref.position_t);
        for i = 1:size(rectList,1)
            p = rectangle('position',[FrictionGrid_ref.position_s(1,rectList(i,1)) t_rev(rectList(i,2),1)-0.05 0.1*rectList(i,3:4)]);
            set(p,'edgecolor',[0.5 0.5 0.5],'linewidth',2,'facecolor',interp1(cmap(:,1),cmap(:,2:4),rectList(i,5)));
        end
        
        colormap(colors);
        colorbar;
        
        caxis([0.1,0.9])
        %%
        mergeTol = 0.1;
        % Sort the list by rectangle area
        [areas,order] = sort(rectList(:,3).*rectList(:,4));
        rectList2 = rectList(order,:);
        for i = 1:rectCounter
            % If one of the rectangle dimensions is less than a threshold, combine
            % it with an adjacent one
            %rectList(i,:)
            if rectList2(i,3) < 5 || rectList2(i,4) < 5
                % Find the adjacent rectangles
                Left = find((rectList2(:,2) == rectList2(i,2)) & (rectList2(:,1)+rectList2(:,3) == rectList2(i,3)));
                Right = find((rectList2(:,2) == rectList2(i,2)) & (rectList2(:,1) == rectList2(i,1)+rectList2(i,3)));
                Down = find((rectList2(:,1) == rectList2(i,1)) & (rectList2(:,2)+rectList2(:,4) == rectList2(i,2)));
                Up = find((rectList2(:,1) == rectList2(i,1)) & (rectList2(:,2) == rectList2(i,2)+rectList2(i,4)));
                % Check to see which ones match dimensions, remove non-matches
                if(isempty(Left) || rectList2(Left,4) ~= rectList2(i,4))
                    Left = [];
                    matchList(1) = Inf;
                else
                    matchList(1) = abs(binned(rectList2(Left,1),rectList2(Left,2)) - binned(rectList2(i,1),rectList2(i,2)));
                end
                if(isempty(Right) || rectList2(Right,4) ~= rectList2(i,4))
                    Right = [];
                    matchList(2) = Inf;
                else
                    matchList(2) = abs(binned(rectList2(Right,1),rectList2(Right,2)) - binned(rectList2(i,1),rectList2(i,2)));
                end
                if(isempty(Up) || rectList2(Up,3) ~= rectList2(i,3))
                    Up = [];
                    matchList(3) = Inf;
                else
                    matchList(3) = abs(binned(rectList2(Up,1),rectList2(Up,2)) - binned(rectList2(i,1),rectList2(i,2)));
                end
                if(isempty(Down) || rectList2(Down,3) ~= rectList2(i,3))
                    Down = [];
                    matchList(4) = Inf;
                else
                    matchList(4) = abs(binned(rectList2(Down,1),rectList2(Down,2)) - binned(rectList2(i,1),rectList2(i,2)));
                end
                % Of the remaining matching rectangles, determine which have closer
                % means and combine
                [minVal,minInd] = min(matchList);
                
                % Update the rectangle list to reflect the merge. Make sure to
                % eliminate rectangle 'i' so that it doesn't get repeated and so
                % the combined rectangle gets checked later.
                if minVal < mergeTol
                    
                    switch(minInd)
                        case 1
                            rectList2(Left,5) = (rectList2(Left,5)*rectList2(Left,6) + rectList2(i,5)*rectList2(i,6))/(rectList2(Left,6)+rectList2(i,6));
                            rectList2(Left,1) = min(rectList2(Left,1),rectList2(i,1));
                            % rectList2(Left,2) = min(rectList2(Left,2),rectList2(i,2))
                            rectList2(Left,3) = rectList2(Left,3) + rectList2(i,3);
                            % rectList2(Left,4) = rectList2(Left,4)
                        case 2
                            rectList2(Right,5) = (rectList2(Right,5)*rectList2(Right,6) + rectList2(i,5)*rectList2(i,6))/(rectList2(Right,6)+rectList2(i,6));
                            rectList2(Right,1) = min(rectList2(Right,1),rectList2(i,1));
                            % rectList2(Right,2) = min(rectList2(Right,2),rectList2(i,2))
                            rectList2(Right,3) = rectList2(Right,3) + rectList2(i,3);
                            % rectList2(Right,4) = rectList2(Right,4)
                        case 3
                            rectList2(Up,5) = (rectList2(Up,5)*rectList2(Up,6) + rectList2(i,5)*rectList2(i,6))/(rectList2(Up,6)+rectList2(i,6));
                            % rectList2(Up,1) = min(rectList2(Up,1),rectList2(i,1))
                            rectList2(Up,2) = min(rectList2(Up,2),rectList2(i,2));
                            % rectList2(Up,3) = rectList2(Up,3)
                            rectList2(Up,4) = rectList2(Up,4) + rectList2(i,4);
                        case 4
                            rectList2(Down,5) = (rectList2(Down,5)*rectList2(Down,6) + rectList2(i,5)*rectList2(i,6))/(rectList2(Down,6)+rectList2(i,6));
                            % rectList2(Down,1) = min(rectList2(Down,1),rectList2(i,1))
                            rectList2(Down,2) = min(rectList2(Down,2),rectList2(i,2));
                            % rectList2(Down,3) = rectList2(Down,3);
                            rectList2(Down,4) = rectList2(Down,4) + rectList2(i,4);
                    end
                    % And eliminate the row that was merged
                    rectList2(i,:) = [NaN NaN NaN NaN NaN NaN];
                end
            end
        end
        
        % Collapse all of the NaN rows
        validInds = find(~isnan(rectList2(:,1)));
        rectList3 = rectList2(validInds,:);
        rectCounter3 = length(validInds);
        
        %
        figure(55583)
        colors = parula(100);
        cmap = [(linspace(0.1,0.9,100))' colors];
        t_rev = flip(FrictionGrid_ref.position_t);
        for i = 1:size(rectList3,1)
            p = rectangle('position',[FrictionGrid_ref.position_s(1,rectList3(i,1)) t_rev(rectList3(i,2),1)-0.05 0.1*rectList3(i,3:4)]);
            set(p,'edgecolor',[0.5 0.5 0.5],'linewidth',2,'facecolor',interp1(cmap(:,1),cmap(:,2:4),rectList3(i,5)));
        end
        
        colormap(colors);
        colorbar;
        box on
        caxis([0.1,0.9])
        
        fprintf('Final number of rectangles is: %d\n',rectCounter3);
        
    end
end


%% critical boundaries analysis
if ~flag.cluster_confidence_interval
    clear boundary_detected_position
    boundary_detected = FrictionGrid_measure.boundary;
%     boundary_detected = mean_cluster_boundary;
    boundary_detected(isnan(boundary_detected)) = 0;
    boundary_detected_position(:,1) = FrictionGrid_measure.position_s(logical(boundary_detected));
    boundary_detected_position(:,2) = FrictionGrid_measure.position_t(logical(boundary_detected));
    
    boundary_detected(boundary_detected==0) = NaN;
    
    Md_critical_boundary_detected_position = KDTreeSearcher(boundary_detected_position); % Mdl is a KDTreeSearcher model. By default, the distance metric it uses to search for neighbors is Euclidean distance.
    [index_bound,D] = knnsearch(Md_critical_boundary_detected_position,critical_boundary_true_position,'k',1); % find the nearest detected boundaries and the distance 
    
    mean(D)
    
    max(D)
    
    std(D)
    
    sqrt(mean(D.^2,'all')) %the root-mean-square error (RMSE)
    
    critical_boundary_detected_position = boundary_detected_position(index_bound,:);
    
    [index_bound_matrix,D_matirx] = knnsearch([reshape(FrictionGrid_measure.position_s,[],1) reshape(FrictionGrid_measure.position_t,[],1)],critical_boundary_detected_position);
  
    critical_boundary_detected = NaN(size(boundary_detected));
    critical_boundary_detected(index_bound_matrix) = 1;
    
    h_fig = figure(444751);
    set(h_fig,'Name','cluster results image');
    
    width=540;%
    height=300;%
    right=100;%
    bottom=400;%
    set(gcf,'position',[right,bottom,width,height])
    clf
    image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        Label_matrix_open,'CDataMapping','scaled')
    hold on
    %s = surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,mean_cluster_boundary,'EdgeColor','r','FaceColor','none'); %,'FaceAlpha',0.1
%     s = surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,boundary_detected,'EdgeColor','r','FaceColor','none'); %,'FaceAlpha',0.1
    
    s2 = surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,critical_boundary_detected,'EdgeColor','r','FaceColor','none'); %,'FaceAlpha',0.1
    
    index_ref = 39;
    plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
    index_lane_center_lateral=[20 58];
    for i= 1:length(index_lane_center_lateral)
        plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
    end
    
%     scatter3(boundary_detected_position(:,1),boundary_detected_position(:,2),ones(size(boundary_detected_position(:,2))),'g.');
%     scatter3(critical_boundary_detected_position(:,1),critical_boundary_detected_position(:,2),ones(size(critical_boundary_detected_position(:,2))),'g.');
    
    set(gca,'YDir','normal')
    colormap parula
    colorbar
    % shading interp
    shading flat
    lighting phong
    view(0,90)
    zlim([0 1])
    caxis([0.1,0.9])
%     s.EdgeColor = 'r';
%     s.LineWidth = 1.3;
    
    s2.EdgeColor = 'r';
    s2.LineWidth = 3;
    
    % axis equal
    box on
    xlabel('Station[m]')
    ylabel('Transverse[m]')
    axes_handle = gca;
    set(axes_handle, 'FontName', 'Times New Roman', 'FontSize', Axis_label_FontSize+3);
    
end





%% Step 6.5: cluster the filling non-noisy friction measurement data

if flag.doDebug
    % split friction data into groups
    nbs_friction_grid = 20;
    [counts,edges] = histcounts(FrictionGrid_measure.friction_mean_true_fillmiss,nbs_friction_grid,'BinLimits',[0,1]); % Calculate a 101-bin histogram for the image.
    % execute the k-means cluster
    nbs_cluster = length(counts(counts>1000));
    [cluster_idx,cluster_centroid_locations,sumDist] = kmeans(reshape(FrictionGrid_measure.friction_mean_true_fillmiss,[],1),nbs_cluster,'Replicates',1,'Display','final');
    
    % histtogram of friction data
    h_fig = figure(15570);
    set(h_fig,'Name','histogram of friction data');
    histogram(reshape(FrictionGrid_measure.friction_mean_true_fillmiss,[],1),edges)
    %stem(edges,counts)
    %% find boundary of each cluster
    method_boundary = 4;
    if method_boundary ==1 % cluster find the boundaries through nearest neighbor
        threshold = 0.15; %0.145; %meters, the Diagonal length of grid( 0.2 > threshold >0.141)
        nbs_neighbor = 8 ; % the number of neighbor nodes for each innner grid
        road_friction_cluster_boundary = [];
        boundary_index = [];
        for i_cluster=1:nbs_cluster
            cluster_index = find(cluster_idx == i_cluster);
            cluster_grid = [FrictionGrid_ref.position_x(cluster_index) FrictionGrid_ref.position_y(cluster_index)];
            cluster_friction = FrictionGrid_ref.friction(cluster_index);
            Md_cluster_grid = KDTreeSearcher(cluster_grid); % Mdl is a KDTreeSearcher model. By default, the distance metric it uses to search for neighbors is Euclidean distance.
            [~,D] = knnsearch(Md_cluster_grid,cluster_grid,'k',nbs_neighbor+1);
            %boundary_index = Idx(D(:,nbs_neighbor+1)>threshold,1);
            cluster_boundary_index = find(D(:,nbs_neighbor+1)>threshold);
            road_friction_cluster_boundary = vertcat(road_friction_cluster_boundary,[cluster_grid(cluster_boundary_index,:) cluster_friction(cluster_boundary_index)] );
            boundary_index =  vertcat(boundary_index,cluster_index(cluster_boundary_index));
            clear D
            
            [row,col] = ind2sub(sz,ind)
        end
    elseif method_boundary ==2
        % step 1 : find the super pixel using the k-means cluster results
        Label_matrix = zeros(size(FrictionGrid_measure.friction_mean_true_fillmiss)); % cluster matrix
        for i_cluster=1:nbs_cluster
            
            cluster_index = find(cluster_idx == i_cluster);
            %         [cluster_sub_row,cluster_sub_col] = ind2sub(size(FrictionGrid_ref.friction),cluster_index); % cluster matrix subscripts
            
            cluster_friction = mean(FrictionGrid_measure.friction_mean_true_fillmiss(cluster_index),'all'); %value of each cluster
            
            Label_matrix(cluster_index) = cluster_friction; % assign values to each cluster
            
        end
        % step 2 : find the boundaries using the boundarymask functon
        Label_matrix(Label_matrix==0) =1;
        Label_matrix_pad = padarray(Label_matrix,[1 1],0,'both');
        
        boundary_pad = boundarymask(Label_matrix_pad);
        boundary = boundary_pad(2:end-1,2:end-1);
        
        figure(44477)
        %     imshow(label2rgb(L))
        clf
        Label_matrix(Label_matrix==1) =NaN;
        image([FrictionGrid_measure.position_s(1,1) FrictionGrid_measure.position_s(1,end)],...
            [FrictionGrid_measure.position_t(1,1) FrictionGrid_measure.position_t(end,1)],...
            Label_matrix,'CDataMapping','scaled')
        %     clf
        %     im = image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
        %         [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
        %         labeloverlay(FrictionGrid_ref.friction,boundary,'Transparency',0));
        
        figure(44478)
        %     imshow(label2rgb(L))
        clf
        surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,Label_matrix);
        hold on
        index_ref = 39;
        plot3(FrictionGrid_ref.position_s(index_ref,:),FrictionGrid_ref.position_t(index_ref,:),ones(size(FrictionGrid_ref.position_t(index_ref,:))),'k--','LineWidth',2)
        index_lane_center_lateral=[20 58];
        for i= 1:length(index_lane_center_lateral)
            plot3(FrictionGrid_ref.position_s(index_lane_center_lateral(i),:),FrictionGrid_ref.position_t(index_lane_center_lateral(i),:),ones(size(FrictionGrid_ref.position_t(index_lane_center_lateral(i),:))),'k-.','LineWidth',1)
        end
        % axis tight
        % grid on
        colormap parula
        colorbar
        % shading interp
        shading flat
        lighting phong
        view(0,90)
        zlim([0 1])
        caxis([0.1,0.9])
        % axis equal
        box on
        xlabel('station [m]')
        ylabel('traversal [m]')
        zlabel('friction\_coeficient')
        
        figure(44485) % original and cluster data
        clf
        surf(FrictionGrid_measure.position_s,FrictionGrid_measure.position_t,Label_matrix);
        
        %     scatter3(reshape(FrictionGrid_measure.position_s,[],1),reshape(FrictionGrid_measure.position_t,[],1),reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),10,reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),'.');
        hold on
        FrictionGrid_measure.boundary = double(boundary);
        FrictionGrid_measure.boundary(FrictionGrid_measure.boundary==0)= NaN;
        scatter3(reshape(FrictionGrid_measure.position_s,[],1),reshape(FrictionGrid_measure.position_t,[],1),reshape(FrictionGrid_measure.boundary,[],1),20,reshape(FrictionGrid_measure.boundary,[],1),'r.');
        colormap parula
        colorbar
        % shading interp
        shading flat
        lighting phong
        view(0,90)
        zlim([0 1])
        caxis([0.1,0.9])
        %     xlim([-5 500])
        xlabel('s[m]')
        ylabel('t[m]')
        box on
        nbs_boundariy_grid = length(find(FrictionGrid_measure.boundary ==1))
        
    elseif method_boundary ==3
        [L,N] = superpixels(FrictionGrid_ref.friction,10,'Compactness',1);
        
        mask = boundarymask(L);
        
        figure(4447)
        %     imshow(label2rgb(L))
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            L,'CDataMapping','scaled')
        
        figure(4448)
        %     imshow(label2rgb(L))
        imshow(mask)
        image([FrictionGrid_ref.position_s(1,1) FrictionGrid_ref.position_s(1,end)],...
            [FrictionGrid_ref.position_t(1,1) FrictionGrid_ref.position_t(end,1)],...
            mask,'CDataMapping','scaled')
        figure(4449)
        scatter3(reshape(FrictionGrid_measure.position_s,[],1),reshape(FrictionGrid_measure.position_t,[],1),reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),10,reshape(FrictionGrid_measure.friction_mean_fillmiss,[],1),'.');
        hold on
        FrictionGrid_measure.boundary = double(boundary);
        FrictionGrid_measure.boundary(FrictionGrid_measure.boundary==0)= NaN;
        scatter3(reshape(FrictionGrid_measure.position_s,[],1),reshape(FrictionGrid_measure.position_t,[],1),...
            reshape(mask,[],1),20,reshape(mask,[],1),'r.');
        
        view(0,90)
        zlim([0 1])
        caxis([0.1,0.9])
        %     xlim([-5 500])
        xlabel('s[m]')
        ylabel('t[m]')
        box on
        
    else
        
        nbs_cluster =6 ;
        data = [reshape(FrictionGrid_measure.position_s,[],1)/max(FrictionGrid_measure.position_s(1,:)),...
            0.5+reshape(FrictionGrid_measure.position_t,[],1)/(2*max(FrictionGrid_measure.position_t(1,:))),...
            30*reshape(FrictionGrid_measure.friction_mean_true_fillmiss,[],1)];
        [cluster_idx,cluster_centroid_locations,sumDist] = kmeans(data,nbs_cluster,'Replicates',3,...
            'Distance','cityblock','Display','final'); % sqeuclidean,  cityblock
        
        figure(55510)
        clf
        hold on
        box on
        grid on
        view(0,90)
        zlim([0 1])
        %     xlim([-5 500])
        xlabel('s[m]')
        ylabel('t[m]')
        for i_cluster=1:nbs_cluster
            cluster_index = find(cluster_idx == i_cluster);
            scatter3(FrictionGrid_measure.position_s(cluster_index),...
                FrictionGrid_measure.position_t(cluster_index),...
                FrictionGrid_measure.friction_mean_true_fillmiss(cluster_index),10,'.');
        end
        %     axis tight
    end
end

%% play a music at the end of a code
load chirp
sound(y,Fs)
load handel
sound(y,Fs)