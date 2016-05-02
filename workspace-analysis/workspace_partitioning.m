%% Load Robot Workspace Data
clear all
clc
d = '/home/nadiafigueroa/dev/MATLAB_2014a/throwing-robot/workspace-data/';

spheres = textread(strcat(d, 'density_align.txt'));
centers = textread(strcat(d, 'center.txt'));

%% Scale spheres

[minmax_vel]  = minmax(spheres);
vel_scaling = ones(length(spheres),1)./(minmax_vel(:,2) - minmax_vel(:,1));
spheres_sc   = (spheres - repmat(minmax_vel(:,1), [1 size(spheres,2)])).*repmat(vel_scaling, [1 size(spheres,2)]) ;


%%  Visualize 2 random spheres
close all
id1 = 1; id2 = 1000;
figure;
subplot(1,2,1)
imagesc(reshape(spheres_sc(id1,:), [30 40]));
title(sprintf('Sphere x: %g, y: %g, z: %g',centers(id1,1),centers(id1,2), centers(id1,3)))
colormap('hot');
colorbar
axis square

subplot(1,2,2)
imagesc(reshape(spheres_sc(id2,:), [30 40]));
title(sprintf('Sphere x: %g, y: %g, z: %g',centers(id2,1),centers(id2,2), centers(id2,3)))
colormap('hot');
colorbar
axis square

%% Plot Sphere images and correpsonding 3d positions in Workspace
%   Visualise the images
%
close all

% Plot sphere datapoints and sphere projections from first 100 indices
idx = [1:100];
plot_sphere_images(spheres_sc, centers, idx, [40 30]);

idx = [1500:1600];
plot_sphere_images(spheres_sc, centers, idx, [40 30]);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%               Do PCA Analysis         %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Start by computing PCA will max number of eigenvectors to analyze
% possible reduction
options = [];
options.method_name       = 'PCA';  
options.nbDimensions      = 1200;      % these are the number of eigenvectors to keep.

[proj_PCA_X, mappingPCA]         = ml_projection(spheres_sc,options);

% Visualize Eigenvalues
eigs_var = mappingPCA.lambda./sum(mappingPCA.lambda);

figure;
plot(cumsum(eigs_var), '--r') 
title('PCA Eigenvalue Analysis')
ylabel('% variance explained')
xlabel('eigenvalue index')
grid on

%% From eigenvalue analysis we choose 300 (explains 89% variance of data)
pc = 300;

options = [];
options.method_name       = 'PCA';  
options.nbDimensions      =  pc;      % these are the number of eigenvectors to keep.

[proj_PCA_X, mappingPCA]         = ml_projection(spheres_sc,options);

% Visualize the first 100 Eigenvectors
ml_plot_images(mappingPCA.M(:,1:100)',[40 30])


%% BIC & AIC (K-means)
%
%   Lets see if we can find the number of clusters through RSS, AIC and BIC
%

cluster_options             = [];
cluster_options.method_name = 'kmeans';
repeats                     = 5;
Ks                          = 1:10:300;

[mus, stds]                 = ml_clustering_optimise(proj_PCA_X,Ks,repeats,cluster_options);
%% Plot RSS, AIC and BIC
h_bic                       = ml_plot_rss_aic_bic(mus,stds,Ks);


