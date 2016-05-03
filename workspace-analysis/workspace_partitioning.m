%% Load Robot Workspace Data
clear all
clc
d = '/Users/Sylvain/Documents/PFE LASA/Test1/textfiles/';

spheres = textread(strcat(d, 'density_aline.txt'));
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
pc = 600;

options = [];
options.method_name       = 'PCA';  
options.nbDimensions      =  pc;      % these are the number of eigenvectors to keep.

[proj_PCA_X, mappingPCA]         = ml_projection(spheres_sc,options);

% Visualize the first PC Eigenvectors
ml_plot_images(mappingPCA.M(:,1:pc)',[40 30])
colormap('hot')

%% BIC & AIC (K-means)
%
%   Lets see if we can find the number of clusters through RSS, AIC and BIC
%

cluster_options             = [];
cluster_options.method_name = 'kmeans';
repeats                     = 3;
Ks                          = [1:500:2000];

[mus, stds]                 = ml_clustering_optimise(proj_PCA_X,Ks,repeats,cluster_options);
%% Plot RSS

figure;
errorbar(Ks,mus(1,:),stds(1,:),'--s');
legend('rss')
xlabel('index k')
grid on

%% k-means clustering
cluster_options.method_name = 'kmeans';
cluster_options.K           = 500;
[result]                    = ml_clustering(proj_PCA_X,cluster_options);
sphere_labels = result.labels;

figure;
for ii=1:cluster_options.K        
    plot3(centers(sphere_labels==ii,1), centers(sphere_labels==ii,2), centers(sphere_labels==ii,3), '*','Color', [rand rand rand])
    hold on;
end
xlabel('x');ylabel('y');zlabel('z');
grid on


%-----> Conclusion: Cannot achieve any clustering on the lower-dimensional
% space projected from PCA

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%              Laplacian Eigenmaps          %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Do Laplacian Eigenmaps

options = [];
options.method_name       = 'Laplacian';  
options.nbDimensions      = 600;      % these are the number of eigenvectors to keep.
options.neighbors         = 7;       % this is the number of neighbors to compute the graph
options.sigma             = 0.3;

[proj_LAP_X, mappingLAP]         = ml_projection(spheres_sc,options);

%% Plot result of Laplacian

% if exist('h3','var') && isvalid(h3), delete(h3);end

plot_options            = [];
plot_options.is_eig     = true;
plot_options.labels     = [];
plot_options.title      = 'Projected data Laplacian Eigenmaps';


h3 = ml_plot_data(proj_LAP_X(:,[1 2 3]),plot_options);


%% Grid Search on the number of neighbors

options = [];
options.method_name       = 'Laplacian';
options.nbDimensions      = 600;
options.sigma             = 0.3; 

neighborsPars = [5 10 20 40 80 160 320]; 

[ eigenvalues ] = ml_neighbors_grid_search(spheres_sc,options,neighborsPars);
h_eig_lap  = ml_plot_kpca_eigenvalues(eigenvalues,neighborsPars);


%% Grid Search on the width of the kernel

options = [];
options.method_name       = 'Laplacian';
options.nbDimensions      = 600;
options.neighbors         = 7; 

sigmaPars = [0.1 : 0.05: 1]; 

[ eigenvalues ] = ml_kernel_lap_grid_search(spheres_sc,options,sigmaPars);

h_eig_lap2  = ml_plot_kpca_eigenvalues(eigenvalues,sigmaPars);

%% RUN Laplacian Projection with Choosen Values
opt_eigen_vec = 15;
opt_neighbors = 7;
opt_width     = 0.25;

options = [];
options.method_name       = 'Laplacian';  
options.nbDimensions      = opt_eigen_vec;       % these are the number of eigenvectors to keep.
options.neighbors         = opt_neighbors;       % this is the number of neighbors to compute the graph
options.sigma             = opt_width;

[proj_LAP_X, mappingLAP]         = ml_projection(spheres_sc,options);

%% BIC & AIC (K-means)
%
%   Lets see if we can find the number of clusters through RSS, AIC and BIC
%

cluster_options             = [];
cluster_options.method_name = 'kmeans';
repeats                     = 3;
Ks                          = [1:5:200];
[mus, stds]                 = ml_clustering_optimise(proj_LAP_X,Ks,repeats,cluster_options);
%% Plot RSS
figure;
errorbar(Ks,mus(1,:),stds(1,:),'--s');
legend('rss')
xlabel('index k')
grid on

%% k-means clustering
cluster_options.method_name = 'kmeans';
cluster_options.K           = 46;
[result]                    = ml_clustering(proj_LAP_X,cluster_options);
sphere_labels = result.labels;

figure;
for ii=1:cluster_options.K        
    plot3(centers(sphere_labels==ii,1), centers(sphere_labels==ii,2), centers(sphere_labels==ii,3), '*','Color', [rand rand rand])
    hold on;
end
xlabel('x');ylabel('y');zlabel('z');
grid on
