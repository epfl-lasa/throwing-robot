function [handle] = plot_sphere_images(spheres, centers, idx, img_dims)
%ML_PLOT_IMAGES Plot images 
%
%   input -----------------------------------------------------------------
%
%       o X     : (N x D), set of N images of dimension D.
%       o idx   : (1 X 2), desired idx
%
%       o dims  : (1 x 2), image dimensions  
%
%  
%   
%   output ----------------------------------------------------------------
%
%       o handle : handle to the image which is plotted
%
%

figure;
plot3(centers(:,1), centers(:,2), centers(:,3), '*k')
grid on;
axis equal
hold on;
plot3(centers(idx,1), centers(idx,2), centers(idx,3), '*r')
xlabel('x'); ylabel('y'); zlabel('z'); 


X = spheres(idx,:);

[N,D] = size(X);

handle = figure;

[Xs,Ys] = meshgrid(1:img_dims(1),1:img_dims(2));

if N >=100
    nbImages = 10;
else
    nbImages = (floor(sqrt(N)));
end
for j = 1:nbImages^2
    subaxis(nbImages,nbImages,j,'Spacing', 0.005, 'Padding', 0, 'Margin', 0.005);
    pcolor(Xs,Ys,reshape(X(j,:),size(Xs)));
    shading interp;
    colormap('hot');
    axis off;
end



end

