function compute_point_cloud(imageNumber)



% This function provides the coordinates of the associated 3D scene point 
% (X; Y;Z) and the associated color channel values for any pixel in the 
% depth image. The output is saved in the output_file in the 
% format of a N x 6 matrix where N is the number of 3D points with 3 
% coordinates and 3 color channel values:
% 
% X_1,Y_1,Z_1,R_1,G_1,B_1
% X_2,Y_2,Z_2,R_2,G_2,B_2
% X_3,Y_3,Z_3,R_3,G_3,B_3
% X_4,Y_4,Z_4,R_4,G_4,B_4
% X_5,Y_5,Z_5,R_5,G_5,B_5
% X_6,Y_6,Z_6,R_6,G_6,B_6
% .
% .
% .
% .
%
% Output file name = 'pointCloudImage_1.mat'


% add the corresponding folder name to the path 
addpath(num2str(imageNumber));

% You can remove any inputs you think you might not need for this part:
rgbImageFileName = strcat('rgbImage_',num2str(imageNumber),'.jpg');
depthImageFileName = strcat('depthImage_',num2str(imageNumber),'.png');
extrinsicFileName = strcat('extrinsic_',num2str(imageNumber),'.txt');
intrinsicsFileName = strcat('intrinsics_',num2str(imageNumber),'.txt');

rgbImage = imread(rgbImageFileName);
depthImage = imread(depthImageFileName);
extrinsic_matrix = load(extrinsicFileName);
intrinsic_matrix = load(intrinsicsFileName);

%%% YOUR IMPLEMENTATION GOES HERE:

% convert type
rgbImage = double(rgbImage);
depthImage = double(depthImage);

% separate rotation and translation
extrinsic_rotation = extrinsic_matrix(:,1:3);
extrinsic_translation = extrinsic_matrix(:,4);

% get the image coordinates
% each pixel x_0,y_0 is indexed by x_0 + (y_0 - 1) * width
[height, width] = size(depthImage);
x_coords = repmat(1:width,[1,height]);
y_coords = repmat(1:height,[width,1]);
y_coords = y_coords(:)';
z_coords = ones(1,width*height);
img_coords = [x_coords; y_coords; z_coords];
% multiply each point by its depth, given by the depth image
% get the flattend matrix of depth
depth = depthImage';
depth = repmat(depth(:)',[3 1]);
img_coords = img_coords .* depth;

% for each image coordinates, multiply inverse of intrinsic matrix
% get the uncalibrated camera coordinates
camera_coords = intrinsic_matrix \ img_coords;

% multiply inverse of extrinsic matrix to get world coordinates
world_coords = camera_coords - extrinsic_translation;
world_coords = extrinsic_rotation \ world_coords;

% attach color
r = rgbImage(:,:,1)';
g = rgbImage(:,:,2)';
b = rgbImage(:,:,3)';
world_coords = [world_coords; r(:)'; g(:)'; b(:)'];
world_coords = world_coords';

%To save your ouptut use the following file name:
outputFileName = strcat('pointCloudImage_',num2str(imageNumber),'.mat');
% the variable world_coords is saved.
save(outputFileName, 'world_coords')
end
