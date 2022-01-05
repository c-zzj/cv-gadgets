function [projectedDepthImage,projectedRGBImage]=compute_2D_projection(imageNumber,omegaT,rotationAxis,translationVector)
% This function create two images: 1) an image that contains the projected 
% depth value  (greyscale) and 2) an image that contains the projected 
% color of the 3D scene point.
%  
% INPUTS: 
% imageNumber
%
% omegaT is a value between 0 to pi/2 make sure not to use degrees!
%
% rotationAxis is either 'x' or 'y' or 'z'
%
% translationVector is a 3x1 vector that indicates the translation
% direction. It should have only 1 non-zero element,
% which defines the translation direction implicitly
% 
% OUTPUTS:
% projectedDepthImage: an image that contains the projected depth
% value (greyscale)
% 
% projectedRGBImage: an image that contains the projected color of the 3D 
% scene point.


% load stuff
addpath(num2str(imageNumber));
rgbImageFileName = strcat('rgbImage_',num2str(imageNumber),'.jpg');
extrinsicFileName = strcat('extrinsic_',num2str(imageNumber),'.txt');
intrinsicsFileName = strcat('intrinsics_',num2str(imageNumber),'.txt');
rgbImage = imread(rgbImageFileName);
extrinsic_matrix = load(extrinsicFileName);
intrinsic_matrix = load(intrinsicsFileName);
% decompose the extrinsic matrix into rotation and translation
extrinsic_rotation = extrinsic_matrix(:,1:3);
extrinsic_translation = extrinsic_matrix(:,4);

% load points in world coordinates
load(strcat('pointCloudImage_',num2str(imageNumber),'.mat'), 'world_coords');
% transform the points from world coordinates to camera coordinates
translated_coords = extrinsic_translation + world_coords(:,1:3)';
camera_coords = extrinsic_rotation * translated_coords;
rgb = world_coords(:,4:6)';

% hard-code rotation matrices
roll = [cos(omegaT) -sin(omegaT) 0; sin(omegaT) cos(omegaT) 0; 0 0 1];
tilt = [1 0 0; 0 cos(omegaT) -sin(omegaT); 0 sin(omegaT) cos(omegaT)];
pan = [cos(omegaT) 0 sin(omegaT); 0 1 0;  -sin(omegaT) 0 cos(omegaT)];
% default rotate around z axis
rotation = roll;
if strcmp(rotationAxis, 'x')
    rotation = tilt;
elseif strcmp(rotationAxis, 'y')
    rotation = pan;
end

% perform rotation and translation
rotated_coords = rotation * camera_coords;
translated_coords = rotated_coords + translationVector;

% perform camera calibration by multiplying the intrinsic matrix
new_image_coords = intrinsic_matrix * translated_coords;


% sort the coordinates based on z in descending order
% this is because two parts of image might overlap after rotation, and
% sorting it keeps the part closer to camera when later filling the rgb and
% depth images (closer parts are filled later)
new_image_coords = [new_image_coords; rgb];
[~, order] = sort(new_image_coords(3,:), 'descend');
new_image_coords = new_image_coords(:, order);

rgb = new_image_coords(4:6,:);
z = new_image_coords(3,:);
new_image_coords = new_image_coords(1:3,:);

% project the image to 2d
new_image_coords = new_image_coords ./ [repmat(z,[2 1]); ones(size(z))];


% get the indices for coordinates that lie in initial image frame
[height, width, channels] = size(rgbImage);

% fill in the image multiple times with different rounding methods
% in order to add more pixels
depth_img = zeros(height,width);
rgb_img = zeros(size(rgbImage));

x = new_image_coords(1,:);
y = new_image_coords(2,:);
rounded_x = uint16(x);
rounded_y = uint16(y);

% fill in the regular rounding last, to ensure precision
[depth_img, rgb_img] = fill_imgs(rounded_x, rounded_y, z, rgb, height, width, depth_img, rgb_img);

% You have to compute the following images
projectedDepthImage = depth_img;
projectedRGBImage = rgb_img;
end

function [depth_img, rgb_img] = fill_imgs(x, y, z, rgb, height, width, prev_depth_img, prev_rgb_img)
% x and y must be of integer type!

% get the indices for coordinates that lie in initial image frame
valid_points_index = x>=1 & x<= width & y>=1 & y<= height;

% filter out points that lie outside the initial image frame
x = x(valid_points_index);
y = y(valid_points_index);
z = z(valid_points_index);
rgb = rgb(:,valid_points_index);

% get the depth image (fill in depth for each pixel)
depth_img = prev_depth_img;
depth_img(sub2ind(size(depth_img),y,x)) = z;
depth_img = uint16(depth_img);

% get the rgb image (fill in color for each pixel)
rgb_img = prev_rgb_img;
rgb_img(sub2ind( size(rgb_img),y,x,ones(size(y)) )) = rgb(1,:);
rgb_img(sub2ind( size(rgb_img),y,x,2*ones(size(y)) )) = rgb(2,:);
rgb_img(sub2ind( size(rgb_img),y,x,3*ones(size(y)) )) = rgb(3,:);
rgb_img = uint8(rgb_img);
end

