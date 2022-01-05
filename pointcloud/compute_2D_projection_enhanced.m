function [projectedDepthImage,projectedRGBImage]=compute_2D_projection_enhanced(imageNumber,omegaT,rotationAxis,translationVector)

% load stuff
addpath(num2str(imageNumber));

rgbImageFileName = strcat('rgbImage_',num2str(imageNumber),'.jpg');
depthImageFileName = strcat('depthImage_',num2str(imageNumber),'.png');
extrinsicFileName = strcat('extrinsic_',num2str(imageNumber),'.txt');
intrinsicsFileName = strcat('intrinsics_',num2str(imageNumber),'.txt');

rgbImage = imread(rgbImageFileName);
depthImage = imread(depthImageFileName);
extrinsic_matrix = load(extrinsicFileName);
intrinsic_matrix = load(intrinsicsFileName);

% decompose the extrinsic matrix into rotation and translation
extrinsic_rotation = extrinsic_matrix(:,1:3);
extrinsic_translation = extrinsic_matrix(:,4);

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


% upscale the image by 4 times
scale = 4;
depthImage = imresize(depthImage, scale, "nearest");
rgbImage = imresize(rgbImage, scale);

% convert type
rgbImage = double(rgbImage);
depthImage = double(depthImage);

% get the image coordinates
% each pixel x_0,y_0 is indexed by x_0 + (y_0 - 1) * width
[height, width] = size(depthImage);

x_coords = repmat(1:width,[1,height]);
y_coords = repmat(1:height,[width,1]);
y_coords = y_coords(:)';
z_coords = ones(1,width*height);

% numerical scale to original image size
% now we have interpolated points between each pair of pixels
height = height / scale;
width = width / scale;
x_coords = x_coords / scale;
y_coords = y_coords / scale;

img_coords = [x_coords; y_coords; z_coords];
% multiply each point by its depth, given by the depth image
% get the flattend matrix of depth
depth = depthImage';
depth = repmat(depth(:)',[3 1]);
img_coords = img_coords .* depth;

% for each image coordinates, multiply inverse of intrinsic matrix to
% get the uncalibrated camera coordinates

% perform rotation and translation, and intrinsic matrix
A = intrinsic_matrix * rotation / intrinsic_matrix;
B = intrinsic_matrix * translationVector;
new_image_coords = A * img_coords + B;

% sort the coordinates based on z in descending order
% this is because two parts of image might overlap after rotation, and
% sorting it keeps the part closer to camera when later filling the rgb and
% depth images (closer parts are filled later)
r = rgbImage(:,:,1)';
g = rgbImage(:,:,2)';
b = rgbImage(:,:,3)';
new_image_coords = [new_image_coords; r(:)'; g(:)'; b(:)'];
[~, order] = sort(new_image_coords(3,:), 'descend');
new_image_coords = new_image_coords(:, order);

% pull out color and depth
rgb = new_image_coords(4:6,:);
z = new_image_coords(3,:);
new_image_coords = new_image_coords(1:3,:);

% project the image to 2d
new_image_coords = new_image_coords ./ [repmat(z,[2 1]); ones(size(z))];


% fill in the image multiple times with different rounding methods to
% in order to add more pixels
depth_img = zeros(height,width);
rgb_img = zeros(height,width,3);

x = new_image_coords(1,:);
y = new_image_coords(2,:);

xp = uint16(x+3);
yp = uint16(y+3);
xm = uint16(x-3);
ym = uint16(y-3);
[depth_img, rgb_img] = fill_imgs(xp, yp, z, rgb, height, width, depth_img, rgb_img);
[depth_img, rgb_img] = fill_imgs(xm, ym, z, rgb, height, width, depth_img, rgb_img);
[depth_img, rgb_img] = fill_imgs(xp, ym, z, rgb, height, width, depth_img, rgb_img);
[depth_img, rgb_img] = fill_imgs(xm, yp, z, rgb, height, width, depth_img, rgb_img);

xp = uint16(x+1.5);
yp = uint16(y+1.5);
xm = uint16(x-1.5);
ym = uint16(y-1.5);
[depth_img, rgb_img] = fill_imgs(xp, yp, z, rgb, height, width, depth_img, rgb_img);
[depth_img, rgb_img] = fill_imgs(xm, ym, z, rgb, height, width, depth_img, rgb_img);
[depth_img, rgb_img] = fill_imgs(xp, ym, z, rgb, height, width, depth_img, rgb_img);
[depth_img, rgb_img] = fill_imgs(xm, yp, z, rgb, height, width, depth_img, rgb_img);

% fill in the regular rounding last, to ensure precision
rounded_x = uint16(x);
rounded_y = uint16(y);
[depth_img, rgb_img] = fill_imgs(rounded_x, rounded_y, z, rgb, height, width, depth_img, rgb_img);

% depth_img = imresize(depth_img, 0.25);
% rgb_img = imresize(rgb_img, 0.25);

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
depth_img(sub2ind(size(prev_depth_img),y,x)) = z;
depth_img = uint16(depth_img);

% get the rgb image (fill in color for each pixel)
rgb_img = prev_rgb_img;
rgb_img(sub2ind( size(prev_rgb_img),y,x,ones(size(y)) )) = rgb(1,:);
rgb_img(sub2ind( size(prev_rgb_img),y,x,2*ones(size(y)) )) = rgb(2,:);
rgb_img(sub2ind( size(prev_rgb_img),y,x,3*ones(size(y)) )) = rgb(3,:);
rgb_img = uint8(rgb_img);
end