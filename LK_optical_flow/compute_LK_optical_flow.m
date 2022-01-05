function [Vx,Vy] = compute_LK_optical_flow(frame_1,frame_2)

% Implementation of the Lucas Kanade algorithm to compute the
% frame to frame motion field estimates. 
% frame_1 and frame_2 are two gray frames where you are given as inputs to 
% this function and you are required to compute the motion field (Vx,Vy)
% based upon them. 

im1 = single(im2gray(frame_1));
im2 = single(im2gray(frame_2));

% complete the code here and define Vx, Vy

% get the partial derivative on x and y directions for each pixel on the
% image convolved with a gaussian
gaussian = fspecial('gaussian', 19, 1);
im1 = conv2(im1, gaussian, 'same');
im2 = conv2(im2, gaussian, 'same');
im_dx = conv2(im1, [0.5 0 -0.5], 'same');
im_dy = conv2(im1, [0.5;0;-0.5], 'same');
im_dx_squared = im_dx .* im_dx;
im_dy_squared = im_dy .* im_dy;
im_dx_times_dy = im_dx .* im_dy;

% get image convolved with gaussian
im1_blurred = conv2(im1, gaussian, 'same');
im2_blurred = conv2(im2, gaussian, 'same');

im_diff = im1_blurred - im2_blurred;

window_size = 31;
weight_filter = fspecial('gaussian', window_size, 2);
neighbor_sum = conv2(weight_filter, ones(window_size, window_size), 'same');
% get entries for the equation
sec_moment_1 = conv2(im_dx_squared, neighbor_sum, 'same');
sec_moment_23 = conv2(im_dx_times_dy, neighbor_sum, 'same');
sec_moment_4 = conv2(im_dy_squared, neighbor_sum, 'same');

result_vec_1 = conv2(im_diff .* im_dx, neighbor_sum, 'same');
result_vec_2 = conv2(im_diff .* im_dy, neighbor_sum, 'same');

% calculate Vx and Vy using the equation 
% (Vx,Vy)' = inverse(sec_moment) * diff vector
det = sec_moment_1 .* sec_moment_4 - sec_moment_23 .* sec_moment_23;
sec_moment_inverse_1 = sec_moment_4 ./ det;
sec_moment_inverse_23 = sec_moment_23 ./ det * -1;
sec_moment_inverse_4 = sec_moment_1 ./ det;
% do not mult. by -1 because it's done when displaying the quiver plot
Vx = (result_vec_1 .* sec_moment_inverse_1 + (result_vec_2 .* sec_moment_inverse_23));
Vy = (result_vec_1 .* sec_moment_inverse_23 + (result_vec_2 .* sec_moment_inverse_4));

% subsample the result
%Vx = Vx(1:window_size:end,1:window_size:end);
%Vy = Vy(1:window_size:end,1:window_size:end);
end
