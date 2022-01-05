function depthImage = depth_reconstruction(im1, im2, Tx, f, F)
Vx = Lucas_Kanade_1d(im1, im2);
depthImage = (f*Tx) ./ Vx;
end


function Vx = Lucas_Kanade_1d(im1,im2)
im1 = single(im2gray(im1));
im2 = single(im2gray(im2));

% complete the code here and define Vx, Vy

% get the partial derivative on x and y directions for each pixel on the
% image convolved with a gaussian
g = fspecial('gaussian', 7,1);
im1 = conv2(im1, g, 'same');
im2 = conv2(im2, g, 'same');
im_dx = conv2(im1, [0.5 0 -0.5], 'same');
im_dx_squared = im_dx .* im_dx;

% get image convolved with gaussian
im1_blurred = conv2(im1, g, 'same');
im2_blurred = conv2(im2, g, 'same');

im_diff = im1_blurred - im2_blurred;

window_size = 31;
neighbor_sum = ones(1, window_size);
% get entries for the equation
sec_moment_1 = conv2(im_dx_squared, neighbor_sum, 'same');
result_vec_1 = conv2(im_diff .* im_dx, neighbor_sum, 'same');
Vx = -(result_vec_1 ./ sec_moment_1);
end

function Vx = Lucas_Kanade_2d(im1,im2)
im1 = single(im2gray(im1));
im2 = single(im2gray(im2));

% complete the code here and define Vx, Vy

% get the partial derivative on x and y directions for each pixel on the
% image convolved with a gaussian
g = fspecial('gaussian', 15, 2);
im1 = conv2(im1, g, 'same');
im2 = conv2(im2, g, 'same');
im_dx = conv2(im1, [0.5 0 -0.5], 'same');
im_dy = conv2(im1, [0.5;0;-0.5], 'same');
im_dx_squared = im_dx .* im_dx;
im_dy_squared = im_dy .* im_dy;
im_dx_times_dy = im_dx .* im_dy;


im_diff = im1 - im2;

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

Vx = -(result_vec_1 .* sec_moment_inverse_1 + (result_vec_2 .* sec_moment_inverse_23));
%Vy = -(result_vec_1 .* sec_moment_inverse_23 + (result_vec_2 .* sec_moment_inverse_4));
end

function result = gaussian(x, mean, sigma)
    result = (1./(sigma.*sqrt(2.*pi)))*exp(-(((x-mean)./sigma).^2)./2); 
end

function f = gaussian1d(sigma_x, half_size)
    % populate the filter with gaussian distribution
    f = -half_size*sigma_x:1:half_size*sigma_x;
    for i=1:length(f)
        f(i) = gaussian(f(i), 0, sigma_x);
    end

    % normalize the filter s.t. entries sum up to 1
    normalize = @(x)(x./sum(f));
    f = arrayfun(normalize, f);
end