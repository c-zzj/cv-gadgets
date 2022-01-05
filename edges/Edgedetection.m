display = true;

% Q2a
sigma_x = 5;

f1 = filter_a(sigma_x, 3, display);
%saveas(gcf,'report/2a.jpg')
% Q2b
sigma_y = sigma_x + 2;

f2 = filter_b(f1, sigma_x, sigma_y, 3, display);
%saveas(gcf,'report/2b.jpg')

% Q2c
% see rotate_filter in the function section at the end
figure
subplot(2,2,1)
rotate_0 = rotate_filter(0, 4,4,3,display);

subplot(2,2,2)
rotate_45 = rotate_filter(45, 4,4,3,display);

subplot(2,2,3)
rotate_90 = rotate_filter(90, 4,4,3,display);

subplot(2,2,4)
rotate_135 = rotate_filter(135, 4,4,3,display);
%saveas(gcf,'report/2c.jpg')

% Q2d
% load the grayscale image of the original image
image = rgb2gray(imread('skyscrapers.jpg'));
figure
imshow(image)
d1 = find_zerocrossings(conv2(image, rotate_0));
d2 = find_zerocrossings(conv2(image, rotate_45));
d3 = find_zerocrossings(conv2(image, rotate_90));
d4 = find_zerocrossings(conv2(image, rotate_135));
figure
subplot(2,2,1),imshow(d1)
subplot(2,2,2),imshow(d2)
subplot(2,2,3),imshow(d3)
subplot(2,2,4),imshow(d4)
figure
imshow(d1)
%saveas(gcf,"report/2d1.jpg")
imshow(d2)
%saveas(gcf,"report/2d2.jpg")
imshow(d3)
%saveas(gcf,"report/2d3.jpg")
imshow(d4)
%saveas(gcf,"report/2d4.jpg")
close(gcf)
figure
imshow(4.*((d1+d2+d3+d4)./4).^1.7)
%saveas(gcf,"report/2d5.jpg")

% Q2e
log_filter = fspecial('log',40,6);
figure
imshow(find_zerocrossings(conv2(image,log_filter)))
%saveas(gcf,"report/2e1.jpg")
e1 = find_zerocrossings(conv2(image,rotate_log(0,false)));
e2 = find_zerocrossings(conv2(image,rotate_log(45,false)));
e3 = find_zerocrossings(conv2(image,rotate_log(90,false)));
e4 = find_zerocrossings(conv2(image,rotate_log(135,false)));

figure
subplot(2,2,1),imshow(e1)
subplot(2,2,2),imshow(e2)
subplot(2,2,3),imshow(e3)
subplot(2,2,4),imshow(e4)
figure
imshow(e1)
%saveas(gcf,"report/2e2.jpg")
imshow(e2)
%saveas(gcf,"report/2e3.jpg")
imshow(e3)
%saveas(gcf,"report/2e4.jpg")
imshow(e4)
%saveas(gcf,"report/2e5.jpg")
close(gcf)







% functions
% the gaussian function
function result = gaussian(x, mean, sigma)
    result = (1./(sigma.*sqrt(2.*pi)))*exp(-(((x-mean)./sigma).^2)./2); 
end

% rotate clockwise, used to interpolate coordinates before counterclockwise
% rotation
function [x,y] = rotate(xr, yr, theta)
    x = round(xr.*cos(theta) + yr.*sin(theta));
    y = round(-xr.*sin(theta) + yr.*cos(theta));
end

% filter for part a
function f = filter_a(sigma_x, half_size, display)
    % populate the filter with gaussian distribution
    f = -half_size*sigma_x:1:half_size*sigma_x;
    for i=1:length(f)
        f(i) = gaussian(f(i), 0, sigma_x);
    end

    % normalize the filter s.t. entries sum up to 1
    ratio = sum(f)./1;
    normalize = @(x)(x./(ratio*(2*half_size+1)));
    f = arrayfun(normalize, f);

    % compute second derivatives, use 'valid' to remove boundaries
    sec_diff = [1 -2 1];
    f = conv(f, sec_diff, 'valid');
    f = repmat(f, length(f), 1);

    % display the filter
    if display
        figure
        surf(f)
        colorbar
    end
end

% filter for part b
function f = filter_b(filter, sigma_x, sigma_y, half_size, display)
    % calculate the gaussian on y axis, sigma_y = sigma_x + 2
    y_gaussian = -half_size*sigma_x + 1 : 1 : half_size*sigma_x - 1; % boundaries were removed
    for i=1:length(filter)
        y_gaussian(i) = gaussian(y_gaussian(i), 0, sigma_y);
    end
    
    % multiply each row by the gaussian on y axis
    f = diag(y_gaussian) * filter;
    if display
        figure
        surf(f)
        colorbar
    end
    
end

% rotate the filter by the angle specified by building a larger one and
% interpolate the positions of each pixel on the rotated filter before
% rotation, and take values from the larger filter
function rotated = rotate_filter(angle, sigma_x, sigma_y, half_size, display)
    offset = half_size * sigma_x - 1 + 1;
    offset_before = 2*half_size * sigma_x - 1 + 1;
    rotated = zeros(2*half_size*sigma_x+1, 2*half_size*sigma_x+1);
    theta = angle ./180.*pi;
    before_rotation = filter_b(filter_a(sigma_x, 2*half_size, false),...
        sigma_x, sigma_y,2*half_size, false);
    % iterate through each point on the rotated filter
    for xr=-half_size*sigma_x+1 : half_size*sigma_x-1
        for yr=-half_size*sigma_x+1 : half_size*sigma_x-1
            % find the point before rotation
            [x,y] = rotate(xr, yr, theta);
            % set the value on the location before rotation
            rotated(yr+offset,xr+offset) = before_rotation(y+offset_before,x+offset_before);
        end
    end
    if display
        surf(rotated)
        colorbar
    end
end

% rotate the log filter
function rotated = rotate_log(angle, display)
    log_filter = fspecial('log',80,6);
    % origin is set to be 20, 20
    % on big filter the origin is 40, 40
    offset = 20;
    offset_before = 40;
    rotated = zeros(40, 40);
    theta = angle ./180.*pi;
    % iterate through each point on the rotated filter
    for xr=-19 : 20
        for yr=-19 : 20
            % find the point before rotation
            [x,y] = rotate(xr, yr, theta);
            % set the value on the location before rotation
            rotated(yr+offset,xr+offset) = log_filter(y+offset_before,x+offset_before);
        end
    end
    if display
        surf(rotated)
        colorbar
    end
end

function zero_crossings = find_zerocrossings(image)
    % shift each direction to see if its a zero crossing
    zero_crossings = sign(circshift(image,-1,2))~=sign(image);
    zero_crossings = (sign(circshift(image,-1,1))~=sign(image)) | zero_crossings;
    zero_crossings = (sign(circshift(image,[-1,-1]))~=sign(image)) | zero_crossings;
    zero_crossings = (sign(circshift(image,[-1,1]))~=sign(image)) | zero_crossings;
end