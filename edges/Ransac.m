% Starter Code

% Read in the image, convert to grayscale, and detect edges.
% Creates an array edges where each row is    (x, y, cos theta, sin theta)   

img = imread("skyscrapers.jpg");
im = imresize(rgb2gray(img), 0.5);

Iedges = edge(im,'canny');
%  imgradient computes the gradient magnitude and gradient direction
%  using the Sobel filter.  
[~,grad_dir]=imgradient(im);
%  imgradient defines the gradient direction as an angle in degrees
%  with a positive angle going (CCW) from positive x axis toward
%  negative y axis.   However, the (cos theta, sin theta) formulas from the lectures define theta
%  positive to mean from positive x axis to positive y axis.  For this
%  reason,  I will flip the grad_dir variable:
grad_dir = - grad_dir;
figure
imshow(Iedges)

%Now find all the edge locations, and add their orientations (cos theta,sin theta). 
%  row, col is  y,x
[row, col] = find(Iedges);
% Each edge is a 4-tuple:   (x, y, cos theta, sin theta)   
edges = [col, row, zeros(length(row),1), zeros(length(row),1) ];
for k = 1:length(row)
     edges(k,3) = cos(grad_dir(row(k),col(k))/180.0*pi);
     edges(k,4) = sin(grad_dir(row(k),col(k))/180.0*pi);
end

% My code

% parameters
tau = 5;
delta = pi/18;
Cmin = 100;
M = 6000;
% this image is the edges image created earlier
%saveas(gcf,"report/31.jpg")

line_models = find_lines_a(tau, delta, Cmin, M, edges);
plot_line_models(line_models, img, 150, 20)
%saveas(gcf,"report/32.jpg")

line_models = find_lines_b(tau, delta, Cmin, M, edges);
plot_line_models(line_models, img, 150, 20)
%saveas(gcf,"report/33.jpg")

function line_models = find_lines_a(tau, delta, Cmin, M, edges)
    edges = repmat(edges,1);
    line_models = [];
    for k=1 : M
        rand_index = randi([1,size(edges,1)]);
        rand_edge = edges(rand_index,:);
        x = rand_edge(1);
        y = rand_edge(2);
        cos_theta = rand_edge(3);
        sin_theta = rand_edge(4);
        r = x*cos_theta + y*sin_theta;
        angle = atan(sin_theta/cos_theta);
        % find perpendicular distance from each point to the line
        distances = abs(cos_theta*edges(:,1)+sin_theta*edges(:,2) - r);
        % find angle between each edge and the chosen edge
        angles = abs(atan(edges(:,4)./edges(:,3)) - angle);
        % indices of neighboring edges
        neighbors = find(distances < tau & angles < delta);
        neighbors = neighbors';
        if length(neighbors) >= Cmin
            % calculate the mean and attach to the list
            x_mean = mean(edges(neighbors,1));
            y_mean = mean(edges(neighbors,2));
            angle_mean = mean(atan(edges(neighbors,4)./edges(neighbors,3)));
            % add the line model to be displayed later
            line_models = [line_models;x_mean, y_mean, angle_mean-pi/2];
            cos_mean =  cos(angle_mean);
            sin_mean = sin(angle_mean);
            
            % remove the consensus points
            edges(neighbors,:) = [];
            
            % add back the fitted edge
            edges = [edges; x_mean, y_mean,...
                cos_mean, sin_mean];
        end
    end
end

function line_models = find_lines_b(tau, delta, Cmin, M, edges)
    edges = repmat(edges,1);
    line_models = [];
    for k=1 : M
        rand_index = randi([1,size(edges,1)]);
        rand_edge = edges(rand_index,:);
        x = rand_edge(1);
        y = rand_edge(2);
        cos_theta = rand_edge(3);
        sin_theta = rand_edge(4);
        r = x*cos_theta + y*sin_theta;
        angle = atan(sin_theta/cos_theta);
        % find perpendicular distance from each point to the line
        distances = abs(cos_theta*edges(:,1)+sin_theta*edges(:,2)- r);
        % find angle between each edge and the chosen edge
        angles = abs(atan(edges(:,4)./edges(:,3)) - angle);
        
        % indices of neighboring edges
        neighbors = find(distances < tau & angles < delta);
        neighbors = neighbors';
        if length(neighbors) >= Cmin
            x_mean = mean(edges(neighbors,1));
            y_mean = mean(edges(neighbors,2));
            
            % least square fitting
            % find eigenvector with lower eigenvalue
            X = [edges(neighbors,1)] - x_mean;
            Y = [edges(neighbors,2)] - y_mean;
            A = [X Y];
            [V, D] = eig(A'*A);
            min_i = 1;
            if D(2,2) < D(1,1)
                min_i = 2;
            end
            cos_theta = V(1,min_i);
            sin_theta = V(2,min_i);
            
            % add the fitted line to be displayed later
            line_models = [line_models; x_mean, y_mean,...
                atan(sin_theta/cos_theta)-pi/2];
            
            % remove the consensus points
            edges(neighbors,:) = [];
            % add back the fitted edge
            edges = [edges; x_mean, y_mean, cos_theta, sin_theta];
        end
    end
end

% num_models means the last n lines to draw
function plot_line_models(line_models, img, line_length, num_models)
    if mod(line_length,2)
        line_length = line_length+1;
    end
    num_models = min(num_models, size(line_models,1));
    for k=1 : num_models%size(line_models,1)
        x = line_models(size(line_models,1)-k+1, 1);
        y = line_models(size(line_models,1)-k+1, 2);
        theta = line_models(size(line_models,1)-k+1, 3);
        % set two points on opposite directions of the line
        p1_x = x + round(line_length/2*cos(theta));
        p1_y = y + round(line_length/2*sin(theta));
        p2_x = x - round(line_length/2*cos(theta));
        p2_y = y - round(line_length/2*sin(theta));
        
        % times 2 for each index because the grayscale image was resized
        img = insertShape(img,'Line',[2*p1_x 2*p1_y 2*p2_x 2*p2_y],...
            'LineWidth',2,'Color','red');
    end
    figure
    imshow(img);
end