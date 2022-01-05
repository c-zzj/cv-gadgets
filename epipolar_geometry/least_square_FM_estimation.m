function F = least_square_FM_estimation(pointArray1, pointArray2)
% pointArray is of the form [x1 y1; ... ; x8 y8]

% obtain the normalization matrices M1 and M2
x1Mean = mean(pointArray1(:,1));
x2Mean = mean(pointArray2(:,1));
y1Mean = mean(pointArray1(:,2));
y2Mean = mean(pointArray2(:,2));

p1XDiff = pointArray1(:,1) - x1Mean;
p1YDiff = pointArray1(:,2) - y1Mean;
p2XDiff = pointArray2(:,1) - x2Mean;
p2YDiff = pointArray2(:,2) - y2Mean;

std1Squared = (p1XDiff'*p1XDiff + p1YDiff'*p1YDiff) / (2*size(pointArray1, 1));
std2Squared = (p2XDiff'*p2XDiff + p2YDiff'*p2YDiff) / (2*size(pointArray2, 1));
std1 = sqrt(std1Squared);
std2 = sqrt(std2Squared);
pointArray1Homogeneous = [pointArray1 ones(size(pointArray1, 1),1)];
pointArray2Homogeneous = [pointArray2 ones(size(pointArray2, 1),1)];
M1 = [1/std1 0 -x1Mean/std1; 0 1/std1 -y1Mean/std1; 0 0 1];
M2 = [1/std2 0 -x2Mean/std2; 0 1/std2 -y2Mean/std2; 0 0 1];

% normalize the point indices
pointArray1 = (M1 * pointArray1Homogeneous')';
pointArray2 = (M2 * pointArray2Homogeneous')';

% build the array A to solve least square
x1x2 = pointArray1(:,1) .* pointArray2(:,1);
y1x2 = pointArray1(:,2) .* pointArray2(:,1);
x2 = pointArray2(:,1);
x1y2 = pointArray1(:,1) .* pointArray2(:,2);
y1y2 = pointArray1(:,2) .* pointArray2(:,2);
y2 = pointArray2(:,2);
x1 = pointArray1(:,1);
y1 = pointArray1(:,2);
if size(pointArray1,1) > 8 && size(pointArray2,1) > 8
    A = [x1x2 y1x2 x2 x1y2 y1y2 y2 x1 y1 ones(size(pointArray1,1),1)];
else
    A = [x1x2 y1x2 x2 x1y2 y1y2 y2 x1 y1 ones(size(pointArray1,1),1);
        zeros(1,9)];
end
% solve for normalized F
[U, S, V] = svd(A);
FNormalized = V(:,9);
FNormalized = [FNormalized(1:3), FNormalized(4:6), FNormalized(7:9)]';

% force normalized F to have rank 2
[U, S, V] = svd(FNormalized);
S(3,3) = 0;
FNormalized = U*S*V';

% obtain F by combining normalization matrices and the normalized F
F = M2' *  FNormalized * M1;
F = F / norm(F);
end

