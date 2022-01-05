function depthArray = reconstruction_triangulation(pointArray1, pointArray2, intrinsic, R, T)
% input: two arrays of matched points, intrinsic matrix, fundamental matrix
% output: array of depth for the matched points
depthArray = [];
for i=1:size(pointArray1,1)
    p1 = intrinsic \ [pointArray1(i,:) 1]';
    p2 = intrinsic \ [pointArray2(i,:) 1]';
    depthArray = [depthArray get_depth(p1, p2, R, T)];
end
end

function depth = get_depth(p1, p2, R, T)
% solve for equation apl-bRpr+c(pl x Rpr) = T
M = [p1 -R'*p2 cross(p1, R'*p2)];
abc = M \ T;
a0 = abc(1);
b0 = abc(2);

p1 = a0 * p1;
p2 = T + b0*R'*p2;
midpoint = (p1 + p2) / 2;
depth = midpoint(3);
end