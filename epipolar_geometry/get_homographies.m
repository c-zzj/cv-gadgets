function [H1, H2] = get_homographies(F)
[e1, e2] = get_epipoles(F);
H1 = homography1(e1);
H2 = homography1(e2);
end

function [e1, e2] = get_epipoles(F)

e1 = null(F);
e2 = null(F');
e1 = e1 / e1(3);
e2 = e2 / e2(3);
disp(e1)
disp(e2)
end

function H = homography1(e)
ex = e(1);
ey = e(2);
H = [1 0 0; -ey/ex 1 0; -1/ex 0 1];
end

function H = homography2(e)
ex = e(1);
ey = e(2);
H = [1 0 0; 0 1 0; -1/(ex^2+ey^2) 0 1] * [ex ey 0; -ey ex 0; 0 0 1];
end