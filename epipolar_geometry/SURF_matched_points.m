function [matchedLeft, matchedRight] = SURF_matched_points(im1, im2)
I1 = rgb2gray(im1);
I2 = rgb2gray(im2);

%returns a SURFPoints object, points, containing information about SURF features detected in the 2-D grayscale input image I. 
ptsLeft  = detectSURFFeatures(I1); 
ptsRight = detectSURFFeatures(I2);

%ExtractFeatures: Returns [features, validPoints], extracted feature vectors, also known as
%descriptors, and their corresponding locations.
[featuresLeft,  validPtsLeft]  = extractFeatures(I1,  ptsLeft);
[featuresRight, validPtsRight] = extractFeatures(I2, ptsRight);

%indexPairs the matching feature vectors and return the matching points
%index in featuresLeft, and featuresRight
indexPairs = matchFeatures(featuresLeft, featuresRight);

%Get the valid points using the index.
matchedLeft  = validPtsLeft(indexPairs(:,1)).Location;
matchedRight = validPtsRight(indexPairs(:,2)).Location;
end
