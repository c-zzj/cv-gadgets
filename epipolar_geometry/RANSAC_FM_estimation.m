function F = RANSAC_FM_estimation(im1, im2, consensusThreshold, epsilon)
[matchedLeft, matchedRight] = SURF_matched_points(im1, im2);

F = [0 0 0; 0 0 0; 0 0 0];
consensusLeft= [];
while size(consensusLeft,1)-8 < consensusThreshold
    randomIndices = randperm(size(matchedLeft,1));
    firstEight = randomIndices(1:8);
    eightLeft = matchedLeft(firstEight,:);
    eightRight = matchedRight(firstEight,:);
    F = least_square_FM_estimation(eightLeft, eightRight);
    consensusLeft= [];
    consensusRight = [];
    for i=1:size(matchedLeft,1)
        pointLeft = matchedLeft(i,:);
        pointRight = matchedRight(i,:);
        if abs([pointRight 1] * F * [pointLeft 1]') < epsilon
            consensusLeft = [consensusLeft; pointLeft];
            consensusRight = [consensusRight; pointRight];
        end
    end
end
F = least_square_FM_estimation(consensusLeft, consensusRight);

end
