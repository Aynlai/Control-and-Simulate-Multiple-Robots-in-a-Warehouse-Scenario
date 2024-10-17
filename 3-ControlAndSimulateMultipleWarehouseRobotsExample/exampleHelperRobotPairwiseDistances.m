function d = exampleHelperRobotPairwiseDistances(pose1, pose2)
%% Given size of a pose as (1, m) and size of pose1, pose2 as (numObs1, m) ,
%and (numObs2, m), respectvely, the
%function returns d of size (numObs1, numObs2) which is the euclidean distance between each pair of
%observations in pose1 and pose2

if(size(pose1, 2) ~= size(pose2, 2))
	error('pose1 and pose2 must have the same number of columns');
end

numObs1 = size(pose1, 1);
numObs2 = size(pose2, 1);
d = zeros(numObs1, numObs2);

for r = 1 : numObs1
	poseDiff = pose2 - repmat(pose1(r, :), numObs2, 1);
	d(r, :) = sqrt(diag(poseDiff * poseDiff'));
end
end
