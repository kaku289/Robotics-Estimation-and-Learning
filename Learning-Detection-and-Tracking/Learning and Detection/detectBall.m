% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
mu = [150.2015  144.7533   60.0073]';
covar = [166.7231  109.4645 -173.8674;
         109.4645  126.5266 -158.6514;
        -173.8674 -158.6514  303.4564];
thre = 1/((2*pi)^1.5*det(covar)^0.5);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
I_d = double(I);
p = zeros(size(I_d,1),size(I_d,2));
for ct_r=1:size(I_d,1)
    for ct_c=1:size(I_d,2)
        dummy = I_d(ct_r,ct_c,:);
        difference = dummy(:)-mu;
        p(ct_r,ct_c) = exp(-0.5*difference'*((covar)\difference))/((2*pi)^1.5*det(covar)^0.5);
%         segI(ct_r,ct_c) = p>thre;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

segI = p>thre/10;
% figure;
% imshow(segI);
% figure;
% imshow(I);

% Compute connected components
CC = bwconncomp(segI);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);

% show the centroid
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
