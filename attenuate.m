function range = attenuate(R, S, T)
% range:    Column vector of attenuated ranges
%           in the direction specified by corresponding UV
% R:        Column vector of base coverage radii
% S:        Row-major matrix of source node coordinates
% T:        Row-major matrix of target node coordinates
%
% Iterate row-wise over input arrays
%     Calculate unit vector UV(i) from S(i) to T(i)
%     Use data sets to integrate attenuation factor along UV(i)
%     Apply attenuation factor to R(i) and save as range(i)
% Return range(:)
