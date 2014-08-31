function range = attenuate(R, S, T)
%% Input
% _R_: Column vector of base node coverage radii
%%
% _S_: Row-major matrix of source node coordinates
%%
% _T_: Row-major matrix of target node coordinates

%% Output
% _range_: Column vector of attenuated ranges in the target directions

% Iterate row-wise over input arrays
%     Calculate unit vector UV(i) from S(i) to T(i)
%     Use data sets to integrate attenuation factor along UV(i)
%     Apply attenuation factor to R(i) and save as range(i)
% Return range(:)
