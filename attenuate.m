%% ATTENUATE
% Calculates and applies directional attenuation
% to the coverage ranges of the given sources.
%
% Copyright 2014 Sidharth Iyer <246964@gmail.com>
%
% Examples:
%
%   Use OPTIM_NODE_CONFIG or STRETCH_CHAINLINK as the entry point.
%
% See also OPTIM_NODE_CONFIG, STRETCH_CHAINLINK, CHAINLINK

%% Function signature
function range = attenuate(R, S, T)

%% Input
% _R_: Column vector of base node coverage radii
%%
% _S_: Vectorized array of source node coordinates such that
% _S_(_i_, :) = [ _Cx_ _Cy_ _Cz_ ]
%%
% _T_: Vectorized array of target node coordinates such that
% _T_(_i_, :) = [ _Cx_ _Cy_ _Cz_ ]

%% Output
% _range_: Column vector of attenuated ranges in the target directions

%% Pseudocode

% Iterate row-wise over input arrays
%   Calculate unit vector UV(i) from S(i) to T(i)
%   Use data sets to integrate attenuation factor along UV(i)
%   Apply attenuation factor to R(i) and save as range(i)
% Return range(:)

%%
% Return the attenuated ranges for the given sources:
  range = R;  % Dummy return value for stub

end
