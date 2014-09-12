%% ATTENUATE
% Calculates and applies directional attenuation
% to the coverage ranges of the given nodes.
%
% Examples:
%
%   Use OPTIM_NODE_CONFIG or STRETCH_CHAINLINK as the entry point.
%
% See also OPTIM_NODE_CONFIG, STRETCH_CHAINLINK, CHAINLINK, NODE_CONFIG_VOL
%
% Copyright 2014 Sidharth Iyer (246964@gmail.com)

%% Function signature
function range = attenuate(N, TL, edge)

%% Input
%%
% _TL_: Column vector of maximum acceptable loss in intensity
% between transmission and detection
%%
% _N_: Node data for node 1 such that
% _N_(i, :) = [ _Cx_ _Cy_ _Cz_ ]
%%
% _edge_: Euclidean distance between nodes

%% Output
% _range_: Column vector of attenuated ranges in the target directions

  % Test input for stub:
  if nargin == 0
    N = [ 0, 10, 20; 30, 40, 50 ];
    TL = [ 150; 140 ];
    edge = norm(N(1,:) - N(2,:));   % Edge length
  end

  tic

  numPoints = ceil(edge) + 1;
  z = linspace(N(1,3), N(2,3), numPoints);
  alphas = zeros(numPoints, 1);

  for i = 1 : numPoints
    alphas(i) = francois_garrison(25, 35, z(i), 7, 10);
  end

  display(alphas.');

  % TODO: Investigate TL == 20 * log10(R/R_1m) + alpha * R
  syms R positive;

  attenuation = trapz(alphas, z)

  % 1. One-way transmission loss (node communications):
  % f = symfun(20 * log10(R) + attenuation * R, R);

  % 2. Two-way transmission loss (echo-based detection):
  % f = symfun(2 * (20 * log10(R) + attenuation * R), R);

  range = zeros(2, 1);

  % range(1) = eval(solve(TL(1) == f, 'Real', true, 'PrincipalValue', true));
  % range(2) = eval(solve(TL(2) == f, 'Real', true, 'PrincipalValue', true));

  range(1) = eval(solve(TL(1) == 2 * (20 * log10(R) + attenuation * R), ...
                        'Real', true, 'PrincipalValue', true));
  range(2) = eval(solve(TL(2) == 2 * (20 * log10(R) + attenuation * R), ...
                        'Real', true, 'PrincipalValue', true));

  toc

  %%
  % Return the attenuated ranges for the given sources:

end
