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
  % if nargin == 0
  %   N = 1e2 * [ 0, 10, 20; 30, 40, 50 ];
  %   TL = [ 150; 140 ];              % 185.756
  %   edge = norm(N(1,:) - N(2,:));   % Edge length
  % end

  % tic

  % numPoints = ceil(edge) + 1;
  % z = linspace(N(1,3), N(2,3), numPoints);
  % alphas = zeros(numPoints, 1);

  numPoints = 10;
  z = linspace(N(1,3), N(2,3), numPoints);
  alphas = zeros(numPoints, 1);

  for i = 1 : numPoints
    alphas(i) = francois_garrison(25, 35, z(i), 7, 10);
  end

  % display(alphas.');

  % TODO: Investigate TL == 20 * log10(R/R_1m) + alpha * R
  % syms R real positive;

  attenuation = trapz(alphas, z);

  % 1 for one-way transmission losses (node communications)
  % 2 for two-way transmission losses (echo-based detection)
  paths = 2;

  % range = zeros(size(TL));

  % range = abs(real(X)) or abs(X)?
  % lambertw(1 / 20) or lambertw(-1 / 20)?

  % range = abs(real((20 ...
  %                   * lambertw(1 / 20 ...
  %                              * (10 .^ (TL / paths) ...
  %                                 * attenuation ^ 20) .^ (1 / 20) ...
  %                              * log(10))) ...
  %                   / (attenuation * log(10))));

  range = abs(real((8.68589 ...
                    * lambertw(0.115129 ...
                               * (10 .^ (TL / paths) ...
                                  * attenuation ^ 20) .^ (1 / 20))) ...
                    / attenuation));

  % f = symfun(paths * (20 * log10(R) + attenuation * R), R);

  % range(1) = eval(solve(TL(1) == f, 'Real', true, 'PrincipalValue', true));
  % range(2) = eval(solve(TL(2) == f, 'Real', true, 'PrincipalValue', true));

  % A = solve(TL(1) == 2 * (20 * log10(R) + attenuation * R), ...
  %                       'Real', true, 'PrincipalValue', true)
  % B = solve(TL(2) == 2 * (20 * log10(R) + attenuation * R), ...
  %                       'Real', true, 'PrincipalValue', true);

  % pretty(A);
  % pretty(B);

  % range(1) = eval(A);
  % range(2) = eval(B);

  % toc

  %%
  % Return the attenuated ranges for the given sources:

end
