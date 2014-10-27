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
function range = attenuate(N, maxTL, edge, FG)

%% Input
%%
% _N_: Node data for node 1 and 2 such that
% _N_(i, :) = [ _Cx_ _Cy_ _Cz_ ]
%%
% _maxTL_: Column vector of maximum acceptable loss in intensity
% between transmission and detection
%%
% _edge_: Euclidean distance between nodes

%% Output
% _range_: Column vector of attenuated ranges in the target directions

  numPoints = 10;   % Integration granularity
  % numPoints = ceil(edge);       % Space intermediate points ~1m apart
  numPaths = 2;     % 1 for node communication, 2 for echo-based detection
  edgeStep = edge / numPoints;  % Step increment size
  interPts = zeros(numPoints + 1, 3);  % Intermediate points
  interPts(:, 1) = linspace(N(1, 1), N(2, 1), numPoints + 1);
  interPts(:, 2) = linspace(N(1, 2), N(2, 2), numPoints + 1);
  interPts(:, 3) = linspace(N(1, 3), N(2, 3), numPoints + 1);
  range = zeros(size(maxTL));
  absorption = 0;
%   display('6');
  absrpStep = edgeStep .* FG(interPts);
%   display('7');

  % Start from first node:
  for pt = 1 : numPoints + 1
    range(1) = range(1) + edgeStep;     % Increment old range
    % absorption = absorption ...
    %              + francois_garrison(25, 35, pts(pt, 3), 8, 10) * edgeStep;
    absorption = absorption + absrpStep(pt);

    % Test new range against given maximum acceptable losses:
    if maxTL(1) < numPaths * (20 * log10(range(1)) + absorption) ...
       || range(1) > edge
      range(1) = range(1) - edgeStep;   % Undo last increment if overshot
      break
    end
  end
%   display('8');
  absorption = 0;

  % Start from other node:
  for pt = numPoints + 1 : -1 : 1
    % Short-circuit if ultimate goal (complete edge coverage) is satisfied
    % Skip entire conditional block if overlapping is required
    % if j <= numPoints && range(1) + range(2) > edge
    % if range(1) + range(2) > edge
    %   return
    % end

    range(2) = range(2) + edgeStep;     % Increment old range
    % absorption = absorption ...
    %              + francois_garrison(25, 35, interPts(pt, 3), 8, 10) * edgeStep;
    absorption = absorption + absrpStep(pt);

    % Test new range against given maximum acceptable losses:
    if maxTL(2) < numPaths * (20 * log10(range(2)) + absorption) ...
       || range(2) > edge
      range(2) = range(2) - edgeStep;   % Undo last increment if overshot
      break
    end
  end

%%
% Return the attenuated ranges for the given sources:
end
