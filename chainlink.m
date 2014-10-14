%% CHAINLINK
% Calculates the volume and separations for the given node configuration.
%
% Examples:
%
%   Use OPTIM_NODE_CONFIG or STRETCH_CHAINLINK as the entry point.
%
% See also: OPTIM_NODE_CONFIG, STRETCH_CHAINLINK, ATTENUATE, NODE_CONFIG_VOL
%
% Copyright 2014 Sidharth Iyer (246964@gmail.com)

%% Function signature
function volume = chainlink(N, NUM, TL, bounds)

%% Input
% _N_(_INDIVS_, 3 * _NUM_): Vectorized array of individuals such that
% _N_(_i_, :) = [ _Cx1_ _Cy1_ _Cz1_ ... _Cx<NUM>_ _Cy<NUM>_ _Cz<NUM>_ ]
%%
% _TL_(_NUM_): Vector of acceptable losses in intesity
% between trasmission and detection
%%
% _NUM_: Number of nodes

%% Output
% _volume_: Column vector of individual scores

%% Objective
% Maximize the volume of the polyhedron defined by a set of coordinates
% _N_(_i_, :), subject to edge coverage constraints.

  %% Preparing the output vector for vectorized input

  INDIVS = size(N, 1);        % Number of incoming individuals
  volume = zeros(INDIVS, 1);  % Column vector for vectorized scores

  % Overlap fraction for total face coverage of equilateral triangle:
  % 2 * edge / sqrt(3) - edge == 0.1547005383792516 * edge
  overlapFraction = 0.155;    % Overlap factor along each edge

  %% Iterating over each individual in the vectorized input

  parfor ind = 1 : INDIVS
    inferior = false;   % Flag for current individual's feasibility status

    %%
    % Reformat each individual into a convenient 2D matrix
    % Reshape _N_(1, :) as _N2_(_NUM_, 3),
    % such that _N2_(_i_, :) = [ _Cx_ _Cy_ _Cz_ ]
    N2 = reshape(N(ind, :), NUM, 3);

    %% Calculating the volume of the point cloud polyhedron
    % Compute the facets and volume of the convex hull over the point cloud:
    [ facets, volume(ind) ] = convhull(N2);

    %% Checking for constraint violation
    % Ensure that there are no gaps in coverage on any edge of any facet.
    % Edge coverage is considered to be complete if the coverage ranges of
    % both nodes at the involved adjacent vertices overlap to
    % at least the specified extent.
    %%
    % Enumerate and iterate over each adjacent vertex pair of each facet:
    numFacets = size(facets, 1);
    numVerts = size(facets, 2);

    for f = 1 : numFacets
      for v1 = 1 : numVerts
        v2 = rem(v1, numVerts) + 1;             % Successive edge pairs
        p = [ facets(f, v1); facets(f, v2) ];   % Vertex indices
        n = [ N2(p(1), :); N2(p(2), :) ];       % Node coordinates
        range = [ TL(p(1)); TL(p(2)) ];         % Node radii

        %%
        % Calculate the separation between two adjacent vertices:
        edge = norm(n(1, :) - n(2, :));         % Euclidean distance

        %%
        % Return attenated ranges between the source and target nodes:
        range = attenuate(n, range, edge);

        if range(1) + range(2) - edge < edge * overlapFraction

          %% Defining the penalty for edge coverage gap
          % Reset the score to zero as soon as
          % the first edge gap is found, and deem
          % the current individual inferior:
          volume(ind) = 0;

          % Escape inferior individual's loop _(1/3)_:
          inferior = true;
          break
        end
      end

      % Escape inferior individual's loop _(2/3)_:
      if inferior == true
        break
      end
    end

    % Escape inferior individual's loop _(3/3)_:
    if inferior == true
      continue
    end

    %% Defining the penalty for not contributing or breaking bounds
    % Set the penalty to be _volume_(_i_) divided by
    % only the number of nodes on the convex hull:
    normals = cross(N2(facets(:, 1), :) - N2(facets(:, 2), :), ...
                    N2(facets(:, 1), :) - N2(facets(:, 3), :), 2);
    normals = bsxfun(@times, normals, 1 ./ sqrt(sum(normals .^ 2, 2)));
    a = N2(facets(true(numFacets, 1), 1), :);
    k = sum(bsxfun(@minus, mean(N2, 1), a) .* normals, 2) < 0;
    normals(k, :) = -normals(k, :);
    penalty = volume(ind) ...
              / (NUM - size(find(all(bsxfun(@minus, ...
                                            normals * N2', ...
                                            sum(normals .* a, 2)) ...
                                     >= -1e-13 * mean(abs(N2(:))), 1)), 2));

    %%
    % Apply the penalty once for each node found breaking bounds:
    for idx = 1 : NUM
      if N2(idx, 3) < bounds(1) || N2(idx, 3) > bounds(2)
        volume(ind) = volume(ind) - penalty;
      end
    end
  end

%%
% Return the score vector for the input individuals:
end
