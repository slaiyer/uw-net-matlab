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
function volume = chainlink(N, TL, NUM)

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
  overlapFraction = 0.155;        % Overlap along edge; only for sphere packing

  %% Iterating over each individual in the vectorized input

  for i = 1 : INDIVS
  %
    % For option 3:
    inferior = false;   % Flag for current individual's feasibility status
  %}

    %%
    % Reformat each individual into a convenient 2D matrix
    % Reshape _N_(1, :) as _N2_(_NUM_, 3),
    % such that _N2_(_i_, :) = [ _Cx_ _Cy_ _Cz_ ]

    % Workaround for MATLAB's column-major matrix policy:
    N2 = reshape(N(i,:), 3, NUM).';

%     if state.Generation == 0
%       for j = 1 : NUM
%         N2(j,3) = 3000 + randi(6000);
%       end
%     end

    %% Calculating the volume of the point cloud polyhedron
    % Use Delaunay triangulation to create a tetrahedral mesh,
    % and find the facets and volume of the convex hull over it.

    DT = delaunayTriangulation(N2);
    [ facets, volume(i) ] = convexHull(DT);

    %% Checking for constraint violation
    % Ensure that there are no gaps in coverage on any edge of any facet.
    % Edge coverage is considered to be complete if the coverage ranges of
    % both nodes at the involved adjacent vertices overlap or at least meet
    % at some point on their common edge.
    %%
    % Enumerate and iterate over each adjacent vertex pair of each facet:

    numFacets = size(facets, 1);
    numVerts = size(facets, 2);

    for f = 1 : numFacets
      for v1 = 1 : numVerts
        % TODO: Investigate round-robin system for face coverage
        v2 = rem(v1, numVerts) + 1;           % Successive edge pairs
        p = [ facets(f,v1); facets(f,v2) ];   % Vertex indices
        n = [ N2(p(1),:); N2(p(2),:) ];       % Node coordinates
        range = [ TL(p(1)); TL(p(2)) ];       % Node radii

        %%
        % Calculate the separation between two adjacent vertices:

        edge = norm(n(1,:) - n(2,:));           % Euclidean distance

        %%
        % Return attenated ranges between the source and target nodes:
        range = attenuate(n, range, edge);

        %%
        % Calculate the total coverage along their common edge:
        coverage = range(1) + range(2);         % Sphere packing

        % Ensure both nodes are in each other's range:
        % coverage = min(range(1), range(2));   % Two-way communication

        overlap = coverage - edge;
        minOverlap = edge * overlapFraction;

        %% Defining the penalty for edge coverage gap
        % _volume_(_i_) = _volume_(_i_) + _edge_ - _coverage_
        % is insufficient because as the volume increases cubically,
        % it easily offsets the linear increase in penalty.

        if overlap < minOverlap
          %%
          % Option 1: Split the gap in edge coverage between
          % the two nodes proportionately and calculate
          % the volume deficits, the sum of which is the penalty.
        %{
          gap = minOverlap - overlap;
          deficit = range * gap / coverage;
          penalty = (range(1) + deficit(1)) ^ 3 - range(1) ^ 3 ...
                    + (range(2) + deficit(2)) ^ 3 - range(2) ^ 3;

          volume(i) = volume(i) - penalty;
        %}

          %%
          % Option 2: Halve the score for each gap in edge coverage:
        %{
          % volume(i) = volume(i) / 2;
        %}

          %%
          % Option 3: Reset the score to zero as soon as
          % the first edge gap is found, and deem
          % the current individual inferior.
        %
          volume(i) = 0;

          % Escape inferior individual's loop _(1/3)_:
          inferior = true;
          break
        %}
        end
      end
  %{
    % For options 1 and 2:
    end
  %}

  %
    % For option 3:

      % Escape inferior individual's loop _(2/3)_:
      if inferior == true
        break
      end
    end

    % Escape inferior individual's loop _(3/3)_:
    if inferior == true
      continue
    end
  %}
  end

%%
% Return the score vector for the input individuals:

end
