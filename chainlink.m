%% CHAINLINK
% Calculates the volume and separations for the given node configuration.

%% Function signature
function volume = chainlink(N, R, NUM)
%%
% Objective: Maximize volume of polyhedron defined by
% a set of coordinates _N_(_r_, :), subject to edge coverage constraints.
%%
% Input: Vectorized array of individuals _N_(_INDIVS_, 3 * _NUM_),
% such that _N_(_r_, :) = [ _Cx1_ _Cy1_ _Cz1_ ... _Cx<NUM>_ _Cy<NUM>_ _Cz<NUM>_ ],
% vector of base node coverage radii _R_, number of nodes _N_.
%%
% Output: Column vector _volume_ of individual scores.

% TODO: Implement anisotropic attenuation

    %% Preparing the output vector for vectorized input
    INDIVS = size(N, 1);        % Number of incoming individuals
    volume = zeros(INDIVS, 1);	% Column vector for vectorized scores

    %% Iterating over each individual in the vectorized input
    for i = 1 : INDIVS
        inferior = false;	% Flag for current individual's status

        %%
        % Reformat each individual into a convenient 2D matrix
        % Reshape _N_(1, :) as _N2_(_NUM_, 3),
        % such that row vector _N2_(_r_, :) = [ _Cx_ _Cy_ _Cz_ ]

        % Workaround for MATLAB's column-major matrix policy
        N2 = reshape(N(i,:), 3, NUM)';

        %% Calculating the volume of the point cloud polyhedron
        % Use Delaunay triangulation to create a tetrahedral mesh,
        % and find the facets and volume of the convex hull over it.

        % TODO: Investigate concave polyhedron volume calculation algorithms
        DT = delaunayTriangulation(N2);
        [ facets, volume(i) ] = convexHull(DT);

        %% Checking for constraint violation
        % Ensure that there are no gaps in coverage on any edge of any facet.
        % Edge coverage is considered to be complete if the coverage ranges of
        % both nodes at the involved adjacent vertices overlap or at least meet
        % at some point on their common edge.

        % Enumerate and iterate over each adjacent vertex pair of each facet:
        numFacets = size(facets, 1);
        numVerts = size(facets, 2);

        for f = 1 : numFacets
            for v1 = 1 : numVerts
                v2 = rem(v1, numVerts) + 1;     % Round robin pair traversal
                p1 = facets(f,v1);
                p2 = facets(f,v2);

                %%
                % Calculate the separation between two adjacent vertices
                % and the total coverage along their common edge:
                edge = norm(N2(p1,:) - N2(p2,:));
                edgeCover = R(p1) + R(p2);

                %% Defining the penalty for edge coverage gap
                % _volume_(_i_) = _volume_(_i_) + _edge_ - _edgeCover_
                % is insufficient because as the volume increases cubically,
                % it easily offsets the linear increase in penalty.
                % Hence, reset the score to zero as soon as
                % the first edge gap is found, and
                % escape from the inferior individual's loop.

                if edgeCover < edge
                    % TODO: Alternatives to zero-tolerance scheme for gaps
                    volume(i) = 0;
                    %%
                    % Escape inferior individual's loop _(1/3)_:
                    inferior = true;
                    break
                end
            end
            %%
            % Escape inferior individual's loop _(2/3)_:
            if inferior == true
                break
            end
        end
        %%
        % Escape inferior individual's loop _(3/3)_:
        if inferior == true
            continue
        end
    end

%% Returning the score vector for the input individuals
end
