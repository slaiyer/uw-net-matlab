%% STRETCH_CHAINLINK
% Uses a vectorized genetic algorithm to optimize _CHAINLINK_.

%% Function signature
function N = stretch_chainlink()
%%
% Output: Optimized node configuration _N_(_NUM_, 3),
% such that row vector _N_(_r_, :) = [ _Cx_ _Cy_ _Cz_ ]

% TODO: Input: Vector of base node coverage radii R

    %% Initializing constant data
    % Hardcoded values for testing and debugging:

    % R = [ 94 62 93 16 ]
    % R = [ 13 25 18 42 25 32 24 24 ]
    R = [ 3686 5526 4895 3450 5278 3184 3475 3967 5557 5279 4281 4853 ...
          3645 3921 3877 5598 4453 3664 5915 3775 4401 3556 5327 4905 3759 ]

    NUM = size(R, 2)	% Number of nodes

    %%
    % Set the initial population seeding range.
    %%
    % min(R) / sqrt(3) is a conservative setting, ensuring that the
    % cubic diagonal of the initial population range will fit in the
    % lowest coverage radius of all nodes.

    HALFRANGE = min(R) / (2 * sqrt(3));     % Center roughly around origin

    %% Genetic algorithm options
    % _'PopInitRange'_ sets the initial population seeding range,
    % within which the first generation is defined using _'CreationFcn'_.
    %%
    % _'Vectorized'_ specifies whether the GA is to be called with
    % multiple individuals passed to it in each iteration or not.

    % TODO: Set options optimally
    oldopts = gaoptimset(@ga);      % Load default options explicitly
    newopts = ...
        struct( ...
            'PopInitRange', [ -HALFRANGE; HALFRANGE ], ...  % { [ -10; 10 ] }
            'PlotFcns',     { @gaplotbestf }, ...           % { [] }
            'Display',      'iter', ...                     % { 'final' }
            'Vectorized',   'on' ...                        % { 'off' }
              );
    options = gaoptimset(oldopts, newopts);     % Overwrite selected parameters

    %% Invoking the genetic algorithm
    % Maximize _CHAINLINK_ by minimizing the negative of its score:
    tic    % Start timer
    objFunc = @(N) -chainlink(N, R, NUM);	% Create function handle for GA
    % Dump GA return values [ x, fval, exitFlag, output, population, scores ]:
    [ N, volume, ~, output, ~, ~ ] = ga(objFunc, 3 * NUM, options)
    toc    % Poll timer

    %%
    % Reshape _N_(1, 3 * _NUM_) as _N_(_NUM_, 3),
    % such that row vector _N_(_r_, :) = [ _Cx_ _Cy_ _Cz_ ]

    % Workaround for MATLAB's column-major matrix policy:
	N = reshape(N, 3, NUM)';

    %%
    % Calculate and map the separation between each pair of nodes.
    sep = zeros(NUM, NUM);

    for i = 1 : NUM - 1
        for j = i + 1 : NUM
            sep(i,j) = norm(N(i,:) - N(j,:));
        end
    end

    labels = cell(1, NUM);	% Row and column headers

    for i = 1 : NUM
        labels{i} = strcat('Node', num2str(i));
    end

    %%
    % Display the Euclidean distances between each pair of nodes
    % in a tabular format:
    sep = array2table(sep', 'VariableNames', labels, 'RowNames', labels)

    %% Displaying the 3D representation of the solution
    figure( ...
           'Name', 'Node configuration optimized for coverage volume', ...
           'NumberTitle', 'off' ...
          );

    %%
    % Show the scatter plot of the optimized node configuration.
    scatter3(N(:,1), N(:,2), N(:,3), '.');

    % Node coverage envelope colours
    edgeGreen = [0.8 1 0.8];
    faceGreen = [0 0.9 0];

    %%
    % Display translucent spheres depicting the coverage volume of each node.
    for i = 1 : NUM
        r = R(i);
        [ x, y, z ] = sphere(16);
        x = x * r + N(i,1);
        y = y * r + N(i,2);
        z = z * r + N(i,3);

        surface(x, y, z, ...
                'EdgeColor', edgeGreen, ...
                'FaceColor', faceGreen, ...
                'FaceAlpha', 0.1 ...
               );

        pause(0.1);     % Pause 0.1 s after each sphere to simulate animation
    end

    %%
    % Use Delaunay triangulation to create and display the tetrahedral mesh
    % for the polyhedral volume enclosed by the node configuration.
    hold on;	% Continue with current figure

    DT = delaunayTriangulation(N);
    tetramesh(DT);

%% Return volume optimized configuration for the given number of nodes
end
