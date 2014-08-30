%% STRETCH_CHAINLINK
% Uses a vectorized genetic algorithm to optimize _CHAINLINK_.

%% Function signature
function [ N, V ] = stretch_chainlink(R, NUM, verbose)
%%
% Input: Vector of base node coverage radii _R_, number of nodes _N_,
% boolean flag _verbose_ to specify output verbosity.
%%
% Output: Optimized node configuration _N_(_NUM_, 3),
% such that row vector _N_(_r_, :) = [ _Cx_ _Cy_ _Cz_ ],
% polyhedral volume _V_ enclosed by _N_.

    %% Initializing constant data
	% Set the initial population seeding range.
    %%
    % min(R) / sqrt(3) is a conservative setting, ensuring that the
    % cubic diagonal of the initial population range will fit in the
    % lowest coverage radius of all nodes.

    HALFRANGE = min(R) / (2 * sqrt(3));     % Center roughly around origin

    %% Setting the genetic algorithm options
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
            'Vectorized',   'on' ...                        % { 'off' }
              );
    options = gaoptimset(oldopts, newopts);     % Overwrite selected parameters

    if verbose == true
        options = gaoptimset(oldopts, 'Display', 'iter');
    else
        options = gaoptimset(oldopts, 'Display', 'final');
    end

    %% Invoking the genetic algorithm
    % Maximize _CHAINLINK_ by minimizing the negative of its score:
    objFunc = @(N) -chainlink(N, R, NUM);	% Create function handle for GA

    tic    % Start timer

    % Dump GA return values [ x, fval, exitFlag, output, population, scores ]:
    if verbose == true
        [ N, fval, ~, output, ~, ~ ] = ga(objFunc, 3 * NUM, options)
    else
        [ N, fval, ~, ~, ~, ~ ] = ga(objFunc, 3 * NUM, options);
    end

    toc    % Poll timer

    %% Processing the genetic algrithm output
    % Reshape _N_(1, 3 * _NUM_) as _N_(_NUM_, 3),
    % such that row vector _N_(_r_, :) = [ _Cx_ _Cy_ _Cz_ ]

    % Workaround for MATLAB's column-major matrix policy:
	N = reshape(N, 3, NUM)';

    %%
    % Return the negative of _fval_ as the positive volume of the polyhedron
    % enclosed by _N_:
    V = -fval;

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
    % in a tabular format.

    if verbose == true
        sep = array2table(sep', 'VariableNames', labels, 'RowNames', labels)
    else
        sep = array2table(sep', 'VariableNames', labels, 'RowNames', labels);
    end

    %% Displaying the 3D representation of the solution
    figTitle = [ 'Node configuration optimized for coverage volume', ...
              ', NUM: ', num2str(NUM) ];
    figure('Name', figTitle, 'NumberTitle', 'on');

    %%
    % Use Delaunay triangulation to create and display the tetrahedral mesh
    % for the polyhedral volume enclosed by the node configuration.

    DT = delaunayTriangulation(N);

    subplot(1, 2, 1);
    scatter3(N(:,1), N(:,2), N(:,3), '.');
    hold on;	% Continue with current figure
    tetramesh(DT);
    title('Node polyhedron');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    axis vis3d;

    %%
    % Display translucent spheres depicting the coverage volume of each node.

    hold on;	% Continue with current figure
    subplot(1, 2, 2);
    scatter3(N(:,1), N(:,2), N(:,3), '.');
    hold on;	% Continue with current figure
    tetramesh(DT);
    hold on;
    title('Node coverages');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    % Node coverage envelope colours
    meshGreen = [0.8 1 0.8];
    faceGreen = [0 0.9 0];

    for i = 1 : NUM
        r = R(i);
        [ x, y, z ] = sphere(16);
        x = x * r + N(i,1);
        y = y * r + N(i,2);
        z = z * r + N(i,3);

        surface(x, y, z, ...
                'EdgeColor', meshGreen, ...
                'FaceColor', faceGreen, ...
                'FaceAlpha', 0.1 ...
               );

        pause(0.1);     % Pause 0.1 s after each sphere to simulate animation
    end

    axis equal;
    axis vis3d;

%% Returning a volume-optimized configuration for the given number of nodes
end
