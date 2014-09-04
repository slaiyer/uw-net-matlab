%% STRETCH_CHAINLINK
% Uses a vectorized genetic algorithm to optimize the node configuration
% for the given base node coverage radii.
%
% Copyright 2014 Sidharth Iyer <246964@gmail.com>
%
% Examples:
%
%   STRETCH_CHAINLINK([ 12 23 34 45 ])
%   Optimizes the node configuration for the given node radii in the vector.
%
%   STRETCH_CHAINLINK([ 12 23 34 45 ], true)
%   Optimizes the node configuration in a verbose fashion.
%
% See also OPTIM_NODE_CONFIG, CHAINLINK, ATTENUATE

%% Function signature
function [ N, V ] = stretch_chainlink(R, verbose)

%% Input
% _R_: Vector of base node coverage radii
%%
% _verbose_: Boolean flag to specify output verbosity

%% Output
% _N_(_NUM_, 3): Optimized node configuration such that
% row vector _N_(_i_, :) = [ _Cx_ _Cy_ _Cz_ ]
%%
% _V_: Polyhedral volume enclosed by _N_

  %%
  % Check for malformed arguments:

  argError = 'Malformed input arguments: Please refer to the source.';

  switch nargin
    case 1
      verbose = false;
    case 2
      if ~islogical(verbose)
        error(argError);
      end
    otherwise
      error(argError);
  end

  %% Setting genetic algorithm options
  % min(R) / sqrt(3) is a conservative setting, ensuring that
  % the cubic diagonal of the initial population range will fit
  % in the lowest coverage radius of all nodes.
  %%
  % _'TolFun'_ set the stopiing criterios based on
  % the average change of fitness function return value.
  %%
  % _'PopInitRange'_ sets the initial population seeding range,
  % within which the first generation is defined using _'CreationFcn'_.
  %%
  % _'Vectorized'_ specifies whether the GA is to be called with
  % multiple individuals passed to it in each iteration or not.

  HALFRANGE = min(R) / (2 * sqrt(3));     % Center roughly around origin

  oldopts = gaoptimset(@ga);          % Load default options
  newopts = ...
    struct( ...
            'TolFun',     1e-4, ...                         % { 1e-6 }
            'PopInitRange', [ -HALFRANGE; HALFRANGE ], ...  % { [ -10; 10 ] }
            'PlotFcns',   { @gaplotbestf }, ...             % { [] }
            'Vectorized',   'on' ...                        % { 'off' }
          );
  options = gaoptimset(oldopts, newopts);   % Overwrite selected parameters

  if verbose == true
    options = gaoptimset(options, 'Display', 'iter');
  else
    options = gaoptimset(options, 'Display', 'final');
  end

  %% Invoking the genetic algorithm
  % Maximize _CHAINLINK_ by minimizing the negative of its score:

  NUM = numel(R);   % Number of nodes

  objFunc = @(N) -chainlink(N, R, NUM);   % Create function handle for GA

  tic  % Start timer

  % Dump GA return values [ x, fval, exitFlag, output, population, scores ]:
  if verbose == true
    [ N, ~, ~, ~, ~, ~ ] = ga(objFunc, 3 * NUM, options)
  else
    [ N, ~, ~, ~, ~, ~ ] = ga(objFunc, 3 * NUM, options);
  end

  toc  % Poll timer

  %% Processing the genetic algrithm output
  % Reshape _N_(1, 3 * _NUM_) as _N_(_NUM_, 3),
  % such that row vector _N_(_i_, :) = [ _Cx_ _Cy_ _Cz_ ]

  % Workaround for MATLAB's column-major matrix policy:
  N = reshape(N, 3, NUM)';

  %%
  % Calculate and map the separation between each pair of nodes:

  if verbose == true
    sep = zeros(NUM, NUM);

    for i = 1 : NUM - 1
      for j = i + 1 : NUM
        sep(i,j) = norm(N(i,:) - N(j,:));
      end
    end

    labels = cell(1, NUM);  % Row and column headers

    for i = 1 : NUM
      labels{i} = strcat('Node', num2str(i));
    end

    sep = array2table(sep', 'VariableNames', labels, 'RowNames', labels)
  end

  %% Displaying the 3D representation of the solution

  figTitle = [ 'Node configuration optimized for coverage volume', ...
               ', NUM: ', num2str(NUM) ];
  figure('Name', figTitle, 'NumberTitle', 'on');

  %%
  % Use Delaunay triangulation to create and display the tetrahedral mesh
  % for the polyhedral volume enclosed by the node configuration:

  DT = delaunayTriangulation(N);
  [ ~, V ] = convexHull(DT);

  % Plot colours
  meshRed = [ 0.666 0 0 ];
  faceOrange = [ 1 0.9 0.7 ];
  green = [ 0.5 1 0.5 ];

  %%
  % 1. Display the polyhedral volume enclosed by _N_:

  subplot(1, 2, 1);
  scatter3(N(:,1), N(:,2), N(:,3), '.');
  hold on;  % Continue with current figure
  tetramesh( ...
            DT, ...
            'EdgeColor', meshRed, ...
            'FaceColor', faceOrange ...
           );
  title('Node polyhedron');
  xlabel('X');
  ylabel('Y');
  zlabel('Z');
  axis equal;
  axis vis3d;

  %%
  % 2. a) Display translucent spheres depicting
  %  the coverage volume of each node:

  subplot(1, 2, 2);
  scatter3(N(:,1), N(:,2), N(:,3), '.');
  hold on;  % Continue with current figure

  for i = 1 : NUM
    r = R(i);
    [ x, y, z ] = sphere(32);
    x = x * r + N(i,1);
    y = y * r + N(i,2);
    z = z * r + N(i,3);

    surface( ...
            x, y, z, ...
            'EdgeColor', green, ...
            'EdgeAlpha', 0.2, ...
            'FaceColor', green, ...
            'FaceAlpha', 0.1 ...
           );
  end

  %%
  % 2. b) Display the polyhedral volume enclosed by _N_:

  hold on;
  tetramesh( ...
            DT, ...
            'EdgeColor', meshRed, ...
            'FaceColor', faceOrange ...
           );
  title('Node coverages');
  xlabel('X');
  ylabel('Y');
  zlabel('Z');
  axis equal;
  axis vis3d;

%%
% Return a volume-optimized configuration for the given number of nodes:

end
