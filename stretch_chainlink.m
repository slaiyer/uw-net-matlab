%% STRETCH_CHAINLINK
% Uses a vectorized genetic algorithm to optimize the node configuration
% for the given base node coverage radii.
%
% Examples:
%
%   STRETCH_CHAINLINK(143:0.5:155, true)
%   Optimizes the node configuration for the given source levels in the vector.
%
%   STRETCH_CHAINLINK(143:0.5:155, true)
%   Optimizes the node configuration in a verbose fashion.
%
% See also: OPTIM_NODE_CONFIG, CHAINLINK, ATTENUATE, NODE_CONFIG_VOL
%
% Copyright 2014 Sidharth Iyer (246964@gmail.com)

%% Function signature
function [ N, V ] = stretch_chainlink(maxTL, verbose)

%% Input
% _maxTL_: Vector of acceptable losses in intesity
% between trasmission and detection
%%
% _verbose_: (Optional) Boolean flag to specify output verbosity

%% Output
% _N_(_NUM_, 3): Optimized node configuration such that
% _N_(_i_, :) = [ _Cx_ _Cy_ _Cz_ ]
%%
% _V_: Polyhedral volume enclosed by _N_

  %%
  % Check for malformed inout:
  argError = 'Malformed input arguments: use "help stretch_chainlink"';

  switch nargin
    case 1
      if isa(maxTL, 'double')
        verbose = false;
      else
        error(argError);
      end
    case 2
      if ~isa(maxTL, 'double') || ~islogical(verbose)
        error(argError);
      end
    otherwise
      error(argError);
  end

  NUM = numel(maxTL);   % Number of nodes

  if size(maxTL, 1) > 1
    % Workaround for MATLAB's column-major matrix policy:
    maxTL = reshape(maxTL.', 1, NUM);
  end

  if NUM > 0
    for i = 1 : NUM
      if maxTL(i) <= 0
        error(argError);
      end
    end
  else
    error(argError);
  end

  format compact;   % Eliminate unnecessay newlines

  %% Setting genetic algorithm options
  % min(maxTL) / sqrt(3) is a conservative setting, ensuring that
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

  %%
  % Inscribe initial population range cube in an octant of the sphere
  % formed by the minimum range at sea level among all nodes:
  minTL = min(maxTL);   % [143,155]
  numPaths = 2;
  range = 0;
  alpha = francois_garrison(25, 35, 0, 8, 10);

  while true
    range = range + 1;

    if minTL < numPaths * (20 * log10(range) + alpha * range)
      range = range - 1;
      break
    end
  end

  bounds = [ 3000; 9000 ];
  popInitRange = ones(2, 3 * NUM) * range / (2 * sqrt(3));
  popInitRange(1,:) = -popInitRange(1,:);
  popInitRange(:,2 * NUM + 1:3 * NUM) = popInitRange(:,2 * NUM + 1:3 * NUM) ...
                                        + mean(bounds);

  oldopts = gaoptimset(@ga);                % Load default options
  newopts = ...
    struct( ...
            'TolFun',       1e-4, ...                       % { 1e-6 }
            'PopInitRange', popInitRange, ...
            'Generations',  100 * NUM, ...
            'Vectorized',   'on' ...                        % { 'off' }
          );
  options = gaoptimset(oldopts, newopts);   % Overwrite default options

  if verbose == true
    options ...
      = gaoptimset(options, ...
                   'Display', 'iter', ...  % { 'final' }
                   'PlotFcns', { @gaplotbestf, ...
                                 @gaplotbestindiv, ...
                                 @gaplotstopping } ...
                  );
  end

  %% Invoking the genetic algorithm
  % Maximize _CHAINLINK_ by minimizing the negative of its score:
  objFunc = @(N) -chainlink(N, NUM, maxTL, bounds);   % Create function handle for GA

  poolchain = parpool([ 1, 200 ]);

  if verbose == true
    tic  % Start timer
  end

  N = ga(objFunc, 3 * NUM, options);

  if verbose == true
    toc  % Poll timer
  end

  %% Processing the genetic algrithm output
  % Reshape _N_(1, 3 * _NUM_) as _N_(_NUM_, 3),
  % such that _N_(_i_, :) = [ _Cx_ _Cy_ _Cz_ ]
  N = reshape(N, NUM, 3);

  %%
  % Calculate and map the separation between each pair of nodes:
  if verbose == true
    sep = NaN(NUM);

    for i = 1 : NUM - 1
      for j = i + 1 : NUM
        sep(i,j) = norm(N(i,:) - N(j,:));
      end
    end

    labels = cell(1, NUM);  % Row and column headers
    units = cell(1, NUM);   % Distance units

    for i = 1 : NUM
      labels{i} = strcat('Node', num2str(i));
      units{i} = 'm';
    end

    sep = array2table(sep(1 : NUM - 1, 2 : NUM).', ...
                      'RowNames', labels(2 : NUM), ...
                      'VariableNames', labels(1 : NUM - 1) ...
                     );

    sep.Properties.Description ...
      = 'Maps the Euclidean distances between each pair of nodes in 3D';
    sep.Properties.VariableUnits = units(1 : NUM - 1);

    display(sep);
  end

  %%
  % Calculate the node polyhedron volume and plot it in 3D:
  V = node_config_vol(N, maxTL, verbose);

  format;   % Restore default output options
  delete(poolchain);

%%
% Return a volume-optimized configuration for the given number of nodes:
end
