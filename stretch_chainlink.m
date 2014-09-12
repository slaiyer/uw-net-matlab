%% STRETCH_CHAINLINK
% Uses a vectorized genetic algorithm to optimize the node configuration
% for the given base node coverage radii.
%
% Examples:
%
%   STRETCH_CHAINLINK([ 12 23 34 45 ])
%   Optimizes the node configuration for the given node radii in the vector.
%
%   STRETCH_CHAINLINK([ 12 23 34 45 ], true)
%   Optimizes the node configuration in a verbose fashion.
%
% See also: OPTIM_NODE_CONFIG, CHAINLINK, ATTENUATE, NODE_CONFIG_VOL
%
% Copyright 2014 Sidharth Iyer (246964@gmail.com)

%% Function signature
function [ N, V ] = stretch_chainlink(TL, verbose)

%% Input
% _TL_: Vector of acceptable losses in intesity
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
      verbose = false;
    case 2
      if ~islogical(verbose)
        error(argError);
      end
    otherwise
      error(argError);
  end

  NUM = numel(TL);   % Number of nodes

  if size(TL, 1) > 1
    % Workaround for MATLAB's column-major matrix policy:
    TL = reshape(TL.', 1, NUM);
  end

  if NUM > 0
    for i = 1 : NUM
      if TL(i) <= 0
        error(argError);
      end
    end
  else
    error(argError);
  end

  format compact;   % Eliminate unnecessay newlines

  %% Setting genetic algorithm options
  % min(TL) / sqrt(3) is a conservative setting, ensuring that
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

  HALFRANGE = min(TL) / (2 * sqrt(3));       % Center roughly around origin

  oldopts = gaoptimset(@ga);                % Load default options
  newopts = ...
    struct( ...
            'TolFun',       1e-4, ...                       % { 1e-6 }
            'PopInitRange', [ -HALFRANGE; HALFRANGE ], ...  % { [ -10; 10 ] }
            'Vectorized',   'on' ...                        % { 'off' }
          );
  options = gaoptimset(oldopts, newopts);   % Overwrite default options

  if verbose == true
    options ...
       = gaoptimset(options, ...
                    'Display', 'iter', ...  % { 'final' }
                    'PlotFcns', { @gaplotbestf, @gaplotstopping } ...
                   );
  end

  %% Invoking the genetic algorithm
  % Maximize _CHAINLINK_ by minimizing the negative of its score:

  objFunc = @(N) -chainlink(N, TL, NUM);     % Create function handle for GA

  if verbose == true
    tic  % Start timer
  end

  [ N, ~, ~, ~, ~, ~ ] = ga(objFunc, 3 * NUM, options);

  if verbose == true
    toc  % Poll timer
  end

  %% Processing the genetic algrithm output
  % Reshape _N_(1, 3 * _NUM_) as _N_(_NUM_, 3),
  % such that _N_(_i_, :) = [ _Cx_ _Cy_ _Cz_ ]

  % Workaround for MATLAB's column-major matrix policy:
  N = reshape(N, 3, NUM).';

  %%
  % Calculate the node polyhedron volume and plot it in 3D:
  V = node_config_vol(N, TL, verbose);

  format;   % Restore default output options

%%
% Return a volume-optimized configuration for the given number of nodes:

end
