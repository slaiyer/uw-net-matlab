%% STRETCH_CHAINLINK
% Uses a vectorized genetic algorithm to optimize _CHAINLINK_.

%% Signature
function N = stretch_chainlink()
% TODO: Implement anisotropic attenuation
% TODO: Input: Vector of base coverage radii of nodes R

    %% Initializing global variables
    % Hardcoded values for testing and debugging:

    % R = [94 62 93 16]             % Random radii for 4 nodes
    R = [13 25 18 42 25 32 24 24]   % Random radii for 8 nodes

    NUM = size(R, 2)                % Number of nodes

    %% Genetic algorithm options
    % _TolFun_ sets stopping criterion based on average change of
    % fitness function value over _StallGen_ generations to the specified value.
    %%
    % _Vectorized_ specifies if the the GA is to be run with
    % multiple individuals passed to it for evaluation at once or not.

    % TODO: Set options optimally
    oldopts = gaoptimset(@ga);      % Load default options explicitly
    newopts = ...
        struct( ...
            'TolFun',       1e-6, ...               % Default: 1e-6
            'PlotFcns',     { @gaplotbestf }, ...   % Default: []
            'Display',      'iter', ...             % 'iter', default: 'final'
            'Vectorized',   'on' ...                % Default: 'off'
              );
    options = gaoptimset(oldopts, newopts);     % Overwrite selected parameters

    %% Invoking the genetic algorithm
    % Maximize _CHAINLINK_ by minimizing its negative score:
    tic    % Start timer
    objFunc = @(N) -chainlink(N, R);    % Create function handle for GA
    %%
    % Dump GA output: [ _x_, _fval_, _exitFlag_, _output_, _population_, _scores_ ]
    [N, volume, ~, output, ~, ~] = ga(objFunc, 3 * NUM, options)
    toc    % Poll timer

    %%
    % Calculate map separation between each pair of nodes.
    % Reshape _N_(1, 3 * _NUM_) as _N_(_NUM_, 3),
    % such that row vector _N_(_r_, :) = [ _Cx_ _Cy_ _Cz_ ]

    % Workaround for MATLAB's column-major matrix policy
	N = reshape(N, 3, NUM)';

    sep = zeros(NUM, NUM);

    for i = 1 : NUM - 1
        for j = i + 1 : NUM
            sep(j,i) = norm(N(i,:) - N(j,:));
        end
    end

    sep

%% Return volume optimized configuration for the given number of nodes
end
