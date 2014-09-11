%% OPTIM_NODE_CONFIG
% Calculates the best volume-optimized node configuration
% for the given base node coverage radii.
%
% Examples:
%
%   OPTIM_NODE_CONFIG('test_radii.csv')
%   Optimizes the node configuration for the node radii in test_radii.csv
%
%   OPTIM_NODE_CONFIG('test_radii.csv', 3)
%   Finds the best of 3 solutions for the node radii in test_radii.csv
%
%   OPTIM_NODE_CONFIG('test_radii.csv', 1, true)
%   Finds 1 solution for the node radii in test_radii.csv in verbose fashion.
%
% See also: STRETCH_CHAINLINK, CHAINLINK, ATTENUATE, NODE_CONFIG_VOL
%
% Copyright 2014 Sidharth Iyer (246964@gmail.com)

%% Function signature
function bestN = optim_node_config(inCSV, iters, verbose)

%% Input
% _inCSV_: Comma-separated file with base node coverage radii
%%
% _iters_: (Optional) Number of initial population iterations of
% the genetic algorithm to seed to select the best solution from
%%
% _verbose_: (Optional) Boolean flag to specify output verbosity

%% Output
% _bestN_(_NUM_, 3): Best node configuration such that
% _bestN_(_r_, :) = [ _Cx_ _Cy_ _Cz_ ],
% based on the maximal polyhedral volume _V_ enclosed by it

  %% Checking for malformed input
  % Input is preferred in a comma-delimited sequence of
  % double-precision values without line breaks.
  %%
  % In the case that multi-line CSV input is provided,
  % and is interpreted with ghost zeros,
  % remove all commas from the end of each line.

  argError = 'Malformed input arguments: use "help optim_node_config"';

  switch nargin
    case 1
      iters = 1;
      verbose = false;
    case 2
      iters = uint64(iters);  % Cast to unsigned 64-bit integer
      if iters > 0
        verbose = false;
      else
        error(argError);
      end
    case 3
      iters = uint64(iters);  % Cast to unsigned 64-bit integer
      if iters == 0 || ~islogical(verbose)
        error(argError);
      end
    otherwise
      error(argError);
  end

  R = csvread(inCSV);
  NUM = numel(R);   % Number of nodes

  %%
  % Read and reshape the input from the CSV if needed into
  % _R_(1, :) = [ _R1_ ... _R<NUM>_ ]
  if size(R, 1) > 1
    % Workaround for MATLAB's column-major matrix policy:
    R = reshape(R.', 1, NUM);
  end

  if NUM > 0
    for i = 1 : NUM
      if R(i) <= 0
        error(argError);
      end
    end
  else
    error(argError);
  end

  if verbose == true
    clc;  % Clean slate if good to go
  end

  format compact;   % Eliminate unnecessay newlines in output

  %% Calling _STRETCH_CHAINLINK_ multiple times
  % _STRETCH_CHAINLINK_ is called as many times as specified in _iters_,
  % and the best among all optimized solutions is then selected.

  N = zeros(NUM, 3, iters);   % 3D matrix for incoming node configurations
  V = zeros(iters, 1);        % Volumes of each incoming node configuration

  for i = 1 : iters
    if verbose == false
      fprintf('Running GA iteration:\t%d/%d... ', i, iters);
    end

    [ N(:,:,i), V(i) ] = stretch_chainlink(R, verbose);
  end

  %% Sorting solutions based on optimality
  % Sort node configurations based on the descending order of
  % the polyhedral volume enclosed by each:

  [ V, rank ] = sort(V, 'descend');
  N = N(:,:,rank);

  %%
  % Display sorted solutions if verbosity is required:

  if verbose == true
    format long g;

    display(V);
    display(N);

    if iters > 1
      fprintf('Iteration-wise ');
      display(rank);  % Display iteration rankings
    end
  else
    node_config_vol(N(:,:,1), R, verbose);  % Visualize best solution only
  end

  %% Saving the output to files
  % Save the best node configuration to _inCSV_-optim_node_config.csv:

  bestN = N(:,:,1);   % Save only the best node configuration
  outN = [ inCSV, '-optim_node_config.csv' ];   % Append to filename
  csvwrite(outN, bestN);

  %%
  % Save the maximal volume achieved to _inCSV_-optim-vol.txt:

  bestV = V(1);       % Save only the maximal volume
  outV = [ inCSV, '-optim_vol.txt' ];           % Append to filename
  csvwrite(outV, bestV);

  format;   % Restore default output options

%%
% Return the best configuration among the calculated optimized solutions:

end
