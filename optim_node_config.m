%% OPTIM_NODE_CONFIG
% Calculates the best volume-optimized node configuration
% for the given base node coverage radii.

%% Function signature
function bestN = optim_node_config(inCSV, iters, verbose)
%% Input
% _inCSV_: Comma-separated file with base node coverage radii
%%
% _iters_: Number of initial population iterations of the genetic algorithm
% to seed to select the best solution from
%%
% _verbose_: Boolean flag to specify output verbosity

%% Output
% _bestN_(_NUM_, 3): Best node configuration such that
% row vector _bestN_(_r_, :) = [ _Cx_ _Cy_ _Cz_ ],
% based on the maximal polyhedral volume _V_ enclosed by it

    %% Checking for malformed arguments
    % Input is preferred in a comma-delimited sequence of
    % double-precision values without line breaks.
    %%
    % In the case that multi-line CSV input is provided,
    % and is interpreted with ghost zeros,
    % remove all commas from the end of each line.

    argError = 'Malformed input arguments: Please refer to the source.';

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
            if iters <= 0 || ~islogical(verbose)
                error(argError);
            end
        otherwise
            error(argError);
    end

    %% Input parsing and reformatting
    % Read and reshape the input from the CSV if needed into
    % _R_(1, :) = [ _R1_ ... _R<NUM>_ ]

    R = csvread(inCSV);
    NUM = numel(R);     % Number of nodes

    if size(R, 1) > 1
        % Workaround for MATLAB's column-major matrix policy:
        R = reshape(R', 1, NUM);
    end

    %% Calling _STRETCH_CHAINLINK_ multiple times
    % _STRETCH_CHAINLINK_ is called as many times as specified in _iters_,
    % and the best among all optimized solutions is then selected.

    N = zeros(NUM, 3, iters);   % 3D matrix for incoming node configurations
    V = zeros(iters, 1);        % Volumes of each incoming node configuration

    for i = 1 : iters
        [ N(:,:,i), V(i) ] = stretch_chainlink(R, verbose);
    end

    %% Sorting solutions based on optimality
    % Sort node configurations based on the descending order of
    % the polyhedral volume onclosed by each.

    [ V, order ] = sort(V, 'descend');
    N = N(:,:,order);

    %%
    % Display sorted solutions if verbosity is required:
    if verbose == true
        V
        N
    end

    %%
    % Display the ranks of each iteration:
    if iters > 1
        order
    end

    %% Saving the output to files
    % Save the best node configuration to _inCSV_-optim_node_config.csv
    bestN = N(:,:,1);   % Save only the best node configuration
    outN = [ inCSV, '-optim_node_config.csv' ];     % Append to filename
    csvwrite(outN, bestN);

    %%
    % Save the maximal volume achieved to _inCSV_-optim-vol.txt
    bestV = V(1);       % Save only the maximal volume
    outV = [ inCSV, '-optim_vol.txt' ];             % Append to filename
    csvwrite(outV, bestV);

%% Returning the best configuration among the calculated optimized solutions
end
