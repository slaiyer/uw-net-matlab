function x = stretch_chainlink()
% STRETCH_CHAINLINK uses multi-objective GA to optimize CHAINLINK objectives

% OBSOLETE: Input: N[4][m] such that column subvector N(1:4,col) = [Cx Cy Cz R]

    N = [49 30 -47 94; 12 28 -34 62; 29 4 -20 93; 38 84 -30 16];

    global M R;
    M = size(N, 1);
    R = N(4, 1:M)

    % FIXME: Set options properly
    options = gaoptimset('PopulationSize', 100); % , ...
              % 'ParetoFraction', 0.7, 'PlotFcns', @gaplotpareto);

    % FIXME: Set constraints, ranges, initial population, etc.
    [x, fval, flag, output, population] = gamultiobj(@chainlink, 3*M, ...
                                          [], [], [], [], [-5,-5], [5,5], options);

    fval
    output

    % Present as x[3][m] such that column subvector x(1:3,col) = [Cx Cy Cz]
	x = reshape(x, 3, 4);

    dist = zeros(M,M);

    for i = 1:M-1
        for j = i:M
            dist(i,j) = norm(x(1:3,i) - x(1:3,j));
        end
    end

    % Map distances between nodes i and j
    dist

end
