function [spread, H_xi]  = chainlink(N)
% CHAINLINK calculates separation and overlap factor for given node configuration N in Euclidean space
% Input: N[3*m] = [Cx1 Cy1 Cz1 ... Cxm Cym Czm]

% TODO: Investigate/implement z_ij from Page 14 of Uhler_sphere_packing.pdf

    global M R;

    % m = numel(N) / 3; % Number of nodes
    sep = zeros(1, nchoosek(M, 2)); % Number of lattice edges
    xi = zeros(1, nchoosek(M, 2)); % Number of overlaps
    idx = 1; % Index of lattice interactions

    for i = 0:M-2
        for j = i+1:M-1
            % Calculate and append distance between centres of nodes i and j
            sep(idx) = norm(N(i*3+(1:3)) - N(j*3+(1:3)));
            % Calculate and append overlap between nodes i and j
            xi(idx) = R(i+1) + R(j+1) - sep(idx);
            idx = idx + 1;
        end
    end

    % Return negative of mean separation
    spread = -mean(sep);
    % Return 2-norm of overlaps
    H_xi = norm(xi);

end
