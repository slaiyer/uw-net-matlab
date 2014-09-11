%% NODE_CONFIG_VOL
% Calculate the volume of the polyhedron formed by
% the given node configuration and optionally visualize it in 3D.
%
% Examples:
%
%   Use OPTIM_NODE_CONFIG or STRETCH_CHAINLINK as the entry point.
%
% See also: OPTIM_NODE_CONFIG, STRETCH_CHAINLINK, CHAINLINK, ATTENUATE
%
% Copyright 2014 Sidharth Iyer (246964@gmail.com)

%% Function signature
function V = node_config_vol(N, R, verbose)

%% Input
% _N_(_NUM_, 3): Optimized node configuration such that
% _N_(_i_, :) = [ _Cx_ _Cy_ _Cz_ ]
%%
% _R_(_NUM_): Vector of base node coverage radii
%%
% _verbose_: (Optional) Boolean flag to specify output verbosity

%% Output
% _V_: Polyhedral volume enclosed by _N_

  %%
  % Check for malformed input:

  argError = 'Malformed input arguments: use "help node_config_vol"';

  NUM = numel(R);   % Number of nodes

  switch nargin
    case 2
      if size(N, 1) == NUM && size(N, 2) == 3
        verbose = false;
      else
        error(argError);
      end
    case 3
      if size(N, 1) ~= NUM || size(N, 2) ~= 3 || ~islogical(verbose)
        error(argError);
      end
    otherwise
      error(argError);
  end

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

  %%
  % Use Delaunay triangulation to create and display the tetrahedral
  % mesh for the polyhedral volume enclosed by the node configuration:

  DT = delaunayTriangulation(N);
  [ ~, V ] = convexHull(DT);

  if verbose == true
    %% Displaying the 3D representation of the solution

    NUM = numel(R);   % Number of nodes

    figTitle = [ 'Volume-optimized ', num2str(NUM), '-node configuration' ];
    figure('Name', figTitle, 'NumberTitle', 'on');

    % Plot colours
    meshRed = [ 1 0.5 0.5 ];
    faceOrange = [ 1 0.95 0.8 ];
    green = [ 0.5 1 0.5 ];

    %%
    % 1. Display the polyhedral volume enclosed by _N_:

    subplot(1, 2, 1);
    scatter3(N(:,1), N(:,2), N(:,3), '.');
    hold on;  % Continue with current figure
    tetramesh(DT, 'EdgeColor', meshRed, 'FaceColor', faceOrange);
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
    tetramesh(DT, 'EdgeColor', meshRed, 'FaceColor', faceOrange);
    title('Node coverages');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    axis vis3d;

    hold off;
  end

end
