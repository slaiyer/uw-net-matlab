%% VIS_NODE_CONFIG
% Visualize the given node configuration in 3D.
%
% Copyright 2014 Sidharth Iyer (246964@gmail.com)
%
% Examples:
%
%   Use OPTIM_NODE_CONFIG or STRETCH_CHAINLINK as the entry point.
%
% See also OPTIM_NODE_CONFIG, STRETCH_CHAINLINK, CHAINLINK, ATTENUATE

%% Function signature
function V = vis_node_config(N, R, verbose)

%% Input
% _N_(_NUM_, 3): Optimized node configuration such that
% row vector _N_(_i_, :) = [ _Cx_ _Cy_ _Cz_ ]
%%
% _R_(_NUM_): Vector of base node coverage radii
%%
% _verbose_: (Optional) Boolean flag to specify output verbosity

%% Output
% _V_: Polyhedral volume enclosed by _N_

  %%
  % Check for malformed arguments:

  argError = 'Malformed input arguments. Use "help vis_node_config".';

  switch nargin
    case 2
      verbose = true;
    case 3
      if ~islogical(verbose)
        error(argError);
      end
    otherwise
      error(argError);
  end

  %%
  % Use Delaunay triangulation to create and display the tetrahedral mesh
  % for the polyhedral volume enclosed by the node configuration:

  DT = delaunayTriangulation(N);
  [ ~, V ] = convexHull(DT);

  if verbose == true
    %% Displaying the 3D representation of the solution

    NUM = numel(R);   % Number of nodes

    figTitle = [ 'Volume-optimized ', num2str(NUM), '-node configuration' ];
    figure('Name', figTitle, 'NumberTitle', 'on');

    % Plot colours
    meshRed = [ 0.666 0 0 ];
    faceOrange = [ 1 0.9 0.7 ];
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
  end

end
