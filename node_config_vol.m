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
function V = node_config_vol(N, maxTL, verbose)

%% Input
% _N_(_NUM_, 3): Optimized node configuration such that
% _N_(_i_, :) = [ _Cx_ _Cy_ _Cz_ ]
%%
% _TL_(_NUM_): Vector of acceptable losses in intesity
% between trasmission and detection
%%
% _verbose_: (Optional) Boolean flag to specify output verbosity

%% Output
% _V_: Polyhedral volume enclosed by _N_

  %%
  % Check for malformed input:

  argError = 'Malformed input arguments: use "help node_config_vol"';

  NUM = numel(maxTL);   % Number of nodes

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

  %%
  % Use Delaunay triangulation to create and display the tetrahedral
  % mesh for the polyhedral volume enclosed by the node configuration:

  DT = delaunayTriangulation(N);
  [ ~, V ] = convexHull(DT);

  if verbose == true
    %% Displaying the 3D representation of the solution

    NUM = numel(maxTL);   % Number of nodes

    figTitle = [ 'Volume-optimized ', num2str(NUM), '-node configuration' ];
    figure('Name', figTitle, 'NumberTitle', 'on');

    % Plot colours
    meshRed = [ 1 0.5 0.5 ];
    faceOrange = [ 1 0.95 0.8 ];
    green = [ 0.5 1 0.5 ];

    %%
    % 1. Display the polyhedral volume enclosed by _N_:

    subplot(1, 2, 1);
    title('Node polyhedron');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    scatter3(N(:,1), N(:,2), N(:,3), '.');
    drawnow;
    hold on;  % Continue with current figure

    [ FBtri, FBpoints ] = freeBoundary(DT);
    [ ~, ~, IB ] = intersect(N, FBpoints, 'rows');
    Xbn = FBpoints(IB,:);
    iIB(IB) = 1 : length(IB);
    trisurf(iIB(FBtri), Xbn(:,1), Xbn(:,2), Xbn(:,3), ...
            'EdgeColor', meshRed, 'FaceColor', faceOrange);

    axis equal;
    axis vis3d;

    drawnow;
    hold on;

    %%
    % 2. a) Display translucent spheres depicting
    %  the coverage volume of each node:

    subplot(1, 2, 2);
    title('Node coverages');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    scatter3(N(:,1), N(:,2), N(:,3), '.');
    drawnow;
    hold on;  % Continue with current figure

    [ FBtri, FBpoints ] = freeBoundary(DT);
    [ ~, ~, IB ] = intersect(N, FBpoints, 'rows');
    Xbn = FBpoints(IB,:);
    iIB(IB) = 1 : length(IB);
    trisurf(iIB(FBtri), Xbn(:,1), Xbn(:,2), Xbn(:,3), ...
            'EdgeColor', meshRed, 'FaceColor', faceOrange);

    drawnow;
    hold on;

    numPaths = 2;
    edgeStep = 5;
    edgeV = [ 0, 0, 0 ];
    edgeUV = [ 0, 0, 0 ];
    z = [ 0 0 0 ];
    range = 0;
    alpha = 0;

    for i = 1 : NUM
      ptCloud = zeros(2 * NUM, 3);

      for j = 1 : NUM
        if j == i
          continue
        end

        edgeV = N(j,:) - N(i,:);
        edgeUV = edgeStep * edgeV / norm(edgeV);
        range = 0;
        alpha = 0;

        while true
          range = range + edgeStep;
          z = N(i,:) + range * edgeUV;
          alpha = francois_garrison(25, 35, z(3), 8, 10);

          if maxTL(i) < numPaths * (20 * log10(range) + alpha * range)
            ptCloud(j,:) = N(i,:) + (range - edgeStep) * edgeUV;
            break
          end
        end

        edgeUV = -edgeUV;
        range = 0;
        alpha = 0;

        while true
          range = range + edgeStep;
          z = N(i,:) + range * edgeUV;
          alpha = francois_garrison(25, 35, z(3), 8, 10);

          if maxTL(i) < numPaths * (20 * log10(range) + alpha * range)
            ptCloud(j + NUM,:) = N(i,:) + (range - edgeStep) * edgeUV;
            break
          end
        end
      end

      ptCloud(i + NUM,:) = [];
      ptCloud(i,:) = [];

      DT = delaunayTriangulation(ptCloud);
      trisurf(convexHull(DT), ptCloud(:,1), ptCloud(:,2), ptCloud(:,3), ...
              'EdgeColor', green, ...
              'EdgeAlpha', 1 / NUM, ...
              'FaceColor', green, ...
              'FaceAlpha', 1 / (2 * NUM) ...
             );

      drawnow;
      hold on;
    end

    axis equal;
    axis vis3d;

    hold off;
  end

end
