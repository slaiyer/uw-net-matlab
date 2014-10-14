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
    for n1 = 1 : NUM
      if maxTL(n1) <= 0
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
    red = [ 1, 0.5, 0.5 ];
    orange = [ 1, 0.95, 0.8 ];
    green = [ 0.5, 1, 0.5 ];

    %%
    % 1. Display the polyhedral volume enclosed by _N_:
    subplot(1, 2, 1);

    scatter3(N(:, 1), N(:, 2), N(:, 3), '.');
    drawnow;
    hold on;  % Continue with current figure

    [ FBtri, FBpoints ] = freeBoundary(DT);
    [ ~, ~, ib ] = intersect(N, FBpoints, 'rows');
    XYZ = FBpoints(ib, :);
    Tri(ib) = 1 : length(ib);
    trisurf(Tri(FBtri), XYZ(:, 1), XYZ(:, 2), XYZ(:, 3), ...
            'EdgeColor', red, 'FaceColor', orange, 'FaceAlpha', 0.75);
    drawnow;

    set(gca, 'ZDir', 'reverse');
    title('Node polyhedron');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    axis vis3d;

    hold on;

    %%
    % 2. a) Display translucent spheres depicting
    %  the coverage volume of each node:
    subplot(1, 2, 2);

    scatter3(N(:, 1), N(:, 2), N(:, 3), '.');
    drawnow;
    hold on;  % Continue with current figure

    trisurf(Tri(FBtri), XYZ(:, 1), XYZ(:, 2), XYZ(:, 3), ...
            'EdgeColor', red, 'FaceColor', orange);
    drawnow;
    hold on;

    edgeStep = 10;
    rubikV = [
               [ -1, -1, -1 ];
               [ -1, -1,  0 ];
               [ -1, -1,  1 ];
               [ -1,  0, -1 ];
               [ -1,  0,  0 ];
               [ -1,  0,  1 ];
               [ -1,  1, -1 ];
               [ -1,  1,  0 ];
               [ -1,  1,  1 ];
               [  0, -1, -1 ];
               [  0, -1,  0 ];
               [  0, -1,  1 ];
               [  0,  0, -1 ];
               [  0,  0,  1 ];
               [  0,  1, -1 ];
               [  0,  1,  0 ];
               [  0,  1,  1 ];
               [  1, -1, -1 ];
               [  1, -1,  0 ];
               [  1, -1,  1 ];
               [  1,  0, -1 ];
               [  1,  0,  0 ];
               [  1,  0,  1 ];
               [  1,  1, -1 ];
               [  1,  1,  0 ];
               [  1,  1,  1 ];
             ];
    cardinals = size(rubikV, 1);
    
    for dir = 1 : cardinals
      rubikV(dir, :) = edgeStep * rubikV(dir, :) / norm(rubikV(dir, :));
    end

    for n1 = 1 : NUM
      % ptCloud1 = zeros(2 * NUM, 3);
      ptCloud2 = zeros(2 * NUM + cardinals, 3);
      range = 0;
      point = N(n1, :);
      absorption = 0;

      for n2 = 1 : NUM
        if n2 == n1
          continue
        end

        edgeV = N(n2, :) - N(n1, :);
        edgeStepV = edgeStep * edgeV / norm(edgeV);

        % Calculate node communication ranges:
        % while true
        %   range = range + edgeStep;
        %   point = point + edgeStepV;
        %   absorption = absorption ...
        %                + francois_garrison(25, 35, point(3), 8, 10) * edgeStep;

        %   if maxTL(n1) < 20 * log10(range) + absorption
        %     ptCloud1(n2, :) = point - edgeStepV;
        %     break
        %   end
        % end

        % % Calculate attenuation in opposite direction:
        % edgeStepV = -edgeStepV;
        % range = 0;
        % point = N(n1, :);
        % absorption = 0;

        % while true
        %   range = range + edgeStep;
        %   point = point + edgeStepV;
        %   absorption = absorption ...
        %                + francois_garrison(25, 35, point(3), 8, 10) * edgeStep;

        %   if maxTL(n1) < 20 * log10(range) + absorption
        %     ptCloud1(n2 + NUM, :) = point - edgeStepV;
        %     break
        %   end
        % end

        % edgeStepV = -edgeStepV;
        % range = 0;
        % point = N(n1, :);
        % absorption = 0;

        % Calculate echo-based detection ranges:
        while true
          range = range + edgeStep;
          point = point + edgeStepV;
          absorption = absorption ...
                       + francois_garrison(25, 35, point(3), 8, 10) * edgeStep;

          if maxTL(n1) < 2 * (20 * log10(range) + absorption)
            ptCloud2(n2, :) = point - edgeStepV;
            break
          end
        end

        % Calculate attenuation in opposite direction:
        edgeStepV = -edgeStepV;
        range = 0;
        point = N(n1, :);
        absorption = 0;

        while true
          range = range + edgeStep;
          point = point + edgeStepV;
          absorption = absorption ...
                       + francois_garrison(25, 35, point(3), 8, 10) * edgeStep;

          if maxTL(n1) < 2 * (20 * log10(range) + absorption)
            ptCloud2(n2 + NUM, :) = point - edgeStepV;
            break
          end
        end
      end
      
      for dir = 1 : cardinals
        range = 0;
        point = N(n1, :);
        absorption = 0;
        edgeStepV = rubikV(dir, :);
        
        while true
          range = range + edgeStep;
          point = point + edgeStepV;
          absorption = absorption ...
                       + francois_garrison(25, 35, point(3), 8, 10) * edgeStep;

          if maxTL(n1) < 2 * (20 * log10(range) + absorption)
            ptCloud2(2 * NUM + dir, :) = point - edgeStepV;
            break
          end
        end
      end

      % Display node communication ranges:
      % Remove reflexive mappings:
      % ptCloud1(n1 + NUM, :) = [];
      % ptCloud1(n1, :) = [];

      % [ FBtri, FBpoints ] = freeBoundary(delaunayTriangulation(ptCloud1));
      % [ ~, ~, IB ] = intersect(ptCloud1, FBpoints, 'rows');
      % Xbn = FBpoints(IB, :);
      % Tri(IB) = 1 : length(IB);
      % trisurf(Tri(FBtri), Xbn(:, 1), Xbn(:, 2), Xbn(:, 3), ...
      %         'EdgeAlpha', 0, 'FaceAlpha', 0.01, 'FaceColor', red);

      % Display echo-based detection ranges:
      % Remove reflexive mappings:
      ptCloud2(n1 + NUM, :) = [];
      ptCloud2(n1, :) = [];

      [ FBtri, FBpoints ] = freeBoundary(delaunayTriangulation(ptCloud2));
      [ ~, ~, ib ] = intersect(ptCloud2, FBpoints, 'rows');
      XYZ = FBpoints(ib, :);
      Tri(ib) = 1 : length(ib);
      trisurf(Tri(FBtri), XYZ(:, 1), XYZ(:, 2), XYZ(:, 3), ...
              'EdgeAlpha', 0, 'FaceAlpha', 0.1, 'FaceColor', green);

      drawnow;
      hold on;
    end

    set(gca, 'ZDir', 'reverse');
    title('Node coverages');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    axis vis3d;

    hold off;
  end

%%
% Return the volume of the convex hull of the point cloud:
end
