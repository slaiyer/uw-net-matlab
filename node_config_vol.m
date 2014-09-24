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

    scatter3(N(:,1), N(:,2), N(:,3), '.');
    drawnow;
    hold on;  % Continue with current figure

    [ FBtri, FBpoints ] = freeBoundary(DT);
    [ ~, ~, IB ] = intersect(N, FBpoints, 'rows');
    Xbn = FBpoints(IB,:);
    iIB(IB) = 1 : length(IB);
    trisurf(iIB(FBtri), Xbn(:,1), Xbn(:,2), Xbn(:,3), ...
            'EdgeColor', meshRed, 'FaceColor', faceOrange, 'FaceAlpha', 0.5);
    drawnow;

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

    scatter3(N(:,1), N(:,2), N(:,3), '.');
    drawnow;
    hold on;  % Continue with current figure

    trisurf(iIB(FBtri), Xbn(:,1), Xbn(:,2), Xbn(:,3), ...
            'EdgeColor', meshRed, 'FaceColor', faceOrange);
    drawnow;
    hold on;

    edgeStep = 100;

    for i = 1 : NUM
      ptCloud1 = zeros(2 * NUM, 3);
      ptCloud2 = zeros(2 * NUM, 3);

      for j = 1 : NUM
        if j == i
          continue
        end

        edgeV = N(j,:) - N(i,:);
        edgeStepV = edgeStep * edgeV / norm(edgeV);
        range = 0;
        point = N(i,:);

        % Calculate node communication ranges:
        % while true
        %   range = range + edgeStep;
        %   point = point + edgeStepV;

        %   if maxTL(i) < 20 * log10(range) ...
        %                   + francois_garrison(25, 35, point(3), 8, 10) ...
        %                     * range
        %     ptCloud1(j,:) = point - edgeStepV;
        %     break
        %   end
        % end

        % % Calculate attenuation in opposite direction:
        % edgeStepV = -edgeStepV;
        % range = 0;
        % point = N(i,:);

        % while true
        %   range = range + edgeStep;
        %   point = point + edgeStepV;

        %   if maxTL(i) < 20 * log10(range) ...
        %                   + francois_garrison(25, 35, point(3), 8, 10) ...
        %                     * range
        %     ptCloud1(j + NUM,:) = point - edgeStepV;
        %     break
        %   end
        % end

        % edgeStepV = -edgeStepV;
        % range = 0;
        % point = N(i,:);

        % Calculate echo-based detection ranges:
        while true
          range = range + edgeStep;
          point = point + edgeStepV;

          if maxTL(i) < 2 * (20 * log10(range) ...
                             + francois_garrison(25, 35, point(3), 8, 10) ...
                               * range)
            ptCloud2(j,:) = point - edgeStepV;
            break
          end
        end

        % Calculate attenuation in opposite direction:
        edgeStepV = -edgeStepV;
        range = 0;
        point = N(i,:);

        while true
          range = range + edgeStep;
          point = point + edgeStepV;

          if maxTL(i) < 2 * (20 * log10(range) ...
                             + francois_garrison(25, 35, point(3), 8, 10) ...
                               * range)
            ptCloud2(j + NUM,:) = point - edgeStepV;
            break
          end
        end
      end

      % Display node communication ranges:
      % Remove reflexive mappings:
      % ptCloud1(i + NUM,:) = [];
      % ptCloud1(i,:) = [];

      % trisurf(convexHull(delaunayTriangulation(ptCloud1)), ...
      %         ptCloud1(:,1), ptCloud1(:,2), ptCloud1(:,3), ...
      %         'EdgeAlpha', 0, ...
      %         'FaceAlpha', 0.01, ...
      %         'FaceColor', green ...
      %        );

      % Display echo-based detection ranges:
      % Remove reflexive mappings:
      ptCloud2(i + NUM,:) = [];
      ptCloud2(i,:) = [];

      trisurf(convexHull(delaunayTriangulation(ptCloud2)), ...
              ptCloud2(:,1), ptCloud2(:,2), ptCloud2(:,3), ...
              'EdgeAlpha', 0, ...
              'FaceAlpha', 0.1, ...
              'FaceColor', green ...  % meshRed when displaying both ranges
             );

      drawnow;
      hold on;
    end

    title('Node coverages');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    axis vis3d;

    hold off;
  end

end
