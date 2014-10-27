function test()

  maxTL = 143:0.5:155;
  N = csvread('sample.csv');
  NUM = numel(maxTL);
  center = sum(N, 1) / NUM;
  kernel = zeros(NUM, 3);
  tmp = zeros(NUM, 1);

  DT = delaunayTriangulation(N);
  [ ~, polyVol ] = convexHull(DT);
  display(polyVol);

  figTitle = [ 'Volume-optimized ', num2str(NUM), '-node configuration' ];
  figure('Name', figTitle, 'NumberTitle', 'on');

  red = [ 1, 0.5, 0.5 ];
  orange = [ 1, 0.95, 0.8 ];
  green = [ 0.5, 1, 0.5 ];

  subplot(1, 2, 1);

  scatter3(N(:, 1), N(:, 2), N(:, 3), '.');
  drawnow;
  hold on;

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

  subplot(1, 2, 2);

  scatter3(N(:, 1), N(:, 2), N(:, 3), '.');
  drawnow;
  hold on;

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

  craters = zeros(NUM, 3, NUM);
  lumps = zeros(NUM - 1, 3, NUM);
  nodeVol = zeros(NUM, 2);

  for n1 = 1 : NUM
    ptCloud2 = zeros(2 * NUM + cardinals, 3);
    range = 0;
    point = N(n1, :);
    craters(n1, :, n1) = point;
    absorption = 0;

    for n2 = 1 : NUM
      if n2 == n1
        continue
      end

      edgeV = N(n2, :) - N(n1, :);
      edgeStepV = edgeStep * edgeV / norm(edgeV);

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

      craters(n2, :, n1) = ptCloud2(n2, :);

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
    
    ptCloud2(n1 + NUM, :) = [];
    ptCloud2(n1, :) = [];

    lumps(:, :, n1) = setdiff(ptCloud2(1 : 2 * (NUM - 1), :), craters(:, :, n1), 'rows');

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
          ptCloud2(2 * (NUM - 1) + dir, :) = point - edgeStepV;
          break
        end
      end
    end

    [ ~, nodeVol(n1, 2) ] = convhull(craters(:, :, n1));

    tmp2 = ptCloud2(:, 3);
    tmp2(tmp2 < 0) = 0;
    ptCloud2(:, 3) = tmp2;

    DT = delaunayTriangulation(ptCloud2);
    [ ~, nodeVol(n1, 1) ] = convexHull(DT);
    [ FBtri, FBpoints ] = freeBoundary(DT);
    [ ~, ~, ib ] = intersect(ptCloud2, FBpoints, 'rows');
    XYZ = FBpoints(ib, :);
    Tri(ib) = 1 : length(ib);
    trisurf(Tri(FBtri), XYZ(:, 1), XYZ(:, 2), XYZ(:, 3), ...
            'EdgeAlpha', 0, 'FaceAlpha', 0.1, 'FaceColor', green);

    drawnow;
    hold on;
  end

  display(nodeVol);

  set(gca, 'ZDir', 'reverse');
  title('Node coverages');
  xlabel('X');
  ylabel('Y');
  zlabel('Z');
  axis equal;
  axis vis3d;

  hold off;

  for i = 1 : NUM
    [ ~, tmp(i) ] = convhull(lumps(:, :, i));
  end
  
  sort(tmp)
  sort(nodeVol(:, 1) - nodeVol(:, 2))
  
  V = polyVol + sum(tmp)
  polyVol + sum(nodeVol(:, 1) - nodeVol(:, 2))

  % Only viable metrics till now
  display((polyVol + sum(nodeVol(:, 1) - nodeVol(:, 2))) / sum(nodeVol(:, 1)));
  display((polyVol + sum(nodeVol(:, 1) - nodeVol(:, 2))) / NUM);
  
end
