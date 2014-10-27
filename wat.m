res = 100;

xgv = -2e4 : res : 2e4;
ygv = -2e4 : res : 2e4;
zgv = 0 : res : 11000;
xSize = numel(xgv);
ySize = numel(ygv);
zSize = numel(zgv);

% absrpCoeff = zeros(xSize, ySize, zSize);
column = zeros(1, 1, zSize);

tic

for zz = zgv
  column(1, 1, 1 - zgv(1) + zz / res) = francois_garrison(25, 35, zz, 8, 10);
end

toc

absrpCoeff = repmat(column, xSize, ySize, 1);

toc

% save([ 'grid', num2str(res) ], 'res', 'xgv', 'ygv', 'zgv', 'absrpCoeff');
toc
FG = griddedInterpolant({ xgv, ygv, zgv }, absrpCoeff, 'spline', 'spline');
toc
