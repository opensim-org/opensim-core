
import org.opensim.modeling.*

% Test numeric typemaps.
% ----------------------
% Vec3.
matrix = [1, 9, 4];
osimVec3 = Vec3.createFromMat(matrix);
disp(osimVec3);
v2 = osimVec3.getAsMat();
assert(v2(1) == 1);
assert(v2(2) == 9);
assert(v2(3) == 4);

% Vector.
mat = [5, 3, 6, 2, 9];
v1 = Vector.createFromMat(mat);
v2 = v1.getAsMat();
for i = 1:length(mat)
    assert(v2(i) == mat(i));
end

% RowVector.
v1 = RowVector.createFromMat(mat);
v2 = v1.getAsMat();
for i = 1:length(mat)
    assert(v2(i) == mat(i));
end

% VectorVec3.
mat = [1, 2, 3; 4, 5, 6; 7, 8, 9; 10, 11, 12];
v1 = VectorVec3.createFromMat(mat);
v2 = v1.getAsMat();
for i = 1:4
    for j = 1:3
        assert(v2(i, j) == mat(i, j));
    end
end

% RowVectorVec3.
mat = [1, 2, 3; 4, 5, 6; 7, 8, 9; 10, 11, 12]';
v1 = RowVectorVec3.createFromMat(mat);
v2 = v1.getAsMat();
for i = 1:4
    for j = 1:3
        assert(v2(j, i) == mat(j, i));
    end
end


% Causes an error: v1 = RowVector([]);

% VectorView and RowVectorView.
% Use a TimeSeriesTable to obtain VectorViews.
table = TimeSeriesTable();
labels = StdVectorString();
labels.add('a');
table.setColumnLabels(labels);
table.appendRow(0.0, RowVector.createFromMat([1.5]));
table.appendRow(1.0, RowVector.createFromMat([2.5]));
column = table.getDependentColumn('a').getAsMat();
assert(length(column) == 2);
assert(column(1) == 1.5);
assert(column(2) == 2.5);
row = table.getRowAtIndex(0).getAsMat();
assert(length(row) == 1);
assert(row(1) == 1.5);
row = table.getRowAtIndex(1).getAsMat();
assert(length(row) == 1);
assert(row(1) == 2.5);

% Use a TimeSeriesTableVec3 to obtain VectorViewVec3s.
table = TimeSeriesTableVec3();
labels = StdVectorString();
labels.add('a');
table.setColumnLabels(labels);
table.appendRow(0.0, RowVectorVec3.createFromMat([1.5; 3.1; 6.4]));
table.appendRow(1.0, RowVectorVec3.createFromMat([2.5; 1.6; 7.3]));
column = table.getDependentColumn('a').getAsMat();
assert(size(column, 1) == 2);
assert(size(column, 2) == 3);
assert(column(1, 1) == 1.5);
assert(column(1, 2) == 3.1);
assert(column(1, 3) == 6.4);
assert(column(2, 1) == 2.5);
assert(column(2, 2) == 1.6);
assert(column(2, 3) == 7.3);
row = table.getRowAtIndex(0).getAsMat();
assert(length(row) == 3);
assert(row(1, 1) == 1.5);
assert(row(2, 1) == 3.1);
assert(row(3, 1) == 6.4);
row = table.getRowAtIndex(1).getAsMat();
assert(length(row) == 3);
assert(row(1, 1) == 2.5);
assert(row(2, 1) == 1.6);
assert(row(3, 1) == 7.3);

% Matrix.
mat = [5, 3; 3, 6; 8, 1];
m1 = Matrix.createFromMat(mat);
m2 = m1.getAsMat();
for i = 1:m1.nrow()
    for j = 1:m1.ncol()
        assert(m2(i, j) == mat(i, j));
    end
end


% Test SimbodyMatterSubsystem.calcResidualForce().
% ------------------------------------------------
% Create a model.
model = Model('arm26.osim');

s = model.initSystem();

% For the given inputs, we will actually be computing the first column
% of the mass matrix. We accomplish this by setting all inputs to 0
% except for the acceleration of the first coordinate.
%   f_residual = M udot + f_inertial + f_applied 
%              = M ~[1, 0, ...] + 0 + 0
model.realizeVelocity(s);
appliedMobilityForces = Vector();
appliedBodyForces = VectorOfSpatialVec();
knownUdot = Vector(s.getNU(), 0.0);
knownUdot.set(0, 1.0);
knownLambda = Vector();
residualMobilityForces = Vector();
smss = model.getMatterSubsystem();
smss.calcResidualForce(s, appliedMobilityForces, appliedBodyForces, ...
                  knownUdot, knownLambda, residualMobilityForces);
assert(residualMobilityForces.size() == s.getNU());

% Explicitly compute the first column of the mass matrix, then copmare.
massMatrixFirstColumn = Vector();
smss.multiplyByM(s, knownUdot, massMatrixFirstColumn);
assert(massMatrixFirstColumn.size() == residualMobilityForces.size());
for i = 0:massMatrixFirstColumn.size()-1
    assert(abs(massMatrixFirstColumn.get(i) - residualMobilityForces.get(i)) < 1e-10);
end

% Test InverseDynamicsSolver.
% ---------------------------
model.realizeAcceleration(s);
idsolver = InverseDynamicsSolver(model);
residual = idsolver.solve(s, s.getUDot());
assert(residual.size() == s.getNU());
for i = 0:residual.size()-1
    assert(abs(residual.get(i)) < 1e-10);
end

% Test Rotation.
% --------------
rot = Rotation()
rot.setRotationFromTwoAxes(UnitVec3(1, 0, 0), CoordinateAxis(0), ...
                           Vec3(0, 1, 0), CoordinateAxis(1));

