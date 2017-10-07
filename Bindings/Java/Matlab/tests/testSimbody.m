
import org.opensim.modeling.*

% Create a model.
model = Model('arm26.osim');

s = model.initSystem();

% Test SimbodyMatterSubsystem.calcResidualForce().
% ------------------------------------------------
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

