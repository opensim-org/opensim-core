
import org.opensim.modeling.*

% Create a model.
model = Model('arm26.osim');

s = model.initSystem();

model.realizeDynamics(s);
appliedMobilityForces = Vector();
appliedBodyForces = VectorOfSpatialVec();
knownUdot = Vector(s.getNU(), 0.0);
knownUdot.set(0, 1.0);
knownLambda = Vector();
residualMobilityForces = Vector();
smss = model.getMatterSubsystem();
% For the given inputs, we will actually be computing the first column
% of the mass matrix.
%   f_residual = M udot + f_inertial + f_applied 
%              = M ~[1, 0, ...] + 0 + 0
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

model.realizeAcceleration(s);
idsolver = InverseDynamicsSolver(model);
residual = idsolver.solve(s, s.getUDot());
assert(residual.size() == s.getNU());
for i = 0:residual.size()-1
    assert(abs(residual.get(i)) < 1e-10);
end

