
import org.opensim.modeling.*

% Create a model.
model = Model('arm26.osim');

s = model.initSystem();

model.realizeDynamics(s);
appliedMobilityForces = Vector();
appliedBodyForces = VectorOfSpatialVec();
knownUdot = Vector();
knownLambda = Vector();
residualMobilityForces = Vector();
smss = model.getMatterSubsystem();
smss.calcResidualForce(s, appliedMobilityForces, appliedBodyForces, ...
                  knownUdot, knownLambda, residualMobilityForces);

idsolver = InverseDynamicsSolver(model);
residual = idsolver.solve(s, knownUdot);

