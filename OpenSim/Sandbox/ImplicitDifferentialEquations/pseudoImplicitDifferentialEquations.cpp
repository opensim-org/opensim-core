/* Consider the following continuous optimal control problem:
 *
 * find       q, u, a, lm (fiber length), e (excitation), mult_t
 * minimize   reaction force in knee joint
 * subject to 
 *                        adot = f_act(a,e)      activation dynamics
 *       f_equil(q,lm,lmdot,a) = 0               fiber dynamics
 *                        qdot = N u             kinematics
 *            M udot + ~G mult = T + ~J*(F-C)    multi-body dynamics
 *               p(t,q)        = 0               holonomic constraints
 *               v(t,q,u)      = 0               non-holonomic constraints
 *               a(t,q,u,udot) = 0               acceleration-only constraints
 *
 *
 * The discretized problem (no derivatives):
 *
 * find       q_t, u_t, a_t, lm_t (fiber length), e_t (excitation), mult_t
 *            for t in [1, T]
 * minimize   sum (reaction_t)
 * subject to 
 *                                (a_t+1-a_t)/h = f_act(a_t,e_t)   
 *        f_equil(q_t,lm_t,(lm_t+1-lm_t)/h,a_t) = 0               
 *                                (q_t+1-q_t)/h = N u_t            
 *                  M (u_t+1-u_t)/h + ~G mult_t = T + ~J*(F-C)   
 *                   p(t,q_t)                   = 0              
 *                   v(t,q_t,u_t)               = 0              
 *                   a(t,q_t,u_t,(u_t+1-u_t)/h) = 0              
 *            (for t in [1, T-1])
 */

/* Here is a sketch of how to solve the above problem using a
 * SimTK::OptimizerSystem, if YDotGuess and LambdaGuess (MultiplierGuess)
 * live in the State.
 */
struct Trajectory {
    struct Step {
        State state;
        Vector excitations;
        int i; // Starting index in global vector of unknowns.
        int N; // Number of unknowns in this step.
    };
    std::vector<Step> steps;
};

class DirectCollocationProblem : public SimTK::OptimizerSystem {
public:
    Trajectory createTrajectory(const Vector& x) const {
        Trajectory traj;
        for (int istep = 0; istep < nsteps; ++istep) {
            Step step; step.state = State();
            step.state.updY() = x[indices];
            step.excitations = x[indices];
            step.state.updYDotGuess() = (x[indices+1] - x[indicies])/h;
            step.state.updMultipliersGuess() = x[indices];
            traj.append(step);
        }
        return traj;
    }
    int objectiveFunc(const Vector& x, Real& f) const {
        // Add up reaction on knee over all time.
        Trajectory traj = createTrajectory(x);
        for (const auto& step : traj) {
            model.realizeAcceleration(step.state);
            f += model.getJoint("knee").calcReactionOnParentExpressedInGround(step.state).norm();
        }
    }
    int constraintFunc(const Vector& x, Vector& constraints) const {
        Trajectory traj = createTrajectory(x);
        // Compute implicit diff eqn error and constraint error for all time.
        for (const auto& step : traj) {
            Vector residual, pvaerrs;
            model.realizeAcceleration(step.state);
            constraints(step.i,step.i+step.N) = [step.state.getResidual();
                                                 step.state.getUDotErr()];
        }
    }
};
/* One issue in the above scheme is that "realize" somewhat indicates
 * that the system is consistent; that the constraints are satisfied. You could
 * imagine that "realize" with an implicit form means that one performs a
 * root-solve on the implicit form to solve for a consistent YDot and Lambda.
 * But if the meaning of "realize" is "compute cache entries based on state
 * variables," then that's fine.
 */


/* The alternative OptimizerSystem, if YDotGuess and LambdaGuess do not live
 * in the State.
 */
struct Trajectory {
    struct Step {
        State state;
        Vector excitations;
        Vector yDotGuess; // Need to hold onto guesses separately.
        Vector lambdaGuess;
        int i; // Starting index in global vector of unknowns.
        int N; // Number of unknowns in this step.
    };
    std::vector<Step> steps;
};
class DirectCollocationProblem : public SimTK::OptimizerSystem {
public:
    Trajectory createTrajectory(const Vector& x) const {
        Trajectory traj;
        for (int istep = 0; istep < nsteps; ++istep) {
            Step step; 
            step.state = State();
            step.state.updY() = x[indices];
            step.excitations = x[indices];
            step.yDotGuess = (x[indices+1] - x[indices])/h;
            step.lambdaGuess = x[indices];
            traj.append(step);
        }
        return traj;
    }
    int objectiveFunc(const Vector& x, Real& f) const {
        // Add up reaction on knee over all time.
        Trajectory traj = createTrajectory(x);
        for (const auto& step : traj) {
            Vector_<SpatialVec> allReactionForces;
            // WRONG: model.realizeAcceleration(step.state);
            // WRONG: f += model.getJoint("knee").calcReactionOnParentExpressedInGround(step.state).norm();
            // Must use operator form.
            // TODO this method does not exist yet.
            model.getMatterSubsystem().calcMobilizerReactionForces(step.state,
                    step.yDotGuess, step.lambda,
                    allReactionForces);
            f += allReactionForces[kneeIndex].norm();
        }
    }
    int constraintFunc(const Vector& x, Vector& constraints) const {
        Trajectory traj = createTrajectory(x);
        // Compute implicit diff eqn error and constraint error for all time.
        for (const auto& step : traj) {
            Vector residual, pvaerrs;
            // Operator.
            model.calcImplicitResidualsAndConstraintErrors(step.state,
                    step.yDotGuess, step.lambdaGuess,
                    residuals, pvaerrs);
            constraints(step.i,step.i+step.N) = [residuals; pvaerrs];
        }
    }
};
