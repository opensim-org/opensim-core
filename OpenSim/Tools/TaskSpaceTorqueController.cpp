/* -------------------------------------------------------------------------- *
 * OpenSim TaskSpace: TaskSpaceTorqueController.cpp                           *
 * -------------------------------------------------------------------------- *
  * Developed by CFD Research Corporation for a project sponsored by the US    *
 * Army Medical Research and Materiel Command under Contract No.              *
 * W81XWH-22-C-0020. Any views, opinions and/or findings expressed in this    *
 * material are those of the author(s) and should not be construed as an      *
 * official Department of the Army position, policy or decision unless so     *
 * designated by other documentation.                                         *
 *                                                                            *
 * Please refer to the following publication for mathematical details, and    *
 * cite this paper if you use this code in your own work:                 *
 *                                                                            *
 * Pickle and Sundararajan. "Predictive simulation of human movement in       *
 * OpenSim using floating-base task space control".                           *
 *                                                                            *
 * Copyright (c) 2023 CFD Research Corporation and the Authors                *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Nathan Pickle, Garrett Tuer               *
 *            and Dimitar Stanev                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <chrono>

#include "OpenSim/Actuators/CoordinateActuator.h"

#include "TaskSpaceComputeControlsEventHandler.h"
#include "TaskSpaceUtilities.h"
#include "TaskSpaceTorqueController.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

TaskSpaceTorqueController::TaskSpaceTorqueController()
{
    constructProperties();
    setNull();
}

TaskSpaceTorqueController::TaskSpaceTorqueController(const TaskSpaceTorqueController& aController)
    : Super(aController)
{
    copyData(aController);
}

void TaskSpaceTorqueController::constructProperties() 
{
    constructProperty_ConstraintModel(NoConstraintModel());
    constructProperty_excluded_coords();
    constructProperty_control_strategy("default");
    constructProperty_use_constraint_nullspace(true);
    
    TaskSpaceTaskSet tasks;
    tasks.setName(IO::Lowercase(tasks.getConcreteClassName()));
    constructProperty_TaskSpaceTaskSet(tasks);
}

void TaskSpaceTorqueController::setNull()
{
    _dt = 0.001;
    _lastDT = 0.0;
    _restoreDT = false;
    _tf = 0;
    _targetDT = 1.0e-2;
    _checkTargetTime = true;
}

void TaskSpaceTorqueController::copyData(const TaskSpaceTorqueController& aController)
{
    copyProperty_ConstraintModel(aController);
    copyProperty_excluded_coords(aController);
    copyProperty_TaskSpaceTaskSet(aController);
    copyProperty_control_strategy(aController);
    copyProperty_use_constraint_nullspace(aController);

    _dt                   = aController._dt;
   _tf                    = aController._tf;
   _lastDT                = aController._lastDT;
   _restoreDT             = aController._restoreDT;
   _checkTargetTime       = aController._checkTargetTime;
}

void TaskSpaceTorqueController::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    //validate priority levels
    std::set<int> priority_levels;
    for (int i = 0; i < get_TaskSpaceTaskSet().getSize(); ++i) {
        auto& task = get_TaskSpaceTaskSet().get(i);
        priority_levels.insert(task.get_priority());
    }
    int i=0;
    for (auto p : priority_levels) {
        if (p!=i) { 
            throw runtime_error(
                "Task priority levels must be consecutive starting at zero.");
        }
        i += 1;
        max_priority = i;
    }
    log_debug("priority_levels: {}", max_priority);
}

Matrix TaskSpaceTorqueController::createSelectionMatrix() const {
    Matrix B;
    if (hasModel()) {
        int ncoords = getModel().getNumCoordinates();
        B.resize(ncoords, ncoords);
        // All coordinates are actuated by default
        B = 1;
        auto state = _model->getWorkingState();
        auto& matter = _model->getMatterSubsystem();
        const Property<string>& excluded_coords = getProperty_excluded_coords();

        const auto& cs = _model->getCoordinateSet();

        for (int c = 0; c < excluded_coords.size(); ++c) {
            auto& coord = cs.get(get_excluded_coords(c));
            int mbti = matter.getMobilizedBody(coord.getBodyIndex())
                            .getFirstUIndex(state) +
                    coord.getMobilizerQIndex();
            B.updDiag()[mbti] = 0.0;
        }
    }
    return B;
}

void TaskSpaceTorqueController::printResults(const string &aPrefix, const string &aDir) {
    appliedForces.print(aDir + "/" + aPrefix + "_TaskSpaceTorqueController.sto");
}

// Controller Interface. 
// compute the control value for all actuators this Controller is responsible for
void TaskSpaceTorqueController::computeControls(const SimTK::State& s, SimTK::Vector& controls)  const
{
    // Make sure to use the order consistent with Simbody's representation
    const auto& cs = getModel().getCoordinatesInMultibodyTreeOrder();
    log_debug("integrator time: {}", s.getTime());
    auto tau = calcTau(s);
    for (int i = 0; i < cs.size(); ++i) {
        string name = cs[i]->getName() + "_control";
        
        getActuatorSet().get(name).addInControls(Vector(1, tau[i]), controls);
    }
}

SimTK::Vector TaskSpaceTorqueController::calcTau(const SimTK::State& s) const
{    
    //Start a clock
    using namespace std::chrono;
    auto start = high_resolution_clock::now();

    Matrix B = createSelectionMatrix();

    //Allocate variable for torque data for the n coordinate actuators
    auto tau = Vector(n, 0.0); //total task torque
    Matrix taua(n, 1, 0.0); //aggregate torque during iterative calculations

    //Evaluate the constraint data
    auto constraint_data = get_ConstraintModel().calcConstraintData(getModel(), s);
    //Constraint Jacobian
    Matrix Jc = constraint_data.Jc;
    //Transpose of the constraint null space matrix
    Matrix NcT = constraint_data.NcT;
    //Constraint bias
    Matrix bc = constraint_data.bc;
    //Mass matrix M
    Matrix M = constraint_data.M;
    //Constraint consistent inverse of the mass matrix
    Matrix McInv = constraint_data.McInv;
    //External (contact) forces
    Matrix Fext = constraint_data.Fext;
    //Total applied forces (gravity, coriolis, passive muscle/tendon, etc)
    Vector Fapplied = calcTotalGeneralizedForces(s, getModel());
    
    //--------------------------------------------------------------------------
    // Construct augmented Jacobian matrices and desired acceleration vectors,
    // sorted by priority level.
    //--------------------------------------------------------------------------
    Matrix Ja; // aggregate Jacobian for all tasks

    /* Priority-sorted augmented Jacobian matrix. */
    std::vector<SimTK::Matrix> Jt_p;

    // Priority-sorted desired accelerations
    std::vector<SimTK::Vector> xtddot_p; 

    // Priority-sorted task bias
    std::vector<SimTK::Vector> bt_p; 

    Jt_p.resize(max_priority+1);
    xtddot_p.resize(max_priority+1);
    bt_p.resize(max_priority+1);

    //Pre-assemble the stacked Jacobian, desired acceleration, and task bias
    //matrices for use in future calculations.
    for (int i = 0; i < get_TaskSpaceTaskSet().getSize(); ++i) {
        auto& task = get_TaskSpaceTaskSet().get(i);
        const int& priority = task.get_priority();
        task.update(s);
        xtddot_p[priority] = concatenate(
                            xtddot_p[priority], 
                            task.getDesiredAccelerations(), 
                            0)
                        .getAsVector();
        Jt_p[priority] = concatenate(Jt_p[priority], task.getJacobian(), 0);
        bt_p[priority] =
                concatenate(bt_p[priority], task.getBias(), 0).getAsVector();
    }

    //Aggregate task null space
    Matrix N_ai(NcT.nrow(), NcT.ncol(), 0.0);
    if (get_use_constraint_nullspace()) {
        //By default, constraints get priority -1. However, this may conflict
        //with movement controllers that utilize the SupportModel, as ground
        //contact is treated as a constraint.
        N_ai = NcT;
    }
    else {
        N_ai = 1.0;
    }

    //If no external forces, set contributions to zero
    if (Fext.nelt() == 0) { 
        Fext = Vector(Jc.nrow(), 0.0); 
    }

    //Iterate over priority levels and calculate task torques for each set of
    //tasks 
    for (int i = 0; i < max_priority; ++i) {
        //Append task Jacobians for current priority level to existing augmented
        //Jacobian matrix Ja
        Ja = concatenate(Ja, Jt_p[i], 0);

        //Task Jacobian at priority level i
        Matrix J_ai = Jt_p[i];
        //Dynamically consistent effective mass matrix
        Matrix Lambda_ai = DampedSVDPseudoinverse(J_ai * McInv * N_ai * ~J_ai);

        //Task space force contribution terms
        Matrix tf_xtddot = Lambda_ai * xtddot_p[i]; //task accelerations
        Matrix tf_gc = Lambda_ai * J_ai * McInv * NcT * Fapplied; //gravity and coriolis
        Matrix tf_bt = Lambda_ai * bt_p[i]; //task bias
        Matrix tf_bc = Lambda_ai * J_ai * McInv * bc; //constraint bias

        // compute total task space force
        auto F = tf_xtddot + tf_gc + tf_bt + tf_bc;

        //Project task forces into null space of higher priority tasks
        taua += N_ai * ~J_ai * F;

        //Update the aggregate 
        N_ai = N_ai * (1 - ~J_ai * Lambda_ai * J_ai * McInv * N_ai);
    }

    //Filter aggregate task torques through constraint null space to make
    //dynamically consistent, and account for floating base (underactuation)
    if (Ja.nelt() > 0) {
        Matrix Lambda_a_bar = DampedSVDPseudoinverse(Ja * McInv * NcT * ~Ja);
        Matrix Ja_bar = Lambda_a_bar * Ja * McInv * NcT;
        
        //Constraint consistent aggregate null space projection
        Matrix Na_bar = 1.0 - (~Ja * Ja_bar);
        
        //Account for "virtual dofs" in a floating base model.
        Matrix Beta;
        if ((1.0-B).norm() <= 1e-8) {
            //B == identity
            Beta = B;
        } else {
            Beta = 1.0 - (Na_bar * DampedSVDPseudoinverse((1.0 - B) * Na_bar) * (1.0 - B));
        }

        //Apply the filter to the computed task torques. The selection matrix B is included 
        //to ensure the constraint tau=B*tau is not violated. In general, Beta*taua will result
        //in zero forces/torques at excluded coordinates. However, in certain infeasible scenarios
        //the constraint will be violated and nonzero forces/torques may be computed for excluded
        //coordinates. Explicitly enforcing tau=B*tau means that the simulation will fail rather
        //than succeeding with fictitious forces/torques applied.
        tau = (B * Beta * taua).getAsVector();
    } else {
        tau = Vector(n, 0.0);
    }

    //FOR REVIEW: Dimitar Stanev's original code utilized these additional
    //            calculations for the control torques. There may be a better
    //            way to incorporate them. Also need to check whether N_ai or
    //            Na_bar should be used.
    if (get_control_strategy() == "default") {
        //Just use tau
    } else if (get_control_strategy() == "force") {
        tau = tau + N_ai*(Fapplied + constraint_data.bc);
    } else if (get_control_strategy() == "velocity") {
        tau = tau - 20*N_ai*s.getU();
    } else {
        log_error("Control strategy not recognized.");
    }

    //Print timing info
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    log_debug("calcTaskDynamics: {}s (last time step {}ms)", s.getTime(), duration.count());

    //Print calculated task torques 
    log_debug("tau= {}", tau);

    return tau;
}

void TaskSpaceTorqueController::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    // construct labels for the applied forces
    Array<string> storageLabels;
    storageLabels.append("time");
    
    // create an actuator for each generalized coordinate in the model
    // add these actuators to the model and set their indexes
    auto& cs = model.getCoordinateSet();
    for (int i = 0; i < cs.getSize(); ++i) {
        string name = cs.get(i).getName() + "_control";
        CoordinateActuator* actuator = NULL;
        if (model.getForceSet().contains(name)) {
            actuator = (CoordinateActuator*)&model.getForceSet().get(name);
        } else {
            actuator = new CoordinateActuator();
            actuator->setCoordinate(&cs.get(i));
            actuator->setName(name);
            // since this object is creating these actuators for its own
            // devices, it should take ownership of them, so that when the
            // controller is removed, so are all the actuators it added
            adoptSubcomponent(actuator);
        }

        actuator->setOptimalForce(1.0);
        updActuators().adoptAndAppend(actuator);
        log_debug("added actuator: {}", actuator->getName());
        // append labels
        storageLabels.append(cs[i].getName());
    }
    // configure Storage
    appliedForces.setColumnLabels(storageLabels);
    appliedForces.reset(0);

    _tau.resize(cs.getSize());
    _tau.setTo(0.0);

    n = getModel().getNumCoordinates();
}

void TaskSpaceTorqueController::extendAddToSystem( SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    // add event handler for updating controls for next window 
    TaskSpaceTorqueController* mutableThis = const_cast<TaskSpaceTorqueController *>(this);
    ComputeControlsEventHandler* computeControlsHandler = 
        new ComputeControlsEventHandler(mutableThis);

    computeControlsHandler->setEventInterval(0.01);

    system.updDefaultSubsystem().addEventHandler(computeControlsHandler );
}

/**
 * Set the requested integrator time step size.
 *
 * @param aDT Step size (0.0 <= aDT).
 */
void TaskSpaceTorqueController::
setDT(double aDT)
{
    _dt = aDT;
    if(_dt<0) _dt=0.0;
}
//_____________________________________________________________________________
/**
 * Get the requested integrator time step size.
 *
 * @return Step size.
 */
double TaskSpaceTorqueController::
getDT() const
{
    return(_dt);
}

//-----------------------------------------------------------------------------
// TARGET TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the target time (or final time) for the controller.
 *
 * The function of the controller is to compute a set of controls that
 * are appropriate from the current time in a simulation to the
 * target time of the controller.  If an integrator is taking time steps
 * prior to the target time, the controls should not have to be computed again.
 *
 * @param aTargetTime Time in the future for which the controls have been
 * computed.
 * @see getCheckTargetTime()
 */
void TaskSpaceTorqueController::
setTargetTime(double aTargetTime)
{
    _tf = aTargetTime;
}
//_____________________________________________________________________________
/**
 * Get the target time.
 *
 * The target time is the time in the future for which the controls have been
 * calculated.  If an integrator is taking time steps prior to the target
 * time, the controls should not have to be computed again.
 *
 * @return Time in the future for which the controls have been
 * computed.
 * @see getCheckTargetTime()
 */
double TaskSpaceTorqueController::
getTargetTime() const
{
    return(_tf);
}

//-----------------------------------------------------------------------------
// TARGET DT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the target time step size.
 *
 * The target time step size is the step size used to compute a new target
 * time, once the former target time has been reached by the integrator.
 *
 * @param aDT Target time step size.  Must be greater than 1.0e-8.
 * @see setTargetTime()
 */
void TaskSpaceTorqueController::
setTargetDT(double aDT)
{
    _targetDT = aDT;
    if(_targetDT<1.0e-8) _targetDT =  1.0e-8;
}
//_____________________________________________________________________________
/**
 * Get the target time step size.
 *
 * The target time step size is the step size used to compute a new target
 * time, once the former target time has been reached by the integrator.
 *
 * @return Target time step size.
 * @see setTargetTime()
 */
double TaskSpaceTorqueController::
getTargetDT() const
{
    return(_targetDT);
}

//-----------------------------------------------------------------------------
// CHECK TARGET TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not to check the target time.
 *
 * @param aTrueFalse If true, the target time will be checked.  If false, the
 * target time will not be checked.
 * @see setTargetTime()
 */
void TaskSpaceTorqueController::
setCheckTargetTime(bool aTrueFalse)
{
    _checkTargetTime = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not to check the target time.
 *
 * @return True if the target time will be checked.  False if the target
 * time will not be checked.
 * @see setTargetTime()
 */
bool TaskSpaceTorqueController::
getCheckTargetTime() const
{
    return(_checkTargetTime);
}
