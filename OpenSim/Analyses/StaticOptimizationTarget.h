#ifndef _StaticOptimizationTarget_h_
#define _StaticOptimizationTarget_h_
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  StaticOptimizationTarget.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Jeffrey A. Reinbolt                                             *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include "OpenSim/Common/Array.h"
#include "osimAnalysesDLL.h"
#include <simmath/Optimizer.h>

#include <OpenSim/Common/GCVSplineSet.h>

//=============================================================================
//=============================================================================
namespace OpenSim {

class Model;

/**
 * This class provides an interface specification for static optimization
 * Objective Function.
 * @author Jeff Reinbolt
 */
class OSIMANALYSES_API StaticOptimizationTarget
        : public SimTK::OptimizerSystem {
    //=============================================================================
    // DATA
    //=============================================================================
public:
    /** Smallest allowable perturbation size for computing derivatives. */
    static const double SMALLDX;

private:
    /** Work model. */
    Model* _model;
    /** Current state */
    const SimTK::State* _currentState;
    /** Reciprocal of actuator area squared. */
    Array<double> _recipAreaSquared;
    /** Reciprocal of optimal force squared accounting for force-length curve if
     * actuator is a muscle. */
    Array<double> _recipOptForceSquared;
    /** Optimal force accounting for force-length curve if desired and if
     * actuator is a muscle. */
    Array<double> _optimalForce;

    SimTK::Matrix _constraintMatrix;
    SimTK::Vector _constraintVector;

    const Storage* _statesStore;
    GCVSplineSet _statesSplineSet;
    const Storage* _statesDerivativeStore;

protected:
    double _activationExponent;
    bool _useMusclePhysiology;
    /** Perturbation size for computing numerical derivatives. */
    Array<double> _dx;
    Array<int> _accelerationIndices;

    //=============================================================================
    // METHODS
    //=============================================================================
public:
    /**
     * Construct an optimization target.
     *
     * @param s The current model state.
     * @param aModel The model to use in optimization.
     * @param aNP The number of parameters.
     * @param aNC The number of constraints.
     * @param useMusclePhysiology If false, ignores muscle force-length and
     * force-velocity relationships as well as pennation angle
     */
    StaticOptimizationTarget(const SimTK::State& s, Model* aModel, int aNP,
            int aNC, const bool useMusclePhysiology = true);

    // SET AND GET

    /**
     * Set the model.
     *
     * @param aModel Model.
     */
    void setModel(Model& aModel);

    /**
     * Set the states storage.
     *
     * @param aStatesStore States storage.
     */
    void setStatesStore(const Storage* aStatesStore);

    /**
     * Set the states derivative storage.
     *
     * @param aStatesDerivativeStore States derivative storage.
     */
    void setStatesDerivativeStore(const Storage* aStatesDerivativeStore);

    /**
     * Set the states spline set.
     *
     * @param aStatesSplineSet States spline set.
     */
    void setStatesSplineSet(GCVSplineSet aStatesSplineSet);

    /**
     * Set the number of parameters.
     *
     * The number of parameters can be set at any time.  However, the
     * perturbation sizes for the parameters (i.e., _dx) is destroyed.
     * Therefore, the perturbation sizes must be reset.
     *
     * @param aNP Number of parameters.
     * @see setDX()
     */
    void setNumParams(const int aNP);

    /**
     * Set the number of constraints.
     *
     * @param aNC Number of constraints.
     */
    void setNumConstraints(const int aNC);

    /**
     * Set the derivative perturbation size.
     */
    void setDX(double aVal);

    /**
     * Set the derivative perturbation size for all controls.
     */
    void setDX(int aIndex, double aVal);

    /**
     * Get the derivative perturbation size.
     */
    double getDX(int aIndex);

    /**
     * Get a pointer to the vector of derivative perturbation sizes.
     */
    double* getDXArray();

    /**
     * Get an optimal force.
     */
    void getActuation(SimTK::State& s, const SimTK::Vector& parameters,
            SimTK::Vector& forces);

    void setActivationExponent(double aActivationExponent) {
        _activationExponent = aActivationExponent;
    }
    double getActivationExponent() const { return _activationExponent; }
    void setCurrentState(const SimTK::State* state) { _currentState = state; }
    const SimTK::State* getCurrentState() const { return _currentState; }

    // UTILITY

    /**
     * Ensure that a derivative perturbation is a valid size
     */
    void validatePerturbationSize(double& aSize);

    virtual void printPerformance(const SimTK::State& s, double* x);

    void computeActuatorAreas(const SimTK::State& s);

    /**
     * Compute derivatives of a constraint with respect to the
     * controls by central differences.
     *
     * @param aTarget Optimization target.
     * @param dx An array of control perturbation values.
     * @param x Values of the controls at time t.
     * @param jacobian System Jacobian matrix.
     *
     * @return -1 if an error is encountered, 0 otherwise.
     */
    static int CentralDifferencesConstraint(
            const StaticOptimizationTarget* aTarget, double* dx,
            const SimTK::Vector& x, SimTK::Matrix& jacobian);

    /**
     * Compute derivatives of performance with respect to the
     * controls by central differences. Note that the gradient array should
     * be allocated as dpdx[nx].
     *
     * @param aTarget Optimization target.
     * @param dx An array of control perturbation values.
     * @param x Values of the controls at time t.
     * @param dpdx The derivatives of the performance criterion.
     *
     * @return -1 if an error is encountered, 0 otherwise.
     */
    static int CentralDifferences(const StaticOptimizationTarget* aTarget,
            double* dx, const SimTK::Vector& x, SimTK::Vector& dpdx);

    bool prepareToOptimize(SimTK::State& s, double* x);

    //--------------------------------------------------------------------------
    // REQUIRED OPTIMIZATION TARGET METHODS
    //--------------------------------------------------------------------------

    /**
     * Compute the objective function given the optimization parameters.
     *
     * @param parameters Vector of optimization parameters.
     * @param new_parameters Flag indicating if the parameters have changed.
     * @param performance Value of the performance criterion.
     * @return Status (normal termination = 0, error < 0).
     */
    int objectiveFunc(const SimTK::Vector& parameters, bool new_parameters,
            SimTK::Real& performance) const override;

    /**
     * Compute the gradient of performance given parameters.
     *
     * @param parameters Vector of optimization parameters.
     * @param new_parameters Flag indicating if the parameters have changed.
     * @param gradient Derivatives of the cost function with respect to the
     * parameters.
     * @return Status (normal termination = 0, error < 0).
     */
    int gradientFunc(const SimTK::Vector& parameters, bool new_parameters,
            SimTK::Vector& gradient) const override;

    /**
     * Compute acceleration constraints given the optimization parameters.
     *
     * @param parameters Vector of optimization parameters.
     * @param new_parameters Flag indicating if the parameters have changed.
     * @param constraints Vector of optimization constraints.
     * @return Status (normal termination = 0, error < 0).
     */
    int constraintFunc(const SimTK::Vector& parameters, bool new_parameters,
            SimTK::Vector& constraints) const override;

    /**
     * Compute the constraint Jacobian given the optimization parameters.
     *
     * @param parameters Vector of parameters.
     * @param new_parameters Flag indicating if the parameters have changed.
     * @param jac The constraint Jacobian with respect to the optimization
     * parameters.
     * @return Status (normal termination = 0, error < 0).
     */
    int constraintJacobian(const SimTK::Vector& parameters, bool new_parameters,
            SimTK::Matrix& jac) const override;

private:
    /**
     * Compute all constraints given the optimization parameters.
     */
    void computeConstraintVector(SimTK::State& s,
            const SimTK::Vector& parameters, SimTK::Vector& constraints) const;
    void computeAcceleration(SimTK::State& s, const SimTK::Vector& parameters,
            SimTK::Vector& rAccel) const;
};

}; // namespace OpenSim

#endif // _StaticOptimizationTarget_h_
