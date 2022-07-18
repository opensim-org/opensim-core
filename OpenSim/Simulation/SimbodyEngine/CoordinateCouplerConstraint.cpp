/* -------------------------------------------------------------------------- *
 *                 OpenSim:  CoordinateCouplerConstraint.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Frank C. Anderson                                    *
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
#include "CoordinateCouplerConstraint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "simbody/internal/Constraint.h"

// Helper class to construct functions when user's specify a dependency as qd = f(qi)
// this function casts as C(q) = 0 = f(qi) - qd;

// Excluding this from Doxygen until it has better documentation! -Sam Hamner
    /// @cond
class CompoundFunction : public SimTK::Function {
// returns f1(x[0]) - x[1];
private:
    std::unique_ptr<const SimTK::Function> f1;
    const double scale;

public:
    
    CompoundFunction(const SimTK::Function *cf, double scale) : f1(cf), scale(scale) {
    }

    double calcValue(const SimTK::Vector& x) const override {
        SimTK::Vector xf(1);
        xf[0] = x[0];
        return scale*f1->calcValue(xf)-x[1];
    }

    double calcDerivative(const std::vector<int>& derivComponents, const SimTK::Vector& x) const {
        return calcDerivative(SimTK::ArrayViewConst_<int>(derivComponents),x); 
    }

    double calcDerivative(const SimTK::Array_<int>& derivComponents, const SimTK::Vector& x) const override {
        if (derivComponents.size() == 1){
            if (derivComponents[0]==0){
                SimTK::Vector x1(1);
                x1[0] = x[0];
                return scale*f1->calcDerivative(derivComponents, x1);
            }
            else if (derivComponents[0]==1)
                return -1;
        }
        else if(derivComponents.size() == 2){
            if (derivComponents[0]==0 && derivComponents[1] == 0){
                SimTK::Vector x1(1);
                x1[0] = x[0];
                return scale*f1->calcDerivative(derivComponents, x1);
            }
        }
        return 0;
    }

    int getArgumentSize() const override {
        return 2;
    }
    int getMaxDerivativeOrder() const override {
        return 2;
    }

    void setFunction(const SimTK::Function *cf) {
        f1.reset(cf);
    }
};


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
CoordinateCouplerConstraint::~CoordinateCouplerConstraint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
CoordinateCouplerConstraint::CoordinateCouplerConstraint() :
    Constraint()
{
    setNull();
    constructProperties();
}

//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Set the data members of this CoordinateCouplerConstraint to their null values.
 */
void CoordinateCouplerConstraint::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void CoordinateCouplerConstraint::constructProperties()
{
    // Coordinate Coupler Function
    constructProperty_coupled_coordinates_function();

    // coordinates that are coupled (by name)
    constructProperty_independent_coordinate_names();

    // coordinates that are coupled (by name)
    constructProperty_dependent_coordinate_name("");

    // scale factor
    constructProperty_scale_factor(1.0);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim Model containing this CoordinateCouplerConstraint.
 */
void CoordinateCouplerConstraint::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    string errorMessage;

    // Look up the bodies and coordinates being coupled by name in the
    // model and keep lists of their indices

    Array<std::string> independentCoordNames;

    for(int i = 0; i < getProperty_independent_coordinate_names().size(); i++) {
        independentCoordNames.append(get_independent_coordinate_names(i));
    }

    for(int i=0; i<independentCoordNames.getSize(); i++){
        if (!getModel().getCoordinateSet().contains(independentCoordNames[i])) {
            errorMessage = "Coordinate coupler: unknown independent coordinate " ;
            errorMessage += independentCoordNames[i];
            throw (Exception(errorMessage));
        }
    }

    // Last coordinate in the coupler is the dependent coordinate
    if (!getModel().getCoordinateSet().contains(get_dependent_coordinate_name())) {
        errorMessage = "Coordinate coupler: unknown dependent coordinate " ;
        errorMessage += get_dependent_coordinate_name();
        throw (Exception(errorMessage));
    }
}


void CoordinateCouplerConstraint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    /** List of mobilized body indices established when constraint is set up */
    std::vector<SimTK::MobilizedBodyIndex> mob_bodies;
    /** List of coordinate (Q) indices corresponding to the respective body */
    std::vector<SimTK::MobilizerQIndex> mob_qs;

    string errorMessage;

    // Look up the bodies and coordinates being coupled by name in the
    // model and keep lists of their indices
    Array<std::string> independentCoordNames;

    for(int i = 0; i < getProperty_independent_coordinate_names().size(); i++) {
        independentCoordNames.append(get_independent_coordinate_names(i));
    }

    for(int i=0; i<independentCoordNames.getSize(); i++){
        // Error checking was handled in extendConnectToModel()
        const Coordinate& aCoordinate = getModel().getCoordinateSet().get(independentCoordNames[i]);
        mob_bodies.push_back(aCoordinate._bodyIndex);
        mob_qs.push_back(SimTK::MobilizerQIndex(aCoordinate._mobilizerQIndex));
    }

    // Last coordinate in the coupler is the dependent coordinate
    const Coordinate& aCoordinate = getModel().getCoordinateSet().get(get_dependent_coordinate_name());
    mob_bodies.push_back(aCoordinate._bodyIndex);
    mob_qs.push_back(SimTK::MobilizerQIndex(aCoordinate._mobilizerQIndex));

    if (!mob_qs.size() & (mob_qs.size() != mob_bodies.size())) {
        errorMessage = "CoordinateCouplerConstraint:: requires at least one body and coordinate." ;
        throw (Exception(errorMessage));
    }

    // Create and set the underlying coupler constraint function;
    const Function& f = getFunction();
    SimTK::Function *simtkCouplerFunction = new CompoundFunction(f.createSimTKFunction(), get_scale_factor());


    // Now create a Simbody Constraint::CoordinateCoupler
    SimTK::Constraint::CoordinateCoupler simtkCoordinateCoupler(system.updMatterSubsystem() ,
                                                                simtkCouplerFunction, 
                                                                mob_bodies, mob_qs);

    // Beyond the const Component get the index so we can access the SimTK::Constraint later
    assignConstraintIndex(simtkCoordinateCoupler.getConstraintIndex());
}

//==============================================================================
// SCALE
//==============================================================================
void CoordinateCouplerConstraint::
extendScale(const SimTK::State& s, const ScaleSet& scaleSet)
{
    Super::extendScale(s, scaleSet);

    Coordinate& depCoordinate =
        _model->updCoordinateSet().get(get_dependent_coordinate_name());

    // Only scale if the dependent coordinate is a translation.
    if (depCoordinate.getMotionType() != Coordinate::Translational)
        return;

    // Get scale factors (if there exists an entry for the base Body of the
    // Joint's parent Frame).
    const Vec3& scaleFactors =
        getScaleFactors(scaleSet, depCoordinate.getJoint().getParentFrame());
    if (scaleFactors == ModelComponent::InvalidScaleFactors)
        return;

    // Constraint scale factor. Assume uniform scaling unless proven otherwise.
    const double scaleFactor = scaleFactors[0];

    // We can handle non-uniform scaling along transform axes of custom joints
    // ONLY at this time.
    const Joint *joint =  dynamic_cast<const Joint*>(depCoordinate._joint.get());
    if (joint) {
        if (scaleFactors[0] != scaleFactors[1] ||
            scaleFactors[0] != scaleFactors[2] )
        {
            // TODO: Non-uniform scaling remains undefined! - ASeth
            throw(Exception("Non-uniform scaling of CoordinateCoupler constraints not implemented."));
        }
    }

    upd_scale_factor() *= scaleFactor;
}
