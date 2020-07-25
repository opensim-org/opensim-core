/* -------------------------------------------------------------------------- *
 *                          OpenSim:  Constraint.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
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
#include "Constraint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "simbody/internal/Constraint.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
//using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Constraint::Constraint() 
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Constraint::~Constraint()
{
}

//_____________________________________________________________________________
/**
 * Set the data members of this Constraint to their null values.
 */
void Constraint::setNull(void)
{
    setAuthors("Frank Anderson, Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Constraint::constructProperties(void)
{
    constructProperty_isEnforced(true);
}

void Constraint::updateFromXMLNode(SimTK::Xml::Element& node,
                                   int versionNumber) {
    if (versionNumber < XMLDocument::getLatestVersion()) {
        if (versionNumber < 30509) {
            // Rename property 'isDisabled' to 'isEnforced' and
            // negate the contained value.
            std::string oldName{ "isDisabled" };
            std::string newName{ "isEnforced" };
            if (node.hasElement(oldName)) {
                auto elem = node.getRequiredElement(oldName);
                bool isDisabled = false;
                elem.getValue().tryConvertToBool(isDisabled);

                // now update tag name to 'isEnforced'
                elem.setElementTag(newName);
                // update its value to be the opposite of 'isDisabled'
                elem.setValue(SimTK::String(!isDisabled));
            }
        }
    }

    Super::updateFromXMLNode(node, versionNumber);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this Constraint.
 */
void Constraint::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);
}

void Constraint::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
    SimTK::Constraint& simConstraint = 
        _model->updMatterSubsystem().updConstraint(_index);

    // Otherwise we have to change the status of the constraint
    if(get_isEnforced())
        simConstraint.enable(s);
    else
        simConstraint.disable(s);
}

void Constraint::extendSetPropertiesFromState(const SimTK::State& state)
{
    Super::extendSetPropertiesFromState(state);
    set_isEnforced(isEnforced(state));
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Update an existing Constraint with parameter values from a
 * new one, but only for the parameters that were explicitly
 * specified in the XML node.
 *
 * @param aConstraint Constraint to update from
 */
void Constraint::updateFromConstraint(SimTK::State& s,
                                      const Constraint &aConstraint)
{
    setIsEnforced(s, aConstraint.isEnforced(s));
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// ENFORCED
//-----------------------------------------------------------------------------

bool Constraint::isEnforced(const SimTK::State& s) const
{
    return !_model->updMatterSubsystem().updConstraint(_index).isDisabled(s);
}

bool Constraint::setIsEnforced(SimTK::State& s, bool isEnforced)
{
    SimTK::Constraint& simConstraint =
        _model->updMatterSubsystem().updConstraint(_index);
    bool modelConstraintIsEnforced = !simConstraint.isDisabled(s);

    // Check if we already have the correct enabling of the constraint then
    // do nothing 
    if(isEnforced == modelConstraintIsEnforced)
        return true;

    // Otherwise we have to change the status of the constraint
    if(isEnforced)
        simConstraint.enable(s);
    else
        simConstraint.disable(s);

    _model->updateAssemblyConditions(s);
    set_isEnforced(isEnforced);
    
    return true;
}


//-----------------------------------------------------------------------------
// FORCES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Ask the constraint for the forces it is imposing on the system
 * Simbody multibody system must be realized to at least position
 * Returns: the bodyForces on those bodies being constrained (constrainedBodies)
 *          a SpatialVec (6 components) describing resulting torque and force
 *          mobilityForces acting along constrained mobilities  
 *
 * @param state State of model
 * @param bodyForcesInAncestor is a Vector of SpatialVecs contain constraint 
          forces
 * @param mobilityForces is a Vector of forces that act along the constrained
 *         mobilities associated with this constraint
 */
void Constraint::
calcConstraintForces(const SimTK::State& s,
                     SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInAncestor, 
                     SimTK::Vector& mobilityForces) const {
    SimTK::Constraint& simConstraint =
        _model->updMatterSubsystem().updConstraint(_index);
    if(!simConstraint.isDisabled(s)){
        SimTK::Vector multipliers = simConstraint.getMultipliersAsVector(s);
        simConstraint.calcConstraintForcesFromMultipliers(s, multipliers,
                                                          bodyForcesInAncestor,
                                                          mobilityForces);
    }
}

/** 
 * Methods to query a Constraint forces for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
Array<std::string> Constraint::getRecordLabels() const
{
    SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_index);
    const SimTK::State &ds = _model->getWorkingState();

    // number of bodies being directly constrained
    int ncb = simConstraint.getNumConstrainedBodies();
    // number of mobilities being directly constrained
    int ncm = simConstraint.getNumConstrainedU(ds);

    const auto& physicalFrames = _model->getComponentList<PhysicalFrame>();
    
    Array<std::string> labels("");

    for(int i=0; i<ncb; ++i){
        const SimTK::MobilizedBody &b = simConstraint.getMobilizedBodyFromConstrainedBody(SimTK::ConstrainedBodyIndex(i));
        const SimTK::MobilizedBodyIndex &bx =  b.getMobilizedBodyIndex();
        const PhysicalFrame *frame = nullptr;
        for(auto& phf : physicalFrames){
            if(phf.getMobilizedBodyIndex() == bx){
                frame = &phf;
                break;
            }
        }
        if(frame == nullptr){
            throw Exception("Constraint "+getName()+" does not have an identifiable body index.");
        }
        string prefix = getName()+"_"+frame->getName();
        labels.append(prefix+"_Fx");
        labels.append(prefix+"_Fy");
        labels.append(prefix+"_Fz");
        labels.append(prefix+"_Mx");
        labels.append(prefix+"_My");
        labels.append(prefix+"_Mz");
    }

    for(int i=0; i<ncm; ++i){
        labels.append(fmt::format("{}_mobility_F{}", getName(), i));
    }
    
    return labels;
}

/**
 * Given SimTK::State object extract all the values necessary to report constraint forces, application 
 * location frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
Array<double> Constraint::getRecordValues(const SimTK::State& state) const
{
    // EOMs are solved for accelerations (udots) and constraint multipliers (lambdas)
    // simultaneously, so system must be realized to acceleration
    _model->getMultibodySystem().realize(state, SimTK::Stage::Acceleration);
    SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_index);

    // number of bodies being directly constrained
    int ncb = simConstraint.getNumConstrainedBodies();
    // number of mobilities being directly constrained
    int ncm = simConstraint.getNumConstrainedU(state);

    SimTK::Vector_<SimTK::SpatialVec> bodyForcesInAncestor(ncb);
    bodyForcesInAncestor.setToZero();
    SimTK::Vector mobilityForces(ncm, 0.0);

    Array<double> values(0.0,6*ncb+ncm);

    calcConstraintForces(state, bodyForcesInAncestor, mobilityForces);
    
    for(int i=0; i<ncb; ++i){
        for(int j=0; j<3; ++j){
            // Simbody constraints have reaction moments first and OpenSim reports forces first
            // so swap them here
            values[i*6+j] = (bodyForcesInAncestor(i)[1])[j]; // moments on constrained body i
            values[i*6+3+j] = (bodyForcesInAncestor(i)[0])[j]; // forces on constrained body i
        }
    }
    for(int i=0; i<ncm; ++i){
        values[6*ncb+i] = mobilityForces[i];
    }

    return values;
};
