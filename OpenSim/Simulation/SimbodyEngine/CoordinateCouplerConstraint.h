#ifndef OPENSIM_COORDINATE_COUPLER_CONSTRAINT_H_
#define OPENSIM_COORDINATE_COUPLER_CONSTRAINT_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  CoordinateCouplerConstraint.h                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

// INCLUDE
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Function.h>
#include "Constraint.h"

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
 /** @file
 * A class implementing a CoordinateCoupler Constraint.  The underlying SimTK
 * Constraint is a Constraint::CoordinateCoupler in Simbody, which relates 
 * coordinates to one another at the position level (i.e. holonomic).
 * Relationship between coordinates is a specified by a function that equates 
 * to zero only when the coordinates satisfy the constraint function.
 *
 * OpenSim::CoordinateCouplerConstraint assumes that there is one coordinate
 * (dependent) whose value is coupled to the value(s) of other (independent)
 * coordinate(s). In reality all coordinates are coupled and at assembly all
 * can be varied to satsify the constraint function.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API CoordinateCouplerConstraint : public Constraint {
OpenSim_DECLARE_CONCRETE_OBJECT(CoordinateCouplerConstraint, Constraint);

//=============================================================================
// DATA
//=============================================================================
protected:

    OpenSim_DECLARE_OPTIONAL_PROPERTY(coupled_coordinates_function, Function,
        "Constraint function of generalized coordinates (to be specified) used "
        "to evaluate the constraint errors and their derivatives, and must valid "
        "to at least 2nd order. Constraint function must evaluate to zero when "
        "coordinates satisfy the constraint.");

    OpenSim_DECLARE_LIST_PROPERTY(independent_coordinate_names, std::string, 
        "List of names of the right hand side (independent) coordinates. "
        "Note the conatraint function above, must be able to handle multiple "
        "coordinate values if more than one coordinate name is provided.");

    OpenSim_DECLARE_PROPERTY(dependent_coordinate_name, std::string, 
        "Name of the left-hand side (dependent) coordinate of the constraint "
        "coupling function.");
    
    OpenSim_DECLARE_PROPERTY(scale_factor, double, 
        "Scale factor for the coupling function.");

//=============================================================================
// METHODS
//=============================================================================
public:
    // CONSTRUCTION
    CoordinateCouplerConstraint();
    virtual ~CoordinateCouplerConstraint();

    // GET AND SET
    /** Access the list of names of the right hand side (independent) coordinates. 
        Note the conatraint function, must be able to handle multiple 
        coordinate values if more than one coordinate name is provided. */
    void setIndependentCoordinateNames(const Array<std::string> &aCoordNames) 
        { set_independent_coordinate_names(aCoordNames); }
    const Array<std::string> getIndependentCoordinateNames() const { 
        Array<std::string> coords;
        for(int i = 0; i < getProperty_independent_coordinate_names().size(); i++) {
            coords.append(get_independent_coordinate_names(i));
        }
        return coords; 
    }
    
    void setDependentCoordinateName(const std::string &aCoordName)
        { set_dependent_coordinate_name(aCoordName); }
    
    const std::string& getDependentCoordinateName() const { return get_dependent_coordinate_name(); }
    
    const Function& getFunction() const {
        const Property<Function>& function = getProperty_coupled_coordinates_function();
        if(function.empty()) {
            throw Exception("CoordinateCouplerConstraint::getFunction(): no Function is defined");
        }
        return function.getValue(); 
    }
    void setFunction(const Function &aFunction)
        { set_coupled_coordinates_function(*aFunction.clone());}
    void setFunction(Function *aFunction)
        { set_coupled_coordinates_function(*aFunction); }

    // SCALE
    virtual void scale(const ScaleSet& aScaleSet);

protected:
    void connectToModel(Model& aModel) override;
    /**
     * Create a SimTK::Constraint::CoordinateCooupler which implements this constraint.
     */
    void addToSystem(SimTK::MultibodySystem& system) const override;

private:
    void setNull();
    void constructProperties();
    friend class SimbodyEngine;

//=============================================================================
};  // END of class CoordinateCouplerConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_COORDINATE_COUPLER_CONSTRAINT_H_


