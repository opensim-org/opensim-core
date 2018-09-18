#ifndef MUSCOLLO_MUCOCONSTRAINT_H
#define MUSCOLLO_MUCOCONSTRAINT_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoConstraint.h                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include "MucoBounds.h"
#include "MuscolloUtilities.h"

#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/osimSimulation.h>

#include <simbody/internal/Constraint.h>


namespace OpenSim {

// ============================================================================
// MucoConstraint
// ============================================================================

class OSIMMUSCOLLO_API MucoConstraint : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoConstraint, Object);
public:
    // Default constructor.
    MucoConstraint();
    // Constructor for holonomic constraints.
    MucoConstraint(const std::string& name, 
        const std::vector<MucoBounds>& bounds, 
        const std::vector<std::string>& suffixes);
    // TODO Generic constructor

    // Get and set methods.
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MucoBounds getBoundsAtIndex(const int& idx) const
    {   return MucoBounds(get_lower_bounds(idx), get_upper_bounds(idx)); }

    // TODO add check that property has correct number of elements
    int getNumberScalarEquations() const 
    {   return getProperty_lower_bounds().size(); }

    void setBoundsAtIndex(const int& idx, const MucoBounds& bounds) {
        upd_lower_bounds(idx) = bounds.getLower();
        upd_upper_bounds(idx) = bounds.getUpper();
    }

    // Function to calculate position errors in constraint equations.
    virtual void calcPositionErrors(const SimTK::State& state, 
        SimTK::Vector_<double> out) const;

    // TODO
    //virtual void calcVelocityErrors(const SimTK::State& state,
    //    SimTK::Vector_<double> out) const;
    //virtual void calcAccelerationErrors(const SimTK::State& state,
    //    SimTK::Vector_<double> out) const;

    

protected:
    OpenSim_DECLARE_LIST_PROPERTY(lower_bounds, double, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY(upper_bounds, double, "TODO");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(suffixes, std::vector<std::string>, 
        "TODO"); 



   int m_num_position_eqs;
   //TODO
   //int m_num_velocity_eqs;
   //int m_num_acceleration_eqs;


   void constructProperties();
};

// ============================================================================
// MucoSimbodyConstraint
// ============================================================================

class OSIMMUSCOLLO_API MucoSimbodyConstraint : public MucoConstraint {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoSimbodyConstraint, MucoConstraint);
public:
    // Default constructor.
    MucoSimbodyConstraint();
    // Generic constructor.
    MucoSimbodyConstraint(const std::string& name);

    void initialize(Model& model) const;

 
private:
    void calcAccelerationsFromMultipliers(const Model& model, 
        const SimTK::State& state, const SimTK::Vector& multipliers, 
        SimTK::Vector& udot) const;
    
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOCONSTRAINT_H