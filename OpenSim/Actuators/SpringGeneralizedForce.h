#ifndef OPENSIM_SPRING_GENERALIZED_FORCE_H_
#define OPENSIM_SPRING_GENERALIZED_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  SpringGeneralizedForce.h                     *
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

//==============================================================================
// INCLUDES
//==============================================================================
#include "osimActuatorsDLL.h"
#include <OpenSim/Simulation/Model/Force.h>

//==============================================================================
//==============================================================================
namespace OpenSim {

class Coordinate;

/**
 * A Force that exerts a generalized force based on spring-like
 * characteristics (stiffness and viscosity).  
 *
 * @author Frank C. Anderson, Ajay Seth
 * @version 2.0
 */
class OSIMACTUATORS_API SpringGeneralizedForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(SpringGeneralizedForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_OPTIONAL_PROPERTY(coordinate, std::string,
        "Name of the coordinate to which this force is applied.");
    OpenSim_DECLARE_PROPERTY(stiffness, double,
        "Spring stiffness.");
    OpenSim_DECLARE_PROPERTY(rest_length, double,
        "Coordinate value at which spring produces no force.");
    OpenSim_DECLARE_PROPERTY(viscosity, double,
        "Damping constant.");


//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** This serves as default constructor or you can specify the coordinate
    name. A name of "" is treated as though unspecified. **/
    SpringGeneralizedForce(const std::string& coordinateName="");

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------

    // STIFFNESS
    void setStiffness(double aStiffness);
    double getStiffness() const;
    // REST LENGTH
    void setRestLength(double aRestLength);
    double getRestLength() const;
    // VISCOSITY
    void setViscosity(double aViscosity);
    double getViscosity() const;
    /** 
     * Methods to query a Force for the value actually applied during simulation
     * The names of the quantities (column labels) is returned by this first function
     * getRecordLabels()
     */
    OpenSim::Array<std::string> getRecordLabels() const override ;
    /**
     * Given SimTK::State object extract all the values necessary to report forces, application location
     * frame, etc. used in conjunction with getRecordLabels and should return same size Array
     */
    OpenSim::Array<double> getRecordValues(const SimTK::State& state) const override ;

    //--------------------------------------------------------------------------
    // COMPUTATIONS
protected:
    // Force interface.
    void computeForce(  const SimTK::State& state, 
                        SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                        SimTK::Vector& mobilityForces) const override;
    
    // ModelComponent interface.
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    // Setup method to initialize coordinate reference
    void extendConnectToModel(Model& model) override;

private:
    void setNull();
    void constructProperties();
    double computeForceMagnitude(const SimTK::State& s) const;

    // Set the Coordinate pointer, and set the corresponding name property
    // to match.
    void setCoordinate(Coordinate* coordinate);
    Coordinate* getCoordinate() const;

    // Note: reference pointers are automatically set to null on construction 
    // and also on copy construction and copy assignment.

    // Corresponding generalized coordinate to which the coordinate actuator
    // is applied.
    SimTK::ReferencePtr<Coordinate> _coord;

    //==============================================================================
};  // END of class SpringGeneralizedForce

}; //namespace
//==============================================================================
//==============================================================================


#endif // OPENSIM_SPRING_GENERALIZED_FORCE_H_
