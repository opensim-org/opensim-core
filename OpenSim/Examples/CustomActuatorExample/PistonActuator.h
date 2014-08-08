#ifndef _PistonActuator_h_
#define _PistonActuator_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PistonActuator.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matt S. DeMers                                                  *
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

/*
 * Author: Matt DeMers
 */

#include <OpenSim/OpenSim.h>

//=============================================================================
//=============================================================================
/**
 * A class that implements a force actuator acting between two points on two bodies.
 * The direction of the force is along the line between the points, with a positive
 * value acting to exapnd the distance between them.  This actuator has no states; 
 * the control is simply the force to be applied to the model.
 *
 * @author Matt DeMers
 * @version 2.0
 */
namespace OpenSim { 

class Body;
class Model;

class PistonActuator : public Actuator {
OpenSim_DECLARE_CONCRETE_OBJECT(PistonActuator, Actuator);

//=============================================================================
// DATA
//=============================================================================
protected:
    // PROPERTIES

    /** Name of Body to which the Body actuator is applied. */
    OpenSim_DECLARE_PROPERTY(bodyA, std::string,
    "Name of Body to which the Body actuator is applied.");

    /** Name of Body to which the equal and opposite torque is applied. */
    OpenSim_DECLARE_PROPERTY(bodyB, std::string,
    "Name of Body to which the equal and opposite torque is applied.");

    /** Point of application on each body. */
    OpenSim_DECLARE_PROPERTY(pointA, SimTK::Vec3,
    "Point of application on each body.");

    /** Name of Body to which the equal and opposite torque is applied. */
    OpenSim_DECLARE_PROPERTY(pointB, SimTK::Vec3,
    "Point of application on each body.");

    /** bool to indicate whether or not the points are expressed in global frame*/
    OpenSim_DECLARE_PROPERTY(points_are_global, bool,
    "bool to indicate whether or not the points are expressed in global frame.");

    /** Optimal force. */
    OpenSim_DECLARE_PROPERTY(optimal_force, double,
    "Optimal force.");

    /** Corresponding Body to which the force actuator is applied. */
    Body *_bodyA;

    /** Corresponding Body to which the equal and force torque is applied. */
    Body *_bodyB;

    // INTERNAL WORKING VARIABLES

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    PistonActuator( std::string aBodyNameA="", std::string abodyNameB="");
    virtual ~PistonActuator();
private:
    void setNull();
    void constructProperties();

public:

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    // GENERALIZED Body
    void setBodyA(Body* aBody);
    void setBodyB(Body* aBody);
    Body* getBodyA() const;
    Body* getBodyB() const;

    // Force points of application
    void setPointA(SimTK::Vec3 aPosition) { set_pointA(aPosition); } ;
    SimTK::Vec3 getPointA() const { return get_pointA(); };
    void setPointB(SimTK::Vec3 aPosition) { set_pointA(aPosition); } ;
    SimTK::Vec3 getPointB() const { return get_pointB(); };

    // flag for reference frame
    void setPointsAreGlobal(bool aBool) {set_points_are_global(aBool); };
    bool getPointsAreGlobal() {return get_points_are_global(); };

    // OPTIMAL FORCE
    void setOptimalForce(double aOptimalForce);
    double getOptimalForce() const;
    // STRESS
#ifndef SWIG
    double getStress( const SimTK::State& s ) const;
    //--------------------------------------------------------------------------
    // APPLICATION
    //--------------------------------------------------------------------------
    virtual void computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                              SimTK::Vector& generalizedForces) const;

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    
    virtual double  computeActuation( const SimTK::State& s) const;

#endif
    // Setup method to initialize Body reference
    void connectToModel(Model& aModel) override;

    //--------------------------------------------------------------------------
    // XML
    //--------------------------------------------------------------------------
    virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

//=============================================================================
};	// END of class PistonActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // __PistonActuator_h__
