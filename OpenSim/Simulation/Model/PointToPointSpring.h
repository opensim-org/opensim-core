#ifndef OPENSIM_POINT_TO_POINT_SPRING_H_
#define OPENSIM_POINT_TO_POINT_SPRING_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  PointToPointSpring.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include "Force.h"

//==============================================================================
//==============================================================================
namespace OpenSim { 

class Model;
class PhysicalFrame;

/**
 * A simple point to point spring with a resting length and stiffness.
 * Points are connected to bodies and are defined in the body frame.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API PointToPointSpring : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(PointToPointSpring, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(point1, SimTK::Vec3,
        "Spring attachment point on body1.");
    OpenSim_DECLARE_PROPERTY(point2, SimTK::Vec3,
        "Spring attachment point on body2.");
    OpenSim_DECLARE_PROPERTY(stiffness, double,
        "Spring stiffness (N/m).");
    OpenSim_DECLARE_PROPERTY(rest_length, double,
        "Spring resting length (m).");

//==============================================================================
// SOCKETS
//==============================================================================
    OpenSim_DECLARE_SOCKET(body1, PhysicalFrame,
        "A frame on the first body that this spring connects to.");
    OpenSim_DECLARE_SOCKET(body2, PhysicalFrame,
        "A frame on the second body that this spring connects to.");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor. **/
    PointToPointSpring();
    /** Convenience constructor for API users.
    @param body1        the first body to which the spring is attached
    @param point1       location where spring is attached on body1
    @param body2        the second body to which the spring is attached
    @param point2       location where spring is attached on body2 
    @param stiffness    spring stiffness
    @param restlength   the resting (zero force) length of the spring
    **/
    PointToPointSpring( const PhysicalFrame& body1, SimTK::Vec3 point1,
                        const PhysicalFrame& body2, SimTK::Vec3 point2,
                        double stiffness, double restlength );

    // default destructor, copy constructor, copy assignment
    
    //-----------------------------------------------------------------------------
    // GET and SET Spring parameters
    //-----------------------------------------------------------------------------
    /**
    * Spring end point bodies 
    */
    void setBody1(const PhysicalFrame& body);
    void setBody2(const PhysicalFrame& body);
    const PhysicalFrame& getBody1() const;
    const PhysicalFrame& getBody2() const;

    /**
    * Spring end points 
    */
    void setPoint1(SimTK::Vec3 aPosition) { set_point1(aPosition); }
    const SimTK::Vec3& getPoint1() const { return get_point1(); }
    void setPoint2(SimTK::Vec3 aPosition) { set_point2(aPosition); }
    const SimTK::Vec3& getPoint2() const { return get_point2(); }

    /**
    * Spring stiffness
    * @param stiffness 
    */
    void setStiffness(double stiffness) {set_stiffness(stiffness);}
    double getStiffness() const {return get_stiffness();}
    /**
    * Spring resting length
    * @param restLength 
    */
    void setRestlength(double restLength) {set_rest_length(restLength);}
    double getRestlength() const {return get_rest_length();}

    //-----------------------------------------------------------------------------
    // Reporting
    //-----------------------------------------------------------------------------
    /** 
     * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
     */
    OpenSim::Array<std::string> getRecordLabels() const override;
    /**
    *  Provide the value(s) to be reported that correspond to the labels
    */
    OpenSim::Array<double> getRecordValues(const SimTK::State& state) const override;

protected:
    //-----------------------------------------------------------------------------
    // ModelComponent interface
    //-----------------------------------------------------------------------------
    void extendAddToSystemAfterSubcomponents(SimTK::MultibodySystem& system) const override;

    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) override;

private:
    void setNull();
    void constructProperties();

//==============================================================================
};  // END of class PointToPointSpring

}; //namespace
//==============================================================================
//==============================================================================

#endif // OPENSIM_POINT_TO_POINT_SPRING_H_
