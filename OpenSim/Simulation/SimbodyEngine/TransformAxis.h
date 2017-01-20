#ifndef OPENSIM_TRANSFORM_AXIS_H_
#define OPENSIM_TRANSFORM_AXIS_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  TransformAxis.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan, Frank C. Anderson, Jeffrey A. Reinbolt, Ajay Seth   *
 *            Michael Sherman                                                 *
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
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <SimTKcommon/internal/ReferencePtr.h>

namespace OpenSim {

class Joint;

//==============================================================================
//                            TRANSFORM AXIS
//==============================================================================
/**
 * A class expressing a transformation of a child body in relation to a parent
 * body along either a translation or about a rotation axis. The TransformAxis
 * function specified the spatial displacement that is achieved as a function
 * of the generalized coordinate(s).
 *
 * @author Peter Loan, Frank C. Anderson, Jeffrey A. Reinbolt, Ajay Seth,
 *         Michael Sherman
 */
class OSIMSIMULATION_API TransformAxis : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(TransformAxis, Object);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** The "coordinates" property holds a list of strings representing the
    generalized coordinate names that serve as the independent variables of
    the transform function. **/
    OpenSim_DECLARE_LIST_PROPERTY(coordinates, std::string,
        "Names of the coordinates that serve as the independent variables \
        of the transform function.");

    /** The "axis" property holds the axis direction of the rotation or 
    translation axis of the transform as a Vec3. **/
    OpenSim_DECLARE_PROPERTY(axis, SimTK::Vec3,
       "Rotation or translation axis for the transform.");

    /** The "function" property holds the transform function of the 
    generalized coordinate(s) used to represent the amount of displacement 
    about/along the specified axis. If none is specified a Constant function
    is applied. **/
    OpenSim_DECLARE_PROPERTY(function, Function,
       "Transform function of the generalized coordinates used to \
       represent the amount of displacement along a specified axis.");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    TransformAxis();
    TransformAxis(const Array<std::string>& coordNames, 
                  const SimTK::Vec3&        axis);
    explicit TransformAxis(SimTK::Xml::Element& node);

    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.

    /** %Set the names of the generalized coordinates that affect the motion
    along the axis controlled by this %TransformAxis object.
    @param coordNames   Names of the generalized coordinates. **/
    void setCoordinateNames(const Array<std::string>& coordNames) 
    {   set_coordinates(coordNames); }
    
    /** Get the generalized coordinate names associated with this object.
    The returned value is a references to the Property\<string> that contains
    the list of coordinate names.
    @see get_coordinates() **/
    const Property<std::string>& getCoordinateNames() const 
    {   return getProperty_coordinates(); }

    /** Copy the coordinate names into an OpenSim::Array for convenience. **/
    // The GUI uses this.
    Array<std::string> getCoordinateNamesInArray() const {
        Array<std::string> coords;
        for (int i=0; i < getProperty_coordinates().size(); ++i)
            coords.append(get_coordinates(i));
        return coords;
    }

    /** %Set the value of the "axis" property. **/
    void setAxis(const SimTK::Vec3& axis) 
    {   set_axis(axis); }

    /** Return the current value of the "axis" property. **/
    const SimTK::Vec3& getAxis() const
    {   return get_axis(); }

    /** Alternate signature that writes the axis value to its argument. **/
    void getAxis(SimTK::Vec3& axis) const {axis = getAxis();}
    /** Get one component (0,1, or 2) of the axis vector. **/
    double getAxis(int which) const 
    {   assert(0<=which && which<=2); return getAxis()[which]; }

    /** Determine whether a custom function has been specified to map between 
    the generalized coordinate and the amount of transformation along the 
    specified axis. **/
    bool hasFunction() const 
    {   return !getProperty_function().empty();}

    /** Get the custom function that maps between the generalized coordinates 
    and the amount of displacement along the specified axis. If no function 
    has been specified, this throws an exception; check first with hasFunction()
    if you aren't sure. **/
    const Function& getFunction() const;
    /** Get writable access to the transform function. **/
    Function& updFunction();

    /** %Set the custom function that maps between the generalized coordinates 
    and the amount of displacement about/along the specified axis. This object 
    adopts ownership of the Function object, don't delete it yourself! It will
    be deleted when this %TransformAxis object is deleted. **/
    void setFunction(Function* function);

    /** %Set the custom function that maps between the generalized coordinates
    and the amount of transformation about/along the specified axis. This method 
    creates a \e copy of the supplied Function object, which is unaffected.
    Use the other signature if you want this %TransformAxis to take over 
    ownership of the Function object. **/
    void setFunction(const Function& function);

    /** Return a reference to the Joint to which this %TransformAxis 
    applies. **/
    const Joint& getJoint() const { return *_joint; }

    double getValue(const SimTK::State& s );


    /** Connect the %TransformAxis to its owning Joint after the model has
    been deserialized or copied. **/
    void connectToJoint(const Joint& owningJoint);

private:
    // Override virtual method from Object to give us a chance to replace the
    // deprecated property name "coordinate" (if it appears in \a node) with 
    // the current property name "coordinates". Then we'll invoke the base
    // class implementation.
    void updateFromXMLNode(SimTK::Xml::Element& node, 
                           int                  versionNumber=-1) override;

    void setNull();
    void constructProperties();


//==============================================================================
// DATA
//==============================================================================
private:
    // Pointer to the joint to which the coordinates belong. This is just a
    // reference -- don't delete it! ReferencePtr means it is zeroed on 
    // construction, copy construction, and copy assignment.
    SimTK::ReferencePtr<const Joint> _joint;


//==============================================================================
};  // END of class TransformAxis
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_TRANSFORM_AXIS_H_


