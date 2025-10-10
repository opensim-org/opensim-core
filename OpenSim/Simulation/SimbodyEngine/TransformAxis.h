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
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
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

#include <OpenSim/Common/Assertion.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <SimTKcommon/internal/ReferencePtr.h>

namespace OpenSim {

class Joint;

/**
 * A class expressing a transformation of a child body in relation to a parent
 * body along either a translation or about a rotation axis. The TransformAxis
 * function specifies the spatial displacement that is achieved as a function
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
    OpenSim_DECLARE_LIST_PROPERTY(coordinates, std::string,
            "The names of the generalized coordinates that serve as the "
            "independent variables of the transform function.");

    OpenSim_DECLARE_PROPERTY(axis, SimTK::Vec3,
            "The rotation or translation axis for the transform. "
            "Default: (1,0,0).");

    OpenSim_DECLARE_PROPERTY(function, Function,
            "The transform function with respect to the generalized "
            "coordinates used to represent the amount of displacement about or "
            "along the TransformAxis. Default: Constant(0).");

//==============================================================================
// METHODS
//==============================================================================
    /**
     * Default constructor.
     */
    TransformAxis();

    /**
     * Construct a TransformAxis with the specified coordinate names and axis.
     * The function is set to a Constant(0) by default.
     * @param coordNames   Names of the generalized coordinates that affect
     *                     motion along this axis.
     * @param axis         The rotation or translation axis for the transform.
     */
    TransformAxis(const Array<std::string>& coordNames,
            const SimTK::Vec3& axis);

    /**
     * Construct a TransformAxis from an XML element node.
     */
    explicit TransformAxis(SimTK::Xml::Element& node);

    //** @name Accessors */
    // @{

    /**
     * %Set the names of the generalized coordinates that serve as the
     * independent variables of the Function in this TransformAxis object.
     *
     * @param coordNames   Names of the generalized coordinates.
     */
    void setCoordinateNames(const Array<std::string>& coordNames);

    /**
     * Get the names of the generalized coordinates that serve as the
     * independent variables of the Function in this TransformAxis object.
     *
     * The returned value is a references to the Property\<std::string> that
     * contains the list of coordinate names.
     *
     * @see get_coordinates()
     */
    const Property<std::string>& getCoordinateNames() const;

    /**
     * Get a copy of the coordinate names returned as an OpenSim::Array.
     **/
    Array<std::string> getCoordinateNamesInArray() const;

    /**
     * %Set the rotation or translation axis for the transform.
     */
    void setAxis(const SimTK::Vec3& axis);

    /**
     * Get the rotation or translation axis for the transform.
     */
    const SimTK::Vec3& getAxis() const;
    /// @copydoc getAxis() const
    void getAxis(SimTK::Vec3& axis) const;

    /**
     * Get one component (0, 1, or 2) of the axis vector.
     */
    double getAxis(int which) const;

    /**
     * Determine whether user-supplied function is present in the "function"
     * property.
     *
     * This returns false if the property contains the default Constant(0)
     * function, and true if it contains any other function.
     */
    bool hasFunction() const;

    /**
     * Get the transform function with respect to the generalized
     * coordinates used to represent the amount of displacement about or
     * along the TransformAxis.
     */
    const Function& getFunction() const;

    /**
     * Get writable access to the transform function with respect to the
     * generalized coordinates used to represent the amount of displacement
     * about or along the TransformAxis.
     */
    Function& updFunction();

    /**
     * %Set the transform function with respect to the generalized coordinates
     * used to represent the amount of displacement about or along the
     * TransformAxis.
     *
     * @note The TransformAxis adopts ownership of the Function object. Do not
     * delete it yourself! It will be deleted when the TransformAxis object is
     * deleted.
     */
    void setFunction(Function* function);

    /**
     * %Set the custom function that maps between the generalized coordinates
     * and the amount of transformation about/along the specified axis.
     *
     * @note This method creates a \e copy of the supplied Function object. Use
     * TransformAxis::setFunction(Function*) if you want the TransformAxis to
     * take ownership of the Function object.
     */
    void setFunction(const Function& function);

    /**
     * Return a reference to the Joint to which this TransformAxis applies.
     */
    const Joint& getJoint() const;

    /**
     * Get the current value of the transform.
     */
    double getValue(const SimTK::State& s);

    // @}

    //** @name Joint methods */
    // @{
    /**
     * Connect the TransformAxis to its owning Joint after the model has been
     * deserialized or copied.
     */
    void connectToJoint(const Joint& owningJoint);
    // @}

private:
    // OBJECT INTERFACE
    // Override virtual method from Object to give us a chance to replace the
    // deprecated property name "coordinate" (if it appears in \a node) with
    // the current property name "coordinates". Then we'll invoke the base
    // class implementation.
    void updateFromXMLNode(SimTK::Xml::Element& node,
            int versionNumber=-1) override;

    // CONVENIENCE METHODS
    void constructProperties();
    // Set the data members of this TransformAxis to their null values.
    void setNull();

    // DATA
    // Pointer to the joint to which the coordinates belong. This is just a
    // reference -- don't delete it! ReferencePtr means it is zeroed on
    // construction, copy construction, and copy assignment.
    SimTK::ReferencePtr<const Joint> _joint;

};  // class TransformAxis

} // namespace OpenSim

#endif // OPENSIM_TRANSFORM_AXIS_H_


