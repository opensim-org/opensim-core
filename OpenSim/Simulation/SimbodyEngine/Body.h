#ifndef OPENSIM_BODY_H_
#define OPENSIM_BODY_H_
/* -------------------------------------------------------------------------- *
 *                              OpenSim:  Body.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Ayman Habib, Matt DeMers                             *
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
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * An OpenSim::Body is a PhysicalFrame (reference frame) with associated 
 * inertia specified by its mass, center-of-mass located in the PhysicalFrame,
 * and its moment of inertia tensor about the center-of-mass.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API Body : public PhysicalFrame {
    OpenSim_DECLARE_CONCRETE_OBJECT(Body, PhysicalFrame);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with a Body. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(mass, double, 
        "The mass of the body (kg)");

    OpenSim_DECLARE_PROPERTY(mass_center, SimTK::Vec3, 
        "The location (Vec3) of the mass center in the body frame.");

    OpenSim_DECLARE_PROPERTY(inertia, SimTK::Vec6, 
        "The elements of the inertia tensor (Vec6) as [Ixx Iyy Izz Ixy Ixz Iyz] "
        "measured about the mass_center and not the body origin.");
    /**@}**/

//=============================================================================
// PUBLIC METHODS
//=============================================================================

public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /** default contructor*/
    Body();

    /** Convenience constructor */  
    Body(const std::string &aName, double aMass, const SimTK::Vec3& aMassCenter,
        const SimTK::Inertia& aInertia);

    // use compiler generated destructor, copy constructor and assignment operator

    /** Access Properties of the Body */
    /** The mass of the body in kg */
    const double& getMass() const { return get_mass(); }
    void setMass(const double& mass) { set_mass(mass); }

    /** The body center of mass location (Vec3) in the Body frame. */
    const SimTK::Vec3& getMassCenter() const { return get_mass_center(); }
    void setMassCenter(const SimTK::Vec3& com) { return set_mass_center(com); }

    /** The body's inertia about the center of mass location. */
    const SimTK::Inertia& getInertia() const;
    void setInertia(const SimTK::Inertia& aInertia);

    /** Assemble body interial properties: mass, center of mass location, moment 
        of inertia about the origin of the body and return as
        SimTK::MassProperties.
     */
    SimTK::MassProperties getMassProperties() const;

    void scale(const SimTK::Vec3& aScaleFactors, bool aScaleMass = false);
    void scaleInertialProperties(const SimTK::Vec3& aScaleFactors, bool aScaleMass = true);
    void scaleMass(double aScaleFactor);
    /** Add a Mesh specified by file name to the list of Geometry owned by the Body.
        Transform is assumed to be the same as the Body.
        Scale defaults to 1.0 but can be changed on the call line.
    */
    void addMeshGeometry(const std::string &aGeometryFileName, const SimTK::Vec3 scale = SimTK::Vec3(1));
 protected:

    // Model component interface.
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& model) override;

    // Underlying multibody tree building operations. Should only be called
    // by the connecting Joint
    Body* addSlave();

private:
    /** Component Interface */
    void constructProperties();

    /** Override of the default implementation to account for versioning. */
    void updateFromXMLNode(SimTK::Xml::Element& aNode,
        int versionNumber = -1) override;

    /** Return the equivalent (internal) SimTK::Rigid::Body for this body.
    Not valid until after extendAddToSystem on Body has been called.*/
    const SimTK::Body& extractInternalRigidBody() const override {
        return _internalRigidBody;
    }

    /** Convert old format Geometry version 3.2 to recent 4.0 format */
    void convertDisplayGeometryToGeometryXML(SimTK::Xml::Element& aNode, 
                                             const SimTK::Vec3& outerScaleFactors, 
                                             const SimTK::Vec6& outerTransform, 
                                             SimTK::Xml::Element& geomSetElement) const;
    void createFrameForXform(const SimTK::Xml::element_iterator&, const std::string& frameName, 
                                            const SimTK::Vec6& localXform, const std::string& bodyName) const;
    // mutable because fist get constructs tensor from properties
    mutable SimTK::Inertia _inertia;

    SimTK::Array_<Body*> _slaves;

    // Internal use for a Master body. Differs from its public MassProperties
    // which is the "effective" mass of the Body including internal slave
    // Bodies. This is just the Rigid::Body of an individual master/ slave body,
    // which will differ ONLY if there are slaves of a composite (master) Body 
    // used to break loops.
    SimTK::Body::Rigid _internalRigidBody;

    // Have to be at the Model level to build the system topology. This
    // involves splitting bodies to satsify loop constraints. Only the Model,
    // therefore, can tell a Body if it must add slaves to implement a valid
    // Multibody tree.
    friend class Model;

    // Joint is a friend because it creates the underlying RigidBody
    // and sets its mass properties.
    friend class Joint;

//=============================================================================
};  // END of class Body
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_BODY_H_


