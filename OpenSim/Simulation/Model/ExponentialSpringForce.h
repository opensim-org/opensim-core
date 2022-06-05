#ifndef OPENSIM_EXPONENTIAL_CONTACT_H_
#define OPENSIM_EXPONENTIAL_CONTACT_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  ExponentialContactForce.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2022 Stanford University and the Authors                     *
 * Author(s): F. C. Anderson                                                  *
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
#include "SimTKsimbody.h"
#include "Force.h"
#include "OpenSim/Common/Set.h"

namespace OpenSim {


//=============================================================================
// ExponentialContact
//=============================================================================
/** This force subclass implements a SimTK::ExponentialSpringForce to model
contact of a specified point on a body (i.e., a "station" in Simbody vocabulary) with
a contact plane that is fixed to Ground.

@author F. C. Anderson **/
class OSIMSIMULATION_API ExponentialContact : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExponentialContact, Force);

public:
    class Parameters;

    //-------------------------------------------------------------------------
    // PROPERTIES
    //-------------------------------------------------------------------------
    OpenSim_DECLARE_PROPERTY(contact_plane_transform, SimTK::Transform,
        "Location and orientation of the contact plane wrt Ground. \
        The positive z-axis of the contact plane defines the normal.");
    OpenSim_DECLARE_PROPERTY(body_name, std::string,
        "Name of the Body to which the resultant contact force is applied.");
    OpenSim_DECLARE_PROPERTY(body_station, SimTK::Vec3,
        "Point on the body, expressed in the Body frame, at which the \
        resultant contact force is applied.");
    OpenSim_DECLARE_PROPERTY(contact_parameters, ExponentialContact::Parameters,
        "Customizable topology-stage parameters.");

    //-------------------------------------------------------------------------
    // Construction
    //-------------------------------------------------------------------------
    /** Default constructor. */
    ExponentialContact();

    /** Construct an ExponentialContact.
    @param XContactPlane Transform specifying the location and orientation of
    the contact plane in Ground.
    @param bodyName Name of the body to which the force is applied.
    @param station Point on the body at which the force is applied.
    @param params Optional parameters object used to customize the
    topology-stage characteristics of the contact model. */
    explicit ExponentialContact(const SimTK::Transform& contactPlaneXform,
        const std::string& bodyName, const SimTK::Vec3& station,
        SimTK::ExponentialSpringParameters params =
        SimTK::ExponentialSpringParameters());

    /** Copy constructor. */
    ExponentialContact(const ExponentialContact& other);

    //-------------------------------------------------------------------------
    // Accessors
    //-------------------------------------------------------------------------
    /** Set the tranform that specifies the location and orientation of the
    contact plane in the Ground frame. */
    void setContactPlaneTransform(const SimTK::Transform& contactPlaneXform);
    /** Get the tranform that specifies the location and orientation of the
    contact plane in the Ground frame. */
    const SimTK::Transform& getContactPlaneTransform() const;

    /** Set the name of the body to which this force is applied. */
    void setBodyName(const std::string& bodyName) { set_body_name(bodyName); }
    /** Get the name of the body to which this force is applied. */
    const std::string& getBodyName() const { return get_body_name(); }

    /** Set the point on the body at which the force is applied. */
    void setBodyStation(const SimTK::Vec3& station) { set_body_station(station); }
    /** Get the point on the body at which the force is applied. */
    const SimTK::Vec3& getBodyStation() const { return get_body_station(); }

    /** Set the customizable Topology-stage spring parameters. Calling this
    method will invalidate the SimTK::System at Stage::Toplogy. The System
    must therefore be realized at Stage::Topology, as well subsequent stages
    when appropriate, before simulation or analysis can proceed. */
    void setParameters(const SimTK::ExponentialSpringParameters& params);
    /** Get the customizable Topology-stage spring parameters. */
    const SimTK::ExponentialSpringParameters& getParameters() const;

    //-------------------------------------------------------------------------
    // Reporting
    //-------------------------------------------------------------------------
    /** Provide name(s) of the quantities (column labels) of the value(s)
    to be reported. */
    OpenSim::Array<std::string> getRecordLabels() const override;
    /** Provide the value(s) to be reported that correspond to the labels. */
    OpenSim::Array<double> getRecordValues(
            const SimTK::State& state) const override;

protected:
    /** Connect to the OpenSim Model. */
    void extendConnectToModel(Model& model) override;

    /** Create a SimTK::ExponentialContact that implements this Force. */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    /** Update this Object base on an XML node. */
    void updateFromXMLNode(SimTK::Xml::Element& node,
        int versionNumber) override;

private:
    void setNull();
    void constructProperties();
    SimTK::ReferencePtr<const PhysicalFrame> _body;
    SimTK::ExponentialSpringForce* _spr{NULL};

}; // END of class ExponentialContact


//=============================================================================
// ExponentialContact::Parameters
//=============================================================================
/** This subclass manages the topology-stage parameters that are available
for customizing the characteristics of an ExponentialContact object. More
specifically, it has two purposes:

- Initialization. Before model initialization there is no underlying
SimTK::ExponentialSpringForce object upon which to set non-default parameters.
This class serves as a storage mechanism so that, upon model initialization,
parameters can be pushed to the underlying SimTK::ExponentialSpringForce
instance.

- Property Maintenance. This class maintains consistence between the
parameters of the underlying SimTK::ExponentialSpringForce inst ance and their
corresponding OpenSim Properties. The local SimTK::ExponentialSpringParameters
member variable (_stkparams) provides a way to ensure the Properties hold
valid values. Before any properties are updated, new parameters are set
on _stkparams.

@author F. C. Anderson **/
class ExponentialContact::Parameters : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExponentialContact::Parameters, Object);

public:
    OpenSim_DECLARE_PROPERTY(exponential_shape_parameters, SimTK::Vec3,
            "Shape parameters for the exponential that is used to model the \
        normal force: d0 (0.0065905 m), d1 (0.5336 N), d2 (1150.0/m).");
    OpenSim_DECLARE_PROPERTY(normal_viscosity, double,
            "Viscosity in the normal direction (0.5 s/m).");
    OpenSim_DECLARE_PROPERTY(max_normal_force, double,
            "Maximum allowed normal force (100,000.0 N).");
    OpenSim_DECLARE_PROPERTY(friction_elasticity, double,
            "Elasticity of the linear friction spring (20,000.0 N/m).");
    OpenSim_DECLARE_PROPERTY(friction_viscocity, double,
            "Viscosity of the linear friction spring (282.8427 N*s/m).");
    OpenSim_DECLARE_PROPERTY(sliding_time_constant, double,
            "Time constant for rise/decay between static and kinetic friction \
        conditions (0.01 s).");
    OpenSim_DECLARE_PROPERTY(settle_velocity, double,
            "Velocity below which static friction conditions are triggered \
         to come into effect (0.01 m/s) .");
    OpenSim_DECLARE_PROPERTY(settle_acceleration, double,
            "Acceleration below which static friction conditions are triggered \
        to come into effect (1.0 m/s^2).");
    OpenSim_DECLARE_PROPERTY(initial_mu_static, double,
            "Initial value of the static coefficient of friction.");
    OpenSim_DECLARE_PROPERTY(initial_mu_kinetic, double,
            "Initial value of the kinetic coefficient of friction.");

    /** Default constructor. */
    Parameters();

    /** Construct an instance based on a SimTK::ExponentialSpringParameters
    object. */
    Parameters(const SimTK::ExponentialSpringParameters& params);

    /** Set the underlying SimTK::ExponentialSpringParameters. */
    void setSimTKParameters(const SimTK::ExponentialSpringParameters& params);

    /** Get a read-only reference to the underlying
    SimTK::ExponentialSpringParameters. To alter the parameters, use
    setParameters(). */
    const SimTK::ExponentialSpringParameters& getSimTKParameters();

private:
    void setNull();
    void constructProperties();
    void updateParameters();
    void updateProperties();
    void updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber);
    SimTK::ExponentialSpringParameters _stkparams;
};


} // end of namespace OpenSim

#endif // OPENSIM_EXPONENTIAL_SPRING_FORCE_H_
