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
#include "OpenSim/Simulation/Model/ForceSet.h"

namespace OpenSim {

//=============================================================================
// ExponentialContact
//=============================================================================
/** Class ExponentialContact uses an "exponential spring" as a means
of modeling contact of a specified point on a Body with a contact
plane that is fixed to Ground. This specified point is referred to in this
documentation as the "body station". Each ExponentialContact instance
acts at only one body station. In practice, you should choose a number of
body stations strategically located across the surface of a Body,
and construct an ExponentialContact instance for each of those body stations.
For example, if the Body were a cube, you would likely choose the body
stations to be the corners of the cube and construct an ExponentialContact
instance for each corner of the cube (so a total of 8 instances). The contact
plane is typically used to model interactions with a floor, but is not
limited to this use case. The contact plane can be rotated and translated
relative to the Ground frame and so can be used to model a wall, ramp, or
some other planar structure.

Aspects of the exponential contact model are described in the following
publication:

        Anderson F.C. and Pandy M.G. (1999). A dynamics optimization
        solution for vertical jumping in three dimensions. Computer Methods
        in Biomechanics and Biomedical Engineering 2(3):201-231.

Under the covers, the OpenSim::ExponentialContact class encapsulates two
SimTK objects: ExponentialSpringForce and ExponentialSpringParameters.
For the details concerning these classes, see the Simbody API
documentation. A condensed version of that documentation is provided here.

----------------------------------
Computations and Coordinate Frames
----------------------------------
The positive z-axis of the contact plane defines its normal. The positive
z-axis is the axis along which the repelling normal force (modeled using an
exponential) is applied. The x-axis and y-axis of the contact plane together
define the tangent plane. Friction forces will always be tangent to the x-y
plane.

In the equations below, all quantities are expressed in the frame of the
contact plane. A variable with a "z" suffix (e.g., pz, vz, or cz) refers
to a quantity that is normal to the contact plane or that pertains to
calculation of the normal force. A variable with an "xy" suffix
(e.g., pxy, vxy, or cxy) refers to a quantity that lies in or is tangent to
the contact plane or that pertains to calculation of the friction force.

### Normal Force (positive z-axis)

The elastic part of the normal force is computed using an exponential
whose shape is a function of three parameters (d₀, d₁, and d₂):

        fzElastic = d₁exp(−d₂(pz−d₀))

Note that pz is the displacement of the body station above (pz > 0.0)
or below (pz < 0.0) the contact plane. The default values of the shape
parameters were chosen to maximize integration step size while maintaining a
number of constraints (e.g., the normal force should fall below 0.01 Newtons
when pz > 1.0 cm). The damping part of the normal force is linear in velocity
and scaled by the elastic part:

        fzDamping = −cz vz fzElastic,

where vz is the normal component of the velocity of the body station and
cz is the damping coefficient for the normal direction. All together, the
spring force in the normal direction is given by

        fz  = fzElastic + fzDamping
            = d₁exp(d₂(py−d₀)) − cz vz d₁exp(d₂(pz−d₀)))
            = d₁exp(d₂(pz−d₀)) (1 − cz vz)

which has the form of the Hunt & Crossley damping model:

        K. H. Hunt and F. R. E. Crossley (1975). Coefficient of Restitution
        Interpreted as Damping in Vibroimpact. ASME Journal of Applied
        Mechanics, pp. 440-445.

### Friction Force (x-y plane)

The friction force is computed by blending two different friction models.
The blending is performed based on the 'Sliding' State of the
ExponentialSpringForce class. 

#### Friction Model 1 - Pure Damping (Sliding = 1.0)
When the body station is sliding with respect to the contact plane, the
friction force is computed using a simple damping term:

        fricDamp = −cxy vxy

where cxy is the damping coefficient in the contact plane and vxy is the
velocity of the body station in the contact plane. The magnitude of the
total frictional force is not allowed to exceed the frictional limit:

        fricLimit = μ fz
        if (|fricDamp| > fricLimit)
            fricDamp = −fricLimit vxy / |vxy| = −μ fz vxy / |vxy|

where μ is the instantaneous coefficient of friction (more below). Note that
fz is always positive and so fricLimit is a positive scalar. Thus, for
velocities in the contact plane above some threshold velocity, which is
typically small (i.e., less than 0.1 m/s), this model is consistent with a
standard Coulomb Friction model.

#### Friction Model 2 - Damped Linear Spring (Sliding = 0.0)
When the body station is anchored with respect to the contact plane, the
friction force is represented by a damped linear spring. The viscous term is
given by the same damping expression as above:

        fricDampSpr = −cxy vxy

and the elastic term is given by

        fricElasSpr = −kxy (pxy−p₀)

where kxy is the friction spring elasticity, pxy is the position of the body
station projected onto the contact plane, and p₀ is the current spring zero
(i.e., the elastic anchor point of the friction spring). Note that p₀ always
resides in the contact plane.

The total friction spring force is then given by the sum of the elastic and
viscous terms:

        fricSpr = fricElasSpr + fricDampSpr

If the magnitude of the fricSpr exceeds the magnitude of the friction limit,
the terms are scaled down:

        if(|fricSpr| > fricLimit)
            scaleFactor = fricLimit / |fricSpr|
            fricDampSpr = scaleFactor * fricDampSpr
            fricElasSpr = scaleFactor * fricElasSpr
            fricSpr = fricElasSpr + fricDampSpr

Scaling down the friction spring force does not alter its direction.

#### Blending the Friction Models
Blending Model 1 and Model 2 is accomplished using linear expressions of the
Sliding State:

        fricElasBlend = fricElasSpr * (1.0 − Sliding)
        fricDampBlend = fricDampSpr + (fricDamp − fricDampSpr)*Sliding
        fricBlend = fricElasBlend + fricDampBlend

Model 1 (Pure Damping) dominates as Sliding → 1.0, and Model 2
(Damped Linear Spring) dominates as Sliding → 0.0.

#### Moving the Friction Spring Zero
The friction spring zero (p₀) (the elastic anchor point) is always altered
to be consistent with the final value of the blended elastic force:

        p₀ = pxy + fricElasBlend / kxy;
        p₀[2] = 0.0;  // ensures that p₀ lies in the contact plane

#### Coefficients of Friction
Coefficients of kinetic (sliding) and static (fixed) friction can be specified
for the spring, subject to the following constraints:

        0.0 ≤ μₖ ≤ μₛ

Note that there is no upper bound on μₛ. The instantaneous coefficient of
friction (μ) is calculated based on the value of the Sliding State:

        μ = μₛ − Sliding*(μₛ − μₖ)

The value of Sliding is calculated using a continuous function of the
ratio of the speed of the elastic anchor point (ṗ₀) to the settle velocity
(vSettle). vSettle is a customizable topology-stage parameter that represents
the speed at which a body station settles into a static state with respect to
the contact plane. See SimTK::ExponentialSpringParameters::setSettleVelocity()
for details. In particular,

        ṗ₀ = |Δp₀| / Δt
        Sliding = SimTK::stepUp( SimTK::clamp(0.0, ṗ₀/vSettle, 1.0) )

where Δp₀ is the change in p₀ and Δt is the change in time since the last
successful integration step. When ṗ₀ ≥ vSettle, Sliding = 1.0, and as
ṗ₀ → 0.0, Sliding → 0.0.

-----------------------
CUSTOMIZABLE PARAMETERS
-----------------------
Customizable Topology-stage parameters specifying the characteristics of the
exponential spring are managed using SimTK::ExponentialSpringParameters.
To customize any of the Topology-stage parameters on an ExponentialContact
instance, you should

1) Create an ExponentialSpringParameters object. For example,

        ExponentialSpringParameters myParams;

2) Use the available 'set' methods in ExponentialSpringParamters to change
the parameters of that object. For example,

        myParams.setNormalViscosity(0.25);

3) Use ExponentialContact::setParameters() to alter the parameters of one
(or many) ExponentialContact instances. For example,

        ExponentialContact spr1, spr2;
        spr1.setParameters(myParams);
        spr2.setParameters(myParams);

4) Realize the system to Stage::Topology. When a new set of parameters is
set on an ExponentialContact instance, as above in step 3, the System will
be invalidated at Stage::Topology. The System must therefore be realized at
Stage::Topology (and hence Stage::Model) before a simulation can proceed.

        system.realizeTopology();

Note that each ExponentialContact instance owns its own private
ExponentialSpringParameters object. The myParams object is just used to set
the desired parameter values of the privately owned parameters object. It is
fine for objects like myParams to go out of scope or for myParams objects
allocated from the heap to be deleted.

Therefore, also note that the parameter values possessed by an
ExponentialContact instance do not necessarily correspond to the values
held by a local instance of ExponentialSpringParameters until a call to
ExponentialContact::setParameters() is made.

The default values of the parameters are expressed in units of Newtons,
meters, seconds, and kilograms; however, you may use an alternate set of
self-consistent units by re-specifying all parameters.

The default values of the parameters work well for typical contact
interactions, but clearly may not be appropriate for simulating many contact
interactions. For the full descriptions of the contact parameters see the
Simbody API documentation for SimTK::ExponentialSpringParameters.

@author F. C. Anderson **/
class OSIMSIMULATION_API ExponentialContact : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExponentialContact, Force);

public:
    class Parameters;

    //-------------------------------------------------------------------------
    // PROPERTIES
    //-------------------------------------------------------------------------
    OpenSim_DECLARE_PROPERTY(contact_plane_transform, SimTK::Transform,
        "Orientation and location of the contact plane wrt Ground. The positive z-axis of the contact plane defines the normal.");
    OpenSim_DECLARE_PROPERTY(body_name, std::string,
        "Name of the Body to which the resultant contact force is applied.");
    OpenSim_DECLARE_PROPERTY(body_station, SimTK::Vec3,
        "Point on the body, expressed in the Body frame, at which the resultant contact force is applied.");
    OpenSim_DECLARE_PROPERTY(contact_parameters, ExponentialContact::Parameters,
        "Customizable topology-stage parameters.");

    //-------------------------------------------------------------------------
    // Construction
    //-------------------------------------------------------------------------
    /** Default constructor. */
    ExponentialContact();

    /** Construct an ExponentialContact instance.
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

    /** Destructor. */
    ~ExponentialContact() {
        if (_spr != NULL) delete _spr;
    }

    //-------------------------------------------------------------------------
    // Utility
    //-------------------------------------------------------------------------
    /** Reset the elastic anchor point (friction spring zero) so that it
    coincides with the projection of the body station onto the contact
    plane. This step is often needed at the beginning of a simulation to
    ensure that a simulation does not begin with large friction forces.
    After this call, the elastic portion of the friction force should be 0.0
    Calling this method will invalidate the System at Stage::Dynamics.
    @param state State object on which to base the reset. */
    void resetAnchorPoint(SimTK::State& state) const;

    /** Reset the elastic anchor points (friction spring zeros) of all
    ExponentialContact instances in an OpenSim::ForceSet. This step is often
    needed at the beginning of a simulation to ensure that a simulation does
    not begin with large friction forces. Calling this method will invalidate
    the System at Stage::Dynamics.
    @param fSet Force set.
    @param state State object on which to base the reset. */
    static void resetAnchorPoints(OpenSim::ForceSet& fSet, SimTK::State& state);

    //-------------------------------------------------------------------------
    // Accessors for properties
    //-------------------------------------------------------------------------
    /** Set the transform that specifies the location and orientation of the
    contact plane in the Ground frame. */
    void setContactPlaneTransform(const SimTK::Transform& contactPlaneXform);
    /** Get the transform that specifies the location and orientation of the
    contact plane in the Ground frame. */
    const SimTK::Transform& getContactPlaneTransform() const;

    /** Set the name of the body to which this force is applied. */
    void setBodyName(const std::string& bodyName) {
        set_body_name(bodyName);
    }
    /** Get the name of the body to which this force is applied. */
    const std::string& getBodyName() const { return get_body_name(); }

    /** Set the point on the body at which this force is applied. */
    void setBodyStation(const SimTK::Vec3& station) {
        set_body_station(station);
    }
    /** Get the point on the body at which this force is applied. */
    const SimTK::Vec3& getBodyStation() const { return get_body_station(); }

    /** Set the customizable Topology-stage spring parameters. Calling this
    method will invalidate the SimTK::System at Stage::Toplogy. */
    void setParameters(const SimTK::ExponentialSpringParameters& params);
    /** Get the customizable Topology-stage spring parameters. Use the copy
    constructor on the returned reference to create an object that can
    be altered. */
    const SimTK::ExponentialSpringParameters& getParameters() const;

    //-------------------------------------------------------------------------
    // Accessors for states
    //-------------------------------------------------------------------------
    /** Set the static coefficient of friction (μₛ) for this exponential
    spring. The value of μₛ is held in the System's State object. Unlike the
    parameters managed by ExponentialSpringParameters, μₛ can be set at any
    time during a simulation. A change to μₛ will invalidate the System at
    Stage::Dynamics, but not at Stage::Topology.
    @param state State object that will be modified.
    @param mus %Value of the static coefficient of friction. No upper bound.
    0.0 ≤ μₛ. If μₛ < μₖ, μₖ is set equal to μₛ. */
    void setMuStatic(SimTK::State& state, SimTK::Real mus);

    /** Get the static coefficient of friction (μₛ) held by the specified
    state for this exponential spring.
    @param state State object from which to retrieve μₛ. */
    SimTK::Real getMuStatic(const SimTK::State& state) const;

    /** Set the kinetic coefficient of friction (μₖ) for this exponential
    spring. The value of μₖ is held in the System's State object. Unlike the
    parameters managed by ExponentialSpringParameters, μₖ can be set at any
    time during a simulation. A change to μₖ will invalidate the System at
    Stage::Dynamics.
    @param state State object that will be modified.
    @param muk %Value of the kinetic coefficient of friction. No upper bound.
    0.0 ≤ μₖ. If μₖ > μₛ, μₛ is set equal to μₖ. */
    void setMuKinetic(SimTK::State& state, SimTK::Real muk);

    /** Get the kinetic coefficient of friction (μₖ) held by the specified
    state for this exponential spring.
    @param state State object from which to retrieve μₖ. */
    SimTK::Real getMuKinetic(const SimTK::State& state) const;

    /** Get the Sliding state of the spring.
    @param state State object from which to retrieve Sliding. */
    SimTK::Real getSliding(const SimTK::State& state) const;

    //-------------------------------------------------------------------------
    // Accessors for data cache entries
    //-------------------------------------------------------------------------
    /** Get the elastic part of the normal force. The system must be realized
    to Stage::Dynamics to access this data.
    @param state State object on which to base the calculations.
    @param inGround Flag for choosing the frame in which the returned quantity
    will be expressed. If true (the default), the quantity will be expressed
    in the Ground frame. If false, the quantity will be expressed in the frame
    of the contact plane. */
    SimTK::Vec3 getNormalForceElasticPart(
        const SimTK::State& state, bool inGround = true) const;

    /** Get the damping part of the normal force. The system must be realized
    to Stage::Dynamics to access this data.
    @param state State object on which to base the calculations.
    @param inGround Flag for choosing the frame in which the returned quantity
    will be expressed. If true (the default), the quantity will be expressed
    in the Ground frame. If false, the quantity will be expressed in the frame
    of the contact plane. */
    SimTK::Vec3 getNormalForceDampingPart(
        const SimTK::State& state, bool inGround = true) const;

    /** Get the normal force. The system must be realized to Stage::Dynamics
    to access this data.
    @param state State object on which to base the calculations.
    @param inGround Flag for choosing the frame in which the returned quantity
    will be expressed. If true (the default), the quantity will be expressed
    in the Ground frame. If false, the quantity will be expressed in the frame
    of the contact plane. */
    SimTK::Vec3 getNormalForce(
        const SimTK::State& state, bool inGround = true) const;

    /** Get the instantaneous coefficient of friction (μ). The system must be
    realized to Stage::Dynamics to access this data. μ is obtained by using
    the Sliding state to transition between μₖ and μₛ:

            μ = μₛ - Sliding*(μₛ - μₖ)

    Because 0.0 ≤ Sliding ≤ 1.0, μₖ ≤ μ ≤ μₛ.
    @param state State object on which to base the calculations. */
    SimTK::Real getMu(const SimTK::State& state) const;

    /** Get the friction limit. The system must be realized to Stage::Dynamics
    to access this data.
    @param state State object on which to base the calculations. */
    SimTK::Real getFrictionForceLimit(const SimTK::State& state) const;

    /** Get the elastic part of the friction force. The system must be
    realized to Stage::Dynamics to access this data.
    @param state State object on which to base the calculations.
    @param inGround Flag for choosing the frame in which the returned quantity
    will be expressed. If true (the default), the quantity will be expressed
    in the Ground frame. If false, the quantity will be expressed in the frame
    of the contact plane. */
    SimTK::Vec3 getFrictionForceElasticPart(
        const SimTK::State& state, bool inGround = true) const;

    /** Get the damping part of the friction force. The system must be
    realized to Stage::Dynamics to access this data.
    @param state State object on which to base the calculations.
    @param inGround Flag for choosing the frame in which the returned quantity
    will be expressed. If true (the default), the quantity will be expressed
    in the Ground frame. If false, the quantity will be expressed in the frame
    of the contact plane. */
    SimTK::Vec3 getFrictionForceDampingPart(
        const SimTK::State& state, bool inGround = true) const;

    /** Get the total friction force. The total frictional force is always
    just the sum of the elastic part and the damping part of the frictional
    force, which may be obtained separately by calling
    getFrictionalForceElasticPart() and getFrictionalForceDampingPart().
    The system must be realized to Stage::Dynamics to access this data.
    @param state State object on which to base the calculations.
    @param inGround Flag for choosing the frame in which the returned quantity
    will be expressed. If true (the default), the quantity will be expressed
    in the Ground frame. If false, the quantity will be expressed in the frame
    of the contact plane. */
    SimTK::Vec3 getFrictionForce(
        const SimTK::State& state, bool inGround = true) const;

    /** Get the total force applied to the body by this
    ExponentialSpringForce instance. The total force is the vector sum of the
    friction force, which may be obtained by a call to getFrictionForce(), and
    the normal force, which may be obtained by a call to getNormalForce().
    The system must be realized to Stage::Dynamics to access this data.
    @param state State object on which to base the calculations.
    @param inGround Flag for choosing the frame in which the returned quantity
    will be expressed. If true (the default), the quantity will be expressed
    in the Ground frame. If false, the quantity will be expressed in the frame
    of the contact plane. */
    SimTK::Vec3 getForce(
        const SimTK::State& state, bool inGround = true) const;

    /** Get the position of the body station (i.e., the point on the body at
    which the force generated by this ExponentialSpringForce is applied). This
    method differs from getStation() in terms of the frame in which the
    station is expressed. getStation() expresses the point in the frame of
    the MobilizedBody. getStationPosition() expresses the point either in the
    Ground frame or in the frame of the Contact Plane. The system must be
    realized to Stage::Position to access this data.
    @param state State object on which to base the calculations.
    @param inGround Flag for choosing the frame in which the returned quantity
    will be expressed. If true (the default), the quantity will be expressed
    in the Ground frame. If false, the quantity will be expressed in the frame
    of the contact plane. */
    SimTK::Vec3 getStationPosition(
        const SimTK::State& state, bool inGround = true) const;

    /** Get the velocity of the body station (i.e., the point on the body at
    which the force generated by this ExponentialSpringForce is applied).
    The system must be realized to Stage::Velocity to access this data.
    @param state State object on which to base the calculations.
    @param inGround Flag for choosing the frame in which the returned quantity
    will be expressed. If true (the default), the quantity will be expressed
    in the Ground frame. If false, the quantity will be expressed in the frame
    of the contact plane. */
    SimTK::Vec3 getStationVelocity(
        const SimTK::State& state, bool inGround = true) const;

    /** Get the position of the elastic anchor point (p₀), which will always
    lie in the Contact Plane. p₀ is the spring zero of the damped linear
    spring used in Friction Model 2. The system must be realized to
    Stage::Dynamics to access this data.
    @param state State object on which to base the calculations.
    @param inGround Flag for choosing the frame in which the returned quantity
    will be expressed. If true (the default), the quantity will be expressed
    in the Ground frame. If false, the quantity will be expressed in the frame
    of the contact plane. */
    SimTK::Vec3 getAnchorPointPosition(
        const SimTK::State& state, bool inGround = true) const;

    //-------------------------------------------------------------------------
    // Reporting
    //-------------------------------------------------------------------------
    /** Provide name(s) of the quantities (column labels) of the value(s)
    to be reported. */
    OpenSim::Array<std::string> getRecordLabels() const override;
    /** Provide the value(s) to be reported that correspond to the labels. */
    OpenSim::Array<double> getRecordValues(
            const SimTK::State& state) const override;

    //-------------------------------------------------------------------------
    // Internal Testing
    //-------------------------------------------------------------------------
    /** Assess consistency between Properties and internal parameters. */
    void assertPropertiesAndParametersEqual() const;

protected:
    /** Connect to the OpenSim Model. */
    void extendConnectToModel(Model& model) override;

    /** Create a SimTK::ExponentialSpringForce objects that implements
    this Force. */
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
/** This subclass helps manage most of the topology-stage parameters of an
OpenSim::ExponentialContact object. It does not provide the interface for
getting and setting contact parameters (directly anyway) but rather provides
the infrastructure for making the underlying SimTK::ExponentialSpringForce and
SimTK::ExponentialSpringParameters classes available in OpenSim.

More specifically, this class chiefly does 3 things:
- Implements OpenSim Properties for most of the customizable contact
parameters of class OpenSim::ExponentialContact, enabling those parameters
to be serialized and de-serialized from file.
- Provides a member variable (_stkparams) for storing user-set parameters
prior to the creation of an underlying SimTK::ExponentialSpringForce object.
During model initialization, when the SimTK::ExponetialSpringForce object is
constructed, the user-set parameters are then pushed to that object.
- Ensures that the values held by the OpenSim properties are kept consistent
with the values held by a SimTK::ExponentialSpringParameters object.
Depending on the circumstance, parameters are updated to match properties or
properties are update to match parameters.

To get or set values of the parameters managed by this class, you should use
ExponentialContact::getParameters() and ExponentialContact::setParameters().
Note that the values of the parameters managed by this class should always be
the same as the values of their corresponding properties.

@author F. C. Anderson **/
class ExponentialContact::Parameters : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExponentialContact::Parameters, Object);

public:
    OpenSim_DECLARE_PROPERTY(exponential_shape_parameters, SimTK::Vec3,
        "Shape parameters for the exponential that models the normal force: d0 (0.0065905 m), d1 (0.5336 N), d2 (1150.0/m).");
    OpenSim_DECLARE_PROPERTY(normal_viscosity, double,
        "Viscosity in the normal direction (0.5 s/m).");
    OpenSim_DECLARE_PROPERTY(max_normal_force, double,
        "Maximum allowed normal force (100,000.0 N).");
    OpenSim_DECLARE_PROPERTY(friction_elasticity, double,
        "Elasticity of the friction spring (20,000.0 N/m).");
    OpenSim_DECLARE_PROPERTY(friction_viscosity, double,
        "Viscosity of the friction spring (282.8427 N*s/m).");
    OpenSim_DECLARE_PROPERTY(sliding_time_constant, double,
        "Time constant for rise/decay between static and kinetic friction conditions (0.01 s).");
    OpenSim_DECLARE_PROPERTY(settle_velocity, double,
        "Velocity below which static friction conditions are triggered (0.01 m/s) .");
    OpenSim_DECLARE_PROPERTY(settle_acceleration, double,
        "Acceleration below which static friction conditions are triggered (1.0 m/s²).");
    OpenSim_DECLARE_PROPERTY(initial_mu_static, double,
        "Initial value of the static coefficient of friction.");
    OpenSim_DECLARE_PROPERTY(initial_mu_kinetic, double,
        "Initial value of the kinetic coefficient of friction.");

    /** Default constructor. */
    Parameters();

    /** Construct an instance based on a SimTK::ExponentialSpringParameters
    object. */
    Parameters(const SimTK::ExponentialSpringParameters& params);

    /** Set the underlying SimTK parameters. This method is used to maintain
    consistency between OpenSim Properties and the underlying parameters.
    The typical user of OpenSim::ExponentialContact will not have reason to
    call this method. For setting contact parameters, the typical user should
    call OpenSim::ExponentialContact::setParameters(). */
    void setSimTKParameters(const SimTK::ExponentialSpringParameters& params);

    /** Get a read-only reference to the underlying SimTK parameters. This
    method is used to maintain consistency between OpenSim Properties and the
    underlying parameters. The typical user of OpenSim::ExponentialContact will
    not have reason to call this method. For getting contact parameters, the
    typical user should call OpenSim::ExponentialContact::getParameters() */
    const SimTK::ExponentialSpringParameters& getSimTKParameters() const;

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
