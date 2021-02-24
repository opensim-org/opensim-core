#ifndef OPENSIM_SMITH_2018_ARTICULAR_CONTACT_FORCE_H_
#define OPENSIM_SMITH_2018_ARTICULAR_CONTACT_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                     Smith2018ArticularContactForce.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Colin Smith                                                     *
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


#include "OpenSim/Simulation/Model/Force.h"
#include "Smith2018ContactMesh.h"


namespace OpenSim {
/**
This Force component models the contact between a pair of triangulated surface
meshes (.vtp, .stl, .obj). It was orginially designed to represent articular 
contact between cartilage, mensici, or joint implants [1], but is 
generalizable so it could also represent foot-floor contact or a skin-device 
interfaces etc. The formulation of the contact model has previously been 
called an elastic foundation model [2] or discrete element analysis [3,4]. 
In this implementation, non-deforming triangulated meshes are allowed to 
interpenetrate and the proximity (distance between the meshes) is calculated 
for each triangle. For every triangle with a positive proximity (ie triangle 
interpenetrated the opposing mesh), the contact pressure on the triangle face 
is then calculated based on the proximity and material properties. This 
formulation is much faster than a finite element approach, but only calculates 
the contact pressures on the mesh surface and not the internal stresses. 
Additionally, it provides a simplifed representation of the contact as the 
meshes do not deform, but instead are geometrically rigid (ie the vertices 
within a mesh do not move relative to each other), but the contacting mesh 
pair are allowed to interpenetrate.

The while the force calculatation is similar to the ElasticFoundationForce
component, the Smith2018ArticularContactForce provides a different 
parameterization of the relationship between the overlap depth and material 
properties to calculate the contact pressure. Additionally, it provides the 
proximity, pressure, and potential_energy for each mesh triangle as an output 
so that maps of these calculated values on the mesh surface can be visualized. 
Finally, it provides improved computational performance through a different 
collision detection method.

To calculate the pressure for each triangle, it is necessary to detect the 
mesh triangles that are interpenetrating (commonly called contact or collision
detection in computer graphics literature) and calculate the proximity. This 
task is extremely slow if a brute force approach is applied to check every 
triangle in one mesh against every triangle in another mesh. Smith et al, 
CMBBE I&V, 2018 [1] introduced a method to efficiently detect contact between 
triangular meshes using Oriented Bounding Boxes (OBBs, a common approach in 
computer graphics) and several additional speed-ups that leverage the fact 
that changes in contact between timesteps are generally small. This approach 
has been implemented in the Smith2018ArticularContactForce component along 
with some additional features.

Two articulating triangular meshes are defined as Smith2018ContactMesh
components (Sockets: casting_mesh and target_mesh). These meshes are fixed to
bodies in the model, and thus their relative poses are determined by the
model coordinates. To detect contact, a normal ray is cast from the center of
each triangle in the casting mesh towards the overlapping target
mesh. Ray intersection tests are then performed against an Oriented Bounding
Box tree constructed around the target mesh. This algorithm is implemented in
the computeMeshProximity() function with the OBB construction and ray
intersection queries managed by the Smith2018ContactMesh.

\image html fig_Smith2018ArticularContactForce_contact_detection.png width=600px

The major speed-up in the algorithm leverages the fact that changes in joint
coordinates and thus contact patterns between time steps are generally small. 
Thus, after reposing the meshes, (i.e. realizePosition()) each
triangle in the casting mesh is tested against the contacting target triangle
from the previous pose. Additional speed-up can be gained by casting the normal
ray in both directions (by setting min_proximity to a negative value), so 
even some of the out-of-contact triangles are "remembered". If the previous 
contacting triangle test fails, the casting ray is checked against the 
neighboring triangles (those that share a vertex) in the target mesh. Then if 
this test fails, the expensive casting ray--OBB test is performed. If the 
meshes were not in contact at the previous time step this does not cause an 
issue, just a slower solution, as here the ray-OBB tests will be peformed for 
every triangle in the casting_mesh.  


# Swapping the contact meshes changes the resulting forces

The ray casting is only performed from the casting_mesh towards the 
target_mesh. Thus, a pressure map is only generated for the casting_mesh. 
A force vector is computed for each triangle in the casting_mesh using
Force = -normal*area*pressure, and the resultant force of
all triangle forces is applied to the body to which the casting_mesh is
attached. An equal and opposite force is applied to the body to which the
target_mesh is attached. The important ramification of this is that if the
casting_mesh and target_mesh are switched, the simulation results will NOT be
exactly the same. For best performance, the casting_mesh should be set to the
mesh that contains the smaller number of triangles.

As it is still useful to visualize the proximity and
pressure maps for the target_mesh, and in most situations the resultant force
from the target_mesh are very close to the mirrored casting_mesh resultant
force, there is a ModelingOption named "flip_meshes" that will cause the ray
casting to also be performed from the target_mesh to calculate triangle
proximity and pressure values. Note the applied contact force in this case is
still only that calculated for the casting_mesh, and the time needed for 
collision detection is approximately doubled. 

# Potential Pitfalls
\image html fig_Smith2018ArticularContactForce_pitfalls.png width=600px

Case 1
The casting_mesh mesh has significant curvature and the material properties 
are set in a manner that results in a compliment contact where large 
interpenetrations with curvature may occur. In this scenario,
the distance along the normal ray from the casting_mesh to the target_mesh
may be unrealistically large, resulting in high contact pressures. If only 
one mesh has high curvature, then this mesh can be defined as the target_mesh
and the excessive proximity values are avoided. Another potential solution is
to ensure that the initial positions of the meshes are set so that there is 
minimal or no contact, thus as the simulation progresses the increasing 
contact pressures will prevent significant interpentration of the meshes. 

Case 2
Another pitfall of highly curved meshes is the potential for false contact 
detection as shown in the figure. Here, a triangle on the side of the patellar
cartilage will be incorrectly identified as in contact with the femur. This 
problem can be avoided by setting min_proximity and max_proximity properties
to values that limit the potential contact search area to feasible locations 
for your application. The min_proximity and max_proximity properties limit the 
search region for a contact triangle along the ray cast from the casting_mesh.
The min_proximity can be set to a negative value if you would like proximity 
maps for the out of contact triangles. For example, if you are visualizing 
kinematics measured with fluoroscopy and only have meshes of the bones, using 
a negative min_proximity enables the distance between the subcondral surfaces 
to be calculated for each triangle in the casting_mesh.

Case 3
To reduce the number of triangles in the Smith2018ContactMesh and thus speed
up the collision detection, the meshes do not need to be closed surfaces
(water tight). However, this can cause issues if the initial positions of the 
meshes are set improperly or the positions of the meshes within a simulation 
progress to infeasible configurations. The figure depicts that the patella 
has been spun 180*, so if soft tissues pull the patella straight into the 
femur, no contact will be detected because the backside of the patella will 
collide with the femur. However, while the meshes do not need to be closed,
they can be closed to avoid this issue for applications such as simulating the
contact between a spinning ball bouncing on a surface.


# The pressure-depth relationship

The relationship between overlap depth and pressure can be either linear or
non-linear, depending on the value of the elastic_foundation_formulation
property. The implemented equations are those proposed in Bei and Fregly, Med
Eng Phys, 2004 [2]:

Linear:
\f[
   P = E\frac{(1-\nu)}{(1 + \nu)(1-2\nu)}\frac{d}{h}
\f]

Non-Linear:
\f[
  P = -E\frac{(1-\nu)}{(1 + \nu)(1-2\nu)}\ln{\left(1-\frac{d}{h}\right)}
\f]

Where:
 - \f$ P \f$: pressure
 - \f$ E \f$: elastic modulus
 - \f$ \nu \f$: Poisson's ratio
 - \f$ d \f$: depth of overlap
 - \f$ h \f$: height (i.e., thickness) of the elastic layer

# Material/mesh properties

The original Bei and Fregly formulation assumes that a rigid object is
contacting an object with a thin elastic layer. This assumption is appropriate
when modeling joint replacements where a metal component (rigid) contacts a
polyethylene component (elastic). To model cartilage-cartilage contact, this
approach requires that the two cartilage layers are lumped together into one
elastic layer, necessitating that a constant thickness, elastic modulus, and
Poisson's ratio is assumed for each contacting triangle pair. Thus the total
overlap depth is split equally between each triangle contact pair. As
cartilage-cartilage contact often involves articulations between cartilage
surfaces with varying thickness and material properties, the Bei and Fregly
approach was extended to accommodate variable properties. The
use_lumped_contact_model property controls whether the constant property or
variable property formulation is used. 

\image html fig_Smith2018ArticularContactForce_lumped_model.png width=600px

The variable property formulation is described in Zevenbergen et al,
PLOS One, 2018 [5]. Here, the following system of four equations must be
solved to obtain the local overlap depth (proximity) and pressure for the
casting and target triangles.

\f[
 \begin{align}
  P_\mathrm{casting} &= f(E,\nu,h,d_\mathrm{casting}) \\
  P_\mathrm{target} &= f(E,\nu,h,d_\mathrm{target}) \\
  P_\mathrm{casting} &= P_\mathrm{target} \\
  d &= d_\mathrm{casting} + d_\mathrm{target}
  \end{align}
\f]

Here, the first two equations use the Bei and Fregly elastic foundation model
(linear or non-linear) to define the relationship between the local mesh
properties, local overlap depth and computed pressure. The third equation is a
force equilibrium, assuming that the force applied to a pair of contacting
triangles is equal and opposite. This formulation further assumes that the
triangles in contact have the same area. The fourth equation states that the
total overlap depth of the meshes (which is readily calculated) is the sum of
the local overlap depths of the two elastic layers in contact.

This system of equations can be solved analytically if the linear pressure-
depth relationship is used. If the non-linear relationship is used, the
system of equations is solved using a numerical solver.

# Outputs

This component has some outputs such as triangle_proximity, triangle_pressure,
and triangle_potential_energy that return a SimTK::Vector (size = number of 
triangles) with a value corresponding to each triangle face in the respective 
target or casting mesh.
There are also "summary" outputs that return values associated with the entire 
mesh such as contact area, mean/max proximity/pressure, center of
proximity/pressure etc. Finally, there are regional summary outputs which
return a SimTK::Vector (size = 6). Here, the entries in the vector reflect the
summary metrics corresponding to subset of mesh triangles located in six 
specific regions. These six regions are defined as the subset of mesh triangles 
whose center is located in the half space [+x, -x, +y, -y, +z, -z] in the 
local mesh coordinate system. If the mesh coordinate system is aligned with 
anatomical axes, then this enables simulation results to be more readily 
interpreted. For example, when performing simulations of the knee, if the 
z axis is aligned to the medial-lateral axis, points medially, and the origin 
is located between the femoral condyles, then the regional outputs 
corresponding to +z and -z will summarize the mesh triangles located on the 
medial and lateral condyles and thus enable comparisons of the loading in the 
medial and lateral compartments.

All outputs are reported in the local mesh reference frame (ie the reference
frame of the mesh_file. The ContactForce and ContactMoment outputs are 
expressed in this frame and calculated at the origin of this frame.). 

# References

   [1] Smith, C. R., Won Choi, K., Negrut, D., & Thelen, D. G. (2018).
       Efficient computation of cartilage contact pressures within dynamic
       simulations of movement. Computer Methods in Biomechanics and
       Biomedical Engineering: Imaging & Visualization, 6(5), 491-498.

   [2] Bei, Y., & Fregly, B. J. (2004). Multibody dynamic simulation of knee
       contact mechanics. Medical engineering & physics, 26(9), 777-789.

   [3] Volokh, K. Y., Chao, E. Y. S., & Armand, M. (2007). On foundations of
       discrete element analysis of contact in diarthrodial joints. Molecular
       & cellular biomechanics: MCB, 4(2), 67.

   [4] Abraham, C. L., Maas, S. A., Weiss, J. A., Ellis, B. J., Peters, C.
       L., & Anderson, A. E. (2013). A new discrete element analysis method
       for predicting hip joint contact stresses. Journal of biomechanics,
       46(6), 1121-1127.

   [5] Zevenbergen, L., Smith, C. R., Van Rossom, S., Thelen, D. G., Famaey,
       N., Vander Sloten, J., & Jonkers, I. (2018). Cartilage defect location
       and stiffness predispose the tibiofemoral joint to aberrant loading
       conditions during stance phase of gait. PloS one, 13(10), e0205842.
*/
class OSIMSIMULATION_API Smith2018ArticularContactForce : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(Smith2018ArticularContactForce, Force)

    struct ContactStats;

public:
    //=========================================================================
    // PROPERTIES
    //=========================================================================
    OpenSim_DECLARE_PROPERTY(min_proximity, double, "The minimum proximity "
        "that is valid between contacting meshes to limit the search distance "
        "along the casting_mesh normal ray used for collision detection. Note "
        "this can be negative if proximity maps should include triangles that "
        "are not in contact."
        "Default value set to 0.0 meters.")

    OpenSim_DECLARE_PROPERTY(max_proximity, double, "The maximum proximity "
        "that is valid between contacting meshes to limit the search distance "
        "along the casting_mesh normal ray used for collision detection."
        "Default value set to 0.01 meters.")

    OpenSim_DECLARE_PROPERTY(elastic_foundation_formulation, std::string,
        "Formulation for depth-pressure relationship: "
        "'linear' or 'nonlinear'. "
        "Default value set to 'linear'.")

    OpenSim_DECLARE_PROPERTY(use_lumped_contact_model, bool,
        "Combine the thickness and the average material properties between "
        "the Smith2018ContactMeshes for both meshes and use Bei & Fregly 2003 "
        "lumped parameter Elastic Foundation model.")

    //=========================================================================
    // Connectors
    //=========================================================================
    OpenSim_DECLARE_SOCKET(target_mesh, Smith2018ContactMesh,
        "Target mesh for collision detection.")

    OpenSim_DECLARE_SOCKET(casting_mesh, Smith2018ContactMesh,
        "Ray casting mesh for collision detection.")


    //=========================================================================
    // OUTPUTS
    //=========================================================================
    // number of colliding triangles
    OpenSim_DECLARE_OUTPUT(target_num_contacting_triangles, int,
        getTargetNumContactingTriangles, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_num_contacting_triangles, int,
        getCastingNumContactingTriangles, SimTK::Stage::Dynamics)

    // tri proximity
    OpenSim_DECLARE_OUTPUT(target_triangle_proximity, SimTK::Vector,
        getTargetTriangleProximity, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_triangle_proximity, SimTK::Vector,
        getCastingTriangleProximity, SimTK::Stage::Position)

    // tri pressure
    OpenSim_DECLARE_OUTPUT(target_triangle_pressure, SimTK::Vector,
        getTargetTrianglePressure, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_triangle_pressure, SimTK::Vector,
        getCastingTrianglePressure, SimTK::Stage::Dynamics)

    // tri potential energy
    OpenSim_DECLARE_OUTPUT(target_triangle_potential_energy, SimTK::Vector,
        getTargetTrianglePotentialEnergy, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_triangle_potential_energy, SimTK::Vector,
        getCastingTrianglePotentialEnergy, SimTK::Stage::Dynamics)

    // contact area
    OpenSim_DECLARE_OUTPUT(target_total_contact_area, double,
        getTargetTotalContactArea, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_total_contact_area, double,
        getCastingTotalContactArea, SimTK::Stage::Position)

    OpenSim_DECLARE_OUTPUT(target_regional_contact_area, SimTK::Vector,
        getTargetRegionalContactArea, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_regional_contact_area, SimTK::Vector,
        getCastingRegionalContactArea, SimTK::Stage::Position)

    // mean proximity
    OpenSim_DECLARE_OUTPUT(target_total_mean_proximity, double,
        getTargetTotalMeanProximity, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_total_mean_proximity, double,
        getCastingTotalMeanProximity, SimTK::Stage::Position)

    OpenSim_DECLARE_OUTPUT(target_regional_mean_proximity, SimTK::Vector,
        getTargetRegionalMeanProximity, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_regional_mean_proximity, SimTK::Vector,
        getCastingRegionalMeanProximity, SimTK::Stage::Position)

    // max proximity
    OpenSim_DECLARE_OUTPUT(target_total_max_proximity, double,
        getTargetTotalMaxProximity, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_total_max_proximity, double,
        getCastingTotalMaxProximity, SimTK::Stage::Position)

    OpenSim_DECLARE_OUTPUT(target_regional_max_proximity, SimTK::Vector,
        getTargetRegionalMaxProximity, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_regional_max_proximity, SimTK::Vector,
        getCastingRegionalMaxProximity, SimTK::Stage::Position)

    // mean pressure
    OpenSim_DECLARE_OUTPUT(target_total_mean_pressure, double,
        getTargetTotalMeanPressure, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_total_mean_pressure, double,
        getCastingTotalMeanPressure, SimTK::Stage::Dynamics)

    OpenSim_DECLARE_OUTPUT(target_regional_mean_pressure, SimTK::Vector,
        getTargetRegionalMeanPressure, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_regional_mean_pressure, SimTK::Vector,
        getCastingRegionalMeanPressure, SimTK::Stage::Dynamics)

    // max pressure
    OpenSim_DECLARE_OUTPUT(target_total_max_pressure, double,
        getTargetTotalMaxPressure, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_total_max_pressure, double,
        getCastingTotalMaxPressure, SimTK::Stage::Dynamics)

    OpenSim_DECLARE_OUTPUT(target_regional_max_pressure, SimTK::Vector,
        getTargetRegionalMaxPressure, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_regional_max_pressure, SimTK::Vector,
        getCastingRegionalMaxPressure, SimTK::Stage::Dynamics)

    // center of proximity
    OpenSim_DECLARE_OUTPUT(target_total_center_of_proximity, SimTK::Vec3,
        getTargetTotalCenterOfProximity, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_total_center_of_proximity, SimTK::Vec3,
        getCastingTotalCenterOfProximity, SimTK::Stage::Position)

    OpenSim_DECLARE_OUTPUT(target_regional_center_of_proximity,
        SimTK::Vector_<SimTK::Vec3>, getTargetRegionalCenterOfProximity,
        SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_regional_center_of_proximity,
        SimTK::Vector_<SimTK::Vec3>, getCastingRegionalCenterOfProximity,
        SimTK::Stage::Position)

    // center of pressure
    OpenSim_DECLARE_OUTPUT(target_total_center_of_pressure, SimTK::Vec3,
        getTargetTotalCenterOfPressure, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_total_center_of_pressure, SimTK::Vec3,
        getCastingTotalCenterOfPressure, SimTK::Stage::Dynamics)

    OpenSim_DECLARE_OUTPUT(target_regional_center_of_pressure,
        SimTK::Vector_<SimTK::Vec3>, getTargetRegionalCenterOfPressure,
        SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_regional_center_of_pressure,
        SimTK::Vector_<SimTK::Vec3>, getCastingRegionalCenterOfPressure,
        SimTK::Stage::Dynamics)

    // contact force
    OpenSim_DECLARE_OUTPUT(target_total_contact_force, SimTK::Vec3,
        getTargetTotalContactForce, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_total_contact_force, SimTK::Vec3,
        getCastingTotalContactForce, SimTK::Stage::Dynamics)

    OpenSim_DECLARE_OUTPUT(target_regional_contact_force,
        SimTK::Vector_<SimTK::Vec3>, getTargetRegionalContactForce,
        SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_regional_contact_force,
        SimTK::Vector_<SimTK::Vec3>, getCastingRegionalContactForce,
        SimTK::Stage::Dynamics)

    // contact moment
    OpenSim_DECLARE_OUTPUT(target_total_contact_moment, SimTK::Vec3,
        getTargetTotalContactMoment, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_total_contact_moment, SimTK::Vec3,
        getCastingTotalContactMoment, SimTK::Stage::Dynamics)

    OpenSim_DECLARE_OUTPUT(target_regional_contact_moment,
        SimTK::Vector_<SimTK::Vec3>, getTargetRegionalContactMoment,
        SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_regional_contact_moment,
        SimTK::Vector_<SimTK::Vec3>, getCastingRegionalContactMoment,
        SimTK::Stage::Dynamics)

    //=========================================================================
    // METHODS
    //=========================================================================

    Smith2018ArticularContactForce();

    Smith2018ArticularContactForce(const std::string& name,
        Smith2018ContactMesh& target_mesh, Smith2018ContactMesh& casting_mesh);

public:

    //-------------------------------------------------------------------------
    // Output Methods
    //-------------------------------------------------------------------------

    //number of contacting triangles
    int getTargetNumContactingTriangles(const SimTK::State& state) const {
        return getCacheVariableValue<int>
            (state, "target.num_contacting_triangles");
    }

    int getCastingNumContactingTriangles(const SimTK::State& state) const {
        return getCacheVariableValue<int>
            (state, "casting.num_contacting_triangles");
    }

    //tri proximity
    SimTK::Vector getTargetTriangleProximity(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "target.triangle.proximity");
    }
    SimTK::Vector getCastingTriangleProximity(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.triangle.proximity");
    }

    //tri pressure
    SimTK::Vector getTargetTrianglePressure(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "target.triangle.pressure");
    }
    SimTK::Vector getCastingTrianglePressure(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.triangle.pressure");
    }

    //tri potential energy
    SimTK::Vector getTargetTrianglePotentialEnergy(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "target.triangle.potential_energy");
    }
    SimTK::Vector getCastingTrianglePotentialEnergy(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.triangle.potential_energy");
    }

    //Lazy computed outputs

    //contact_area
    double getTargetTotalContactArea(const SimTK::State& state) const {
        if (!isCacheVariableValid(state, "target.total.contact_area")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<double>
            (state, "target.total.contact_area");
    }

    double getCastingTotalContactArea(const SimTK::State& state) const {
        if (!isCacheVariableValid(state, "casting.total.contact_area")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<double>
            (state, "casting.total.contact_area");
    }

    SimTK::Vector getTargetRegionalContactArea(
        const SimTK::State& state) const {
        
        if (!isCacheVariableValid(state, "target.regional.contact_area")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector>
            (state, "target.regional.contact_area");
    }

    SimTK::Vector getCastingRegionalContactArea(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.regional.contact_area")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.regional.contact_area");
    }

    //mean proximity
    double getTargetTotalMeanProximity(const SimTK::State& state) const {
        
        if (!isCacheVariableValid(state, "target.total.mean_proximity")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<double>
            (state, "target.total.mean_proximity");
    }

    double getCastingTotalMeanProximity(const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.total.mean_proximity")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<double>
            (state, "casting.total.mean_proximity");
    }

    SimTK::Vector getTargetRegionalMeanProximity(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "target.regional.mean_proximity")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector>
            (state, "target.regional.mean_proximity");
    }

    SimTK::Vector getCastingRegionalMeanProximity(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.regional.mean_proximity")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.regional.mean_proximity");
    }

    //max proximity
    double getTargetTotalMaxProximity(const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "target.total.max_proximity")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<double>
            (state, "target.total.max_proximity");
    }

    double getCastingTotalMaxProximity(const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.total.max_proximity")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<double>
            (state, "casting.total.max_proximity");
    }

    SimTK::Vector getTargetRegionalMaxProximity(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "target.regional.max_proximity")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector>
            (state, "target.regional.max_proximity");
    }

    SimTK::Vector getCastingRegionalMaxProximity(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.regional.max_proximity")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.regional.max_proximity");
    }

    //mean pressure
    double getTargetTotalMeanPressure(const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "target.total.mean_pressure")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<double>
            (state, "target.total.mean_pressure");
    }

    double getCastingTotalMeanPressure(const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.total.mean_pressure")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<double>
            (state, "casting.total.mean_pressure");
    }

    SimTK::Vector getTargetRegionalMeanPressure(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.total.mean_pressure")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector>
            (state, "target.regional.mean_pressure");
    }

    SimTK::Vector getCastingRegionalMeanPressure(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.regional.mean_pressure")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.regional.mean_pressure");
    }

    //max pressure
    double getTargetTotalMaxPressure(const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "target.total.max_pressure")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<double>
            (state, "target.total.max_pressure");
    }

    double getCastingTotalMaxPressure(const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.total.max_pressure")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<double>
            (state, "casting.total.max_pressure");
    }

    SimTK::Vector getTargetRegionalMaxPressure(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "target.regional.max_pressure")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector>
            (state, "target.regional.max_pressure");
    }

    SimTK::Vector getCastingRegionalMaxPressure(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.regional.max_pressure")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.regional.max_pressure");
    }

    //center of proximity
    SimTK::Vec3 getTargetTotalCenterOfProximity(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "target.total.center_of_proximity")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vec3>
            (state, "target.total.center_of_proximity");
    }

    SimTK::Vec3 getCastingTotalCenterOfProximity(
        const SimTK::State& state) const {
        
        if (!isCacheVariableValid(state, "target.total.center_of_proximity")) {
            realizeContactMetricCaches(state);
        }
        
        return getCacheVariableValue<SimTK::Vec3>
            (state, "casting.total.center_of_proximity");
    }

    SimTK::Vector_<SimTK::Vec3> getTargetRegionalCenterOfProximity(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, 
            "target.regional.center_of_proximity")) {

            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "target.regional.center_of_proximity");
    }

    SimTK::Vector_<SimTK::Vec3> getCastingRegionalCenterOfProximity(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, 
            "casting.regional.center_of_proximity")) {

            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "casting.regional.center_of_proximity");
    }

    //center of pressure
    SimTK::Vec3 getTargetTotalCenterOfPressure(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "target.total.center_of_pressure")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vec3>
            (state, "target.total.center_of_pressure");
    }

    SimTK::Vec3 getCastingTotalCenterOfPressure(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.total.center_of_pressure")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vec3>
            (state, "casting.total.center_of_pressure");
    }

    SimTK::Vector_<SimTK::Vec3> getTargetRegionalCenterOfPressure(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, 
            "target.regional.center_of_pressure")) {

            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "target.regional.center_of_pressure");
    }

    SimTK::Vector_<SimTK::Vec3> getCastingRegionalCenterOfPressure(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, 
            "casting.regional.center_of_pressure")) {

            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "casting.regional.center_of_pressure");
    }

    //contact force
    SimTK::Vec3 getTargetTotalContactForce(const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "target.total.contact_force")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vec3>
            (state, "target.total.contact_force");
    }

    SimTK::Vec3 getCastingTotalContactForce(const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.total.contact_force")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vec3>
            (state, "casting.total.contact_force");
    }

    SimTK::Vector_<SimTK::Vec3> getTargetRegionalContactForce(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "target.regional.contact_force")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "target.regional.contact_force");
    }

    SimTK::Vector_<SimTK::Vec3> getCastingRegionalContactForce(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.regional.contact_force")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "casting.regional.contact_force");
    }

    //contact moment
    SimTK::Vec3 getTargetTotalContactMoment(const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "target.total.contact_moment")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vec3>
            (state, "target.total.contact_moment");
    }

    SimTK::Vec3 getCastingTotalContactMoment(const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.total.contact_moment")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vec3>
            (state, "casting.total.contact_moment");
    }

    SimTK::Vector_<SimTK::Vec3> getTargetRegionalContactMoment(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "target.regional.contact_moment")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "target.regional.contact_moment");
    }

    SimTK::Vector_<SimTK::Vec3> getCastingRegionalContactMoment(
        const SimTK::State& state) const {

        if (!isCacheVariableValid(state, "casting.regional.contact_moment")) {
            realizeContactMetricCaches(state);
        }

        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "casting.regional.contact_moment");
    }

    // END Lazy outputs

    double computePotentialEnergy(
        const SimTK::State& state) const override;

    void computeForce(const SimTK::State& state,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& generalizedForces) const override;

    OpenSim::Array<double> getRecordValues(const SimTK::State& s) const;
    OpenSim::Array<std::string> getRecordLabels() const;

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendRealizeReport(const SimTK::State & state) const override;

    void computeMeshProximity(const SimTK::State& state,
        const Smith2018ContactMesh& casting_mesh,
        const Smith2018ContactMesh& target_mesh,
        const std::string& cache_mesh_name) const;

    void computeMeshProximity(const SimTK::State& state,
        const Smith2018ContactMesh& casting_mesh,
        const Smith2018ContactMesh& target_mesh,
        const std::string& cache_mesh_name,
        SimTK::Vector& triangle_proximity) const;

    void computeMeshDynamics(const SimTK::State& state,
        const Smith2018ContactMesh& casting_mesh,
        const Smith2018ContactMesh& target_mesh) const;

    void computeMeshDynamics(const SimTK::State& state,
        const Smith2018ContactMesh& casting_mesh,
        const Smith2018ContactMesh& target_mesh,
        SimTK::Vector_<SimTK::Vec3>& triangle_force,
        SimTK::Vector& triangle_pressure,SimTK::Vector& triangle_energy) const;

    SimTK::Vec3 computeContactForceVector(
        double pressure, double area, SimTK::Vec3 normal) const;

    SimTK::Vec3 computeContactMomentVector(
        double pressure, double area, SimTK::Vec3 normal,
        SimTK::Vec3 center) const;

    ContactStats computeContactStats(const Smith2018ContactMesh& mesh,
        const SimTK::Vector& total_triangle_proximity,
        const SimTK::Vector& total_triangle_pressure,
        const std::vector<int>& triIndices) const;

    void realizeContactMetricCaches(const SimTK::State& state) const;
    
    //void computeRegionalContactStats(const SimTK::State& state) const;

private:
    void setNull();
    void constructProperties();

    double calcTrianglePressureVariableNonlinearModel(double proximity,
        double casting_thickness, double target_thickness,
        double casting_E, double target_E,
        double casting_v, double target_v, double init_guess) const;

    /*
    * A utility function used by computeTriPressure for the function lmdif_C, which
    * is used to solve the nonlinear equation for pressure.
    *
    * h1(1-exp(-P1/k1))+h2(1-exp(P1/k2))-dc = 0;
    *
    * @param nEqn The number of Equations (1)
    * @param nVars The number of variables (1)
    * @param q Array of values of the degrees of freedom
    * @param resid Array of residuals to be calculated
    * @param flag2 A status flag
    * @param ptr Pointer to data structure containing values for h1, h2, k1, k2, dc
    */
    static void calcNonlinearPressureResidual(
        int nEqn, int nVar, double q[], double resid[],
        int *flag2, void *ptr);

    //=========================================================================
    // Member Variables
    //=========================================================================
    struct NonlinearContactParams {
        double h1, h2, k1, k2, dc;
    };

    struct ContactStats
    {
        double contact_area;
        double mean_proximity;
        double max_proximity;
        SimTK::Vec3 center_of_proximity;
        double mean_pressure;
        double max_pressure;
        SimTK::Vec3 center_of_pressure;
        SimTK::Vec3 contact_force;
        SimTK::Vec3 contact_moment;
    };

    std::vector<std::string> _region_names;
    std::vector<std::string> _stat_names;
    std::vector<std::string> _stat_names_vec3;
    std::vector<std::string> _mesh_data_names;
};
//=============================================================================
// END of class Smith2018ArticularContactForce
//=============================================================================
}; // end of namespace OpenSim

#endif // OPENSIM_SMITH_2018_ARTICULAR_CONTACT_FORCE_H_
