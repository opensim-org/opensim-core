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

// INCLUDE
#include "OpenSim/Simulation/Model/Force.h"
#include "Smith2018ContactMesh.h"


namespace OpenSim {
/**
 This Force component models the articular contact between trianglated surface 
 meshes representing cartilage, mensici, or implant components [1]. The 
 formulation of the contact model has previously been called an elastic 
 foundation model [2] or discrete element analysis [3,4]. In this 
 implementation, non-deforming triangulated meshes are allowed to 
 interpenetrate and the local overlap depth (proximity) is calculated for 
 each triangle. The contact pressure on each triangle face is then calculated 
 based on the overlap depth (see below).
 
 To calculate the local overlap depth, it is necessary to detect the mesh 
 triangles that are interpenetrating (commonly called contact or collision 
 detection in computer graphics literature). This process is extremely slow if 
 a brute force approach is applied to check every triangle in one mesh against 
 every triangle in another mesh. Smith et al, CMBBE I&V, 2018 [1] introduced a 
 method to efficiently detect contact between triangular meshes using object 
 Oriented Bounding Boxes (OBB, a common approach in computer graphics) and 
 several additional speed ups that take advantage of the constrained nature of 
 articular contact. This approach has been implemented in the 
 Smith2018ArticularContactForce component along with some additional 
 features. 
 
 Two articulating triangular meshes are defined as Smith2018ContactMesh 
 components (Sockets: casting_mesh and target_mesh). These meshes are fixed to
 bodies in the model, and thus their relative poses are determined by the 
 model coordinates. To detect contact, a normal ray is cast from the center of
 each triangle in the casting mesh backwards towards the overlapping target
 mesh. Ray intersection tests are then performed against an Oriented Bounding 
 Box tree constructed around the target mesh. This algorithm is implemented in
 the computeTriProximity() function with the OBB construction and ray 
 intersection queries managed by the Smith2018ContactMesh.

 The major speed up in the algorithm leverages the fact that changes in joint 
 coordinates and thus articular contact patterns between time steps are 
 generally small. Thus, after reposing the meshes, (i.e. realizePosition) each 
 triangle in the casting mesh is tested against the contacting target triangle 
 from the previous pose. Additional speed up is gained by casting the normal 
 ray in both directions, so even some of the out of contact triangles are 
 "remembered". If the previous contacting triangle test fails, the casting ray 
 is checked against the neighboring triangles (those that share a vertex) in 
 the target mesh. Then if this test fails, the expensive casting ray - OBB 
 test is performed.

 When computing the contact force the ray casting is only performed from the
 casting_mesh towards the target_mesh. Thus, a pressure map is only generated 
 for the casting_mesh. A force vector is computed for each triangle in the 
 casting_mesh using Force = -normal*area*pressure, and the resultant force of 
 the summation of all triangle forces is applied to the casting_mesh in the 
 computeForce() function. The equal and opposite force is applied to the 
 target_mesh. The important ramification of this is that if the casting_mesh 
 and target_mesh are switched, the simulation results will NOT be exactly the 
 same. For best performance, the casting_mesh should be set to the mesh that 
 contains the smaller number of triangles.
 
 As it is still useful to visualize the proximity and 
 pressure maps for the target_mesh, and in most situations the resultant force 
 from the target_mesh are very close to the mirrored casting_mesh resultant 
 force, there is a ModelingOption named "flip_meshes" that will cause the ray 
 casting to also be performed from the target_mesh to calculate triangle 
 proximity and pressure values. Note the applied contact force in this case is 
 still only that calculated for the casting_mesh. 
 
 A linear and non-linear relationship between overlap depth and pressure can 
 be used via the elastic_foundation_formulation property. The implemented 
 equations are those proposed in Bei and Fregly, Med Eng Phys, 2004 [2]:

 Linear: 
 \f[
    P = E*\frac{(1-\nu)}{(1 + \nu)(1-2\nu)}*\frac{d}{h}
 \f]
 
 Non-Linear: 
 \f[
   P = -E*\frac{(1-\nu)}{(1 + \nu)(1-2\nu)}*ln(1-\frac{d}{h})
 \f]
 
 Where:
 \f[ P = pressure \f]
 \f[ E = elastic\: modulus \f]
 \f[ \nu = Poisson's\: ratio \f]
 \f[ d = depth\: of\: overlap \f]
 \f[ h = height\: (i.e.\:thickness)\: of\: the\: elastic\: layer \f]


 The original Bei and Fregly formulation assumes that a rigid object is 
 contacting an object with a thin elastic layer. This is straightforward to 
 apply to joint replacements where a metal component contacts a polyethelene 
 component. To model cartilage-cartilage contact, this approach requires that
 the two cartilage layers are lumped together into one elastic layer, 
 necessitating that a constant thickness, elastic modulus, and poissons ratio 
 is assumed for each contacting triangle pair. Thus the total overlap depth is 
 split equally between each triangle contact pair. As cartilage-cartilage 
 contact often involves articulations between cartilage surfaces with varying 
 thickness and material properties, the Bei and Fregly approach was extended 
 to accomodate variable properties. The use_lumped_contact_model property 
 controls whether the constant property or variable property formulation is 
 used.
 
 The variable property formulation is described in Zevenbergen et al, 
 PLOS One, 2018 [5]. Here, the following system of four equations must be
 solved to obtain the local overlap depth (proximity) and pressure for the 
 casting and target triangles. 
  
 \f[
   P_\mathrm{casting} = f(E,\nu,h,d_\mathrm{casting})\:
   (linear or non-linear formulation above)
 \f]
 

 \f[
   P_\mathrm{target} = f(E,\nu,h,d_\mathrm{target})\:
   (linear or non-linear formulation above) 
 \f]

 \f[
   P_\mathrm{casting} = P_\mathrm{target}
 \f]
 
 \f[
   d = d_\mathrm{casting} + d_\mathrm{target}
 \f]

 Here, the first two equations use the Bei and Fregly elastic foundation model
 to define the relationship between the local mesh properties, local overlap 
 depth and computed pressure. The third equation is a force equilibrium, 
 assuming that the force applied to a pair of contacting triangles is 
 equal and opposite. This formulation further assumes that the triangles in 
 contact have the same area. The fourth equation states that the total overlap
 depth of the meshes (which is readily calculated) is the sum of the 
 local overlap depths of the two elastic layes in contact. 

 This system of equations can be solved analytically if the linear Pressure-
 depth relationship is used. If the non-linear relationship is used, the 
 system of equations is solved using a numerical solver. 

 The min_proximity and max_proximity properties limit the search region for a 
 contact triangle along a ray cast from the casting_mesh. These values should 
 be set to resonable limits for your application to prevent invalid contacts
 from being found. This is particularly important for situations with highly
 curved meshes where a ray cast from the casting_mesh may intersect the 
 target_mesh multiple times, or at infeasible contact locations. An example
 of this is patellofemoral contact, where rays cast from the patella 
 cartilage mesh can interesect the backsides of the femoral condyles, or the 
 the sides of the trochealar groove multiple times. The min_proximity can be 
 set to a negative value if you would like distance maps for the out of 
 contact triangles. For example, if you are visualizing kinematics measured 
 with fluoroscopy and only have meshes of the bones, using a negative 
 min_proximity enables the distance between the subcondral surfaces to be 
 mapped.

 This component has some outputs such as tri_proximity, tri_pressure, and 
 tri_potential_energy that return a SimTK::Vector with a value corresponding 
 to each  triangle face in the respective target or casting mesh. There are 
 also "summary" outputs that return values associated with the entire mesh 
 such as contact area, mean/max proximity/pressure, center of 
 proximity/pressure etc. Finally, there are regional summary outputs which 
 return a 1 x 6 SimTK::Vector. Here, the entries in the vector reflect the 
 summary metrics in a subspace of the mesh. The entries in the 
 1 x 6 SimTK::Vector correspond to the subset of 
 triangles whose center is located in the half space [+x, -x, +y, -y, +z, -z].
 If the mesh coordinate system is aligned with anatomical axes, then this 
 enables simulation results to be more readily interpreted. For example, 
 when performing simulations of the knee, if the z axis is aligned to the 
 medial-lateral axis, points medially, and the origin is located between the 
 femoral condyles, then the regional outputs corresponding to +z and -z will
 enable comparisons of the loading in the medial and lateral compartments. 

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

        struct contact_stats;

public:
    class ContactParameters;


    //=========================================================================
    // PROPERTIES
    //=========================================================================
    OpenSim_DECLARE_PROPERTY(min_proximity, double, "The minimum overlap "
        "depth between contacting meshes. Note this can be negative if "
        "distance maps should include triangles that are not in contact.")
    OpenSim_DECLARE_PROPERTY(max_proximity, double, "The maximum overlap "
        "depth between contacting meshes.")
    OpenSim_DECLARE_PROPERTY(elastic_foundation_formulation, std::string,
        "Formulation for depth-pressure relationship: "
        "'linear' or 'nonlinear'")
    OpenSim_DECLARE_PROPERTY(use_lumped_contact_model, bool,
        "Combine the thickness and the average material properties between "
        "the ContactParams for both meshes and use Bei & Fregly 2003"
        " lumped parameter Elastic Foundation model")
    OpenSim_DECLARE_PROPERTY(target_mesh_contact_params,
        Smith2018ArticularContactForce::ContactParameters,
        "target_mesh material properties")
    OpenSim_DECLARE_PROPERTY(casting_mesh_contact_params,
        Smith2018ArticularContactForce::ContactParameters,
        "casting_mesh material properties")


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
    //number of colliding triangles
    OpenSim_DECLARE_OUTPUT(target_total_n_contacting_tri, int,
        getTargetNContactingTri, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_total_n_contacting_tri, int,
        getCastingNContactingTri, SimTK::Stage::Dynamics)

    //tri proximity
    OpenSim_DECLARE_OUTPUT(target_tri_proximity, SimTK::Vector,
        getTargetTriProximity, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_tri_proximity, SimTK::Vector,
        getCastingTriProximity, SimTK::Stage::Position)

    //tri pressure
    OpenSim_DECLARE_OUTPUT(target_tri_pressure, SimTK::Vector,
        getTargetTriPressure, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_tri_pressure, SimTK::Vector,
        getCastingTriPressure, SimTK::Stage::Dynamics)

    //tri potential energy
    OpenSim_DECLARE_OUTPUT(target_tri_potential_energy, SimTK::Vector,
        getTargetTriPotentialEnergy, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_tri_potential_energy, SimTK::Vector,
        getCastingTriPotentialEnergy, SimTK::Stage::Dynamics)

    //contact_area
    OpenSim_DECLARE_OUTPUT(target_total_contact_area, double,
        getTargetContactArea, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_total_contact_area, double,
        getCastingContactArea, SimTK::Stage::Position)

    OpenSim_DECLARE_OUTPUT(target_regional_contact_area, SimTK::Vector,
        getTargetRegionalContactArea, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_regional_contact_area, SimTK::Vector,
        getCastingRegionalContactArea, SimTK::Stage::Position)

    //mean proximity
    OpenSim_DECLARE_OUTPUT(target_total_mean_proximity, double,
        getTargetMeanProximity, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_total_mean_proximity, double,
        getCastingMeanProximity, SimTK::Stage::Position)

    OpenSim_DECLARE_OUTPUT(target_regional_mean_proximity, SimTK::Vector,
        getTargetRegionalMeanProximity, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_regional_mean_proximity, SimTK::Vector,
        getCastingRegionalMeanProximity, SimTK::Stage::Position)

    //max proximity
    OpenSim_DECLARE_OUTPUT(target_total_max_proximity, double,
        getTargetMaxProximity, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_total_max_proximity, double,
        getCastingMaxProximity, SimTK::Stage::Position)

    OpenSim_DECLARE_OUTPUT(target_regional_max_proximity, SimTK::Vector,
        getTargetRegionalMaxProximity, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_regional_max_proximity, SimTK::Vector,
        getCastingRegionalMaxProximity, SimTK::Stage::Position)

    //mean pressure
    OpenSim_DECLARE_OUTPUT(target_total_mean_pressure, double,
        getTargetMeanPressure, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_total_mean_pressure, double,
        getCastingMeanPressure, SimTK::Stage::Dynamics)

    OpenSim_DECLARE_OUTPUT(target_regional_mean_pressure, SimTK::Vector,
        getTargetRegionalMeanPressure, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_regional_mean_pressure, SimTK::Vector,
        getCastingRegionalMeanPressure, SimTK::Stage::Dynamics)

    //max pressure
    OpenSim_DECLARE_OUTPUT(target_total_max_pressure, double,
        getTargetMaxPressure, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_total_max_pressure, double,
        getCastingMaxPressure, SimTK::Stage::Dynamics)

    OpenSim_DECLARE_OUTPUT(target_regional_max_pressure, SimTK::Vector,
        getTargetRegionalMaxPressure, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_regional_max_pressure, SimTK::Vector,
        getCastingRegionalMaxPressure, SimTK::Stage::Dynamics)

    //center of proximity
    OpenSim_DECLARE_OUTPUT(target_total_center_of_proximity, SimTK::Vec3,
        getTargetCenterOfProximity, SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_total_center_of_proximity, SimTK::Vec3,
        getCastingCenterOfProximity, SimTK::Stage::Position)

    OpenSim_DECLARE_OUTPUT(target_regional_center_of_proximity,
        SimTK::Vector_<SimTK::Vec3>, getTargetRegionalCenterOfProximity,
        SimTK::Stage::Position)
    OpenSim_DECLARE_OUTPUT(casting_regional_center_of_proximity,
        SimTK::Vector_<SimTK::Vec3>, getCastingRegionalCenterOfProximity,
        SimTK::Stage::Position)

    //center of pressure
    OpenSim_DECLARE_OUTPUT(target_total_center_of_pressure, SimTK::Vec3,
        getTargetCenterOfPressure, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_total_center_of_pressure, SimTK::Vec3,
        getCastingCenterOfPressure, SimTK::Stage::Dynamics)

    OpenSim_DECLARE_OUTPUT(target_regional_center_of_pressure,
        SimTK::Vector_<SimTK::Vec3>, getTargetRegionalCenterOfPressure,
        SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_regional_center_of_pressure,
        SimTK::Vector_<SimTK::Vec3>, getCastingRegionalCenterOfPressure,
        SimTK::Stage::Dynamics)

    //contact force
    OpenSim_DECLARE_OUTPUT(target_total_contact_force, SimTK::Vec3,
        getTargetContactForce, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_total_contact_force, SimTK::Vec3,
        getCastingContactForce, SimTK::Stage::Dynamics)

    OpenSim_DECLARE_OUTPUT(target_regional_contact_force,
        SimTK::Vector_<SimTK::Vec3>, getTargetRegionalContactForce,
        SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_regional_contact_force,
        SimTK::Vector_<SimTK::Vec3>, getCastingRegionalContactForce,
        SimTK::Stage::Dynamics)

    //contact moment
    OpenSim_DECLARE_OUTPUT(target_total_contact_moment, SimTK::Vec3,
        getTargetContactMoment, SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_total_contact_moment, SimTK::Vec3,
        getCastingContactMoment, SimTK::Stage::Dynamics)

    OpenSim_DECLARE_OUTPUT(target_regional_contact_moment,
        SimTK::Vector_<SimTK::Vec3>, getTargetRegionalContactMoment,
        SimTK::Stage::Dynamics)
    OpenSim_DECLARE_OUTPUT(casting_regional_contact_moment,
        SimTK::Vector_<SimTK::Vec3>, getCastingRegionalContactMoment,
        SimTK::Stage::Dynamics)

    //=========================================================================
    //METHODS
    //=========================================================================

    Smith2018ArticularContactForce();

    Smith2018ArticularContactForce(const std::string& name, Smith2018ContactMesh& target_mesh,
        Smith2018ContactMesh& casting_mesh);

    Smith2018ArticularContactForce(const std::string& name,
        Smith2018ContactMesh& target_mesh,
        Smith2018ArticularContactForce::ContactParameters
        target_mesh_params, Smith2018ContactMesh& casting_mesh,
        Smith2018ArticularContactForce::ContactParameters
        casting_mesh_params);

    Smith2018ArticularContactForce(const std::string& name,
        Smith2018ContactMesh& target_mesh,Smith2018ContactMesh& casting_mesh,
        double elastic_modulus, double poissons_ratio, double thickness);

public:

    //-------------------------------------------------------------------------
    //Output Methods
    //-------------------------------------------------------------------------

    //number of contacting triangles
    int getTargetNContactingTri(const SimTK::State& state) const {
        return getCacheVariableValue<int>
            (state, "target.n_contacting_tri");
    }

    int getCastingNContactingTri(const SimTK::State& state) const {
        return getCacheVariableValue<int>
            (state, "casting.n_contacting_tri");
    }

    //tri proximity
    SimTK::Vector getTargetTriProximity(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "target.tri.proximity");
    }
    SimTK::Vector getCastingTriProximity(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.tri.proximity");
    }

    //tri pressure
    SimTK::Vector getTargetTriPressure(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "target.tri.pressure");
    }
    SimTK::Vector getCastingTriPressure(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.tri.pressure");
    }

    //tri potential energy
    SimTK::Vector getTargetTriPotentialEnergy(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "target.tri.potential_energy");
    }
    SimTK::Vector getCastingTriPotentialEnergy(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.tri.potential_energy");
    }

    //contact_area
    double getTargetContactArea(const SimTK::State& state) const {
        return getCacheVariableValue<double>
            (state, "target.contact_area");
    }
    double getCastingContactArea(const SimTK::State& state) const {
        return getCacheVariableValue<double>
            (state, "casting.contact_area");
    }

    SimTK::Vector getTargetRegionalContactArea(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "target.regional.contact_area");
    }
    SimTK::Vector getCastingRegionalContactArea(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.regional.contact_area");
    }

    //mean proximity
    double getTargetMeanProximity(const SimTK::State& state) const {
        return getCacheVariableValue<double>
            (state, "target.mean_proximity");
    }
    double getCastingMeanProximity(const SimTK::State& state) const {
        return getCacheVariableValue<double>
            (state, "casting.mean_proximity");
    }

    SimTK::Vector getTargetRegionalMeanProximity(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "target.regional.mean_proximity");
    }
    SimTK::Vector getCastingRegionalMeanProximity(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.regional.mean_proximity");
    }

    //max proximity
    double getTargetMaxProximity(const SimTK::State& state) const {
        return getCacheVariableValue<double>
            (state, "target.max_proximity");
    }
    double getCastingMaxProximity(const SimTK::State& state) const {
        return getCacheVariableValue<double>
            (state, "casting.max_proximity");
    }

    SimTK::Vector getTargetRegionalMaxProximity(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "target.regional.max_proximity");
    }
    SimTK::Vector getCastingRegionalMaxProximity(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.regional.max_proximity");
    }

    //mean pressure
    double getTargetMeanPressure(const SimTK::State& state) const {
        return getCacheVariableValue<double>
            (state, "target.mean_pressure");
    }
    double getCastingMeanPressure(const SimTK::State& state) const {
        return getCacheVariableValue<double>
            (state, "casting.mean_pressure");
    }

    SimTK::Vector getTargetRegionalMeanPressure(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "target.regional.mean_pressure");
    }
    SimTK::Vector getCastingRegionalMeanPressure(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.regional.mean_pressure");
    }

    //max pressure
    double getTargetMaxPressure(const SimTK::State& state) const {
        return getCacheVariableValue<double>
            (state, "target.max_pressure");
    }
    double getCastingMaxPressure(const SimTK::State& state) const {
        return getCacheVariableValue<double>
            (state, "casting.max_pressure");
    }

    SimTK::Vector getTargetRegionalMaxPressure(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "target.regional.max_pressure");
    }
    SimTK::Vector getCastingRegionalMaxPressure(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector>
            (state, "casting.regional.max_pressure");
    }

    //center of proximity
    SimTK::Vec3 getTargetCenterOfProximity(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vec3>
            (state, "target.center_of_proximity");
    }
    SimTK::Vec3 getCastingCenterOfProximity(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vec3>
            (state, "casting.center_of_proximity");
    }

    SimTK::Vector_<SimTK::Vec3> getTargetRegionalCenterOfProximity(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "target.regional.center_of_proximity");
    }
    SimTK::Vector_<SimTK::Vec3> getCastingRegionalCenterOfProximity(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "casting.regional.center_of_proximity");
    }

    //center of pressure
    SimTK::Vec3 getTargetCenterOfPressure(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vec3>
            (state, "target.center_of_pressure");
    }
    SimTK::Vec3 getCastingCenterOfPressure(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vec3>
            (state, "casting.center_of_pressure");
    }

    SimTK::Vector_<SimTK::Vec3> getTargetRegionalCenterOfPressure(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "target.regional.center_of_pressure");
    }
    SimTK::Vector_<SimTK::Vec3> getCastingRegionalCenterOfPressure(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "casting.regional.center_of_pressure");
    }

    //contact force
    SimTK::Vec3 getTargetContactForce(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vec3>
            (state, "target.contact_force");
    }
    SimTK::Vec3 getCastingContactForce(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vec3>
            (state, "casting.contact_force");
    }

    SimTK::Vector_<SimTK::Vec3> getTargetRegionalContactForce(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "target.regional.contact_force");
    }
    SimTK::Vector_<SimTK::Vec3> getCastingRegionalContactForce(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "casting.regional.contact_force");
    }

    //contact moment
    SimTK::Vec3 getTargetContactMoment(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vec3>
            (state, "target.contact_moment");
    }
    SimTK::Vec3 getCastingContactMoment(const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vec3>
            (state, "casting.contact_moment");
    }

    SimTK::Vector_<SimTK::Vec3> getTargetRegionalContactMoment(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "target.regional.contact_moment");
    }
    SimTK::Vector_<SimTK::Vec3> getCastingRegionalContactMoment(
        const SimTK::State& state) const {
        return getCacheVariableValue<SimTK::Vector_<SimTK::Vec3>>
            (state, "casting.regional.contact_moment");
    }

    OpenSim::Array<double> getRecordValues(const SimTK::State& s) const;
    OpenSim::Array<std::string> getRecordLabels() const;

protected:
    double computePotentialEnergy(
        const SimTK::State& state) const override;

    void computeForce(const SimTK::State& state,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& generalizedForces) const override;

    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& state)	const override;
    void extendRealizeReport(const SimTK::State & state)	const override;

    void computeTriProximity(const SimTK::State& state,
        const Smith2018ContactMesh& casting_mesh,
        const Smith2018ContactMesh& target_mesh,
        const std::string& cache_mesh_name,
        SimTK::Vector& tri_proximity) const;

    void computeTriDynamics(const SimTK::State& state,
        const Smith2018ContactMesh& casting_mesh,
        const Smith2018ContactMesh& target_mesh,
        SimTK::Vector_<SimTK::Vec3>& tri_force,
        SimTK::Vector& tri_pressure,SimTK::Vector& tri_energy) const;

    SimTK::Vec3 computeContactForceVector(
        double pressure, double area, SimTK::Vec3 normal) const;

    SimTK::Vec3 computeContactMomentVector(
        double pressure, double area, SimTK::Vec3 normal,
        SimTK::Vec3 center) const;

    contact_stats computeContactStats(const Smith2018ContactMesh& mesh,
        const SimTK::Vector& total_tri_proximity,
        const SimTK::Vector& total_tri_pressure,
        const std::vector<int>& triIndices) const;

private:
    void setNull();
    void constructProperties();

    static void calcNonlinearPressureResid(
        int nEqn, int nVar, double q[], double resid[],
        int *flag2, void *ptr);

    //=========================================================================
    //Member Variables
    //=========================================================================
    struct nonlinearContactParams {
        double h1, h2, k1, k2, dc;
    };

    struct contact_stats
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


//=============================================================================
//              Smith2018ArticularContactForce :: CONTACT PARAMETERS
//=============================================================================
class OSIMSIMULATION_API Smith2018ArticularContactForce::ContactParameters :
    public Object { 
    OpenSim_DECLARE_CONCRETE_OBJECT(
        Smith2018ArticularContactForce::ContactParameters, Object)
        
public:
    //=========================================================================
    // PROPERTIES
    //=========================================================================
        OpenSim_DECLARE_PROPERTY(elastic_modulus, double, 
            "Uniform Elastic Modulus value for entire mesh")
        OpenSim_DECLARE_PROPERTY(poissons_ratio, double, 
            "Uniform Poissons Ratio value for entire mesh")
        OpenSim_DECLARE_PROPERTY(thickness, double,
            "Uniform thickness of elastic layer for entire mesh")
        OpenSim_DECLARE_PROPERTY(use_variable_thickness, bool,
            "Flag to use variable thickness."
            "Note: mesh_back_file must defined in Smith2018ContactMesh")
        //=====================================================================
        // METHODS
        //=====================================================================
        ContactParameters();
        ContactParameters(double youngs_modulus, double poissons_ratio,
            double thickness);

private:
    void constructProperties();
};


} // end of namespace OpenSim

#endif // OPENSIM_SMITH_2018_ARTICULAR_CONTACT_FORCE_H_