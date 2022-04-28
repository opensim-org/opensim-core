/* -------------------------------------------------------------------------- *
 *                 Smith2018ArticularContactForce.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
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



//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/GCVSpline.h>
#include "Smith2018ArticularContactForce.h"
#include "Smith2018ContactMesh.h"
#include <cctype>
#include <OpenSim/Common/Lmdif.h>

//=============================================================================
// USING
//=============================================================================
using namespace OpenSim;
using namespace SimTK;

//=============================================================================
// CONSTRUCTOR(S) 
//=============================================================================

Smith2018ArticularContactForce::Smith2018ArticularContactForce() : Force()
{
    setNull();
    constructProperties();
}

Smith2018ArticularContactForce::Smith2018ArticularContactForce(
    const std::string& name,
    Smith2018ContactMesh& target_mesh,Smith2018ContactMesh& casting_mesh)
{
    setNull();
    constructProperties();

    setName(name);

    updSocket<Smith2018ContactMesh>("target_mesh").connect(target_mesh);
    updSocket<Smith2018ContactMesh>("casting_mesh").connect(casting_mesh);
}

void Smith2018ArticularContactForce::setNull()
{
    setAuthors("Colin Smith");
    setReferences(
        "Smith, C. R., Won Choi, K., Negrut, D., & Thelen, D. G. (2018)."
        "Efficient computation of cartilage contact pressures within dynamic "
        "simulations of movement. Computer Methods in Biomechanics and "
        "Biomedical Engineering: Imaging & Visualization, 6(5), 491-498.");
}

void Smith2018ArticularContactForce::constructProperties()
{
    constructProperty_min_proximity(0.00);
    constructProperty_max_proximity(0.01);
    constructProperty_elastic_foundation_formulation("linear");
    constructProperty_use_lumped_contact_model(true);
}

void Smith2018ArticularContactForce::
extendAddToSystem(MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    int target_mesh_nTri =
        getSocket<Smith2018ContactMesh>("target_mesh").
        getConnectee().getNumFaces();
    int casting_mesh_nTri = 
        getSocket<Smith2018ContactMesh>("casting_mesh").
        getConnectee().getNumFaces();

    Vector target_mesh_def_vec(target_mesh_nTri);
    Vector casting_mesh_def_vec(casting_mesh_nTri);
    target_mesh_def_vec = -1;
    casting_mesh_def_vec = -1;

    Vector_<Vec3> target_mesh_def_vec3(target_mesh_nTri,Vec3(0.0));
    Vector_<Vec3> casting_mesh_def_vec3(casting_mesh_nTri,Vec3(0.0));
    

    std::vector<int> target_mesh_def_vector_int(target_mesh_nTri,-1);
    std::vector<int> casting_mesh_def_vector_int(casting_mesh_nTri,-1);

    //TODO: These need to be accessed at in Stage::Position in computeProximity()
    // for rechecking the same triangle that was in contact in the previous
    // state. Is there a better way to make them accessible without setting
    // the stage to LowestRuntime???

    this->_target_triangle_previous_contacting_triangleCV = 
        addCacheVariable(
        "target_triangle_previous_contacting_triangle",
        target_mesh_def_vector_int, Stage::LowestRuntime);

    this->_casting_triangle_previous_contacting_triangleCV =
        addCacheVariable(
        "casting_triangle_previous_contacting_triangle",
        casting_mesh_def_vector_int, Stage::LowestRuntime);

    //Triangles with ray intersections
    this->_target_num_active_trianglesCV = 
        addCacheVariable("target_num_active_triangles",
        0, Stage::Position);

    this->_casting_num_active_trianglesCV =
        addCacheVariable("casting_num_active_triangles",
        0, Stage::Position);

    //Subset of num_active_triangles with positive proximity
    this->_target_num_contacting_trianglesCV = 
        addCacheVariable("target_num_contacting_triangles",
        0, Stage::Position);

    this->_casting_num_contacting_trianglesCV = 
        addCacheVariable("casting_num_contacting_triangles",
        0, Stage::Position);
    
    //same, neighbor, and different are useful for debugging issues with
    //newly constructed contact meshes
    //Subset of n_contacting_tri that contact same triangle as previous step
    this->_target_num_contacting_triangles_sameCV =
        addCacheVariable("target_num_contacting_triangles_same",
        0, Stage::Position);

    this->_casting_num_contacting_triangles_sameCV =
        addCacheVariable("casting_num_contacting_triangles_same",
        0, Stage::Position);

    //Subset of n_contacting_tri that contact 
    //neighboring triangle to previous step
    this->_target_num_contacting_triangles_neighborCV =
        addCacheVariable("target_num_contacting_triangles_neighbor",
        0, Stage::Position);

    this->_casting_num_contacting_triangles_neighborCV = 
        addCacheVariable("casting_num_contacting_triangles_neighbor",
        0, Stage::Position);

    //Subset of n_contacting_tri that contact different triangle from previous 
    //step (not same or neighbor), this means expensive OBB check was used
    this->_target_num_contacting_triangles_differentCV =
        addCacheVariable("target_num_contacting_triangles_different",
        0, Stage::Position);

    this->_casting_num_contacting_triangles_differentCV = 
        addCacheVariable("casting_num_contacting_triangles_different",
        0, Stage::Position);


    this->_target_triangle_proximityCV =
        addCacheVariable("target_triangle_proximity",
        target_mesh_def_vec, Stage::Position);
    this->_casting_triangle_proximityCV =
        addCacheVariable("casting_triangle_proximity",
        casting_mesh_def_vec, Stage::Position);
    
    this->_target_triangle_pressureCV =
        addCacheVariable("target_triangle_pressure",
        target_mesh_def_vec, Stage::Position);
    this->_casting_triangle_pressureCV =
        addCacheVariable("casting_triangle_pressure",
        casting_mesh_def_vec, Stage::Position);

    this->_target_triangle_potential_energyCV = 
        addCacheVariable("target_triangle_potential_energy",
        target_mesh_def_vec, Stage::Position);
    this->_casting_triangle_potential_energyCV =
        addCacheVariable("casting_triangle_potential_energy",
        casting_mesh_def_vec, Stage::Position);

    this->_target_triangle_forceCV =
        addCacheVariable("target_triangle_force",
        target_mesh_def_vec3, Stage::Position);
    this->_casting_triangle_forceCV = 
        addCacheVariable("casting_triangle_force",
        casting_mesh_def_vec3, Stage::Position);


    //Lazy Evaluations (only computed getXX is called)
    this->_target_total_contact_areaCV = 
        addCacheVariable
        ("target_total_contact_area", 0.0, Stage::Position);
    this->_target_total_mean_proximityCV =
        addCacheVariable
        ("target_total_mean_proximity", 0.0, Stage::Position);
    this->_target_total_max_proximityCV = 
        addCacheVariable
        ("target_total_max_proximity", 0.0, Stage::Position);
    this->_target_total_center_of_proximityCV =
        addCacheVariable
        ("target_total_center_of_proximity", Vec3(0), Stage::Position);
    this->_target_total_mean_pressureCV =
        addCacheVariable
        ("target_total_mean_pressure", 0.0, Stage::Position);
    this->_target_total_max_pressureCV =
        addCacheVariable
        ("target_total_max_pressure", 0.0, Stage::Position);
    this->_target_total_center_of_pressureCV =
        addCacheVariable
        ("target_total_center_of_pressure", Vec3(0), Stage::Position);
    this->_target_total_contact_forceCV = 
        addCacheVariable
        ("target_total_contact_force", Vec3(0), Stage::Position);
    this->_target_total_contact_momentCV =
        addCacheVariable(
        "target_total_contact_moment", Vec3(0), Stage::Position);
    this->_casting_total_contact_areaCV =
        addCacheVariable(
        "casting_total_contact_area", 0.0, Stage::Position);
    this->_casting_total_mean_proximityCV =
        addCacheVariable(
        "casting_total_mean_proximity", 0.0, Stage::Position);
    this->_casting_total_max_proximityCV =
        addCacheVariable(
        "casting_total_max_proximity", 0.0, Stage::Position);
    this->_casting_total_center_of_proximityCV =
        addCacheVariable(
        "casting_total_center_of_proximity", Vec3(0), Stage::Position);
    this->_casting_total_mean_pressureCV =
        addCacheVariable(
        "casting_total_mean_pressure", 0.0, Stage::Position);
    this->_casting_total_max_pressureCV =
        addCacheVariable(
        "casting_total_max_pressure", 0.0, Stage::Position);
    this->_casting_total_center_of_pressureCV =
        addCacheVariable(
        "casting_total_center_of_pressure", Vec3(0), Stage::Position);
    this->_casting_total_contact_forceCV =
        addCacheVariable(
        "casting_total_contact_force", Vec3(0), Stage::Position);
    this->_casting_total_contact_momentCV =
        addCacheVariable(
        "casting_total_contact_moment", Vec3(0), Stage::Position);
    this->_target_regional_contact_areaCV =
        addCacheVariable(
        "target_regional_contact_area", Vector(6,0.0), Stage::Position);
    this->_target_regional_mean_proximityCV =
        addCacheVariable(
        "target_regional_mean_proximity", Vector(6,0.0), Stage::Position);
    this->_target_regional_max_proximityCV =
        addCacheVariable(
        "target_regional_max_proximity", Vector(6,0.0), Stage::Position);
    this->_target_regional_center_of_proximityCV =
        addCacheVariable(
        "target_regional_center_of_proximity", Vector_<Vec3>(6,Vec3(0)),
        Stage::Position);
    this->_target_regional_mean_pressureCV =
        addCacheVariable(
        "target_regional_mean_pressure", Vector(6,0.0), Stage::Position);
    this->_target_regional_max_pressureCV =
        addCacheVariable(
        "target_regional_max_pressure",Vector(6,0.0), Stage::Position);
    this->_target_regional_center_of_pressureCV =
        addCacheVariable(
        "target_regional_center_of_pressure", Vector_<Vec3>(6,Vec3(0)),
        Stage::Position);
    this->_target_regional_contact_forceCV =
        addCacheVariable(
        "target_regional_contact_force", Vector_<Vec3>(6,Vec3(0)),
        Stage::Position);
    this->_target_regional_contact_momentCV =
        addCacheVariable(
        "target_regional_contact_moment", Vector_<Vec3>(6,Vec3(0)),
        Stage::Position);
    this->_casting_regional_contact_areaCV =
        addCacheVariable(
        "casting_regional_contact_area", Vector(6,0.0), Stage::Position);
    this->_casting_regional_mean_proximityCV =
        addCacheVariable(
        "casting_regional_mean_proximity", Vector(6,0.0), Stage::Position);
    this->_casting_regional_max_proximityCV =
        addCacheVariable(
        "casting_regional_max_proximity", Vector(6,0.0), Stage::Position);
    this->_casting_regional_center_of_proximityCV =
        addCacheVariable(
        "casting_regional_center_of_proximity", Vector_<Vec3>(6, Vec3(0)),
        Stage::Position);
    this->_casting_regional_mean_pressureCV =
        addCacheVariable(
        "casting_regional_mean_pressure", Vector(6,0.0), Stage::Position);
    this->_casting_regional_max_pressureCV =
        addCacheVariable(
        "casting_regional_max_pressure", Vector(6,0.0), Stage::Position);
    this->_casting_regional_center_of_pressureCV =
        addCacheVariable(
        "casting_regional_center_of_pressure", Vector_<Vec3>(6,Vec3(0)),
        Stage::Position);
    this->_casting_regional_contact_forceCV =
        addCacheVariable(
        "casting_regional_contact_force", Vector_<Vec3>(6,Vec3(0)),
        Stage::Position);
    this->_casting_regional_contact_momentCV =
        addCacheVariable(
        "casting_regional_contact_moment", Vector_<Vec3>(6,Vec3(0)),
        Stage::Position);

    //Modeling Options
    //----------------
    addModelingOption("flip_meshes", 1);
}

void Smith2018ArticularContactForce::computeMeshProximity(
    const State& state, const Smith2018ContactMesh& casting_mesh,
    const Smith2018ContactMesh& target_mesh, const std::string& cache_mesh_name) const
{
    SimTK::Vector triangle_proximity;
    computeMeshProximity(state, casting_mesh, target_mesh,
        cache_mesh_name, triangle_proximity);
}

void Smith2018ArticularContactForce::computeMeshProximity(
    const State& state, const Smith2018ContactMesh& casting_mesh,
    const Smith2018ContactMesh& target_mesh, const std::string& cache_mesh_name,
    SimTK::Vector& triangle_proximity) const
{
    // Get Mesh Properties
    Vector_<SimTK::Vec3> tri_cen = casting_mesh.getTriangleCenters();
    Vector_<SimTK::UnitVec3> tri_nor = casting_mesh.getTriangleNormals();

    Transform MeshCtoMeshT = casting_mesh.getMeshFrame().
        findTransformBetween(state, target_mesh.getMeshFrame());

    //Initialize contact variables
    //----------------------------

    //Number of triangles with positive ray intersection tests
    int nActiveTri = 0;

    //Subset of nActiveTri with positive proximity
    int nContactingTri = 0;

    triangle_proximity.resize(casting_mesh.getNumFaces());
    triangle_proximity = 0;

    std::vector<int>& target_tri = (cache_mesh_name == "target") ?
        this->updCacheVariableValue(
            state, this->_target_triangle_previous_contacting_triangleCV) :
        this->updCacheVariableValue(
            state, this->_casting_triangle_previous_contacting_triangleCV);

    //Keep track of triangle collision type for debugging
    int nSameTri = 0;
    int nNeighborTri = 0;
    int nDiffTri = 0;

    //Collision Detection
    //-------------------

    //Loop through all triangles in casting mesh
    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        bool contact_detected = false;
        double distance = 0.0;
        SimTK::Vec3 contact_point;
        SimTK::Vec3 origin = MeshCtoMeshT.shiftFrameStationToBase(tri_cen(i));
        SimTK::UnitVec3 direction(
            MeshCtoMeshT.xformFrameVecToBase(tri_nor(i)));

        //If triangle was in contact in previous timestep, 
        //recheck same contact triangle and neighbors
        if (target_tri[i] >= 0) {
            //same triangle
            if (target_mesh._obb.rayIntersectTri(
                target_mesh.getPolygonalMesh(), origin, -direction,
                target_tri[i], contact_point, distance))
            {
                if (distance >= get_min_proximity() &&
                    distance <= get_max_proximity()) {

                    triangle_proximity(i) = distance;

                    nActiveTri++;
                    nSameTri++;

                    if (triangle_proximity(i) > 0.0) { nContactingTri++; }
                }
                continue;

            }

            //neighboring triangles
            std::set<int> neighborTris =
                target_mesh.getNeighborTris(target_tri[i]);

            for (int neighbor_tri : neighborTris) {
                if (target_mesh._obb.rayIntersectTri(
                    target_mesh.getPolygonalMesh(), origin, -direction,
                    neighbor_tri, contact_point, distance))
                {
                    if (distance >= get_min_proximity() &&
                        distance <= get_max_proximity()) {

                        triangle_proximity(i) = distance;

                        target_tri[i] = neighbor_tri;

                        nActiveTri++;
                        nNeighborTri++;
                        if (triangle_proximity(i) > 0.0) { nContactingTri++; }

                        contact_detected = true;
                        break;
                    }
                }
            }
            if (contact_detected) {
                continue;
            }
        }

        //No luck in rechecking same triangle and neighbors
        //Go through the expensive OBB hierarchy
        int contact_target_tri = -1;

        if (target_mesh.rayIntersectMesh(origin, -direction,
            get_min_proximity(), get_max_proximity(),
            contact_target_tri, contact_point, distance)) {

            target_tri[i] = contact_target_tri;
            triangle_proximity(i) = distance;

            nActiveTri++;
            nDiffTri++;
            if (triangle_proximity(i) > 0.0) { nContactingTri++; }
            continue;
        }

        //Else - triangle is not in contact
        target_tri[i] = -1;
    }

    //Store Contact Info
    if (cache_mesh_name == "casting"){
        this->setCacheVariableValue(state, 
            this->_casting_triangle_proximityCV, triangle_proximity);
        this->setCacheVariableValue(state, 
            this->_casting_triangle_previous_contacting_triangleCV, target_tri);
        this->setCacheVariableValue(state, 
            this->_casting_num_active_trianglesCV, nActiveTri);
        this->setCacheVariableValue(state, 
            this->_casting_num_contacting_trianglesCV, nContactingTri);
        this->setCacheVariableValue(state, 
            this->_casting_num_contacting_triangles_sameCV, nSameTri);
        this->setCacheVariableValue(state, 
            this->_casting_num_contacting_triangles_neighborCV, nNeighborTri);
        this->setCacheVariableValue(state, 
            this->_casting_num_contacting_triangles_differentCV, nDiffTri);
    }
    else {
        this->setCacheVariableValue(state, 
            this->_target_triangle_proximityCV, triangle_proximity);
        this->setCacheVariableValue(state, 
            this->_target_triangle_previous_contacting_triangleCV, target_tri);
        this->setCacheVariableValue(state, 
            this->_target_num_active_trianglesCV, nActiveTri);
        this->setCacheVariableValue(state, 
            this->_target_num_contacting_trianglesCV, nContactingTri);
        this->setCacheVariableValue(state, 
            this->_target_num_contacting_triangles_sameCV, nSameTri);
        this->setCacheVariableValue(state, 
            this->_target_num_contacting_triangles_neighborCV, nNeighborTri);
        this->setCacheVariableValue(state, 
            this->_target_num_contacting_triangles_differentCV, nDiffTri);
    }
}

void Smith2018ArticularContactForce::computeMeshDynamics(
    const State& state, const Smith2018ContactMesh& casting_mesh,
    const Smith2018ContactMesh& target_mesh) const
{
    SimTK::Vector_<SimTK::Vec3> triangle_force;
    SimTK::Vector triangle_pressure;
    SimTK::Vector triangle_energy;

    computeMeshDynamics(
        state, casting_mesh, target_mesh,
        triangle_force, triangle_pressure, triangle_energy);
}

void Smith2018ArticularContactForce::computeMeshDynamics(
    const State& state, const Smith2018ContactMesh& casting_mesh,
    const Smith2018ContactMesh& target_mesh,
    SimTK::Vector_<SimTK::Vec3>& triangle_force,
    SimTK::Vector& triangle_pressure,
    SimTK::Vector& triangle_energy) const
{
    std::string casting_path = getConnectee<Smith2018ContactMesh>
        ("casting_mesh").getAbsolutePathString();

    bool flipped_meshes;
    if (casting_path == casting_mesh.getAbsolutePathString())
    {
        flipped_meshes = false;
    }
    else {
        flipped_meshes = true;
    }

    const Vector& triangle_proximity = (flipped_meshes) ?
        this->getCacheVariableValue(state, 
            this->_target_triangle_proximityCV) :
        this->getCacheVariableValue(state, 
            this->_casting_triangle_proximityCV);

    const std::vector<int>& target_tri = (flipped_meshes) ?
        this->getCacheVariableValue(state,
            this->_target_triangle_previous_contacting_triangleCV) :
        this->getCacheVariableValue(state,
            this->_casting_triangle_previous_contacting_triangleCV);

    const Vector& triangle_area = casting_mesh.getTriangleAreas();

    triangle_pressure.resize(casting_mesh.getNumFaces());
    triangle_pressure = 0;
    triangle_energy.resize(casting_mesh.getNumFaces());
    triangle_energy = 0;

    double hT, hC; //thickness
    double ET, EC; //elastic modulus
    double vT, vC; //poissons ratio

    //Compute Tri Pressure and Potential Energy
    //-----------------------------------------
    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        if (triangle_proximity(i) <= 0) {
            triangle_pressure(i) = 0;
            triangle_energy(i) = 0;
            continue;
        }

        //Material Properties
        hT = target_mesh.getTriangleThickness(target_tri[i]);
        ET = target_mesh.getTriangleElasticModulus(target_tri[i]);
        vT = target_mesh.getTrianglePoissonsRatio(target_tri[i]);
        
        hC = casting_mesh.getTriangleThickness(i);
        EC = casting_mesh.getTriangleElasticModulus(i);
        vC = casting_mesh.getTrianglePoissonsRatio(i);

        //Compute pressure & energy using the lumped contact model
        if (get_use_lumped_contact_model()) {
            double E = (ET + EC) / 2;
            double v = (vT + vC) / 2;
            double h = (hT + hC);

            double K = (1 - v)*E / ((1 + v)*(1 - 2 * v));


            if (get_elastic_foundation_formulation() == "linear") {
                triangle_pressure(i) = K * triangle_proximity(i) / h;
                triangle_energy(i) = 0.5 * triangle_area(i) * K *
                    SimTK::square(triangle_proximity(i)) / h;
                continue;
            }

            if (get_elastic_foundation_formulation() == "nonlinear") {
                triangle_pressure(i) = -K * log(1 - triangle_proximity(i) / h);
                triangle_energy(i) = -triangle_area(i)* K * ((triangle_proximity(i) - h) *
                    log(1 - triangle_proximity(i) / h) - triangle_proximity(i));
                continue;
            }
        }

        //Compute pressure & energy using variable property model

        //linear solution
        double kT = ((1 - vT)*ET) / ((1 + vT)*(1 - 2 * vT)*hT);
        double kC = ((1 - vC)*EC) / ((1 + vC)*(1 - 2 * vC)*hC);

        double linearPressure = (kT*kC) / (kT + kC)*triangle_proximity(i);

        if (get_elastic_foundation_formulation() == "linear") {
            triangle_pressure(i) = linearPressure;

            double depthT = kC / (kT + kC)*triangle_proximity(i);
            double depthC = kT / (kT + kC)*triangle_proximity(i);

            double energyC = 0.5 * triangle_area(i) * kC * SimTK::square(depthC);
            double energyT = 0.5 * triangle_area(i) * kT * SimTK::square(depthT);
            triangle_energy(i) = energyC + energyT;
            continue;
        }

        //nonlinear solution
        else{ //(get_elastic_foundation_formulation() == "nonlinear") 
            double nonlinearPressure = 
                calcTrianglePressureVariableNonlinearModel(
                    triangle_proximity(i),hC,hT,EC,ET,vC,vT,linearPressure);

            triangle_pressure(i) = nonlinearPressure;

            double depthC = hC * (1 - exp(-nonlinearPressure / kC));
            double depthT = hT * (1 - exp(-nonlinearPressure / kT));

            double energyC = -triangle_area(i)* kC * 
                ((depthC - hC)*log(1 - depthC / hC) - depthC);
            double energyT = -triangle_area(i)* kT * 
                ((depthT - hT)*log(1 - depthT / hT) - depthT);
            triangle_energy(i) = energyC + energyT;
            continue;
        }
    }

    //Compute Triangle Forces 
    //-----------------------
    const Vector_<UnitVec3>& triangle_normal = casting_mesh.getTriangleNormals();

    triangle_force.resize(casting_mesh.getNumFaces());
    triangle_force = Vec3(0.0);

    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        for (int j = 0; j < 3; ++j) {
            triangle_force(i)(j) = 
                triangle_pressure(i) * triangle_area(i) * -triangle_normal(i)(j);
        }
    }

    
    if (flipped_meshes) {
        this->setCacheVariableValue(state,
            this->_target_triangle_pressureCV, triangle_pressure);
        this->setCacheVariableValue(state,
            this->_target_triangle_potential_energyCV, triangle_energy);
        this->setCacheVariableValue(state, 
            this->_target_triangle_forceCV, triangle_force);
    }
    else {
        this->setCacheVariableValue(state,
            this->_casting_triangle_pressureCV, triangle_pressure);
        this->setCacheVariableValue(state,
            this->_casting_triangle_potential_energyCV, triangle_energy);
        this->setCacheVariableValue(state, 
            this->_casting_triangle_forceCV, triangle_force);
    }

    return;
}

double Smith2018ArticularContactForce::
    calcTrianglePressureVariableNonlinearModel(double proximity, 
    double casting_thickness, double target_thickness,
    double casting_E, double target_E, double casting_v, double target_v,
    double init_guess) const {
    
    NonlinearContactParams cp;

    cp.dc = proximity;
    cp.h1 = casting_thickness;
    cp.h2 = target_thickness;
    double kC = (1 - casting_v)*casting_E / ((1 + casting_v)*(1 - 2 * casting_v));
    double kT = (1 - target_v)*target_E / ((1 + target_v)*(1 - 2 * target_v));
    cp.k1 = kC;
    cp.k2 = kT;

    int nEqn = 1;
    int nVar = 1;
    double x[1], fvec[1];

    //solution params
    double ftol = 1e-4, xtol = 1e-4, gtol = 0.0;
    int maxfev = 500; //max iterations
    double epsfcn = 0.0;
    double diag[1];
    int mode = 1; //variables scaled internally
    double step_factor = 100;
    int nprint = 0;
    int info;
    int num_func_calls;
    double fjac[1];
    int ldfjac = 1;
    int ipvt[1];
    double qtf[1];
    double wa1[1], wa2[1], wa3[1], wa4[1];

    //initial guess
    x[0] = init_guess;

    //Solve nonlinear equation
    lmdif_C(calcNonlinearPressureResidual, nEqn, nVar, x, fvec,
        ftol, xtol, gtol, maxfev, epsfcn, diag, mode, step_factor,
        nprint, &info, &num_func_calls, fjac, ldfjac, ipvt, qtf,
        wa1, wa2, wa3, wa4, (void*)&cp);

    return x[0];
}

void Smith2018ArticularContactForce::calcNonlinearPressureResidual(
    int nEqn, int nVar, double x[], double fvec[], int *flag2, void *ptr)
{
    NonlinearContactParams * cp = (NonlinearContactParams*)ptr;

    double h1 = cp->h1;
    double h2 = cp->h2;
    double k1 = cp->k1;
    double k2 = cp->k2;
    double dc = cp->dc;

    double P = x[0];

    fvec[0] = h1*(1 - exp(-P / k1)) + h2*(1 - exp(-P / k2)) - dc;

}

void Smith2018ArticularContactForce::computeForce(const State& state,
    Vector_<SpatialVec>& bodyForces,
    Vector& generalizedForces) const
{
    const Smith2018ContactMesh& casting_mesh =
        getConnectee<Smith2018ContactMesh>("casting_mesh");

    const Smith2018ContactMesh& target_mesh =
        getConnectee<Smith2018ContactMesh>("target_mesh");

    //Proximity
    SimTK::Vector casting_triangle_proximity;
    if (!this->isCacheVariableValid(state, this->_casting_triangle_proximityCV)) {
        computeMeshProximity(state, casting_mesh, target_mesh,
            "casting", casting_triangle_proximity);
    }
    else {
        casting_triangle_proximity = 
            this->getCacheVariableValue(state, this->_casting_triangle_proximityCV);
    }

    SimTK::Vector casting_triangle_pressure;
    SimTK::Vector casting_triangle_energy;
    SimTK::Vector_<Vec3> casting_triangle_force;

    //Pressure
    computeMeshDynamics(state, casting_mesh, target_mesh, casting_triangle_force,
        casting_triangle_pressure, casting_triangle_energy);

    //Force
    const PhysicalFrame& target_frame = target_mesh.getMeshFrame();
    const PhysicalFrame& casting_frame = casting_mesh.getMeshFrame();

    const Vector_<Vec3>& triangle_center = casting_mesh.getTriangleCenters();

    Transform T_casting_to_ground = casting_frame.getTransformInGround(state);
    Transform T_casting_to_target =
        casting_frame.findTransformBetween(state, target_frame);

    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        Vec3 casting_force_ground =
            T_casting_to_ground.xformFrameVecToBase(casting_triangle_force(i));

        applyForceToPoint(state, casting_frame, triangle_center(i),
            casting_force_ground, bodyForces);

        Vec3 target_force_ground = -casting_force_ground;
        Vec3 triangle_center_target =
            T_casting_to_target.shiftFrameStationToBase(triangle_center(i));

        applyForceToPoint(state, target_frame, triangle_center_target,
            target_force_ground, bodyForces);
    }

    //Compute Proximity and Pressure for Target mesh (optional for analysis)
    if (getModelingOption(state, "flip_meshes")) {
        SimTK::Vector target_triangle_proximity;

        //target proximity
        if (!this->isCacheVariableValid(state,
            this->_target_triangle_proximityCV)) {
            computeMeshProximity(state, target_mesh, casting_mesh,
                "target", target_triangle_proximity);
        }
        else {
            target_triangle_proximity =
                this->getCacheVariableValue(state,
                    this->_target_triangle_proximityCV);
        }
    
        
        //target pressure
        SimTK::Vector target_triangle_pressure;
        SimTK::Vector_<SimTK::Vec3> target_triangle_force;
        SimTK::Vector target_triangle_energy;

        computeMeshDynamics(state, target_mesh, casting_mesh,
            target_triangle_force, target_triangle_pressure, target_triangle_energy);
            
        /*setCacheVariableValue(state,
            "target.triangle.proximity", target_triangle_proximity);

        setCacheVariableValue(state,
            "target.triangle.pressure", target_triangle_pressure);

        setCacheVariableValue(state,
            "target.triangle.potential_energy", target_triangle_energy);*/
    }
}

//Compute Contact Stats
void Smith2018ArticularContactForce::realizeContactMetricCaches(
    const SimTK::State& state) const
{
    const Smith2018ContactMesh& casting_mesh =
        getConnectee<Smith2018ContactMesh>("casting_mesh");

    const Smith2018ContactMesh& target_mesh =
        getConnectee<Smith2018ContactMesh>("target_mesh");

    const SimTK::Vector& casting_triangle_proximity = 
        this->getCacheVariableValue(
            state, this->_casting_triangle_proximityCV);

    const SimTK::Vector& casting_triangle_pressure = 
        this->getCacheVariableValue(
            state, this->_casting_triangle_pressureCV);

    std::vector<int> casting_faces;
    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        casting_faces.push_back(i);
    }

    auto stats = computeContactStats(casting_mesh, casting_triangle_proximity,
        casting_triangle_pressure,casting_faces);
    
    this->setCacheVariableValue(state, 
        this->_casting_total_contact_areaCV, stats.contact_area);
    this->setCacheVariableValue(state, 
        this->_casting_total_mean_proximityCV, stats.mean_proximity);
    this->setCacheVariableValue(state, 
        this->_casting_total_max_proximityCV, stats.max_proximity);
    this->setCacheVariableValue(state, 
        this->_casting_total_center_of_proximityCV, stats.center_of_proximity);
    this->setCacheVariableValue(state, 
        this->_casting_total_mean_pressureCV, stats.mean_pressure);
    this->setCacheVariableValue(state, 
        this->_casting_total_max_pressureCV, stats.max_pressure);
    this->setCacheVariableValue(state, 
        this->_casting_total_center_of_pressureCV, stats.center_of_pressure);
    this->setCacheVariableValue(state, 
        this->_casting_total_contact_forceCV, stats.contact_force);
    this->setCacheVariableValue(state, 
        this->_casting_total_contact_momentCV, stats.contact_moment);
 

    //Target mesh computations (not used in applied contact force calculation)
    SimTK::Vector target_triangle_proximity;
    SimTK::Vector target_triangle_pressure;

    if (getModelingOption(state, "flip_meshes")) {
        
        target_triangle_proximity = 
            this->getCacheVariableValue(state, 
                this->_target_triangle_proximityCV);

        target_triangle_pressure = 
            this->getCacheVariableValue(state, 
                this->_target_triangle_pressureCV);

        //target contact stats
        std::vector<int> Tcasting_faces;
        for (int i = 0; i < target_mesh.getNumFaces(); ++i) {
            Tcasting_faces.push_back(i);
        }

        auto Tstats = computeContactStats(target_mesh, target_triangle_proximity,
            target_triangle_pressure,Tcasting_faces);
    
        this->setCacheVariableValue(state,
            this->_target_total_contact_areaCV, Tstats.contact_area);
        this->setCacheVariableValue(state,
            this->_target_total_mean_proximityCV, Tstats.mean_proximity);
        this->setCacheVariableValue(state,
            this->_target_total_max_proximityCV, Tstats.max_proximity);
        this->setCacheVariableValue(state,
            this->_target_total_center_of_proximityCV, Tstats.center_of_proximity);
        this->setCacheVariableValue(state,
            this->_target_total_mean_pressureCV, Tstats.mean_pressure);
        this->setCacheVariableValue(state,
            this->_target_total_max_pressureCV, Tstats.max_pressure);
        this->setCacheVariableValue(state,
            this->_target_total_center_of_pressureCV, Tstats.center_of_pressure);
        this->setCacheVariableValue(state,
            this->_target_total_contact_forceCV, Tstats.contact_force);
        this->setCacheVariableValue(state,
            this->_target_total_contact_momentCV, Tstats.contact_moment);
    }

    //regional casting stats
    std::vector<std::vector<int>> casting_region_tri_ind = 
        casting_mesh.getRegionalTriangleIndices();

    SimTK::Vector reg_contact_area(6, 0.0);
    SimTK::Vector reg_mean_proximity(6, 0.0);
    SimTK::Vector reg_max_proximity(6, 0.0);
    SimTK::Vector_<SimTK::Vec3> reg_COPrx(6, SimTK::Vec3(0));
    SimTK::Vector reg_mean_pressure(6, 0.0);
    SimTK::Vector reg_max_pressure(6, 0.0);
    SimTK::Vector_<SimTK::Vec3> reg_COP(6, SimTK::Vec3(0));
    SimTK::Vector_<SimTK::Vec3> reg_contact_force(6, SimTK::Vec3(0));
    SimTK::Vector_<SimTK::Vec3> reg_contact_moment(6, SimTK::Vec3(0));

    for (int i = 0; i < 6; ++i) {
        auto stats = computeContactStats(casting_mesh, 
            casting_triangle_proximity, casting_triangle_pressure,
            casting_region_tri_ind[i]);

        reg_contact_area(i) = stats.contact_area;
        reg_mean_proximity(i) = stats.mean_proximity;
        reg_max_proximity(i) = stats.max_proximity;
        reg_COPrx(i) = stats.center_of_proximity;
        reg_mean_pressure(i) = stats.mean_pressure;
        reg_max_pressure(i) = stats.max_pressure;
        reg_COP(i) = stats.center_of_pressure;
        reg_contact_force(i) = stats.contact_force;
        reg_contact_moment(i) = stats.contact_moment;
    }
    this->setCacheVariableValue(state,
        this->_casting_regional_contact_areaCV, reg_contact_area);
    this->setCacheVariableValue(state,
        this->_casting_regional_mean_proximityCV, reg_mean_proximity);
    this->setCacheVariableValue(state,
        this->_casting_regional_max_proximityCV, reg_max_proximity);
    this->setCacheVariableValue(state,
        this->_casting_regional_center_of_proximityCV, reg_COPrx);
    this->setCacheVariableValue(state,
        this->_casting_regional_mean_pressureCV, reg_mean_pressure);
    this->setCacheVariableValue(state,
        this->_casting_regional_max_pressureCV, reg_max_pressure);
    this->setCacheVariableValue(state,
        this->_casting_regional_center_of_pressureCV, reg_COP);
    this->setCacheVariableValue(state,
        this->_casting_regional_contact_forceCV, reg_contact_force);
    this->setCacheVariableValue(state,
        this->_casting_regional_contact_momentCV, reg_contact_moment);

    //target
    if (getModelingOption(state, "flip_meshes")) {
        
        std::vector<std::vector<int>> target_region_tri_ind =
            target_mesh.getRegionalTriangleIndices();

        for (int i = 0; i < 6; ++i) {
            auto stats = computeContactStats(target_mesh, 
                target_triangle_proximity, target_triangle_pressure, target_region_tri_ind[i]);

            reg_contact_area(i) = stats.contact_area;
            reg_mean_proximity(i) = stats.mean_proximity;
            reg_max_proximity(i) = stats.max_proximity;
            reg_COPrx(i) = stats.center_of_proximity;
            reg_mean_pressure(i) = stats.mean_pressure;
            reg_max_pressure(i) = stats.max_pressure;
            reg_COP(i) = stats.center_of_pressure;
            reg_contact_force(i) = stats.contact_force;
            reg_contact_moment(i) = stats.contact_moment;
        }

        this->setCacheVariableValue(state,
            this->_target_regional_contact_areaCV, reg_contact_area);
        this->setCacheVariableValue(state,
            this->_target_regional_mean_proximityCV, reg_mean_proximity);
        this->setCacheVariableValue(state,
            this->_target_regional_max_proximityCV, reg_max_proximity);
        this->setCacheVariableValue(state,
            this->_target_regional_center_of_proximityCV, reg_COPrx);
        this->setCacheVariableValue(state,
            this->_target_regional_mean_pressureCV, reg_mean_pressure);
        this->setCacheVariableValue(state,
            this->_target_regional_max_pressureCV, reg_max_pressure);
        this->setCacheVariableValue(state,
            this->_target_regional_center_of_pressureCV, reg_COP);
        this->setCacheVariableValue(state,
            this->_target_regional_contact_forceCV, reg_contact_force);
        this->setCacheVariableValue(state,
            this->_target_regional_contact_momentCV, reg_contact_moment);
    }
}

double Smith2018ArticularContactForce::
computePotentialEnergy(const SimTK::State& state) const
{
    if (!this->isCacheVariableValid(state, this->_casting_triangle_potential_energyCV)) {
        const Smith2018ContactMesh& casting_mesh =
            getConnectee<Smith2018ContactMesh>("casting_mesh");

        const Smith2018ContactMesh& target_mesh =
            getConnectee<Smith2018ContactMesh>("target_mesh");

        computeMeshDynamics(state,target_mesh, casting_mesh);
    }
    const SimTK::Vector& triangle_energy = this->getCacheVariableValue(
        state, this->_casting_triangle_potential_energyCV);
    return triangle_energy.sum();
}

Vec3 Smith2018ArticularContactForce::
computeContactForceVector(double pressure, double area, Vec3 normal) const
{
    Vec3 force = normal * pressure * area;
    return force;
}

Vec3 Smith2018ArticularContactForce::
computeContactMomentVector(double pressure, double area, Vec3 normal,
    Vec3 center) const
{
    Vec3 force = normal * pressure * area;
    Vec3 moment = SimTK::cross(center,force);
    return moment;
}

Smith2018ArticularContactForce::ContactStats
Smith2018ArticularContactForce::computeContactStats(
    const Smith2018ContactMesh& mesh,
    const SimTK::Vector& total_triangle_proximity,
    const SimTK::Vector& total_triangle_pressure,
    const std::vector<int>& triIndices) const
{
    Smith2018ArticularContactForce::ContactStats stats;

    int nTri = static_cast<int>(triIndices.size());

    SimTK::Vector triangle_proximity(nTri);
    SimTK::Vector triangle_pressure(nTri);

    const SimTK::Vector& total_triangle_area = mesh.getTriangleAreas();
    SimTK::Vector triangle_area(nTri);

    const SimTK::Vector_<UnitVec3>& total_triangle_normal = mesh.getTriangleNormals();
    SimTK::Vector_<UnitVec3> triangle_normal(nTri);

    const SimTK::Vector_<Vec3>& total_triangle_center = mesh.getTriangleCenters();
    SimTK::Vector_<Vec3> triangle_center(nTri);
    int nContactingTri = 0;

    for (int i = 0; i < nTri; ++i) {
        triangle_area(i) = total_triangle_area(triIndices[i]);
        triangle_normal(i) = total_triangle_normal(triIndices[i]);
        triangle_proximity(i) = total_triangle_proximity(triIndices[i]);
        triangle_pressure(i) = total_triangle_pressure(triIndices[i]);
        triangle_center(i) = total_triangle_center(triIndices[i]);

        if(triangle_pressure(i) > 0.0){
            nContactingTri++;
        }
    }
    
    SimTK::Vector triangle_cenX(nTri);
    SimTK::Vector triangle_cenY(nTri);
    SimTK::Vector triangle_cenZ(nTri);

    for (int i = 0; i < nTri; ++i) {
        triangle_cenX(i) = triangle_center(i)(0);
        triangle_cenY(i) = triangle_center(i)(1);
        triangle_cenZ(i) = triangle_center(i)(2);
    }

    //Mean Pressure
    if (nContactingTri == 0) {
        stats.mean_pressure = 0.0;
        stats.mean_proximity = 0.0;
    }
    else
    {
        stats.mean_pressure = triangle_pressure.sum() / nContactingTri;
        stats.mean_proximity = triangle_proximity.sum() / nContactingTri;
    }
    //Max Pressure
    stats.max_pressure = triangle_pressure.normInf();
    stats.max_proximity = triangle_proximity.normInf();

    //Contact Area
    double contact_area = 0.0;

    for (int i = 0; i < nTri; i++) {
        if (triangle_pressure(i) > 0.0) {
            contact_area += triangle_area(i);
        }
    }

    stats.contact_area = contact_area;

    //Center of Proximity
    SimTK::Vector num_prx = triangle_proximity.elementwiseMultiply(triangle_area);
    SimTK::Vector den_prx = triangle_proximity.elementwiseMultiply(triangle_area);
    double denVal_prx = den_prx.sum();

    SimTK::Vector xNum_prx = num_prx.elementwiseMultiply(triangle_cenX);
    double xNumVal_prx = xNum_prx.sum();
    double COPrx_x = xNumVal_prx / denVal_prx;

    SimTK::Vector yNum_prx = num_prx.elementwiseMultiply(triangle_cenY);
    double yNumVal_prx = yNum_prx.sum();
    double COPrx_y = yNumVal_prx / denVal_prx;

    SimTK::Vector zNum_prx = num_prx.elementwiseMultiply(triangle_cenZ);
    double zNumVal_prx = zNum_prx.sum();
    double COPrx_z = zNumVal_prx / denVal_prx;

    if (denVal_prx == 0) {
        stats.center_of_proximity(0) = -1;
        stats.center_of_proximity(1) = -1;
        stats.center_of_proximity(2) = -1;
    }
    else {
        stats.center_of_proximity(0) = COPrx_x;
        stats.center_of_proximity(1) = COPrx_y;
        stats.center_of_proximity(2) = COPrx_z;
    }
    //Center of Pressure
    SimTK::Vector num = triangle_pressure.elementwiseMultiply(triangle_area);
    SimTK::Vector den = triangle_pressure.elementwiseMultiply(triangle_area);
    double denVal = den.sum();

    SimTK::Vector xNum = num.elementwiseMultiply(triangle_cenX);
    double xNumVal = xNum.sum();
    double COPx = xNumVal / denVal;

    SimTK::Vector yNum = num.elementwiseMultiply(triangle_cenY);
    double yNumVal = yNum.sum();
    double COPy = yNumVal / denVal;

    SimTK::Vector zNum = num.elementwiseMultiply(triangle_cenZ);
    double zNumVal = zNum.sum();
    double COPz = zNumVal / denVal;

    if (denVal_prx == 0) {
        stats.center_of_pressure(0) = -1;
        stats.center_of_pressure(1) = -1;
        stats.center_of_pressure(2) = -1;
    }
    else {
        stats.center_of_pressure(0) = COPx;
        stats.center_of_pressure(1) = COPy;
        stats.center_of_pressure(2) = COPz;
    }
    //Contact Force
    stats.contact_force = 0.0;
    stats.contact_moment = 0.0;
    

    for (int i = 0; i < nTri; ++i) {
        stats.contact_force += computeContactForceVector(
            triangle_pressure(i), triangle_area(i), -triangle_normal(i));
        stats.contact_moment += computeContactMomentVector(
            triangle_pressure(i), triangle_area(i), -triangle_normal(i),
            triangle_center(i));
    }

    return stats;

}

OpenSim::Array<std::string> Smith2018ArticularContactForce::
getRecordLabels() const {
    // Can only return casting_mesh computations because target_mesh is 
    // not used in computation of force, so if the flip_meshes ModelingOption
    // is not set, target_mesh values will not be valid

    const std::string& c_mesh = getSocket<Smith2018ContactMesh>(
        "casting_mesh").getConnectee().getName();

    const std::string& t_mesh = getSocket<Smith2018ContactMesh>(
        "target_mesh").getConnectee().getName();

    OpenSim::Array<std::string> labels("");

    labels.append(getName() + "." + c_mesh + ".total.contact_area");
    labels.append(getName() + "." + c_mesh + ".total.mean_proximity");
    labels.append(getName() + "." + c_mesh + ".total.max_proximity"); 
    labels.append(getName() + "." + c_mesh + ".total.center_of_proximity_x");
    labels.append(getName() + "." + c_mesh + ".total.center_of_proximity_y");
    labels.append(getName() + "." + c_mesh + ".total.center_of_proximity_z");
    labels.append(getName() + "." + c_mesh + ".total.mean_pressure");
    labels.append(getName() + "." + c_mesh + ".total.max_pressure");
    labels.append(getName() + "." + c_mesh + ".total.center_of_pressure_x");
    labels.append(getName() + "." + c_mesh + ".total.center_of_pressure_y");
    labels.append(getName() + "." + c_mesh + ".total.center_of_pressure_z");
    labels.append(getName() + "." + c_mesh + ".total.contact_force_x");
    labels.append(getName() + "." + c_mesh + ".total.contact_force_y");
    labels.append(getName() + "." + c_mesh + ".total.contact_force_z");
    labels.append(getName() + "." + c_mesh + ".total.contact_moment_x");
    labels.append(getName() + "." + c_mesh + ".total.contact_moment_y");
    labels.append(getName() + "." + c_mesh + ".total.contact_moment_z");

    labels.append(getName() + "." + c_mesh + ".regional.contact_area_0");
    labels.append(getName() + "." + c_mesh + ".regional.contact_area_1");
    labels.append(getName() + "." + c_mesh + ".regional.contact_area_2");
    labels.append(getName() + "." + c_mesh + ".regional.contact_area_3");
    labels.append(getName() + "." + c_mesh + ".regional.contact_area_4");
    labels.append(getName() + "." + c_mesh + ".regional.contact_area_5");

    labels.append(getName() + "." + c_mesh + ".regional.mean_proximity_0");
    labels.append(getName() + "." + c_mesh + ".regional.mean_proximity_1");
    labels.append(getName() + "." + c_mesh + ".regional.mean_proximity_2");
    labels.append(getName() + "." + c_mesh + ".regional.mean_proximity_3");
    labels.append(getName() + "." + c_mesh + ".regional.mean_proximity_4");
    labels.append(getName() + "." + c_mesh + ".regional.mean_proximity_5");

    labels.append(getName() + "." + c_mesh + ".regional.max_proximity_0");
    labels.append(getName() + "." + c_mesh + ".regional.max_proximity_1");
    labels.append(getName() + "." + c_mesh + ".regional.max_proximity_2");
    labels.append(getName() + "." + c_mesh + ".regional.max_proximity_3");
    labels.append(getName() + "." + c_mesh + ".regional.max_proximity_4");
    labels.append(getName() + "." + c_mesh + ".regional.max_proximity_5");

    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_0_x");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_0_y");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_0_z");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_1_x");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_1_y");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_1_z");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_2_x");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_2_y");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_2_z");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_3_x");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_3_y");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_3_z");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_4_x");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_4_y");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_4_z");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_5_x");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_5_y");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_proximity_5_z");

    labels.append(getName() + "." + c_mesh + ".regional.mean_pressure_0");
    labels.append(getName() + "." + c_mesh + ".regional.mean_pressure_1");
    labels.append(getName() + "." + c_mesh + ".regional.mean_pressure_2");
    labels.append(getName() + "." + c_mesh + ".regional.mean_pressure_3");
    labels.append(getName() + "." + c_mesh + ".regional.mean_pressure_4");
    labels.append(getName() + "." + c_mesh + ".regional.mean_pressure_5");

    labels.append(getName() + "." + c_mesh + ".regional.max_pressure_0");
    labels.append(getName() + "." + c_mesh + ".regional.max_pressure_1");
    labels.append(getName() + "." + c_mesh + ".regional.max_pressure_2");
    labels.append(getName() + "." + c_mesh + ".regional.max_pressure_3");
    labels.append(getName() + "." + c_mesh + ".regional.max_pressure_4");
    labels.append(getName() + "." + c_mesh + ".regional.max_pressure_5");

    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_0_x");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_0_y");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_0_z");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_1_x");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_1_y");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_1_z");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_2_x");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_2_y");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_2_z");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_3_x");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_3_y");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_3_z");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_4_x");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_4_y");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_4_z");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_5_x");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_5_y");
    labels.append(getName() + "." + c_mesh + ".regional.center_of_pressure_5_z");

    labels.append(getName() + "." + c_mesh + ".regional.contact_force_0_x");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_0_y");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_0_z");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_1_x");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_1_y");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_1_z");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_2_x");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_2_y");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_2_z");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_3_x");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_3_y");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_3_z");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_4_x");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_4_y");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_4_z");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_5_x");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_5_y");
    labels.append(getName() + "." + c_mesh + ".regional.contact_force_5_z");
    
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_0_x");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_0_y");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_0_z");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_1_x");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_1_y");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_1_z");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_2_x");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_2_y");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_2_z");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_3_x");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_3_y");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_3_z");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_4_x");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_4_y");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_4_z");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_5_x");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_5_y");
    labels.append(getName() + "." + c_mesh + ".regional.contact_moment_5_z");
    
    labels.append(getName() + "." + t_mesh + ".total.contact_area");
    labels.append(getName() + "." + t_mesh + ".total.mean_proximity");
    labels.append(getName() + "." + t_mesh + ".total.max_proximity"); 
    labels.append(getName() + "." + t_mesh + ".total.center_of_proximity_x");
    labels.append(getName() + "." + t_mesh + ".total.center_of_proximity_y");
    labels.append(getName() + "." + t_mesh + ".total.center_of_proximity_z");
    labels.append(getName() + "." + t_mesh + ".total.mean_pressure");
    labels.append(getName() + "." + t_mesh + ".total.max_pressure");
    labels.append(getName() + "." + t_mesh + ".total.center_of_pressure_x");
    labels.append(getName() + "." + t_mesh + ".total.center_of_pressure_y");
    labels.append(getName() + "." + t_mesh + ".total.center_of_pressure_z");
    labels.append(getName() + "." + t_mesh + ".total.contact_force_x");
    labels.append(getName() + "." + t_mesh + ".total.contact_force_y");
    labels.append(getName() + "." + t_mesh + ".total.contact_force_z");
    labels.append(getName() + "." + t_mesh + ".total.contact_moment_x");
    labels.append(getName() + "." + t_mesh + ".total.contact_moment_y");
    labels.append(getName() + "." + t_mesh + ".total.contact_moment_z");

    labels.append(getName() + "." + t_mesh + ".regional.contact_area_0");
    labels.append(getName() + "." + t_mesh + ".regional.contact_area_1");
    labels.append(getName() + "." + t_mesh + ".regional.contact_area_2");
    labels.append(getName() + "." + t_mesh + ".regional.contact_area_3");
    labels.append(getName() + "." + t_mesh + ".regional.contact_area_4");
    labels.append(getName() + "." + t_mesh + ".regional.contact_area_5");

    labels.append(getName() + "." + t_mesh + ".regional.mean_proximity_0");
    labels.append(getName() + "." + t_mesh + ".regional.mean_proximity_1");
    labels.append(getName() + "." + t_mesh + ".regional.mean_proximity_2");
    labels.append(getName() + "." + t_mesh + ".regional.mean_proximity_3");
    labels.append(getName() + "." + t_mesh + ".regional.mean_proximity_4");
    labels.append(getName() + "." + t_mesh + ".regional.mean_proximity_5");

    labels.append(getName() + "." + t_mesh + ".regional.max_proximity_0");
    labels.append(getName() + "." + t_mesh + ".regional.max_proximity_1");
    labels.append(getName() + "." + t_mesh + ".regional.max_proximity_2");
    labels.append(getName() + "." + t_mesh + ".regional.max_proximity_3");
    labels.append(getName() + "." + t_mesh + ".regional.max_proximity_4");
    labels.append(getName() + "." + t_mesh + ".regional.max_proximity_5");

    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_0_x");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_0_y");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_0_z");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_1_x");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_1_y");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_1_z");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_2_x");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_2_y");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_2_z");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_3_x");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_3_y");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_3_z");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_4_x");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_4_y");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_4_z");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_5_x");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_5_y");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_proximity_5_z");

    labels.append(getName() + "." + t_mesh + ".regional.mean_pressure_0");
    labels.append(getName() + "." + t_mesh + ".regional.mean_pressure_1");
    labels.append(getName() + "." + t_mesh + ".regional.mean_pressure_2");
    labels.append(getName() + "." + t_mesh + ".regional.mean_pressure_3");
    labels.append(getName() + "." + t_mesh + ".regional.mean_pressure_4");
    labels.append(getName() + "." + t_mesh + ".regional.mean_pressure_5");

    labels.append(getName() + "." + t_mesh + ".regional.max_pressure_0");
    labels.append(getName() + "." + t_mesh + ".regional.max_pressure_1");
    labels.append(getName() + "." + t_mesh + ".regional.max_pressure_2");
    labels.append(getName() + "." + t_mesh + ".regional.max_pressure_3");
    labels.append(getName() + "." + t_mesh + ".regional.max_pressure_4");
    labels.append(getName() + "." + t_mesh + ".regional.max_pressure_5");

    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_0_x");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_0_y");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_0_z");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_1_x");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_1_y");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_1_z");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_2_x");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_2_y");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_2_z");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_3_x");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_3_y");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_3_z");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_4_x");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_4_y");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_4_z");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_5_x");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_5_y");
    labels.append(getName() + "." + t_mesh + ".regional.center_of_pressure_5_z");

    labels.append(getName() + "." + t_mesh + ".regional.contact_force_0_x");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_0_y");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_0_z");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_1_x");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_1_y");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_1_z");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_2_x");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_2_y");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_2_z");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_3_x");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_3_y");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_3_z");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_4_x");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_4_y");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_4_z");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_5_x");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_5_y");
    labels.append(getName() + "." + t_mesh + ".regional.contact_force_5_z");
    
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_0_x");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_0_y");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_0_z");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_1_x");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_1_y");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_1_z");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_2_x");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_2_y");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_2_z");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_3_x");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_3_y");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_3_z");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_4_x");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_4_y");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_4_z");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_5_x");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_5_y");
    labels.append(getName() + "." + t_mesh + ".regional.contact_moment_5_z");
    
    return labels;
}


OpenSim::Array<double> Smith2018ArticularContactForce::
getRecordValues(const SimTK::State& state) const {
    
    if (!this->isCacheVariableValid(state, _casting_total_contact_areaCV)) {
        realizeContactMetricCaches(state);
    }

    //Total
    const double& casting_contact_area = this->getCacheVariableValue(state,
        this->_casting_total_contact_areaCV);

    const double& casting_mean_proximity = this->getCacheVariableValue(state, 
        this->_casting_total_mean_proximityCV);

    const double& casting_max_proximity = this->getCacheVariableValue(state, 
        this->_casting_total_max_proximityCV);

    const SimTK::Vec3& casting_center_of_proximity = 
        this->getCacheVariableValue(state, 
            this->_casting_total_center_of_proximityCV);

    const double& casting_mean_pressure = this->getCacheVariableValue(state, 
        this->_casting_total_mean_pressureCV);

    const double& casting_max_pressure = this->getCacheVariableValue(state, 
        this->_casting_total_max_pressureCV);

    const SimTK::Vec3& casting_center_of_pressure = 
        this->getCacheVariableValue(state, 
            this->_casting_total_center_of_pressureCV);

    const SimTK::Vec3& casting_contact_force = this->getCacheVariableValue(
        state, this->_casting_total_contact_forceCV);

    const SimTK::Vec3& casting_contact_moment =  this->getCacheVariableValue(
        state, this->_casting_total_contact_momentCV);

    // Regional
    const SimTK::Vector& casting_regional_contact_area = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_contact_areaCV);

    const SimTK::Vector& casting_regional_mean_proximity = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_mean_proximityCV);

    const SimTK::Vector& casting_regional_max_proximity = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_max_proximityCV);

    const SimTK::Vector_<SimTK::Vec3>& casting_regional_center_of_proximity = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_center_of_proximityCV);
    
    const SimTK::Vector& casting_regional_mean_pressure = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_mean_pressureCV);

    const SimTK::Vector& casting_regional_max_pressure = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_max_pressureCV);

    const SimTK::Vector_<SimTK::Vec3>& casting_regional_center_of_pressure = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_center_of_pressureCV);

    const SimTK::Vector_<SimTK::Vec3>& casting_regional_contact_force = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_contact_forceCV);

    const SimTK::Vector_<SimTK::Vec3>& casting_regional_contact_moment = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_contact_momentCV);

    OpenSim::Array<double> values;
    
    values.append(casting_contact_area);
    values.append(casting_mean_proximity);
    values.append(casting_max_proximity); 
    values.append(casting_center_of_proximity(0));
    values.append(casting_center_of_proximity(1));
    values.append(casting_center_of_proximity(2));
    values.append(casting_mean_pressure);
    values.append(casting_max_pressure);    
    values.append(casting_center_of_pressure(0));
    values.append(casting_center_of_pressure(1));
    values.append(casting_center_of_pressure(2));
    values.append(casting_contact_force(0));
    values.append(casting_contact_force(1));
    values.append(casting_contact_force(2));
    values.append(casting_contact_moment(0));
    values.append(casting_contact_moment(1));
    values.append(casting_contact_moment(2));

    values.append(casting_regional_contact_area(0));
    values.append(casting_regional_contact_area(1));
    values.append(casting_regional_contact_area(2));
    values.append(casting_regional_contact_area(3));
    values.append(casting_regional_contact_area(4));
    values.append(casting_regional_contact_area(5));

    values.append(casting_regional_mean_proximity(0));
    values.append(casting_regional_mean_proximity(1));
    values.append(casting_regional_mean_proximity(2));
    values.append(casting_regional_mean_proximity(3));
    values.append(casting_regional_mean_proximity(4));
    values.append(casting_regional_mean_proximity(5));

    values.append(casting_regional_max_proximity(0));
    values.append(casting_regional_max_proximity(1));
    values.append(casting_regional_max_proximity(2));
    values.append(casting_regional_max_proximity(3));
    values.append(casting_regional_max_proximity(4));
    values.append(casting_regional_max_proximity(5));

    values.append(casting_regional_center_of_proximity(0)(0));
    values.append(casting_regional_center_of_proximity(0)(1));
    values.append(casting_regional_center_of_proximity(0)(2));
    values.append(casting_regional_center_of_proximity(1)(0));
    values.append(casting_regional_center_of_proximity(1)(1));
    values.append(casting_regional_center_of_proximity(1)(2));
    values.append(casting_regional_center_of_proximity(2)(0));
    values.append(casting_regional_center_of_proximity(2)(1));
    values.append(casting_regional_center_of_proximity(2)(2));
    values.append(casting_regional_center_of_proximity(3)(0));
    values.append(casting_regional_center_of_proximity(3)(1));
    values.append(casting_regional_center_of_proximity(3)(2));
    values.append(casting_regional_center_of_proximity(4)(0));
    values.append(casting_regional_center_of_proximity(4)(1));
    values.append(casting_regional_center_of_proximity(4)(2));
    values.append(casting_regional_center_of_proximity(5)(0));
    values.append(casting_regional_center_of_proximity(5)(1));
    values.append(casting_regional_center_of_proximity(5)(2));

    values.append(casting_regional_mean_pressure(0));
    values.append(casting_regional_mean_pressure(1));
    values.append(casting_regional_mean_pressure(2));
    values.append(casting_regional_mean_pressure(3));
    values.append(casting_regional_mean_pressure(4));
    values.append(casting_regional_mean_pressure(5));

    values.append(casting_regional_max_pressure(0));
    values.append(casting_regional_max_pressure(1));
    values.append(casting_regional_max_pressure(2));
    values.append(casting_regional_max_pressure(3));
    values.append(casting_regional_max_pressure(4));
    values.append(casting_regional_max_pressure(5));

    values.append(casting_regional_center_of_pressure(0)(0));
    values.append(casting_regional_center_of_pressure(0)(1));
    values.append(casting_regional_center_of_pressure(0)(2));
    values.append(casting_regional_center_of_pressure(1)(0));
    values.append(casting_regional_center_of_pressure(1)(1));
    values.append(casting_regional_center_of_pressure(1)(2));
    values.append(casting_regional_center_of_pressure(2)(0));
    values.append(casting_regional_center_of_pressure(2)(1));
    values.append(casting_regional_center_of_pressure(2)(2));
    values.append(casting_regional_center_of_pressure(3)(0));
    values.append(casting_regional_center_of_pressure(3)(1));
    values.append(casting_regional_center_of_pressure(3)(2));
    values.append(casting_regional_center_of_pressure(4)(0));
    values.append(casting_regional_center_of_pressure(4)(1));
    values.append(casting_regional_center_of_pressure(4)(2));
    values.append(casting_regional_center_of_pressure(5)(0));
    values.append(casting_regional_center_of_pressure(5)(1));
    values.append(casting_regional_center_of_pressure(5)(2));

    values.append(casting_regional_contact_force(0)(0));
    values.append(casting_regional_contact_force(0)(1));
    values.append(casting_regional_contact_force(0)(2));
    values.append(casting_regional_contact_force(1)(0));
    values.append(casting_regional_contact_force(1)(1));
    values.append(casting_regional_contact_force(1)(2));
    values.append(casting_regional_contact_force(2)(0));
    values.append(casting_regional_contact_force(2)(1));
    values.append(casting_regional_contact_force(2)(2));
    values.append(casting_regional_contact_force(3)(0));
    values.append(casting_regional_contact_force(3)(1));
    values.append(casting_regional_contact_force(3)(2));
    values.append(casting_regional_contact_force(4)(0));
    values.append(casting_regional_contact_force(4)(1));
    values.append(casting_regional_contact_force(4)(2));
    values.append(casting_regional_contact_force(5)(0));
    values.append(casting_regional_contact_force(5)(1));
    values.append(casting_regional_contact_force(5)(2));

    values.append(casting_regional_contact_moment(0)(0));
    values.append(casting_regional_contact_moment(0)(1));
    values.append(casting_regional_contact_moment(0)(2));
    values.append(casting_regional_contact_moment(1)(0));
    values.append(casting_regional_contact_moment(1)(1));
    values.append(casting_regional_contact_moment(1)(2));
    values.append(casting_regional_contact_moment(2)(0));
    values.append(casting_regional_contact_moment(2)(1));
    values.append(casting_regional_contact_moment(2)(2));
    values.append(casting_regional_contact_moment(3)(0));
    values.append(casting_regional_contact_moment(3)(1));
    values.append(casting_regional_contact_moment(3)(2));
    values.append(casting_regional_contact_moment(4)(0));
    values.append(casting_regional_contact_moment(4)(1));
    values.append(casting_regional_contact_moment(4)(2));
    values.append(casting_regional_contact_moment(5)(0));
    values.append(casting_regional_contact_moment(5)(1));
    values.append(casting_regional_contact_moment(5)(2));

    
    if (getModelingOption(state, "flip_meshes")) {
        //Total
        const double& target_contact_area = this->getCacheVariableValue(
            state, this->_target_total_contact_areaCV);

        const double& target_mean_proximity = this->getCacheVariableValue(
            state, this->_target_total_mean_proximityCV);

        const double& target_max_proximity = this->getCacheVariableValue(
            state, this->_target_total_max_proximityCV);

        const SimTK::Vec3& target_center_of_proximity = 
            this->getCacheVariableValue(state,
               this->_target_total_center_of_proximityCV);

        const double& target_mean_pressure = this->getCacheVariableValue(
            state, this->_target_total_mean_pressureCV);

        const double& target_max_pressure = this->getCacheVariableValue(
            state, this->_target_total_max_pressureCV);

        const SimTK::Vec3& target_center_of_pressure = 
            this->getCacheVariableValue(
                state, this->_target_total_center_of_pressureCV);

        const SimTK::Vec3& target_contact_force = 
            this->getCacheVariableValue(state,
                this->_target_total_contact_forceCV);

        const SimTK::Vec3& target_contact_moment = 
            this->getCacheVariableValue(state, 
                this->_target_total_contact_momentCV);
        // Regional
        const SimTK::Vector& target_regional_contact_area = 
            this->getCacheVariableValue(state,
                this->_target_regional_contact_areaCV);

        const SimTK::Vector& target_regional_mean_proximity = 
            this->getCacheVariableValue(state,
                this->_target_regional_mean_proximityCV);

        const SimTK::Vector& target_regional_max_proximity = 
            this->getCacheVariableValue(state, 
                this->_target_regional_max_proximityCV);

        const SimTK::Vector_<SimTK::Vec3>& target_regional_center_of_proximity = 
            this->getCacheVariableValue(state, 
                this->_target_regional_center_of_proximityCV);
    
        const SimTK::Vector& target_regional_mean_pressure = 
            this->getCacheVariableValue(state, 
                this->_target_regional_mean_pressureCV);

        const SimTK::Vector& target_regional_max_pressure = 
            this->getCacheVariableValue(state, 
                this->_target_regional_max_pressureCV);

        const SimTK::Vector_<SimTK::Vec3>& target_regional_center_of_pressure = 
            this->getCacheVariableValue(state, 
                this->_target_regional_center_of_pressureCV);

        const SimTK::Vector_<SimTK::Vec3>& target_regional_contact_force = 
            this->getCacheVariableValue(state, 
                this->_target_regional_contact_forceCV);

        const SimTK::Vector_<SimTK::Vec3>& target_regional_contact_moment = 
            this->getCacheVariableValue(state, 
                this->_target_regional_contact_momentCV);
    
        values.append(target_contact_area);
        values.append(target_mean_proximity);
        values.append(target_max_proximity); 
        values.append(target_center_of_proximity(0));
        values.append(target_center_of_proximity(1));
        values.append(target_center_of_proximity(2));
        values.append(target_mean_pressure);
        values.append(target_max_pressure);
        values.append(target_center_of_pressure(0));
        values.append(target_center_of_pressure(1));
        values.append(target_center_of_pressure(2));
        values.append(target_contact_force(0));
        values.append(target_contact_force(1));
        values.append(target_contact_force(2));
        values.append(target_contact_moment(0));
        values.append(target_contact_moment(1));
        values.append(target_contact_moment(2));

        values.append(target_regional_contact_area(0));
        values.append(target_regional_contact_area(1));
        values.append(target_regional_contact_area(2));
        values.append(target_regional_contact_area(3));
        values.append(target_regional_contact_area(4));
        values.append(target_regional_contact_area(5));

        values.append(target_regional_mean_proximity(0));
        values.append(target_regional_mean_proximity(1));
        values.append(target_regional_mean_proximity(2));
        values.append(target_regional_mean_proximity(3));
        values.append(target_regional_mean_proximity(4));
        values.append(target_regional_mean_proximity(5));

        values.append(target_regional_max_proximity(0));
        values.append(target_regional_max_proximity(1));
        values.append(target_regional_max_proximity(2));
        values.append(target_regional_max_proximity(3));
        values.append(target_regional_max_proximity(4));
        values.append(target_regional_max_proximity(5));

        values.append(target_regional_center_of_proximity(0)(0));
        values.append(target_regional_center_of_proximity(0)(1));
        values.append(target_regional_center_of_proximity(0)(2));
        values.append(target_regional_center_of_proximity(1)(0));
        values.append(target_regional_center_of_proximity(1)(1));
        values.append(target_regional_center_of_proximity(1)(2));
        values.append(target_regional_center_of_proximity(2)(0));
        values.append(target_regional_center_of_proximity(2)(1));
        values.append(target_regional_center_of_proximity(2)(2));
        values.append(target_regional_center_of_proximity(3)(0));
        values.append(target_regional_center_of_proximity(3)(1));
        values.append(target_regional_center_of_proximity(3)(2));
        values.append(target_regional_center_of_proximity(4)(0));
        values.append(target_regional_center_of_proximity(4)(1));
        values.append(target_regional_center_of_proximity(4)(2));
        values.append(target_regional_center_of_proximity(5)(0));
        values.append(target_regional_center_of_proximity(5)(1));
        values.append(target_regional_center_of_proximity(5)(2));

        values.append(target_regional_mean_pressure(0));
        values.append(target_regional_mean_pressure(1));
        values.append(target_regional_mean_pressure(2));
        values.append(target_regional_mean_pressure(3));
        values.append(target_regional_mean_pressure(4));
        values.append(target_regional_mean_pressure(5));

        values.append(target_regional_max_pressure(0));
        values.append(target_regional_max_pressure(1));
        values.append(target_regional_max_pressure(2));
        values.append(target_regional_max_pressure(3));
        values.append(target_regional_max_pressure(4));
        values.append(target_regional_max_pressure(5));

        values.append(target_regional_center_of_pressure(0)(0));
        values.append(target_regional_center_of_pressure(0)(1));
        values.append(target_regional_center_of_pressure(0)(2));
        values.append(target_regional_center_of_pressure(1)(0));
        values.append(target_regional_center_of_pressure(1)(1));
        values.append(target_regional_center_of_pressure(1)(2));
        values.append(target_regional_center_of_pressure(2)(0));
        values.append(target_regional_center_of_pressure(2)(1));
        values.append(target_regional_center_of_pressure(2)(2));
        values.append(target_regional_center_of_pressure(3)(0));
        values.append(target_regional_center_of_pressure(3)(1));
        values.append(target_regional_center_of_pressure(3)(2));
        values.append(target_regional_center_of_pressure(4)(0));
        values.append(target_regional_center_of_pressure(4)(1));
        values.append(target_regional_center_of_pressure(4)(2));
        values.append(target_regional_center_of_pressure(5)(0));
        values.append(target_regional_center_of_pressure(5)(1));
        values.append(target_regional_center_of_pressure(5)(2));

        values.append(target_regional_contact_force(0)(0));
        values.append(target_regional_contact_force(0)(1));
        values.append(target_regional_contact_force(0)(2));
        values.append(target_regional_contact_force(1)(0));
        values.append(target_regional_contact_force(1)(1));
        values.append(target_regional_contact_force(1)(2));
        values.append(target_regional_contact_force(2)(0));
        values.append(target_regional_contact_force(2)(1));
        values.append(target_regional_contact_force(2)(2));
        values.append(target_regional_contact_force(3)(0));
        values.append(target_regional_contact_force(3)(1));
        values.append(target_regional_contact_force(3)(2));
        values.append(target_regional_contact_force(4)(0));
        values.append(target_regional_contact_force(4)(1));
        values.append(target_regional_contact_force(4)(2));
        values.append(target_regional_contact_force(5)(0));
        values.append(target_regional_contact_force(5)(1));
        values.append(target_regional_contact_force(5)(2));

        values.append(target_regional_contact_moment(0)(0));
        values.append(target_regional_contact_moment(0)(1));
        values.append(target_regional_contact_moment(0)(2));
        values.append(target_regional_contact_moment(1)(0));
        values.append(target_regional_contact_moment(1)(1));
        values.append(target_regional_contact_moment(1)(2));
        values.append(target_regional_contact_moment(2)(0));
        values.append(target_regional_contact_moment(2)(1));
        values.append(target_regional_contact_moment(2)(2));
        values.append(target_regional_contact_moment(3)(0));
        values.append(target_regional_contact_moment(3)(1));
        values.append(target_regional_contact_moment(3)(2));
        values.append(target_regional_contact_moment(4)(0));
        values.append(target_regional_contact_moment(4)(1));
        values.append(target_regional_contact_moment(4)(2));
        values.append(target_regional_contact_moment(5)(0));
        values.append(target_regional_contact_moment(5)(1));
        values.append(target_regional_contact_moment(5)(2));
    }
    else {
    // No Target data computed
        int nTargetLabels = 119;

        for (int i = 0; i < nTargetLabels; i++) {
            values.append(-1);
        }
    }

    return values;
}