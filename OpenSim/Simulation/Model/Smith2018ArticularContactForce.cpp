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

Smith2018ArticularContactForce::Smith2018ArticularContactForce(
    const std::string& name,
    Smith2018ContactMesh& target_mesh,
    Smith2018ArticularContactForce::ContactParameters target_mesh_params,
    Smith2018ContactMesh& casting_mesh,
    Smith2018ArticularContactForce::ContactParameters casting_mesh_params)
{
    setNull();
    constructProperties();

    setName(name);

    set_target_mesh_contact_params(target_mesh_params);
    set_casting_mesh_contact_params(casting_mesh_params);
    updSocket<Smith2018ContactMesh>("target_mesh").connect(target_mesh);
    updSocket<Smith2018ContactMesh>("casting_mesh").connect(casting_mesh);
}

Smith2018ArticularContactForce::Smith2018ArticularContactForce(
    const std::string& name,
    Smith2018ContactMesh& target_mesh,Smith2018ContactMesh& casting_mesh,
    double elastic_modulus, double poissons_ratio, double thickness)
{
    setNull();
    constructProperties();

    setName(name);

    updSocket<Smith2018ContactMesh>("target_mesh").connect(target_mesh);
    updSocket<Smith2018ContactMesh>("casting_mesh").connect(casting_mesh);

    Smith2018ArticularContactForce::ContactParameters 
        params(elastic_modulus, poissons_ratio, thickness);

    set_target_mesh_contact_params(params);
    set_casting_mesh_contact_params(params);
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
    constructProperty_target_mesh_contact_params(
        Smith2018ArticularContactForce::ContactParameters());
    constructProperty_casting_mesh_contact_params(
        Smith2018ArticularContactForce::ContactParameters());

}

void Smith2018ArticularContactForce::
extendAddToSystem(MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    Smith2018ArticularContactForce::ContactParameters target_mesh_params = 
        get_target_mesh_contact_params();
    Smith2018ArticularContactForce::ContactParameters casting_mesh_params = 
        get_casting_mesh_contact_params();

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

    Vector_<Vec3> casting_mesh_def_vec3(casting_mesh_nTri,Vec3(0.0));

    std::vector<int> target_mesh_def_vector_int(target_mesh_nTri,-1);
    std::vector<int> casting_mesh_def_vector_int(casting_mesh_nTri,-1);

    //TODO: These need to be accessed at in Stage::Position in computeProximity()
    // for rechecking the same triangle that was in contact in the previous
    // state. Is there a better way to make them accessible without setting
    // the stage to LowestRuntime???

    addCacheVariable<std::vector<int>>("target.tri.previous_contacting_tri",
        target_mesh_def_vector_int, Stage::LowestRuntime);
    addCacheVariable<std::vector<int>>("casting.tri.previous_contacting_tri",
        casting_mesh_def_vector_int, Stage::LowestRuntime);

    //Triangles with ray intersections
    addCacheVariable<int>("target.n_active_tri",
        0, Stage::Position);
    addCacheVariable<int>("casting.n_active_tri",
        0, Stage::Position);

    //Subset of n_active_tri with positive proximity
    addCacheVariable<int>("target.n_contacting_tri",
        0, Stage::Position);
    addCacheVariable<int>("casting.n_contacting_tri",
        0, Stage::Position);
    
    //same, neighbor, and different are useful for debugging issues with
    //newly constructed contact meshes
    //Subset of n_contacting_tri that contact same triangle as previous step
    addCacheVariable<int>("target.n_contacting_tri_same",
        0, Stage::Position);
    addCacheVariable<int>("casting.n_contacting_tri_same",
        0, Stage::Position);

    //Subset of n_contacting_tri that contact 
    //neighboring triangle to previous step
    addCacheVariable<int>("target.n_contacting_tri_neighbor",
        0, Stage::Position);
    addCacheVariable<int>("casting.n_contacting_tri_neighbor",
        0, Stage::Position);

    //Subset of n_contacting_tri that contact different triangle from previous 
    //step (not same or neighbor), this means expensive OBB check was used
    addCacheVariable<int>("target.n_contacting_tri_different",
        0, Stage::Position);
    addCacheVariable<int>("casting.n_contacting_tri_different",
        0, Stage::Position);

    addCacheVariable<Vector>("target.tri.proximity",
        target_mesh_def_vec, Stage::Position);
    addCacheVariable<Vector>("casting.tri.proximity",
        casting_mesh_def_vec, Stage::Position);
    
    addCacheVariable<Vector>("target.tri.pressure",
        target_mesh_def_vec, Stage::Dynamics);
    addCacheVariable<Vector>("casting.tri.pressure",
        casting_mesh_def_vec, Stage::Dynamics);

    addCacheVariable<Vector>("target.tri.potential_energy",
        target_mesh_def_vec, Stage::Dynamics);
    addCacheVariable<Vector>("casting.tri.potential_energy",
        casting_mesh_def_vec, Stage::Dynamics);

    addCacheVariable<Vector_<Vec3>>("casting.tri.force",
        casting_mesh_def_vec3, Stage::Dynamics);
    addCacheVariable<Vector_<Vec3>>("target.tri.force",
        casting_mesh_def_vec3, Stage::Dynamics);

    addCacheVariable<double>("target.contact_area",0, Stage::Dynamics);
    addCacheVariable<double>("target.mean_proximity",0, Stage::Dynamics);
    addCacheVariable<double>("target.max_proximity",0, Stage::Dynamics);
    addCacheVariable<Vec3>("target.center_of_proximity",
        Vec3(0), Stage::Dynamics);
    addCacheVariable<double>("target.mean_pressure",0, Stage::Dynamics);
    addCacheVariable<double>("target.max_pressure",0, Stage::Dynamics);
    addCacheVariable<Vec3>("target.center_of_pressure",
        Vec3(0), Stage::Dynamics);
    addCacheVariable<Vec3>("target.contact_force", Vec3(0), Stage::Dynamics);
    addCacheVariable<Vec3>("target.contact_moment", Vec3(0), Stage::Dynamics);

    addCacheVariable<double>("casting.contact_area",0, Stage::Dynamics);
    addCacheVariable<double>("casting.mean_proximity",0, Stage::Dynamics);
    addCacheVariable<double>("casting.max_proximity",0, Stage::Dynamics);
    addCacheVariable<Vec3>("casting.center_of_proximity",
        Vec3(0), Stage::Dynamics);
    addCacheVariable<double>("casting.mean_pressure",0, Stage::Dynamics);
    addCacheVariable<double>("casting.max_pressure",0, Stage::Dynamics);
    addCacheVariable<Vec3>("casting.center_of_pressure",
        Vec3(0), Stage::Dynamics);
    addCacheVariable<Vec3>("casting.contact_force", Vec3(0), Stage::Dynamics);
    addCacheVariable<Vec3>("casting.contact_moment", Vec3(0), Stage::Dynamics);

    addCacheVariable<Vector>("target.regional.contact_area",
        Vector(6,0.0), Stage::Report);
    addCacheVariable<Vector>("target.regional.mean_proximity",
        Vector(6,0.0), Stage::Report);
    addCacheVariable<Vector>("target.regional.max_proximity",
        Vector(6,0.0), Stage::Report);
    addCacheVariable<Vector_<Vec3>>("target.regional.center_of_proximity",
        Vector_<Vec3>(6,Vec3(0)), Stage::Report);
    addCacheVariable<Vector>("target.regional.mean_pressure",
        Vector(6,0.0), Stage::Report);
    addCacheVariable<Vector>("target.regional.max_pressure",Vector(6,0.0),
        Stage::Report);    
    addCacheVariable<Vector_<Vec3>>("target.regional.center_of_pressure",
        Vector_<Vec3>(6,Vec3(0)), Stage::Report);
    addCacheVariable<Vector_<Vec3>>("target.regional.contact_force",
        Vector_<Vec3>(6,Vec3(0)), Stage::Report);
    addCacheVariable<Vector_<Vec3>>("target.regional.contact_moment",
        Vector_<Vec3>(6,Vec3(0)), Stage::Report);

    addCacheVariable<Vector>("casting.regional.contact_area",Vector(6,0.0),
        Stage::Report);
    addCacheVariable<Vector>("casting.regional.mean_proximity",Vector(6,0.0),
        Stage::Report);
    addCacheVariable<Vector>("casting.regional.max_proximity",Vector(6,0.0),
        Stage::Report);
    addCacheVariable<Vector_<Vec3>>("casting.regional.center_of_proximity",
        Vector_<Vec3>(6,Vec3(0)), Stage::Report);
    addCacheVariable<Vector>("casting.regional.mean_pressure",Vector(6,0.0),
        Stage::Report);
    addCacheVariable<Vector>("casting.regional.max_pressure",Vector(6,0.0),
        Stage::Report);    
    addCacheVariable<Vector_<Vec3>>("casting.regional.center_of_pressure",
        Vector_<Vec3>(6,Vec3(0)), Stage::Report);
    addCacheVariable<Vector_<Vec3>>("casting.regional.contact_force",
        Vector_<Vec3>(6,Vec3(0)), Stage::Report);
    addCacheVariable<Vector_<Vec3>>("casting.regional.contact_moment",
        Vector_<Vec3>(6,Vec3(0)), Stage::Report);

    //Modeling Options
    //----------------
    addModelingOption("flip_meshes", 1);
}

void Smith2018ArticularContactForce::computeTriProximity(
    const State& state, const Smith2018ContactMesh& casting_mesh,
    const Smith2018ContactMesh& target_mesh,const std::string& cache_mesh_name,
    SimTK::Vector& tri_proximity = SimTK::Vector()) const
{
    // Get Mesh Properties    
    Vector_<SimTK::Vec3> tri_cen = casting_mesh.getTriangleCenters();
    Vector_<SimTK::UnitVec3> tri_nor = casting_mesh.getTriangleNormals();
   
    Transform MeshCtoMeshT = casting_mesh.getMeshFrame().
        findTransformBetween(state,target_mesh.getMeshFrame());

    //Collision Detection
    //-------------------
    
    //Initialize contact variables

    //Number of triangles with positive ray intersection tests
    int nActiveTri = 0;

    //Subset of nActiveTri with positive proximity
    int nContactingTri = 0;
    tri_proximity.resize(casting_mesh.getNumFaces());
    tri_proximity = 0;

    std::vector<int>& target_tri = updCacheVariableValue<std::vector<int>>
            (state, cache_mesh_name + ".tri.previous_contacting_tri");
    
    
    //Keep track of triangle collision type for debugging
    int nSameTri = 0;
    int nNeighborTri = 0;
    int nDiffTri = 0;
        
    //Loop through all triangles in casting mesh
    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        bool contact_detected = false;
        double distance = 0.0;
        SimTK::Vec3 contact_point;
        SimTK::Vec3 origin = MeshCtoMeshT.shiftFrameStationToBase(tri_cen(i));
        SimTK::UnitVec3 direction(MeshCtoMeshT.xformFrameVecToBase(tri_nor(i)));

        //If triangle was in contact in previous timestep, 
        //recheck same contact triangle and neighbors
        if (target_tri[i] >= 0) {
            if (target_mesh._obb.rayIntersectTri(
                target_mesh.getPolygonalMesh(), origin, -direction,
                target_tri[i], contact_point, distance))
            {
                tri_proximity(i) = distance;

                nActiveTri++;
                nSameTri++;

                if (tri_proximity(i) > 0.0) { nContactingTri++; }
                continue;

            }

            //Check neighboring triangles

            std::set<int> neighborTris = 
                target_mesh.getNeighborTris(target_tri[i]);

            for (int neighbor_tri : neighborTris) {
                if (target_mesh._obb.rayIntersectTri(
                    target_mesh.getPolygonalMesh(), origin, -direction,
                    neighbor_tri, contact_point, distance))
                {
                    tri_proximity(i) = distance;
                    target_tri[i] = neighbor_tri;

                    nActiveTri++;
                    nNeighborTri++;
                    if (tri_proximity(i) > 0.0) { nContactingTri++; }

                    contact_detected = true;
                    break;
                }
            }
            if (contact_detected) {
                continue;
            }
        }

        //Go through the OBB hierarchy
        int contact_target_tri=-1;
        
        if (target_mesh.rayIntersectMesh(origin,-direction,
            get_min_proximity(), get_max_proximity(),
            contact_target_tri,contact_point,distance)){

            target_tri[i] = contact_target_tri;
            tri_proximity(i) = distance;

            nActiveTri++;
            nDiffTri++;
            if (tri_proximity(i) > 0.0) { nContactingTri++;}
            continue;
        }

        //Else - triangle is not in contact
        target_tri[i] = -1;
    }
       
    //Store Contact Info
    setCacheVariableValue(state, cache_mesh_name + 
        ".tri.proximity", tri_proximity);    
    setCacheVariableValue(state, cache_mesh_name + 
        ".tri.previous_contacting_tri",target_tri);
    setCacheVariableValue(state, cache_mesh_name + 
        ".n_active_tri", nActiveTri);
    setCacheVariableValue(state, cache_mesh_name + 
        ".n_contacting_tri", nContactingTri);
    setCacheVariableValue(state, cache_mesh_name + 
        ".n_contacting_tri_same", nSameTri);
    setCacheVariableValue(state, cache_mesh_name + 
        ".n_contacting_tri_neighbor", nNeighborTri);
    setCacheVariableValue(state, cache_mesh_name + 
        ".n_contacting_tri_different", nDiffTri);
}

void Smith2018ArticularContactForce::computeTriDynamics(
    const State& state, const Smith2018ContactMesh& casting_mesh,
    const Smith2018ContactMesh& target_mesh,
    SimTK::Vector_<SimTK::Vec3>& tri_force = SimTK::Vector_<SimTK::Vec3>(),
    SimTK::Vector& tri_pressure = SimTK::Vector(),
    SimTK::Vector& tri_energy = SimTK::Vector()) const
{
    std::string casting_path = getConnectee<Smith2018ContactMesh>
        ("casting_mesh").getAbsolutePathString();

    std::string cache_mesh_name;
    Smith2018ArticularContactForce::ContactParameters cContactParams;
    Smith2018ArticularContactForce::ContactParameters tContactParams;

    if (casting_path == casting_mesh.getAbsolutePathString())
    {
        cache_mesh_name = "casting";
        cContactParams = get_casting_mesh_contact_params();
        tContactParams = get_target_mesh_contact_params();

    }
    else {
        cache_mesh_name = "target";
        tContactParams = get_casting_mesh_contact_params();
        cContactParams = get_target_mesh_contact_params();
    }
        
    const Vector& tri_proximity = getCacheVariableValue<Vector>(state,
        cache_mesh_name + ".tri.proximity");
    const std::vector<int>& target_tri = 
        getCacheVariableValue<std::vector<int>>(state,
        cache_mesh_name + ".tri.previous_contacting_tri");

    const Vector& tri_area = casting_mesh.getTriangleAreas();

    tri_pressure.resize(casting_mesh.getNumFaces());    
    tri_pressure = 0;
    tri_energy.resize(casting_mesh.getNumFaces());
    tri_energy = 0;

    double hT, hC; //thickness
    double ET, EC; //elastic modulus
    double vT, vC; //poissons ratio

    //Compute Tri Pressure and Potential Energy
    //-----------------------------------------
    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        if (tri_proximity(i) <= 0) {
            tri_pressure(i) = 0;
            tri_energy(i) = 0;
            continue;
        }

        //Get Contact Material Parameters
        if (tContactParams.get_use_variable_thickness())
            hT = target_mesh.getTriangleThickness()(target_tri[i]);
        else
            hT = tContactParams.get_thickness();

        //Commented code is a place holder to be able to read variable 
        //elastic modulus and poissons ratio properties for each triangle 
        //from a .vtp file. This reader isn't implemented yet. 

        //if (tContactParams.get_use_variable_elastic_modulus())
        //    ET = target_mesh.getTriangleElasticModulus()(target_tri[i]);
        //else
            ET = tContactParams.get_elastic_modulus();

        //if (tContactParams.get_use_variable_poissons_ratio())
        //    vT = target_mesh.getTrianglePoissonsRatio()(target_tri[i]);
        //else
            vT = tContactParams.get_poissons_ratio();

        if (cContactParams.get_use_variable_thickness())
            hC = casting_mesh.getTriangleThickness()(i);
        else
            hC = cContactParams.get_thickness();

        //if (cContactParams.get_use_variable_elastic_modulus())
        //    EC = casting_mesh.getTriangleElasticModulus()(i);
        //else
            EC = cContactParams.get_elastic_modulus();

        //if (cContactParams.get_use_variable_poissons_ratio())
        //    vC = casting_mesh.getTrianglePoissonsRatio()(i);
        //else
            vC = cContactParams.get_poissons_ratio();

        //Compute pressure & energy using the lumped contact model

        if (get_use_lumped_contact_model()) {
            double E = (ET + EC) / 2;
            double v = (vT + vC) / 2;
            double h = (hT + hC);

            double K = (1 - v)*E / ((1 + v)*(1 - 2 * v));


            if (get_elastic_foundation_formulation() == "linear") {
                tri_pressure(i) = K * tri_proximity(i) / h;
                tri_energy(i) = 0.5 * tri_area(i) * K *
                    pow(tri_proximity(i), 2) / h;                
                continue;
            }

            if (get_elastic_foundation_formulation() == "nonlinear") {
                tri_pressure(i) = -K * log(1 - tri_proximity(i) / h);
                tri_energy(i) = -tri_area(i)* K * ((tri_proximity(i) - h)*
                    log(1 - tri_proximity(i) / h) - tri_proximity(i));
                continue;
            }
        }

        //Compute pressure & energy using variable property model

        //linear solution
        double kT = ((1 - vT)*ET) / ((1 + vT)*(1 - 2 * vT)*hT);
        double kC = ((1 - vC)*EC) / ((1 + vC)*(1 - 2 * vC)*hC);

        double linearPressure = (kT*kC) / (kT + kC)*tri_proximity(i);

        if (get_elastic_foundation_formulation() == "linear") {
            tri_pressure(i) = linearPressure;

            double depthT = kC / (kT + kC)*tri_proximity(i);
            double depthC = kT / (kT + kC)*tri_proximity(i);

            double energyC = 0.5 * tri_area(i) * kC * pow(depthC, 2);
            double energyT = 0.5 * tri_area(i) * kT * pow(depthT, 2);
            tri_energy(i) = energyC + energyT;
            continue;
        }

        //nonlinear solution
        else{ //(get_elastic_foundation_formulation() == "nonlinear") 

            nonlinearContactParams cp;

            cp.dc = tri_proximity(i);
            cp.h1 = hC;
            cp.h2 = hT;
            kC = (1 - vC)*EC / ((1 + vC)*(1 - 2 * vC));
            kT = (1 - vT)*ET / ((1 + vT)*(1 - 2 * vT));
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
            x[0] = linearPressure;

            //Solve nonlinear equation
            lmdif_C(calcNonlinearPressureResid, nEqn, nVar, x, fvec,
                ftol, xtol, gtol, maxfev, epsfcn, diag, mode, step_factor,
                nprint, &info, &num_func_calls, fjac, ldfjac, ipvt, qtf,
                wa1, wa2, wa3, wa4, (void*)&cp);

            double nonlinearPressure = x[0];
            tri_pressure(i) = nonlinearPressure;

            double depthC = hC * (1 - exp(-nonlinearPressure / kC));
            double depthT = hT * (1 - exp(-nonlinearPressure / kT));

            double energyC = -tri_area(i)* kC * 
                ((depthC - hC)*log(1 - depthC / hC) - depthC);
            double energyT = -tri_area(i)* kT * 
                ((depthT - hT)*log(1 - depthT / hT) - depthT);
            tri_energy(i) = energyC + energyT;
            continue;
        }
    }

    setCacheVariableValue(state, cache_mesh_name + 
        ".tri.pressure", tri_pressure);    
    setCacheVariableValue(state, cache_mesh_name + 
        ".tri.potential_energy", tri_energy);

    //Compute Triangle Forces 
    //-----------------------
    const Vector_<UnitVec3>& tri_normal = casting_mesh.getTriangleNormals();

    tri_force.resize(casting_mesh.getNumFaces());
    tri_force = Vec3(0.0);

    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        for (int j = 0; j < 3; ++j) {
            tri_force(i)(j) = 
                tri_pressure(i) * tri_area(i) * -tri_normal(i)(j);
        }
    }
    setCacheVariableValue(state, cache_mesh_name + ".tri.force", tri_force);
    return;
}

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
void Smith2018ArticularContactForce::calcNonlinearPressureResid(
    int nEqn, int nVar, double x[], double fvec[], int *flag2, void *ptr)
{
    nonlinearContactParams * cp = (nonlinearContactParams*)ptr;

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
    SimTK::Vector casting_tri_proximity;
    if (!isCacheVariableValid(state, "casting.tri.proximity")) {
        computeTriProximity(state, casting_mesh, target_mesh,
            "casting", casting_tri_proximity);
    }
    else {
        casting_tri_proximity = getCacheVariableValue<SimTK::Vector>
            (state, "casting_tri_proximity");
    }
    
    SimTK::Vector casting_tri_pressure;
    SimTK::Vector casting_tri_energy;
    SimTK::Vector_<Vec3> casting_tri_force;

    //Pressure
    computeTriDynamics(state, casting_mesh, target_mesh, casting_tri_force,
        casting_tri_pressure);

    //Force        
    const PhysicalFrame& target_frame = target_mesh.getMeshFrame();
    const PhysicalFrame& casting_frame = casting_mesh.getMeshFrame();

    const Vector_<Vec3>& tri_center = casting_mesh.getTriangleCenters();

    Transform T_casting_to_ground = casting_frame.getTransformInGround(state);
    Transform T_casting_to_target = 
        casting_frame.findTransformBetween(state,target_frame);

    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        Vec3 casting_force_ground = 
            T_casting_to_ground.xformFrameVecToBase(casting_tri_force(i));
        
        applyForceToPoint(state, casting_frame, tri_center(i),
            casting_force_ground, bodyForces);

        Vec3 target_force_ground = -casting_force_ground;
        Vec3 tri_center_target = 
            T_casting_to_target.shiftFrameStationToBase(tri_center(i));

        applyForceToPoint(state, target_frame, tri_center_target,
            target_force_ground, bodyForces);
    }

    //Compute Contact Stats
    std::vector<int> casting_faces;
    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        casting_faces.push_back(i);
    }

    auto stats = computeContactStats(casting_mesh, casting_tri_proximity,
        casting_tri_pressure,casting_faces);
    
    setCacheVariableValue(state, 
        "casting.contact_area", stats.contact_area);
    setCacheVariableValue(state, 
        "casting.mean_proximity", stats.mean_proximity);        
    setCacheVariableValue(state, 
        "casting.max_proximity", stats.max_proximity);
    setCacheVariableValue(state, 
        "casting.center_of_proximity", stats.center_of_proximity);
    setCacheVariableValue(state, 
        "casting.mean_pressure", stats.mean_pressure);
    setCacheVariableValue(state, 
        "casting.max_pressure", stats.max_pressure);
    setCacheVariableValue(state, 
        "casting.center_of_pressure", stats.center_of_pressure);
    setCacheVariableValue(state, 
        "casting.contact_force", stats.contact_force);
    setCacheVariableValue(state, 
        "casting.contact_moment", stats.contact_moment); 

    //Target mesh computations (not used in applied contact force calculation)
    if (getModelingOption(state, "flip_meshes")) {
        //target proximity
        SimTK::Vector target_tri_proximity;
        if (!isCacheVariableValid(state, "target.tri.proximity")) {
            computeTriProximity(state, target_mesh, casting_mesh,
                "target", target_tri_proximity);
        }
        else {
            target_tri_proximity = getCacheVariableValue<SimTK::Vector>
                (state, "target.tri.proximity");
        }

        //target pressure
        SimTK::Vector target_tri_pressure;
        SimTK::Vector_<SimTK::Vec3> target_tri_force;

        computeTriDynamics(state, target_mesh, casting_mesh,
            target_tri_force, target_tri_pressure);

        //target contact stats
        std::vector<int> casting_faces;
        for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
            casting_faces.push_back(i);
        }

        auto stats = computeContactStats(target_mesh, target_tri_proximity,
            target_tri_pressure,casting_faces);
    
        setCacheVariableValue(state,
            "target.contact_area", stats.contact_area);
        setCacheVariableValue(state,
            "target.mean_proximity", stats.mean_proximity);
        setCacheVariableValue(state,
            "target.max_proximity", stats.max_proximity);
        setCacheVariableValue(state,
            "target.center_of_proximity", stats.center_of_proximity);
        setCacheVariableValue(state,
            "target.mean_pressure", stats.mean_pressure);
        setCacheVariableValue(state,
            "target.max_pressure", stats.max_pressure);
        setCacheVariableValue(state,
            "target.center_of_pressure", stats.center_of_pressure);
        setCacheVariableValue(state,
            "target.contact_force", stats.contact_force);
        setCacheVariableValue(state,
            "target.contact_moment", stats.contact_moment);
    }
}

double Smith2018ArticularContactForce::
computePotentialEnergy(const SimTK::State& state) const
{
    if (!isCacheVariableValid(state, "casting.tri.potential_energy")) {
        _model->realizeDynamics(state);
    }
    SimTK::Vector tri_energy = getCacheVariableValue<SimTK::Vector>(
        state, "casting.tri.potential_energy");
    return tri_energy.sum();
}

Vec3 Smith2018ArticularContactForce::
computeContactForceVector(double pressure, double area, Vec3 normal) const
{
    Vec3 force;
    force(0) = normal(0) * pressure * area;
    force(1) = normal(1) * pressure * area;
    force(2) = normal(2) * pressure * area;
    return force;
}

Vec3 Smith2018ArticularContactForce::
computeContactMomentVector(double pressure, double area, Vec3 normal,
    Vec3 center) const
{
    Vec3 moment;
    Vec3 force;

    force(0) = normal(0) * pressure * area;
    force(1) = normal(1) * pressure * area;
    force(2) = normal(2) * pressure * area;

    moment(0) = force(2) * center(1) - force(1) * center(2);
    moment(1) = force(0) * center(2) - force(2) * center(0);
    moment(2) = force(1) * center(0) - force(0) * center(1);
    return moment;
}


void Smith2018ArticularContactForce::
extendRealizeReport(const State & state) const
{
    Super::extendRealizeReport(state);
    const auto& casting_mesh = 
        getConnectee<Smith2018ContactMesh>("casting_mesh");
    const auto& target_mesh = 
        getConnectee<Smith2018ContactMesh>("target_mesh");

    const SimTK::Vector& casting_tri_pressure = 
        getCacheVariableValue<SimTK::Vector>(state, "casting.tri.pressure");

    const SimTK::Vector& casting_tri_proximity = 
        getCacheVariableValue<SimTK::Vector>(state, "casting.tri.proximity");
    
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
        auto stats = computeContactStats(casting_mesh, casting_tri_proximity,
        casting_tri_pressure, casting_region_tri_ind[i]);

        reg_contact_area(i) = stats.contact_area;
        reg_mean_proximity(i) = stats.mean_proximity;
        reg_max_proximity(i) = stats.max_proximity;
        reg_COPrx(i) = stats.center_of_proximity;
        reg_mean_pressure(i) = stats.mean_pressure;
        reg_max_pressure(i) = stats.max_pressure;
        reg_COP(i) = stats.center_of_pressure;
    }
    setCacheVariableValue(state,
        "casting.regional.contact_area", reg_contact_area);
    setCacheVariableValue(state,
        "casting.regional.mean_proximity", reg_mean_proximity);
    setCacheVariableValue(state,
        "casting.regional.max_proximity", reg_max_proximity);
    setCacheVariableValue(state,
        "casting.regional.center_of_proximity", reg_COPrx);
    setCacheVariableValue(state,
        "casting.regional.mean_pressure", reg_mean_pressure);
    setCacheVariableValue(state,
        "casting.regional.max_pressure", reg_max_pressure);
    setCacheVariableValue(state,
        "casting.regional.center_of_pressure", reg_COP);
    setCacheVariableValue(state,
        "casting.regional.contact_force", reg_contact_force);
    setCacheVariableValue(state,
        "casting.regional.contact_moment", reg_contact_moment);

    //target
    if (getModelingOption(state, "flip_meshes")) {

        const SimTK::Vector& target_tri_pressure = 
            getCacheVariableValue<SimTK::Vector>(
                state, "target.tri.pressure");

        const SimTK::Vector& target_tri_proximity = 
            getCacheVariableValue<SimTK::Vector>(
                state, "target.tri.proximity");

        std::vector<std::vector<int>> target_region_tri_ind =
            target_mesh.getRegionalTriangleIndices();

        for (int i = 0; i < 6; ++i) {
            auto stats = computeContactStats(target_mesh, target_tri_proximity,
                target_tri_pressure, target_region_tri_ind[i]);

            reg_contact_area(i) = stats.contact_area;
            reg_mean_proximity(i) = stats.mean_proximity;
            reg_max_proximity(i) = stats.max_proximity;
            reg_COPrx(i) = stats.center_of_proximity;
            reg_mean_pressure(i) = stats.mean_pressure;
            reg_max_pressure(i) = stats.max_pressure;
            reg_COP(i) = stats.center_of_pressure;
        }

        setCacheVariableValue(state,
            "target.regional.contact_area", reg_contact_area);
        setCacheVariableValue(state,
            "target.regional.mean_proximity", reg_mean_proximity);
        setCacheVariableValue(state,
            "target.regional.max_proximity", reg_max_proximity);
        setCacheVariableValue(state,
            "target.regional.center_of_proximity", reg_COPrx);
        setCacheVariableValue(state,
            "target.regional.mean_pressure", reg_mean_pressure);
        setCacheVariableValue(state,
            "target.regional.max_pressure", reg_max_pressure);
        setCacheVariableValue(state,
            "target.regional.center_of_pressure", reg_COP);
        setCacheVariableValue(state,
            "target.regional.contact_force", reg_contact_force);
        setCacheVariableValue(state,
            "target.regional.contact_moment", reg_contact_moment);
    }
}

Smith2018ArticularContactForce::contact_stats
Smith2018ArticularContactForce::computeContactStats(
    const Smith2018ContactMesh& mesh,
    const SimTK::Vector& total_tri_proximity,
    const SimTK::Vector& total_tri_pressure,    
    const std::vector<int>& triIndices) const
{
    Smith2018ArticularContactForce::contact_stats stats;

    int nTri = static_cast<int>(triIndices.size());

    SimTK::Vector tri_proximity(nTri);
    SimTK::Vector tri_pressure(nTri);

    SimTK::Vector total_tri_area = mesh.getTriangleAreas();
    SimTK::Vector tri_area(nTri);

    SimTK::Vector_<UnitVec3> total_tri_normal = mesh.getTriangleNormals();
    SimTK::Vector_<UnitVec3> tri_normal(nTri);

    SimTK::Vector_<Vec3> total_tri_center = mesh.getTriangleCenters();
    SimTK::Vector_<Vec3> tri_center(nTri);
    int nContactingTri = 0;

    for (int i = 0; i < nTri; ++i) {
        tri_area(i) = total_tri_area(triIndices[i]);
        tri_normal(i) = total_tri_normal(triIndices[i]);
        tri_proximity(i) = total_tri_proximity(triIndices[i]);
        tri_pressure(i) = total_tri_pressure(triIndices[i]);
        tri_center(i) = total_tri_center(triIndices[i]);

        if(tri_pressure(i) > 0.0){
            nContactingTri++;
        }
    }
    
    SimTK::Vector tri_cenX(nTri);
    SimTK::Vector tri_cenY(nTri);
    SimTK::Vector tri_cenZ(nTri);

    for (int i = 0; i < nTri; ++i) {
        tri_cenX(i) = tri_center(i)(0);
        tri_cenY(i) = tri_center(i)(1);
        tri_cenZ(i) = tri_center(i)(2);
    }

    //Mean Pressure
    stats.mean_pressure = tri_pressure.sum() / nContactingTri;
    stats.mean_proximity = tri_proximity.sum() / nContactingTri;

    //Max Pressure
    stats.max_pressure = tri_pressure.normInf();
    stats.max_proximity = tri_proximity.normInf();

    //Contact Area
    double contact_area = 0.0;

    for (int i = 0; i < nTri; i++) {
        if (tri_pressure(i) > 0.0) {
            contact_area += tri_area(i);
        }
    }

    stats.contact_area = contact_area;

    //Center of Proximity
    SimTK::Vector Num_prx = tri_proximity.elementwiseMultiply(tri_area);
    SimTK::Vector Den_prx = tri_proximity.elementwiseMultiply(tri_area);
    double DenVal_prx = Den_prx.sum();

    SimTK::Vector xNum_prx = Num_prx.elementwiseMultiply(tri_cenX);
    double xNumVal_prx = xNum_prx.sum();
    double COPrx_x = xNumVal_prx / DenVal_prx;

    SimTK::Vector yNum_prx = Num_prx.elementwiseMultiply(tri_cenY);
    double yNumVal_prx = yNum_prx.sum();
    double COPrx_y = yNumVal_prx / DenVal_prx;

    SimTK::Vector zNum_prx = Num_prx.elementwiseMultiply(tri_cenZ);
    double zNumVal_prx = zNum_prx.sum();
    double COPrx_z = zNumVal_prx / DenVal_prx;

    stats.center_of_proximity(0) = COPrx_x;
    stats.center_of_proximity(1) = COPrx_y;
    stats.center_of_proximity(2) = COPrx_z;

    //Center of Pressure
    SimTK::Vector Num = tri_pressure.elementwiseMultiply(tri_area);
    SimTK::Vector Den = tri_pressure.elementwiseMultiply(tri_area);
    double DenVal = Den.sum();

    SimTK::Vector xNum = Num.elementwiseMultiply(tri_cenX);
    double xNumVal = xNum.sum();
    double COPx = xNumVal / DenVal;

    SimTK::Vector yNum = Num.elementwiseMultiply(tri_cenY);
    double yNumVal = yNum.sum();
    double COPy = yNumVal / DenVal;

    SimTK::Vector zNum = Num.elementwiseMultiply(tri_cenZ);
    double zNumVal = zNum.sum();
    double COPz = zNumVal / DenVal;

    stats.center_of_pressure(0) = COPx;
    stats.center_of_pressure(1) = COPy;
    stats.center_of_pressure(2) = COPz;

    //Contact Force
    stats.contact_force = 0.0;
    stats.contact_moment = 0.0;
    

    for (int i = 0; i < nTri; ++i) {
        stats.contact_force += computeContactForceVector(
            tri_pressure(i), tri_area(i), -tri_normal(i));
        stats.contact_moment += computeContactMomentVector(
            tri_pressure(i), tri_area(i), -tri_normal(i), tri_center(i));
    }

    return stats;

}

OpenSim::Array<std::string> Smith2018ArticularContactForce::
getRecordLabels() const {
    // Can only return casting_mesh computations because target_mesh is 
    // not used in computation of force, so if the flip_meshes ModelingOption
    // is not set, target_mesh values will not be valid

    OpenSim::Array<std::string> labels("");

    labels.append(getName() + ".casting.contact_area");    
    labels.append(getName() + ".casting.mean_proximity");
    labels.append(getName() + ".casting.max_proximity"); 
    labels.append(getName() + ".casting.center_of_proximity_x");
    labels.append(getName() + ".casting.center_of_proximity_y");
    labels.append(getName() + ".casting.center_of_proximity_z");
    labels.append(getName() + ".casting.mean_pressure");
    labels.append(getName() + ".casting.max_pressure");    
    labels.append(getName() + ".casting.center_of_pressure_x");
    labels.append(getName() + ".casting.center_of_pressure_y");
    labels.append(getName() + ".casting.center_of_pressure_z");
    labels.append(getName() + ".casting.contact_force_x");
    labels.append(getName() + ".casting.contact_force_y");
    labels.append(getName() + ".casting.contact_force_z");
    labels.append(getName() + ".casting.contact_moment_x");
    labels.append(getName() + ".casting.contact_moment_y");
    labels.append(getName() + ".casting.contact_moment_z");    

    return labels;
}


OpenSim::Array<double> Smith2018ArticularContactForce::
getRecordValues(const SimTK::State& state) const {

    double contact_area = getCacheVariableValue<double>
        (state, "casting.contact_area");
    double mean_proximity = getCacheVariableValue<double>
        (state, "casting.mean_proximity");
    double max_proximity = getCacheVariableValue<double>
        (state, "casting.max_proximity");
    SimTK::Vec3 center_of_proximity = getCacheVariableValue<SimTK::Vec3>
        (state, "casting.center_of_proximity");
    double mean_pressure = getCacheVariableValue<double>
        (state, "casting.mean_pressure");
    double max_pressure = getCacheVariableValue<double>
        (state, "casting.max_pressure");
    SimTK::Vec3 center_of_pressure = getCacheVariableValue<SimTK::Vec3>
        (state, "casting.center_of_pressure");
    SimTK::Vec3 contact_force = getCacheVariableValue<SimTK::Vec3>
        (state, "casting.contact_force");
    SimTK::Vec3 contact_moment =  getCacheVariableValue<SimTK::Vec3>
        (state, "casting.contact_moment");
    
    OpenSim::Array<double> values(1);
    
    values.append(contact_area);
    values.append(mean_proximity);
    values.append(max_proximity); 
    values.append(center_of_proximity(0));
    values.append(center_of_proximity(1));
    values.append(center_of_proximity(2));
    values.append(mean_pressure);
    values.append(max_pressure);    
    values.append(center_of_pressure(0));
    values.append(center_of_pressure(1));
    values.append(center_of_pressure(2));
    values.append(contact_force(0));
    values.append(contact_force(1));
    values.append(contact_force(2));
    values.append(contact_moment(0));
    values.append(contact_moment(1));
    values.append(contact_moment(2));

    return values;
}


//=============================================================================
//               Smith2018_ELASTIC FOUNDATION FORCE :: CONTACT PARAMETERS
//=============================================================================

// Default constructor.
Smith2018ArticularContactForce::ContactParameters::ContactParameters()
{
    constructProperties();
}

// Constructor specifying material properties.
Smith2018ArticularContactForce::ContactParameters::ContactParameters
(double elastic_modulus, double poissons_ratio, double thickness)
{
    constructProperties();
    set_elastic_modulus(elastic_modulus);
    set_poissons_ratio(poissons_ratio);
    set_thickness(thickness);
}

void Smith2018ArticularContactForce::ContactParameters::constructProperties()
{
    constructProperty_elastic_modulus(0.0);
    constructProperty_poissons_ratio(0.0);
    constructProperty_thickness(0.0);
    constructProperty_use_variable_thickness(true);
}