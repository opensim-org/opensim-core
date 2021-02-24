#ifndef OPENSIM_SMITH2018_CONTACT_MESH_H_
#define OPENSIM_SMITH2018_CONTACT_MESH_H_
/* -------------------------------------------------------------------------- *
 *                           Smith2018ContactMesh.h                           *
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

#include "OpenSim/Simulation/Model/ContactGeometry.h"
#include "OpenSim/Simulation/Model/PhysicalOffsetFrame.h"

namespace OpenSim {


//=============================================================================
//                       Smith2018ArticularContact
//=============================================================================
/**
This component is used to represent the articular surfaces
in the Smith2018ArticularContactForce component. The Smith2018ContactMesh reads
in a triangle mesh from the mesh_file property (.stl, .vtp, .obj) that 
represents only the contact surface. This component can only be used with the 
Smith2018ArticularContactForce component to be in contact with another 
Smith2018ContactMesh.

# Mesh Requirements
The mesh does not need to be closed (ie water tight) 
and a smaller number of triangles in the mesh will lead to faster 
collision detection performance. The normal vectors of the mesh should be 
pointing outwards from the articular surface towards the opposing contact 
mesh. Misdirected triangle normals is a common issue when constructing new 
meshes. 

\image html fig_Smith2018ContactMesh.png width=600px

# Variable Thickness
The Smith2018ContactMesh can calculate the local thickness at each triangle 
to generate spatially varying thickness maps. Here the optional mesh_back_file
property must be defined using a mesh to represent the subchondral bone or
back side of an implant component. The thickness is calculated by casting a 
normal ray from the center of each triangle in the mesh_file mesh towards the
mesh_back_file mesh. The thickness is clipped so that it remains within the 
range defined by min_thickness and max_thickness. 

# Scaling
The mesh can be linearly scaled in the x,y,z directions using the 
scale_factors property. When using the ScaleTool, these scale factors are 
set based on the frame set in the scale_frame socket. It is generally 
advisable to scale both Smith2018ContactMesh meshes used as a contacting pair 
by the same scale factors to ensure the congruency of the articulating 
surfaces is not substaintially altered. For example, in a knee joint the 
scale_frame socket for both the femur and tibia Smith2018ContactMeshes would 
be connected to the femur frame to ensure both meshes are scaled by the femur 
scale factors.

# Choosing the coarseness of the mesh
Because the contact force and potential energy are calculated based on 
the triangle areas and normals, and the derivatives of these outputs with 
respect to joint coordinates are commonly calculated in integrators and 
optimizers, the best performance will be achieved with a smooth mesh whose 
triangle areas are similar sized. In some extreme cases where the mesh becomes
excessively coarse, the simulation slow down caused by jumps in the computed 
forces outweighs the speed up in collison detection gained by reducing the 
number of triangles in the mesh. For details on a convergence study based on 
triangle area see [1]. Note that the GPU implementation described in the paper 
is not implemented here.

# Collison Detection
The collision detection algorithm is described in the Smith2018ArticularContact
class description. The Smith2018ContactMesh stores all geometric mesh data and
also performs ray intersection tests with a individual mesh triangles or an
Oriented Bounding Box (OBB) hierarchy. Here, a SimTK::OrientedBoundingBox 
is constructed for the mesh_file geometry using code adapted from
SimTK::ContactGeometry::TriangularMesh::OBBTreeNodeImpl.

# References

   [1] Smith, C. R., Won Choi, K., Negrut, D., & Thelen, D. G. (2018).
       Efficient computation of cartilage contact pressures within dynamic
       simulations of movement. Computer Methods in Biomechanics and
       Biomedical Engineering: Imaging & Visualization, 6(5), 491-498.
*/



class OSIMSIMULATION_API Smith2018ContactMesh : public ContactGeometry {
OpenSim_DECLARE_CONCRETE_OBJECT(Smith2018ContactMesh, ContactGeometry)

public:
    class OBBTreeNode;
    //=====================================================================
    // PROPERTIES
    //=====================================================================
    OpenSim_DECLARE_PROPERTY(mesh_file, std::string,
        "Path to triangle mesh geometry file representing the contact surface "
        "(supports .obj, .stl, .vtp).")

    OpenSim_DECLARE_PROPERTY(elastic_modulus, double, 
        "Uniform Elastic Modulus value for every triangle in mesh. "
        "The default value is 1000000.0 Pa.")

    OpenSim_DECLARE_PROPERTY(poissons_ratio, double, 
        "Uniform Poissons Ratio value for every triangle in mesh. "
        "The default value is 0.5.")

    OpenSim_DECLARE_PROPERTY(thickness, double,
        "Uniform thickness of elastic layer for entire mesh. "
        "The default value is 0.005 meters")

    OpenSim_DECLARE_PROPERTY(use_variable_thickness, bool,
        "Compute the local thickness for each triangle in mesh_file by "
        "calculating the distance along a normal ray cast from the center of "
        "each triangle in the mesh_file to intersection with the "
        "mesh_back_file. If use_variable_thickness is true, mesh_back_file "
        "must be defined and the 'thickness' property value is not used."
        "The Default value is false.")

    OpenSim_DECLARE_OPTIONAL_PROPERTY(mesh_back_file, std::string,
        "Path to traingle mesh geometry file representing the backside of "
        "contact surface elastic layer (bone / backside of artifical "
        "component) mesh geometry file (supports .obj, .stl, .vtp). ")

    OpenSim_DECLARE_OPTIONAL_PROPERTY(min_thickness, double,
        "Minimum thickness threshold for elastic layer [m] when calculating "
        "variable thickness for each triangle.")

    OpenSim_DECLARE_OPTIONAL_PROPERTY(max_thickness, double,
        "Maximum thickness threshold for elastic layer [m] when calculating "
        "variable thickness for each triangle.")

    OpenSim_DECLARE_PROPERTY(scale_factors, SimTK::Vec3,
        "[x,y,z] scale factors applied to vertex locations of the mesh_file "
        "and mesh_back_file meshes.")

    //=========================================================================
    // SOCKETS
    //=========================================================================
    OpenSim_DECLARE_SOCKET(scale_frame, PhysicalFrame,
        "When using the ScaleTool, the scale factors from this frame will be "
        "used to scale the mesh.")

    //=========================================================================
    // METHODS
    //=========================================================================
public:
    // CONSTRUCORS
    Smith2018ContactMesh();

    Smith2018ContactMesh(const std::string& name,
        const std::string& mesh_file, const PhysicalFrame& frame);

    Smith2018ContactMesh(const std::string& name,
        const std::string& mesh_file, const PhysicalFrame& frame,
        const SimTK::Vec3& location, const SimTK::Vec3& orientation);

    Smith2018ContactMesh(const std::string& name,
        const std::string& mesh_file, const PhysicalFrame& frame,
        const SimTK::Vec3& location, const SimTK::Vec3& orientation,
        bool use_variable_thickness,
        const std::string& mesh_back_file, 
        double  min_thickness, double max_thickness);

    //TODO This function must be overriden for because this component is 
    //derived from OpenSim::ContactGeometry.
    SimTK::ContactGeometry createSimTKContactGeometry() const override {
        return SimTK::ContactGeometry();
    };

    const SimTK::PolygonalMesh& getPolygonalMesh() const {
        return _mesh;
    }

    int getNumFaces() const {
        return _mesh.getNumFaces();
    }

    int getNumVertices() const {
        return _mesh.getNumVertices();
    }

    const std::set<int>& getNeighborTris(int tri) const {
        return _tri_neighbors[tri];
    }

    const std::vector<std::vector<int>>& getRegionalTriangleIndices() const {
        return _regional_tri_ind;
    }

    const double& getTriangleThickness(int i) const {
        return _tri_thickness(i);
    }

    const double& getTriangleElasticModulus(int i) const {
        return _tri_elastic_modulus(i);
    }

    const double& getTrianglePoissonsRatio(int i) const {
        return _tri_poissons_ratio(i);
    }

    const SimTK::Vector& getTriangleAreas() const {
        return _tri_area;
    }

    const SimTK::Vector_<SimTK::Vec3>& getTriangleCenters() const {
        return _tri_center;
    }

    const SimTK::Vector_<SimTK::UnitVec3>& getTriangleNormals() const {
        return _tri_normal;
    }

    const SimTK::Matrix_<SimTK::Vec3>& getFaceVertexLocations() const {
        return _face_vertex_locations;
    }

    const SimTK::Vector_<SimTK::Vec3>& getVertexLocations() const {
        return _vertex_locations;
    }

    const OBBTreeNode& getOBBTreeNode() const {
        return _obb;
    }

    int getOBBNumTriangles() const {
        return _obb._numTriangles;
    }
    
    bool rayIntersectMesh(
        const SimTK::Vec3& origin, const SimTK::UnitVec3& direction,
        const double& min_proximity, const double& max_proximity,
        int& tri, SimTK::Vec3 intersection_point,
        SimTK::Real& distance) const;

    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& s, 
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const override;

    const PhysicalFrame& getMeshFrame() const {
        return getComponent<PhysicalOffsetFrame>("mesh_frame");
    };

    void printMeshDebugInfo() const;

private:
    void setNull();
    void constructProperties();

    void extendFinalizeFromProperties() override;
    void extendScale(const SimTK::State& s, const ScaleSet& scaleSet) override;
    void extendConnectToModel(Model& model) override;

    void initializeMesh();
    std::string findMeshFile(const std::string& file);

    void createObbTree
        (OBBTreeNode& node, const SimTK::PolygonalMesh& mesh,
        const SimTK::Array_<int>& faceIndices);

    void splitObbAxis(const SimTK::PolygonalMesh& mesh,
        const SimTK::Array_<int>& parentIndices,
        SimTK::Array_<int>& child1Indices,
        SimTK::Array_<int>& child2Indices, int axis);

    void computeVariableThickness();

    // Member Variables
    SimTK::PolygonalMesh _mesh;
    SimTK::PolygonalMesh _mesh_back;
    SimTK::Vector_<SimTK::Vec3> _tri_center;
    SimTK::Vector_<SimTK::UnitVec3> _tri_normal;
    SimTK::Vector _tri_area;
    std::vector<std::vector<int>> _regional_tri_ind;
    std::vector<int> _regional_n_tri;
    std::vector<std::set<int>> _tri_neighbors;
    SimTK::Vector_<SimTK::Vec3> _vertex_locations;
    SimTK::Matrix_<SimTK::Vec3> _face_vertex_locations;
    SimTK::Vector _tri_thickness;
    SimTK::Vector _tri_elastic_modulus;
    SimTK::Vector _tri_poissons_ratio;
    bool _mesh_is_cached;


    // We cache the DecorativeMeshFile if we successfully
    // load the mesh from file so we don't try loading from disk every frame.
    // This is mutable since it is not part of the public interface.
    mutable SimTK::ResetOnCopy<std::unique_ptr<SimTK::DecorativeMeshFile>>
        _decorative_mesh;

//=========================================================================
//                            OBB TREE NODE
//=========================================================================
#ifndef SWIG
public:
    class OBBTreeNode {
        public:
            OBBTreeNode() : _child1(nullptr), _child2(nullptr), _numTriangles(0) {
            }
            OBBTreeNode(const OBBTreeNode& copy);
            ~OBBTreeNode();
                
            bool rayIntersectOBB(
                const SimTK::PolygonalMesh& mesh,
                const SimTK::Vec3& origin,
                const SimTK::UnitVec3& direction,
                int& tri_index, SimTK::Vec3& intersection_point,
                double& distance) const;

            bool rayIntersectTri(
                const SimTK::PolygonalMesh& mesh,
                SimTK::Vec3 origin, SimTK::Vec3 direction,
                int tri_index,
                SimTK::Vec3& intersection_pt, double& distance) const;

            const SimTK::OrientedBoundingBox& getBounds() const ;
            bool isLeafNode() const;
            const OBBTreeNode getFirstChildNode() const ;
            const OBBTreeNode getSecondChildNode() const;
            const SimTK::Array_<int>& getTriangles() const;
            int getNumTriangles() const;

            SimTK::OrientedBoundingBox _bounds;
            OBBTreeNode* _child1;
            OBBTreeNode* _child2;
            SimTK::Array_<int> _triangles;
            int _numTriangles;

    };// END of class OBBTreeNode

    OBBTreeNode _obb;
    OBBTreeNode _back_obb;
#endif //SWIG

    //=========================================================================
};  // END of class ContactGeometry
    //=========================================================================
} // end of namespace OpenSim

#endif // OPENSIM_SMITH2018_CONTACT_MESH_H_
