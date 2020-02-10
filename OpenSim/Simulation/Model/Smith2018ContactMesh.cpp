/* -------------------------------------------------------------------------- *
 *                      Smith2018ContactMesh.cpp                              *
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

#include "Smith2018ContactMesh.h"
#include <OpenSim/Common/ScaleSet.h>
#include "OpenSim/Common/Object.h"
#include "OpenSim/Simulation/SimbodyEngine/Body.h"
#include "OpenSim/Simulation/Model/Model.h"
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include "simmath/internal/ContactGeometry.h"
#include "simmath/internal/OrientedBoundingBox.h"
#include "simmath/internal/OBBTree.h"
#include <set>
#include <cmath>

using namespace OpenSim;

using std::set;

//=============================================================================
// CONSTRUCTOR
//=============================================================================

Smith2018ContactMesh::Smith2018ContactMesh() 
{
    setNull();
    constructProperties();
    _mesh_is_cached = false;
}

Smith2018ContactMesh::Smith2018ContactMesh(const std::string& name, 
    const std::string& mesh_file, const PhysicalFrame& parent_frame) :
    ContactGeometry (parent_frame)
{
    setNull();
    constructProperties();
    _mesh_is_cached = false;

    setName(name);
    set_mesh_file(mesh_file);
    updSocket<PhysicalFrame>("scale_frame").connect(parent_frame);
}

Smith2018ContactMesh::Smith2018ContactMesh(const std::string& name, 
    const std::string& mesh_file, const PhysicalFrame& frame,
    const SimTK::Vec3& location, const SimTK::Vec3& orientation) :
    ContactGeometry(location, orientation, frame)
{
    Smith2018ContactMesh(name, mesh_file, frame);
}

Smith2018ContactMesh::Smith2018ContactMesh(const std::string& name,
    const std::string& mesh_file, const PhysicalFrame& frame,
    const SimTK::Vec3& location, const SimTK::Vec3& orientation,
    const std::string& mesh_back_file, 
    double min_thickness, double max_thickness) :
    ContactGeometry(location, orientation, frame)
{
    Smith2018ContactMesh(name, mesh_file, frame, location, orientation);
    set_mesh_back_file(mesh_back_file);
    set_min_thickness(min_thickness);
    set_max_thickness(max_thickness);
}

void Smith2018ContactMesh::setNull()
{
    setAuthors("Colin Smith");
    setReferences(
        "Smith, C. R., Won Choi, K., Negrut, D., & Thelen, D. G. (2018)."
        "Efficient computation of cartilage contact pressures within dynamic "
        "simulations of movement. Computer Methods in Biomechanics and "
        "Biomedical Engineering: Imaging & Visualization, 6(5), 491-498.");
}

void Smith2018ContactMesh::constructProperties()
{
    constructProperty_mesh_file("");    
    constructProperty_mesh_back_file("");
    constructProperty_min_thickness(0.001);
    constructProperty_max_thickness(0.01);
    constructProperty_scale_factors(SimTK::Vec3(1.0));
}

void Smith2018ContactMesh::extendScale(
    const SimTK::State& s, const ScaleSet& scaleSet)
{
    Super::extendScale(s, scaleSet);
    SimTK::Vec3 scale_factors = getScaleFactors(scaleSet,
        getConnectee<PhysicalFrame>("scale_frame"));

    set_scale_factors(scale_factors);
    _mesh_is_cached = false;
}

void Smith2018ContactMesh::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    if (!_mesh_is_cached) {
        initializeMesh();

        if(!get_mesh_back_file().empty()){
            computeVariableCartilageThickness();
        }
    }
}

void Smith2018ContactMesh::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    //Add in a local frame for the mesh reprsenting the resulting translation
    // and orientation from this component's parent frame

    PhysicalOffsetFrame* mesh_frame = 
        new PhysicalOffsetFrame(getFrame(), SimTK::Transform());

    mesh_frame->setName("mesh_frame");
    mesh_frame->set_translation(get_location());
    mesh_frame->set_orientation(get_orientation());

    adoptSubcomponent(mesh_frame);
    setNextSubcomponentInSystem(*mesh_frame);
}

std::string Smith2018ContactMesh::findMeshFile(const std::string& file)
{
    /*This is a modified version of the code found in Geometry.cpp
    Mesh::extendFinalizeFromProperties to find geometry files in
    the same directory as the modelDir, modelDir/Geometry, or 
    the installDir: OPENSIM_HOME/Geometry

    It plays some games to figure out the modelDir, because the
    Smith2018ContactMesh can't call getModel() at this stage
    */

    bool isAbsolutePath; 
    std::string directory, fileName, extension;
    
    SimTK::Pathname::deconstructPathname(file, isAbsolutePath, directory,
        fileName, extension);
    const std::string lowerExtension = SimTK::String::toLower(extension);
    
    //Check for correct extension
    if (lowerExtension != ".vtp" && lowerExtension != ".obj" && 
        lowerExtension != ".stl") {

        std::cout << "Smith2018ContactMesh ERROR: '" << file << 
            "'; only .vtp .stl and .obj files currently supported.\n";

        OPENSIM_THROW(Exception,"Smith2018ContactMesh: Bad file type.");
    }

    // Find OpenSim modelDir
    const Component* rootModel = nullptr;
    if (!hasOwner()) {
        std::cout << "Mesh " << file << " not connected to model..ignoring\n";
        return file;   // Orphan Mesh not part of a model yet
    }
    const Component* parent = &getOwner();
    while (parent != nullptr) {
        if (dynamic_cast<const Model*>(parent) != nullptr) {
            rootModel = parent;
            break;
        }
        if (parent->hasOwner())
            parent = &(parent->getOwner()); // traverse up Component tree
        else
            break; // can't traverse up.
    }

    if (rootModel == nullptr) {
        std::cout << "Mesh " << file << " not connected to model..ignoring\n";
        return file;   // Orphan Mesh not descendent of a model
    }
    //const Model& model = dynamic_cast<const Model&>(*rootModel);
    std::string osimFileName = rootModel->getDocumentFileName();
    

    //Find geometry file
    Model model;
    model.setInputFileName(osimFileName);
    
    SimTK::Array_<std::string> attempts;

    bool foundIt = ModelVisualizer::findGeometryFile(model,
        file, isAbsolutePath, attempts);

    if (!foundIt) {
        std::cout << "Smith2018ContactMesh couldn't find file '" <<
            file << "'; tried\n";

        for (unsigned i = 0; i < attempts.size(); ++i)
            std::cout << "  " << attempts[i] << "\n";
        if (!isAbsolutePath && 
            !SimTK::Pathname::environmentVariableExists("OPENSIM_HOME")) {
            
            std::cout << "Set environment variable OPENSIM_HOME " <<
                "to search $OPENSIM_HOME/Geometry.\n";
        }
        throw OpenSim::Exception("Smith2018ContactMesh: " + getName() +
            "File NOT found: " + file);
    }

    return attempts.back();
}

void Smith2018ContactMesh::initializeMesh()
{
    _mesh_is_cached = true;

    // Initialize Reused Variables
 

    // Load Mesh from file
    std::string file = findMeshFile(get_mesh_file());
    _mesh.loadFile(file);

    //Scale Mesh
    SimTK::Real xscale = get_scale_factors()(0);
    SimTK::Real yscale = get_scale_factors()(1);
    SimTK::Real zscale = get_scale_factors()(2);

    SimTK::Rotation scale_rot;
    scale_rot.set(0, 0, xscale);
    scale_rot.set(1, 1, yscale);
    scale_rot.set(2, 2, zscale);
    SimTK::Transform scale_transform(scale_rot,SimTK::Vec3(0.0));
    _mesh.transformMesh(scale_transform);
    
    //Allocate space
    _tri_center.resize(_mesh.getNumFaces());
    _tri_normal.resize(_mesh.getNumFaces());
    _tri_area.resize(_mesh.getNumFaces());
    _tri_thickness.resize(_mesh.getNumFaces());
    _tri_elastic_modulus.resize(_mesh.getNumFaces());
    _tri_poissons_ratio.resize(_mesh.getNumFaces());

    _tri_thickness=-1;
    _tri_elastic_modulus=-1;
    _tri_poissons_ratio=-1;

    _vertex_locations.resize(_mesh.getNumVertices());
    _face_vertex_locations.resize(_mesh.getNumFaces(), 3);
        
    _regional_tri_ind.resize(6);    
    _regional_n_tri.assign(6,0);

    // Compute Mesh Properties
    //========================

    for (int i = 0; i < _mesh.getNumFaces(); ++i) {

        // Get Triangle Vertice Positions
        int v1_i = _mesh.getFaceVertex(i, 0);
        int v2_i = _mesh.getFaceVertex(i, 1);
        int v3_i = _mesh.getFaceVertex(i, 2);

        SimTK::Vec3 v1 = _mesh.getVertexPosition(v1_i);
        SimTK::Vec3 v2 = _mesh.getVertexPosition(v2_i);
        SimTK::Vec3 v3 = _mesh.getVertexPosition(v3_i);

        // Compute Triangle Center
        _tri_center(i)(0) = (v1(0) + v2(0) + v3(0)) / 3.0;
        _tri_center(i)(1) = (v1(1) + v2(1) + v3(1)) / 3.0;
        _tri_center(i)(2) = (v1(2) + v2(2) + v3(2)) / 3.0;

        // Compute Triangle Normal
        SimTK::Vec3 e1 = v3 - v1;
        SimTK::Vec3 e2 = v2 - v1;

        SimTK::Vec3 cross = SimTK::cross(e1, e2);

        double mag = sqrt(
            cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);

        for (int j = 0; j < 3; ++j) {
            _tri_normal(i).set(j,-cross[j] / mag);
        }

        
        // Compute Triangle Area
        // Compute length of each side of the triangle
        double s1 = 0;
        double s2 = 0;
        double s3 = 0;
                
        for (int j = 0; j<3; j++) {
            s1 += (v2[j] - v1[j])*(v2[j] - v1[j]);
            s2 += (v3[j] - v2[j])*(v3[j] - v2[j]);
            s3 += (v1[j] - v3[j])*(v1[j] - v3[j]);
        }

        s1 = sqrt(s1);
        s2 = sqrt(s2);
        s3 = sqrt(s3);

        // Now employ Heron's formula
        double s = (s1 + s2 + s3) / 2.0;
        _tri_area[i] = sqrt(s*(s - s1)*(s - s2)*(s - s3));
        
        //Determine regional triangle indices
        for (int j = 0; j < 3; ++j) {
            if (_tri_center(i)(j) < 0.0) {
                _regional_tri_ind[j*2].push_back(i);
                _regional_n_tri[j * 2]++;
            }
            else {
                _regional_tri_ind[j * 2 + 1].push_back(i);
                _regional_n_tri[j * 2 +1]++;
            }
        }  
    }

    //Vertex Locations
    for (int i = 0; i < _mesh.getNumVertices(); ++i) {
        _vertex_locations(i) = _mesh.getVertexPosition(i);
    }

    //Face Vertex Locations
    for (int i = 0; i < _mesh.getNumFaces(); ++i) {
        for (int j = 0; j < 3; ++j) {
            int v_ind = _mesh.getFaceVertex(i, j);
            _face_vertex_locations(i,j) = _mesh.getVertexPosition(v_ind);
        }
    }

    //Vertex Connectivity
    std::vector<std::vector<int>> ver_tri_ind(_mesh.getNumVertices());
    std::vector<int> ver_nTri(_mesh.getNumVertices());

    for (int i = 0; i < _mesh.getNumFaces(); ++i) {
        for (int j = 0; j < 3; ++j) {
            int ver = _mesh.getFaceVertex(i, j);
            ver_tri_ind[ver].push_back(i);
            ver_nTri[ver]++;
        }
    }

    //Triangle Neighbors
    _tri_neighbors.resize(_mesh.getNumFaces());

    for (int i = 0; i < _mesh.getNumFaces(); ++i) {
        for (int j = 0; j < 3; ++j) {

            int ver = _mesh.getFaceVertex(i, j);

            for (int k = 0; k < ver_nTri[ver]; ++k) {
                int tri = ver_tri_ind[ver][k];

                //triange can't be neighbor with itself
                if (tri == i) {
                    continue;
                }
                _tri_neighbors[i].insert(tri);
            }

        }
    }

    //Construct the OBB Tree
    SimTK::Array_<int> allFaces(_mesh.getNumFaces());
    for (int i = 0; i < _mesh.getNumFaces(); ++i) {
        allFaces[i] = i;}

    createObbTree(_obb, _mesh, allFaces);

    //Create Decorative Mesh
    _decorative_mesh.reset(new SimTK::DecorativeMeshFile(file));
    _decorative_mesh->setScaleFactors(get_scale_factors());
}

void Smith2018ContactMesh::computeVariableCartilageThickness() {

    // Get Mesh Properties
    double min_thickness = get_min_thickness();
    double max_thickness = get_max_thickness();

    // Load mesh_back_file
    std::string file = findMeshFile(get_mesh_back_file());
    _mesh_back.loadFile(file);

    //Scale _mesh_back
    SimTK::Real xscale = get_scale_factors()(0);
    SimTK::Real yscale = get_scale_factors()(1);
    SimTK::Real zscale = get_scale_factors()(2);

    SimTK::Rotation scale_rot;
    scale_rot.set(0, 0, xscale);
    scale_rot.set(1, 1, yscale);
    scale_rot.set(2, 2, zscale);
    SimTK::Transform scale_transform(scale_rot,SimTK::Vec3(0.0));
    _mesh_back.transformMesh(scale_transform);

    // Create OBB tree for back mesh
    SimTK::Array_<int> allFaces(_mesh_back.getNumFaces());

    for (int i = 0; i < _mesh_back.getNumFaces(); ++i) {
        allFaces[i] = i;
    }    

    createObbTree(_back_obb,_mesh_back, allFaces);

    //Loop through all triangles in cartilage mesh
    for (int i = 0; i < _mesh.getNumFaces(); ++i) {

        //Use mech_back OBB tree to find cartilage thickness
        //--------------------------------------------------

        int tri;
        SimTK::Vec3 intersection_point;
        double depth = 0.0;

        if (_back_obb.rayIntersectOBB(_mesh_back,
            _tri_center(i), -_tri_normal(i), tri, intersection_point, depth)) {

            if (depth < min_thickness) {
                depth = min_thickness;
            }
            if (depth > max_thickness) {
                depth = min_thickness;
            }
        }
        else{ //Normal from mesh missed mesh back
            depth = min_thickness;
        }
        _tri_thickness(i) = depth;
    }
}

void Smith2018ContactMesh::generateDecorations(
    bool fixed, const ModelDisplayHints& hints,const SimTK::State& s,
    SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const
{
    Super::generateDecorations(fixed, hints, s, geometry);

    // There is no fixed geometry to generate here.
    if (fixed) { return; }

    // Guard against the case where the Force was disabled
    // or mesh failed to load.
    if (_decorative_mesh == nullptr) return;
    if (!hints.get_show_contact_geometry()) return;
    
    const Frame& myFrame = getMeshFrame();
    const Frame& bFrame = myFrame.findBaseFrame();
    const PhysicalFrame* bPhysicalFrame =
        dynamic_cast<const PhysicalFrame*>(&bFrame);
    if (bPhysicalFrame == nullptr) {
        // throw exception something is wrong
        throw (Exception("Frame for Geometry " + getName() +
            " is not attached to a PhysicalFrame."));
    }
     
    SimTK::MobilizedBodyIndex mbidx = bPhysicalFrame->getMobilizedBodyIndex();
    SimTK::Transform transformInBaseFrame = myFrame.findTransformInBaseFrame();
    
    _decorative_mesh->setBodyId(mbidx);
    _decorative_mesh->setTransform(transformInBaseFrame);
    _decorative_mesh->setIndexOnBody(0);        
}


void Smith2018ContactMesh::createObbTree
    (OBBTreeNode& node, const SimTK::PolygonalMesh& mesh, 
    const SimTK::Array_<int>& faceIndices)
{   // Find all vertices in the node and build the OrientedBoundingBox.
    node._numTriangles = faceIndices.size();
    
    set<int> vertexIndices;
    for (int i = 0; i < (int)faceIndices.size(); i++) {        
        for (int j = 0; j < 3; j++) {
            vertexIndices.insert(mesh.getFaceVertex(faceIndices[i], j));
        }
    }
    SimTK::Vector_<SimTK::Vec3> points((int)vertexIndices.size());
    int index = 0;
    for (set<int>::iterator iter = vertexIndices.begin();
        iter != vertexIndices.end(); ++iter) {
        points[index++] = mesh.getVertexPosition(*iter);
        
    }
    node._bounds = SimTK::OrientedBoundingBox(points);
    if (faceIndices.size() > 3) {

        // Order the axes by size.

        int axisOrder[3];
        const SimTK::Vec3& size = node._bounds.getSize();
        if (size[0] > size[1]) {
            if (size[0] > size[2]) {
                axisOrder[0] = 0;
                if (size[1] > size[2]) {
                    axisOrder[1] = 1;
                    axisOrder[2] = 2;
                }
                else {
                    axisOrder[1] = 2;
                    axisOrder[2] = 1;
                }
            }
            else {
                axisOrder[0] = 2;
                axisOrder[1] = 0;
                axisOrder[2] = 1;
            }
        }
        else if (size[0] > size[2]) {
            axisOrder[0] = 1;
            axisOrder[1] = 0;
            axisOrder[2] = 2;
        }
        else {
            if (size[1] > size[2]) {
                axisOrder[0] = 1;
                axisOrder[1] = 2;
            }
            else {
                axisOrder[0] = 2;
                axisOrder[1] = 1;
            }
            axisOrder[2] = 0;
        }

        // Try splitting along each axis.

        for (int i = 0; i < 3; i++) {
            SimTK::Array_<int> child1Indices, child2Indices;
            splitObbAxis(mesh,faceIndices, child1Indices, child2Indices, 
                         axisOrder[i]);
            if (child1Indices.size() > 0 && child2Indices.size() > 0) {
                // It was successfully split, so create the child nodes.

                node._child1 = new OBBTreeNode();
                node._child2 = new OBBTreeNode();
                createObbTree(*node._child1, mesh, child1Indices);
                createObbTree(*node._child2, mesh, child2Indices);
                return;
            }
        }
    }
    
    // This is a leaf node.    
    node._triangles.insert(node._triangles.begin(), faceIndices.begin(), 
                          faceIndices.end());
}

void Smith2018ContactMesh::splitObbAxis
   (const SimTK::PolygonalMesh& mesh, const SimTK::Array_<int>& parentIndices,
       SimTK::Array_<int>& child1Indices, SimTK::Array_<int>& child2Indices,
       int axis) 
{   // For each face, find its minimum and maximum extent along the axis.
    SimTK::Vector minExtent(parentIndices.size());
    SimTK::Vector maxExtent(parentIndices.size());
    SimTK::Array_<int> vertexIndices(3);
    for (int i = 0; i < (int) parentIndices.size(); i++) {
        for (int j = 0; j < 3; j++) {
            vertexIndices[j] = mesh.getFaceVertex(parentIndices[i], j);
        }
        SimTK::Real minVal = mesh.getVertexPosition(vertexIndices[0])(axis);
        SimTK::Real maxVal = mesh.getVertexPosition(vertexIndices[0])(axis);
        minVal =
            std::min(minVal, mesh.getVertexPosition(vertexIndices[1])(axis));
        maxVal =
            std::max(maxVal, mesh.getVertexPosition(vertexIndices[1])(axis));
        minExtent[i] =
            std::min(minVal, mesh.getVertexPosition(vertexIndices[2])(axis));
        maxExtent[i] =
            std::max(maxVal, mesh.getVertexPosition(vertexIndices[2])(axis));
    }
    
    // Select a split point that tries to put as many faces as possible 
    // entirely on one side or the other.
    
    SimTK::Real split = (median(minExtent)+median(maxExtent)) / 2;
    
    // Choose a side for each face.
    
    for (int i = 0; i < (int) parentIndices.size(); i++) {
        if (maxExtent[i] <= split)
            child1Indices.push_back(parentIndices[i]);
        else if (minExtent[i] >= split)
            child2Indices.push_back(parentIndices[i]);
        else if (0.5*(minExtent[i]+maxExtent[i]) <= split)
            child1Indices.push_back(parentIndices[i]);
        else
            child2Indices.push_back(parentIndices[i]);
    }
}

bool Smith2018ContactMesh::rayIntersectMesh(
    const SimTK::Vec3& origin, const SimTK::UnitVec3& direction,
    const double& min_proximity, const double& max_proximity, 
    int& tri, SimTK::Vec3 intersection_point, SimTK::Real& distance) const {

    double obb_distance=-1;
    SimTK::Array_<int> obb_triangles;

    if (_obb.rayIntersectOBB(_mesh, origin, direction, tri,
        intersection_point, distance)) {

        if ((distance > min_proximity) && (distance < max_proximity)) {
            return true;
        }
    }

    //Shoot the ray in the opposite direction
    if (min_proximity < 0.0) {        
        if (_obb.rayIntersectOBB(_mesh, origin, -direction, tri,
            intersection_point, distance)) {

            distance = -distance;
            if ((distance > min_proximity) && (distance < max_proximity)) {
                return true;
            }
        }
    }
    
    //ray didn't intersect
    distance = -1;
    intersection_point = -1;
    return false;
}

//=============================================================================
//               Smith2018ContactMesh :: OBBTreeNode
//=============================================================================
Smith2018ContactMesh::OBBTreeNode::OBBTreeNode(const OBBTreeNode& copy):
   _bounds(copy._bounds), _triangles(copy._triangles), 
    _numTriangles(copy._numTriangles) {
    if (copy._child1 == NULL) {
        _child1 = NULL;
        _child2 = NULL;
    }
    else {
        _child1 = new OBBTreeNode(*copy._child1);
        _child2 = new OBBTreeNode(*copy._child2);
    }
}

Smith2018ContactMesh::OBBTreeNode::~OBBTreeNode() {
    if (_child1 != NULL)
        delete _child1;
    if (_child2 != NULL)
        delete _child2;
}

bool Smith2018ContactMesh::OBBTreeNode::rayIntersectOBB(
    const SimTK::PolygonalMesh& mesh,
    const SimTK::Vec3& origin, const SimTK::UnitVec3& direction,
    int& tri_index, SimTK::Vec3& intersection_point, double& distance) const
{

    if (_child1 != NULL) {
        // Recursively check the child nodes.
        SimTK::Real child1distance, child2distance;
        SimTK::Array_<int> child1triangles, child2triangles;
        bool child1intersects = 
            _child1->_bounds.intersectsRay(origin, direction, child1distance);
        bool child2intersects = 
            _child2->_bounds.intersectsRay(origin, direction, child2distance);
        
        if (child1intersects) {
            if (child2intersects) {
                // The ray intersects both child nodes.
                // First check the closer one.

                if (child1distance < child2distance) {
                    child1intersects =_child1->rayIntersectOBB(
                        mesh, origin, direction, tri_index,
                        intersection_point, child1distance);

                    if (!child1intersects || child2distance < child1distance)
                        child2intersects = _child2->rayIntersectOBB(
                            mesh,origin, direction, tri_index,
                            intersection_point, child2distance);
                }
                else {
                    child2intersects =_child2->rayIntersectOBB(
                        mesh, origin, direction, tri_index,
                        intersection_point, child2distance);

                    if (!child2intersects || child1distance < child2distance)
                        child1intersects =_child1->rayIntersectOBB(
                            mesh, origin, direction, tri_index,
                            intersection_point, child1distance);
                }
            }
            else
                child1intersects = _child1->rayIntersectOBB(mesh, origin,
                    direction, tri_index, intersection_point, child1distance);
        }
        else if (child2intersects)
            child2intersects = _child2->rayIntersectOBB(mesh, origin,
                direction, tri_index, intersection_point, child2distance);

        // If either one had an intersection, return the closer one.  

        if (child1intersects){
            if (!child2intersects || child1distance < child2distance) {
                distance = child1distance;
                return true;
            }
        }
        if (child2intersects) {
            distance = child2distance;
            return true;
        }
        return false;
    }

    //Reached a leaf node, check all containing triangles
    for (int i = 0; i < (int)_triangles.size(); i++) {
        if (rayIntersectTri(mesh, origin, direction, _triangles[i],
            intersection_point, distance)) {

            tri_index = _triangles[i];
            return true;
        }
    }
    return false;
}

bool Smith2018ContactMesh::OBBTreeNode::rayIntersectTri(
    const SimTK::PolygonalMesh& mesh,
    SimTK::Vec3 origin, SimTK::Vec3 direction, 
    int tri_index,
    SimTK::Vec3& intersection_pt, double& distance) const
{
    
    
/* 
origin - reference point of casting ray 
        (i.e. center of triangle from which ray is cast)

direction - casting ray direction vector 
            (i.e. normal to triangle from which ray is cast)

tri_index - The triangle index of the test target
    
www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/
*/  
    int v0_ind = mesh.getFaceVertex(tri_index, 0);
    int v1_ind = mesh.getFaceVertex(tri_index, 1);
    int v2_ind = mesh.getFaceVertex(tri_index, 2);
    

    SimTK::Vec3 v0 = mesh.getVertexPosition(v0_ind);
    SimTK::Vec3 v1 = mesh.getVertexPosition(v1_ind);
    SimTK::Vec3 v2 = mesh.getVertexPosition(v2_ind);

   
    SimTK::Vec3 e1, e2, h, s, q;
    double a, f, u, v;

    // find triangle edges
    for (int i = 0; i < 3; ++i) {
        e1(i) = v1(i) - v0(i);
        e2(i) = v2(i) - v0(i);
    }

    h = SimTK::cross(direction, e2);
    a = dot(e1, h);

    //If ray is perpendicular to the triangle, no interestion
    //this should be adjusted for precision, a=0 when e1 and h are pependicular

    if (a > -0.00000001 && a < 0.00000001) {
        return(false);
    }

    // Else on to second test
    // find triangle edges
    f = 1 / a;
    for (int i = 0; i < 3; i++) {
        s(i) = origin(i) - v0(i);
    }

    u = f * SimTK::dot(s, h);
    if (u < 0 || u > 1.0)
        return(false);

    q = SimTK::cross(s, e1);

    v = f*SimTK::dot(direction, q);
    double w = 1 - u - v;
    
    if (v < 0.0 || w < 0.0)
        return(false);
    
    
    else {
        // else there is a line intersection
        // at this stage we can compute the distance to the intersection
        // point on the line
        //     point(t) = p + t * d
        //  where
        //      p is a point in the line
        //      d is a vector that provides the line's direction
        //      t is the distance
        intersection_pt = w * v0 + u * v1 + v * v2;
        distance = f * SimTK::dot(e2, q);
        return(true);
    }
}

const SimTK::OrientedBoundingBox& 
Smith2018ContactMesh::OBBTreeNode::getBounds() const {
    return _bounds;
}

bool Smith2018ContactMesh::OBBTreeNode::isLeafNode() const {
    return (_child1 == NULL);
}

const Smith2018ContactMesh::OBBTreeNode 
Smith2018ContactMesh::OBBTreeNode::getFirstChildNode() const {
    SimTK_ASSERT_ALWAYS(_child1, 
        "Called getFirstChildNode() on a leaf node");
    return OBBTreeNode(*_child1);
}

const Smith2018ContactMesh::OBBTreeNode 
Smith2018ContactMesh::OBBTreeNode::getSecondChildNode() const {
    SimTK_ASSERT_ALWAYS(_child2, 
        "Called getFirstChildNode() on a leaf node");
    return OBBTreeNode(*_child2);
}

const SimTK::Array_<int>& 
Smith2018ContactMesh::OBBTreeNode::getTriangles() const {
    SimTK_ASSERT_ALWAYS(_child2 == NULL, 
        "Called getTriangles() on a non-leaf node");
    return _triangles;
}

int Smith2018ContactMesh::OBBTreeNode::getNumTriangles() const {
    return _numTriangles;
}

