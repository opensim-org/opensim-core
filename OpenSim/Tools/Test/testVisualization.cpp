/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testVisualization.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

#include <fstream>
#include <stdint.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;
using namespace SimTK;

void testVisModel(Model& model, const std::string filename_for_standard);
Model createModel4AppearanceTest();
void populate_doublePendulumPrimitives(SimTK::Array_<DecorativeGeometry>&); 
void populate_composedTransformPrimitives(SimTK::Array_<DecorativeGeometry>&);
void populate_contactModelPrimitives(SimTK::Array_<DecorativeGeometry>&);
void populate_wrapModelPrimitives(SimTK::Array_<DecorativeGeometry>&, bool includeFrames=true);
bool testVisModelAgainstStandard(Model& model, const SimTK::Array_<DecorativeGeometry>& stdPrimitives);

// Implementation of DecorativeGeometryImplementation that prints the representation to 
// a StringStream for comparison
class DecorativeGeometryImplementationText : public SimTK::DecorativeGeometryImplementation
{

    void implementPointGeometry(const DecorativePoint& dp) override{
        printout << "DecorativePoint:" << dp.getPoint() << printCommonProps(dp) << std::endl;
    };
    void implementLineGeometry(const DecorativeLine& dl) override{
        printout << "DecorativeLine:" << dl.getPoint1() << dl.getPoint2() << printCommonProps(dl) << std::endl;
    };
    void implementBrickGeometry(const DecorativeBrick& db) override{
        printout << "DecorativeBrick:" << db.getHalfLengths() << printCommonProps(db) << std::endl;
    };
    void implementCylinderGeometry(const DecorativeCylinder& dc) override{
        printout << "DecorativeCylinder:" << dc.getHalfHeight() << dc.getRadius() << printCommonProps(dc) << std::endl;
    };
    void implementCircleGeometry(const DecorativeCircle& dc) override{
        printout << "DecorativeCircle:" << dc.getRadius() << printCommonProps(dc) << std::endl;
    };
    void implementSphereGeometry(const DecorativeSphere& dp) override{
        printout << "DecorativeSphere:" << dp.getRadius() << printCommonProps(dp) << std::endl;
    };
    void implementEllipsoidGeometry(const DecorativeEllipsoid& dp) override{
        printout << "DecorativeEllipsoid:" << dp.getRadii() << printCommonProps(dp) << std::endl;
    };
    void implementFrameGeometry(const DecorativeFrame& dp) override{
        printout << "DecorativeFrame:" << dp.getAxisLength() << printCommonProps(dp) << std::endl;
    };
    void implementTextGeometry(const DecorativeText& dp) override{
        printout << "DecorativeText:" << dp.getText() << printCommonProps(dp) << std::endl;
    };
    void implementMeshGeometry(const DecorativeMesh& dp) override{
        printout << "DecorativeMesh:" << dp.getMesh().getNumFaces() << " " << dp.getMesh().getNumVertices() << printCommonProps(dp) << std::endl;
    };
    void implementMeshFileGeometry(const DecorativeMeshFile& dp) override{
        std::string filename = dp.getMeshFile();
        std::size_t found = filename.find_last_of("/\\");
        if (found == string::npos)
            printout << "DecorativeMeshFile:" << filename << " " << printCommonProps(dp) << std::endl;
        else
            printout << "DecorativeMeshFile:" << filename.substr(found+1) << " " << printCommonProps(dp) << std::endl;

    };
    void implementArrowGeometry(const DecorativeArrow& dp) override{
        printout << "DecorativeArrow:" << dp.getStartPoint() << dp.getEndPoint() << dp.getTipLength() << printCommonProps(dp) << std::endl;
    };
    void implementTorusGeometry(const DecorativeTorus& dp) override{
        printout << "DecorativeTorus:" << dp.getTorusRadius() << dp.getTubeRadius() << printCommonProps(dp) << std::endl;
    };
    void implementConeGeometry(const DecorativeCone& dp) override{
        printout << "DecorativeTorus:" << dp.getBaseRadius() << dp.getDirection() << dp.getHeight() << printCommonProps(dp) << std::endl;
    };

public:
    std::string getAsString() {
        return printout.str();
    }
    void setPrintTransforms(bool toPrint) { printTransform = toPrint;}
private:
    std::stringstream printout;
    bool printTransform = false; // Flag to indicate whether or not to output transform as String
    std::string printCommonProps(const DecorativeGeometry& dg){
        std::stringstream oneDGStream;
        oneDGStream << " bodyId:" << dg.getBodyId() << " color:" << dg.getColor() << " indexOnBody:"
            << dg.getIndexOnBody() << " Opacity:" << dg.getOpacity() << " Rep:" << dg.getRepresentation() << " Scale:"
            << dg.getScaleFactors();
        if (printTransform)
            oneDGStream << " Transform:" << dg.getTransform();
        return oneDGStream.str();
    }

};

int main()
{
    try {
        LoadOpenSimLibrary("osimActuators");
        
        Model testModel("BuiltinGeometry.osim");
        testVisModel(testModel, "vis_BuiltinGeometry.txt");
        std::cout << "BuiltinGeometry Passed" << std::endl;
        Model testModel2 = createModel4AppearanceTest();
        testVisModel(testModel2, "vis_AppearanceTest.txt");
        std::cout << "Appearance test Passed" << std::endl;
        // Load Model in 3.3 format that had transforms attached to Geometry
        Model testModel3("double_pendulum33.osim");
        
        SimTK::Array_<DecorativeGeometry> standard;
        testModel3.updDisplayHints().set_show_frames(true);
        populate_doublePendulumPrimitives(standard);
        testVisModelAgainstStandard(testModel3, standard);
        std::cout << "double_pendulum33 test Passed" << std::endl;

        // Now a model from 3.3 where both GeometrySet and individual DisplayGeometry 
        // have a non-trivial transform.
        Model composedTransformsModel("doubletransform33.osim");
        composedTransformsModel.updDisplayHints().set_show_frames(true);
        populate_composedTransformPrimitives(standard);
        testVisModelAgainstStandard(composedTransformsModel, standard);

        // Model with contacts
        Model modelWithContacts("visualize_contacts.osim");
        modelWithContacts.updDisplayHints().set_show_frames(true);
        populate_contactModelPrimitives(standard);
        testVisModelAgainstStandard(modelWithContacts, standard);
        
        // Model with WrapObjects
        Model modelWithWrap("test_wrapAllVis.osim");
        modelWithWrap.updDisplayHints().set_show_frames(true);
        populate_wrapModelPrimitives(standard);
        modelWithWrap.updDisplayHints().set_show_frames(false);
        populate_wrapModelPrimitives(standard, false);
        testVisModelAgainstStandard(modelWithWrap, standard);
    }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void testVisModel(Model& model, const std::string standard_filename)
{
    bool visualDebug = false; // Turn on only if you want to see API visualizer live
    if (visualDebug) 
        model.setUseVisualizer(true);
    SimTK::State& si = model.initSystem();

    // Compute muscle dynamics to evaluate Muscle color
    model.realizeDynamics(si);
    
    if (visualDebug) 
        model.getVisualizer().show(si);
    ModelDisplayHints mdh; 
    mdh.upd_show_frames() = true;
    SimTK::Array_<SimTK::DecorativeGeometry> geometryToDisplay;
    model.generateDecorations(true, mdh, si, geometryToDisplay);
    cout << geometryToDisplay.size() << endl;
    model.generateDecorations(false, mdh, si, geometryToDisplay);
    cout << geometryToDisplay.size() << endl;
    DecorativeGeometryImplementationText dgiText;
    dgiText.setPrintTransforms(true); // In toy models Transforms can be printed and compared as strings
    for (unsigned i = 0; i < geometryToDisplay.size(); i++)
        geometryToDisplay[i].implementGeometry(dgiText);

    std::ifstream t(standard_filename);
    if (!t.good()) throw OpenSim::Exception("Could not open file. "+ standard_filename);
    std::stringstream buffer;
    buffer << t.rdbuf();
    std::string fromFile = buffer.str();
    std::string fromModel = dgiText.getAsString();
    cout << "From Model " << endl << "=====" << endl;
    cout << fromModel << endl;
    cout << "From File " << endl << "=====" << endl;
    cout << fromFile << endl;
    cout << "Length:" << fromModel.length() << "vs." << fromFile.length() << std::endl;
    int same = fromFile.compare(fromModel);

    if (visualDebug) {
        std::cout << "press Enter (or Return) to continue" << std::endl;
        std::cin.get();
        std::cout << "Continuing..." << std::endl;
    }

    ASSERT(same == 0, __FILE__, __LINE__, 
        "Visualization primitives from model do not match standard from file `"
        + standard_filename + "'.");
}

Model createModel4AppearanceTest()
{
    Model modelWithGroundOnly;
    Sphere* unitSphere = new Sphere(1.0);
    const Ground& ground = modelWithGroundOnly.getGround();
    //DecorativeGeometry default is Rep : 3 shaded in std file
    modelWithGroundOnly.updGround().attachGeometry(unitSphere);
    // Create offset frame and add to model
    SimTK::Transform translate(Vec3(1.0, 0., 0.));
    PhysicalOffsetFrame* oframe = new PhysicalOffsetFrame(ground, translate);
    modelWithGroundOnly.addComponent(oframe);
    // Change color and opacity
    Sphere* offsetSphere = unitSphere->clone();
    offsetSphere->upd_Appearance().set_color(SimTK::Cyan);
    offsetSphere->upd_Appearance().set_opacity(0.5);
    oframe->attachGeometry(offsetSphere);
    PhysicalOffsetFrame* ooframe = new PhysicalOffsetFrame(ground, Vec3(2.0, 0., 0.));
    modelWithGroundOnly.addComponent(ooframe);
    // invisible Sphere
    Sphere* ooffsetSphere = unitSphere->clone();
    // Hidden   
    ooffsetSphere->upd_Appearance().set_visible(false);
    ooframe->attachGeometry(ooffsetSphere);
    PhysicalOffsetFrame* oooframe = new PhysicalOffsetFrame(ground, Vec3(3.0, 0., 0.));
    modelWithGroundOnly.addComponent(oooframe);
    // Wireframe Sphere
    Sphere* oooffsetSphere = unitSphere->clone();
    //DecorativeGeometry::DrawWireframe is Rep : 2 in std file
    oooffsetSphere->upd_Appearance().set_representation(DecorativeGeometry::DrawWireframe);
    oooframe->attachGeometry(oooffsetSphere);
    return modelWithGroundOnly; // Return a copy
}
// Eventually all tests will go thru this function instead of comparing files then reference
// can be passed in.
bool testVisModelAgainstStandard(Model& model, const SimTK::Array_<DecorativeGeometry>& stdPrimitives) {
    bool visualDebug = false; // Turn on only if you want to see API visualizer live
    if (visualDebug)
        model.setUseVisualizer(true);
    SimTK::State& si = model.initSystem();
    if (visualDebug)
        model.getVisualizer().show(si);
    const ModelDisplayHints& mdh = model.getDisplayHints();
    SimTK::Array_<SimTK::DecorativeGeometry> geometryToDisplay;
    model.generateDecorations(true, mdh, si, geometryToDisplay);
    cout << "Number of fixed geometries: " << geometryToDisplay.size() << endl;
    model.generateDecorations(false, mdh, si, geometryToDisplay);
    cout << "Number of fixed and non-fixed geometries: "
         << geometryToDisplay.size() << endl;
    /*
    DecorativeGeometryImplementationText textFromModel;
    textFromModel.setPrintTransforms(true); // Debugging
    for (const SimTK::DecorativeGeometry* nextGeom = geometryToDisplay.begin();
    nextGeom != geometryToDisplay.end();
        ++nextGeom) {
        nextGeom->implementGeometry(textFromModel);
    }
    std::cout << textFromModel.getAsString() << std::endl;
    */
    int i = 0;
    for (const SimTK::DecorativeGeometry* nextGeom = stdPrimitives.begin();
        nextGeom != stdPrimitives.end();
        ++nextGeom) {
            DecorativeGeometryImplementationText dgiTextFromStandard;
            nextGeom->implementGeometry(dgiTextFromStandard);
            DecorativeGeometryImplementationText dgiTextFromModel;
            geometryToDisplay[i].implementGeometry(dgiTextFromModel);
            if (!(dgiTextFromStandard.getAsString() == dgiTextFromModel.getAsString()))
                throw  OpenSim::Exception("failed comparing " + dgiTextFromStandard.getAsString() + "vs." + dgiTextFromModel.getAsString());
            // Compare transforms, this has to be more lenient than String comparison due to roundoff
            SimTK::Mat44 diffTransform = nextGeom->getTransform().toMat44() - geometryToDisplay[i].getTransform().toMat44();
            // std::cout << "Transform from standard: "
            //      << nextGeom->getTransform().toMat44() << std::endl;
            // std::cout << "Transform from model: "
            //      << geometryToDisplay[i].getTransform().toMat44() << std::endl;
            double norm = diffTransform.norm();
            SimTK_TEST_EQ(norm, 0.)
             ++i;
    }
    if (visualDebug) {
        std::cout << "press Enter (or Return) to continue" << std::endl;
        std::cin.get();
        std::cout << "Continuing..." << std::endl;
    }
    return true;
}

void populate_doublePendulumPrimitives(SimTK::Array_<DecorativeGeometry>& stdPrimitives) {
    stdPrimitives.clear();
    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(0).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));
    // Offset frame rod1
    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(1).setColor(SimTK::White).setIndexOnBody(0).setScale(0.2)
        .setOpacity(1).setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(SimTK::Transform(Vec3{ 0., .25, 0 })));
    // Cylinder rod1
    stdPrimitives.push_back(
        DecorativeMeshFile("cylinder.vtp").setBodyId(1).setColor(SimTK::White)
        .setIndexOnBody(0).setScaleFactors(Vec3{ 0.02,0.5,0.02 }).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(SimTK::Transform(Vec3{ 0., .25, 0 })));
    // Frame rod 1
    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(1).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));
    // Block rod 1
    stdPrimitives.push_back(
        DecorativeMeshFile("block.vtp").setBodyId(1).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(1).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));

    // Offset frame rod2
    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(2).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(SimTK::Transform(Vec3{ 0., .25, 0 })));
    // Cylinder rod2
    stdPrimitives.push_back(
        DecorativeMeshFile("cylinder.vtp").setBodyId(2).setColor(SimTK::White)
        .setIndexOnBody(0).setScaleFactors(Vec3{ 0.02,0.5,0.02 }).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(SimTK::Transform(Vec3{ 0., .25, 0 })));
    // Frame body 2
    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(2).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));
    // Block rod 2
    stdPrimitives.push_back(
        DecorativeMeshFile("block.vtp").setBodyId(2).setColor(SimTK::White)
        .setIndexOnBody(0).setScaleFactors(Vec3{ 1, 1.5, 2 }).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));

    // Two more offset frames and 2 for the Joint
    stdPrimitives.push_back(
        DecorativeFrame(1).setBodyId(0).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(SimTK::Transform(Vec3{ 0., 0, 0 })));
    stdPrimitives.push_back(
        DecorativeFrame(1).setBodyId(1).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(SimTK::Transform(Vec3{ 0., .5, 0 })));
    stdPrimitives.push_back(
        DecorativeFrame(1).setBodyId(1).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(SimTK::Transform(Vec3{ 0., 0, 0 })));
    stdPrimitives.push_back(
        DecorativeFrame(1).setBodyId(2).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(SimTK::Transform(Vec3{ 0., .5, 0 })));

}

void populate_composedTransformPrimitives(SimTK::Array_<DecorativeGeometry>& stdPrimitives) {
    stdPrimitives.clear();
    // In addition to Ground Frame, those frames for Joint
    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(0).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));
    // This the frame of the composed transform and attached geometry
    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(1).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(SimTK::Transform(Vec3{ 0.3, 0.3, 0.3 })));
    stdPrimitives.push_back(
        DecorativeMeshFile("block.vtp").setBodyId(1).setColor(SimTK::White)
        .setIndexOnBody(0).setScaleFactors(Vec3{ 1, 2, 3 }).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(SimTK::Transform(Vec3{ 0.3, 0.3, 0.3 })));

    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(1).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));


    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(0).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(SimTK::Transform(Vec3{ 0., 0.05, 0. })));
    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(1).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(SimTK::Transform(Vec3{ 0., 0.5, 0. })));
}

void populate_contactModelPrimitives(SimTK::Array_<DecorativeGeometry>& stdPrimitives) {
    stdPrimitives.clear();
    // Frame for Ground
    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(0).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));
    // Frame for Ball Body
    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(1).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));
    // Frames for the Joint
    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(0).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));
    stdPrimitives.push_back(
        DecorativeFrame(1.0).setBodyId(1).setColor(SimTK::White)
        .setIndexOnBody(0).setScale(0.2).setOpacity(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));
     // 2 Contact Surfaces as Meshes (from sphere.vtp)
    SimTK::PolygonalMesh mesh;
    mesh.loadFile("sphere.vtp");
    stdPrimitives.push_back(
        DecorativeMesh(mesh).setBodyId(0).setColor(SimTK::Cyan)
        .setIndexOnBody(-1).setOpacity(1).setScale(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawWireframe)
        .setTransform(SimTK::Transform(Vec3{ 1, 2, 0. })));
    stdPrimitives.push_back(
        DecorativeMesh(mesh).setBodyId(1).setColor(SimTK::Cyan)
        .setIndexOnBody(-1).setOpacity(1).setScale(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawWireframe)
        .setTransform(SimTK::Transform(Vec3{ 1, 1, 0. })));
    // ContactSphere
    stdPrimitives.push_back(
        DecorativeSphere(0.25).setBodyId(1).setColor(SimTK::Cyan)
        .setIndexOnBody(-1).setOpacity(1).setScale(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawWireframe)
        .setTransform(SimTK::Transform(Vec3{ 0, 1, 0. })));
    // ContactHalfSpace as thin block
    SimTK::Transform transform;
    transform.updR().setRotationFromAngleAboutZ(.5);
    stdPrimitives.push_back(
        DecorativeBrick({ 0.0005,0.5,0.5 }).setBodyId(0).setColor(SimTK::Cyan)
        .setIndexOnBody(-1).setOpacity(0.7).setScale(1)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(transform * Transform(Vec3(0.0005, 0, 0))));

}

void populate_wrapModelPrimitives(SimTK::Array_<DecorativeGeometry>& stdPrimitives, bool includeFrames) {
    stdPrimitives.clear();
    // Frame for Ground & Bodies
    if (includeFrames) {
        stdPrimitives.push_back(
            DecorativeFrame(1.0).setBodyId(0).setColor(SimTK::White)
            .setIndexOnBody(0).setScale(0.2).setOpacity(1)
            .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));
        stdPrimitives.push_back(
            DecorativeFrame(1.0).setBodyId(1).setColor(SimTK::White)
            .setIndexOnBody(0).setScale(0.2).setOpacity(1)
            .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));
        stdPrimitives.push_back(
            DecorativeFrame(1.0).setBodyId(2).setColor(SimTK::White)
            .setIndexOnBody(0).setScale(0.2).setOpacity(1)
            .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));
        stdPrimitives.push_back(
            DecorativeFrame(1.0).setBodyId(4).setColor(SimTK::White)
            .setIndexOnBody(0).setScale(0.2).setOpacity(1)
            .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));
        stdPrimitives.push_back(
            DecorativeFrame(1.0).setBodyId(3).setColor(SimTK::White)
            .setIndexOnBody(0).setScale(0.2).setOpacity(1)
            .setRepresentation(SimTK::DecorativeGeometry::DrawSurface));
        // Frames for the Joint
        stdPrimitives.push_back(
            DecorativeFrame(1.0).setBodyId(1).setColor(SimTK::White)
            .setIndexOnBody(0).setScale(0.2).setOpacity(1)
            .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
            .setTransform(Vec3({ -.01, -.03, .03 })));
    }
    Transform cylTransform;
    // This transform parallels the code in generateDecorations to reflect
    // that DecorativeCylinder is Y aligned while WrapCylinder is Z aligned
    cylTransform.updR().setRotationFromAngleAboutX(SimTK_PI / 2);
    stdPrimitives.push_back(
        DecorativeCylinder(.025, .05).setBodyId(0).setColor(Vec3(0, 0.2, 0.8))
        .setIndexOnBody(-1).setScale(1).setOpacity(0.5)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(Transform(cylTransform.R(), Vec3({ .01, -.4, .01 }))));
    stdPrimitives.push_back(
        DecorativeCylinder(.03, .05).setBodyId(0).setColor(SimTK::Cyan)
        .setIndexOnBody(-1).setScale(1).setOpacity(0.5)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(Transform(cylTransform.R(), Vec3({ .025, -.25, 0 }))));
    stdPrimitives.push_back(
        DecorativeCylinder(.03, .05).setBodyId(0).setColor(SimTK::Cyan)
        .setIndexOnBody(-1).setScale(1).setOpacity(0.5)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(Transform(cylTransform.R(), Vec3({ .05, -.25, 0 }))));
    stdPrimitives.push_back(
        DecorativeSphere(.055).setBodyId(0).setColor(Vec3(0, 0.2, 0.8))
        .setIndexOnBody(-1).setScale(1).setOpacity(0.5)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(Vec3({ .01, -.3, .01 })));
    stdPrimitives.push_back(
        DecorativeEllipsoid(Vec3(.1, .05, .15)).setBodyId(0).setColor(Vec3(0, 0.2, 0.8))
        .setIndexOnBody(-1).setScale(1).setOpacity(0.5)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(Vec3({ -.01, -.5, .0 })));
    stdPrimitives.push_back(
        DecorativeTorus(.08, .035).setBodyId(1).setColor(Vec3(0, 0.2, 0.8))
        .setIndexOnBody(-1).setScale(1).setOpacity(0.5)
        .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
        .setTransform(Vec3({ -.02, -.6, .01 })));


}
