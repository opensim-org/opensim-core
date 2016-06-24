/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testVisualization.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
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

#include <stdint.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;
using namespace SimTK;

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
        printout << "DecorativeMeshFile:" << dp.getMeshFile() << " " << printCommonProps(dp) << std::endl;
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
private:
    std::stringstream printout;
    std::string printCommonProps(const DecorativeGeometry& dg){
        std::stringstream oneDGStream;
        oneDGStream << " bodyId:" << dg.getBodyId() << " color:" << dg.getColor() << " indexOnBody:"
            << dg.getIndexOnBody() << " Opacity:" << dg.getOpacity() << " Rep:" << dg.getRepresentation() << " Scale:"
            << dg.getScaleFactors() << " Transform:" << dg.getTransform();
        return oneDGStream.str();
    }
};

void testVisModel(string fileName)
{

    Model* model = new Model(fileName, true);
    SimTK::State& si = model->initSystem();
    ModelDisplayHints mdh; 
    SimTK::Array_<SimTK::DecorativeGeometry> geometryToDisplay;
    model->generateDecorations(true, mdh, si, geometryToDisplay);
    cout << geometryToDisplay.size() << endl;
    model->generateDecorations(false, mdh, si, geometryToDisplay);
    cout << geometryToDisplay.size() << endl;
    DecorativeGeometryImplementationText dgiText;
    for (unsigned i = 0; i < geometryToDisplay.size(); i++)
        geometryToDisplay[i].implementGeometry(dgiText);

    std::string baseName = fileName.substr(0, fileName.find_last_of('.'));
    std::ifstream t("vis_" + baseName + ".txt");
    if (!t.good()) throw OpenSim::Exception("Could not open file.");
    std::stringstream buffer;
    buffer << t.rdbuf();
    std::string fromFile = buffer.str();
    std::string fromModel = dgiText.getAsString();
    cout << "From Model " << endl << "=====" << endl;
    cout << fromModel << endl;
    cout << "From File " << endl << "=====" << endl;
    cout << fromFile << endl;
    int same = fromFile.compare(fromModel);
    delete model;
    ASSERT(same == 0, __FILE__, __LINE__, "Files do not match.");
}

namespace OpenSim {

//
class FloatingFrame : public Frame {
OpenSim_DECLARE_CONCRETE_OBJECT(FloatingFrame, Frame);
public:
    FloatingFrame(double speed) {
        constructProperty_speed(0);
        set_speed(speed);
    }
    OpenSim_DECLARE_PROPERTY(speed, double,
    "This frame moves forward in the x direction at `speed` m/s.");
protected:
    SimTK::Transform
        calcTransformInGround(const SimTK::State& s) const override {
        return Vec3(s.getTime(), 0, 0);
    }
    SimTK::SpatialVec
        calcVelocityInGround(const SimTK::State& s) const override {
        return SimTK::SpatialVec(0);
        // TODO OPENSIM_THROW(Exception, "Not implemented.");
    }
    SimTK::SpatialVec
        calcAccelerationInGround(const SimTK::State& s) const override {
        return SimTK::SpatialVec(0);
        // TODO OPENSIM_THROW(Exception, "Not implemented.");
    }
private:
    const Frame& extendFindBaseFrame() const override {
        // TODO is this well-defined for FloatingFrames?
        return *this;
    }
    SimTK::Transform extendFindTransformInBaseFrame() const override {
        return SimTK::Transform();
    }
};
} // end namespace OpenSim




void testNonPhysicalFrame() {
    Model model;
    auto* frame = new FloatingFrame(4.0);
    frame->setName("floater");
    frame->attachGeometry(new Sphere(2.0));
    model.addComponent(frame);
    SimTK::State& si = model.initSystem();
    
    // Set time to 1.0.
    si.setTime(1);
    
    ModelDisplayHints mdh;
    
    // Fixed.
    {
        // The FloatingFrame is not fixed, so we expect to get only
        // ground's FrameGeometry.
        // use raw string literal so we can use newlines nicely.
        std::string expectedText = R"(
DecorativeFrame:1 bodyId:0 color:~[1,1,1] indexOnBody:0 Opacity:1 Rep:3 Scale:~[0.2,0.2,0.2] Transform:
[1,0,0,0]
[0,1,0,0]
[0,0,1,0]
[0,0,0,1]
        )";
        SimTK::Array_<SimTK::DecorativeGeometry> geometryToDisplay;
        model.generateDecorations(true, mdh, si, geometryToDisplay);
        SimTK_TEST(geometryToDisplay.size() == 1);
        cout << geometryToDisplay.size() << endl;
        DecorativeGeometryImplementationText dgiText;
        for (unsigned i = 0; i < geometryToDisplay.size(); i++)
            geometryToDisplay[i].implementGeometry(dgiText);
        SimTK_TEST(dgiText.getAsString() == expectedText);
    }
    
    // Not fixed.
    {
        std::string expectedText = R"(
DecorativeSphere:1 bodyId:0 color:~[1,1,1] indexOnBody:0 Opacity:1 Rep:3 Scale:~[0.2,0.2,0.2] Transform:
[1,0,0,0]
[0,1,0,0]
[0,0,1,0]
[0,0,0,1]

DecorativeFrame:1 bodyId:0 color:~[1,1,1] indexOnBody:0 Opacity:1 Rep:3 Scale:~[0.2,0.2,0.2] Transform:
[1,0,0,4]
[0,1,0,0]
[0,0,1,0]
[0,0,0,1]
        )";
        // Now we expect to get only the Sphere
        // attached to floater (radius 2) and floater's FrameGeometry
        // (at x = 4, b/c speed is 4 m/s, and we're evaluating at 1 second).
        SimTK::Array_<SimTK::DecorativeGeometry> geometryToDisplay;
        model.generateDecorations(true, mdh, si, geometryToDisplay);
        SimTK_TEST(geometryToDisplay.size() == 3);
        cout << geometryToDisplay.size() << endl;
        DecorativeGeometryImplementationText dgiText;
        for (unsigned i = 0; i < geometryToDisplay.size(); i++)
            geometryToDisplay[i].implementGeometry(dgiText);
        SimTK_TEST(dgiText.getAsString() == expectedText);
    }
}

int main()
{
    LoadOpenSimLibrary("osimActuators");
    SimTK_START_TEST("testVisualization");
        SimTK_SUBTEST1(testVisModel, "BuiltinGeometry.osim");
        SimTK_SUBTEST(testNonPhysicalFrame);
    SimTK_END_TEST();
}



