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

void testVisModel(string fileName);

// Implmenetation of DecorativeGeometryImplementation that prints the representation to
// a StringStream for comparison
class DecorativeGeometryImplementationText : public SimTK::DecorativeGeometryImplementation
{

    void implementPointGeometry(const DecorativePoint& dp){
        printout << "DecorativePoint:" << dp.getPoint() << printCommonProps(dp) << std::endl;
    };
    void implementLineGeometry(const DecorativeLine& dl){
        printout << "DecorativeLine:" << dl.getPoint1() << dl.getPoint2() << printCommonProps(dl) << std::endl;
    };
    void implementBrickGeometry(const DecorativeBrick& db){
        printout << "DecorativeBrick:" << db.getHalfLengths() << printCommonProps(db) << std::endl;
    };
    void implementCylinderGeometry(const DecorativeCylinder& dc){
        printout << "DecorativeCylinder:" << dc.getHalfHeight() << dc.getRadius() << printCommonProps(dc) << std::endl;
    };
    void implementCircleGeometry(const DecorativeCircle& dc){
        printout << "DecorativeCircle:" << dc.getRadius() << printCommonProps(dc) << std::endl;
    };
    void implementSphereGeometry(const DecorativeSphere& dp){
        printout << "DecorativeSphere:" << dp.getRadius() << printCommonProps(dp) << std::endl;
    };
    void implementEllipsoidGeometry(const DecorativeEllipsoid& dp){
        printout << "DecorativeEllipsoid:" << dp.getRadii() << printCommonProps(dp) << std::endl;
    };
    void implementFrameGeometry(const DecorativeFrame& dp){
        printout << "DecorativeFrame:" << dp.getAxisLength() << printCommonProps(dp) << std::endl;
    };
    void implementTextGeometry(const DecorativeText& dp){
        printout << "DecorativeText:" << dp.getText() << printCommonProps(dp) << std::endl;
    };
    void implementMeshGeometry(const DecorativeMesh& dp){
        printout << "DecorativeMesh:" << dp.getMesh().getNumFaces() << " " << dp.getMesh().getNumVertices() << printCommonProps(dp) << std::endl;
    };
    void implementMeshFileGeometry(const DecorativeMeshFile& dp){
        printout << "DecorativeMeshFile:" << dp.getMeshFile() << " " << printCommonProps(dp) << std::endl;
    };
    void implementArrowGeometry(const DecorativeArrow& dp){
        printout << "DecorativeArrow:" << dp.getStartPoint() << dp.getEndPoint() << dp.getTipLength() << printCommonProps(dp) << std::endl;
    };
    void implementTorusGeometry(const DecorativeTorus& dp){
        printout << "DecorativeTorus:" << dp.getTorusRadius() << dp.getTubeRadius() << printCommonProps(dp) << std::endl;
    };
    void implementConeGeometry(const DecorativeCone& dp){
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

int main()
{
    try {
        LoadOpenSimLibrary("osimActuators");
        testVisModel("BuiltinGeometry.osim");
    }
    catch (const OpenSim::Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

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
    ASSERT(same == 0);
}
