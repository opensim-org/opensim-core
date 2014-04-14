/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testVisualization.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ModelVisualizer.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include "SimTKsimbody.h"
#include "SimTKmath.h"

using namespace OpenSim;
using namespace std;
using namespace SimTK;


class DecorativeGeometryImplementationToStream : public SimTK::DecorativeGeometryImplementation {
public:
   DecorativeGeometryImplementationToStream(std::stringstream& oStream, Model& modelToUse):
        outStream(oStream), model(modelToUse){ };
    virtual ~DecorativeGeometryImplementationToStream() {}
private:
    virtual void implementPointGeometry(    const DecorativePoint& dg){
    outStream << "DecorativePoint" << dg.getPoint() << getTranformInGround(dg) << dg.getIndexOnBody() << dg.getBodyId() << std::endl; };
    virtual void implementLineGeometry(     const DecorativeLine& dg){
    outStream << "DecorativeLine" << dg.getPoint1() << dg.getPoint2() << getTranformInGround(dg) << dg.getIndexOnBody() << dg.getBodyId() << std::endl; };
    virtual void implementBrickGeometry(    const DecorativeBrick& dg){
    outStream << "DecorativeBrick" << getTranformInGround(dg) << dg.getIndexOnBody() << dg.getBodyId() << std::endl; };
    virtual void implementCylinderGeometry( const DecorativeCylinder& dg){
    outStream << "DecorativeCylinder" << dg.getRadius() << dg.getHalfHeight() << getTranformInGround(dg) << dg.getIndexOnBody() << dg.getBodyId() << std::endl; };
    virtual void implementCircleGeometry(   const DecorativeCircle& dg){
    outStream << "DecorativeCircle" << dg.getRadius() << getTranformInGround(dg) << dg.getIndexOnBody() << dg.getBodyId() << std::endl; };
    virtual void implementSphereGeometry(   const DecorativeSphere& dg){
    outStream << "DecorativeSphere" << getTranformInGround(dg) << dg.getIndexOnBody() << dg.getBodyId() << std::endl; };
    virtual void implementEllipsoidGeometry(const DecorativeEllipsoid& dg){
    outStream << "DecorativeEllipsoid" << dg.getRadii() << getTranformInGround(dg) << dg.getIndexOnBody() << dg.getBodyId() << std::endl; };
    virtual void implementFrameGeometry(    const DecorativeFrame& dg){
    outStream << "DecorativeFrame" << getTranformInGround(dg) << dg.getIndexOnBody() << dg.getBodyId() << std::endl; };
    virtual void implementTextGeometry(     const DecorativeText& dg){
    outStream << "DecorativeText" << dg.getText() << getTranformInGround(dg) << dg.getIndexOnBody() << dg.getBodyId() << std::endl; };
    virtual void implementMeshGeometry(     const DecorativeMesh& dg){
    outStream << "DecorativeMesh" << getTranformInGround(dg) << dg.getIndexOnBody() << dg.getBodyId() << std::endl; };
    virtual void implementMeshFileGeometry(     const DecorativeMeshFile& dg){
    outStream << "DecorativeMeshFile" << getUserRefObj(dg) << dg.getMeshFile() << getTranformInGround(dg) << dg.getIndexOnBody() << dg.getBodyId() << std::endl; };
private:
    SimTK::Transform getTranformInGround(const DecorativeGeometry& dg) {
        if (dg.getBodyId()!=0){
            const SimTK::Transform& inBodyXform = dg.getTransform();
            const SimTK::SimbodyMatterSubsystem& matter = model.getMatterSubsystem();
            const Transform& X_GB = 
            matter.getMobilizedBody(MobilizedBodyIndex(dg.getBodyId())).getBodyTransform(model.getWorkingState());
            Transform X_GW = X_GB*inBodyXform;
            return X_GW;
        }
        else
            return dg.getTransform();
    }
    String getUserRefObj(const DecorativeGeometry& dg) {
        if (dg.getUserRef()==0)
            return "NoUserRef";
        else {
            OpenSim::Object* obj = (Object*)dg.getUserRef();
            String objDescription = obj->getConcreteClassName()+":"+obj->getName();
            return objDescription;
        }
    }
    std::stringstream& outStream;
    Model& model;
};

void testVisModel(string fileName)
{
	Model *model = new Model(fileName);
    model->setUseVisualizer(true);
	SimTK::State& defaultState = model->initSystem();

    model->getVisualizer().show(defaultState);
    std::stringstream ss;
    DecorativeGeometryImplementationToStream dgiStream(ss, *model);
    ModelDisplayHints mdh;
    SimTK::Array_<SimTK::DecorativeGeometry> dgArray;
    model->generateDecorations(true, mdh, defaultState, dgArray);
    std::cout << "Fixed Geometry" << std::endl;
    for (unsigned i=0; i < dgArray.size(); i++)
        dgArray.getElt(i).implementGeometry(dgiStream);

	std::cout << ss.str();
    
    std::cout << "Variable Geometry" << std::endl;
    dgArray.clear();
    model->generateDecorations(false, mdh, defaultState, dgArray);
    for (unsigned i=0; i < dgArray.size(); i++)
        dgArray.getElt(i).implementGeometry(dgiStream);

	std::cout << ss.str();
    std::cout.flush();
}

int main()
{
	try {
		LoadOpenSimLibrary("osimActuators");
		testVisModel("C:/OpenSim3.2x86/Models/Arm26/arm26.osim");
	}
	catch (const OpenSim::Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
	return 0;
}