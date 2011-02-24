// testContext.cpp
// Author:  Ayman Habib
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "../OpenSimContext.h"
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/Model/Marker.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/MovingPathPoint.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/PathPoint.h>
#include <OpenSim/Simulation/Wrap/PathWrap.h>
#include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Wrap/WrapObject.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Tools/ModelScaler.h>
#include <OpenSim/Tools/Measurement.h>
#include <OpenSim/Tools/MarkerPlacer.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

#define ASSERT_EQUAL(expected, found, tolerance) { \
double tol = std::max((tolerance), std::abs((expected)*(tolerance))); \
if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) throw(exception());}

int main()
{
    int  status = 0;

	// To Retrace the steps taken by the GUI this test case follows the same call sequence:
	// new Model(file)
	// new OpenSimContext(model.initSystem(), model);
	// context.updateDisplayer(muscle)	// to display muscles
	// context.getCurrentPath(muscle)
	// context.getTransform(body)
	// context.transformPosition(body, loc, global)  // to display markers
	// context.getLocked(Coordinate)
	// context.getValue(cooridnate)
	LoadOpenSimLibrary("osimActuators");
	LoadOpenSimLibrary("osimSimulation");
	LoadOpenSimLibrary("osimJavaJNI");

	Model *model = new Model("wrist.osim");
	OpenSimContext* context = new OpenSimContext(&model->initSystem(), model);
	const ForceSet& fs = model->getForceSet();
	int n1 = fs.getNumGroups();
	const ObjectGroup* grp = fs.getGroup("wrist");
	assert(grp);
	const Array<Object*>& members = grp->getMembers();
	int sz = members.getSize();
	ASSERT_EQUAL(sz,5,0);
	assert(members.get(0)->getName()=="ECRB");
	delete model;
	delete context;
	model = new Model("arm26_20.osim");
	context = new OpenSimContext(&model->initSystem(), model);
	Array<std::string> stateNames;
	model->getStateNames(stateNames);
	OpenSim::Force* dForce=&(model->updForceSet().get("TRIlong"));
	Muscle* dTRIlong = dynamic_cast<Muscle*>(dForce);
	assert(dTRIlong);
	context->updateDisplayer(*dTRIlong);
	const OpenSim::Array<PathPoint*>& path = context->getCurrentPath(*dTRIlong);
	cout << "Muscle Path" << endl;
	cout << path.getSize() << endl;
	for(int i=0; i< path.getSize(); i++)
		cout << path.get(i)->getBodyName() << path.get(i)->getLocation() << endl;
	// Compare to known path 
	const OpenSim::Body& dBody = model->getBodySet().get("r_ulna_radius_hand");
	Transform xform = context->getTransform(dBody);
	cout << xform << endl;
	double flat[16];
	context->getTransformAsDouble16(xform, flat);
	// Compare to known xform
	double markerPosition[] = {.005000000000, -0.290400000000, 0.030000000000};
	double markerPositionInGround[3];
	context->transformPosition(dBody, markerPosition, markerPositionInGround);  // to display markers
	cout << "Global frame position = " << markerPositionInGround[0] <<  
		markerPositionInGround[1] << markerPositionInGround[2]<< endl;
	// Check xformed point against known position
	const Coordinate& dr_elbow_flex = model->getCoordinateSet().get("r_elbow_flex");
	bool isLocked = context->getLocked(dr_elbow_flex);
	assert(!isLocked);
	double startValue = context->getValue(dr_elbow_flex);
	cout << "Coordinate start value = " << startValue << endl;
	double length1 = context->getMuscleLength(*dTRIlong);
	cout << length1 << endl;
	ASSERT_EQUAL(.277609, length1, 1e-5);
	// Coordinate Slider
	context->setValue(dr_elbow_flex, 100*SimTK_PI/180.);
	// Get body transform, marker position and muscle path (tests wrapping as well)
	xform = context->getTransform(dBody);
	cout << "After setting coordinate to 100 deg." << endl;
	cout << xform << endl;
	// Compare to known xform
	context->updateDisplayer(*dTRIlong);
	const OpenSim::Array<PathPoint*>& newPath = context->getCurrentPath(*dTRIlong);
	// Compare to known path 
	cout << "New Muscle Path" << endl;
	cout << path.getSize() << endl;
	for(int i=0; i< path.getSize(); i++)
		cout << path.get(i)->getBodyName() << path.get(i)->getLocation() << endl;
	double length2 = context->getMuscleLength(*dTRIlong);
	cout << length2 << endl;
	ASSERT_EQUAL(.315748, length2, 1e-5);
	// Analyze Tool for plotting?
	return status;
}

