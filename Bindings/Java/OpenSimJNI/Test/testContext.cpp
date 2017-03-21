/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testContext.cpp                          *
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
#include <OpenSim/Actuators/Thelen2003Muscle.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

#define ASSERT_EQUAL(expected, found, tolerance) { \
double tol = std::max((tolerance), std::abs((expected)*(tolerance))); \
if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) throw(exception());}

int main()
{
  try {
    int  status = 0;

    // To Retrace the steps taken by the GUI this test case follows the same call sequence:
    // new Model(file)
    // new OpenSimContext(model.initSystem(), model);
    // context.updateDisplayer(muscle)  // to display muscles
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
    const Array<const Object*>& members = grp->getMembers();
    int sz = members.getSize();
    ASSERT_EQUAL(sz,5,0);
    assert(members.get(0)->getName()=="ECRB");
    delete model;
    delete context;
    model = new Model("arm26_20.osim");
    context = new OpenSimContext(&model->initSystem(), model);
    // Make a copy of state contained in context ad make sure content match 
    SimTK::State stateCopy = context->getCurrentStateCopy();
    assert(context->getCurrentStateRef().toString()==stateCopy.toString());

    Array<std::string> stateNames = model->getStateVariableNames();
    OpenSim::Force* dForce=&(model->updForceSet().get("TRIlong"));
    Muscle* dTRIlong = dynamic_cast<Muscle*>(dForce);
    assert(dTRIlong);
    context->setPropertiesFromState();
    OpenSim::Thelen2003Muscle* thelenMsl = dynamic_cast<Thelen2003Muscle*>(dTRIlong);
    AbstractProperty& dProp = thelenMsl->updPropertyByName("ignore_tendon_compliance");
    
    PropertyHelper::setValueBool(true, dProp);
    cout << "Prop after is " << dProp.toString() << endl;

    bool exceptionThrown = false;
    try{// adding to the system should cause Muscle that do not handle
        // ignore_tendon_compliance to throw an exception
        context->recreateSystemKeepStage();
    }   
    catch (const std::exception& e) {
        cout << e.what() << endl;
        exceptionThrown = true;
        PropertyHelper::setValueBool(false, dProp);
        cout << "Prop reset to " << dProp.toString() << endl;
        // recreate the system so test can continue
        context->recreateSystemKeepStage();
    }

    SimTK_ASSERT_ALWAYS(exceptionThrown, "Setting ignore_tendon_compliance must throw an exception.");


    AbstractProperty& dProp2 = thelenMsl->updPropertyByName("ignore_tendon_compliance");
    cout << "Prop after create system is " << dProp2.toString() << endl;
    bool after = PropertyHelper::getValueBool(dProp);
    SimTK_ASSERT_ALWAYS(!after, "Property has wrong value!!");
    dTRIlong->updGeometryPath().updateGeometry(context->getCurrentStateRef());
    const OpenSim::Array<AbstractPathPoint*>& path = context->getCurrentPath(*dTRIlong);
    cout << "Muscle Path" << endl;
    cout << path.getSize() << endl;
    for(int i=0; i< path.getSize(); i++)
        cout << path[i]->getParentFrame().getName()
             << path[i]->getLocation(stateCopy) << endl;
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
    dTRIlong->updGeometryPath().updateGeometry(context->getCurrentStateRef());
    const OpenSim::Array<AbstractPathPoint*>& newPath = context->getCurrentPath(*dTRIlong);
    // Compare to known path 
    cout << "New Muscle Path" << endl;
    cout << path.getSize() << endl;
    for(int i=0; i< path.getSize(); i++)
        cout << path[i]->getParentFrame().getName() 
             << path[i]->getLocation(stateCopy) << endl;
    double length2 = context->getMuscleLength(*dTRIlong);
    cout << length2 << endl;
    ASSERT_EQUAL(.315748, length2, 1e-5);
    // Test that we can lock coordinates to specific value and make this persistant.
    Coordinate& dr_elbow_flex_mod = model->updCoordinateSet().get("r_elbow_flex");
    //dr_elbow_flex_mod.setDefaultValue(0.5);
    dr_elbow_flex_mod.setDefaultLocked(true);
    context->setValue(dr_elbow_flex_mod, 0.5);
    //model->print("wrist_locked_elbow.osim");
    context->recreateSystemKeepStage();
    const Coordinate& dr_elbow_flexNew = model->getCoordinateSet().get("r_elbow_flex");
    assert(context->getLocked(dr_elbow_flexNew));
    ASSERT_EQUAL(0.5, context->getValue(dr_elbow_flexNew), 0.000001);

    // Exercise Editing workflow
    // These are the same calls done from GUI code base through Property edits
    OpenSim::Body& bdy = model->updBodySet().get("r_humerus");
    AbstractProperty& massProp = bdy.updPropertyByName("mass");
    double oldValue = PropertyHelper::getValueDouble(massProp);
    double v = oldValue + 1.0;
    context->cacheModelAndState();
    PropertyHelper::setValueDouble(v, massProp);
    context->restoreStateFromCachedModel();
    return status;
  } catch (const std::exception& e) {
      cout << "Exception: " << e.what() << endl;
      return 1;
  }
}

