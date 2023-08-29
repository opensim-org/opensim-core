/* -------------------------------------------------------------------------- *
 *                          OpenSim:  testScale.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Peter Loan, Tom Uchida                             *
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


// INCLUDES
#include <string>
#include <OpenSim/version.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/Ligament.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/EllipsoidJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Tools/GenericModelMaker.h>

using namespace OpenSim;
using std::cout; using std::endl;

void scaleGait2354();
void scaleGait2354_GUI(bool useMarkerPlacement);
void scaleModelWithLigament();
bool compareStdScaleToComputed(const ScaleSet& std, const ScaleSet& comp);

// Test scaling PhysicalOffsetFrames and models with atypical ownership trees.
void scalePhysicalOffsetFrames();

// Test scaling EllipsoidJoint, CustomJoint, and CoordinateCouplerConstraint.
void scaleJointsAndConstraints();

int main()
{
    try {
        scaleGait2354();
        scaleGait2354_GUI(false);
        scaleModelWithLigament();
        scalePhysicalOffsetFrames();
        scaleJointsAndConstraints();
    }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void compareModelToStandard(const std::string&  resultFilename,
                            const std::string&  targetFilename,
                            const double        tol)
{
    using SimTK::Vec3;

    // Load the result and target models.
    std::unique_ptr<Model> result{ new Model(resultFilename) };
    std::unique_ptr<Model> target{ new Model(targetFilename) };

    // Component paths can be guaranteed to be equivalent only once connections
    // have been made.
    result->setup();
    target->setup();

    // Check number of Marker Components (otherwise, test will pass even if
    // markers are missing from result model).
    {
        const unsigned nmResult = result->countNumComponents<Marker>();
        const unsigned nmTarget = target->countNumComponents<Marker>();
        OPENSIM_THROW_IF(nmResult != nmTarget, Exception,
            "Incorrect number of Marker Components in result Model.");
    }

    // Check Marker locations.
    cout << "Checking marker locations..." << endl;
    for (const Marker& mResult : result->getComponentList<Marker>())
    {
        const std::string& absPathStr = mResult.getAbsolutePathString();

        // Ensure marker exists in target model.
        OPENSIM_THROW_IF(!target->hasComponent(absPathStr), Exception,
            "Marker '" + absPathStr + "' not found in standard model.");

        const Vec3& result_loc = mResult.get_location();
        const Vec3& target_loc =
            target->getComponent<Marker>(absPathStr).get_location();

        cout << "  '" << absPathStr << "' - location: " << result_loc << endl;
        ASSERT_EQUAL(result_loc, target_loc, tol, __FILE__, __LINE__,
            "Marker '" + absPathStr + "' location in scaled model does not "
            + "match standard of " + target_loc.toString());
    }

    // Check number of GeometryPath Components (otherwise, test will pass even
    // if path actuators, ligaments, etc. are missing from result model).
    {
        const unsigned ngpResult = result->countNumComponents<GeometryPath>();
        const unsigned ngpTarget = target->countNumComponents<GeometryPath>();
        OPENSIM_THROW_IF(ngpResult != ngpTarget, Exception,
            "Incorrect number of GeometryPath Components in result Model.");
    }

    // Check GeometryPath path point locations.
    cout << "Checking path point locations..." << endl;
    SimTK::State& sResult = result->initSystem();
    SimTK::State& sTarget = target->initSystem();
    for (const GeometryPath& gpResult : result->getComponentList<GeometryPath>())
    {
        const std::string& absPathStr = gpResult.getAbsolutePathString();

        // Ensure GeometryPath exists in target model.
        OPENSIM_THROW_IF(!target->hasComponent(absPathStr), Exception,
            "GeometryPath '" + absPathStr + "' not found in standard model.");

        cout << "  '" << absPathStr << "'" << endl;
        for (int i = 0; i < gpResult.getPathPointSet().getSize(); ++i)
        {
            const Vec3& result_loc =
                gpResult.getPathPointSet()[i].getLocation(sResult);
            const Vec3& target_loc =
                target->getComponent<GeometryPath>(absPathStr)
                .getPathPointSet()[i].getLocation(sTarget);

            ASSERT_EQUAL(result_loc, target_loc, tol, __FILE__, __LINE__,
                "The location of point " + std::to_string(i)
                + " in GeometryPath '" + absPathStr + "' is "
                + result_loc.toString() + ", which does not match standard of "
                + target_loc.toString());
        }
    }
}

void scaleGait2354()
{
    // SET OUTPUT FORMATTING
    IO::SetDigitsPad(4);

    std::string setupFilePath;

    // Remove old results if any
    FILE* file2Remove = IO::OpenFile(setupFilePath+"subject01_scaleSet_applied.xml", "w");
    fclose(file2Remove);
    file2Remove = IO::OpenFile(setupFilePath+"subject01_simbody.osim", "w");
    fclose(file2Remove);

    // Construct model and read parameters file
    std::unique_ptr<ScaleTool> subject(new ScaleTool("subject01_Setup_Scale.xml"));

    // Keep track of the folder containing setup file, will be used to locate results to compare against
    setupFilePath=subject->getPathToSubject();

    subject->run();

    // Compare ScaleSet
    ScaleSet stdScaleSet = ScaleSet(
            setupFilePath+"std_subject01_scaleSet_applied.xml");
    {
        const ScaleSet& computedScaleSet = ScaleSet(
                setupFilePath+"subject01_scaleSet_applied.xml");
        ASSERT(compareStdScaleToComputed(stdScaleSet, computedScaleSet));
    }

    // See if we have any issues when calling run() twice.
    file2Remove = IO::OpenFile(setupFilePath+"subject01_scaleSet_applied.xml", "w");
    fclose(file2Remove);

    subject->run();
    {
        const ScaleSet& computedScaleSet = ScaleSet(
                setupFilePath+"subject01_scaleSet_applied.xml");
        ASSERT(compareStdScaleToComputed(stdScaleSet, computedScaleSet));
    }

    compareModelToStandard(setupFilePath + "subject01_simbody.osim",
                           "std_subject01_simbody.osim", 1.0e-6);
}

void scaleGait2354_GUI(bool useMarkerPlacement)
{
    // SET OUTPUT FORMATTING
    IO::SetDigitsPad(4);

    // Construct model and read parameters file
    std::unique_ptr<ScaleTool> subject{ new ScaleTool("subject01_Setup_Scale_GUI.xml") };
    std::string setupFilePath=subject->getPathToSubject();

    // Remove old results if any
    FILE* file2Remove = IO::OpenFile(setupFilePath+"subject01_scaleSet_applied_GUI.xml", "w");
    fclose(file2Remove);
    file2Remove = IO::OpenFile(setupFilePath+"subject01_scaledOnly_GUI.osim", "w");
    fclose(file2Remove);

    Model guiModel("gait2354_simbody.osim");
    
    // Keep track of the folder containing setup file, will be used to locate results to compare against
    std::unique_ptr<MarkerSet> markerSet{ 
        new MarkerSet(setupFilePath + subject->getGenericModelMaker().getMarkerSetFileName()) };
    guiModel.updateMarkerSet(*markerSet);

    guiModel.initSystem();

    // processedModelContext.processModelScale(scaleTool.getModelScaler(), processedModel, "", scaleTool.getSubjectMass())
    guiModel.getMultibodySystem().realizeTopology();
    SimTK::State* configState=&guiModel.updWorkingState();
    subject->getModelScaler().processModel(&guiModel, setupFilePath, subject->getSubjectMass());
    // Model has changed need to recreate a valid state 
    guiModel.getMultibodySystem().realizeTopology();
    configState=&guiModel.updWorkingState();
    guiModel.getMultibodySystem().realize(*configState, SimTK::Stage::Position);


    if (!subject->isDefaultMarkerPlacer() && subject->getMarkerPlacer().getApply()) {
        const MarkerPlacer& placer = subject->getMarkerPlacer();
        if( false == placer.processModel(&guiModel, subject->getPathToSubject())) 
            throw Exception("testScale failed to place markers");
    }

    // Compare ScaleSet
    ScaleSet stdScaleSet = ScaleSet(setupFilePath+"std_subject01_scaleSet_applied.xml");

    const ScaleSet& computedScaleSet = ScaleSet(setupFilePath+"subject01_scaleSet_applied_GUI.xml");

    ASSERT(compareStdScaleToComputed(stdScaleSet, computedScaleSet));

    compareModelToStandard(setupFilePath + "subject01_simbody.osim",
                           "std_subject01_simbody.osim", 1.0e-6);
}

void scaleModelWithLigament()
{
    // SET OUTPUT FORMATTING
    IO::SetDigitsPad(4);

    std::string setupFilePath("");

    // Remove old model if any
    FILE* file2Remove = IO::OpenFile(setupFilePath + "toyLigamentModelScaled.osim", "w");
    fclose(file2Remove);

    // Construct model and read parameters file
    std::unique_ptr<ScaleTool> scaleTool(
            new ScaleTool("toyLigamentModel_Setup_Scale.xml"));

    // Keep track of the folder containing setup file, will be used to locate results to compare against
    setupFilePath = scaleTool->getPathToSubject();
    const ModelScaler& scaler = scaleTool->getModelScaler();
    const std::string& scaledModelFile = scaler.getOutputModelFileName();
    const std::string& std_scaledModelFile = "std_toyLigamentModelScaled.osim";

    // Run the scale tool.
    scaleTool->run();

    Model comp(scaledModelFile);
    Model std(std_scaledModelFile);

    // the latest model will not match the standard because the naming convention has
    // been updated to store path names and connecting a model results in connectors
    // storing relative paths so that collections of components are more portable.
    // The models must be equivalent after being connected.
    comp.setup();
    std.setup();

    std.print("std_toyLigamentModelScaled_latest.osim");
    comp.print("comp_toyLigamentModelScaled_latest.osim");

    auto compLigs = comp.getComponentList<Ligament>();
    auto stdLigs = std.getComponentList<Ligament>();

    ComponentList<Ligament>::const_iterator itc = compLigs.begin();
    ComponentList<Ligament>::const_iterator its = stdLigs.begin();

    for (; its != stdLigs.end() && itc != compLigs.end(); ++its, ++itc){
        cout << "std:" << its->getName() << "==";
        cout << "comp:" << itc->getName() << " : ";
        cout << (*its == *itc) << endl;
        ASSERT(*its == *itc, __FILE__, __LINE__,
            "Scaled ligament " + its->getName() + " did not match standard.");
    }

    //Finally make sure we didn't incorrectly scale anything else in the model
    ASSERT(std == comp, __FILE__, __LINE__, 
            "Standard model failed to match scaled.");

    compareModelToStandard("comp_toyLigamentModelScaled_latest.osim",
                           std_scaledModelFile, 1.0e-6);
}

bool compareStdScaleToComputed(const ScaleSet& std, const ScaleSet& comp) {
    for (int i = 0; i < std.getSize(); ++i) {
        const Scale& scaleStd = std[i];
        int fix = -1;
        //find corresponding scale factor by segment name
        for (int j = 0 ; j < comp.getSize(); ++j) {
            if (comp[j].getSegmentName() == scaleStd.getSegmentName()) {
                fix = j;
                break;
            }
        }
        if (fix < 0) {
            cout << "Computed ScaleSet does not contain factors for ";
            cout << std[i].getSegmentName() << "." << endl;
            return false;
        }
        if (!(scaleStd == comp[fix])) {
            return false;
        }
    }
    return true;
}

void scalePhysicalOffsetFrames()
{
    cout << "Scaling PhysicalOffsetFrames and models with atypical ownership "
         << "trees..." << endl;

    using namespace SimTK;
    const Transform tfY = Transform(Vec3(0,1,0));

    // Create ScaleSet to scale the OpenSim::Body named "body".
    const double scaleFactor = 1.234;
    ScaleSet scaleSet;
    Scale* scale = new Scale();
    scale->setSegmentName("body");
    scale->setScaleFactors(Vec3(scaleFactor));
    scale->setApply(true);
    scaleSet.adoptAndAppend(scale);

    // Expected location of COM in Ground after scaling.
    Vec3 expectedLoc = Vec3(0, -scaleFactor, 0);

    // Helper function to scale the model, report the COM location, and compare
    // to the expected location.
    auto testScaling = [&](Model* model, State& s) -> void
    {
        const OpenSim::Body& body = model->getBodySet().get("body");
        const Vec3 initialLoc = body.findStationLocationInGround(s, Vec3(0));
        model->scale(s, scaleSet, false);
        const Vec3 finalLoc = body.findStationLocationInGround(s, Vec3(0));

        ASSERT_EQUAL(finalLoc, expectedLoc, SimTK::SignificantReal,
            __FILE__, __LINE__,
            "Incorrect final COM location:\n  initial: " + initialLoc.toString()
            + "\n    final: " + finalLoc.toString() + " (expected: "
            + expectedLoc.toString() + ")");
    };

    // Case 1: Use PinJoint's convenience constructor to create the child POF
    //         automatically. The POF will be stored in PinJoint's "frames" list
    //         property. This is the most typical case.
    {
        cout << "- case 1" << endl;

        Model* model = new Model();
        OpenSim::Body* body = new OpenSim::Body("body", 1, Vec3(0), Inertia(0));
        model->addBody(body);

        PinJoint* pin = new PinJoint("pin", model->getGround(), Vec3(0), Vec3(0),
                                            *body, Vec3(0,1,0), Vec3(0));
        model->addJoint(pin);

        State& s = model->initSystem();
        testScaling(model, s);
    }

    // Case 2: Create the child POF manually and store it in
    //         (a) PinJoint's "frames" list property (i==0)
    //         (b) PinJoint's "components" list property (i==1)
    //         (c) Model's "components" list property (i==2)
    for (int i=0; i<3; ++i)
    {
        cout << "- case 2(" << std::string("abc").substr(i,1) << ")" << endl;

        Model* model = new Model();
        OpenSim::Body* body = new OpenSim::Body("body", 1, Vec3(0), Inertia(0));
        model->addBody(body);

        PinJoint* pin = new PinJoint();
        pin->setName("pin");
        model->addJoint(pin);

        PhysicalOffsetFrame* pof = new PhysicalOffsetFrame("pof", *body, tfY);
        if (i==0)
            pin->updProperty_frames().adoptAndAppendValue(pof);
        else if (i==1)
            pin->addComponent(pof);
        else
            model->addComponent(pof);

        pin->connectSocket_parent_frame(model->getGround());
        pin->connectSocket_child_frame(*pof);

        State& s = model->initSystem();
        testScaling(model, s);
    }

    // Case 3: Create the child POF manually and store it in
    //         (a) a different PinJoint's "frames" list property (i==0)
    //         (b) a different PinJoint's "components" list property (i==1)

    // TODO: These cases currently throw "Assigned an invalid
    //       SimTK::MobilizedBodyIndex" Exception on initSystem(). See GitHub
    //       Issue #1970.

    /*
    for (int i=0; i<2; ++i)
    {
        cout << "- case 3(" << std::string("ab").substr(i,1) << ")" << endl;

        Model* model = new Model();

        // First add a body and joint as in Case 1.
        OpenSim::Body* otherBody = new OpenSim::Body("otherBody", 1, Vec3(0),
                                                     Inertia(0));
        model->addBody(otherBody);

        PinJoint* otherPin = new PinJoint("otherPin",
                                          model->getGround(), Vec3(0), Vec3(0),
                                          *otherBody, Vec3(0,1,0), Vec3(0));
        model->addJoint(otherPin);

        // Now add the components for the test.
        OpenSim::Body* body = new OpenSim::Body("body", 1, Vec3(0), Inertia(0));
        model->addBody(body);

        PinJoint* pin = new PinJoint();
        pin->setName("pin");
        model->addJoint(pin);

        PhysicalOffsetFrame* pof = new PhysicalOffsetFrame("pof", *body, tfY);
        if (i==0)
            otherPin->updProperty_frames().adoptAndAppendValue(pof);
        else
            otherPin->addComponent(pof);

        pin->connectSocket_parent_frame(model->getGround());
        pin->connectSocket_child_frame(*pof);

        State& s = model->initSystem();
        testScaling(model, s);
    }
    */

    // Case 4: Attach Markers to PhysicalOffsetFrames and assign arbitrary
    //         ownership. All Markers in this example model should be coincident
    //         before and after scaling.
    {
        cout << "- case 4" << endl;

        Model* model = new Model();
        OpenSim::Body* body = new OpenSim::Body("body", 1, Vec3(0), Inertia(0));
        model->addBody(body);

        FreeJoint* free = new FreeJoint("free", model->getGround(), *body);
        free->setName("free");
        model->addJoint(free);

        // First Marker is attached to and owned by the Body.
        const Vec3 offset1 = Vec3(0.3, 0.6, 0.9);
        Marker* marker1 = new Marker("marker1", *body, offset1);
        body->addComponent(marker1);

        // Second Marker is attached to the Body via one PhysicalOffsetFrame.
        // The POF is owned by the Model; the Marker is owned by the Joint.
        const Vec3 offset2  = offset1 / 2.;
        const Transform tf2 = Transform(offset2);
        PhysicalOffsetFrame* pof2 = new PhysicalOffsetFrame("pof2", *body, tf2);
        model->addComponent(pof2);
        Marker* marker2 = new Marker("marker2", *pof2, offset2);
        free->addComponent(marker2);

        // Third Marker is attached to the Body via two PhysicalOffsetFrames.
        // All new ModelComponents are owned by the first Marker.
        const Vec3 offset3  = offset1 / 3.;
        const Transform tf3 = Transform(offset3);
        PhysicalOffsetFrame* pof3a = new PhysicalOffsetFrame("pof3a", *body, tf3);
        marker1->addComponent(pof3a);
        PhysicalOffsetFrame* pof3b = new PhysicalOffsetFrame("pof3b", *pof3a, tf3);
        marker1->addComponent(pof3b);
        Marker* marker3 = new Marker("marker3", *pof3b, offset3);
        marker1->addComponent(marker3);

        State& s = model->initSystem();

        // Ensure all Markers are coincident before scaling.
        Vec3 expectedMarkerLoc = offset1;

        auto testMarkerLoc =
            [&](Marker* marker, const State& s, const std::string& msg) -> void
        {
            const Vec3 loc = marker->getLocationInGround(s);

            ASSERT_EQUAL(loc, expectedMarkerLoc, SimTK::SignificantReal,
                __FILE__, __LINE__,
                "Incorrect location of Marker '" + marker->getName()
                + "' (" + msg + ")\n  location: " + loc.toString()
                + "\n  expected: " + expectedMarkerLoc.toString());
        };

        testMarkerLoc(marker1, s, "before scaling");
        testMarkerLoc(marker2, s, "before scaling");
        testMarkerLoc(marker3, s, "before scaling");

        // Scale the model (updates the model's properties, reinitializes the
        // computational system, and updates the State that is passed in).
        model->scale(s, scaleSet, false);
        expectedMarkerLoc =
            expectedMarkerLoc.elementwiseMultiply(Vec3(scaleFactor));

        // Ensure all Markers are coincident after scaling.
        testMarkerLoc(marker1, s, "after scaling");
        testMarkerLoc(marker2, s, "after scaling");
        testMarkerLoc(marker3, s, "after scaling");
    }

    // Case 5: Attach a PathActuator to a PhysicalOffsetFrame. The two
    //         PathActuators in this example model should be coincident before
    //         and after scaling.
    {
        cout << "- case 5" << endl;

        Model* model = new Model();
        OpenSim::Body* body = new OpenSim::Body("body", 1, Vec3(0), Inertia(0));
        model->addBody(body);

        FreeJoint* free = new FreeJoint("free", model->getGround(), *body);
        free->setName("free");
        model->addJoint(free);

        // First PathActuator is attached to and owned by the Body.
        const Vec3 offset1 = Vec3(0.2, 0.4, 0.6);
        PathActuator* act1 = new PathActuator();
        act1->setName("pathActuator1");
        act1->addNewPathPoint("point1a", model->updGround(), Vec3(0));
        act1->addNewPathPoint("point1b", *body, offset1);
        body->addComponent(act1);

        // Second PathActuator is attached to the Body via a POF. Both new
        // ModelComponents are owned by the first PathActuator.
        const Vec3 offset2 = offset1 / 2.;
        const Transform tf2 = Transform(offset2);

        PhysicalOffsetFrame* pof2 = new PhysicalOffsetFrame("pof2", *body, tf2);
        act1->addComponent(pof2);

        PathActuator* act2 = new PathActuator();
        act2->setName("pathActuator2");
        act2->addNewPathPoint("point2a", model->updGround(), Vec3(0));
        act2->addNewPathPoint("point2b", *pof2, offset2);
        act1->addComponent(act2);

        State& s = model->initSystem();

        const std::string pathToAct1 = act1->getAbsolutePathString();
        const std::string pathToAct2 = act2->getAbsolutePathString();

        // Ensure PathPoints are coincident before scaling.
        auto testPathPointLoc =
            [&](const State& s, const std::string& msg) -> void
        {
            const PathActuator& pa1 =
                model->getComponent<PathActuator>(pathToAct1);
            const PathActuator& pa2 =
                model->getComponent<PathActuator>(pathToAct2);

            const PathPointSet& pps1 = pa1.getGeometryPath().getPathPointSet();
            const PathPointSet& pps2 = pa2.getGeometryPath().getPathPointSet();

            for (int i = 0; i < 2; ++i)
            {
                const Vec3& p1 = pps1[i].getLocationInGround(s);
                const Vec3& p2 = pps2[i].getLocationInGround(s);

                ASSERT_EQUAL(p1, p2, SimTK::SignificantReal, __FILE__, __LINE__,
                    "The location of point " + std::to_string(i)
                    + " does not match (" + msg + ")\n  PathActuator1: "
                    + p1.toString() + "\n  PathActuator2: " + p2.toString());
            }
        };

        testPathPointLoc(s, "before scaling");

        // Scale the model (updates the model's properties, reinitializes the
        // computational system, and updates the State that is passed in).
        model->scale(s, scaleSet, false);

        // Ensure PathPoints are coincident after scaling.
        testPathPointLoc(s, "after scaling");
    }
}

void scaleJointsAndConstraints()
{
    cout << "Scaling Joints and Constraints..." << endl;

    using namespace SimTK;

    // Create ScaleSet to scale "body1" and "body2".
    const Vec3 scaleFactors = Vec3(1.1, 2.22, 3.333);
    ScaleSet scaleSet;
    auto addBodyScale = [&](const std::string& bodyName) -> void {
        Scale* scale = new Scale();
        scale->setSegmentName(bodyName);
        scale->setScaleFactors(scaleFactors);
        scale->setApply(true);
        scaleSet.adoptAndAppend(scale);
    };
    addBodyScale("body1");
    addBodyScale("body2");

    // Test EllipsoidJoint scaling. Ensure radii are scaled if the parent is a
    // Body but does not change if the parent is Ground.
    {
        cout << "- EllipsoidJoint" << endl;

        Model* model = new Model();
        OpenSim::Body* body1 = new OpenSim::Body("body1", 1, Vec3(0), Inertia(0));
        model->addBody(body1);
        OpenSim::Body* body2 = new OpenSim::Body("body2", 1, Vec3(0), Inertia(0));
        model->addBody(body2);

        const Vec3 radii = Vec3(0.123, 0.456, 0.789);

        EllipsoidJoint* ellipsoid1 =
            new EllipsoidJoint("ellipsoid1", model->getGround(), *body1, radii);
        model->addJoint(ellipsoid1);
        EllipsoidJoint* ellipsoid2 =
            new EllipsoidJoint("ellipsoid2", *body2, model->getGround(), radii);
        model->addJoint(ellipsoid2);

        // Scale the model (updates the model's properties, reinitializes the
        // computational system, and updates the State that is passed in).
        State& s = model->initSystem();
        model->scale(s, scaleSet, false);

        // Radii of ellipsoid1 should not have changed.
        const Vec3& actual1   = ellipsoid1->get_radii_x_y_z();
        const Vec3& expected1 = radii;
        ASSERT_EQUAL(actual1, expected1, SimTK::SignificantReal,
            __FILE__, __LINE__,
            "EllipsoidJoint 'ellipsoid1' has incorrect radii.\n  actual: "
            + actual1.toString() + ", expected: " + expected1.toString());

        // Radii of ellipsoid2 should have been scaled using the scale factors
        // corresponding to body2.
        const Vec3& actual2   = ellipsoid2->get_radii_x_y_z();
        const Vec3& expected2 = radii.elementwiseMultiply(scaleFactors);
        ASSERT_EQUAL(actual2, expected2, SimTK::SignificantReal,
            __FILE__, __LINE__,
            "EllipsoidJoint 'ellipsoid2' has incorrect radii.\n  actual: "
            + actual2.toString() + ", expected: " + expected2.toString());
    }

    // Test CustomJoint scaling. Ensure SpatialTransform is scaled if the parent
    // is a Body but does not change if the parent is Ground.
    {
        cout << "- CustomJoint" << endl;

        Model* model = new Model();
        OpenSim::Body* body1 = new OpenSim::Body("body1", 1, Vec3(0), Inertia(0));
        model->addBody(body1);
        OpenSim::Body* body2 = new OpenSim::Body("body2", 1, Vec3(0), Inertia(0));
        model->addBody(body2);

        SpatialTransform transform;
        transform[3].setCoordinateNames(OpenSim::Array<std::string>("x", 1, 1));
        transform[3].setFunction(new LinearFunction(1.234, 5.678));

        // Evaluate LinearFunction before scaling and compare after scaling.
        const Vector xA = Vector(1, 0.);
        const double yA = transform[3].getFunction().calcValue(xA);
        const Vector xB = Vector(1, 123.456);
        const double yB = transform[3].getFunction().calcValue(xB);

        CustomJoint* custom1 =
            new CustomJoint("custom1", model->getGround(), *body1, transform);
        model->addJoint(custom1);
        CustomJoint* custom2 =
            new CustomJoint("custom2", *body2, model->getGround(), transform);
        model->addJoint(custom2);

        // Scale the model (updates the model's properties, reinitializes the
        // computational system, and updates the State that is passed in).
        State& s = model->initSystem();
        model->scale(s, scaleSet, false);

        // Transform of custom1 should not have changed.
        const OpenSim::Function& fn1 =
            custom1->getSpatialTransform()[3].getFunction();
        ASSERT_EQUAL(fn1.calcValue(xA), yA, SimTK::SignificantReal,
            __FILE__, __LINE__, "CustomJoint 'custom1' was incorrectly scaled.");
        ASSERT_EQUAL(fn1.calcValue(xB), yB, SimTK::SignificantReal,
            __FILE__, __LINE__, "CustomJoint 'custom1' was incorrectly scaled.");

        // Transform of custom2 should have been scaled using the scale factors
        // corresponding to body2.
        const OpenSim::Function& fn2 =
            custom2->getSpatialTransform()[3].getFunction();
        ASSERT_EQUAL(fn2.calcValue(xA), yA * scaleFactors[0],
            SimTK::SignificantReal, __FILE__, __LINE__,
            "CustomJoint 'custom2' was incorrectly scaled.");
        ASSERT_EQUAL(fn2.calcValue(xB), yB * scaleFactors[0],
            SimTK::SignificantReal, __FILE__, __LINE__,
            "CustomJoint 'custom2' was incorrectly scaled.");
    }

    // Test CoordinateCouplerConstraint scaling.
    {
        cout << "- CoordinateCouplerConstraint" << endl;

        Model* model = new Model();
        OpenSim::Body* body1 = new OpenSim::Body("body1", 1, Vec3(0), Inertia(0));
        model->addBody(body1);
        OpenSim::Body* body2 = new OpenSim::Body("body2", 1, Vec3(0), Inertia(0));
        model->addBody(body2);

        // Attach bodies to Ground with slider joints.
        SliderJoint* slider1 =
            new SliderJoint("slider1", model->getGround(), *body1);
        slider1->updCoordinate().setName("indepCoord");
        model->addJoint(slider1);

        SliderJoint* slider2 = new SliderJoint("slider2", *body1, *body2);
        slider2->updCoordinate().setName("depCoord");
        model->addJoint(slider2);

        // Constrain Coordinate of slider2.
        OpenSim::Array<std::string> indepCoordNameArray("indepCoord", 1, 1);
        const double slope     = 1.234;
        const double intercept = 5.678;

        CoordinateCouplerConstraint* ccc = new CoordinateCouplerConstraint();
        ccc->setIndependentCoordinateNames(indepCoordNameArray);
        ccc->setDependentCoordinateName("depCoord");
        ccc->setFunction(LinearFunction(slope, intercept));
        model->addConstraint(ccc);

        // Scale the model with non-uniform scale factors. Should fetch the
        // scale factors corresponding to body2 and then throw because only
        // uniform scaling is currently supported.
        State& s = model->initSystem();
        ASSERT_THROW(OpenSim::Exception, model->scale(s, scaleSet, false));

        // Test coupling function before scaling.
        slider1->getCoordinate().setValue(s, 0., true);
        ASSERT_EQUAL(slider2->getCoordinate().getValue(s), intercept,
            SimTK::SignificantReal, __FILE__, __LINE__,
            "SliderJoint has incorrect Coordinate value before scaling.");

        slider1->getCoordinate().setValue(s, 1., true);
        ASSERT_EQUAL(slider2->getCoordinate().getValue(s), intercept + slope,
            SimTK::SignificantReal, __FILE__, __LINE__,
            "SliderJoint has incorrect Coordinate value before scaling.");

        // Scale body2 uniformly.
        const double uniformFactor = 1.23456;
        ScaleSet scaleSetUniform;
        Scale* scale = new Scale();
        scale->setSegmentName("body1");
        scale->setScaleFactors(Vec3(uniformFactor));
        scale->setApply(true);
        scaleSetUniform.adoptAndAppend(scale);

        model->scale(s, scaleSetUniform, false);

        // Test coupling function after scaling.
        slider1->getCoordinate().setValue(s, 0., true);
        ASSERT_EQUAL(slider2->getCoordinate().getValue(s),
            intercept * uniformFactor,
            SimTK::SignificantReal, __FILE__, __LINE__,
            "SliderJoint has incorrect Coordinate value after scaling.");

        slider1->getCoordinate().setValue(s, 1., true);
        ASSERT_EQUAL(slider2->getCoordinate().getValue(s),
            (intercept + slope) * uniformFactor,
            SimTK::SignificantReal, __FILE__, __LINE__,
            "SliderJoint has incorrect Coordinate value after scaling.");
    }
}
