/* -------------------------------------------------------------------------- *
 *                          OpenSim:  testScale.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Peter Loan                                         *
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
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void scaleGait2354();
void scaleGait2354_GUI(bool useMarkerPlacement);
void scaleModelWithLigament();
bool compareStdScaleToComputed(const ScaleSet& std, const ScaleSet& comp);

int main()
{
    try {
        scaleGait2354();
        scaleGait2354_GUI(false);
        scaleModelWithLigament();
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void compareModel(const Model& resultModel, const std::string& stdFileName, double tol)
{
    // Load std model and realize it to Velocity just in case
    Model* refModel = new Model(stdFileName);
    SimTK::State& sStd = refModel->initSystem();

    const SimTK::State& s = resultModel.getWorkingState();
    resultModel.getMultibodySystem().realize(s, SimTK::Stage::Velocity);

    ASSERT(sStd.getNQ()==s.getNQ());    
    // put them in same configuration
    sStd.updQ() = s.getQ();
    refModel->getMultibodySystem().realize(sStd, SimTK::Stage::Velocity);

    ASSERT(sStd.getNU()==s.getNU());    
    ASSERT(sStd.getNZ()==s.getNZ());    

    delete refModel;
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
}

void scaleGait2354_GUI(bool useMarkerPlacement)
{
    // SET OUTPUT FORMATTING
    IO::SetDigitsPad(4);

    // Construct model and read parameters file
    ScaleTool* subject = new ScaleTool("subject01_Setup_Scale_GUI.xml");
    std::string setupFilePath=subject->getPathToSubject();

    // Remove old results if any
    FILE* file2Remove = IO::OpenFile(setupFilePath+"subject01_scaleSet_applied_GUI.xml", "w");
    fclose(file2Remove);
    file2Remove = IO::OpenFile(setupFilePath+"subject01_scaledOnly_GUI.osim", "w");
    fclose(file2Remove);

    Model guiModel("gait2354_simbody.osim");
    
    // Keep track of the folder containing setup file, will be used to locate results to compare against
    guiModel.initSystem();
    MarkerSet *markerSet = new MarkerSet(guiModel, setupFilePath + subject->getGenericModelMaker().getMarkerSetFileName());
    guiModel.updateMarkerSet(*markerSet);

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

    delete subject;
}

void scaleModelWithLigament()
{
    // SET OUTPUT FORMATTING
    IO::SetDigitsPad(4);

    std::string setupFilePath("");

    // Truncate old model if any
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

    ComponentList<Ligament> compLigs = comp.getComponentList<Ligament>();
    ComponentList<Ligament> stdLigs = std.getComponentList<Ligament>();

    ComponentList<Ligament>::const_iterator itc = compLigs.begin();
    ComponentList<Ligament>::const_iterator its = stdLigs.begin();

    for (; its != stdLigs.end() && itc != compLigs.end(); ++its, ++itc){
        cout << "std:" << its->getName() << "==";
        cout << "comp:" << itc->getName() << " : ";
        cout << (*its == *itc) << endl;
        ASSERT(*its == *itc, __FILE__, __LINE__,
            "Scaled ligament " + its->getName() + " did not match standard.");
    }

    std.print("std_toyLigamentModelScaled_latest.osim");
    comp.print("comp_toyLigamentModelScaled_latest.osim");

    // the latest model will not match the standard because the naming convention has
    // been updated to store path names and connecting a model results in connectors
    // storing relative paths so that collections of components are more portable.
    // The models must be equivalent after being connected.
    comp.setup();
    std.setup();

    //Finally make sure we didn't incorrectly scale anything else in the model
    ASSERT(std == comp);
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
