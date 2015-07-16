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
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void scaleGait2354();
void scaleGait2354_GUI(bool useMarkerPlacement);
void scaleModelWithLigament();

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

    // Now cycle thru ModelComponents recursively

    delete refModel;
}

void scaleGait2354()
{
    // SET OUTPUT FORMATTING
    IO::SetDigitsPad(4);

    std::string setupFilePath;
    ScaleTool* subject;
    Model* model;

    // Remove old results if any
    FILE* file2Remove = IO::OpenFile(setupFilePath+"subject01_scaleSet_applied.xml", "w");
    fclose(file2Remove);
    file2Remove = IO::OpenFile(setupFilePath+"subject01_simbody.osim", "w");
    fclose(file2Remove);

    // Construct model and read parameters file
    subject = new ScaleTool("subject01_Setup_Scale.xml");

    // Keep track of the folder containing setup file, wil be used to locate results to comapre against
    setupFilePath=subject->getPathToSubject();

    model = subject->createModel();

    SimTK::State& s = model->updWorkingState();
    model->getMultibodySystem().realize(s, SimTK::Stage::Position );

    if(!model) {
        //throw Exception("scale: ERROR- No model specified.",__FILE__,__LINE__);
        cout << "scale: ERROR- No model specified.";
    }

    ASSERT(!subject->isDefaultModelScaler() && subject->getModelScaler().getApply());
    ModelScaler& scaler = subject->getModelScaler();
    ASSERT(scaler.processModel(model, subject->getPathToSubject(), subject->getSubjectMass()));

    /*
    if (!subject->isDefaultMarkerPlacer() && subject->getMarkerPlacer().getApply()) {
        MarkerPlacer& placer = subject->getMarkerPlacer();
        if( false == placer.processModel(s, model, subject->getPathToSubject())) return(false);
    }
    else {
        return(1);
    }
    */
    // Compare ScaleSet
    ScaleSet stdScaleSet = ScaleSet(setupFilePath+"std_subject01_scaleSet_applied.xml");

    const ScaleSet& computedScaleSet = ScaleSet(setupFilePath+"subject01_scaleSet_applied.xml");

    ASSERT(computedScaleSet == stdScaleSet);
    
    delete model;
    delete subject;
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
    
    // Keep track of the folder containing setup file, wil be used to locate results to comapre against
    guiModel.initSystem();
    MarkerSet *markerSet = new MarkerSet(guiModel, setupFilePath + subject->getGenericModelMaker().getMarkerSetFileName());
    guiModel.updateMarkerSet(*markerSet);

    // processedModelContext.processModelScale(scaleTool.getModelScaler(), processedModel, "", scaleTool.getSubjectMass())
    guiModel.getMultibodySystem().realizeTopology();
    SimTK::State* configState=&guiModel.updWorkingState();
    bool retValue= subject->getModelScaler().processModel(&guiModel, setupFilePath, subject->getSubjectMass());
    // Model has changed need to recreate a valid state 
    guiModel.getMultibodySystem().realizeTopology();
    configState=&guiModel.updWorkingState();
    guiModel.getMultibodySystem().realize(*configState, SimTK::Stage::Position);


    if (!subject->isDefaultMarkerPlacer() && subject->getMarkerPlacer().getApply()) {
        MarkerPlacer& placer = subject->getMarkerPlacer();
        if( false == placer.processModel(&guiModel, subject->getPathToSubject())) 
            throw Exception("testScale failed to place markers");
    }

    // Compare ScaleSet
    ScaleSet stdScaleSet = ScaleSet(setupFilePath+"std_subject01_scaleSet_applied.xml");

    const ScaleSet& computedScaleSet = ScaleSet(setupFilePath+"subject01_scaleSet_applied_GUI.xml");

    ASSERT(computedScaleSet == stdScaleSet);

    delete subject;
}

void scaleModelWithLigament()
{
    // SET OUTPUT FORMATTING
    IO::SetDigitsPad(4);

    std::string setupFilePath("");
    ScaleTool* subject;
    Model* model;

    // Remove old model if any
    FILE* file2Remove = IO::OpenFile(setupFilePath + "toyLigamentModelScaled.osim", "w");
    fclose(file2Remove);

    // Construct model and read parameters file
    subject = new ScaleTool("toyLigamentModel_Setup_Scale.xml");

    // Keep track of the folder containing setup file, wil be used to locate results to comapre against
    setupFilePath = subject->getPathToSubject();

    model = subject->createModel();

    SimTK::State& s = model->updWorkingState();
    model->getMultibodySystem().realize(s, SimTK::Stage::Position);

    if (!model) {
        //throw Exception("scale: ERROR- No model specified.",__FILE__,__LINE__);
        cout << "scale: ERROR- No model specified.";
    }

    ASSERT(!subject->isDefaultModelScaler() && subject->getModelScaler().getApply());
    ModelScaler& scaler = subject->getModelScaler();
    ASSERT(scaler.processModel(model, setupFilePath, subject->getSubjectMass()));

    const std::string& scaledModelFile = scaler.getOutputModelFileName();
    const std::string& std_scaledModelFile = "std_toyLigamentModelScaled.osim";

    ASSERT(Model(scaledModelFile) == Model(std_scaledModelFile));

    delete model;
    delete subject;
}
