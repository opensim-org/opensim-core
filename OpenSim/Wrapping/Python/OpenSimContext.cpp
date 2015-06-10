/* -------------------------------------------------------------------------- *
 *                        OpenSim:  OpenSimContext.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/SimbodyEngine/TransformAxis.h>
#include <OpenSim/Simulation/Model/Marker.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MovingPathPoint.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/PathPoint.h>
#include <OpenSim/Simulation/Wrap/PathWrap.h>
#include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/TransformAxis.h>
#include <OpenSim/Simulation/Wrap/WrapObject.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Tools/ModelScaler.h>
#include <OpenSim/Tools/Measurement.h>
#include <OpenSim/Tools/MarkerPlacer.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include "OpenSimContext.h"

namespace OpenSim {

OpenSimContext::OpenSimContext( SimTK::State* s, Model* model ) :
    _configState(s),
  _model(model) {}


// Transforms
void OpenSimContext::transformPosition(const PhysicalFrame& body, double* offset, double* gOffset) {
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
    _model->getSimbodyEngine().transformPosition(*_configState, body, offset, gOffset );
}

SimTK::Transform OpenSimContext::getTransform(const PhysicalFrame& body) { // Body Should be made const
   _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
     return _model->getSimbodyEngine().getTransform(*_configState, body );
}

void OpenSimContext::transform(const PhysicalFrame& ground, double* d, PhysicalFrame& body, double* dragVectorBody) {
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
    _model->getSimbodyEngine().transform(*_configState, ground, SimTK::Vec3(d), body, SimTK::Vec3::updAs(dragVectorBody) );
    return;
}

//-------------------------------------------------------------------
void OpenSimContext::recreateSystemAfterSystemExistsKeepStage( )
{
   SimTK::Stage stageBeforeRecreatingSystem = _configState->getSystemStage();
   this->recreateSystemAfterSystemExists();
  _model->getMultibodySystem().realize( *_configState, stageBeforeRecreatingSystem );
}

//-------------------------------------------------------------------
void OpenSimContext::recreateSystemAfterSystemExists( )
{
  SimTK::Vector y1 = _configState->getY();
  SimTK::State* newState = &_model->initSystem();
  newState->updY() = y1;
  this->setState( newState );
}

// Markers
void OpenSimContext::setBody(Marker& currentMarker, PhysicalFrame&  newBody, bool b) {
    if( b ) {
         currentMarker.changeFramePreserveLocation( *_configState, newBody );
    } else {
         currentMarker.changeFrame( newBody );
    }
    return;
}

int OpenSimContext::replaceMarkerSet(Model& model, MarkerSet& aMarkerSet) {
  return model.replaceMarkerSet( *_configState, aMarkerSet);
}

// Analyses
int OpenSimContext::step(Analysis& analysis)
{
  return analysis.step( *_configState, 0);
}

// Tools
/*
bool OpenSimContext::initializeTrial(IKTool& ikTool, int i) {
  return ikTool.initializeTrial(*_configState, i);
}
*/
bool OpenSimContext::solveInverseKinematics( InverseKinematicsTool& ikTool) {
  return ikTool.run();
}

void OpenSimContext::setStatesFromMotion(AnalyzeTool& analyzeTool, const Storage &aMotion, bool aInDegrees) {
  analyzeTool.setStatesFromMotion(*_configState, aMotion, aInDegrees);
}

void OpenSimContext::loadStatesFromFile(AnalyzeTool& analyzeTool) {
  analyzeTool.loadStatesFromFile(*_configState);
}

bool OpenSimContext::processModelScale(ModelScaler& modelScaler,
                     Model* aModel,
                     const std::string& aPathToSubject,
                     double aFinalMass) {
  aModel->getMultibodySystem().realizeTopology();
    _configState=&aModel->updWorkingState();
  bool retValue= modelScaler.processModel(aModel, aPathToSubject, aFinalMass);
  // Model has changed need to recreate a valid state
  aModel->getMultibodySystem().realizeTopology();
    _configState=&aModel->updWorkingState();
  aModel->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  return retValue;
}

bool OpenSimContext::processModelMarkerPlacer( MarkerPlacer& markerPlacer,
                        Model* aModel,
                        const std::string& aPathToSubject) {
  return markerPlacer.processModel(aModel, aPathToSubject);
}

double OpenSimContext::computeMeasurementScaleFactor(ModelScaler& modelScaler,
                           const Model& aModel,
                           const MarkerData& aMarkerData,
                           const Measurement& aMeasurement) const {
  return modelScaler.computeMeasurementScaleFactor(*_configState, aModel, aMarkerData, aMeasurement);
}

void OpenSimContext::replaceTransformAxisFunction(TransformAxis& aDof, OpenSim::Function& aFunction) {
   aDof.setFunction(&aFunction);
   this->recreateSystemAfterSystemExists();
   realizeVelocity();
}

// Force re-realization
void OpenSimContext::realizePosition() {
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);

}
void OpenSimContext::realizeVelocity() {
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Velocity);

}
} // namespace
