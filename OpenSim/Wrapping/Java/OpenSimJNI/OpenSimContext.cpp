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

// States
void OpenSimContext::setStates( Array<double>&  states) {
    _model->setStateValues(*_configState, &states[0]);
}
void OpenSimContext::setStates( double* statesBuffer) {
  _model->setStateValues(*_configState, statesBuffer);
 }

void OpenSimContext::computeConstrainedCoordinates( double* statesBuffer) {
//        _model->getSimbodyEngine().computeConstrainedCoordinates(*_configState, statesBuffer );
}

void OpenSimContext::getStates( double* statesBuffer) {
        Array<double> rStateValues;
    _model->getStateValues(*_configState, rStateValues);
    //string statedump= _configState->toString();
        for(int i=0;i<_model->getNumStateVariables();i++ ) *(statesBuffer+i) = rStateValues[i];
}

void OpenSimContext::getStates( Array<double>&  rStates) {
    _model->getStateValues(*_configState, rStates);
}
// Transforms
void OpenSimContext::transformPosition(const Body& body, double* offset, double* gOffset) {
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
    _model->getSimbodyEngine().transformPosition(*_configState, body, offset, gOffset );
}

SimTK::Transform OpenSimContext::getTransform(const Body& body) { // Body Should be made const
   _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
     return _model->getSimbodyEngine().getTransform(*_configState, body );
}

void OpenSimContext::transform(const Body& ground, double* d, Body& body, double* dragVectorBody) {
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
    _model->getSimbodyEngine().transform(*_configState, ground, SimTK::Vec3(d), body, SimTK::Vec3::updAs(dragVectorBody) );
    return;
}


// Coordinates
double OpenSimContext::getValue(const Coordinate& coord) {
        return( coord.getValue( *_configState));
}

bool OpenSimContext::getLocked(const Coordinate& coord) {
    return coord.getLocked(*_configState);
}
void OpenSimContext::setValue(const Coordinate& coord, double d, bool enforceConstraints) {
  coord.setValue(*_configState, d, enforceConstraints);
    return;
}

void OpenSimContext::setClamped(Coordinate&  coord, bool newValue) {
   coord.setDefaultClamped(newValue);
   recreateSystemKeepStage();
   return;
}

bool OpenSimContext::getClamped(const Coordinate& coord) {
  return coord.getClamped(*_configState);
}

void OpenSimContext::setLocked(Coordinate& coord, bool newValue) {
   coord.setDefaultValue(getValue(coord));
   coord.setDefaultLocked(newValue);
   recreateSystemKeepStage();
   return;
}

bool OpenSimContext::isPrescribed(const Coordinate& coord) const {
  return (coord.isPrescribed(*_configState));
}

bool OpenSimContext::isConstrained(const Coordinate& coord) const {
  return (coord.isDependent(*_configState));
}

// Muscles
double OpenSimContext::getActivation(Muscle& m) {
  // realize to dynamics as required by new muscle models before asking for activation
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Dynamics);
  return m.getActivation(*_configState);
}

double OpenSimContext::getMuscleLength(Muscle& m) {
  return m.getLength(*_configState);
}

const Array<PathPoint*>& OpenSimContext::getCurrentPath(Muscle& m) {
  return m.getGeometryPath().getCurrentPath(*_configState);
}

const Array<PathPoint*>& OpenSimContext::getCurrentDisplayPath(GeometryPath& g) {
  g.updateGeometry(*_configState);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Velocity);
  return g.getCurrentDisplayPath(*_configState);
}

void OpenSimContext::updateDisplayer(Force& f) {
  // If muscle or force acting along a path then call respective updateDisplayer otherwise do nothing for now
  realizeVelocity();
  if (dynamic_cast<Muscle*>(&f)!= NULL)
    return dynamic_cast<Muscle*>(&f)->updateDisplayer(*_configState);
  if (f.hasGeometryPath()){
    AbstractProperty& pathProp = f.updPropertyByName("GeometryPath");
    Object& pathObj = pathProp.updValueAsObject();
    return dynamic_cast<GeometryPath*>(&pathObj)->updateDisplayer(*_configState);
  }
  if (f.getDisplayer()!= NULL)
    f.updateDisplayer(*_configState);
}

void OpenSimContext::copyMuscle(Muscle& from, Muscle& to) {
  to = from;
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Velocity);
  to.updGeometryPath().updateGeometry(*_configState);
}

// Muscle Points
void OpenSimContext::setXFunction(MovingPathPoint& mmp, Function& newFunction) {
    mmp.setXFunction(*_configState, newFunction );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  mmp.getPath()->updateGeometry(*_configState);
   return;
}

void OpenSimContext::setYFunction(MovingPathPoint& mmp, Function& newFunction) {
    mmp.setYFunction(*_configState, newFunction );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  mmp.getPath()->updateGeometry(*_configState);
    return;
}

void OpenSimContext::setZFunction(MovingPathPoint& mmp, Function& newFunction) {
    mmp.setZFunction(*_configState, newFunction );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  mmp.getPath()->updateGeometry(*_configState);
    return;
}

void OpenSimContext::setXCoordinate(MovingPathPoint& mmp, Coordinate&  newCoord) {
    mmp.setXCoordinate(*_configState, newCoord );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  mmp.getPath()->updateGeometry(*_configState);
  return;
}

void OpenSimContext::setYCoordinate(MovingPathPoint& mmp, Coordinate&  newCoord) {
    mmp.setYCoordinate(*_configState, newCoord );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  mmp.getPath()->updateGeometry(*_configState);
    return;
}

void OpenSimContext::setZCoordinate(MovingPathPoint& mmp, Coordinate&  newCoord) {
    mmp.setZCoordinate(*_configState, newCoord );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  mmp.getPath()->updateGeometry(*_configState);
    return;
}

void OpenSimContext::setBody(PathPoint& pathPoint, Body&  newBody) {
   pathPoint.changeBodyPreserveLocation(*_configState, newBody);
   this->recreateSystemAfterSystemExists();
   realizeVelocity();
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


void OpenSimContext::setCoordinate(ConditionalPathPoint&  via, Coordinate&  newCoord) {
    via.setCoordinate(*_configState, newCoord );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  via.getPath()->updateGeometry(*_configState);
    return;
}

void OpenSimContext::setRangeMin(ConditionalPathPoint&  via, double d) {
     via.setRangeMin(*_configState, d );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  via.getPath()->updateGeometry(*_configState);
     return;
}

void OpenSimContext::setRangeMax(ConditionalPathPoint&  via, double d) {
    via.setRangeMax(*_configState, d );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  via.getPath()->updateGeometry(*_configState);
    return;
}

bool OpenSimContext::replacePathPoint(GeometryPath& p, PathPoint& mp, PathPoint& newPoint) {
   bool ret= p.replacePathPoint(*_configState, &mp, &newPoint );
   recreateSystemAfterSystemExists();
   realizeVelocity();
   p.updateGeometry(*_configState);
   return ret;
}

void OpenSimContext::setLocation(PathPoint& mp, int i, double d) {
    mp.setLocation(*_configState, i, d);
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  mp.getPath()->updateGeometry(*_configState);
    return;
}

void OpenSimContext::setEndPoint(PathWrap& mw, int newEndPt) {
    mw.setEndPoint(*_configState, newEndPt );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  mw.getPath()->updateGeometry(*_configState);
    return;
}

void OpenSimContext::addPathPoint(GeometryPath& p, int menuChoice, Body&  body) {
    p.addPathPoint(*_configState, menuChoice, body );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  p.updateGeometry(*_configState);
    return;
}

bool OpenSimContext::deletePathPoint(GeometryPath& p, int menuChoice) {
    bool ret = p.deletePathPoint(*_configState, menuChoice );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  p.updateGeometry(*_configState);
  return ret;
}

bool OpenSimContext::isActivePathPoint(PathPoint& mp) {
  return mp.isActive(*_configState);
};

//_____________________________________________________________________________
/**
 * Replace one of the actuator's functions in the property array.
 *
 * @param aOldFunction the function being replaced.
 * @param aNewFunction the new function.
 */
void OpenSimContext::replacePropertyFunction(OpenSim::Object& obj, OpenSim::Function* aOldFunction, OpenSim::Function* aNewFunction)
{
  if (aOldFunction && aNewFunction) {
    PropertySet& propSet = obj.getPropertySet();

    for (int i=0; i <propSet.getSize(); i++) {
      Property_Deprecated* prop = propSet.get(i);
      if (prop->getType() == Property_Deprecated::ObjPtr) {
        if (prop->getValueObjPtr() == aOldFunction) {
          prop->setValue(aNewFunction);
        }
      }
    }
  }
}

// Muscle Wrapping
void OpenSimContext::setStartPoint(PathWrap& mw, int newStartPt) {
     mw.setStartPoint(*_configState, newStartPt );
     return;
}

void OpenSimContext::addPathWrap(GeometryPath& p, WrapObject& awo) {
    p.addPathWrap( awo );
    return;
}

void OpenSimContext::moveUpPathWrap(GeometryPath& p, int num) {
    p.moveUpPathWrap( *_configState, num );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  p.updateGeometry(*_configState);
    return;
}

void OpenSimContext::moveDownPathWrap(GeometryPath& p, int num) {
    p.moveDownPathWrap( *_configState, num );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  p.updateGeometry(*_configState);
    return;
}

void OpenSimContext::deletePathWrap(GeometryPath& p, int num) {
    p.deletePathWrap( *_configState, num );
  _configState->invalidateAll(SimTK::Stage::Position);
  _model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  p.updateGeometry(*_configState);
    return;
}

// Markers
void OpenSimContext::setBody(Marker& currentMarker, Body&  newBody, bool b) {
    if( b ) {
         currentMarker.changeBodyPreserveLocation( *_configState, newBody );
    } else {
         currentMarker.changeBody( newBody );
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
  bool retValue= modelScaler.processModel(*_configState, aModel, aPathToSubject, aFinalMass);
  // Model has changed need to recreate a valid state
  aModel->getMultibodySystem().realizeTopology();
    _configState=&aModel->updWorkingState();
  aModel->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
  return retValue;
}

bool OpenSimContext::processModelMarkerPlacer( MarkerPlacer& markerPlacer,
                        Model* aModel,
                        const std::string& aPathToSubject) {
  return markerPlacer.processModel(*_configState, aModel, aPathToSubject);
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

void OpenSimContext::cacheModelAndState() 
{
	clonedModel = _model->clone();
	clonedState = this->getCurrentStateCopy();
}

void OpenSimContext::restoreStateFromCachedModel() 
{
	_model->initSystem();
	clonedModel->initSystem();

	Array<std::string> modelVariableNames = _model->getStateVariableNames();
	Array<std::string> clonedModelVariableNames = clonedModel->getStateVariableNames();

	for(int i = 0; i < modelVariableNames.getSize(); i++)
	{
		std::string name = modelVariableNames.get(i);
		if(clonedModelVariableNames.findIndex(name) >= 0)
		{
			double value = clonedModel->getStateVariable(clonedState, name);
			_model->setStateVariable(_model->updWorkingState(), name, value);
		}
	}
	this->setState(&(_model->updWorkingState()));
	this->realizePosition();
	delete clonedModel;
}
} // namespace
