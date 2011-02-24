
#include "OpenSimContext.h"
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
        for(int i=0;i<_model->getNumStates();i++ ) *(statesBuffer+i) = rStateValues[i];
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

void OpenSimContext::setClamped(const Coordinate&  coord, bool newValue) {
	SimTK::Stage stg = _configState->getSystemStage();
	coord.setClamped(*_configState, newValue );
 	_model->getMultibodySystem().realize(*_configState, stg);
   return;
}

bool OpenSimContext::getClamped(const Coordinate& coord) {
	return coord.getClamped(*_configState);
}


void OpenSimContext::setLocked(const Coordinate& coord, bool newValue) {
	// This invalidates state back to Model, try to restore by re-realizing
	SimTK::Stage stg = _configState->getSystemStage();
	coord.setLocked(*_configState, newValue );
	_model->getMultibodySystem().realize(*_configState, stg);
    return;
}

bool OpenSimContext::isConstrained(const Coordinate& coord) const {
	return (coord.isConstrained(*_configState));
}

// Muscles
double OpenSimContext::getActivation(Muscle& m) {
	return m.getActivation(*_configState);
}

double OpenSimContext::getMuscleLength(Muscle& m) {
	return m.getLength(*_configState);
}

const Array<PathPoint*>& OpenSimContext::getCurrentPath(Muscle& m) {
	return m.getGeometryPath().getCurrentPath(*_configState);
}

const Array<PathPoint*>& OpenSimContext::getCurrentDisplayPath(Muscle& m) {
	m.getGeometryPath().updateGeometry(*_configState);
	return m.getGeometryPath().getCurrentDisplayPath(*_configState);
}

void OpenSimContext::updateDisplayer(Muscle& m) {
	return m.updateDisplayer(*_configState);
}

void OpenSimContext::copyMuscle(Muscle& from, Muscle& to) {
	to.copy(from);
	_configState->invalidateAll(SimTK::Stage::Position);
	_model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
	to.getGeometryPath().updateGeometry(*_configState);
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
	_model->setDefaultsFromState(*_configState);
	SimTK::State* newState = &_model->initSystem();
	setState(newState);
	return;
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
	_configState->invalidateAll(SimTK::Stage::Position);
	_model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
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

// Muscle Wrapping
void OpenSimContext::setStartPoint(PathWrap& mw, int newStartPt) {
     mw.setStartPoint(*_configState, newStartPt );
     return;
}

void OpenSimContext::addPathWrap(GeometryPath& p, WrapObject& awo) {
    p.addPathWrap( *_configState, awo );
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
    _configState=&aModel->updMultibodySystem().updDefaultState();
	bool retValue= modelScaler.processModel(*_configState, aModel, aPathToSubject, aFinalMass);
	// Model has changed need to recreate a valid state 
	aModel->getMultibodySystem().realizeTopology();
    _configState=&aModel->updMultibodySystem().updDefaultState();
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
	_model->setDefaultsFromState(*_configState);
	SimTK::State* newState = &_model->initSystem();
	setState(newState);
}

// Force re-realization
void OpenSimContext::realizePosition() {
	_model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);

}
void OpenSimContext::realizeVelocity() {
	_model->getMultibodySystem().realize(*_configState, SimTK::Stage::Velocity);

}
}	// namespace
