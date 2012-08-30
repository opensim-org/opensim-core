#ifndef OPENSIM_OPENSIM_CONTEXT_H_
#define OPENSIM_OPENSIM_CONTEXT_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  OpenSimContext.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Jack Middleton, Ayman Habib                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>

#include "Simbody.h"

namespace OpenSim {

class Body;
class Coordinate;
class TransformAxis;
class Function;
class Marker;
class MarkerSet;
class Model;
class MovingPathPoint;
class Muscle;
class GeometryPath;
class PathPoint;
class PathWrap;
class ConditionalPathPoint;
class WrapObject;
class Analysis;
class AnalyzeTool;
class ModelScaler;
class MarkerPlacer;
class MarkerData;
class Measurement;

static bool mapCxxExceptionsToJava = true;


class OpenSimContext : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(OpenSimContext, Object);
//=============================================================================
// DATA
//=============================================================================

private:
    SimTK::State* _configState;
    Model* _model;

public:
    OpenSimContext(SimTK::State* s, Model* model);

    void setState( SimTK::State* s) { _configState = s; }
    void setModel( Model* m) { _model = m; }

	// States
	void setStates( Array<double>& states);
	void setStates( double statesBuffer[]);
	void computeConstrainedCoordinates( double statesBuffer[]);
	void getStates( double statesBuffer[]);
	void getStates( Array<double>&  rStates);
        void recreateSystemAfterSystemExistsKeepStage(); 
        void recreateSystemAfterSystemExists(); 

	// Transforms
    void transformPosition(const Body& body, double offset[], double gOffset[]);
    Transform getTransform(const Body& body);
	void transform(const Body& ground, double d[], Body& body, double dragVectorBody[]);
	// Coordinates
    double getValue(const Coordinate& coord);
	bool getLocked(const Coordinate& coord);
	void setValue(const Coordinate& coord, double d, bool enforceConstraints=true);
	void setClamped(Coordinate& coord, bool newValue);
	bool getClamped(const Coordinate& coord);
	void setLocked(Coordinate& coord, bool newValue);
	bool isConstrained(const Coordinate& coord) const;
	// Constraints
	bool isDisabled(const Constraint& constraint) const {
		return  constraint.isDisabled(*_configState);
	}
	void setDisabled(Constraint& constraint, bool disable) {
		constraint.setDisabled(*_configState, disable);
		_model->assemble(*_configState);
	}
	// Forces
	bool isDisabled(const Force& force) const {
		return  force.isDisabled(*_configState);
	}
	void setDisabled(Force& force, bool disable) const {
		force.setDisabled(*_configState, disable);
		_model->getMultibodySystem().realize(*_configState, SimTK::Stage::Position);
	}
	// Muscles
	double getActivation(Muscle& act);
	double getMuscleLength(Muscle& act);
	const Array<PathPoint*>& getCurrentPath(Muscle& act);
	const Array<PathPoint*>& getCurrentDisplayPath(GeometryPath& path);
	void updateDisplayer(Force& f);
    void copyMuscle(Muscle& from, Muscle& to);
	void replacePropertyFunction(OpenSim::Object& obj, OpenSim::Function* aOldFunction, OpenSim::Function* aNewFunction);

	// Muscle Points
	void setXFunction(MovingPathPoint& mmp, Function& newFunction);
    void setYFunction(MovingPathPoint& mmp, Function& newFunction);
    void setZFunction(MovingPathPoint& mmp, Function& newFunction);
    void setXCoordinate(MovingPathPoint& mmp, Coordinate& newCoord);
    void setYCoordinate(MovingPathPoint& mmp, Coordinate& newCoord);
    void setZCoordinate(MovingPathPoint& mmp, Coordinate& newCoord);
    void setBody(PathPoint& pathPoint, Body& newBody);
    void setCoordinate(ConditionalPathPoint& via, Coordinate& newCoord);
    void setRangeMin(ConditionalPathPoint& via, double d);
    void setRangeMax(ConditionalPathPoint& via, double d);
    bool replacePathPoint(GeometryPath& p, PathPoint& mp, PathPoint& newPoint);
    void setLocation(PathPoint& mp, int i, double d);
    void setEndPoint(PathWrap& mw, int newEndPt);
	void addPathPoint(GeometryPath& p, int menuChoice, Body& body);
	bool deletePathPoint(GeometryPath& p, int menuChoice);
	bool isActivePathPoint(PathPoint& mp) ; 
	// Muscle Wrapping
	void setStartPoint(PathWrap& mw, int newStartPt);
	void addPathWrap(GeometryPath& p, WrapObject& awo);
    void moveUpPathWrap(GeometryPath& p, int num);
    void moveDownPathWrap(GeometryPath& p, int num);
    void deletePathWrap(GeometryPath& p, int num);
	// Markers
	void setBody(Marker& currentMarker, Body& newBody, bool  b);
	int replaceMarkerSet(Model& model, MarkerSet& aMarkerSet);

	void getCenterOfMassInGround(double com[3]) const {
		SimTK::Vec3 comV = _model->getMatterSubsystem().calcSystemMassCenterLocationInGround(*_configState);
		for(int i=0; i<3; i++) com[i] = comV[i];
	}
	// Analyses
	int step(Analysis& analysis);
	// Tools
	//bool initializeTrial(IKTool& ikTool, int i);
	//bool solveTrial( IKTool& ikTool, int i);
	bool solveInverseKinematics( InverseKinematicsTool& ikTool);
	void setStatesFromMotion(AnalyzeTool& analyzeTool, const Storage &aMotion, bool aInDegrees);
	void loadStatesFromFile(AnalyzeTool& analyzeTool);
	bool processModelScale(ModelScaler& modelScaler, 
		Model* aModel, const std::string& aPathToSubject="", double aFinalMass = -1.0);
	bool processModelMarkerPlacer( MarkerPlacer& markerPlacer, 
		Model* aModel, const std::string& aPathToSubject="");
	double computeMeasurementScaleFactor(ModelScaler& modelScaler, 
		const Model& aModel, const MarkerData& aMarkerData, const Measurement& aMeasurement) const;
   void replaceTransformAxisFunction(TransformAxis& aDof, OpenSim::Function& aFunction);

	// Utilities
    static bool isNaN( double v ) { return (SimTK::isNaN(v)); }

	double getTime() { 
		assert(_configState); 
		return (_configState->getTime()); 
	}

	static void getTransformAsDouble16(const Transform& aTransform, double flattened[]){
		 double* matStart = &aTransform.toMat44()[0][0];
		 for (int i=0; i<16; i++) flattened[i]=matStart[i];
	}
    // Sets the property values in the model from the current state if there
    // are state variables that correspond to properties.
	void setPropertiesFromState() {	
		_model->setPropertiesFromState(*_configState);
	}
    /**
     * Create a new System under the model then realize it to the same stage it had
     */
    void recreateSystemKeepStage() {
        SimTK::Stage stageBeforeRecreatingSystem = _configState->getSystemStage();
        SimTK::State* newState = &_model->initSystem();
        setState( newState );
        _model->getMultibodySystem().realize( *_configState, stageBeforeRecreatingSystem );
    }
	// Force re-realization
	void realizePosition();
	void realizeVelocity();

}; // class OpenSimContext

// Concrete class to be used on the GUI side
class OpenSimJavaObject : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(OpenSimJavaObject, Object);
};

// Class used as base class for Java classes deriving from Analysis (used to be callback)
// It lives on the C++ side so that it gets access to SimTK::State.
class AnalysisWrapper : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(AnalysisWrapper, Analysis);
	double* statesCache;
	int		statesCacheSize;
	double  simulationTime;
public:
	AnalysisWrapper(Model *aModel=0):
	  Analysis(aModel),
	  statesCacheSize(aModel->getNumStateVariables()){
		statesCache = new double[statesCacheSize];
		simulationTime = -1.0;
	}
	virtual int step( const SimTK::State& s, int stepNumber) {
        Array<double> rStateValues;
		//std::string statedump= s.toString();
		_model->getStateValues(s, rStateValues);
        for(int i=0;i<statesCacheSize;i++ ) 
			*(statesCache+i) = rStateValues[i];
		simulationTime = s.getTime();
		return 0;
	}
	void getStates( double statesBuffer[]){
		for(int i=0;i<statesCacheSize;i++ ) 
			*(statesBuffer+i) = statesCache[i];
	}
	double getSimulationTime() {
		return simulationTime;
	}

}; // Class AnalysisWrapper


// Class to handle interrupts
class InterruptCallback : public AnalysisWrapper {
	bool _throwException;
public:
	InterruptCallback(Model *aModel=0):
	  AnalysisWrapper(aModel),
	  _throwException(false){};

	void interrupt() {
		_throwException=true;
	}
	virtual int step( const SimTK::State& s, int stepNumber) {
		if (_throwException)
			throw Exception("Operation Aborted");
		return 0;
	}
	
};


// This class allows access to property values using template-free
// methods. Note that this will work regardless of whether the given
// AbstractProperty is the deprecated kind or the new one.
//
// An AbstractProperty represents a (name, list-of-values) pair, possibly
// with restrictions on the minimum and maximum list length. Basic container
// methods size(), resize(), clear(), and empty() are available; use resize()
// before assigning a value to an indexed element.
//
// For properties that contain objects, you can obtain the values directly
// from the base class via non-templatized methods.
class PropertyHelper {
public:
    static bool getValueBool(const AbstractProperty& p, int index=-1) 
    {   return p.getValue<bool>(index); }
    static void setValueBool(bool v, AbstractProperty& p, int index=-1) 
    {   p.updValue<bool>(index) = v; }
    static void appendValueBool(bool v, AbstractProperty& p) 
    {   p.appendValue<bool>(v); }

    static int getValueInt(const AbstractProperty& p, int index=-1) 
    {   return p.getValue<int>(index); }
    static void setValueInt(int v, AbstractProperty& p, int index=-1) 
    {   p.updValue<int>(index) = v; }
    static void appendValueInt(int v, AbstractProperty& p) 
    {   p.appendValue<int>(v); }

    static double getValueDouble(const AbstractProperty& p, int index=-1) 
    {   return p.getValue<double>(index); }
    static void setValueDouble(double v, AbstractProperty& p, int index=-1) 
    {   p.updValue<double>(index) = v; }
    static void appendValueDouble(double v, AbstractProperty& p) 
    {   p.appendValue<double>(v); }

    static std::string getValueString(const AbstractProperty& p, int index=-1) 
    {   return p.getValue<std::string>(index); }
    static void setValueString(const std::string& v, 
                               AbstractProperty& p, int index=-1) 
    {   p.updValue<std::string>(index) = v; }
    static void appendValueString(const std::string& v, AbstractProperty& p) 
    {   p.appendValue<std::string>(v); }

    static double getValueTransform(const AbstractProperty& p, int index) 
    {   
        const PropertyTransform& pd = dynamic_cast<const PropertyTransform&>(p);
        double array6[] = {0., 0., 0., 0., 0., 0.};
        pd.getRotationsAndTranslationsAsArray6(array6);
        return array6[index]; 
    }
    static void setValueTransform(double v, AbstractProperty& p, int index) 
    {   
        PropertyTransform& pd = dynamic_cast<PropertyTransform&>(p);
        double array6[] = {0., 0., 0., 0., 0., 0.};
        pd.getRotationsAndTranslationsAsArray6(array6);
        array6[index] = v; 
        pd.setValue(6, array6);
    }

    static double getValueVec3(const AbstractProperty& p, int index) 
    {   
        const Property<SimTK::Vec3>& pd = dynamic_cast<const Property<SimTK::Vec3>&>(p);
        const SimTK::Vec3& vec3 = pd.getValue();
        return vec3[index]; 
    }
    static void setValueVec3(double v, AbstractProperty& p, int index) 
    {   
        Property<SimTK::Vec3>& pd = dynamic_cast<Property<SimTK::Vec3>&>(p);
        pd.updValue()[index] = v;
     }

};

} // namespace OpenSim

#endif // OPENSIM_OPENSIM_CONTEXT_H_

