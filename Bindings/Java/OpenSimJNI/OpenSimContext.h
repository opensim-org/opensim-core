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
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Jack Middleton, Ayman Habib                                     *
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyTransform.h>
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
class AbstractPathPoint;
class PathWrap;
class ConditionalPathPoint;
class WrapObject;
class Analysis;
class AnalyzeTool;
class ModelScaler;
class MarkerPlacer;
class MarkerData;
class Measurement;

// Flag to indicate whether calls to the API are made from within try/catch block
// so that exceptions due to misuse, typos etc. are handled gracefully in scripts
// Set to true by default.
static bool mapCxxExceptionsToJava = true;

//==============================================================================
//                                 OpenSimContext
//==============================================================================
/** Class intended to keep the SimTK::State under an OpenSim model to make it possible
to get/set values in the SimTK::State without exposing the SimTK::State class itself.

The class provides convenient methods to get/set various state entries and query the 
state for cache values. The main function this class provides is an adaptor of various
data types from Java and scripting supported primitive, wrapped and array types to the 
corresponding possibly templatized or SimTK native data types.

Most methods of this class are implementated by delegating the call to the SimTK::State
under the object, for example:
Context::isDisabled(const Force& force) -> force.isDisabled(state)

The class also provides convenient services to recreateSystem and realize to various stages.

@author Ayman Habib & Jack Middleton 
**/

class OpenSimContext : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(OpenSimContext, Object);


public:
    OpenSimContext(SimTK::State* s, Model* model);

    void setState( SimTK::State* s) { _configState = s; }
    void setModel( Model* m) { _model = m; }

    /** Get reference to the single instance of SimTK::State maintained by the Context object **/
    const SimTK::State& getCurrentStateRef() const { return (*_configState); };
    /** Return a "clone" of  the single instance of SimTK::State maintained by the Context object **/
    SimTK::State getCurrentStateCopy() const { return SimTK::State(*_configState); };
        void recreateSystemAfterSystemExistsKeepStage(); 
        void recreateSystemAfterSystemExists(); 
        void resetStateToDefault() {
             SimTK::Stage stageBeforeRecreatingSystem = _configState->getSystemStage();
             SimTK::State* newState = &_model->initSystem();
             setState( newState );
            _model->getMultibodySystem().realize( *_configState, stageBeforeRecreatingSystem );
        }
    // Transforms
    void transformPosition(const PhysicalFrame& body, double offset[], double gOffset[]);
    SimTK::Transform getTransform(const PhysicalFrame& body);
    void transform(const PhysicalFrame& ground, double d[], PhysicalFrame& body, double dragVectorBody[]);
    // Coordinates
    double getValue(const Coordinate& coord);
    bool getLocked(const Coordinate& coord);
    void setValue(const Coordinate& coord, double d, bool enforceConstraints=true);
    void setClamped(Coordinate& coord, bool newValue);
    bool getClamped(const Coordinate& coord);
    void setLocked(Coordinate& coord, bool newValue);
    bool isPrescribed(const Coordinate& coord) const;
    bool isConstrained(const Coordinate& coord) const;
    // Constraints
    bool isEnforced(const Constraint& constraint) const {
        return constraint.isEnforced(*_configState);
    }
    void setIsEnforced(Constraint& constraint, bool isEnforced) {
        constraint.setIsEnforced(*_configState, isEnforced);
        _model->assemble(*_configState);
    }
    // Forces
    bool appliesForce(const Force& force) const {
        return  force.appliesForce(*_configState);
    }
    void setAppliesForce(Force& force, bool applyForce) const {
        force.setAppliesForce(*_configState, applyForce);
        _model->getMultibodySystem().realize(*_configState,
                                             SimTK::Stage::Position);
    }
    // Muscles
    double getActivation(Muscle& act);
    double getMuscleLength(Muscle& act);
    const Array<AbstractPathPoint*>& getCurrentPath(Muscle& act);
    void copyMuscle(Muscle& from, Muscle& to);
    void replacePropertyFunction(OpenSim::Object& obj, OpenSim::Function* aOldFunction, OpenSim::Function* aNewFunction);

    // Muscle Points
    void setXFunction(MovingPathPoint& mmp, Function& newFunction);
    void setYFunction(MovingPathPoint& mmp, Function& newFunction);
    void setZFunction(MovingPathPoint& mmp, Function& newFunction);
    void setXCoordinate(MovingPathPoint& mmp, Coordinate& newCoord);
    void setYCoordinate(MovingPathPoint& mmp, Coordinate& newCoord);
    void setZCoordinate(MovingPathPoint& mmp, Coordinate& newCoord);
    void setBody(AbstractPathPoint& pathPoint, PhysicalFrame& newBody);
    void setCoordinate(ConditionalPathPoint& via, Coordinate& newCoord);
    void setRangeMin(ConditionalPathPoint& via, double d);
    void setRangeMax(ConditionalPathPoint& via, double d);
    bool replacePathPoint(GeometryPath& p, AbstractPathPoint& mp, AbstractPathPoint& newPoint);
    void setLocation(PathPoint& mp, int i, double d);
    void setEndPoint(PathWrap& mw, int newEndPt);
    void addPathPoint(GeometryPath& p, int menuChoice, PhysicalFrame& body);
    bool deletePathPoint(GeometryPath& p, int menuChoice);
    bool isActivePathPoint(AbstractPathPoint& mp) ; 
    // Muscle Wrapping
    void setStartPoint(PathWrap& mw, int newStartPt);
    void addPathWrap(GeometryPath& p, WrapObject& awo);
    void moveUpPathWrap(GeometryPath& p, int num);
    void moveDownPathWrap(GeometryPath& p, int num);
    void deletePathWrap(GeometryPath& p, int num);
    // Markers
    void setBody(Marker& currentMarker, PhysicalFrame& newBody, bool  b);
    int replaceMarkerSet(Model& model, MarkerSet& aMarkerSet);

    void getCenterOfMassInGround(double com[3]) const {
        SimTK::Vec3 comV = _model->getMatterSubsystem().calcSystemMassCenterLocationInGround(*_configState);
        for(int i=0; i<3; i++) com[i] = comV[i];
    }
    // Analyses
    int step(Analysis& analysis);
    // Tools
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
    // Convert SimTK::Transform into a double[] array of 16 doubles
    static void getTransformAsDouble16(const SimTK::Transform& aTransform, double flattened[]){
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
        SimTK::Vector y1 = _configState->getY();
        SimTK::State* newState = &_model->initSystem();
        newState->updY() = y1;
        setState( newState );
        _model->getMultibodySystem().realize( *_configState, stageBeforeRecreatingSystem );
    }
    // Force re-realization
    void realizePosition();
    void realizeVelocity();

    void cacheModelAndState();
    void restoreStateFromCachedModel()  SWIG_DECLARE_EXCEPTION;
//=============================================================================
// DATA
//=============================================================================

private:
    // SimTK::State supporting the OpenSim::Model 
    SimTK::State* _configState;
    // The OpenSim::model 
    Model* _model;

    Model* clonedModel;
    SimTK::State clonedState;
}; // class OpenSimContext

//==============================================================================
//                                 OpenSimJavaObject
//==============================================================================
/**
In some cases, the GUI ad/or scripting language needs to create objects that derive from OpenSim::Object
The class OpenSim::Object however is not a concrete class, so we introduce OpenSimJavaObject
for this purpose 
**/
class OpenSimJavaObject : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(OpenSimJavaObject, Object);
};

class AdhocModelComponent : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(AdhocModelComponent, ModelComponent);
};
//==============================================================================
//                                 AnalysisWrapper
//==============================================================================
/**
Class used as base class for Java classes deriving from Analysis (used to be callback)
It lives on the C++ side so that it gets access to SimTK::State, but it returns quantities
in Java data types
**/

class AnalysisWrapper : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(AnalysisWrapper, Analysis);
public:
    AnalysisWrapper(Model *aModel=0):
      Analysis(aModel){
    }
    virtual ~AnalysisWrapper() {}
}; // Class AnalysisWrapper


//==============================================================================
//                                 InterruptCallback
//==============================================================================
/**
Class used to handle interrupts (synchronously). Works by adding it as an analysis
And when the client (GUI in most cases) decides to interrupt the simulation/analysis,
it calls the interrupt() method. When the step method is invoked later, an exception 
is thrown.
**/
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

//==============================================================================
//                                 PropertyHelper
//==============================================================================
/**
This class allows access to property values using template-free
 methods. Note that this will work regardless of whether the given
 AbstractProperty is the deprecated kind or the new one.

 An AbstractProperty represents a (name, list-of-values) pair, possibly
 with restrictions on the minimum and maximum list length. Basic container
 methods size(), resize(), clear(), and empty() are available; use resize()
 before assigning a value to an indexed element.

 For properties that contain objects, you can obtain the values directly
 from the base class via non-templatized methods.
 **/
class PropertyHelper {
public:
    //=================Boolean Properties==================
    // Recover boolean value from an AbstractProperty that was assumed to contain a boolean
    // Will throw exception if the assumption was wrong/invalid. Use index only if the
    // property contains an array of booleans.
    static bool getValueBool(const AbstractProperty& p, int index=-1) 
    {   return p.getValue<bool>(index); }
    // Set boolean value in an AbstractProperty that was assumed to hold a boolean
    // Will throw exception if the assumption was wrong/invalid. Use index only if the
    // property contains an array of booleans.
    static void setValueBool(bool v, AbstractProperty& p, int index=-1) 
    {   p.updValue<bool>(index) = v; }
    // Append a new boolean value to an AbstractProperty that was assumed to hold a variable size
    // array of booleans. Will throw exception if the assumption was wrong/invalid. 
    static void appendValueBool(bool v, AbstractProperty& p) 
    {   p.appendValue<bool>(v); }
    //=================Int Properties, see Boolean Properties for details ==================
    static int getValueInt(const AbstractProperty& p, int index=-1) 
    {   return p.getValue<int>(index); }
    static void setValueInt(int v, AbstractProperty& p, int index=-1) 
    {   p.updValue<int>(index) = v; }
    static void appendValueInt(int v, AbstractProperty& p) 
    {   p.appendValue<int>(v); }
    //=================Double Properties, see Boolean Properties for details ==================
    static double getValueDouble(const AbstractProperty& p, int index=-1) 
    {   return p.getValue<double>(index); }
    static void setValueDouble(double v, AbstractProperty& p, int index=-1) 
    {   p.updValue<double>(index) = v; }
    static void appendValueDouble(double v, AbstractProperty& p) 
    {   p.appendValue<double>(v); }
    //=================String Properties, see Boolean Properties for details ==================
    static std::string getValueString(const AbstractProperty& p, int index=-1) 
    {   return p.getValue<std::string>(index); }
    static void setValueString(const std::string& v, 
                               AbstractProperty& p, int index=-1) 
    {   p.updValue<std::string>(index) = v; }
    static void appendValueString(const std::string& v, AbstractProperty& p) 
    {   p.appendValue<std::string>(v); }
    //=================Transform Properties, treated as six Doubles ==================
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
    //=================Vec3 Properties, treated as three Doubles ==================
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
    // ================ String arrays ===================================================
    static OpenSim::Array<std::string> getValueStringArray(const AbstractProperty& p)
    {
        OpenSim::Array<std::string> val = OpenSim::Array<std::string>();
        for (int i=0; i< p.size(); i++)
            val.append(p.getValue<std::string>(i));
        return val;
    }
    static void setValueStringArray(AbstractProperty& p,  OpenSim::Array<std::string>& aStringArray)
    {
        p.clear();
        for (int i=0; i< aStringArray.getSize(); i++)
            try {
                p.appendValue<std::string>(aStringArray.get(i));
            } catch (OpenSim::Exception e) {
                OpenSim::Exception ex("ERROR- Invalid input (invalid character/spaces in input string)");
                throw ex;
            }
    }

    static void removeItem(AbstractProperty& p, int index)
    {
        if (p.size()>index){
            AbstractProperty* cloneP = p.clone();
            p.clear();
            for(int i=0; i<cloneP->size();i++){
                if (i!= index){
                if (p.getTypeName()=="string")
                    p.appendValue(cloneP->getValue<std::string>(i));
                else if (p.getTypeName()=="int")
                    p.appendValue(cloneP->getValue<int>(i));
                else if (p.getTypeName()=="double")
                    p.appendValue(cloneP->getValue<double>(i));
                else if (p.getTypeName()=="bool")
                    p.appendValue(cloneP->getValue<bool>(i));
                }
            }
        }
    }

};

} // namespace OpenSim

#endif // OPENSIM_OPENSIM_CONTEXT_H_

