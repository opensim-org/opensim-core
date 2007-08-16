%module(directors="1") opensimModel
%module opensimModel
%{
#include <xercesc/util/XercesVersion.hpp>
#include <xercesc/util/XercesDefs.hpp>
#include <OpenSim/Common/osimCommonDLL.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyGroup.h>
#include <OpenSim/Common/PropertySet.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ObjectGroup.h>
#include <OpenSim/Common/Material.h>
#include <OpenSim/Common/VisibleProperties.h>
#include <OpenSim/Common/Transform.h>
#include <OpenSim/Common/Geometry.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/MaterialSet.h>
#include <OpenSim/Common/StateVector.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Range.h>
#include <OpenSim/Common/Scale.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Units.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/version.h>

#include <OpenSim/Simulation/Model/AbstractActuator.h>
#include <OpenSim/Simulation/Model/ActuatorSet.h>
#include <OpenSim/Simulation/Model/ContactForceSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Control/Control.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Integrator/Integrand.h>
#include <OpenSim/Simulation/Integrator/RKF.h>
#include <OpenSim/Simulation/Integrator/IntegRKF.h>
#include <OpenSim/Simulation/Model/ModelIntegrand.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Callback.h>
#include <OpenSim/Simulation/Model/CallbackSet.h>
#include <OpenSim/Simulation/Model/IntegCallback.h>
#include <OpenSim/Simulation/Model/InterruptingIntegCallback.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/AbstractTool.h>
#include <OpenSim/Simulation/Model/AbstractMarker.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>

#include <OpenSim/Java/OpenSimJNI/Hooks/SimtkAnimationCallback.h>
#include <OpenSim/Java/OpenSimJNI/Hooks/SimtkLogCallback.h>

#include <OpenSim/Tools/osimToolsDLL.h>
#include <OpenSim/Tools/ForwardTool.h>
#include <OpenSim/Tools/PerturbationTool.h>

#include <OpenSim/Analyses/osimAnalysesDLL.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/IndAcc.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/GeneralizedForces.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include <OpenSim/Analyses/InverseDynamics.h>

#include <OpenSim/Simulation/Wrap/AbstractWrapObject.h>
#include <OpenSim/Simulation/Wrap/MuscleWrapPoint.h>
#include <OpenSim/Simulation/Wrap/WrapSphere.h>
#include <OpenSim/Simulation/Wrap/WrapCylinder.h>
#include <OpenSim/Simulation/Wrap/WrapTorus.h>
#include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
#include <OpenSim/Simulation/Wrap/WrapObjectSet.h>
#include <OpenSim/Simulation/Wrap/MuscleWrap.h>
#include <OpenSim/Simulation/Wrap/MuscleWrapSet.h>

#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/osimSimmKinematicsEngineDLL.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmBody.h>
#include <OpenSim/Simulation/Model/BodySet.h>

#include <OpenSim/Simulation/Model/BodyScale.h>
#include <OpenSim/Simulation/Model/BodyScaleSet.h>

#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmCoordinate.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>

#include <OpenSim/Simulation/Model/AbstractDof.h>

#include <OpenSim/Simulation/Model/AbstractJoint.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmJoint.h>
#include <OpenSim/Simulation/Model/JointSet.h>

#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/Model/AbstractMarker.h>
#include <OpenSim/Simulation/Model/Marker.h>

#include <OpenSim/Simulation/Model/MusclePoint.h>
#include <OpenSim/Simulation/Model/MusclePointSet.h>
#include <OpenSim/Simulation/Model/MuscleViaPoint.h>
#include <OpenSim/Common/SimmPoint.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmRotationDof.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmTranslationDof.h>

#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/Model/GeneralizedForce.h>
#include <OpenSim/Actuators/Torque.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Actuators/Schutte1993Muscle.h>

#include <OpenSim/Tools/IKTrial.h>
#include <OpenSim/Tools/IKTrialSet.h>

#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmFileWriter.h>
#include <OpenSim/Tools/IKTask.h>
#include <OpenSim/Tools/IKMarkerTask.h>
#include <OpenSim/Tools/IKCoordinateTask.h>
#include <OpenSim/Tools/IKTaskSet.h>
#include <OpenSim/Common/MarkerData.h>

#include <OpenSim/Tools/IKSolverInterface.h>
#include <OpenSim/Tools/MarkerPair.h>
#include <OpenSim/Tools/MarkerPairSet.h>
#include <OpenSim/Tools/Measurement.h>
#include <OpenSim/Tools/MeasurementSet.h>

#include <OpenSim/Tools/GenericModelMaker.h>
#include <OpenSim/Tools/ModelScaler.h>
#include <OpenSim/Tools/MarkerPlacer.h>

#include <OpenSim/Tools/IKTool.h>
#include <OpenSim/Tools/IKSolverImpl.h>

#include <OpenSim/Tools/CMCTool.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Tools/AnalyzeTool.h>

#include <OpenSim/Tools/AnalyzeTool.h>

using namespace OpenSim;
%}

%feature("director") OpenSim::SimtkAnimationCallback;
%feature("director") OpenSim::SimtkLogCallback;


%rename(OpenSimObject) OpenSim::Object;
%rename(OpenSimException) OpenSim::Exception;

/* This file is for creation/handling of arrays */
%include "arrays_java.i";

/* This interface file is for better handling of pointers and references */
%include "typemaps.i"
%include "std_string.i"


%typemap(javacode) OpenSim::Object %{
  public boolean equals(Object obj) {
    boolean equal = false;
    if (obj instanceof $javaclassname)
      equal = ((($javaclassname)obj).swigCPtr == this.swigCPtr);
    return equal;
  }
  private int cacheId=-1;  // cache the Id to avoid recomputation for hashing purposes
 
  public int hashCode() {
     if (cacheId==-1)
        cacheId=(int)swigCPtr;
     
    return( cacheId );
  }
  // Flag to indicate if an object is pickable in the GUI
  // Example of a non-pickable object would be a MuscleWrapPoint
  private boolean pickable=true;
  
  public boolean isPickable() {
	 return pickable;
  }
  
  public void setPickable(boolean onOff) {
	 pickable=onOff;
  }
  
%}

%typemap(javacode) OpenSim::MarkerData %{
  public double[] getTimeRange() { return new double[]{getStartFrameTime(), getLastFrameTime()}; }
%}

%typemap(javacode) OpenSim::Array<std::string> %{
   public java.util.Vector<String> toVector() {
      java.util.Vector<String> vector = new java.util.Vector<String>();
      vector.setSize(getSize());
      for(int i=0; i<getSize(); i++) vector.set(i, getitem(i));
      return vector;
   }
   public void append(java.util.Vector<String> vector) {
      for(int i=0; i<vector.size(); i++) append(vector.get(i));
   }
   public static ArrayStr fromVector(java.util.Vector<String> vector) {
      ArrayStr array = new ArrayStr();
      array.append(vector);
      return array;
   }
%}

%pragma(java) jniclassclassmodifiers="public class"

%pragma(java) jniclassimports="import org.opensim.utils.TheApp;"

%pragma(java) jniclasscode=%{
  static {
      try{
        System.loadLibrary("osimJavaJNI");		// All OpenSim classes required for GUI operation.
        System.loadLibrary("osimSdfastEngine");	//to load sdfast based models
        System.loadLibrary("osimSimbodyEngine");	//to load sdfast based models
      }
      catch(UnsatisfiedLinkError e){
           TheApp.exitApp("Required library failed to load. Check that the dynamic library osimJavaJNI is in your PATH\n"+e);
      }
  }
%}

/* make getCPtr public not package private */
%typemap(javabody) SWIGTYPE, SWIGTYPE *, SWIGTYPE &, SWIGTYPE [], SWIGTYPE (CLASS::*)%{
  private long swigCPtr;
  protected boolean swigCMemOwn;

  public $javaclassname(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  public static long getCPtr($javaclassname obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }
%}

%typemap(javabody_derived) SWIGTYPE %{
  private long swigCPtr;

  public $javaclassname(long cPtr, boolean cMemoryOwn) {
    super(opensimModelJNI.SWIGStorageUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  public static long getCPtr($javaclassname obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }
%}
// Generic Exception handling
%typemap(throws) SWIGTYPE, SWIGTYPE &, SWIGTYPE *, SWIGTYPE [ANY] %{
  SWIG_JavaThrowException(jenv, SWIG_JavaIOException,
                          "C++ $1_type exception thrown");
  return $null;
%}

%typemap(throws, throws="java.io.IOException") OpenSim::Exception {
  jclass excep = jenv->FindClass("java/io/IOException");
  if (excep)
    jenv->ThrowNew(excep, ($1).getMessage());
  return $null;
}

%exception OpenSim::AnalyticGeometry::dynamic_cast(Geometry *geometry) {
    $action
    if (!result) {
        jclass excep = jenv->FindClass("java/lang/ClassCastException");
        if (excep) {
            jenv->ThrowNew(excep, "dynamic_cast exception");
        }
    }
}
%extend OpenSim::AnalyticGeometry {
    static AnalyticGeometry *dynamic_cast(Geometry *geometry) {
        return dynamic_cast<AnalyticGeometry *>(geometry);
    }
};

%extend OpenSim::LineGeometry {
    static LineGeometry *dynamic_cast(Geometry *geometry) {
        return dynamic_cast<LineGeometry *>(geometry);
    }
};

%extend OpenSim::AnalyticSphere {
    static AnalyticSphere *dynamic_cast(Geometry *geometry) {
        return dynamic_cast<AnalyticSphere *>(geometry);
    }
};

%extend OpenSim::AnalyticCylinder {
    static AnalyticCylinder *dynamic_cast(Geometry *geometry) {
        return dynamic_cast<AnalyticCylinder *>(geometry);
    }
};

%extend OpenSim::AnalyticEllipsoid {
    static AnalyticEllipsoid *dynamic_cast(Geometry *geometry) {
        return dynamic_cast<AnalyticEllipsoid *>(geometry);
    }
};

%extend OpenSim::AnalyticTorus {
    static AnalyticTorus *dynamic_cast(Geometry *geometry) {
        return dynamic_cast<AnalyticTorus *>(geometry);
    }
};

/* rest of header files to be wrapped */
%include <OpenSim/Common/osimCommonDLL.h>
%include <OpenSim/Common/Exception.h>
%include <OpenSim/Common/Array.h>
%include <OpenSim/Common/ArrayPtrs.h>
%include <OpenSim/Common/Property.h>
%include <OpenSim/Common/PropertyStr.h>
%template(ArrayPtrsProperty) OpenSim::ArrayPtrs<OpenSim::Property>;
%include <OpenSim/Common/PropertyGroup.h>
%template(ArrayPtrsPropertyGroup) OpenSim::ArrayPtrs<OpenSim::PropertyGroup>;
%include <OpenSim/Common/PropertySet.h>
%include <OpenSim/Common/Object.h>
%include <OpenSim/Common/ObjectGroup.h>
%include <OpenSim/Common/VisibleProperties.h>
%include <OpenSim/Common/Transform.h>
%include <OpenSim/Common/Geometry.h>
%include <OpenSim/Common/VisibleObject.h>
%include <OpenSim/Common/Set.h>
%include <OpenSim/Common/StateVector.h>
%include <OpenSim/Common/Storage.h>
%include <OpenSim/Common/Units.h>
%include <OpenSim/Common/rdMath.h>
%include <OpenSim/Common/IO.h>
%include <OpenSim/version.h>

%include <OpenSim/Simulation/osimSimulationDLL.h>
%include <OpenSim/Simulation/Model/AbstractActuator.h>
%template(SetActuators) OpenSim::Set<OpenSim::AbstractActuator>;
%include <OpenSim/Simulation/Model/ActuatorSet.h>
%include <OpenSim/Simulation/Model/ContactForceSet.h>
%include <OpenSim/Simulation/Model/Callback.h>
%template(SetCallback) OpenSim::Set<OpenSim::Callback>;
%include <OpenSim/Simulation/Model/CallbackSet.h>
%include <OpenSim/Simulation/Model/IntegCallback.h>
%include <OpenSim/Simulation/Model/InterruptingIntegCallback.h>
%template(ArrayStorage) OpenSim::ArrayPtrs<OpenSim::Storage>;
%include <OpenSim/Simulation/Model/Analysis.h>
%template(SetAnalysis) OpenSim::Set<OpenSim::Analysis>;
%include <OpenSim/Simulation/Model/AnalysisSet.h>
%include <OpenSim/Simulation/Model/Model.h>

%include <OpenSim/Simulation/Control/Control.h>
%template(SetControls) OpenSim::Set<OpenSim::Control>;
%include <OpenSim/Simulation/Control/ControlSet.h>
%include <OpenSim/Simulation/Control/ControlConstant.h>
%include <OpenSim/Simulation/Control/ControlLinear.h>
%include <OpenSim/Simulation/Control/Controller.h>

%include <OpenSim/Simulation/Manager/Manager.h>
%include <OpenSim/Simulation/Model/AbstractTool.h>
%include <OpenSim/Analyses/osimAnalysesDLL.h>
%include <OpenSim/Tools/osimToolsDLL.h>
%include <OpenSim/Tools/ForwardTool.h>
%include <OpenSim/Tools/PerturbationTool.h>

%include <OpenSim/Java/OpenSimJNI/Hooks/SimtkAnimationCallback.h>
%include <OpenSim/Java/OpenSimJNI/Hooks/SimtkLogCallback.h>

%include <OpenSim/Analyses/osimAnalysesDLL.h>
%include <OpenSim/Analyses/Kinematics.h>
%include <OpenSim/Analyses/Actuation.h>
%include <OpenSim/Analyses/IndAcc.h>
%include <OpenSim/Analyses/GeneralizedForces.h>
%include <OpenSim/Analyses/MuscleAnalysis.h>
%include <OpenSim/Analyses/InverseDynamics.h>

%template(ArrayBool) OpenSim::Array<bool>;
%template(ArrayDouble) OpenSim::Array<double>;
%template(ArrayInt) OpenSim::Array<int>;
%template(ArrayStr) OpenSim::Array<std::string>;
%template(ArrayObjPtr) OpenSim::Array<OpenSim::Object*>;
%template(ArrayPtrsObj) OpenSim::ArrayPtrs<OpenSim::Object>;
%include <OpenSim/Simulation/Model/AbstractMarker.h>
%template(SetMarkers) OpenSim::Set<OpenSim::AbstractMarker>;
%include <OpenSim/Simulation/Model/MarkerSet.h>
%include <OpenSim/Common/Range.h>
%include <OpenSim/Common/Scale.h>
%template(SetScales) OpenSim::Set<OpenSim::Scale>;
%include <OpenSim/Common/ScaleSet.h>

%include <OpenSim/Simulation/Wrap/AbstractWrapObject.h>
%include <OpenSim/Simulation/Wrap/WrapSphere.h>
%include <OpenSim/Simulation/Wrap/WrapCylinder.h>
%include <OpenSim/Simulation/Wrap/WrapTorus.h>
%include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
%template(SetWrapObject) OpenSim::Set<OpenSim::AbstractWrapObject>;
%include <OpenSim/Simulation/Wrap/WrapObjectSet.h>
%include <OpenSim/Simulation/Wrap/MuscleWrap.h>
%template(SetMuscleWrap) OpenSim::Set<OpenSim::MuscleWrap>;
%include <OpenSim/Simulation/Wrap/MuscleWrapSet.h>

%include <OpenSim/Simulation/Model/AbstractBody.h>
%include <OpenSim/DynamicsEngines/SimmKinematicsEngine/osimSimmKinematicsEngineDLL.h>
%include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmBody.h>
%template(SetBodies) OpenSim::Set<OpenSim::AbstractBody>;
%include <OpenSim/Simulation/Model/BodySet.h>

%include <OpenSim/Simulation/Model/BodyScale.h>
%template(SetBodyScales) OpenSim::Set<OpenSim::BodyScale>;
%include <OpenSim/Simulation/Model/BodyScaleSet.h>

%include <OpenSim/Common/Function.h>
%include <OpenSim/Common/Constant.h>

%include <OpenSim/Simulation/Model/AbstractDof.h>
%include <OpenSim/Simulation/Model/AbstractCoordinate.h>
%include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmCoordinate.h>
%template(SetCoordinates) OpenSim::Set<OpenSim::AbstractCoordinate>;
%include <OpenSim/Simulation/Model/CoordinateSet.h>

%include <OpenSim/Simulation/Model/AbstractJoint.h>
%include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmJoint.h>
%template(SetJoints) OpenSim::Set<OpenSim::AbstractJoint>;
%include <OpenSim/Simulation/Model/JointSet.h>

%include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
%include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmKinematicsEngine.h>

%include <OpenSim/Simulation/Model/AbstractMarker.h>

%include <OpenSim/Simulation/Model/MusclePoint.h>
%include <OpenSim/Simulation/Wrap/MuscleWrapPoint.h>
%include <OpenSim/Simulation/Model/MuscleViaPoint.h>
%template(SetMusclePoint) OpenSim::Set<OpenSim::MusclePoint>;
%template(ArrayMusclePoint) OpenSim::Array<OpenSim::MusclePoint*>;
%include <OpenSim/Simulation/Model/MusclePointSet.h>

%include <OpenSim/Actuators/osimActuatorsDLL.h>
%include <OpenSim/Simulation/Model/AbstractMuscle.h>
%include <OpenSim/Simulation/Model/Force.h>
%include <OpenSim/Simulation/Model/GeneralizedForce.h>
%include <OpenSim/Actuators/Torque.h>
%include <OpenSim/Actuators/Thelen2003Muscle.h>
%include <OpenSim/Actuators/Schutte1993Muscle.h>

%include <OpenSim/Common/SimmPoint.h>
%include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmRotationDof.h>
%include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmTranslationDof.h>

%include <OpenSim/Tools/IKTrial.h>
%template(SetIKTrial) OpenSim::Set<OpenSim::IKTrial>;
%include <OpenSim/Tools/IKTrialSet.h>

%include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmFileWriter.h>
%include <OpenSim/Tools/IKTask.h>
%template(SetIKTasks) OpenSim::Set<OpenSim::IKTask>;
%include <OpenSim/Tools/IKMarkerTask.h>
%include <OpenSim/Tools/IKCoordinateTask.h>
%include <OpenSim/Tools/IKTaskSet.h>
%include <OpenSim/Common/MarkerData.h>

%include <OpenSim/Tools/IKSolverInterface.h>
%include <OpenSim/Tools/IKTool.h>
%include <OpenSim/Tools/MarkerPair.h>
%template(SetMarkerPairs) OpenSim::Set<OpenSim::MarkerPair>;
%include <OpenSim/Tools/MarkerPairSet.h>
%include <OpenSim/Tools/Measurement.h>
%template(SetMeasurements) OpenSim::Set<OpenSim::Measurement>;
%include <OpenSim/Tools/MeasurementSet.h>
%include <OpenSim/Tools/GenericModelMaker.h>
%include <OpenSim/Tools/ModelScaler.h>
%include <OpenSim/Tools/MarkerPlacer.h>
%include <OpenSim/Tools/IKSolverImpl.h>
%include <OpenSim/Tools/CMCTool.h>
%include <OpenSim/Tools/ScaleTool.h>
%include <OpenSim/Tools/AnalyzeTool.h>
