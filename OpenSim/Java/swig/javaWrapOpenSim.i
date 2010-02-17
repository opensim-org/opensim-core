%module(directors="1") opensimModel
%module opensimModel
%{
#include <xercesc/util/XercesVersion.hpp>
#include <xercesc/util/XercesDefs.hpp>
#include <OpenSim/version.h>
#include <SimTKsimbody.h>
#include <OpenSim/Common/osimCommonDLL.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/OldVersionException.h>
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
#include <OpenSim/Common/Geometry.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/StateVector.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Range.h>
#include <OpenSim/Common/Scale.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Units.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/NaturalCubicSpline.h>
#include <OpenSim/Common/StepFunction.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/MultiplierFunction.h>
#include <OpenSim/Common/XYFunctionInterface.h>

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/Model/CustomForce.h>
#include <OpenSim/Simulation/Model/PrescribedForce.h>
#include <OpenSim/Simulation/Model/Ligament.h>
#include <OpenSim/Simulation/Model/ContactGeometry.h>
#include <OpenSim/Simulation/Model/ContactGeometrySet.h>

#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/CustomActuator.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Control/Control.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include <OpenSim/Simulation/Control/ControlLinearNode.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/AbstractTool.h>
#include <OpenSim/Simulation/Model/Marker.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>

#include <OpenSim/Tools/osimToolsDLL.h>
#include <OpenSim/Tools/ForwardTool.h>

#include <OpenSim/Analyses/osimAnalysesDLL.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include <OpenSim/Analyses/InverseDynamics.h>
#include <OpenSim/Analyses/StaticOptimization.h>

#include <OpenSim/Simulation/Wrap/WrapObject.h>
#include <OpenSim/Simulation/Wrap/PathWrapPoint.h>
#include <OpenSim/Simulation/Wrap/WrapSphere.h>
#include <OpenSim/Simulation/Wrap/WrapCylinder.h>
#include <OpenSim/Simulation/Wrap/WrapTorus.h>
#include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
#include <OpenSim/Simulation/Wrap/WrapObjectSet.h>
#include <OpenSim/Simulation/Wrap/PathWrap.h>
#include <OpenSim/Simulation/Wrap/PathWrapSet.h>
#include <OpenSim/Simulation/Wrap/WrapCylinderObst.h>
#include <OpenSim/Simulation/Wrap/WrapSphereObst.h>
#include <OpenSim/Simulation/Wrap/WrapDoubleCylinderObst.h>

#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/Model/BodySet.h>

#include <OpenSim/Simulation/Model/BodyScale.h>
#include <OpenSim/Simulation/Model/BodyScaleSet.h>

#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>

#include <OpenSim/Simulation/SimbodyEngine/TransformAxis.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>

#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/Model/JointSet.h>

#include <OpenSim/Simulation/Model/Marker.h>

#include <OpenSim/Simulation/Model/PathPoint.h>
#include <OpenSim/Simulation/Model/PathPointSet.h>
#include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
#include <OpenSim/Simulation/Model/MovingPathPoint.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>

#include <OpenSim/Common/SimmPoint.h>

#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Actuators/Schutte1993Muscle.h>
#include <OpenSim/Actuators/Delp1990Muscle.h>

#include <OpenSim/Tools/IKTrial.h>
#include <OpenSim/Tools/IKTrialSet.h>

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
#include <OpenSim/Tools/PerturbationTool.h>
#include <OpenSim/Java/OpenSimJNI/Hooks/SimtkLogCallback.h>

#include <OpenSim/Utilities/simmFileWriterDLL/SimmFileWriter.h>

#include <OpenSim/Java/OpenSimJNI/OpenSimContext.h>

using namespace OpenSim;
using namespace SimTK;
using OpenSim::Event;

static bool trace=false;
%}

%feature("director") OpenSim::AnalysisWrapper;
%feature("director") OpenSim::SimtkLogCallback;

%feature("notabstract") ControlLinear;

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
  // Example of a non-pickable object would be a PathWrapPoint
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

%typemap(javacode) OpenSim::Model %{
  private String originalModelPath = null;
  // Important that we only refer to originalModelPath if the model's getInputFileName() is not set
  public void setOriginalModelPathFromModel(Model model) {
    originalModelPath = null;
    if(model.getInputFileName()!=null && !model.getInputFileName().equals(""))
      originalModelPath = (new java.io.File(model.getInputFileName())).getParent();
	 else if(model.originalModelPath!=null && !model.originalModelPath.equals(""))
      originalModelPath = model.originalModelPath;
  }
  public String getFilePath() {
    if(getInputFileName()!=null && !getInputFileName().equals("") && (new java.io.File(getInputFileName())).getParent()!=null)
      return (new java.io.File(getInputFileName())).getParent() + java.io.File.separator;
    else if(originalModelPath!=null && !originalModelPath.equals(""))
      return originalModelPath + java.io.File.separator;
    else return "";
  }
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

%typemap(javacode) OpenSim::XYFunctionInterface %{
  private Function  dFunction;  // cache pointer to function so it's not garbage collected early

  public XYFunctionInterface(Function aFunction, Boolean unused) {
		this(aFunction);
		dFunction = aFunction;
  }
%}

%pragma(java) jniclassclassmodifiers="public class"

%pragma(java) jniclassimports="import org.opensim.utils.TheApp;"

%pragma(java) jniclasscode=%{
  static {
      try{
        System.loadLibrary("osimJavaJNI");		// All OpenSim classes required for GUI operation.
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

%exception {
  if (Object::getDebugLevel()>=3){
	  try {
	  $action
	  if (trace)
		std::cout << "In Function:" << __FUNCTION__ << std::endl;
	  }
	  catch(std::exception& _ex){
		  jclass excep = jenv->FindClass("java/lang/RuntimeException");
		  if (excep)
		  jenv->ThrowNew(excep,_ex.what());
	  }
  }
  else
	$action
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
    static OpenSim::AnalyticGeometry *dynamic_cast(OpenSim::Geometry *geometry) {
        return dynamic_cast<OpenSim::AnalyticGeometry *>(geometry);
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

%extend OpenSim::PolyhedralGeometry {
    static PolyhedralGeometry *dynamic_cast(Geometry *geometry) {
        return dynamic_cast<PolyhedralGeometry *>(geometry);
    }
};

/* rest of header files to be wrapped */
%include <OpenSim/version.h>
// osimCommon Library
%include <OpenSim/Common/osimCommonDLL.h>
%include <OpenSim/Common/Exception.h>
%include <OpenSim/Common/OldVersionException.h>
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
%include <OpenSim/Common/Geometry.h>
%include <OpenSim/Common/VisibleObject.h>
%include <OpenSim/Common/Set.h>
%include <OpenSim/Common/StateVector.h>
%include <OpenSim/Common/Storage.h>
%include <OpenSim/Common/Units.h>
%include <OpenSim/Common/IO.h>
%include <OpenSim/Common/Function.h>
%include <OpenSim/Common/Constant.h>
%include <OpenSim/Common/NaturalCubicSpline.h>
%include <OpenSim/Common/StepFunction.h>
%include <OpenSim/Common/LinearFunction.h>
%include <OpenSim/Common/PiecewiseLinearFunction.h>
%include <OpenSim/Common/MultiplierFunction.h>
%include <OpenSim/Common/XYFunctionInterface.h>
%template(ArrayXYPoint) OpenSim::Array<XYPoint>;
%template(ArrayBool) OpenSim::Array<bool>;
%template(ArrayDouble) OpenSim::Array<double>;
%template(ArrayInt) OpenSim::Array<int>;
%template(ArrayStr) OpenSim::Array<std::string>;
%template(ArrayObjPtr) OpenSim::Array<OpenSim::Object*>;
%template(ArrayPtrsObj) OpenSim::ArrayPtrs<OpenSim::Object>;
%include <OpenSim/Common/Scale.h>
%template(SetScales) OpenSim::Set<OpenSim::Scale>;
%include <OpenSim/Common/ScaleSet.h>
%include <OpenSim/Common/MarkerData.h>
%include <OpenSim/Common/SimmPoint.h>

// osimSimulation
%include <OpenSim/Simulation/osimSimulationDLL.h>
%include <OpenSim/Simulation/Model/ModelComponent.h>
%template(SetModelComponents) OpenSim::Set<OpenSim::ModelComponent>;
%include <OpenSim/Simulation/Model/ModelComponentSet.h>

%include <OpenSim/Simulation/Model/Force.h>
%template(SetForces) OpenSim::Set<OpenSim::Force>;
%template(ModelComponentSetForces) OpenSim::ModelComponentSet<OpenSim::Force>;
%include <OpenSim/Simulation/Model/ForceSet.h>
%include <OpenSim/Simulation/Model/CustomForce.h>
%include <OpenSim/Simulation/Model/PrescribedForce.h>
%include <OpenSim/Simulation/Model/Ligament.h>

%include <OpenSim/Simulation/Model/ContactGeometry.h>
%template(SetContactGeometry) OpenSim::Set<OpenSim::ContactGeometry>;
%template(ModelComponentSetContactGeometry) OpenSim::ModelComponentSet<OpenSim::ContactGeometry>;
%include <OpenSim/Simulation/Model/ContactGeometrySet.h>

%include <OpenSim/Simulation/Model/Actuator.h>
%template(SetActuators) OpenSim::Set<OpenSim::Actuator>;
%include <OpenSim/Simulation/Model/CustomActuator.h>
%template(ArrayStorage) OpenSim::ArrayPtrs<OpenSim::Storage>;
%include <OpenSim/Simulation/Model/Analysis.h>
%template(SetAnalysis) OpenSim::Set<OpenSim::Analysis>;
%include <OpenSim/Simulation/Model/AnalysisSet.h>

%include <OpenSim/Simulation/Control/Control.h>
%template(SetControls) OpenSim::Set<OpenSim::Control>;
%include <OpenSim/Simulation/Control/ControlSet.h>
%include <OpenSim/Simulation/Control/ControlConstant.h>
%include <OpenSim/Simulation/Control/ControlLinearNode.h>
%template(SetControlNodes) OpenSim::ArrayPtrs<OpenSim::ControlLinearNode>;
%include <OpenSim/Simulation/Control/ControlLinear.h>
%include <OpenSim/Simulation/Control/Controller.h>

%include <OpenSim/Simulation/Manager/Manager.h>
%include <OpenSim/Simulation/Model/AbstractTool.h>
%include <OpenSim/Simulation/Model/Marker.h>
%template(SetMarkers) OpenSim::Set<OpenSim::Marker>;
%include <OpenSim/Simulation/Model/MarkerSet.h>

%include <OpenSim/Simulation/Wrap/WrapObject.h>
%include <OpenSim/Simulation/Wrap/WrapSphere.h>
%include <OpenSim/Simulation/Wrap/WrapCylinder.h>
%include <OpenSim/Simulation/Wrap/WrapTorus.h>
%include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
%template(SetWrapObject) OpenSim::Set<OpenSim::WrapObject>;
%include <OpenSim/Simulation/Wrap/WrapObjectSet.h>
%include <OpenSim/Simulation/Wrap/PathWrap.h>
%template(SetPathWrap) OpenSim::Set<OpenSim::PathWrap>;
%include <OpenSim/Simulation/Wrap/PathWrapSet.h>
%include <OpenSim/Simulation/Wrap/WrapCylinderObst.h>
%include <OpenSim/Simulation/Wrap/WrapSphereObst.h>
%include <OpenSim/Simulation/Wrap/WrapDoubleCylinderObst.h>

%include <OpenSim/Simulation/SimbodyEngine/Body.h>
%template(SetBodies) OpenSim::Set<OpenSim::Body>;
%template(ModelComponentSetBodies) OpenSim::ModelComponentSet<OpenSim::Body>;
%include <OpenSim/Simulation/Model/BodySet.h>

%include <OpenSim/Simulation/Model/BodyScale.h>
%template(SetBodyScales) OpenSim::Set<OpenSim::BodyScale>;
%include <OpenSim/Simulation/Model/BodyScaleSet.h>

%include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
%include <OpenSim/Simulation/SimbodyEngine/TransformAxis.h>
%include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
%include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
%template(SetCoordinates) OpenSim::Set<OpenSim::Coordinate>;
%include <OpenSim/Simulation/Model/CoordinateSet.h>

%include <OpenSim/Simulation/SimbodyEngine/Joint.h>
%include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
%template(SetJoints) OpenSim::Set<OpenSim::Joint>;
%template(ModelComponentSetJoints) OpenSim::ModelComponentSet<OpenSim::Joint>;
%include <OpenSim/Simulation/Model/JointSet.h>

%include <OpenSim/Simulation/Model/Model.h>

%include <OpenSim/Simulation/Model/PathPoint.h>
%include <OpenSim/Simulation/Wrap/PathWrapPoint.h>
%include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
%include <OpenSim/Simulation/Model/MovingPathPoint.h>
%template(SetPathPoint) OpenSim::Set<OpenSim::PathPoint>;
%template(ArrayPathPoint) OpenSim::Array<OpenSim::PathPoint*>;
%include <OpenSim/Simulation/Model/PathPointSet.h>
%include <OpenSim/Simulation/Model/GeometryPath.h>
%include <OpenSim/Simulation/Model/Muscle.h>

//osimAnalyses
%include <OpenSim/Analyses/osimAnalysesDLL.h>
%include <OpenSim/Analyses/Kinematics.h>
%include <OpenSim/Analyses/Actuation.h>
%include <OpenSim/Analyses/MuscleAnalysis.h>
%include <OpenSim/Analyses/InverseDynamics.h>
%include <OpenSim/Analyses/StaticOptimization.h>

//osimActuators
%include <OpenSim/Actuators/osimActuatorsDLL.h>
%include <OpenSim/Actuators/CoordinateActuator.h>
%include <OpenSim/Actuators/Thelen2003Muscle.h>
%include <OpenSim/Actuators/Schutte1993Muscle.h>
%include <OpenSim/Actuators/Delp1990Muscle.h>

//osimTools
%include <OpenSim/Tools/osimToolsDLL.h>
%include <OpenSim/Tools/IKTrial.h>
%template(SetIKTrial) OpenSim::Set<OpenSim::IKTrial>;
%include <OpenSim/Tools/IKTrialSet.h>
%include <OpenSim/Tools/IKTask.h>
%template(SetIKTasks) OpenSim::Set<OpenSim::IKTask>;
%include <OpenSim/Tools/IKMarkerTask.h>
%include <OpenSim/Tools/IKCoordinateTask.h>
%include <OpenSim/Tools/IKTaskSet.h>
%include <OpenSim/Tools/IKSolverInterface.h>
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
%include <OpenSim/Tools/ScaleTool.h>
%include <OpenSim/Tools/IKTool.h>
%include <OpenSim/Tools/ForwardTool.h>
%include <OpenSim/Tools/CMCTool.h>
%include <OpenSim/Tools/AnalyzeTool.h>
%include <OpenSim/Tools/PerturbationTool.h>

%include <OpenSim/Utilities/simmFileWriterDLL/SimmFileWriter.h>

%include <OpenSim/Java/OpenSimJNI/OpenSimContext.h>

//OpenSim20%include <OpenSim/Java/OpenSimJNI/Hooks/SimtkAnimationCallback.h>
%include <OpenSim/Java/OpenSimJNI/Hooks/SimtkLogCallback.h>
