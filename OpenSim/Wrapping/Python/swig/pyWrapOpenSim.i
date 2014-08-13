%module(directors="1") opensim
%module opensim
#pragma SWIG nowarn=822,451,503,516,325

/*
For consistency with the rest of the API, we use camel-case for variable names.
This breaks Python PEP 8 convention, but allows us to be consistent within our
own project.
*/

%{
#define SWIG_FILE_WITH_INIT


#include <OpenSim/version.h>
#include <SimTKsimbody.h>
#include <OpenSim/Common/osimCommonDLL.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/AbstractProperty.h>
#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/PropertyGroup.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ObjectGroup.h>
#include <OpenSim/Common/Geometry.h>
#include <OpenSim/Common/DisplayGeometry.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/GeometrySet.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/StateVector.h>
#include <OpenSim/Common/StorageInterface.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Scale.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Units.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Common/StepFunction.h>
#include <OpenSim/Common/PiecewiseConstantFunction.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/MultiplierFunction.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/Sine.h>
#include <OpenSim/Common/PolynomialFunction.h>
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>
#include <OpenSim/Common/SmoothSegmentedFunction.h>
#include <OpenSim/Common/XYFunctionInterface.h>
#include <OpenSim/Common/FunctionSet.h>

#include <OpenSim/Common/LoadOpenSimLibrary.h>

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>
#include <OpenSim/Simulation/Model/ComponentSet.h>

#include <OpenSim/Simulation/Solver.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Simulation/MomentArmSolver.h>

#include <OpenSim/Simulation/Model/Frame.h>
#include <OpenSim/Simulation/Model/FixedFrame.h>

#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/Model/PrescribedForce.h>
#include <OpenSim/Simulation/Model/CoordinateLimitForce.h>
#include <OpenSim/Simulation/Model/ExternalForce.h>
#include <OpenSim/Simulation/Model/ContactGeometry.h>
#include <OpenSim/Simulation/Model/ContactHalfSpace.h>
#include <OpenSim/Simulation/Model/ContactMesh.h>
#include <OpenSim/Simulation/Model/ContactSphere.h>

#include <OpenSim/Simulation/Model/ElasticFoundationForce.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce.h>

#include <OpenSim/Simulation/Model/ContactGeometrySet.h>
#include <OpenSim/Simulation/Model/Probe.h>
#include <OpenSim/Simulation/Model/ProbeSet.h>
#include <OpenSim/Simulation/Model/SystemEnergyProbe.h>
#include <OpenSim/Simulation/Model/JointInternalPowerProbe.h>
#include <OpenSim/Simulation/Model/ActuatorPowerProbe.h>
#include <OpenSim/Simulation/Model/ActuatorForceProbe.h>
#include <OpenSim/Simulation/Model/MuscleActiveFiberPowerProbe.h>
#include <OpenSim/Simulation/Model/Bhargava2004MuscleMetabolicsProbe.h>
#include <OpenSim/Simulation/Model/Umberger2010MuscleMetabolicsProbe.h>

#include <OpenSim/Simulation/Model/ModelDisplayHints.h>
#include <OpenSim/Simulation/Model/ModelVisualizer.h>

#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/ModelVisualizer.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/FixedFrame.h>

#include <OpenSim/Simulation/Control/Control.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include <OpenSim/Simulation/Control/ControlLinearNode.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Control/PrescribedController.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/ControllerSet.h>
#include <OpenSim/Simulation/Model/ExternalLoads.h>
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
#include <OpenSim/Analyses/ForceReporter.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Analyses/JointReaction.h>
#include <OpenSim/Analyses/StatesReporter.h>
#include <OpenSim/Analyses/InducedAccelerations.h>
#include <OpenSim/Analyses/ProbeReporter.h>

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
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/EllipsoidJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/BallJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/GimbalJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/UniversalJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
#include <OpenSim/Simulation/Model/JointSet.h>

#include <OpenSim/Simulation/Model/Marker.h>

#include <OpenSim/Simulation/Model/PathPoint.h>
#include <OpenSim/Simulation/Model/PathPointSet.h>
#include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
#include <OpenSim/Simulation/Model/MovingPathPoint.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>
#include <OpenSim/Simulation/Model/Ligament.h>

#include <OpenSim/Simulation/SimbodyEngine/Constraint.h>
#include <OpenSim/Simulation/Model/ConstraintSet.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/PointConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/ConstantDistanceConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/PointOnLineConstraint.h>

#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/PathActuator.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>
#include <OpenSim/Simulation/Model/ExpressionBasedPointToPointForce.h>
#include <OpenSim/Simulation/Model/PointToPointSpring.h>
#include <OpenSim/Simulation/Model/PathSpring.h>
#include <OpenSim/Simulation/Model/BushingForce.h>
#include <OpenSim/Simulation/Model/FunctionBasedBushingForce.h>
#include <OpenSim/Simulation/Model/ExpressionBasedBushingForce.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Actuators/PointActuator.h>
#include <OpenSim/Actuators/TorqueActuator.h>
#include <OpenSim/Actuators/PointToPointActuator.h>
#include <OpenSim/Actuators/ClutchedPathSpring.h>
#include <OpenSim/Actuators/SpringGeneralizedForce.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Actuators/RigidTendonMuscle.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Actuators/Millard2012AccelerationMuscle.h>
#include <OpenSim/Actuators/ClutchedPathSpring.h>

#include <OpenSim/Tools/IKTask.h>
#include <OpenSim/Tools/IKMarkerTask.h>
#include <OpenSim/Tools/IKCoordinateTask.h>
#include <OpenSim/Tools/IKTaskSet.h>
#include <OpenSim/Common/MarkerData.h>

#include <OpenSim/Tools/MarkerPair.h>
#include <OpenSim/Tools/MarkerPairSet.h>
#include <OpenSim/Tools/Measurement.h>
#include <OpenSim/Tools/MeasurementSet.h>

#include <OpenSim/Tools/GenericModelMaker.h>
#include <OpenSim/Tools/ModelScaler.h>
#include <OpenSim/Tools/MarkerPlacer.h>
#include <OpenSim/Tools/Tool.h>

#include <OpenSim/Simulation/Solver.h>
#include <OpenSim/Simulation/AssemblySolver.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/CoordinateReference.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Tools/DynamicsTool.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>

#include <OpenSim/Tools/CMCTool.h>
#include <OpenSim/Tools/RRATool.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>

#include <OpenSim/Wrapping/Python/OpenSimContext.h>

using namespace OpenSim;
using namespace SimTK;

%}

%feature("director") OpenSim::AnalysisWrapper;
%feature("director") OpenSim::SimtkLogCallback;
%feature("director") SimTK::DecorativeGeometryImplementation;
%feature("notabstract") ControlLinear;

/** Pass Doxygen documentation to python wrapper */
%feature("autodoc", "3");

%rename(OpenSimObject) OpenSim::Object;
%rename(OpenSimException) OpenSim::Exception;

%rename(printToXML) OpenSim::Object::print(const std::string&);
%rename(printToXML) OpenSim::XMLDocument::print(const std::string&);
%rename(printToXML) OpenSim::XMLDocument::print();
%rename(printToFile) OpenSim::Storage::print;

/* If needed %extend will be used, these operators are not supported.*/
%ignore *::operator[];
%ignore *::operator=;

/* This file is for creation/handling of arrays */
%include "std_carray.i";

/* This interface file is for better handling of pointers and references */
%include "typemaps.i"
%include "std_string.i"
/*
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

%}

%typemap(javacode) OpenSim::MarkerData %{
  public double[] getTimeRange() { return new double[]{getStartFrameTime(), getLastFrameTime()}; }
%}

%typemap(javacode) OpenSim::Array<double> %{
	public void fromString(String string) {
      // Remove open and close parenth if any
      String workString= new String(string);
      int liveStart = workString.indexOf("(");
      int liveEnd = workString.indexOf(")");
      if (liveStart!=-1 && liveEnd!=-1){
          workString = workString.substring(liveStart+1, liveEnd);
      }
      else if (liveStart!=liveEnd){
          //throw new ParseException("Illegal format: Expect space separated values, optionally between matched parentheses", liveEnd);
          return;
      }
      String[] splits = workString.split(" ");
      double[] values = new double[splits.length];
      for(int i=0; i<splits.length; i++){
           values[i]=Double.parseDouble(splits[i]);
       }
       this.setValues(values, splits.length);
	}
%}

%javamethodmodifiers OpenSim::Model::addModelComponent "private";
%javamethodmodifiers OpenSim::Model::addBody "private";
%javamethodmodifiers OpenSim::Model::addConstraint "private";
%javamethodmodifiers OpenSim::Model::addForce "private";
%javamethodmodifiers OpenSim::Model::addProbe "private";
%javamethodmodifiers OpenSim::Model::addContactGeometry "private";
%javamethodmodifiers OpenSim::Model::addController "private";
%javamethodmodifiers OpenSim::Model::addAnalysis "private";

%rename OpenSim::Model::addModelComponent private_addModelComponent;
%rename OpenSim::Model::addBody private_addBody;
%rename OpenSim::Model::addConstraint private_addConstraint;
%rename OpenSim::Model::addForce private_addForce;
%rename OpenSim::Model::addProbe private_addProbe;
%rename OpenSim::Model::addContactGeometry private_addContactGeometry;
%rename OpenSim::Model::addController private_addController;
%rename OpenSim::Model::addAnalysis private_addAnalysis;


%typemap(javacode) OpenSim::FunctionSet %{
  public boolean adoptAndAppend(Function aFunction) {
	aFunction.markAdopted();
    return super.adoptAndAppend(aFunction);
  }
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

  public void addProbe(Probe aProbe) {
	aProbe.markAdopted();
    private_addProbe(aProbe);
  }  

  public void addForce(Force aForce) {
	aForce.markAdopted();
	private_addForce(aForce);
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

SWIG_JAVABODY_PROXY(public, public, SWIGTYPE)

%pragma(java) jniclassimports="import javax.swing.JOptionPane;"

%pragma(java) jniclasscode=%{
  static {
      try{
        System.loadLibrary("osimJavaJNI");		// All OpenSim classes required for GUI operation.
      }
      catch(UnsatisfiedLinkError e){
           new JOptionPane("Required library failed to load. Check that the dynamic library osimJavaJNI is in your PATH\n"+e, 
				JOptionPane.ERROR_MESSAGE).createDialog(null, "Error").setVisible(true);
      }
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
  if (OpenSim::mapCxxExceptionsToJava){
	  try {
	  $action
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

%extend OpenSim::Body {
	void getCenterOfMass(double dCom[3]) {
		self->getMassCenter(SimTK::Vec3::updAs(dCom));
	};
	void getInertia(Array<double>& rInertia) {
		SimTK::Mat33 inertia;
		self->getInertia(inertia);
		rInertia[0]=inertia[0][0];
		rInertia[1]=inertia[1][1];
		rInertia[2]=inertia[2][2];
		rInertia[3]=inertia[0][1];
		rInertia[4]=inertia[0][2];
		rInertia[5]=inertia[1][2];

	};
	void setInertia(Array<double>& aInertia) {
		self->setInertia(SimTK::Inertia(aInertia[0], 
		aInertia[1], aInertia[2], aInertia[3], aInertia[4], aInertia[5]));
	}
};
*/

// Memory management
// =================
/*
This facility will help us avoid segfaults that occur when two different
objects believe they own a pointer, and so they both try to delete it. We can
instead notify the object that something else has adopted it, and will take
care of deleting it.
*/
%extend OpenSim::Object {
%pythoncode %{
    def _markAdopted(self):
        if self.this and self.thisown:
            self.thisown = False
%}
};

/*
The added component is not responsible for its own memory management anymore
once added to the Model.  These lines must go at this point in this .i file. I
originally had them at the bottom, and then they didn't work!

note: ## is a "glue" operator: `a ## b` --> `ab`.
*/
%define MODEL_ADOPT_HELPER(NAME)
%pythonappend OpenSim::Model::add ## NAME %{
    args[0]._markAdopted()
%}
%enddef

MODEL_ADOPT_HELPER(ModelComponent);
MODEL_ADOPT_HELPER(Body);
MODEL_ADOPT_HELPER(Probe);
MODEL_ADOPT_HELPER(Constraint);
MODEL_ADOPT_HELPER(ContactGeometry);
MODEL_ADOPT_HELPER(Analysis);
MODEL_ADOPT_HELPER(Force);
MODEL_ADOPT_HELPER(Controller);

// Would prefer to modify the Joint abstract class constructor,
// but the proxy classes don't even call it.
%define JOINT_ADOPT_HELPER(NAME)
%pythonappend OpenSim::Joint:: ## NAME %{
    self._markAdopted()
%}
%enddef

JOINT_ADOPT_HELPER(FreeJoint);
JOINT_ADOPT_HELPER(CustomJoint);
JOINT_ADOPT_HELPER(EllipsoidJoint);
JOINT_ADOPT_HELPER(BallJoint);
JOINT_ADOPT_HELPER(PinJoint);
JOINT_ADOPT_HELPER(SliderJoint);
JOINT_ADOPT_HELPER(WeldJoint);
JOINT_ADOPT_HELPER(GimbalJoint);
JOINT_ADOPT_HELPER(UniversalJoint);
JOINT_ADOPT_HELPER(PlanarJoint);

%extend OpenSim::Array<double> {
	void appendVec3(SimTK::Vec3 vec3) {
		for(int i=0; i<3; i++)
			self->append(vec3[i]);
	}
	void appendVector(SimTK::Vector vec) {
		for(int i=0; i<vec.size(); i++)
			self->append(vec[i]);
	}

	SimTK::Vec3 getAsVec3() {
		return SimTK::Vec3::getAs(self->get());
	};
	
	static SimTK::Vec3 createVec3(double e1, double e2, double e3) {
		Array<double>* arr = new Array<double>(e1, 3);
		arr->set(1, e2);
		arr->set(2, e3);
		return SimTK::Vec3::getAs(arr->get());
	};
  
   static SimTK::Vec3 createVec3(double e1) {
		Array<double>* arr = new Array<double>(e1, 3);
		return SimTK::Vec3::getAs(arr->get());
  };
   
   static SimTK::Vec3  createVec3(double es[3]) {
		Array<double>* arr = new Array<double>(es[0], 3);
		arr->set(1, es[1]);
		arr->set(2, es[2]);
		return SimTK::Vec3::getAs(arr->get());
  };

   SimTK::Vector_<double>  getAsVector() {
		return SimTK::Vector(self->getSize(), &(*self)[0]);
  };

   void populateFromVector(SimTK::Vector_<double> aVector) {
		int sz = aVector.size();
		for(int i=0; i<sz; ++i)
			self->append(aVector[i]);
   }

   static  OpenSim::Array<double> getValuesFromVec3(SimTK::Vec3 vec3) {
		OpenSim::Array<double> arr(0, 3);
		for (int i=0; i<3; i++) arr[i] = vec3[i];
		return arr;
  };
  
  std::string toString() const {
		std::stringstream stream;
		for (int i=0; i< self->getSize(); i++)
			stream <<  self->get(i) << " ";
		return stream.str(); 
  }

  void setFromPyArray(double* dValues, int size) {
		self->setSize(size);
		for(int i=0; i< size; ++i)
		    self->set(i, dValues[i]);
};
};
/*
%extend OpenSim::Model {
	static void LoadOpenSimLibrary(std::string libraryName){
		LoadOpenSimLibrary(libraryName);
	}

	void setDefaultControls(SimTK::Vector& newControls) {
		self->updDefaultControls() = newControls;
	}
}

%extend OpenSim::Manager {
	void setIntegratorAccuracy(double accuracy){
		self->getIntegrator().setAccuracy(accuracy);
	}
}

%extend OpenSim::Object {
	static OpenSim::Array<std::string> getFunctionClassNames() {
		  OpenSim::Array<std::string> availableClassNames;
		  ArrayPtrs<OpenSim::Function> rArray;
		  Object::getRegisteredObjectsOfGivenType<OpenSim::Function>(rArray);
		  for (int i=0;i<rArray.size(); i++)
			availableClassNames.append(rArray[i]->getConcreteClassName());
		  
		  return availableClassNames;
	}
}

%include "numpy.i"

%init %{
import_array();
%}

%apply (double* IN_ARRAY1, int DIM1) {(double* dValues, int size)}
*/
/* rest of header files to be wrapped */
%include <OpenSim/version.h>
%include <SimTKcommon.h>

%include <SimTKcommon/Constants.h>
%include <SWIGSimTK/Vec.h>
%include <SimTKcommon/SmallMatrix.h>
// Vec3
namespace SimTK {
%template(Vec3) Vec<3>;
}
// Mat33
%include <SWIGSimTK/Mat.h>
namespace SimTK {
%template(Mat33) Mat<3, 3>;
}
// Vector and Matrix
%include <SWIGSimTK/BigMatrix.h>
namespace SimTK {
%template(MatrixBaseDouble) SimTK::MatrixBase<double>;
%template(VectorBaseDouble) SimTK::VectorBase<double>;
%template(Vector) SimTK::Vector_<double>;
%template(Matrix) SimTK::Matrix_<double>;
}

%include <SWIGSimTK/SpatialAlgebra.h>
namespace SimTK {
%template(SpatialVec) Vec<2,   Vec3>;
%template(VectorOfSpatialVec) Vector_<SpatialVec>;
%template(VectorOfVec3) Vector_<Vec3>;
}
// Transform
%include <SWIGSimTK/Transform.h>
namespace SimTK {
%template(Transform) SimTK::Transform_<double>;
}

%include <SWIGSimTK/MassProperties.h>
namespace SimTK {
%template(Inertia) SimTK::Inertia_<double>;
%template(MassProperties) SimTK::MassProperties_<double>;
}
%include <SWIGSimTK/common.h>
%include <SWIGSimTK/Array.h>

typedef int MobilizedBodyIndex;
typedef int SubsystemIndex;
typedef int SystemQIndex;
typedef int SystemQErrIndex;
typedef int SystemZIndex;
typedef int SystemYIndex;
typedef int SystemYErrIndex;
typedef int SystemUIndex;
typedef int SystemUErrIndex;
typedef int SystemUDotErrIndex;

namespace SimTK {
%template(ArrayIndexUnsigned) ArrayIndexTraits<unsigned>; 
%template(ArrayIndexInt) ArrayIndexTraits<int>; 
}

%include <SWIGSimTK/DecorativeGeometry.h>

namespace SimTK {
%template(ArrayDecorativeGeometry) SimTK::Array_<SimTK::DecorativeGeometry>;
}

// State & Stage
%include <SWIGSimTK/Stage.h>
%include <SWIGSimTK/State.h>

// osimCommon Library
%include <OpenSim/Common/osimCommonDLL.h>
%include <OpenSim/Common/Exception.h>
%include <OpenSim/Common/Array.h>
%include <OpenSim/Common/ArrayPtrs.h>
%include <OpenSim/Common/AbstractProperty.h>
%include <OpenSim/Common/Property.h>
%include <OpenSim/Common/PropertyGroup.h>
%template(ArrayPtrsPropertyGroup) OpenSim::ArrayPtrs<OpenSim::PropertyGroup>;
%include <OpenSim/Common/Object.h>
%include <OpenSim/Common/ObjectGroup.h>
%include <OpenSim/Common/Geometry.h>
%include <OpenSim/Common/DisplayGeometry.h>
%include <OpenSim/Common/Set.h>
%template(SetGeometry) OpenSim::Set<OpenSim::DisplayGeometry>;
%include <OpenSim/Common/GeometrySet.h>
%include <OpenSim/Common/VisibleObject.h>
%include <OpenSim/Common/StateVector.h>
%include <OpenSim/Common/StorageInterface.h>
%include <OpenSim/Common/Storage.h>
%include <OpenSim/Common/Units.h>
%include <OpenSim/Common/IO.h>
%include <OpenSim/Common/Function.h>

%template(SetFunctions) OpenSim::Set<OpenSim::Function>;
%include <OpenSim/Common/FunctionSet.h>

%include <OpenSim/Common/Constant.h>
%include <OpenSim/Common/SimmSpline.h>
%include <OpenSim/Common/StepFunction.h>
%include <OpenSim/Common/PiecewiseConstantFunction.h>
%include <OpenSim/Common/LinearFunction.h>
%include <OpenSim/Common/PiecewiseLinearFunction.h>
%include <OpenSim/Common/MultiplierFunction.h>
%include <OpenSim/Common/GCVSpline.h>
%include <OpenSim/Common/Sine.h>
%include <OpenSim/Common/PolynomialFunction.h>
%include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>
%include <OpenSim/Common/SmoothSegmentedFunction.h>

%include <OpenSim/Common/XYFunctionInterface.h>
%template(ArrayXYPoint) OpenSim::Array<XYPoint>;
%template(ArrayBool) OpenSim::Array<bool>;
%template(ArrayDouble) OpenSim::Array<double>;
%template(ArrayInt) OpenSim::Array<int>;
%template(ArrayStr) OpenSim::Array<std::string>;
%template(ArrayObjPtr) OpenSim::Array<OpenSim::Object*>;
%template(ArrayPtrsObj) OpenSim::ArrayPtrs<OpenSim::Object>;
%include <OpenSim/Common/ComponentOutput.h>
%include <OpenSim/Common/ComponentConnector.h>
%include <OpenSim/Common/Component.h>
%include <OpenSim/Common/Scale.h>
%template(SetScales) OpenSim::Set<OpenSim::Scale>;
%include <OpenSim/Common/ScaleSet.h>
%include <OpenSim/Common/MarkerData.h>

// osimSimulation
%include <OpenSim/Simulation/osimSimulationDLL.h>
%include <OpenSim/Simulation/Model/ModelComponent.h>
%template(SetModelComponents) OpenSim::Set<OpenSim::ModelComponent>;
%include <OpenSim/Simulation/Model/ModelComponentSet.h>
%template(ModelComponentSetModelComponent) OpenSim::ModelComponentSet<OpenSim::ModelComponent>;
%include <OpenSim/Simulation/Model/ComponentSet.h>

%template(SetMuscles) OpenSim::Set<OpenSim::Muscle>;

%include <OpenSim/Simulation/Solver.h>
%include <OpenSim/Simulation/InverseDynamicsSolver.h>
%include <OpenSim/Simulation/MomentArmSolver.h>

%include <OpenSim/Simulation/Model/Frame.h>
%include <OpenSim/Simulation/Model/FixedFrame.h>

%include <OpenSim/Simulation/Model/Force.h>
%template(SetForces) OpenSim::Set<OpenSim::Force>;
%template(ModelComponentSetForces) OpenSim::ModelComponentSet<OpenSim::Force>;
%include <OpenSim/Simulation/Model/ForceSet.h>
%include <OpenSim/Simulation/Model/ExternalForce.h>
%template(SetExternalForces) OpenSim::Set<OpenSim::ExternalForce>;

%include <OpenSim/Simulation/Control/Controller.h>
%template(SetControllers) OpenSim::Set<OpenSim::Controller>;
%template(ModelComponentSetControllers) OpenSim::ModelComponentSet<OpenSim::Controller>;
%include <OpenSim/Simulation/Model/ControllerSet.h>

%template(ModelComponentSetExternalForces) OpenSim::ModelComponentSet<OpenSim::ExternalForce>;
%include <OpenSim/Simulation/Model/ExternalLoads.h>
%include <OpenSim/Simulation/Model/PrescribedForce.h>
%include <OpenSim/Simulation/Model/CoordinateLimitForce.h>

%include <OpenSim/Simulation/Model/ContactGeometry.h>
%template(SetContactGeometry) OpenSim::Set<OpenSim::ContactGeometry>;
%template(ModelComponentSetContactGeometry) OpenSim::ModelComponentSet<OpenSim::ContactGeometry>;
%include <OpenSim/Simulation/Model/ContactGeometrySet.h>
%include <OpenSim/Simulation/Model/ContactHalfSpace.h>
%include <OpenSim/Simulation/Model/ContactMesh.h>
%include <OpenSim/Simulation/Model/ContactSphere.h>
%include <OpenSim/Simulation/Model/ElasticFoundationForce.h>
%include <OpenSim/Simulation/Model/HuntCrossleyForce.h>

%include <OpenSim/Simulation/Model/Actuator.h>
%template(SetActuators) OpenSim::Set<OpenSim::Actuator>;
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
%include <OpenSim/Simulation/Control/PrescribedController.h>

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
%template(ModelComponentSetCoordinates) OpenSim::ModelComponentSet<OpenSim::Coordinate>;
%include <OpenSim/Simulation/Model/CoordinateSet.h>

%include <OpenSim/Simulation/SimbodyEngine/Joint.h>
%include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/EllipsoidJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/BallJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/GimbalJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/UniversalJoint.h>
%include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>

%template(SetJoints) OpenSim::Set<OpenSim::Joint>;
%template(ModelComponentSetJoints) OpenSim::ModelComponentSet<OpenSim::Joint>;
%include <OpenSim/Simulation/Model/JointSet.h>


%include <OpenSim/Simulation/SimbodyEngine/Constraint.h>
%template(SetConstraints) OpenSim::Set<OpenSim::Constraint>;
%template(ModelComponentSetConstraints) OpenSim::ModelComponentSet<OpenSim::Constraint>;
%include <OpenSim/Simulation/Model/ConstraintSet.h>
%include <OpenSim/Simulation/SimbodyEngine/WeldConstraint.h>
%include <OpenSim/Simulation/SimbodyEngine/PointConstraint.h>
%include <OpenSim/Simulation/SimbodyEngine/ConstantDistanceConstraint.h>
%include <OpenSim/Simulation/SimbodyEngine/CoordinateCouplerConstraint.h>
%include <OpenSim/Simulation/SimbodyEngine/PointOnLineConstraint.h>

%include <OpenSim/Simulation/Model/Probe.h>
%template(SetProbes) OpenSim::Set<OpenSim::Probe>;
%template(ModelComponentSetProbes) OpenSim::ModelComponentSet<OpenSim::Probe>;
%include <OpenSim/Simulation/Model/ProbeSet.h>
%include <OpenSim/Simulation/Model/SystemEnergyProbe.h>
%include <OpenSim/Simulation/Model/JointInternalPowerProbe.h>
%include <OpenSim/Simulation/Model/ActuatorPowerProbe.h>
%include <OpenSim/Simulation/Model/ActuatorForceProbe.h>

%include <OpenSim/Simulation/Model/MuscleActiveFiberPowerProbe.h>
%include <OpenSim/Simulation/Model/Bhargava2004MuscleMetabolicsProbe.h>
%include <OpenSim/Simulation/Model/Umberger2010MuscleMetabolicsProbe.h>
%include <OpenSim/Simulation/Model/ModelDisplayHints.h>
%include <OpenSim/Simulation/Model/ModelVisualizer.h>
%include <OpenSim/Simulation/Model/Model.h>

%include <OpenSim/Simulation/Model/PathPoint.h>
%include <OpenSim/Simulation/Wrap/PathWrapPoint.h>
%include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
%include <OpenSim/Simulation/Model/MovingPathPoint.h>
%template(SetPathPoint) OpenSim::Set<OpenSim::PathPoint>;
%template(ArrayPathPoint) OpenSim::Array<OpenSim::PathPoint*>;
%include <OpenSim/Simulation/Model/PathPointSet.h>
%include <OpenSim/Simulation/Model/GeometryPath.h>
%include <OpenSim/Simulation/Model/Ligament.h>
%include <OpenSim/Simulation/Model/PathActuator.h>
%include <OpenSim/Simulation/Model/Muscle.h>
%include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>
%include <OpenSim/Simulation/Model/PointToPointSpring.h>
%include <OpenSim/Simulation/Model/ExpressionBasedPointToPointForce.h>
%include <OpenSim/Simulation/Model/PathSpring.h>
%include <OpenSim/Simulation/Model/BushingForce.h>
%include <OpenSim/Simulation/Model/FunctionBasedBushingForce.h>
%include <OpenSim/Simulation/Model/ExpressionBasedBushingForce.h>

//osimAnalyses
%include <OpenSim/Analyses/osimAnalysesDLL.h>
%include <OpenSim/Analyses/Kinematics.h>
%include <OpenSim/Analyses/Actuation.h>
%include <OpenSim/Analyses/MuscleAnalysis.h>
%include <OpenSim/Analyses/InverseDynamics.h>
%include <OpenSim/Analyses/StaticOptimization.h>
%include <OpenSim/Analyses/ForceReporter.h>
%include <OpenSim/Analyses/PointKinematics.h>
%include <OpenSim/Analyses/BodyKinematics.h>
%include <OpenSim/Analyses/JointReaction.h>
%include <OpenSim/Analyses/StatesReporter.h>
%include <OpenSim/Analyses/InducedAccelerations.h>
%include <OpenSim/Analyses/ProbeReporter.h>

//osimActuators
%include <OpenSim/Actuators/osimActuatorsDLL.h>
%include <OpenSim/Actuators/CoordinateActuator.h>
%include <OpenSim/Actuators/PointActuator.h>
%include <OpenSim/Actuators/TorqueActuator.h>
%include <OpenSim/Actuators/PointToPointActuator.h>
%include <OpenSim/Actuators/ClutchedPathSpring.h>
%include <OpenSim/Actuators/SpringGeneralizedForce.h>
%include <OpenSim/Actuators/Thelen2003Muscle.h>
%include <OpenSim/Actuators/RigidTendonMuscle.h>
%include <OpenSim/Actuators/ActiveForceLengthCurve.h>
%include <OpenSim/Actuators/FiberCompressiveForceCosPennationCurve.h>
%include <OpenSim/Actuators/FiberCompressiveForceLengthCurve.h>
%include <OpenSim/Actuators/FiberForceLengthCurve.h>
%include <OpenSim/Actuators/ForceVelocityCurve.h>
%include <OpenSim/Actuators/ForceVelocityInverseCurve.h>
%include <OpenSim/Actuators/TendonForceLengthCurve.h>
%include <OpenSim/Actuators/ClutchedPathSpring.h>

%include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
%include <OpenSim/Actuators/Millard2012AccelerationMuscle.h>

//osimTools
%include <OpenSim/Tools/osimToolsDLL.h>
%include <OpenSim/Tools/IKTask.h>
%template(SetIKTasks) OpenSim::Set<OpenSim::IKTask>;
%template(SetMarkerWeights) OpenSim::Set<MarkerWeight>;
%include <OpenSim/Tools/IKMarkerTask.h>
%include <OpenSim/Tools/IKCoordinateTask.h>
%include <OpenSim/Tools/IKTaskSet.h>
%include <OpenSim/Tools/MarkerPair.h>
%template(SetMarkerPairs) OpenSim::Set<OpenSim::MarkerPair>;
%include <OpenSim/Tools/MarkerPairSet.h>
%include <OpenSim/Tools/Measurement.h>
%template(SetMeasurements) OpenSim::Set<OpenSim::Measurement>;
%include <OpenSim/Tools/MeasurementSet.h>
%include <OpenSim/Tools/GenericModelMaker.h>
%include <OpenSim/Tools/ModelScaler.h>
%include <OpenSim/Tools/MarkerPlacer.h>
%include <OpenSim/Tools/ScaleTool.h>
%include <OpenSim/Simulation/Solver.h>
%include <OpenSim/Simulation/AssemblySolver.h>
%include <OpenSim/Simulation/Reference.h>

%template(ReferenceVec3) OpenSim::Reference_<SimTK::Vec3>;
%template(ReferenceDouble) OpenSim::Reference_<double>;
%template(ArrayCoordinateReference) SimTK::Array_<OpenSim::CoordinateReference>;
%template(SimTKArrayDouble) SimTK::Array_<double>;
%template(SimTKArrayVec3) SimTK::Array_<SimTK::Vec3>;

%include <OpenSim/Simulation/MarkersReference.h>
%include <OpenSim/Simulation/CoordinateReference.h>
%include <OpenSim/Simulation/InverseKinematicsSolver.h>
%include <OpenSim/Tools/Tool.h>
%include <OpenSim/Tools/DynamicsTool.h>
%include <OpenSim/Tools/InverseDynamicsTool.h>
%include <OpenSim/Tools/ForwardTool.h>
%include <OpenSim/Tools/CMCTool.h>
%include <OpenSim/Tools/RRATool.h>
%include <OpenSim/Tools/AnalyzeTool.h>
%include <OpenSim/Tools/InverseKinematicsTool.h>

%include <OpenSim/Wrapping/Python/OpenSimContext.h>

// Memory management
// =================

/*
A macro to facilitate adding adoptAndAppend methods to these sets. For NAME ==
Geometry, the macro expands to:

%extend OpenSim::GeometrySet {
%pythoncode %{
    def adoptAndAppend(self, aGeometry):
        aGeometry._markAdopted()
        return super(GeometrySet, self).adoptAndAppend(aGeometry)
%}
};

note: ## is a "glue" operator: `a ## b` --> `ab`.
*/
%define SET_ADOPT_HELPER(NAME)
%extend OpenSim:: ## NAME ## Set {
%pythoncode %{
    def adoptAndAppend(self, a ## NAME):
        a ## NAME._markAdopted()
        return super(NAME ## Set, self).adoptAndAppend(a ## NAME)
%}
};
%enddef

SET_ADOPT_HELPER(Geometry);
SET_ADOPT_HELPER(Scale);
SET_ADOPT_HELPER(Force);
SET_ADOPT_HELPER(Controller);
SET_ADOPT_HELPER(ContactGeometry);
SET_ADOPT_HELPER(Analysis);
SET_ADOPT_HELPER(Control);
SET_ADOPT_HELPER(Marker);
SET_ADOPT_HELPER(Body);
SET_ADOPT_HELPER(BodyScale);
SET_ADOPT_HELPER(Coordinate);
SET_ADOPT_HELPER(Joint);
SET_ADOPT_HELPER(Constraint);
SET_ADOPT_HELPER(PathPoint);
SET_ADOPT_HELPER(IKTask);
SET_ADOPT_HELPER(MarkerPair);
SET_ADOPT_HELPER(Measurement);

// These didn't work with the macro for some reason. I got complaints about
// multiple definitions of, e.g.,  Function in the target language.
%extend OpenSim::FunctionSet {
%pythoncode %{
    def adoptAndAppend(self, aFunction):
        aFunction._markAdopted()
        return super(FunctionSet, self).adoptAndAppend(aFunction)
%}
};

%extend OpenSim::ProbeSet {
%pythoncode %{
    def adoptAndAppend(self, aProbe):
        aProbe._markAdopted()
        return super(ProbeSet, self).adoptAndAppend(aProbe)
%}
};
