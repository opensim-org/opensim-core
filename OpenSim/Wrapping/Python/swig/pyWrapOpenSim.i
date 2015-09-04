%module(directors="1") opensim
#pragma SWIG nowarn=822,451,503,516,325
// 401 is "Nothing known about base class *some-class*.
//         Maybe you forgot to instantiate *some-template* using %template."
// When wrapping new classes it's good to uncomment the line below to make sure
// you've wrapped your new class properly. However, SWIG generates lots of 401
// errors that are very difficult to resolve.
#pragma SWIG nowarn=401

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
#include <OpenSim/Common/Set.h>
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
#include <OpenSim/Common/Component.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>
#include <OpenSim/Simulation/Model/ComponentSet.h>

#include <OpenSim/Simulation/Solver.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Simulation/MomentArmSolver.h>

#include <OpenSim/Simulation/Model/Frame.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>
#include <OpenSim/Simulation/Model/Ground.h>
#include <OpenSim/Simulation/Model/OffsetFrame.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>

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
#include <OpenSim/Simulation/Model/PointForceDirection.h>
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
#include <OpenSim/Simulation/Model/ExpressionBasedCoordinateForce.h>
#include <OpenSim/Simulation/Model/PointToPointSpring.h>
#include <OpenSim/Simulation/Model/PathSpring.h>
#include <OpenSim/Simulation/Model/BushingForce.h>
#include <OpenSim/Simulation/Model/FunctionBasedBushingForce.h>
#include <OpenSim/Simulation/Model/ExpressionBasedBushingForce.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Actuators/PointActuator.h>
#include <OpenSim/Actuators/TorqueActuator.h>
#include <OpenSim/Actuators/BodyActuator.h>
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

#include <OpenSim/Tools/TrackingTask.h>
#include <OpenSim/Tools/CMC_Task.h>
#include <OpenSim/Tools/CMC_Joint.h>
#include <OpenSim/Tools/CMC_Point.h>
#include <OpenSim/Tools/CMC_TaskSet.h>

#include <OpenSim/Tools/CMCTool.h>
#include <OpenSim/Tools/RRATool.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>

using namespace OpenSim;
using namespace SimTK;

%}

%feature("director") OpenSim::SimtkLogCallback;
%feature("director") SimTK::DecorativeGeometryImplementation;
%feature("notabstract") ControlLinear;

/** Pass Doxygen documentation to python wrapper */
%feature("autodoc", "3");

%rename(OpenSimObject) OpenSim::Object;
%rename(OpenSimException) OpenSim::Exception;

// Relay exceptions to the target language.
// This causes substantial code bloat and possibly hurts performance.
// Without these try-catch block, a SimTK or OpenSim exception causes the
// program to crash.
%include exception.i
%exception {
    try {
        $action
    } catch (const std::exception& e) {
        std::string str("std::exception in '$fulldecl': ");
        std::string what(e.what());
        SWIG_exception(SWIG_RuntimeError, (str + what).c_str());
    }
}
// The following exception handling is preferred but causes too much bloat.
//%exception {
//    try {
//        $action
//    } catch (const SimTK::Exception::IndexOutOfRange& e) {
//        SWIG_exception(SWIG_IndexError, e.what());
//    } catch (const SimTK::Exception::Base& e) {
//        std::string str("SimTK Simbody error in '$fulldecl': ");
//        std::string what(e.what());
//        SWIG_exception(SWIG_RuntimeError, (str + what).c_str());
//    } catch (const OpenSim::Exception& e) {
//        std::string str("OpenSim error in '$fulldecl': ");
//        std::string what(e.what());
//        SWIG_exception(SWIG_RuntimeError, (str + what).c_str());
//    } catch (const std::exception& e) {
//        std::string str("std::exception in '$fulldecl': ");
//        std::string what(e.what());
//        SWIG_exception(SWIG_RuntimeError, (str + what).c_str());
//    } catch (...) {
//        SWIG_exception(SWIG_RuntimeError, "Unknown exception in '$fulldecl'.");
//    }
//}

%newobject *::clone;

%rename(printToXML) OpenSim::Object::print(const std::string&) const;
%rename(printToXML) OpenSim::XMLDocument::print(const std::string&);
%rename(printToXML) OpenSim::XMLDocument::print();
%rename(printToFile) OpenSim::Storage::print;
%rename(NoType) OpenSim::Geometry::None;
%rename(NoPreference) OpenSim::DisplayGeometry::None;

%rename(appendNative) OpenSim::ForceSet::append(Force* aForce);

// Operators
// =========
/* If needed %extend will be used, these operators are not supported.*/
%ignore *::operator[];
%ignore *::operator=;

// For reference (doesn't work and should not be necessary):
// %rename(__add__) operator+;

/* This file is for creation/handling of arrays */
%include "std_carray.i";

/* This interface file is for better handling of pointers and references */
%include "typemaps.i"
%include "std_string.i"

// To access std::vector from Python.
// %include "std_vector.i"

// Typemaps
// ========
// Allow passing python objects into OpenSim functions. For example,
// pass a list of 3 float's into a function that takes a Vec3 as an argument.
/*
TODO disabling these for now as the typemaps remove the ability to pass
arguments using the C++ types. For example, with these typemaps,
Model::setGravity() can no longer take a Vec3. We can revisit typemaps if we
can keep the ability to use the original argument types.
These typemaps work, though.
%typemap(in) SimTK::Vec3 {
    SimTK::Vec3 v;
    if (!PySequence_Check($input)) {
        PyErr_SetString(PyExc_ValueError, "Expected a sequence.");
        return NULL;
    }
    if (PySequence_Length($input) != v.size()) {
        PyErr_SetString(PyExc_ValueError,
                "Size mismatch. Expected 3 elements.");
        return NULL;
    }
    for (int i = 0; i < v.size(); ++i) {
        PyObject* o = PySequence_GetItem($input, i);
        if (PyNumber_Check(o)) {
            v[i] = PyFloat_AsDouble(o);
        } else {
            PyErr_SetString(PyExc_ValueError,
                "Sequence elements must be numbers.");
            return NULL;
        }
    }
    $1 = &v;
};
%typemap(in) const SimTK::Vec3& {
    SimTK::Vec3 v;
    if (!PySequence_Check($input)) {
        PyErr_SetString(PyExc_ValueError, "Expected a sequence.");
        return NULL;
    }
    if (PySequence_Length($input) != v.size()) {
        PyErr_SetString(PyExc_ValueError,
                "Size mismatch. Expected 3 elements.");
        return NULL;
    }
    for (int i = 0; i < v.size(); ++i) {
        PyObject* o = PySequence_GetItem($input, i);
        if (PyNumber_Check(o)) {
            v[i] = PyFloat_AsDouble(o);
        } else {
            PyErr_SetString(PyExc_ValueError,
                "Sequence elements must be numbers.");
            return NULL;
        }
    }
    $1 = &v;
};
// This is how one would apply a generic typemap to specific arguments:
//%apply const SimTK::Vec3& INPUT { const SimTK::Vec3& aGrav };
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
    adoptee._markAdopted()
%}
%enddef

MODEL_ADOPT_HELPER(ModelComponent);
MODEL_ADOPT_HELPER(Body);
MODEL_ADOPT_HELPER(Probe);
MODEL_ADOPT_HELPER(Joint);
MODEL_ADOPT_HELPER(Frame);
MODEL_ADOPT_HELPER(Constraint);
MODEL_ADOPT_HELPER(ContactGeometry);
MODEL_ADOPT_HELPER(Analysis);
MODEL_ADOPT_HELPER(Force);
MODEL_ADOPT_HELPER(Controller);

// Make sure clone does not leak memory
%newobject *::clone; 

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

%include "std_vector.i"
%template(StdVecInt) std::vector<int>;

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

// Vec3
// Extend the template Vec class; these methods will apply for all template
// parameters. This extend block must appear before the %template call below.
%extend SimTK::Vec {
    std::string __str__() const {
        return $self->toString();
    }
    int __len__() const {
        return $self->size();
    }
};
namespace SimTK {
%template(Vec2) Vec<2>;
%template(Vec3) Vec<3>;
%template(Vec4) Vec<4>;
%template(Vec6) Vec<6>;
}
%extend SimTK::Vec<3> {
     double __getitem__(int i) const {
        SimTK_INDEXCHECK_ALWAYS(i, $self->size(), "Vec3.__getitem__()");
        return $self->operator[](i);
    }
    void __setitem__(int i, double value) {
        SimTK_INDEXCHECK_ALWAYS(i, $self->size(), "Vec3.__setitem__()");
        $self->operator[](i) = value;
    }
    SimTK::Vec<3> __add__(const SimTK::Vec<3>& v) const {
        return *($self) + v;
    }
};

// Mat33
%include <SWIGSimTK/Mat.h>
namespace SimTK {
%template(Mat33) Mat<3, 3>;
}
%include <SWIGSimTK/CoordinateAxis.h>
%include <SWIGSimTK/UnitVec.h>
namespace SimTK {
%template(UnitVec3)  SimTK::UnitVec<double,1>;
}

// Vector and Matrix
%include <SWIGSimTK/BigMatrix.h>
// Extend the template Vec class; these methods will apply for all template
// parameters. This extend block must appear before the %template call below.
%extend SimTK::Vector_ {
    std::string __str__() const {
        return $self->toString();
    }
    int __len__() const {
        return $self->size();
    }
};
namespace SimTK {
%template(MatrixBaseDouble) SimTK::MatrixBase<double>;
%template(VectorBaseDouble) SimTK::VectorBase<double>;
%template(Vector) SimTK::Vector_<double>;
%template(Matrix) SimTK::Matrix_<double>;
}
%extend SimTK::Vector_<double> {
    double __getitem__(int i) const {
        SimTK_INDEXCHECK_ALWAYS(i, $self->size(), "Vector.__getitem__()");
        return $self->operator[](i);
    }
    void __setitem__(int i, double value) {
        SimTK_INDEXCHECK_ALWAYS(i, $self->size(), "Vector.__setitem__()");
        $self->operator[](i) = value;
    }
};

%include <SWIGSimTK/SpatialAlgebra.h>
namespace SimTK {
%template(SpatialVec) Vec<2,   Vec3>;
%template(VectorBaseOfSpatialVec) VectorBase<SpatialVec>;
%template(VectorBaseOfVec3) VectorBase<Vec3>;
%template(VectorOfSpatialVec) Vector_<SpatialVec>;
%template(VectorOfVec3) Vector_<Vec3>;
}


%include <SWIGSimTK/Rotation.h>
namespace SimTK {
%template(Rotation) SimTK::Rotation_<double>;
%template(InverseRotation) SimTK::InverseRotation_<double>;
}
// Transform
%include <SWIGSimTK/Transform.h>
namespace SimTK {
%template(Transform) SimTK::Transform_<double>;
}

//
%include <SWIGSimTK/MassProperties.h>
namespace SimTK {
%template(Inertia) SimTK::Inertia_<double>;
%template(MassProperties) SimTK::MassProperties_<double>;
}
%include <SWIGSimTK/common.h>
%include <SWIGSimTK/Array.h>
namespace SimTK {
    %template(ArrayView) ArrayView_<double, unsigned int>;
    %template(ArrayViewOfVec3) ArrayView_<Vec3, unsigned int>;
}

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
%template(SimTKArrayString) SimTK::Array_<std::string>;
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
%template (PropertyString) OpenSim::Property<std::string>;
%include <OpenSim/Common/Object.h>
%include <OpenSim/Common/ObjectGroup.h>

%extend OpenSim::Set {
%pythoncode %{
    class SetIterator(object):
        """
        Use this object to iterate over a Set. You create an instance of
        this nested class by calling Set.__iter__().
        """
        def __init__(self, set_obj, index):
            """Construct an iterator for the Set `set`."""
            self._set_obj = set_obj
            self._index = index
        def __iter__(self):
            """This iterator is also iterable."""
            return self
        def next(self):
            if self._index < self._set_obj.getSize():
                current_index = self._index
                self._index += 1
                return self._set_obj.get(current_index)
            else:
                # This is how Python knows to stop iterating.
                 raise StopIteration()

    def __iter__(self):
        """Get an iterator for this Set, starting at index 0."""
        return self.SetIterator(self, 0)

    def items(self):
        """
        A generator function that allows you to iterate over the key-value
        pairs of this Set. You can use this in a for-loop as such::

            for key, val in my_function_set.items():
                # `val` is an item in the Set, and `key` is its name.
                print key, val
        """
        index = 0
        while index < self.getSize():
            yield self.get(index).getName(), self.get(index)
            index += 1
%}
};

%include <OpenSim/Common/Set.h>
%include <OpenSim/Common/StateVector.h>
%template(ArrayStateVector) OpenSim::Array<OpenSim::StateVector>;
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
namespace OpenSim {
    // Python does not support these operators.
    %ignore ComponentListIterator::operator++();
    %ignore ComponentListIterator::operator++(int);
};

%include <OpenSim/Common/ComponentList.h>
%template(ComponentsList) OpenSim::ComponentList<OpenSim::Component>;
%template(ComponentIterator) OpenSim::ComponentListIterator<OpenSim::Component>;

%template(FramesList) OpenSim::ComponentList<OpenSim::Frame>;
%template(FrameIterator) OpenSim::ComponentListIterator<OpenSim::Frame>;

%template(BodiesList) OpenSim::ComponentList<OpenSim::Body>;
%template(BodyIterator) OpenSim::ComponentListIterator<OpenSim::Body>;

%template(MusclesList) OpenSim::ComponentList<OpenSim::Muscle>;
%template(MuscleIterator) OpenSim::ComponentListIterator<OpenSim::Muscle>;

%template(ModelComponentList) OpenSim::ComponentList<OpenSim::ModelComponent>;
%template(ModelComponentIterator) OpenSim::ComponentListIterator<OpenSim::ModelComponent>;

%template(JointsList) OpenSim::ComponentList<OpenSim::Joint>;
%template(JointIterator) OpenSim::ComponentListIterator<OpenSim::Joint>;

%include <OpenSim/Common/Component.h>

%template(getFramesList) OpenSim::Component::getComponentList<OpenSim::Frame>;
%template(getBodiesList) OpenSim::Component::getComponentList<OpenSim::Body>;
%template(getMusclesList) OpenSim::Component::getComponentList<OpenSim::Muscle>;
%template(getComponentsList) OpenSim::Component::getComponentList<OpenSim::Component>;
%template(getModelComponentList) OpenSim::Component::getComponentList<OpenSim::ModelComponent>;
%template(getJointList) OpenSim::Component::getComponentList<OpenSim::Joint>;

%include <OpenSim/Common/Scale.h>
%template(SetScales) OpenSim::Set<OpenSim::Scale>;
%include <OpenSim/Common/ScaleSet.h>
%include <OpenSim/Common/MarkerFrame.h>
%include <OpenSim/Common/MarkerData.h>

// osimSimulation
%include <OpenSim/Simulation/osimSimulationDLL.h>
%include <OpenSim/Simulation/Model/Appearance.h>
%include <OpenSim/Simulation/Model/Geometry.h>
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
// Followig thee lines hacked in out of order to work around WrapObjects use in PhysicalFrame
%include <OpenSim/Simulation/Wrap/WrapObject.h>
%template(SetWrapObject) OpenSim::Set<OpenSim::WrapObject>;
%include <OpenSim/Simulation/Wrap/WrapObjectSet.h>

%include <OpenSim/Simulation/Model/PhysicalFrame.h>
%include <OpenSim/Simulation/Model/Ground.h>
%include <OpenSim/Simulation/Model/OffsetFrame.h>
%template(PhysicalFrameWithOffset)   OpenSim::OffsetFrame<OpenSim::PhysicalFrame>; 
%include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
%template(SetFrames) OpenSim::Set<OpenSim::Frame>;
%template(ModelComponentSetFrames) OpenSim::ModelComponentSet<OpenSim::Frame>;
%include <OpenSim/Simulation/Model/FrameSet.h>

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
%include <OpenSim/Simulation/Model/Station.h>
%include <OpenSim/Simulation/Model/Marker.h>
%template(SetMarkers) OpenSim::Set<OpenSim::Marker>;
%template(ModelComponentSetMarkers) OpenSim::ModelComponentSet<OpenSim::Marker>;
%include <OpenSim/Simulation/Model/MarkerSet.h>

//%include <OpenSim/Simulation/Wrap/WrapObject.h>
%include <OpenSim/Simulation/Wrap/WrapSphere.h>
%include <OpenSim/Simulation/Wrap/WrapCylinder.h>
%include <OpenSim/Simulation/Wrap/WrapTorus.h>
%include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
//%template(SetWrapObject) OpenSim::Set<OpenSim::WrapObject>;
//%include <OpenSim/Simulation/Wrap/WrapObjectSet.h>
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
namespace OpenSim {
    // This method overshadows setFunction(const Function&).
    %ignore TransformAxis::setFunction(Function*);
};
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
namespace OpenSim {
    // This method overshadows setFunction(const Function&).
    %ignore CoordinateCouplerConstraint::setFunction(Function*);
};
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
%template(SetBhargava2004MuscleMetabolicsProbeMetabolicMuscleParameter)
    OpenSim::Set<OpenSim::Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter>;
%include <OpenSim/Simulation/Model/Bhargava2004MuscleMetabolicsProbe.h>
%template(SetUmberger2010MuscleMetabolicsProbeMetabolicMuscleParameter)
    OpenSim::Set<OpenSim::Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter>;
%include <OpenSim/Simulation/Model/Umberger2010MuscleMetabolicsProbe.h>
%include <OpenSim/Simulation/Model/ModelDisplayHints.h>
%include <OpenSim/Simulation/Model/ModelVisualPreferences.h>
%include <OpenSim/Simulation/Model/ModelVisualizer.h>
%include <OpenSim/Simulation/Model/Model.h>

%include <OpenSim/Simulation/Model/PathPoint.h>
%include <OpenSim/Simulation/Wrap/PathWrapPoint.h>
%include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
%include <OpenSim/Simulation/Model/MovingPathPoint.h>

%template(SetPathPoint) OpenSim::Set<OpenSim::PathPoint>;
%template(ArrayPathPoint) OpenSim::Array<OpenSim::PathPoint*>;
%include <OpenSim/Simulation/Model/PathPointSet.h>

%include <OpenSim/Simulation/Model/PointForceDirection.h>
%template(ArrayPointForceDirection) OpenSim::Array<OpenSim::PointForceDirection*>;

%include <OpenSim/Simulation/Model/GeometryPath.h>
%include <OpenSim/Simulation/Model/Ligament.h>
%include <OpenSim/Simulation/Model/PathActuator.h>
%include <OpenSim/Simulation/Model/Muscle.h>
%include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>
%include <OpenSim/Simulation/Model/PointToPointSpring.h>
%include <OpenSim/Simulation/Model/ExpressionBasedPointToPointForce.h>
%include <OpenSim/Simulation/Model/ExpressionBasedCoordinateForce.h>

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
%include <OpenSim/Actuators/BodyActuator.h>
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
%include <OpenSim/Actuators/MuscleFirstOrderActivationDynamicModel.h>
%include <OpenSim/Actuators/MuscleFixedWidthPennationModel.h>

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
%template(ArrayViewConstCoordinateReference)
    SimTK::ArrayViewConst_<OpenSim::CoordinateReference>;
%ignore ArrayViewConstCoordinateReference;
%template(ArrayViewCoordinateReference)
    SimTK::ArrayView_<OpenSim::CoordinateReference>;
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

%include <OpenSim/Tools/TrackingTask.h>
%include <OpenSim/Tools/CMC_Task.h>
%include <OpenSim/Tools/CMC_Joint.h>
%include <OpenSim/Tools/CMC_Point.h>
%template (SetTrackingTasks) OpenSim::Set<OpenSim::TrackingTask>;
%include <OpenSim/Tools/CMC_TaskSet.h>

%include <OpenSim/Tools/CMCTool.h>
%include <OpenSim/Tools/RRATool.h>
%include <OpenSim/Tools/AnalyzeTool.h>
%include <OpenSim/Tools/InverseKinematicsTool.h>

// Memory management
// =================

/*
A macro to facilitate adding adoptAndAppend methods to these sets. For NAME ==
Geometry, the macro expands to:

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

SET_ADOPT_HELPER(Scale);
SET_ADOPT_HELPER(BodyScale);
SET_ADOPT_HELPER(PathPoint);
SET_ADOPT_HELPER(IKTask);
SET_ADOPT_HELPER(MarkerPair);
SET_ADOPT_HELPER(Measurement);
SET_ADOPT_HELPER(Marker);
SET_ADOPT_HELPER(Control);
SET_ADOPT_HELPER(Frame);
SET_ADOPT_HELPER(Force);

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

// Attempt to solve segfault when calling ForceSet::append()
// from scripting.
%extend OpenSim::ForceSet {
%pythoncode %{
    def append(self, aForce):
        aForce._markAdopted()
        return self.appendNative(aForce)
%}
};
