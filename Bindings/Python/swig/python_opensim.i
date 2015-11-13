%module(directors="1") opensim
#pragma SWIG nowarn=822,451,503,516,325
// 401 is "Nothing known about base class *some-class*.
//         Maybe you forgot to instantiate *some-template* using %template."
// When wrapping new classes it's good to uncomment the line below to make sure
// you've wrapped your new class properly. However, SWIG generates lots of 401
// errors that are very difficult to resolve.
#pragma SWIG nowarn=401


%{
#define SWIG_FILE_WITH_INIT
#include <Bindings/OpenSimHeaders.h>
%}
%{
using namespace OpenSim;
using namespace SimTK;

%}

%include "python_preliminaries.i"

// Tell SWIG about the simbody classes.
%import "python_simbody.i"

%feature("director") OpenSim::SimtkLogCallback;
%feature("director") SimTK::DecorativeGeometryImplementation;
%feature("notabstract") ControlLinear;

%rename(OpenSimObject) OpenSim::Object; // TODO remove
%rename(OpenSimException) OpenSim::Exception;

// Relay exceptions to the target language.
// This causes substantial code bloat and possibly hurts performance.
// Without these try-catch block, a SimTK or OpenSim exception causes the
// program to crash.
// We use SWIG_exception_fail instead of SWIG_exception due to some fluke where
// SWIG_exception isn't defined for this module (because we're importing
// another module?).
%include "exception.i"
// Delete any previous exception handlers.
%exception;
%exception {
    try {
        $action
    } catch (const std::exception& e) {
        std::string str("std::exception in '$fulldecl': ");
        std::string what(e.what());
        SWIG_exception_fail(SWIG_RuntimeError, (str + what).c_str());
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

%rename(printToXML) OpenSim::Object::print(const std::string&) const;
%rename(printToXML) OpenSim::XMLDocument::print(const std::string&);
%rename(printToXML) OpenSim::XMLDocument::print();
%rename(printToFile) OpenSim::Storage::print;
%rename(NoType) OpenSim::Geometry::None;
%rename(NoPreference) OpenSim::DisplayGeometry::None;

%rename(appendNative) OpenSim::ForceSet::append(Force* aForce);

// For reference (doesn't work and should not be necessary):
// %rename(__add__) operator+;


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

/*
Extend concrete Joints to use the inherited base constructors.
This is only necessary because SWIG does not generate these inherited
constructors provided by C++11's 'using' (e.g. using Joint::Joint) declaration.
Note that CustomJoint and EllipsoidJoint do implement their own
constructors because they have additional arguments.
*/
%define EXPOSE_JOINT_CONSTRUCTORS_HELPER(NAME)
%extend OpenSim::NAME {
	NAME(const std::string& name,
         const std::string& parentName,
         const std::string& childName) {
		return new NAME(name, parentName, childName, false);
	}
	
	NAME(const std::string& name,
         const PhysicalFrame& parent,
         const SimTK::Vec3& locationInParent,
         const SimTK::Vec3& orientationInParent,
         const PhysicalFrame& child,
         const SimTK::Vec3& locationInChild,
         const SimTK::Vec3& orientationInChild) {
		return new NAME(name, parent, locationInParent, orientationInParent,
					child, locationInChild, orientationInChild, false);
	}
};
%enddef

EXPOSE_JOINT_CONSTRUCTORS_HELPER(FreeJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(BallJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(PinJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(SliderJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(WeldJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(GimbalJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(UniversalJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(PlanarJoint);


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

// Load OpenSim plugins TODO
// ====================
//%include <OpenSim/Common/LoadOpenSimLibrary.h>

// %include "numpy.i"

// Pythonic operators
// ==================
// Extend the template Vec class; these methods will apply for all template
// parameters. This extend block must appear before the %template call in
// opensim.i.

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

%include <Bindings/preliminaries.i>
%include <Bindings/opensim.i>

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
