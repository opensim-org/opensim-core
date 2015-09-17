%module(directors="1") opensim
%module opensim
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
#include <Bindings/OpenSimHeaders.h>
#include <Bindings/Python/OpenSimContext.h>
%}
%{
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

%newobject *::clone;

%rename(printToXML) OpenSim::Object::print(const std::string&) const;
%rename(printToXML) OpenSim::XMLDocument::print(const std::string&);
%rename(printToXML) OpenSim::XMLDocument::print();
%rename(printToFile) OpenSim::Storage::print;

%rename(appendNative) OpenSim::ForceSet::append(Force* aForce);

/* If needed %extend will be used, these operators are not supported.*/
%ignore *::operator[];
%ignore *::operator=;

/* This file is for creation/handling of arrays */
%include "std_carray.i";

/* This interface file is for better handling of pointers and references */
%include "typemaps.i"
%include "std_string.i"

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


// Make sure clone does not leak memory
%newobject *::clone; 

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


%include <Bindings/opensim.i>
%include <Bindings/Python/OpenSimContext.h>

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
