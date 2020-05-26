%module(package="opensim", directors="1") common
#pragma SWIG nowarn=822,451,503,516,325
// 401 is "Nothing known about base class *some-class*.
//         Maybe you forgot to instantiate *some-template* using %template."
// When wrapping new classes it's good to uncomment the line below to make sure
// you've wrapped your new class properly. However, SWIG generates lots of 401
// errors that are very difficult to resolve.
#pragma SWIG nowarn=401


%{
#define SWIG_FILE_WITH_INIT
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/PropertyHelper.h>
%}

%{
using namespace OpenSim;
using namespace SimTK;
%}

%include "python_preliminaries.i"

// Tell SWIG about the simbody module.
%import "python_simbody.i"

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
once added to another Component. This snippet is based on the
MODEL_ADOPT_HELPER macro in python_simulation.i.

note: ## is a "glue" operator: `a ## b` --> `ab`.
*/
%pythonappend OpenSim::Component::addComponent %{
    subcomponent._markAdopted()
%}


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
// common.i.

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
        __next__ = next # For Python 3.

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

// This function tries to downcast the component; if that does not succeed,
// then it just returns a Component (or whatever the type for this specific
// ComponentList is).
%extend OpenSim::ComponentList {
%pythoncode %{

    def __iter__(self):
        """Get an iterator for this ComponentList, to be used as such::
           
            for c in model.getComponentsList():
                c.getName()
        """
        import sys
        opensim_pkg = sys.modules[__name__.partition('.')[0]]
        it = self.begin()
        while it != self.end():
            component = it.__deref__()
            try:
                ConcreteClass = getattr(opensim_pkg, component.getConcreteClassName())
                concrete_component = ConcreteClass.safeDownCast(component)
                yield concrete_component 
            except:
                yield component
            it.next()
%}
};

// Return concrete instances from certain methods.
// -----------------------------------------------
// http://stackoverflow.com/questions/27392602/swig-downcasting-from-base-to-derived
// "$1" holds the original return value from getConnectee() (an Object*).
// An Object with name Foo is registered with SWIG as "OpenSim::Foo *".
// If the type query fails, we just return an Object*
%typemap(out) const OpenSim::Object& OpenSim::Component::getConnectee {
    swig_type_info * const outtype = SWIG_TypeQuery(
            ("OpenSim::" + ($1)->getConcreteClassName() + " *").c_str());
    if (outtype) {
        $result = SWIG_NewPointerObj(SWIG_as_voidptr($1), outtype, $owner);
    } else {
        swig_type_info * const outtype = SWIG_TypeQuery("OpenSim::Object *");
        $result = SWIG_NewPointerObj(SWIG_as_voidptr($1), outtype, $owner);
    }
}
%typemap(out) const OpenSim::Component& OpenSim::Component::getComponent {
    swig_type_info * const outtype = SWIG_TypeQuery(
            ("OpenSim::" + ($1)->getConcreteClassName() + " *").c_str());
    if (outtype) {
        $result = SWIG_NewPointerObj(SWIG_as_voidptr($1), outtype, $owner);
    } else {
        swig_type_info * const outtype = SWIG_TypeQuery("OpenSim::Component *");
        $result = SWIG_NewPointerObj(SWIG_as_voidptr($1), outtype, $owner);
    }
}
%typemap(out) OpenSim::Component& OpenSim::Component::updComponent {
    swig_type_info * const outtype = SWIG_TypeQuery(
            ("OpenSim::" + ($1)->getConcreteClassName() + " *").c_str());
    if (outtype) {
        $result = SWIG_NewPointerObj(SWIG_as_voidptr($1), outtype, $owner);
    } else {
        swig_type_info * const outtype = SWIG_TypeQuery("OpenSim::Component *");
        $result = SWIG_NewPointerObj(SWIG_as_voidptr($1), outtype, $owner);
    }
}

%extend OpenSim::DataTable_ {
    std::string __str__() const {
        return $self->toString();
    }
}

// Include all the OpenSim code.
// =============================
%include <Bindings/preliminaries.i>
%include <Bindings/common.i>


// Memory management
// =================
SET_ADOPT_HELPER(Scale);

// This didn't work with the macro for some reason. I got complaints about
// multiple definitions of, e.g.,  Function in the target language.
%extend OpenSim::FunctionSet {
%pythoncode %{
    def adoptAndAppend(self, aFunction):
        aFunction._markAdopted()
        return super(FunctionSet, self).adoptAndAppend(aFunction)
%}
};

