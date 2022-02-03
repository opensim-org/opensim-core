
/*
For consistency with the rest of the API, we use camel-case for variable names.
This breaks Python PEP 8 convention, but allows us to be consistent within our
own project.
*/

/** The following line produces legacy documentation in python wrapper, replaced with -doxygen in swig 4.0+ */
//%feature("autodoc", "3");

// Make sure clone does not leak memory
%newobject *::clone;

/* This file is for creation/handling of arrays */
%include "std_carray.i";

/* This interface file is for better handling of pointers and references */
%include "typemaps.i"
%include "std_string.i"

/* If needed %extend will be used, these operators are not supported.*/
%ignore *::operator[];
%ignore *::operator=;

// For reference (doesn't work and should not be necessary):
// %rename(__add__) operator+;

// Rename
// ======
%rename(NoType) OpenSim::Geometry::None;
%rename(NoPreference) OpenSim::DisplayGeometry::None;

// Memory management
// =================
/*
A macro to facilitate adding adoptAndAppend methods to these sets.

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

// https://github.com/swig/swig/blob/master/Lib/python/std_auto_ptr.i
%define opensim_unique_ptr(TYPE)
%template() std::unique_ptr<TYPE>;
%newobject std::unique_ptr<TYPE>::release;
%typemap(out) std::unique_ptr<TYPE> %{
%set_output(SWIG_NewPointerObj($1.release(), $descriptor(TYPE *), SWIG_POINTER_OWN | %newpointer_flags));
%}
%enddef
