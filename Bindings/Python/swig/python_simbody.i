%module(package="opensim", directors="1") simbody
#pragma SWIG nowarn=822,451,503,516,325
// 401 is "Nothing known about base class *some-class*.
//         Maybe you forgot to instantiate *some-template* using %template."
// When wrapping new classes it's good to uncomment the line below to make sure
// you've wrapped your new class properly. However, SWIG generates lots of 401
// errors that are very difficult to resolve.
#pragma SWIG nowarn=401

%{
#define SWIG_FILE_WITH_INIT
#include <Simbody.h>
%}

%{
using namespace SimTK;
%}

// Add support for converting between NumPy and C arrays.
%include "numpy.i"
%init %{
    import_array();
%}

%include "python_preliminaries.i"


// Relay exceptions to the target language.
// This causes substantial code bloat and possibly hurts performance.
// Without these try-catch block, a SimTK or OpenSim exception causes the
// program to crash.
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

// Typemaps
// ========
// Python users may feel more comfortable providing/getting SimTK numeric types
// as NumPy types. We can use SWIG NumPy typemaps to accomplish
// this.
// https://docs.scipy.org/doc/numpy/reference/swig.interface-file.html
// https://scipy-cookbook.readthedocs.io/items/SWIG_NumPy_examples.html
// For constructor.
// The Python version of any OpenSim function taking the pair of arguments
// (int n, double* data) will instead take a single NumPy array.
%apply (int DIM1, double* IN_ARRAY1) {
    (int n, double* numpydata)
};
// The Python version of any Moco function taking sets of arguments
// (int nrow, int ncol, double* data) will instead take a single
// NumPy 2D array.
%apply (int DIM1, int DIM2, double* IN_ARRAY2) {
    (int nrow, int ncol, double* numpydata)
};
// The typemaps for functions that return matrices is more complicated: we must
// pass in a NumPy array that already has the correct size. We create a (hidden)
// C++ function that takes, for example, (int n, double* out)
// (e.g., _getAsMat()).
// Then we create a Python function that invokes the hidden C++ function with
// a correctly-sized NumPy array (e.g., getAsMat()).
%apply (int DIM1, double* ARGOUT_ARRAY1) {
    (int n, double* numpyout)
};
// Similar to above but for 2D arrays.
%apply (int DIM1, int DIM2, double* INPLACE_FARRAY2) {
    (int nrow, int ncol, double* numpyout)
};

// An alternative is to use typemaps to allow OpenSim functions to accept and
// return Python/NumPy types. For example, passing a list of 3 floats into a
// function that takes a Vec3 as an argument.
// The typemaps below allow this, but we opted not to go this route, because
// it remove the ability to pass arguments using the C++ types.
// For example, with these typemaps, Model::setGravity() can no longer take a
// Vec3.
/*
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

// Pythonic operators
// ==================
// Extend the template Vec class; these methods will apply for all template
// parameters. This extend block must appear before the %template call in
// simbody.i.
%extend SimTK::Vec {
    std::string __str__() const {
        return $self->toString();
    }
    int __len__() const {
        return $self->size();
    }
};

%extend SimTK::Vector_ {
    std::string __str__() const {
        return $self->toString();
    }
    int __len__() const {
        return $self->size();
    }
};

%include <Bindings/preliminaries.i>
%include <Bindings/simbody.i>

// Pythonic operators
// ==================
%extend SimTK::Vec<3> {
    SimTK::Vec(int n, double* numpydata) {
        SimTK_ERRCHK_ALWAYS(n == 3, "Vec3()", "Size of input must be 3.");
        return new SimTK::Vec<3>(numpydata);
    }
    static SimTK::Vec<3> createFromMat(int n, double* numpydata) {
        SimTK_ERRCHK_ALWAYS(n == 3, "createFromMat()",
                    "Size of input must be 3.");
        return SimTK::Vec3(numpydata);
    }
    double __getitem__(int i) const {
        SimTK_INDEXCHECK_ALWAYS(i, $self->size(), "Vec3.__getitem__()");
        return $self->operator[](i);
    }
    void __setitem__(int i, double value) {
        SimTK_INDEXCHECK_ALWAYS(i, $self->size(), "Vec3.__setitem__()");
        $self->operator[](i) = value;
    }
    //SimTK::Vec<3> __add__(const SimTK::Vec<3>& v) const {
    //    return *($self) + v;
    //}
    void _getAsMat(int n, double* numpyout) const {
        SimTK_ASSERT_ALWAYS(n == 3, "Size of input must be 3.");
        for (int i = 0; i < n; ++i) {
            numpyout[i] = $self->operator[](i);
        }
    }
%pythoncode %{
    def getAsMat(self):
        return self._getAsMat(self.size())
%};
};

%extend SimTK::Vector_<double> {
    // SimTK::Vector_(int n, double* numpydata) {
    //     return new SimTK::Vector_<double>(n, data, true);
    // }
    static SimTK::Vector_<double> createFromMat(int n, double* numpydata) {
        return SimTK::Vector_<double>(n, numpydata, true);
    }
    double __getitem__(int i) const {
        SimTK_INDEXCHECK_ALWAYS(i, $self->size(), "Vector.__getitem__()");
        return $self->operator[](i);
    }
    void __setitem__(int i, double value) {
        SimTK_INDEXCHECK_ALWAYS(i, $self->size(), "Vector.__setitem__()");
        $self->operator[](i) = value;
    }
    void _getAsMat(int n, double* numpyout) const {
        SimTK_ASSERT1_ALWAYS(n == $self->size(), "Size of input must be %i.",
                $self->size());
        std::copy_n($self->getContiguousScalarData(), n, numpyout);
    }
%pythoncode %{
    def getAsMat(self):
        return self._getAsMat(self.size())
%};
};

