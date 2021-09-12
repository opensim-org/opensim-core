%module(package="opensim", directors="1") jam
#pragma SWIG nowarn=822,451,503,516,325
#pragma SWIG nowarn=401

%{
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
#include <Bindings/OpenSimHeaders_actuators.h>
#include <Bindings/OpenSimHeaders_analyses.h>
#include <Bindings/OpenSimHeaders_tools.h>
#include <Bindings/OpenSimHeaders_moco.h>
#include <Bindings/OpenSimHeaders_jam.h>

using namespace OpenSim;
using namespace SimTK;
%}

//%include <Bindings/preliminaries.i>
%include "python_preliminaries.i";

%import <python_common.i>
%import <python_simulation.i>
%import <python_actuators.i>


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



%include <Bindings/jam.i>
