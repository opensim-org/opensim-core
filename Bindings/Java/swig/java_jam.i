%module(directors="1") opensimMoco

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

%include <Bindings/preliminaries.i>
%include "java_preliminaries.i";

%import "java_actuators.i"

%include <Bindings/jam.i>


