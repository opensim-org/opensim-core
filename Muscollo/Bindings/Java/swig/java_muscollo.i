%module(directors="1") opensimMuscollo

%include java_exception.i

%{
#include <Bindings/OpenSimHeaders_muscollo.h>
using namespace OpenSim;
using namespace SimTK;
%}

/* TODO TODO TODO loadLibrary("osimMuscolloJavaJNI"); */

%import "java_actuators.i"

%include <Bindings/muscollo.i>

