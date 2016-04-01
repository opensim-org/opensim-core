%module(directors="1") opensimModel
%module opensimModel

#pragma SWIG nowarn=822,451,503,516,325,401

%{
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
#include <Bindings/OpenSimHeaders_actuators.h>
#include <Bindings/OpenSimHeaders_analyses.h>
#include <Bindings/OpenSimHeaders_tools.h>
#include <OpenSim/Utilities/simmFileWriterDLL/SimmFileWriter.h>

#include <Bindings/Java/OpenSimJNI/Hooks/SimtkLogCallback.h>
#include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

using namespace OpenSim;
using namespace SimTK;
%}

%include "arrays_java.i";

/* Load the required libraries when this module is loaded.                    */
%pragma(java) jniclassclassmodifiers="public class"
SWIG_JAVABODY_PROXY(public, public, SWIGTYPE)
%pragma(java) jniclassimports="import javax.swing.JOptionPane;"
%pragma(java) jniclasscode=%{
  static {
      try{
          // All OpenSim classes required for GUI operation.
          System.loadLibrary("osimJavaJNISimbody");
          System.loadLibrary("osimJavaJNICommon");
          System.loadLibrary("osimJavaJNISimulation1");
          System.loadLibrary("osimJavaJNISimulation2");
          System.loadLibrary("osimJavaJNI");
      }
      catch(UnsatisfiedLinkError e){
          new JOptionPane("Required library failed to load. Check that the " +
                          "dynamic library osimJavaJNI is in your PATH\n" + e, 
        JOptionPane.ERROR_MESSAGE).createDialog(null, "Error").setVisible(true);
      }
  }
%}


%import "java_simulation2.i"

%include <Bindings/actuators.i>
%include <Bindings/analyses.i>
%include <Bindings/tools.i>
%include <OpenSim/Utilities/simmFileWriterDLL/SimmFileWriter.h>

%include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

%include <Bindings/Java/OpenSimJNI/Hooks/SimtkLogCallback.h>
