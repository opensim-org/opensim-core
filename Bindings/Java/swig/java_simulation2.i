%module(directors="1") opensimModelSimulation2
%module opensimModelSimulation2

#pragma SWIG nowarn=822,451,503,516,325,401

%{
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
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
          System.loadLibrary("osimJavaJNI");
      }
      catch(UnsatisfiedLinkError e){
          new JOptionPane("Required library failed to load. Check that the " +
                          "dynamic library osimJavaJNI is in your PATH\n" + e, 
        JOptionPane.ERROR_MESSAGE).createDialog(null, "Error").setVisible(true);
      }
  }
%}


%import "java_simulation1.i"

%include <Bindings/simulation2.i>
