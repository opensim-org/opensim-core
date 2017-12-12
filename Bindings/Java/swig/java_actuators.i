%module(directors="1") opensimActuatorsAnalysesTools
%module opensimActuatorsAnalysesTools

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

%feature("director") OpenSim::AnalysisWrapper;
%feature("director") OpenSim::SimtkLogCallback;


%include "arrays_java.i";

/* Load the required libraries when this module is loaded.                    */
%pragma(java) jniclassclassmodifiers="public class"
SWIG_JAVABODY_PROXY(public, public, SWIGTYPE)
%pragma(java) jniclassimports="import javax.swing.JOptionPane;import java.awt.GraphicsEnvironment;"
%pragma(java) jniclasscode=%{
  static {
      try{
          // All OpenSim classes required for GUI operation.
          System.loadLibrary("osimJavaJNI");
      }
      catch(UnsatisfiedLinkError e){
          String envVar = new String();
          String OS = System.getProperty("os.name").toLowerCase();
          if (OS.indexOf("win") >= 0) {
              envVar = "PATH";
          } else if (OS.indexOf("mac") >= 0) {
              envVar = "DYLD_LIBRARY_PATH";
          } else {
              envVar = "LD_LIBRARY_PATH";
          }
          String msg = new String("Required library failed to load. " +
                  "Check that the dynamic library osimJavaJNI " +
                  "is on your " + envVar + ".\n" + e);
          System.out.println(msg);
          if (!GraphicsEnvironment.isHeadless()) {
              new JOptionPane(msg, JOptionPane.ERROR_MESSAGE).createDialog(null, "Error").setVisible(true);
          }
      }
  }
%}


%import "java_simulation.i"

%include <Bindings/actuators.i>
%include <Bindings/analyses.i>
        
// When used from GUI or matlab ModelScaler takes ownership on these calls, 
// communicate that fact to the interpreter to avoid Garbage Collection issues
%javamethodmodifiers OpenSim::ModelScaler::addMeasurement "private";
%javamethodmodifiers OpenSim::ModelScaler::addScale "private";

%rename OpenSim::ModelScaler::addMeasurement private_addMeasurement;
%rename OpenSim::ModelScaler::addScale private_addScale;

%typemap(javacode) OpenSim::ModelScaler %{
    public void addScale(Scale scale){
        scale.markAdopted();
        private_addScale(scale);
    }

    public void addMeasurement(Measurement meas){
        meas.markAdopted();
        private_addMeasurement(meas);
    }
%}

%include <Bindings/tools.i>
%include <OpenSim/Utilities/simmFileWriterDLL/SimmFileWriter.h>

%include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

%include <Bindings/Java/OpenSimJNI/Hooks/SimtkLogCallback.h>

