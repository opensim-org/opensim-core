%module(directors="1") opensimMuscollo

%include java_exception.i

%{
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
#include <Bindings/OpenSimHeaders_actuators.h>
#include <Bindings/OpenSimHeaders_analyses.h>
#include <Bindings/OpenSimHeaders_tools.h>
#include <Bindings/OpenSimHeaders_muscollo.h>
using namespace OpenSim;
using namespace SimTK;
%}

%include "java_preliminaries.i"

/* Load the required libraries when this module is loaded.                    */
/* TODO be more clever about detecting location of library. */
%pragma(java) jniclassclassmodifiers="public class"
SWIG_JAVABODY_PROXY(public, public, SWIGTYPE)
%pragma(java) jniclassimports="import javax.swing.JOptionPane;import java.awt.GraphicsEnvironment;"
 
%pragma(java) jniclasscode=%{
  static {
      try{
          System.loadLibrary("osimMuscolloJavaJNI");
      }
      catch(UnsatisfiedLinkError e){
          String envVar = new String("PATH");
          String OS = System.getProperty("os.name").toLowerCase();
          if (OS.indexOf("win") >= 0) {
          } else if (OS.indexOf("mac") >= 0) {
              envVar = "DYLD_LIBRARY_PATH";
          } else {
              envVar = "LD_LIBRARY_PATH";
          }
          String msg = new String("Required library failed to load. " +
                  "Check that the dynamic library osimMuscolloJavaJNI " +
                  "is on your " + envVar + ".\n" + e);
          System.out.println(msg);
          if (!GraphicsEnvironment.isHeadless()) {
              new JOptionPane(msg, JOptionPane.ERROR_MESSAGE).createDialog(null, "Error").setVisible(true);
          }
      }
  }
%}

%import "java_actuators.i"

%include <Bindings/muscollo.i>

