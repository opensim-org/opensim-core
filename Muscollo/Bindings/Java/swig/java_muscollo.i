%module(directors="1") opensimMuscollo

%include java_exception.i

%{
#include <Bindings/OpenSimHeaders_muscollo.h>
using namespace OpenSim;
using namespace SimTK;
%}

/* Load the required libraries when this module is loaded.                    */
/* TODO send console output instead of JOptionPane in case user is headless. */
/* TODO create different error message for mac/linux. */
/* TODO be more clever about detecting location of library. */
/* TODO isHeadless(): https://stackoverflow.com/questions/1014620/best-way-to-detect-whether-code-is-running-in-an-application-server-java */
%pragma(java) jniclassclassmodifiers="public class"
SWIG_JAVABODY_PROXY(public, public, SWIGTYPE)
%pragma(java) jniclassimports="import javax.swing.JOptionPane;"
%pragma(java) jniclasscode=%{
  static {
      try{
          System.loadLibrary("osimMuscolloJavaJNI");
      }
      catch(UnsatisfiedLinkError e){
          new JOptionPane("Required library failed to load. Check that the " +
                          "dynamic library osimMuscolloJavaJNI is in your PATH\n" + e, 
        JOptionPane.ERROR_MESSAGE).createDialog(null, "Error").setVisible(true);
      }
  }
%}

%import "java_actuators.i"

%include <Bindings/muscollo.i>

