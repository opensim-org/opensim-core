

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
          String OS = System.getProperty("os.name").toLowerCase();
          String tip = "";
          if (OS.indexOf("win") >= 0) {
              tip = "\nMake sure OpenSim's bin directory is on your PATH.";
          } else if (OS.indexOf("mac") >= 0) {
              // Nothing for now; our use of RPATH means we were probably able
              // to locate the OpenSim dynamic libraries.
          } else /* linux */ {
              // Nothing for now; our use of RPATH means we were probably able
              // to locate the OpenSim dynamic libraries.
          }
          String msg = new String(
                  "Failed to load one or more dynamic libraries for OpenSim.\n"
                  + e + tip);

          String javaHome = System.getProperties().getProperty("java.home");
          boolean inMatlab = javaHome.toLowerCase().indexOf("matlab") >= 0;
          if (inMatlab) {
              msg +=  "\nSee https://simtk-confluence.stanford.edu/display/OpenSim40/Scripting+with+Matlab";
          }
          
          System.out.println(msg);
          String title = "Error: Failed to load OpenSim libraries";
          if (!GraphicsEnvironment.isHeadless()) {
              new JOptionPane(msg, JOptionPane.ERROR_MESSAGE)
                    .createDialog(null, title).setVisible(true);
          }
      }
  }
%}



/* General exception handling for Java wrapping.                              */

%typemap(throws) SWIGTYPE, SWIGTYPE &, SWIGTYPE *, SWIGTYPE [ANY] %{
    SWIG_JavaThrowException(jenv, SWIG_JavaIOException,
                            "C++ $1_type exception thrown");
    return $null;
%}

%typemap(throws, throws="java.io.IOException") OpenSim::Exception {
    jclass excep = jenv->FindClass("java/io/IOException");
    if (excep)
        jenv->ThrowNew(excep, ($1).getMessage());
    return $null;
}

%exception {
	  try {
	  $action
	  }
	  catch(std::exception& _ex){
              jclass excep = jenv->FindClass("java/lang/RuntimeException");
              if (excep){
                  jenv->ThrowNew(excep,_ex.what());
                  return $null;
              }
	  }
}
