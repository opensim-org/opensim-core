

/* Load the required libraries when this module is loaded.                    */
%pragma(java) jniclassclassmodifiers="public class"
SWIG_JAVABODY_PROXY(public, public, SWIGTYPE)
%pragma(java) jniclassimports="import javax.swing.JOptionPane;import java.awt.GraphicsEnvironment;import java.io.File;import java.nio.file.Path;import java.nio.file.Files;import java.util.List;"
%pragma(java) jniclasscode=%{
  static {
      String OS = System.getProperty("os.name").toLowerCase();
      try {
          // Try to load osimJavaJNI's dependencies so we don't rely on the
          // PATH environment variable.
          // https://stackoverflow.com/questions/2591083/getting-java-version-at-runtime
          double javaVersion = Double.parseDouble(System.getProperty("java.specification.version"));
          if (OS.indexOf("win") >= 0 && javaVersion >= 1.7) {
              File jardir = new File(OpenSimObject.class.getProtectionDomain().getCodeSource().getLocation().toURI()).getParentFile();
              // java.nio.file.Paths requires Java 1.7.
              Path depfilePath = java.nio.file.Paths.get(jardir.toString(), "osimJavaJNI_dependencies.txt").normalize();
              File depfile = depfilePath.toFile();
              if (!depfile.exists() || depfile.isDirectory()) {
                  throw new Exception("OpenSim could not read " + depfile.toString() + ".");
              }
              // readAllLines() requires Java 1.7.
              List<String> libraries = Files.readAllLines(depfilePath, java.nio.charset.Charset.defaultCharset());
              for (String lib : libraries) {
                  String libToLoad = java.nio.file.Paths.get(jardir.toString(), lib).normalize().toString();
                  try {
                      System.load(libToLoad);
                  } catch (UnsatisfiedLinkError e) {
                      // Not fatal, but we can give an informative message to help the user
                      // fix the dependencies text file.
                      System.out.println("OpenSim cannot load " + libToLoad + " listed in " + depfile.toString() + ": " + e);
                  }
              }
          }
      } catch (Exception e) {
          // We do not treat this as an error, so we do not print any diagnostics.
          // We expect to end up here if run from the GUI or opensim-core tests (build directory).
          System.out.println("OpenSim is relying on Windows PATH to find libraries.");
      }
      try {
          // All OpenSim classes required for GUI operation.
          System.loadLibrary("osimJavaJNI");
      }
      catch(UnsatisfiedLinkError e){
          String tip = "";
          if (OS.indexOf("win") >= 0) {
              tip = "\nPlace OpenSim's bin directory on your Windows PATH or edit sdk\\Java\\osimJavaJNI_dependencies.txt.";
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
              msg +=  "\nSee https://simtk-confluence.stanford.edu/display/OpenSim/Scripting+with+Matlab";
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
