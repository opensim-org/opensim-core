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
              if (excep)
                  jenv->ThrowNew(excep,_ex.what());
	  }
}
