%module(directors="1") opensimModel
%module opensimModel
#pragma SWIG nowarn=822,451,503,516,325,401
%{

#include <Bindings/OpenSimHeaders_opensim.h>
#include <Bindings/OpenSimHeaders_tools.h>
#include <OpenSim/Utilities/simmFileWriterDLL/SimmFileWriter.h>

#include <Bindings/Java/OpenSimJNI/Hooks/SimtkLogCallback.h>
#include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

using namespace OpenSim;
using namespace SimTK;

%}


/* This file is for creation/handling of arrays */
%include "arrays_java.i";

%typemap(javacode) OpenSim::Object %{
  public boolean equals(Object obj) {
    boolean equal = false;
    if (obj instanceof $javaclassname)
      equal = ((($javaclassname)obj).swigCPtr == this.swigCPtr);
    return equal;
  }
  private int cacheId=-1;  // cache the Id to avoid recomputation for hashing purposes
 
  public int hashCode() {
     if (cacheId==-1)
        cacheId=(int)swigCPtr;
     
    return( cacheId );
  }
  // Flag to indicate if an object is pickable in the GUI
  // Example of a non-pickable object would be a PathWrapPoint
  private boolean pickable=true;
  
  public boolean isPickable() {
	 return pickable;
  }
  
  public void setPickable(boolean onOff) {
	 pickable=onOff;
  }
  public void markAdopted() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) swigCMemOwn = false;
    }
  }  

%}

%typemap(out) OpenSim::Joint %{ $result = $1; markAdopted(); %}

%typemap(javacode) OpenSim::MarkerData %{
  public double[] getTimeRange() { return new double[]{getStartFrameTime(), getLastFrameTime()}; }
%}

%newobject *::clone; 

%typemap(javacode) OpenSim::Array<double> %{
	public void fromString(String string) {
      // Remove open and close parenth if any
      String workString= new String(string);
      int liveStart = workString.indexOf("(");
      int liveEnd = workString.indexOf(")");
      if (liveStart!=-1 && liveEnd!=-1){
          workString = workString.substring(liveStart+1, liveEnd);
      }
      else if (liveStart!=liveEnd){
          //throw new ParseException("Illegal format: Expect space separated values, optionally between matched parentheses", liveEnd);
          return;
      }
      String[] splits = workString.split(" ");
      double[] values = new double[splits.length];
      for(int i=0; i<splits.length; i++){
           values[i]=Double.parseDouble(splits[i]);
       }
       this.setValues(values, splits.length);
	}
%}

%typemap(javacode) SimTK::Vec3 %{
    double[] getAsJavaArray() {
		return new double[]{this->get(0), this->get(1), this->get(2)};
	}
%}

%extend  SimTK::DecorativeGeometry {
public:
	bool hasUserRef() const {
		return (self->getUserRef()!=0);
	}

	OpenSim::Object& getUserRefAsObject() {
		return *((OpenSim::Object*)self->getUserRef());
	}
}
%extend SimTK::DecorativeMeshFile {
     SimTK::DecorativeMeshFile* clone() { return new SimTK::DecorativeMeshFile(*self); }
}
%extend SimTK::DecorativeSphere {
     SimTK::DecorativeSphere* clone() { return new SimTK::DecorativeSphere(*self); }
}
%extend SimTK::DecorativeBrick {
     SimTK::DecorativeBrick* clone() { return new SimTK::DecorativeBrick(*self); }
}

%extend SimTK::DecorativeLine {
     SimTK::DecorativeLine* clone() { return new SimTK::DecorativeLine(*self); }
}
%extend SimTK::DecorativeCylinder {
     SimTK::DecorativeCylinder* clone() { return new SimTK::DecorativeCylinder(*self); }
}
%extend SimTK::DecorativeEllipsoid {
     SimTK::DecorativeEllipsoid* clone() { return new SimTK::DecorativeEllipsoid(*self); }
}
%extend SimTK::DecorativeFrame {
     SimTK::DecorativeFrame* clone() { return new SimTK::DecorativeFrame(*self); }
}
%extend SimTK::DecorativeArrow {
     SimTK::DecorativeArrow* clone() { return new SimTK::DecorativeArrow(*self); }
}
%extend SimTK::DecorativeTorus {
     SimTK::DecorativeTorus* clone() { return new SimTK::DecorativeTorus(*self); }
}
%extend SimTK::DecorativeCone {
     SimTK::DecorativeCone* clone() { return new SimTK::DecorativeCone(*self); }
}
%javamethodmodifiers OpenSim::Model::addModelComponent "private";
%javamethodmodifiers OpenSim::Model::addBody "private";
%javamethodmodifiers OpenSim::Model::addConstraint "private";
%javamethodmodifiers OpenSim::Model::addForce "private";
%javamethodmodifiers OpenSim::Model::addProbe "private";
%javamethodmodifiers OpenSim::Model::addContactGeometry "private";
%javamethodmodifiers OpenSim::Model::addController "private";
%javamethodmodifiers OpenSim::Model::addAnalysis "private";

%rename OpenSim::Model::addModelComponent private_addModelComponent;
%rename OpenSim::Model::addBody private_addBody;
%rename OpenSim::Model::addConstraint private_addConstraint;
%rename OpenSim::Model::addForce private_addForce;
%rename OpenSim::Model::addProbe private_addProbe;
%rename OpenSim::Model::addContactGeometry private_addContactGeometry;
%rename OpenSim::Model::addController private_addController;
%rename OpenSim::Model::addAnalysis private_addAnalysis;


%typemap(javacode) OpenSim::FunctionSet %{
  public boolean adoptAndAppend(Function aFunction) {
	aFunction.markAdopted();
    return super.adoptAndAppend(aFunction);
  }
%}

%typemap(javacode) OpenSim::SetMarkerWeights %{
  public boolean adoptAndAppend(MarkerWeight aObject) {
	aObject.markAdopted();
    return super.adoptAndAppend(aObject);
  }
%}
%typemap(javacode) OpenSim::Model %{
  private String originalModelPath = null;
  // Important that we only refer to originalModelPath if the model's getInputFileName() is not set
  public void setOriginalModelPathFromModel(Model model) {
    originalModelPath = null;
    if(model.getInputFileName()!=null && !model.getInputFileName().equals(""))
      originalModelPath = (new java.io.File(model.getInputFileName())).getParent();
	 else if(model.originalModelPath!=null && !model.originalModelPath.equals(""))
      originalModelPath = model.originalModelPath;
  }
  public String getFilePath() {
    if(getInputFileName()!=null && !getInputFileName().equals("") && (new java.io.File(getInputFileName())).getParent()!=null)
      return (new java.io.File(getInputFileName())).getParent() + java.io.File.separator;
    else if(originalModelPath!=null && !originalModelPath.equals(""))
      return originalModelPath + java.io.File.separator;
    else return "";
  }

  public void addModelComponent(ModelComponent aComponent) {
	aComponent.markAdopted();
    private_addModelComponent(aComponent);
  }

  public void addBody(Body aBody) {
	aBody.markAdopted();
    private_addBody(aBody);
  }

  public void addConstraint(Constraint aConstraint) {
	aConstraint.markAdopted();
    private_addConstraint(aConstraint);
  }

  public void addProbe(Probe aProbe) {
	aProbe.markAdopted();
    private_addProbe(aProbe);
  }  
  
  public void addContactGeometry(ContactGeometry aContactGeometry) {
	aContactGeometry.markAdopted();
    private_addContactGeometry(aContactGeometry);
  }

  public void addAnalysis(Analysis aAnalysis) {
	aAnalysis.markAdopted();
	private_addAnalysis(aAnalysis);
  }

  public void addForce(Force aForce) {
	aForce.markAdopted();
	private_addForce(aForce);
  }

  public void addController(Controller aController) {
	aController.markAdopted();
	private_addController(aController);
  }
%}

%javamethodmodifiers OpenSim::ForceSet::append "private";
%rename OpenSim::ForceSet::append private_append;
%typemap(javacode) OpenSim::ForceSet %{
   public boolean append(Force aFroce) {
      aFroce.markAdopted();
	  return private_append(aFroce);
   }
%}

%typemap(javacode) OpenSim::Array<std::string> %{
   public java.util.Vector<String> toVector() {
      java.util.Vector<String> vector = new java.util.Vector<String>();
      vector.setSize(getSize());
      for(int i=0; i<getSize(); i++) vector.set(i, getitem(i));
      return vector;
   }
   public void append(java.util.Vector<String> vector) {
      for(int i=0; i<vector.size(); i++) append(vector.get(i));
   }
   public static ArrayStr fromVector(java.util.Vector<String> vector) {
      ArrayStr array = new ArrayStr();
      array.append(vector);
      return array;
   }
%}

%typemap(javacode) OpenSim::XYFunctionInterface %{
  private Function  dFunction;  // cache pointer to function so it's not garbage collected early

  public XYFunctionInterface(Function aFunction, Boolean unused) {
		this(aFunction);
		dFunction = aFunction;
  }
%}

%pragma(java) jniclassclassmodifiers="public class"

SWIG_JAVABODY_PROXY(public, public, SWIGTYPE)

%pragma(java) jniclassimports="import javax.swing.JOptionPane;"

%pragma(java) jniclasscode=%{
  static {
      try{
        System.loadLibrary("osimJavaJNI");		// All OpenSim classes required for GUI operation.
      }
      catch(UnsatisfiedLinkError e){
           new JOptionPane("Required library failed to load. Check that the dynamic library osimJavaJNI is in your PATH\n"+e, 
				JOptionPane.ERROR_MESSAGE).createDialog(null, "Error").setVisible(true);
      }
  }
%}


// Generic Exception handling
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
  if (OpenSim::mapCxxExceptionsToJava){
	  try {
	  $action
	  }
	  catch(std::exception& _ex){
		  jclass excep = jenv->FindClass("java/lang/RuntimeException");
		  if (excep)
		  jenv->ThrowNew(excep,_ex.what());
	  }
  }
  else
	$action
}


%extend OpenSim::Body {
	void getInertia(Array<double>& rInertia) {
		SimTK::Mat33 inertia= self->getInertia().toMat33();
		rInertia[0]=inertia[0][0];
		rInertia[1]=inertia[1][1];
		rInertia[2]=inertia[2][2];
		rInertia[3]=inertia[0][1];
		rInertia[4]=inertia[0][2];
		rInertia[5]=inertia[1][2];

	};
	void setInertia(Array<double>& aInertia) {
		self->setInertia(SimTK::Inertia(aInertia[0], 
		aInertia[1], aInertia[2], aInertia[3], aInertia[4], aInertia[5]));
	}
};

/*
Extend concrete Joints to use the inherited base constructors.
This is only necessary because SWIG does not generate these inherited
constructors provided by C++11's 'using' (e.g. using Joint::Joint) declaration.
Note that CustomJoint and EllipsoidJoint do implement their own
constructors because they have additional arguments.
*/
%define EXPOSE_JOINT_CONSTRUCTORS_HELPER(NAME)
%extend OpenSim::NAME {
	NAME(const std::string& name,
         const std::string& parentName,
         const std::string& childName) {
		return new NAME(name, parentName, childName, false);
	}
	
	NAME(const std::string& name,
         const PhysicalFrame& parent,
         const SimTK::Vec3& locationInParent,
         const SimTK::Vec3& orientationInParent,
         const PhysicalFrame& child,
         const SimTK::Vec3& locationInChild,
         const SimTK::Vec3& orientationInChild) {
		return new NAME(name, parent, locationInParent, orientationInParent,
					child, locationInChild, orientationInChild, false);
	}
};
%enddef

EXPOSE_JOINT_CONSTRUCTORS_HELPER(FreeJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(BallJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(PinJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(SliderJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(WeldJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(GimbalJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(UniversalJoint);
EXPOSE_JOINT_CONSTRUCTORS_HELPER(PlanarJoint);


%extend OpenSim::Array<double> {
	void setValues(double dValues[], int size) {
		self->setSize(size);
		for(int i=0; i< size; i++)
		 self->set(i, dValues[i]);
	};
	SimTK::Vec3 getAsVec3() {
		return SimTK::Vec3::getAs(self->get());
	};
	
	static SimTK::Vec3 createVec3(double e1, double e2, double e3) {
		Array<double>* arr = new Array<double>(e1, 3);
		arr->set(1, e2);
		arr->set(2, e3);
		return SimTK::Vec3::getAs(arr->get());
	};
  
   static SimTK::Vec3 createVec3(double e1) {
		Array<double>* arr = new Array<double>(e1, 3);
		return SimTK::Vec3::getAs(arr->get());
  };
   
   static SimTK::Vec3  createVec3(double es[3]) {
		Array<double>* arr = new Array<double>(es[0], 3);
		arr->set(1, es[1]);
		arr->set(2, es[2]);
		return SimTK::Vec3::getAs(arr->get());
  };

   SimTK::Vector_<double>  getAsVector() {
		return SimTK::Vector(self->getSize(), &(*self)[0]);
  };

   void populateFromVector(SimTK::Vector_<double> aVector) {
		int sz = aVector.size();
		for(int i=0; i<sz; ++i)
			self->append(aVector[i]);
   }

   static  OpenSim::Array<double> getValuesFromVec3(SimTK::Vec3 vec3) {
		OpenSim::Array<double> arr(0, 3);
		for (int i=0; i<3; i++) arr[i] = vec3[i];
		return arr;
  };
  
  std::string toString() const {
		std::stringstream stream;
		for (int i=0; i< self->getSize(); i++)
			stream <<  self->get(i) << " ";
		return stream.str(); 
  }

};

%extend OpenSim::Model {
	static void LoadOpenSimLibrary(std::string libraryName){
		LoadOpenSimLibrary(libraryName);
	}

	void setDefaultControls(SimTK::Vector& newControls) {
		self->updDefaultControls() = newControls;
	}
}

%extend OpenSim::Manager {
	void setIntegratorAccuracy(double accuracy){
		self->getIntegrator().setAccuracy(accuracy);
	}
}

%extend OpenSim::Object {
	static OpenSim::Array<std::string> getFunctionClassNames() {
		  OpenSim::Array<std::string> availableClassNames;
		  ArrayPtrs<OpenSim::Function> rArray;
		  Object::getRegisteredObjectsOfGivenType<OpenSim::Function>(rArray);
		  for (int i=0;i<rArray.size(); i++)
			availableClassNames.append(rArray[i]->getConcreteClassName());
		  
		  return availableClassNames;
	}
}

%include <Bindings/preliminaries.i>
%include <Bindings/simbody.i>
%include <Bindings/opensim.i>
%include <Bindings/tools.i>
%include <OpenSim/Utilities/simmFileWriterDLL/SimmFileWriter.h>

%include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

%include <Bindings/Java/OpenSimJNI/Hooks/SimtkLogCallback.h>
