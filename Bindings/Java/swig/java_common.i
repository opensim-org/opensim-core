%module(directors="1") opensimCommon
%module opensimCommon

#pragma SWIG nowarn=822,451,503,516,325,401

%{
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/PropertyHelper.h>
#include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

using namespace OpenSim;
using namespace SimTK;
%}

%include "java_preliminaries.i";

%include "arrays_java.i";

%typemap(javacode) OpenSim::Object %{
  public boolean equals(Object obj) {
    boolean equal = false;
    if (obj instanceof $javaclassname)
      equal = ((($javaclassname)obj).swigCPtr == this.swigCPtr);
    return equal;
  }
  // cache the Id to avoid recomputation for hashing purposes
  private int cacheId=-1;  
 
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

%typemap(javacode) OpenSim::MarkerData %{
  public double[] getTimeRange() { 
      return new double[]{getStartFrameTime(), getLastFrameTime()}; 
  }
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
          // throw new ParseException("Illegal format: Expect space separated" +
          //                          " values, optionally between matched " +
          //                          "parentheses", liveEnd);
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
    // cache pointer to function so it's not garbage collected early.
  private Function  dFunction;  

  public XYFunctionInterface(Function aFunction, Boolean unused) {
      this(aFunction);
      dFunction = aFunction;
  }
%}

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

// This method takes ownership of the passed-in object; make sure the bindings
// don't try to handle ownership.
%javamethodmodifiers OpenSim::Component::addComponent "private";
%rename OpenSim::Component::addComponent private_addComponent;
%typemap(javacode) OpenSim::Component %{
  public void addComponent(Component comp) {
      comp.markAdopted();
      private_addComponent(comp);
  }
%}

%typemap(javacode) OpenSim::LogSink %{
  public void markAdopted() {
      if (swigCPtr != 0) {
          if (swigCMemOwn) swigCMemOwn = false;
      }
  }
%}
// This method takes ownership of the passed-in object; make sure the bindings
// don't try to handle ownership.
%javamethodmodifiers OpenSim::Logger::addSink "private";
%rename OpenSim::Logger::addSink private_addSink;
%typemap(javacode) OpenSim::Logger %{
  public static void addSink(LogSink sink) {
      sink.markAdopted();
      private_addSink(sink);
  }
%}

%import "java_simbody.i"

%include <Bindings/common.i>
