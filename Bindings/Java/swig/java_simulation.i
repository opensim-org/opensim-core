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

%typemap(out) OpenSim::Joint %{ $result = $1; markAdopted(); %}

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

%typemap(javacode) OpenSim::Model %{
  private String originalModelPath = null;
  // Important that we only refer to originalModelPath if the model's 
  // getInputFileName() is not set.
  public void setOriginalModelPathFromModel(Model model) {
    originalModelPath = null;
    if(model.getInputFileName()!=null && 
       !model.getInputFileName().equals(""))
        originalModelPath = 
            (new java.io.File(model.getInputFileName())).getParent();
    else if(model.originalModelPath!=null && 
            !model.originalModelPath.equals(""))
      originalModelPath = model.originalModelPath;
  }
  public String getFilePath() {
      if(getInputFileName()!=null && 
         !getInputFileName().equals("") && 
         (new java.io.File(getInputFileName())).getParent()!=null)
          return (new java.io.File(getInputFileName())).getParent() + 
              java.io.File.separator;
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

%pragma(java) jniclassclassmodifiers="public class"

SWIG_JAVABODY_PROXY(public, public, SWIGTYPE)

%pragma(java) jniclassimports="import javax.swing.JOptionPane;"

%pragma(java) jniclasscode=%{
  static {
      try{
          // All OpenSim classes required for GUI operation.
          System.loadLibrary("osimJavaJNISimbody");
          System.loadLibrary("osimJavaJNICommon");
          System.loadLibrary("osimJavaJNI");
      }
      catch(UnsatisfiedLinkError e){
          new JOptionPane("Required library failed to load. Check that the " +
                          "dynamic library osimJavaJNI is in your PATH\n" + e, 
        JOptionPane.ERROR_MESSAGE).createDialog(null, "Error").setVisible(true);
      }
  }
%}

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
        self->setInertia(SimTK::Inertia(aInertia[0], aInertia[1], aInertia[2], 
                                        aInertia[3], aInertia[4], aInertia[5]));
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


%import "java_common.i"

%include <Bindings/simulation.i>
%include <Bindings/actuators.i>
%include <Bindings/analyses.i>
%include <Bindings/tools.i>
%include <OpenSim/Utilities/simmFileWriterDLL/SimmFileWriter.h>

%include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

%include <Bindings/Java/OpenSimJNI/Hooks/SimtkLogCallback.h>
