%module(directors="1") opensimSimulation
%module opensimSimulation

#pragma SWIG nowarn=822,451,503,516,325,401

%{
#include <Bindings/OpenSimHeaders_common.h>
#include <Bindings/OpenSimHeaders_simulation.h>
#include <Bindings/Java/OpenSimJNI/OpenSimContext.h>

using namespace OpenSim;
using namespace SimTK;
%}

%include "java_preliminaries.i";

%include "arrays_java.i";

%typemap(out) OpenSim::Joint %{ $result = $1; markAdopted(); %}

%rename OpenSim::OrientationsReference::clone  unused_clone;

%javamethodmodifiers OpenSim::ForceSet::append "private";
%rename OpenSim::ForceSet::append private_append;
%typemap(javacode) OpenSim::ForceSet %{
   public boolean append(Force aForce) {
       aForce.markAdopted();
       return private_append(aForce);
   }
%}

%rename OpenSim::PathPointSet::clone unused_clone;

// replace insert with a variant that takes ownership of passed in AbstractPathPoint.
// in java the method 'insert' ends in super class so we can call it internally
// and that hides the super class method 
%typemap(javacode) OpenSim::PathPointSet %{
    public boolean insert(int aIndex, AbstractPathPoint aObject) {
       aObject.markAdopted();
       return super.insert(aIndex, aObject);
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

%javamethodmodifiers OpenSim::Model::addModelComponent "private";
%javamethodmodifiers OpenSim::Model::addBody "private";
%javamethodmodifiers OpenSim::Model::addMarker "private";
%javamethodmodifiers OpenSim::Model::addConstraint "private";
%javamethodmodifiers OpenSim::Model::addForce "private";
%javamethodmodifiers OpenSim::Model::addProbe "private";
%javamethodmodifiers OpenSim::Model::addContactGeometry "private";
%javamethodmodifiers OpenSim::Model::addController "private";
%javamethodmodifiers OpenSim::Model::addAnalysis "private";
%javamethodmodifiers OpenSim::Model::addJoint "private";

%rename OpenSim::Model::addModelComponent private_addModelComponent;
%rename OpenSim::Model::addBody private_addBody;
%rename OpenSim::Model::addMarker private_addMarker;
%rename OpenSim::Model::addConstraint private_addConstraint;
%rename OpenSim::Model::addForce private_addForce;
%rename OpenSim::Model::addProbe private_addProbe;
%rename OpenSim::Model::addContactGeometry private_addContactGeometry;
%rename OpenSim::Model::addController private_addController;
%rename OpenSim::Model::addAnalysis private_addAnalysis;
%rename OpenSim::Model::addJoint private_addJoint;

%rename OpenSim::PrescribedController::prescribeControlForActuator prescribeControlForActuator_private;
%javamethodmodifiers OpenSim::PrescribedController::prescribeControlForActuator_private "private";

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

  public void addMarker(Marker aMarker) {
      aMarker.markAdopted();
      private_addMarker(aMarker);
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

  public void addJoint(Joint aJoint) {
      aJoint.markAdopted();
      private_addJoint(aJoint);
  }
%}

%extend OpenSim::Model {
    static void LoadOpenSimLibrary(std::string libraryName){
        LoadOpenSimLibrary(libraryName, true);
    }

    void setDefaultControls(SimTK::Vector& newControls) {
        self->updDefaultControls() = newControls;
    }
}

%typemap(javacode) OpenSim::PrescribedController %{
    public void prescribeControlForActuator(int index, Function prescribedFunction) {
       prescribedFunction.markAdopted();
       prescribeControlForActuator_private(index, prescribedFunction);
    }

    public void prescribeControlForActuator(String name, Function prescribedFunction) {
       prescribedFunction.markAdopted();
       prescribeControlForActuator_private(name, prescribedFunction);
    }
%}


%javamethodmodifiers OpenSim::Frame::attachGeometry "private";
%rename OpenSim::Frame::attachGeometry private_attachGeometry;
%typemap(javacode) OpenSim::Frame %{
  public void attachGeometry(Geometry geom) {
      geom.markAdopted();
      private_attachGeometry(geom);
  }
%}

%javamethodmodifiers OpenSim::PhysicalFrame::addWrapObject "private";
%rename OpenSim::PhysicalFrame::addWrapObject private_addWrapObject;
%typemap(javacode) OpenSim::PhysicalFrame %{
  public void addWrapObject(WrapObject wrapObject) {
      wrapObject.markAdopted();
      private_addWrapObject(wrapObject);
  }
%}

%javamethodmodifiers OpenSim::TransformAxis::setFunction "private";
%rename OpenSim::TransformAxis::setFunction private_setFunction;
%typemap(javacode) OpenSim::TransformAxis %{
  public void setFunction(Function func) {
      func.markAdopted();
      private_setFunction(func);
  }
%}

%import "java_common.i"
opensim_unique_ptr(OpenSim::PositionMotion);
%include <Bindings/simulation.i>

