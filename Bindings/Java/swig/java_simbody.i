%module(directors="1") opensimSimbody
%module opensimSimbody

#pragma SWIG nowarn=822,451,503,516,325,401

%{
#include <SimTKsimbody.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ObjectGroup.h>

using namespace SimTK;
%}

/* This file is for creation/handling of arrays */
%include "arrays_java.i";

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
     SimTK::DecorativeMeshFile* clone() { 
         return new SimTK::DecorativeMeshFile(*self); 
     }
}

%extend SimTK::DecorativeSphere {
     SimTK::DecorativeSphere* clone() { 
         return new SimTK::DecorativeSphere(*self); 
     }
}

%extend SimTK::DecorativeBrick {
     SimTK::DecorativeBrick* clone() { 
         return new SimTK::DecorativeBrick(*self); 
     }
}

%extend SimTK::DecorativeLine {
     SimTK::DecorativeLine* clone() { 
         return new SimTK::DecorativeLine(*self); 
     }
}

%extend SimTK::DecorativeCylinder {
     SimTK::DecorativeCylinder* clone() { 
         return new SimTK::DecorativeCylinder(*self); 
     }
}

%extend SimTK::DecorativeEllipsoid {
     SimTK::DecorativeEllipsoid* clone() { 
         return new SimTK::DecorativeEllipsoid(*self); 
     }
}

%extend SimTK::DecorativeFrame {
     SimTK::DecorativeFrame* clone() { 
         return new SimTK::DecorativeFrame(*self); 
     }
}

%extend SimTK::DecorativeArrow {
     SimTK::DecorativeArrow* clone() { 
         return new SimTK::DecorativeArrow(*self); 
     }
}

%extend SimTK::DecorativeTorus {
     SimTK::DecorativeTorus* clone() { 
         return new SimTK::DecorativeTorus(*self); 
     }
}

%extend SimTK::DecorativeCone {
     SimTK::DecorativeCone* clone() { 
         return new SimTK::DecorativeCone(*self); 
     }
}


%include <Bindings/preliminaries.i>
%include <Bindings/simbody.i>
