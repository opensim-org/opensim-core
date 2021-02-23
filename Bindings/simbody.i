%feature("director") SimTK::DecorativeGeometryImplementation;

%include <SimTKcommon.h>

%include <SimTKcommon/Constants.h>

%include <SimTKcommon/internal/IteratorRange.h>

%include <SWIGSimTK/Vec.h>

// Vec3
namespace SimTK {
%template(Vec2) Vec<2>;
%template(Vec3) Vec<3>;
%template(Vec4) Vec<4>;
%template(Vec6) Vec<6>;
}
%define ADD_VEC_OPERATOR_METHODS(VEC_TYPE)
%extend VEC_TYPE {
	%template(scalarEq) scalarEq<double>;
	%template(scalarPlusEq) scalarPlusEq<double>;
	%template(scalarMinusEq) scalarMinusEq<double>;
	%template(scalarTimesEq) scalarTimesEq<double>;
	%template(scalarDivideEq) scalarDivideEq<double>;
}
%enddef

ADD_VEC_OPERATOR_METHODS(SimTK::Vec<2>)

ADD_VEC_OPERATOR_METHODS(SimTK::Vec<3>)

ADD_VEC_OPERATOR_METHODS(SimTK::Vec<4>)

ADD_VEC_OPERATOR_METHODS(SimTK::Vec<6>)

// Mat33
%include <SWIGSimTK/Mat.h>
%include <SimTKcommon/SmallMatrix.h> // for typedefs like Mat33.
namespace SimTK {
%template(Mat33) Mat<3, 3>;
}
%include <SWIGSimTK/CoordinateAxis.h>
%include <SWIGSimTK/UnitVec.h>
%include <SWIGSimTK/Quaternion.h>
namespace SimTK {
%template(UnitVec3)  SimTK::UnitVec<double,1>;
%template(Quaternion)  SimTK::Quaternion_<double>;
}

// Vector and Matrix
//%include <Bindings/std.i>
%include <SWIGSimTK/BigMatrix.h>
%template(StdVectorVec3) std::vector<SimTK::Vec3>;

namespace SimTK {
%extend RowVectorBase<double> {
     double __getitem__(int i) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         return $self->getElt(0, i);
     }

     void __setitem__(int i, double value) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         $self->updElt(0, i) = value;
     }
 }

%extend RowVector_<double> {
     RowVector_(const std::vector<double>& row) {
         return new RowVector_<double>{static_cast<int>(row.size()),
                                       row.data()};
     }

     Vector_<double> transpose() {
         return $self->operator~();
     }
 }
%extend VectorBase<double> {
     double __getitem__(int i) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         return $self->getElt(i, 0);
     }

     void __setitem__(int i, double value) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         $self->updElt(i, 0) = value;
     }
}
%extend Vector_<double> {
     Vector_(const std::vector<double>& row) {
         return new Vector_<double>{static_cast<int>(row.size()),
                                    row.data()};
     }

     RowVector_<double> transpose() {
         return $self->operator~();
     }
 }
%template(MatrixBaseDouble)    SimTK::MatrixBase<double>;
%template(MatrixView)          SimTK::MatrixView_<double>;
%template(Matrix)              SimTK::Matrix_<double>;
%template(VectorBaseDouble)    SimTK::VectorBase<double>;
%template(VectorView)          SimTK::VectorView_<double>;
%template(Vector)              SimTK::Vector_<double>;
%template(RowVectorBaseDouble) SimTK::RowVectorBase<double>;
%template(RowVectorView)       SimTK::RowVectorView_<double>;
%template(RowVector)           SimTK::RowVector_<double>;
// This typedef should not be necessary but, in practice, this typedef
// helps SWIG recognize the use of SimTK::RowVector in OpenSim's headers (e.g.,
// MocoTrajectory.h).
typedef SimTK::RowVector_<double> RowVector;

%extend RowVectorBase<Vec3> {
     Vec3 __getitem__(int i) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         return $self->getElt(0, i);
     }

     void __setitem__(int i, Vec3 value) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         $self->updElt(0, i) = value;
     }

 }
%extend RowVector_<Vec3> {
     RowVector_(const std::vector<Vec3>& row) {
         return new RowVector_<Vec3>{static_cast<int>(row.size()),
                                     row.data()};
     }

     Vector_<Vec3> transpose() {
         Vector_<Vec3> colVec{static_cast<int>($self->nelt())};
         for(int i = 0; i < colVec.nelt(); ++i)
             colVec[i] = $self->operator[](i);
         return colVec;
     }
 }
%extend VectorBase<Vec3> {
     Vec3 __getitem__(int i) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         return $self->getElt(i, 0);
     }

     void __setitem__(int i, Vec3 value) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         $self->updElt(i, 0) = value;
     }
}
%extend Vector_<Vec3> {
     Vector_(const std::vector<Vec3>& row) {
         return new Vector_<Vec3>{static_cast<int>(row.size()),
                                  row.data()};
     }

     RowVector_<Vec3> transpose() {
         RowVector_<Vec3> rowVec{static_cast<int>($self->nelt())};
         for(int i = 0; i < rowVec.nelt(); ++i)
             rowVec[i] = $self->operator[](i);
         return rowVec;
     }
 }

// Swig macro to instantiate all template classes needed 
// to make a DataTable of type ETY
%define INSTANTIATE_MATRIXTYPES(ETY)
%template(MatrixBase ## ETY)    SimTK::MatrixBase<ETY>;
%template(MatrixView ## ETY)    SimTK::MatrixView_<ETY>;
%template(Matrix ## ETY)        SimTK::Matrix_<ETY>;
%template(VectorBase ## ETY)    SimTK::VectorBase<ETY>;
%template(VectorView ## ETY)    SimTK::VectorView_<ETY>;
%template(Vector ## ETY)        SimTK::Vector_<ETY>;
%template(RowVectorBase ## ETY) SimTK::RowVectorBase<ETY>;
%template(RowVectorView ## ETY) SimTK::RowVectorView_<ETY>;
%template(RowVector ## ETY)     SimTK::RowVector_<ETY>;
%enddef

INSTANTIATE_MATRIXTYPES(Vec3)
INSTANTIATE_MATRIXTYPES(Vec6)


%template(MatrixBaseQuaternion)    SimTK::MatrixBase<Quaternion_<double>>;
//%template(MatrixViewQuaternion)    SimTK::MatrixView_<Quaternion_<double>>;
%template(MatrixQuaternion)        SimTK::Matrix_<Quaternion_<double>>;
%template(VectorBaseQuaternion)    SimTK::VectorBase<Quaternion_<double>>;
//%template(VectorViewQuaternion)    SimTK::VectorView_<Quaternion_<double>>;
%template(VectorQuaternion)        SimTK::Vector_<Quaternion_<double>>;
%template(RowVectorBaseQuaternion) SimTK::RowVectorBase<Quaternion_<double>>;
%template(RowVectorViewQuaternion) SimTK::RowVectorView_<Quaternion_<double>>;
%template(RowVectorQuaternion)     SimTK::RowVector_<Quaternion_<double>>;

}

%include <SWIGSimTK/SpatialAlgebra.h>
namespace SimTK {
%template(SpatialVec) Vec<2,   Vec3>;
%template(VectorOfSpatialVec) Vector_<SpatialVec>;
%template(MatrixOfSpatialVec) Matrix_<SpatialVec>;
}

%include <SWIGSimTK/Rotation.h>
namespace SimTK {
%template(Rotation) SimTK::Rotation_<double>;
%template(InverseRotation) SimTK::InverseRotation_<double>;

INSTANTIATE_MATRIXTYPES(Mat33)
%template(MatrixBaseRotation)    SimTK::MatrixBase<SimTK::Rotation_<double>>;
//%template(MatrixViewRotation)    SimTK::MatrixView_<SimTK::Rotation_<double>>;
%template(MatrixRotation)        SimTK::Matrix_<SimTK::Rotation_<double>>;
%template(VectorBaseRotation)    SimTK::VectorBase<SimTK::Rotation_<double>>;
//%template(VectorViewRotation)    SimTK::VectorView_<SimTK::Rotation_<double>>;
%template(VectorRotation)        SimTK::Vector_<SimTK::Rotation_<double>>;
%template(RowVectorBaseRotation) SimTK::RowVectorBase<SimTK::Rotation_<double>>;
%template(RowVectorViewRotation) SimTK::RowVectorView_<SimTK::Rotation_<double>>;
%template(RowVectorRotation)     SimTK::RowVector_<SimTK::Rotation_<double>>;

}

%extend SimTK::Rotation_<double> {
    Vec3 multiply(const Vec3& v) {
        return operator*(*$self, v);
    }

    Rotation_<double> multiply(const Rotation_<double>& r) {
        return operator*(*$self, r);
    }

    Rotation_<double> multiply(const InverseRotation_<double>& r) {
        return operator*(*$self, r);
    }

    RowVector_<Vec3> multiply(const RowVector_<Vec3>& row) {
        return operator*(*$self, row);
    }

    RowVector_<Vec3> multiply(const RowVectorView_<Vec3>& row) {
        return operator*(*$self, row);
    }
}


// Transform
%include <SWIGSimTK/Transform.h>
namespace SimTK {
%template(Transform) SimTK::Transform_<double>;
}

//
%include <SWIGSimTK/MassProperties.h>
namespace SimTK {
%template(Inertia) SimTK::Inertia_<double>;
%template(MassProperties) SimTK::MassProperties_<double>;
}
%include <SWIGSimTK/common.h>
%include <SWIGSimTK/Array.h>
namespace SimTK {
%template(SimTKArrayString) SimTK::Array_<std::string>;
%template(SimTKArrayDouble) SimTK::Array_<double>;
%template(SimTKArrayVec3) SimTK::Array_<SimTK::Vec3>;
%template(SimTKArrayInt) SimTK::Array_<int>;
%template(SimTKArrayRotation) SimTK::Array_<SimTK::Rotation_<double>>;
}

typedef int SubsystemIndex;
typedef int SystemQIndex;
typedef int SystemQErrIndex;
typedef int SystemZIndex;
typedef int SystemYIndex;
typedef int SystemYErrIndex;
typedef int SystemUIndex;
typedef int SystemUErrIndex;
typedef int SystemUDotErrIndex;
namespace SimTK {
%template(SimTKArrayMobilizedBodyIndex) SimTK::Array_<MobilizedBodyIndex>;
}
typedef int MobilizedBodyIndex;

namespace SimTK {
%template(ArrayIndexUnsigned) ArrayIndexTraits<unsigned>;
%template(ArrayIndexInt) ArrayIndexTraits<int>;
}

%include <SWIGSimTK/PolygonalMesh.h>
%include <SWIGSimTK/DecorativeGeometry.h>


namespace SimTK {
%template(ArrayDecorativeGeometry) SimTK::Array_<SimTK::DecorativeGeometry>;
}

// State & Stage
%include <SWIGSimTK/Stage.h>
%include <SWIGSimTK/State.h>
// Used for StatesTrajectory iterating.
%template(StdVectorState) std::vector<SimTK::State>;
%include <SWIGSimTK/SimbodyMatterSubsystem.h>

%rename(SimTKVisualizer) SimTK::Visualizer;
%include <simbody/internal/Visualizer.h>

// Wrap SimTK::Visualizer and InputSilo to put geometry in Visualizer and
// obtain keyboard input.
// Nested classes are inaccessible from MATLAB.
%feature("flatnested") SimTK::Visualizer::InputListener;
%feature("flatnested") SimTK::Visualizer::InputSilo;
%rename(SimTKVisualizerInputListener) SimTK::Visualizer::InputListener;
%rename(SimTKVisualizerInputSilo) SimTK::Visualizer::InputSilo;
%include <simbody/internal/Visualizer_InputListener.h>
// The following is necessary because the BackgroundType enum cannot be used
// from MATLAB.
// The new version of takeKeyHit() allows returning the value rather than using
// an output variable as an argument, which is difficult to manage with SWIG.
// The alternative `waitForKeyHit()` is not ideal for MATLAB, as MATLAB is not
// able to interrupt native functions, and `waitForKeyHit()` will hang if the
// simbody-visualizer is killed.
namespace SimTK {
%extend Visualizer {
    const Visualizer& setBackgroundTypeByInt(int index) {
        if (index == 1) $self->setBackgroundType(SimTK::Visualizer::GroundAndSky);
        else if (index == 2) $self->setBackgroundType(SimTK::Visualizer::SolidColor);
        return *($self);
    }
}
%extend Visualizer::InputSilo {
    unsigned takeKeyHitKeyOnly() {
        unsigned key = 0;
        unsigned modifier = 0;
        $self->takeKeyHit(key, modifier);
        return key;
    }
}
}

