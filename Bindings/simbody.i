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

// Mat33
%include <SWIGSimTK/Mat.h>
%include <SimTKcommon/SmallMatrix.h> // for typedefs like Mat33.
namespace SimTK {
%template(Mat33) Mat<3, 3>;
}
%include <SWIGSimTK/CoordinateAxis.h>
%include <SWIGSimTK/UnitVec.h>
namespace SimTK {
%template(UnitVec3)  SimTK::UnitVec<double,1>;
}

// Vector and Matrix
//%include <Bindings/std.i>
%include <SWIGSimTK/BigMatrix.h>
%template(StdVectorVec3) std::vector<SimTK::Vec3>;

namespace SimTK {
%extend RowVectorBase<double> {
     double __getitem__(size_t i) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         return $self->getElt(0, i);
     }

     void __setitem__(size_t i, double value) {
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
     double __getitem__(size_t i) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         return $self->getElt(i, 0);
     }

     void __setitem__(size_t i, double value) {
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

%extend RowVectorBase<Vec3> {
     Vec3 __getitem__(size_t i) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         return $self->getElt(0, i);
     }

     void __setitem__(size_t i, Vec3 value) {
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
         for(unsigned i = 0; i < colVec.nelt(); ++i)
             colVec[i] = $self->operator[](i);
         return colVec;
     }
 }
%extend VectorBase<Vec3> {
     Vec3 __getitem__(size_t i) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         return $self->getElt(i, 0);
     }

     void __setitem__(size_t i, Vec3 value) {
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
         for(unsigned i = 0; i < rowVec.nelt(); ++i)
             rowVec[i] = $self->operator[](i);
         return rowVec;
     }
 }
%template(MatrixBaseVec3)    SimTK::MatrixBase<Vec3>;
%template(MatrixViewVec3)    SimTK::MatrixView_<Vec3>;
%template(MatrixVec3)        SimTK::Matrix_<Vec3>;
%template(VectorBaseVec3)    SimTK::VectorBase<Vec3>;
%template(VectorViewVec3)    SimTK::VectorView_<Vec3>;
// Following is wrapped few lines below.
// %template(VectorVec3)        SimTK::Vector_<Vec3>;
%template(RowVectorBaseVec3) SimTK::RowVectorBase<Vec3>;
%template(RowVectorViewVec3) SimTK::RowVectorView_<Vec3>;
%template(RowVectorOfVec3)     SimTK::RowVector_<Vec3>;
}

%include <SWIGSimTK/SpatialAlgebra.h>
namespace SimTK {
%template(SpatialVec) Vec<2,   Vec3>;
%template(VectorOfSpatialVec) Vector_<SpatialVec>;
%template(VectorOfVec3) Vector_<Vec3>;
%template(MatrixOfSpatialVec) Matrix_<SpatialVec>;
}

%include <SWIGSimTK/Rotation.h>
namespace SimTK {
%template(Rotation) SimTK::Rotation_<double>;
%template(InverseRotation) SimTK::InverseRotation_<double>;
}

%extend SimTK::Rotation_<double> {
    Vec3 multiply(const Vec3& v) {
        return operator*(*$self, v);
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
