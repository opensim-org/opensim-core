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
namespace SimTK {
%template(Mat33) Mat<3, 3>;
}
%include <SWIGSimTK/CoordinateAxis.h>
%include <SWIGSimTK/UnitVec.h>
namespace SimTK {
%template(UnitVec3)  SimTK::UnitVec<double,1>;
}

// Vector and Matrix
%include <Bindings/std.i>
%include <SWIGSimTK/BigMatrix.h>
namespace SimTK {
%extend RowVectorBase<double> {
     double __getitem__(size_t i) {
         if(i >= $self->nelt())
             throw Exception::Base{"Index out of Range."};

         return $self->getElt(0, i);
     }

     void __setitem__(size_t i, double value) {
         if(i >= $self->nelt())
             throw Exception::Base{"Index out of Range."};

         $self->updElt(0, i) = value;
     }
 }
%extend RowVector_<double> {
     RowVector_(const std::vector<double>& row) {
         return new RowVector_<double>{static_cast<int>(row.size()),
                                       row.data()};
     }
 }
%extend VectorBase<double> {
     double __getitem__(size_t i) {
         if(i >= $self->nelt())
             throw Exception::Base{"Index out of Range."};

         return $self->getElt(i, 0);
     }

     void __setitem__(size_t i, double value) {
         if(i >= $self->nelt())
             throw Exception::Base{"Index out of Range."};

         $self->updElt(i, 0) = value;
     }
 }
%extend Vector_<double> {
     Vector_(const std::vector<double>& row) {
         return new Vector_<double>{static_cast<int>(row.size()),
                                    row.data()};
     }
 }
%template(MatrixBaseDouble)    SimTK::MatrixBase<double>;
%template(Matrix)              SimTK::Matrix_<double>;
%template(VectorBaseDouble)    SimTK::VectorBase<double>;
%template(VectorView)          SimTK::VectorView_<double>;
%template(Vector)              SimTK::Vector_<double>;
%template(RowVectorBaseDouble) SimTK::RowVectorBase<double>;
%template(RowVectorView)       SimTK::RowVectorView_<double>;
%template(RowVector)           SimTK::RowVector_<double>;
}

%include <SWIGSimTK/SpatialAlgebra.h>
namespace SimTK {
%template(SpatialVec) Vec<2,   Vec3>;
%template(VectorOfSpatialVec) Vector_<SpatialVec>;
%template(VectorOfVec3) Vector_<Vec3>;
}


%include <SWIGSimTK/Rotation.h>
namespace SimTK {
%template(Rotation) SimTK::Rotation_<double>;
%template(InverseRotation) SimTK::InverseRotation_<double>;
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
}

typedef int MobilizedBodyIndex;
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
%template(ArrayIndexUnsigned) ArrayIndexTraits<unsigned>; 
%template(ArrayIndexInt) ArrayIndexTraits<int>; 
}

%include <SWIGSimTK/DecorativeGeometry.h>

namespace SimTK {
%template(ArrayDecorativeGeometry) SimTK::Array_<SimTK::DecorativeGeometry>;
}

// State & Stage
%include <SWIGSimTK/Stage.h>
%include <SWIGSimTK/State.h>
// Used for StatesTrajectory iterating.
%template(StdVectorState) std::vector<SimTK::State>;
