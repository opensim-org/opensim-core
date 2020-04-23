%module(directors="1") opensimSimbody
 // %module opensimSimbody

#pragma SWIG nowarn=822,451,503,516,325,401

%{
#include <SimTKsimbody.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ObjectGroup.h>

using namespace SimTK;
%}

%include "java_preliminaries.i";

/* This file is for creation/handling of arrays */
%include "arrays_java.i";

%typemap(javacode) SimTK::Vec<3> %{
    public static Vec3 createFromMat(double[] data) throws Exception {
        if (data.length != 3) {
            throw new Exception("data size != 3");
        }
        return new Vec3(data[0], data[1], data[2]);
    }
    public double[] getAsMat() {
        return new double[]{get(0), get(1), get(2)};
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

%include exception.i

%extend SimTK::RowVectorBase<double> {
    double get(int i) {
        if(i >= $self->nelt())
            throw std::out_of_range{"Index out of Range."};

        return $self->getElt(0, i);
    }

    void set(int i, double value) {
        if(i >= $self->nelt())
            throw std::out_of_range{"Index out of Range."};

        $self->updElt(0, i) = value;
    }
}

%typemap(javacode) SimTK::RowVectorBase<double> %{
    @Override
    public double[][] getAsMat() {
        double[][] ret = new double[1][size()];
        for (int i = 0; i < size(); ++i) { ret[0][i] = get(i); }
        return ret;
    }
%}

%typemap(javacode) SimTK::RowVector_<double> %{
    public static RowVector createFromMat(double[] data) {
        RowVector v = new RowVector(data.length);
        for (int i = 0; i < data.length; ++i) { v.set(i, data[i]); }
        return v;
    }
%}

%extend SimTK::VectorBase<double> {
     double get(int i) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         return $self->getElt(i, 0);
     }

     void set(int i, double value) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         $self->updElt(i, 0) = value;
     }
}

%typemap(javacode) SimTK::VectorBase<double> %{
    public double[][] getAsMat() {
        double[][] ret = new double[size()][1];
        for (int i = 0; i < size(); ++i) { ret[i][0] = get(i); }
        return ret;
    }
%}

%typemap(javacode) SimTK::Vector_<double> %{
    public static Vector createFromMat(double[] data) {
        Vector v = new Vector(data.length, 0.0);
        for (int i = 0; i < data.length; ++i) { v.set(i, data[i]); }
        return v;
    }
%}

%extend SimTK::RowVectorBase<SimTK::Vec3> {
     Vec3 get(int i) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         return $self->getElt(0, i);
     }

     void set(int i, Vec3 value) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         $self->updElt(0, i) = value;
     }
}

%typemap(javacode) SimTK::RowVectorBase<SimTK::Vec3> %{
    public double[][] getAsMat() {
        double[][] ret = new double[3][size()];
        for (int i = 0; i < size(); ++i) {
            ret[0][i] = get(i).get(0);
            ret[1][i] = get(i).get(1);
            ret[2][i] = get(i).get(2);
        }
        return ret;
    }
%}

%typemap(javacode) SimTK::RowVector_<SimTK::Vec3> %{
    public static RowVectorVec3 createFromMat(double[][] data) throws Exception {
        int numRows = data.length;
        int numCols = 0;
        if (numRows > 0) {
            numCols = data[0].length;
        }
        if (numRows != 3) {
            throw new Exception("Number of rows must be 3.");
        }
        RowVectorVec3 v = new RowVectorVec3(numCols);
        for (int i = 0; i < numCols; ++i) {
            v.set(i, new Vec3(data[0][i], data[1][i], data[2][i]));
        }
        return v;
    }
%}

%extend SimTK::VectorBase<SimTK::Vec3> {
     Vec3 get(int i) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         return $self->getElt(i, 0);
     }

     void set(int i, Vec3 value) {
         if(i >= $self->nelt())
             throw std::out_of_range{"Index out of Range."};

         $self->updElt(i, 0) = value;
     }
}

%typemap(javacode) SimTK::VectorBase<SimTK::Vec3> %{
    public double[][] getAsMat() {
        double[][] ret = new double[size()][3];
        for (int i = 0; i < size(); ++i) {
            ret[i][0] = get(i).get(0);
            ret[i][1] = get(i).get(1);
            ret[i][2] = get(i).get(2);
        }
        return ret;
    }
%}

%typemap(javacode) SimTK::Vector_<SimTK::Vec3> %{
    public static VectorVec3 createFromMat(double[][] data) throws Exception {
        int numRows = data.length;
        int numCols = 0;
        if (numRows > 0) {
            numCols = data[0].length;
        }
        if (numCols != 3) {
            throw new Exception("Number of columns must be 3.");
        }
        VectorVec3 v = new VectorVec3(numRows, new Vec3(0));
        for (int i = 0; i < numRows; ++i) {
            v.set(i, new Vec3(data[i][0], data[i][1], data[i][2]));
        }
        return v;
    }
%}

%typemap(javacode) SimTK::MatrixBase<double> %{
    public double[][] getAsMat() {
        double[][] ret = new double[nrow()][ncol()];
        for (int i = 0; i < nrow(); ++i) {
            for (int j = 0; j < ncol(); ++j) {
                ret[i][j] = getElt(i, j);
            }
        }
        return ret;
    }
%}

%typemap(javacode) SimTK::Matrix_<double> %{
    public static Matrix createFromMat(double[][] data) {
        int numRows = data.length;
        int numCols = 0;
        if (numRows > 0) {
            numCols = data[0].length;
        }
        Matrix matrix = new Matrix(numRows, numCols);
        for (int i = 0; i < numRows; ++i) {
            for (int j = 0; j < numCols; ++j) {
                matrix.set(i, j, data[i][j]);
            }
        }
        return matrix;
    }
%}

%include <Bindings/preliminaries.i>
%include <Bindings/simbody.i>
