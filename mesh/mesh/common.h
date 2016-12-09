#ifndef MESH_COMMON_H
#define MESH_COMMON_H

#include <adolc/adolc.h>
#include <Eigen/Dense>

namespace mesh {

using VectorXa = Eigen::Matrix<adouble, Eigen::Dynamic, 1>;
using MatrixXa = Eigen::Matrix<adouble, Eigen::Dynamic, Eigen::Dynamic>;

template<typename T>
using VectorX = Eigen::Matrix<T, Eigen::Dynamic, 1>;

} // namespace mesh

#endif // MESH_COMMON_H
