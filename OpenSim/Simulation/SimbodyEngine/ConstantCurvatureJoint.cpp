/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ConstantCurvatureJoint.cpp *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Keenon Werling                                                  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//==============================================================================
// INCLUDES
//==============================================================================
#include "ConstantCurvatureJoint.h"

#include "simbody/internal/MobilizedBody_Custom.h"

#include <SimTKcommon/Scalar.h>
#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/DecorativeGeometry.h>
#include <SimTKcommon/internal/Transform.h>

#include <OpenSim/Simulation/Model/Model.h>

using namespace SimTK;

#ifndef M_PI
#define M_PI 3.14159
#endif

Vec3 OpenSim::ConstantCurvatureJoint::clamp(const SimTK::Vec3& q)
{
  Vec3 pos = q;
  double bound = (M_PI / 2) - 0.01;
  if (pos(0) > bound) {
    std::cout << "WARNING! ConstantCurvatureJoint position outside of its supported range! X rotation reached position " << pos(0) << ", which is above upper bound " << bound << ". This will lead to unphysical behavior. Please adjust your model / simulation to avoid this state." << std::endl;
    assert(false);
    pos(0) = bound;
  }
  if (pos(0) < -bound) {
    std::cout << "WARNING! ConstantCurvatureJoint position outside of its supported range! X rotation reached position " << pos(0) << ", which is below lower bound " << -bound << ". This will lead to unphysical behavior. Please adjust your model / simulation to avoid this state." << std::endl;
    assert(false);
    pos(0) = -bound;
  }
  if (pos(1) > bound) {
    std::cout << "WARNING! ConstantCurvatureJoint position outside of its supported range! Z rotation reached position " << pos(1) << ", which is above upper bound " << bound << ". This will lead to unphysical behavior. Please adjust your model / simulation to avoid this state." << std::endl;
    assert(false);
    pos(1) = bound;
  }
  if (pos(1) < -bound) {
    std::cout << "WARNING! ConstantCurvatureJoint position outside of its supported range! Z rotation reached position " << pos(1) << ", which is below lower bound " << -bound << ". This will lead to unphysical behavior. Please adjust your model / simulation to avoid this state." << std::endl;
    assert(false);
    pos(1) = -bound;
  }
  if (pos(2) > bound) {
    std::cout << "WARNING! ConstantCurvatureJoint position outside of its supported range! Y rotation reached position " << pos(2) << ", which is above upper bound " << bound << ". This will lead to unphysical behavior. Please adjust your model / simulation to avoid this state." << std::endl;
    assert(false);
    pos(2) = bound;
  }
  if (pos(2) < -bound) {
    std::cout << "WARNING! ConstantCurvatureJoint position outside of its supported range! Y rotation reached position " << pos(2) << ", which is below lower bound " << -bound << ". This will lead to unphysical behavior. Please adjust your model / simulation to avoid this state." << std::endl;
    assert(false);
    pos(2) = -bound;
  }
  return pos;
}

Mat33 OpenSim::ConstantCurvatureJoint::eulerXZYToMatrix(const Vec3& _angle)
{
    // +-           -+   +-                                        -+
  // | r00 r01 r02 |   |  cy*cz           -sz      cz*sy          |
  // | r10 r11 r12 | = |  sx*sy+cx*cy*sz   cx*cz  -cy*sx+cx*sy*sz |
  // | r20 r21 r22 |   | -cx*sy+cy*sx*sz   cz*sx   cx*cy+sx*sy*sz |
  // +-           -+   +-                                        -+

  Mat33 ret;

  const double cx = cos(_angle(0));
  const double sx = sin(_angle(0));
  const double cz = cos(_angle(1));
  const double sz = sin(_angle(1));
  const double cy = cos(_angle(2));
  const double sy = sin(_angle(2));

  ret(0, 0) = cy * cz;
  ret(1, 0) = sx * sy + cx * cy * sz;
  ret(2, 0) = -cx * sy + cy * sx * sz;

  ret(0, 1) = -sz;
  ret(1, 1) = cx * cz;
  ret(2, 1) = cz * sx;

  ret(0, 2) = cz * sy;
  ret(1, 2) = -cy * sx + cx * sy * sz;
  ret(2, 2) = cx * cy + sx * sy * sz;

  return ret;
}

/// This gives the gradient of an XZY rotation matrix with respect to the
/// specific index (0, 1, or 2)
Mat33 OpenSim::ConstantCurvatureJoint::eulerXZYToMatrixGrad(const Vec3& _angle, int index)
{
  // Original
  // +-           -+   +-                                        -+
  // | r00 r01 r02 |   |  cy*cz           -sz      cz*sy          |
  // | r10 r11 r12 | = |  sx*sy+cx*cy*sz   cx*cz  -cy*sx+cx*sy*sz |
  // | r20 r21 r22 |   | -cx*sy+cy*sx*sz   cz*sx   cx*cy+sx*sy*sz |
  // +-           -+   +-                                        -+

  Mat33 ret;

  const double cx = cos(_angle(0));
  const double sx = sin(_angle(0));
  const double cz = cos(_angle(1));
  const double sz = sin(_angle(1));
  const double cy = cos(_angle(2));
  const double sy = sin(_angle(2));

  if (index == 0)
  {
    ret(0, 0) = 0;
    ret(1, 0) = (cx)*sy + (-sx) * cy * sz;
    ret(2, 0) = -(-sx) * sy + cy * (cx)*sz;

    ret(0, 1) = 0;
    ret(1, 1) = (-sx) * cz;
    ret(2, 1) = cz * (cx);

    ret(0, 2) = 0;
    ret(1, 2) = -cy * (cx) + (-sx) * sy * sz;
    ret(2, 2) = (-sx) * cy + (cx)*sy * sz;
  }
  else if (index == 1)
  {
    ret(0, 0) = cy * (-sz);
    ret(1, 0) = cx * cy * (cz);
    ret(2, 0) = cy * sx * (cz);

    ret(0, 1) = -(cz);
    ret(1, 1) = cx * (-sz);
    ret(2, 1) = (-sz) * sx;

    ret(0, 2) = (-sz) * sy;
    ret(1, 2) = cx * sy * (cz);
    ret(2, 2) = sx * sy * (cz);
  }
  else if (index == 2)
  {
    ret(0, 0) = (-sy) * cz;
    ret(1, 0) = sx * (cy) + cx * (-sy) * sz;
    ret(2, 0) = -cx * (cy) + (-sy) * sx * sz;

    ret(0, 1) = 0;
    ret(1, 1) = 0;
    ret(2, 1) = 0;

    ret(0, 2) = cz * (cy);
    ret(1, 2) = -(-sy) * sx + cx * (cy)*sz;
    ret(2, 2) = cx * (-sy) + sx * (cy)*sz;
  }
  else
  {
    assert(false);
  }

  return ret;
}

Mat63 OpenSim::ConstantCurvatureJoint::getEulerJacobian(const Vec3& q)
{
  Mat63 J;
  J.setToZero();

  // s_t q0 = _positions[0];
  const double q1 = q(1);
  const double q2 = q(2);

  // s_t c0 = cos(q0);
  const double c1 = cos(q1);
  const double c2 = cos(q2);

  // s_t s0 = sin(q0);
  const double s1 = sin(q1);
  const double s2 = sin(q2);

  //------------------------------------------------------------------------
  // S = [ c1*c2,  -s2,   0
  //         -s1,    0,   1
  //       c1*s2,   c2,   0
  //           0,    0,   0
  //           0,    0,   0
  //           0,    0,   0 ];
  //------------------------------------------------------------------------
  J(0,0) = c1 * c2;
  J(0,1) = -s2;
  J(1,0) = -s1;
  J(1,2) = 1;
  J(2,0) = c1*s2;
  J(2,1) = c2;

  return J;
}

Mat63 OpenSim::ConstantCurvatureJoint::getEulerJacobianDerivWrtPos(
  const Vec3& q,
    int index)
{
  assert(index < 3);

  Mat63 DJ_Dq;
  DJ_Dq.setToZero();

  const double q1 = q[1];
  const double q2 = q[2];

  // s_t c0 = cos(q0);
  const double c1 = cos(q1);
  const double c2 = cos(q2);

  // s_t s0 = sin(q0);
  const double s1 = sin(q1);
  const double s2 = sin(q2);

  //------------------------------------------------------------------------
  // S = [ c1*c2,  -s2,   0
  //         -s1,    0,   1
  //       c1*s2,   c2,   0
  //           0,    0,   0
  //           0,    0,   0
  //           0,    0,   0 ];
  //------------------------------------------------------------------------

  if (index == 0)
  {
    // DS/Dq0 = 0;
  }
  else if (index == 1)
  {
    // DS/Dq1 = [-s1*c2,  0,   0
    //              -c1,  0,   0
    //           -s1*s2,  0,   0
    //                0,  0,   0
    //                0,  0,   0
    //                0,  0,   0 ];

    DJ_Dq(0, 0) = -s1 * c2;
    DJ_Dq(1, 0) = -c1;
    DJ_Dq(2, 0) = -s1 * s2;
  }
  else if (index == 2)
  {
    // DS/Dq2 = [-c1*s2,  -c2,   0
    //                0,    0,   0
    //            c1*c2,  -s2,   0
    //                0,    0,   0
    //                0,    0,   0
    //                0,    0,   0 ];

    DJ_Dq(0, 0) = -c1 * s2;
    DJ_Dq(2, 0) = c1 * c2;

    DJ_Dq(0, 1) = -c2;
    DJ_Dq(2, 1) = -s2;
  }

  return DJ_Dq;
}

Mat63 OpenSim::ConstantCurvatureJoint::getConstantCurveJacobian(const Vec3& pos, double d)
{
  // 1. Do the euler rotation
  Mat33 rot = eulerXZYToMatrix(pos);
  Mat63 J = getEulerJacobian(pos);

  // Remember, this is X,*Z*,Y

  double cx = cos(pos(0));
  double sx = sin(pos(0));
  double cz = cos(pos(1));
  double sz = sin(pos(1));

  Vec3 linearAngle = Vec3(-sz, cx * cz, cz * sx);
  Mat33 dLinearAngle;
  dLinearAngle.col(0) = Vec3(0, -sx * cz, cz * cx);
  dLinearAngle.col(1) = Vec3(-cz, -cx * sz, -sz * sx);
  dLinearAngle.col(2) = Vec3(0,0,0);

  double sinTheta
      = sqrt(linearAngle(0) * linearAngle(0) + linearAngle(2) * linearAngle(2));

  if (sinTheta < 0.001 || sinTheta > 0.999)
  {
    // Near very vertical angles, don't worry about the bend, just approximate
    // with an euler joint

    // 2. Computing translation from vertical
    Vec3 translation = rot * Vec3(0, d, 0);

    J.col(0).updSubVec<3>(3) = 0.5 * SimTK::cross(J.col(0).getSubVec<3>(0), translation);
    J.col(1).updSubVec<3>(3) = 0.5 * SimTK::cross(J.col(1).getSubVec<3>(0), translation);
    J.col(2).updSubVec<3>(3) = 0.5 * SimTK::cross(J.col(2).getSubVec<3>(0), translation);
  }
  else
  {
    // Compute the bend as a function of the angle from vertical
    Vec3 dSinTheta;
    for (int i = 0; i < 3; i++)
    {
      dSinTheta(i) = (0.5
                      / sqrt(
                          linearAngle(0) * linearAngle(0)
                          + linearAngle(2) * linearAngle(2)))
                     * (2 * linearAngle(0) * dLinearAngle(0, i)
                        + 2 * linearAngle(2) * dLinearAngle(2, i));
    }

    double theta = asin(sinTheta);
    Vec3 dTheta
        = (1.0 / sqrt(1.0 - (sinTheta * sinTheta))) * dSinTheta;

    double r = (d / theta);
    Vec3 dR = (-d / (theta * theta)) * dTheta;

    double horizontalDist = r - r * cos(theta);

    Vec3 dHorizontalDist
        = dR + r * sin(theta) * dTheta - dR * cos(theta);
    Vec3 dVerticalDist = r * cos(theta) * dTheta + dR * sinTheta;

    Mat33 dTranslation;
    dTranslation.row(0)
        = (linearAngle(0) / sinTheta) * dHorizontalDist.transpose()
          + (horizontalDist / sinTheta) * dLinearAngle.row(0)
          + (horizontalDist * linearAngle(0)) * (-1.0 / (sinTheta * sinTheta))
                * dSinTheta.transpose();
    dTranslation.row(1) = dVerticalDist.transpose();
    dTranslation.row(2)
        = (linearAngle(2) / sinTheta) * dHorizontalDist.transpose()
          + (horizontalDist / sinTheta) * dLinearAngle.row(2)
          + (horizontalDist * linearAngle(2)) * (-1.0 / (sinTheta * sinTheta))
                * dSinTheta.transpose();

    J.col(0).updSubVec<3>(3)
        = rot.transpose() * dTranslation.col(0);
    J.col(1).updSubVec<3>(3)
        = rot.transpose() * dTranslation.col(1);
    J.col(2).updSubVec<3>(3)
        = rot.transpose() * dTranslation.col(2);
  }

  return J;
}

Mat63 OpenSim::ConstantCurvatureJoint::getConstantCurveJacobianDerivWrtPosition(const Vec3& pos, double d, int index)
{
  // 1. Do the euler rotation
  const Mat33 rot = eulerXZYToMatrix(pos);
  const Mat33 rot_dFirst = eulerXZYToMatrixGrad(pos, index);

  Mat63 J_dFirst = getEulerJacobianDerivWrtPos(pos, index);

  // Remember, this is X,*Z*,Y

  const double cx = cos(pos(0));
  const double sx = sin(pos(0));
  const double cz = cos(pos(1));
  const double sz = sin(pos(1));

  const Vec3 linearAngle(-sz, cx * cz, cz * sx);

  Mat33 dLinearAngle;
  dLinearAngle.col(0) = Vec3(0, -sx * cz, cz * cx);
  dLinearAngle.col(1) = Vec3(-cz, -cx * sz, -sz * sx);
  dLinearAngle.col(2).setToZero();

  Mat33 dLinearAngle_dFirst;
  dLinearAngle_dFirst.setToZero();

  Vec3 linearAngle_dFirst = Vec3(0,0,0);
  if (index == 0)
  {
    linearAngle_dFirst = Vec3(0, -sx * cz, cz * cx);
    dLinearAngle_dFirst.col(0) = Vec3(0, -cx * cz, cz * -sx);
    dLinearAngle_dFirst.col(1) = Vec3(0, sx * sz, -sz * cx);
    dLinearAngle_dFirst.col(2).setToZero();
  }
  else if (index == 1)
  {
    linearAngle_dFirst = Vec3(-cz, cx * -sz, -sz * sx);
    dLinearAngle_dFirst.col(0) = Vec3(0, -sx * -sz, -sz * cx);
    dLinearAngle_dFirst.col(1) = Vec3(sz, -cx * cz, -cz * sx);
    dLinearAngle_dFirst.col(2).setToZero();
  }

  const double sinTheta
      = sqrt(linearAngle(0) * linearAngle(0) + linearAngle(2) * linearAngle(2));

  if (sinTheta < 0.001 || sinTheta > 0.999)
  {
    // Near very vertical angles, don't worry about the bend, just approximate
    // with an euler joint
    const Mat63 J = getEulerJacobian(pos);

    // 2. Computing translation from vertical
    Vec3 translation = rot * Vec3(0,d,0);

    const Vec3 translation_dFirst
        = rot * SimTK::cross(J.col(index).getSubVec<3>(0), translation);

    J_dFirst.col(0).updSubVec<3>(3) = 0.5
          * (SimTK::cross(J_dFirst.col(0).getSubVec<3>(0), translation)
             + SimTK::cross(J.col(0).getSubVec<3>(0), translation_dFirst));
    J_dFirst.col(1).updSubVec<3>(3) = 0.5
          * (SimTK::cross(J_dFirst.col(1).getSubVec<3>(0), translation)
             + SimTK::cross(J.col(1).getSubVec<3>(0), translation_dFirst));
    J_dFirst.col(2).updSubVec<3>(3) = 0.5
          * (SimTK::cross(J_dFirst.col(2).getSubVec<3>(0), translation)
             + SimTK::cross(J.col(2).getSubVec<3>(0), translation_dFirst));
  }
  else
  {
    Vec3 dSinTheta;
    Vec3 dSinTheta_dFirst;
    for (int i = 0; i < 3; i++)
    {
      const double part1
          = (0.5
             / sqrt(
                 linearAngle(0) * linearAngle(0)
                 + linearAngle(2) * linearAngle(2)));
      const double part2
          = (2 * linearAngle(0) * dLinearAngle(0, i)
             + 2 * linearAngle(2) * dLinearAngle(2, i));
      dSinTheta(i) = part1 * part2;

      const double part1_dFirst
          = ((-0.25
              / pow(
                  linearAngle(0) * linearAngle(0)
                      + linearAngle(2) * linearAngle(2),
                  1.5))
             * (2 * linearAngle(0) * linearAngle_dFirst(0)
                + 2 * linearAngle(2) * linearAngle_dFirst(2)));
      const double part2_dFirst
          = (2 * linearAngle(0) * dLinearAngle_dFirst(0, i)
             + 2 * linearAngle_dFirst(0) * dLinearAngle(0, i)
             + 2 * linearAngle(2) * dLinearAngle_dFirst(2, i)
             + 2 * linearAngle_dFirst(2) * dLinearAngle(2, i));

      dSinTheta_dFirst(i) = part1_dFirst * part2 + part1 * part2_dFirst;
    }

    const double sinTheta_dFirst
        = (0.5
           / sqrt(
               linearAngle(0) * linearAngle(0)
               + linearAngle(2) * linearAngle(2)))
          * (2 * linearAngle(0) * linearAngle_dFirst(0)
             + 2 * linearAngle(2) * linearAngle_dFirst(2));

    // Compute the bend as a function of the angle from vertical
    const double theta = asin(sinTheta);
    const double theta_dFirst
        = (1.0 / sqrt(1.0 - (sinTheta * sinTheta))) * sinTheta_dFirst;
    (void)theta_dFirst;

    const Vec3 dTheta
        = (1.0 / sqrt(1.0 - (sinTheta * sinTheta))) * dSinTheta;
    const Vec3 dTheta_dFirst
        = (1.0 / pow(1.0 - (sinTheta * sinTheta), 1.5)) * sinTheta
              * sinTheta_dFirst * dSinTheta
          + (1.0 / sqrt(1.0 - (sinTheta * sinTheta))) * dSinTheta_dFirst;
    (void)dTheta_dFirst;

    const double r = (d / theta);
    const double r_dFirst = (-d / (theta * theta)) * theta_dFirst;
    (void)r_dFirst;

    const Vec3 dR = (-d / (theta * theta)) * dTheta;
    const Vec3 dR_dFirst
        = (2 * d / (theta * theta * theta)) * theta_dFirst * dTheta
          + (-d / (theta * theta)) * dTheta_dFirst;
    (void)dR_dFirst;

    const double horizontalDist = r - r * cos(theta);
    const double horizontalDist_dFirst
        = r_dFirst - (r_dFirst * cos(theta) - r * sin(theta) * theta_dFirst);
    (void)horizontalDist_dFirst;

    const Vec3 dHorizontalDist
        = dR + r * sin(theta) * dTheta - dR * cos(theta);
    const Vec3 dHorizontalDist_dFirst
        = dR_dFirst
          + (r_dFirst * sin(theta) * dTheta
             + r * cos(theta) * theta_dFirst * dTheta
             + r * sin(theta) * dTheta_dFirst)
          - (dR_dFirst * cos(theta) - dR * sin(theta) * theta_dFirst);
    (void)dHorizontalDist_dFirst;

    const Vec3 dVerticalDist
        = r * cos(theta) * dTheta + dR * sinTheta;
    const Vec3 dVerticalDist_dFirst
        = (r_dFirst * cos(theta) * dTheta
           - r * sin(theta) * theta_dFirst * dTheta
           + r * cos(theta) * dTheta_dFirst)
          + (dR_dFirst * sinTheta + dR * sinTheta_dFirst);
    (void)dVerticalDist_dFirst;

    Mat33 dTranslation;
    dTranslation.row(0)
        = (linearAngle(0) / sinTheta) * dHorizontalDist.transpose()
          + (horizontalDist / sinTheta) * dLinearAngle.row(0)
          + (horizontalDist * linearAngle(0)) * (-1.0 / (sinTheta * sinTheta))
                * dSinTheta.transpose();
    dTranslation.row(1) = dVerticalDist.transpose();
    dTranslation.row(2)
        = (linearAngle(2) / sinTheta) * dHorizontalDist.transpose()
          + (horizontalDist / sinTheta) * dLinearAngle.row(2)
          + (horizontalDist * linearAngle(2)) * (-1.0 / (sinTheta * sinTheta))
                * dSinTheta.transpose();

    Mat33 dTranslation_dFirst;
    dTranslation_dFirst.row(0)
        = ((linearAngle_dFirst(0) / sinTheta) * dHorizontalDist.transpose()
           - (linearAngle(0) / (sinTheta * sinTheta)) * sinTheta_dFirst
                 * dHorizontalDist.transpose()
           + (linearAngle(0) / sinTheta) * dHorizontalDist_dFirst.transpose())
          + ((horizontalDist_dFirst / sinTheta) * dLinearAngle.row(0)
             - (horizontalDist / (sinTheta * sinTheta)) * sinTheta_dFirst
                   * dLinearAngle.row(0)
             + (horizontalDist / sinTheta) * dLinearAngle_dFirst.row(0))
          + ((horizontalDist_dFirst * linearAngle(0)
              + horizontalDist * linearAngle_dFirst(0))
                 * (-1.0 / (sinTheta * sinTheta)) * dSinTheta.transpose()
             + (horizontalDist * linearAngle(0))
                   * (2.0 * sinTheta_dFirst / (sinTheta * sinTheta * sinTheta))
                   * dSinTheta.transpose()
             + (horizontalDist * linearAngle(0))
                   * (-1.0 / (sinTheta * sinTheta))
                   * dSinTheta_dFirst.transpose());
    dTranslation_dFirst.row(1) = dVerticalDist_dFirst.transpose();
    // This looks like a whole new mess, but it's actually identical to row(0),
    // except with the indices changed to 2
    dTranslation_dFirst.row(2)
        = ((linearAngle_dFirst(2) / sinTheta) * dHorizontalDist.transpose()
           - (linearAngle(2) / (sinTheta * sinTheta)) * sinTheta_dFirst
                 * dHorizontalDist.transpose()
           + (linearAngle(2) / sinTheta) * dHorizontalDist_dFirst.transpose())
          + ((horizontalDist_dFirst / sinTheta) * dLinearAngle.row(2)
             - (horizontalDist / (sinTheta * sinTheta)) * sinTheta_dFirst
                   * dLinearAngle.row(2)
             + (horizontalDist / sinTheta) * dLinearAngle_dFirst.row(2))
          + ((horizontalDist_dFirst * linearAngle(2)
              + horizontalDist * linearAngle_dFirst(2))
                 * (-1.0 / (sinTheta * sinTheta)) * dSinTheta.transpose()
             + (horizontalDist * linearAngle(2))
                   * (2.0 * sinTheta_dFirst / (sinTheta * sinTheta * sinTheta))
                   * dSinTheta.transpose()
             + (horizontalDist * linearAngle(2))
                   * (-1.0 / (sinTheta * sinTheta))
                   * dSinTheta_dFirst.transpose());

    J_dFirst.col(0).updSubVec<3>(3)
        = rot.transpose() * dTranslation_dFirst.col(0)
          + rot_dFirst.transpose() * dTranslation.col(0);
    J_dFirst.col(1).updSubVec<3>(3)
        = rot.transpose() * dTranslation_dFirst.col(1)
          + rot_dFirst.transpose() * dTranslation.col(1);
    J_dFirst.col(2).updSubVec<3>(3)
        = rot.transpose() * dTranslation_dFirst.col(2)
          + rot_dFirst.transpose() * dTranslation.col(2);
  }

  return J_dFirst;
}

Mat63 OpenSim::ConstantCurvatureJoint::getConstantCurveJacobianDerivWrtTime(const Vec3& pos, const Vec3& dPos, double d)
{
  Mat63 dJ;
  dJ.setToZero();
  for (int i = 0; i < 3; i++) {
    dJ += dPos(i) * getConstantCurveJacobianDerivWrtPosition(pos, d, i);
  }
  return dJ;
}

Transform OpenSim::ConstantCurvatureJoint::getTransform(Vec3 pos, double d)
{
  // 1. Do the euler rotation
  Mat33 rot = eulerXZYToMatrix(pos);

  // 2. Computing translation from vertical
  double cx = cos(pos(0));
  double sx = sin(pos(0));
  double cz = cos(pos(1));
  double sz = sin(pos(1));

  Vec3 linearAngle
      = Vec3(-sz, cx * cz, cz * sx); // rot.linear().col(1);

  double sinTheta
      = sqrt(linearAngle(0) * linearAngle(0) + linearAngle(2) * linearAngle(2));
  Vec3 translation(0, d, 0);
  if (sinTheta < 0.001 || sinTheta > 0.999)
  {
    // Near very vertical angles, don't worry about the bend, just approximate
    // with an euler joint
    translation = rot * translation;
  }
  else
  {
    // Compute the bend as a function of the angle from vertical
    double theta = asin(sinTheta);
    double r = (d / theta);
    double horizontalDist = r - r * cos(theta);
    double verticalDist = r * sinTheta;

    translation = Vec3(
        horizontalDist * (linearAngle(0) / sinTheta),
        verticalDist,
        horizontalDist * (linearAngle(2) / sinTheta));
  }

  return Transform(
    Rotation(rot, true), translation
  );
}

//==============================================================================
// IMPLEMENTATION
//==============================================================================
class ConstantCurvatureJointImpl
        : public MobilizedBody::Custom::Implementation {
public:
    ConstantCurvatureJointImpl(SimbodyMatterSubsystem& matter)
            : MobilizedBody::Custom::Implementation(matter, 3, 3, 0), neutralPos(0,0,0), length(1.0) {}

    MobilizedBody::Custom::Implementation* clone() const {
        return new ConstantCurvatureJointImpl(*this);
    };

    Transform calcMobilizerTransformFromQ(
            const State& s, int nq, const Real* q) const {
      return OpenSim::ConstantCurvatureJoint::getTransform(OpenSim::ConstantCurvatureJoint::clamp(Vec3::getAs(q) + neutralPos), length);
    }

    SpatialVec multiplyByHMatrix(const State& s, int nu, const Real* u) const {
      auto rawQ = getQ(s);
      Vec3 q = Vec3(rawQ(0), rawQ(1), rawQ(2));
      Mat63 J = OpenSim::ConstantCurvatureJoint::getConstantCurveJacobian(OpenSim::ConstantCurvatureJoint::clamp(q + neutralPos), length);
      Vec6 result = J * Vec3(u[0], u[1], u[2]);

      assert(!q.isNaN());
      assert(!J.isNaN());
      assert(!J.isInf());
      assert(!result.isNaN());

      return SpatialVec(result.getSubVec<3>(0), result.getSubVec<3>(3));
    }

    void multiplyByHTranspose(
            const State& s, const SpatialVec& F, int nu, Real* f) const {
      auto rawQ = getQ(s);
      Vec3 q = Vec3(rawQ(0), rawQ(1), rawQ(2));
      Mat63 J = OpenSim::ConstantCurvatureJoint::getConstantCurveJacobian(OpenSim::ConstantCurvatureJoint::clamp(q + neutralPos), length);

      Vec6 rawSpatial;
      rawSpatial.updSubVec<3>(0) = F[0];
      rawSpatial.updSubVec<3>(3) = F[1];

      Vec3 result = J.transpose() * rawSpatial;

      assert(!q.isNaN());
      assert(!J.isNaN());
      assert(!J.isInf());
      assert(!result.isNaN());

      Vec3::updAs(f) = result;
    }

    SpatialVec multiplyByHDotMatrix(
            const State& s, int nu, const Real* u) const {
        return SpatialVec(Vec3(0), Vec3(0));
      auto rawQ = getQ(s);
      auto rawQDot = getQDot(s);
      Vec3 q = Vec3(rawQ(0), rawQ(1), rawQ(2));
      Vec3 dq = Vec3(rawQDot(0), rawQDot(1), rawQDot(2));
      Mat63 dJ = OpenSim::ConstantCurvatureJoint::getConstantCurveJacobianDerivWrtTime(OpenSim::ConstantCurvatureJoint::clamp(q + neutralPos), dq, length);
      Vec6 result = dJ * Vec3(u[0], u[1], u[2]);

      assert(!q.isNaN());
      assert(!dq.isNaN());
      assert(!dJ.isNaN());
      assert(!dJ.isInf());
      assert(!result.isNaN());

      return SpatialVec(result.getSubVec<3>(0), result.getSubVec<3>(3));
    }

    void multiplyByHDotTranspose(
            const State& s, const SpatialVec& F, int nu, Real* f) const {
      auto rawQ = getQ(s);
      auto rawQDot = getQDot(s);
      Vec3 q = Vec3(rawQ(0), rawQ(1), rawQ(2));
      Vec3 dq = Vec3(rawQDot(0), rawQDot(1), rawQDot(2));
      Mat63 dJ = OpenSim::ConstantCurvatureJoint::getConstantCurveJacobianDerivWrtTime(OpenSim::ConstantCurvatureJoint::clamp(q + neutralPos), dq, length);

      Vec6 rawSpatial;
      rawSpatial.updSubVec<3>(0) = F[0];
      rawSpatial.updSubVec<3>(3) = F[1];

      Vec3 result = dJ.transpose() * rawSpatial;

      assert(!q.isNaN());
      assert(!dq.isNaN());
      assert(!dJ.isNaN());
      assert(!dJ.isInf());
      assert(!result.isNaN());

      Vec3::updAs(f) = result;
    }

    void setQToFitTransform(
            const State&, const Transform& X_FM, int nq, Real* q) const {
        assert(false);
        Vec3::updAs(q) = X_FM.p();
    }

    void setUToFitVelocity(
            const State&, const SpatialVec& V_FM, int nu, Real* u) const {
        assert(false);
        Vec3::updAs(u) = V_FM[1];
    }

    void setLength(double l) {
      length = l;
    }

    void setNeutralPos(Vec3 neutral) {
      neutralPos = neutral;
    }

protected:
  Vec3 neutralPos;
  double length;
};

class ConstantCurvatureJointWrapper : public MobilizedBody::Custom {
public:
    ConstantCurvatureJointWrapper(MobilizedBody& parent, const Transform& X_PF,
            const SimTK::Body& bodyInfo, const Transform& X_BM,
            Direction direction = Forward)
            : MobilizedBody::Custom(parent,
                      new ConstantCurvatureJointImpl(
                              parent.updMatterSubsystem()),
                      X_PF, bodyInfo, X_BM, direction){};

    ConstantCurvatureJointWrapper(MobilizedBody& parent, const SimTK::Body& bodyInfo,
            Direction direction = Forward)
            : MobilizedBody::Custom(parent,
                      new ConstantCurvatureJointImpl(
                              parent.updMatterSubsystem()),
                      bodyInfo, direction){};

    void setLength(double l) {
      static_cast<ConstantCurvatureJointImpl&>(updImplementation()).setLength(l);
    }

    void setNeutralPos(Vec3 neutral) {
      static_cast<ConstantCurvatureJointImpl&>(updImplementation()).setNeutralPos(neutral);
    }

    // Returns (Vec4,Vec3) where the Vec4 is a normalized quaternion.
    const Vec3 getDefaultQ() const { return Vec3(Zero); }
};

//==============================================================================
// STATICS
//==============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//==============================================================================
// CONSTRUCTORS AND DESTRUCTOR
//==============================================================================
ConstantCurvatureJoint::ConstantCurvatureJoint() : Super() {
    constructProperties();
}

ConstantCurvatureJoint::ConstantCurvatureJoint(const std::string& name,
        const PhysicalFrame& parent, const PhysicalFrame& child,
        const SimTK::Vec3& neutralAngleXZY, const double length)
        : Super(name, parent, child) {
    constructProperties();
    set_neutral_angle_x_z_y(neutralAngleXZY);
    set_length(length);
}

ConstantCurvatureJoint::ConstantCurvatureJoint(const std::string& name,
        const PhysicalFrame& parent, const SimTK::Vec3& locationInParent,
        const SimTK::Vec3& orientationInParent, const PhysicalFrame& child,
        const SimTK::Vec3& locationInChild,
        const SimTK::Vec3& orientationInChild,
        const SimTK::Vec3& neutralAngleXZY, const double length)
        : Super(name, parent, locationInParent, orientationInParent, child,
                  locationInChild, orientationInChild) {
    constructProperties();
    set_neutral_angle_x_z_y(neutralAngleXZY);
    set_length(length);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ConstantCurvatureJoint::constructProperties() {
    setAuthors("Keenon Werling");
    SimTK::Vec3 neutralAngleXZY(Zero);
    constructProperty_neutral_angle_x_z_y(neutralAngleXZY);
    constructProperty_length(1.0);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the ConstantCurvatureJoint's neutral angle, which it will bend to when
 * its DOFs are set to zero.
 *
 * @param Vec3 of radii: X, Y, Z in the parent frame.
 */
void ConstantCurvatureJoint::setNeutralAngleXZY(
        const SimTK::Vec3& neutralAngleXZY) {
    set_neutral_angle_x_z_y(neutralAngleXZY);
}

void ConstantCurvatureJoint::setLength(const double length) {
    set_length(length);
}

//==============================================================================
// SCALING
//==============================================================================
void ConstantCurvatureJoint::extendScale(
        const SimTK::State& s, const ScaleSet& scaleSet) {
    Super::extendScale(s, scaleSet);

    // Get scale factors (if an entry for the parent Frame's base Body exists).
    const Vec3& scaleFactors = getScaleFactors(scaleSet, getParentFrame());
    if (scaleFactors == ModelComponent::InvalidScaleFactors) return;

    // TODO: Need to scale transforms appropriately, given an arbitrary axis.
    upd_length() = get_length() * scaleFactors.get(1);
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void ConstantCurvatureJoint::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    // CREATE MOBILIZED BODY
    ConstantCurvatureJointWrapper mobod =
            createMobilizedBody<ConstantCurvatureJointWrapper>(system);
    mobod.setLength(get_length());
    mobod.setNeutralPos(get_neutral_angle_x_z_y());
}

void ConstantCurvatureJoint::extendInitStateFromProperties(
        SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);

    getCoordinate(ConstantCurvatureJoint::Coord::RotationX).setClamped(s, true);
    getCoordinate(ConstantCurvatureJoint::Coord::RotationY).setClamped(s, true);
    getCoordinate(ConstantCurvatureJoint::Coord::RotationZ).setClamped(s, true);

    /*
    double xangle = getCoordinate(ConstantCurvatureJoint::Coord::Rotation1X)
                            .getDefaultValue();
    double yangle = getCoordinate(ConstantCurvatureJoint::Coord::Rotation2Y)
                            .getDefaultValue();
    double zangle = getCoordinate(ConstantCurvatureJoint::Coord::Rotation3Z)
                            .getDefaultValue();
    SimTK::Vector v(3);
    v.set(0, xangle);
    v.set(1, yangle);
    v.set(2, zangle);

    getChildFrame().getMobilizedBody().setQFromVector(s, v);
    */
}

void ConstantCurvatureJoint::extendSetPropertiesFromState(
        const SimTK::State& state) {
    Super::extendSetPropertiesFromState(state);

    /*
    // Override default in case of quaternions.
    const SimbodyMatterSubsystem& matter = getModel().getMatterSubsystem();
    if (!matter.getUseEulerAngles(state)) {

        Rotation r = getChildFrame().getMobilizedBody().getBodyRotation(state);
        Vec3 angles = r.convertRotationToBodyFixedXYZ();

        updCoordinate(ConstantCurvatureJoint::Coord::Rotation1X)
                .setDefaultValue(angles[0]);
        updCoordinate(ConstantCurvatureJoint::Coord::Rotation2Y)
                .setDefaultValue(angles[1]);
        updCoordinate(ConstantCurvatureJoint::Coord::Rotation3Z)
                .setDefaultValue(angles[2]);
    }
    */
}

void ConstantCurvatureJoint::generateDecorations(bool fixed,
        const ModelDisplayHints& hints, const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometryArray) const {
    // invoke parent class method, this draws 2 Frames
    Super::generateDecorations(fixed, hints, state, geometryArray);
    // The next line is necessary to avoid ellipsoid below added twice
    // since this method is called with fixed={true, false}
    if (fixed) return;

    // Construct the visible line
    double length = get_length();
    Vec3 neutralPos = get_neutral_angle_x_z_y();
    auto rawQ = getChildFrame().getMobilizedBody().getQAsVector(state);
    Vec3 q = Vec3(rawQ(0), rawQ(1), rawQ(2));
    Vec3 pos = clamp(neutralPos + q);

    std::vector<Vec3> points;
    const int numPoints = 10;
    for (int i = 0; i <= numPoints; i++) {
      double percentage = ((double)i) / numPoints;

      Transform t = getTransform(clamp(pos * percentage), length * percentage);
      points.push_back(t.p());
    }

    for (int i = 1; i < (int)points.size(); i++) {
      Vec3 lastPoint = points[i-1];
      Vec3 thisPoint = points[i];
      SimTK::DecorativeLine line(lastPoint, thisPoint);
      const OpenSim::PhysicalFrame& frame = getParentFrame();
      line.setColor(Vec3(1.0, 0.0, 0.0));
      line.setBodyId(frame.getMobilizedBodyIndex());
      line.setTransform(frame.findTransformInBaseFrame());
      geometryArray.push_back(line);
    }
}
