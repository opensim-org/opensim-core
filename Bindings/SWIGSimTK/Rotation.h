#ifndef SimTK_SimTKCOMMON_ROTATION_H_
#define SimTK_SimTKCOMMON_ROTATION_H_

/* -------------------------------------------------------------------------- *
 *                       Simbody(tm): SimTKcommon                             *
 * -------------------------------------------------------------------------- *
 * This is part of the SimTK biosimulation toolkit originating from           *
 * Simbios, the NIH National Center for Physics-Based Simulation of           *
 * Biological Structures at Stanford, funded under the NIH Roadmap for        *
 * Medical Research, grant U54 GM072970. See https://simtk.org/home/simbody.  *
 *                                                                            *
 * Portions Copyright (c) 2005-2017 Stanford University and the Authors.      *
 * Authors: Paul Mitiguy, Michael Sherman                                     *
 * Contributors:                                                              *
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

//------------------------------------------------------------------------------

#include "SimTKcommon/SmallMatrix.h"
#include "SimTKcommon/internal/CoordinateAxis.h"
#include "SimTKcommon/internal/UnitVec.h"
#include "SimTKcommon/internal/Quaternion.h"

//------------------------------------------------------------------------------
#include <iosfwd>  // Forward declaration of iostream
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace SimTK {


enum BodyOrSpaceType { BodyRotationSequence=0, SpaceRotationSequence=1 };

//------------------------------------------------------------------------------
// Forward declarations
template <class P> class Rotation_;
template <class P> class InverseRotation_;

typedef Rotation_<Real>             Rotation;
typedef Rotation_<float>           fRotation;
typedef Rotation_<double>          dRotation;

typedef InverseRotation_<Real>      InverseRotation;
typedef InverseRotation_<float>    fInverseRotation;
typedef InverseRotation_<double>   dInverseRotation;

//------------------------------------------------------------------------------
/**
 * The Rotation class is a Mat33 that guarantees that the matrix is a legitimate
 * 3x3 array associated with the relative orientation of two right-handed,
 * orthogonal, unit vector bases. The Rotation class takes advantage of
 * known properties of orthogonal matrices. For example, multiplication by a
 * rotation matrix preserves a vector's length so unit vectors are still unit
 * vectors afterwards and don't need to be re-normalized.
 *
 * A rotation is an orthogonal matrix whose columns and rows are directions
 * (that is, unit vectors) that are mutually orthogonal. Furthermore, if the
 * columns (or rows) are labeled x,y,z it always holds that z = x X y (rather
 * than -(x X y)) ensuring that this is a right-handed rotation matrix and not
 * a reflection. This is equivalent to saying that the determinant of a rotation
 * matrix is 1, not -1.
 *
 * Suppose there is a vector v_F expressed in terms of the right-handed,
 * orthogonal unit vectors Fx, Fy, Fz and one would like to express v instead
 * as v_G, in terms of a right-handed, orthogonal unit vectors Gx, Gy, Gz. To
 * calculate it, we form a rotation matrix R_GF whose columns are the F unit
 * vectors re-expressed in G:
 * <pre>
 *             G F   (      |      |      )
 *      R_GF =  R  = ( Fx_G | Fy_G | Fz_G )
 *                   (      |      |      )
 * where
 *      Fx_G = ~( ~Fx*Gx, ~Fx*Gy, ~Fx*Gz ), etc.
 * </pre>
 * (~Fx*Gx means dot(Fx,Gx)). Note that we use "monogram" notation R_GF in
 * code to represent the more typographically demanding superscripted notation
 * for rotation matrices. Now we can re-express the vector v from frame F to
 * frame G via
 * <pre>
 *      v_G = R_GF * v_F.
 * </pre>
 * Because a rotation is orthogonal, its transpose is its inverse. Hence
 * R_FG = ~R_GF (where ~ is the SimTK "transpose" operator). This transpose
 * matrix can be used to expressed v_G in terms of Fx, Fy, Fz as
 * <pre>
 *      v_F = R_FG * v_G  or  v_F = ~R_GF * v_G
 * </pre>
 * In either direction, correct behavior can be obtained by using the
 * recommended notation and then matching up the frame labels (after
 * interpreting the "~" operator as reversing the labels).
 */
//------------------------------------------------------------------------------
template <class P=double> // templatized by precision
class Rotation_ : public Mat<3, 3, P> {
#ifndef SWIG
    typedef P               P;
    typedef Mat<2,2,P>      Mat22P;
    typedef Mat<3,2,P>      Mat32P;
    typedef Mat<3,3,P>      Mat33;
    typedef Mat<4,3,P>      Mat43P;
    typedef Mat<3,4,P>      Mat34P;
    typedef Vec<2,P>        Vec2;
    typedef Vec<3,P>        Vec3;
    typedef Vec<4,P>        Vec4;
    typedef UnitVec<P,1>    UnitVec3; // stride is 1 here, length is always 3
    typedef SymMat<3,P>     SymMat33;
#endif

public:
    // Default constructor and constructor-like methods
    Rotation_() : Mat33(1) {}
    Rotation_&  setRotationToIdentityMatrix()  { Mat33::operator=(P(1));  return *this; }
    Rotation_&  setRotationToNaN()             { Mat33::setToNaN();    return *this; }

    // Default copy constructor and assignment operator
    Rotation_( const Rotation_& R ) : Mat33(R)  {}
    Rotation_&  operator=( const Rotation_& R )  { Mat33::operator=( R.asMat33() );  return *this; }

    /// Constructor for right-handed rotation by an angle (in radians) about a coordinate axis.
    //@{
    Rotation_( P angle, const CoordinateAxis& axis )             { setRotationFromAngleAboutAxis( angle, axis ); }
#ifndef SWIG
    Rotation_( P angle, const CoordinateAxis::XCoordinateAxis )  { setRotationFromAngleAboutX( std::cos(angle), std::sin(angle) ); }
    Rotation_( P angle, const CoordinateAxis::YCoordinateAxis )  { setRotationFromAngleAboutY( std::cos(angle), std::sin(angle) ); }
    Rotation_( P angle, const CoordinateAxis::ZCoordinateAxis )  { setRotationFromAngleAboutZ( std::cos(angle), std::sin(angle) ); }
#endif
    //@}
    /// Set this Rotation_ object to a right-handed rotation by an angle (in radians) about a coordinate axis.
    //@{
    Rotation_&  setRotationFromAngleAboutAxis( P angle, const CoordinateAxis& axis )  { return axis.isXAxis() ? setRotationFromAngleAboutX(angle) : (axis.isYAxis() ? setRotationFromAngleAboutY(angle) : setRotationFromAngleAboutZ(angle) ); }
    Rotation_&  setRotationFromAngleAboutX( P angle )  { return setRotationFromAngleAboutX( std::cos(angle), std::sin(angle) ); }
    Rotation_&  setRotationFromAngleAboutY( P angle )  { return setRotationFromAngleAboutY( std::cos(angle), std::sin(angle) ); }
    Rotation_&  setRotationFromAngleAboutZ( P angle )  { return setRotationFromAngleAboutZ( std::cos(angle), std::sin(angle) ); }
    Rotation_&  setRotationFromAngleAboutX( P cosAngle, P sinAngle )  { Mat33& R = *this;  R[0][0] = 1;   R[0][1] = R[0][2] = R[1][0] = R[2][0] = 0;   R[1][1] = R[2][2] = cosAngle;  R[1][2] = -(R[2][1] = sinAngle);  return *this; }
    Rotation_&  setRotationFromAngleAboutY( P cosAngle, P sinAngle )  { Mat33& R = *this;  R[1][1] = 1;   R[0][1] = R[1][0] = R[1][2] = R[2][1] = 0;   R[0][0] = R[2][2] = cosAngle;  R[2][0] = -(R[0][2] = sinAngle);  return *this; }
    Rotation_&  setRotationFromAngleAboutZ( P cosAngle, P sinAngle )  { Mat33& R = *this;  R[2][2] = 1;   R[0][2] = R[1][2] = R[2][0] = R[2][1] = 0;   R[0][0] = R[1][1] = cosAngle;  R[0][1] = -(R[1][0] = sinAngle);  return *this; }
    //@}

    /// Constructor for right-handed rotation by an angle (in radians) about an arbitrary vector.
    //@{
#ifndef SWIG
    Rotation_( P angle, const UnitVec3& unitVector ) { setRotationFromAngleAboutUnitVector(angle,unitVector); }
#endif
    Rotation_( P angle, const Vec3& nonUnitVector )  { setRotationFromAngleAboutNonUnitVector(angle,nonUnitVector); }
    //@}
    /// Set this Rotation_ object to a right-handed rotation of an angle (in radians) about an arbitrary vector.
    //@{
    Rotation_&  setRotationFromAngleAboutNonUnitVector( P angle, const Vec3& nonUnitVector )  { return setRotationFromAngleAboutUnitVector( angle, UnitVec3(nonUnitVector) ); }
#ifndef SWIG
    /* SimTK_SimTKCOMMON_EXPORT */ Rotation_&  setRotationFromAngleAboutUnitVector( P angle, const UnitVec3& unitVector );
#endif
    //@}

    /// Constructor for two-angle, two-axes, Body-fixed or Space-fixed rotation sequences (angles are in radians)
    Rotation_( BodyOrSpaceType bodyOrSpace, P angle1, const CoordinateAxis& axis1, P angle2, const CoordinateAxis& axis2 )                                            { setRotationFromTwoAnglesTwoAxes(    bodyOrSpace,angle1,axis1,angle2,axis2); }
    /// Constructor for three-angle Body-fixed or Space-fixed rotation sequences (angles are in radians)
    Rotation_( BodyOrSpaceType bodyOrSpace, P angle1, const CoordinateAxis& axis1, P angle2, const CoordinateAxis& axis2, P angle3, const CoordinateAxis& axis3 )  { setRotationFromThreeAnglesThreeAxes(bodyOrSpace,angle1,axis1,angle2,axis2,angle3,axis3); }
    /// Set this Rotation_ object to a two-angle, two-axes, Body-fixed or Space-fixed rotation sequences (angles are in radians)
    /* SimTK_SimTKCOMMON_EXPORT */ Rotation_&  setRotationFromTwoAnglesTwoAxes(     BodyOrSpaceType bodyOrSpace, P angle1, const CoordinateAxis& axis1, P angle2, const CoordinateAxis& axis2 );
    /// Set this Rotation_ object to a three-angle Body-fixed or Space-fixed rotation sequences (angles are in radians)
    /* SimTK_SimTKCOMMON_EXPORT */ Rotation_&  setRotationFromThreeAnglesThreeAxes( BodyOrSpaceType bodyOrSpace, P angle1, const CoordinateAxis& axis1, P angle2, const CoordinateAxis& axis2, P angle3, const CoordinateAxis& axis3 );

    /// Set this Rotation_ to represent a rotation characterized by subsequent rotations of:
    /// +v[0] about the body frame's X axis,      followed by a rotation of
    /// +v[1] about the body frame's NEW Y axis.  See Kane, Spacecraft Dynamics, pg. 423, body-three: 1-2-3.
    void setRotationToBodyFixedXY( const Vec2& v)   { setRotationFromTwoAnglesTwoAxes(     BodyRotationSequence, v[0], XAxis, v[1], YAxis ); }
    /// Set this Rotation_ to represent a rotation characterized by subsequent rotations of:
    /// +v[0] about the body frame's X axis,      followed by a rotation of
    /// +v[1] about the body frame's NEW Y axis,  followed by a rotation of
    /// +v[2] about the body frame's NEW Z axis.  See Kane, Spacecraft Dynamics, pg. 423, body-three: 1-2-3.
    void setRotationToBodyFixedXYZ( const Vec3& v)  { setRotationFromThreeAnglesThreeAxes( BodyRotationSequence, v[0], XAxis, v[1], YAxis, v[2], ZAxis ); }

    /// Constructor for creating a rotation matrix from a quaternion.
    explicit Rotation_( const Quaternion_<double>& q )  { setRotationFromQuaternion(q); }
    /// Method for creating a rotation matrix from a quaternion.
    /* SimTK_SimTKCOMMON_EXPORT */ Rotation_&  setRotationFromQuaternion( const Quaternion_<double>& q );

    /// Construct a Rotation_ directly from a Mat33 (we trust that m is a valid Rotation_!)
    Rotation_( const Mat33& m, bool ) : Mat33(m) {}

    /// Constructs an (hopefully nearby) orthogonal rotation matrix from a generic Mat33.
    explicit Rotation_( const Mat33& m )  { setRotationFromApproximateMat33(m); }
    /// Set this Rotation_ object to an (hopefully nearby) orthogonal rotation matrix from a generic Mat33.
    /* SimTK_SimTKCOMMON_EXPORT */ Rotation_&  setRotationFromApproximateMat33( const Mat33& m );

    /// Calculate R_AB by knowing one of B's unit vector expressed in A.
    /// Note: The other vectors are perpendicular (but somewhat arbitrarily so).
    //@{
    Rotation_( const UnitVec3& uvec, const CoordinateAxis axis )  { setRotationFromOneAxis(uvec,axis); }
    /* SimTK_SimTKCOMMON_EXPORT */ Rotation_&  setRotationFromOneAxis( const UnitVec3& uvec, const CoordinateAxis axis );
    //@}

    /// Calculate R_AB by knowing one of B's unit vectors u1 (could be Bx, By, or Bz)
    /// expressed in A and a vector v (also expressed in A) that is approximately in
    /// the desired direction for a second one of B's unit vectors, u2 (!= u1).
    /// If v is not perpendicular to u1, no worries - we'll find a direction for u2
    /// that is perpendicular to u1 and comes closest to v. The third vector u3
    /// is +/- u1 X u2, as appropriate for a right-handed rotation matrix.
    //@{
    Rotation_( const UnitVec3& uveci, const CoordinateAxis& axisi, const Vec3& vecjApprox, const CoordinateAxis& axisjApprox )  { setRotationFromTwoAxes(uveci,axisi,vecjApprox,axisjApprox); }
    /* SimTK_SimTKCOMMON_EXPORT */ Rotation_&  setRotationFromTwoAxes( const UnitVec3& uveci, const CoordinateAxis& axisi, const Vec3& vecjApprox, const CoordinateAxis& axisjApprox );
    //@}

    // Converts rotation matrix to one or two or three orientation angles.
    // Note:  The result is most meaningful if the Rotation_ matrix is one that can be produced by such a sequence.
    // Use1:  someRotation.convertOneAxisRotationToOneAngle( XAxis );
    // Use2:  someRotation.convertTwoAxesRotationToTwoAngles(     SpaceRotationSequence, YAxis, ZAxis );
    // Use3:  someRotation.convertThreeAxesRotationToThreeAngles( SpaceRotationSequence, ZAxis, YAxis, XAxis );
    // Use4:  someRotation.convertRotationToAngleAxis();   Return: [angleInRadians, unitVectorX, unitVectorY, unitVectorZ].

    /// Converts rotation matrix to a single orientation angle.
    /// Note:  The result is most meaningful if the Rotation_ matrix is one that can be produced by such a sequence.
    /* SimTK_SimTKCOMMON_EXPORT */ P  convertOneAxisRotationToOneAngle( const CoordinateAxis& axis1 ) const;
    /// Converts rotation matrix to two orientation angles.
    /// Note:  The result is most meaningful if the Rotation_ matrix is one that can be produced by such a sequence.
    /* SimTK_SimTKCOMMON_EXPORT */ Vec2  convertTwoAxesRotationToTwoAngles(     BodyOrSpaceType bodyOrSpace, const CoordinateAxis& axis1, const CoordinateAxis& axis2 ) const;
    /// Converts rotation matrix to three orientation angles.
    /// Note:  The result is most meaningful if the Rotation_ matrix is one that can be produced by such a sequence.
    /* SimTK_SimTKCOMMON_EXPORT */ Vec3  convertThreeAxesRotationToThreeAngles( BodyOrSpaceType bodyOrSpace, const CoordinateAxis& axis1, const CoordinateAxis& axis2, const CoordinateAxis& axis3 ) const;
    /// Converts rotation matrix to a quaternion.

    /* SimTK_SimTKCOMMON_EXPORT */ Quaternion_<double> convertRotationToQuaternion() const;

    /// Converts rotation matrix to angle-axis form.
    Vec4  convertRotationToAngleAxis() const  { return convertRotationToQuaternion().convertQuaternionToAngleAxis(); }

    /// A convenient special case of convertTwoAxesRotationToTwoAngles().
    Vec2  convertRotationToBodyFixedXY() const   { return convertTwoAxesRotationToTwoAngles( BodyRotationSequence, XAxis, YAxis ); }
    /// A convenient special case of convertThreeAxesRotationToThreeAngles().
    Vec3  convertRotationToBodyFixedXYZ() const  { return convertThreeAxesRotationToThreeAngles( BodyRotationSequence, XAxis, YAxis, ZAxis ); }

    /// Perform an efficient transform of a symmetric matrix that must be re-expressed with
    /// a multiply from both left and right, such as an inertia matrix. Details: assuming
    /// this Rotation is R_AB, and given a symmetric dyadic matrix S_BB expressed in B,
    /// we can reexpress it in A using S_AA=R_AB*S_BB*R_BA. The matrix should be one that
    /// is formed as products of vectors expressed in A, such as inertia, gyration or
    /// covariance matrices. This can be done efficiently exploiting properties of R
    /// (orthogonal) and S (symmetric). Total cost is 57 flops.
#ifndef SWIG
    /* SimTK_SimTKCOMMON_EXPORT */ SymMat33 reexpressSymMat33(const SymMat33& S_BB) const;
#endif
    /// Return true if "this" Rotation is nearly identical to "R" within a specified pointing angle error
    //@{
    /* SimTK_SimTKCOMMON_EXPORT */ bool  isSameRotationToWithinAngle( const Rotation_& R, P okPointingAngleErrorRads ) const;
    bool isSameRotationToWithinAngleOfMachinePrecision( const Rotation_& R) const
    {   return isSameRotationToWithinAngle( R, NTraits<P>::getSignificant() ); }
    //@}
    P  getMaxAbsDifferenceInRotationElements( const Rotation_& R ) const {
        const Mat33& A = asMat33();  const Mat33& B = R.asMat33();  P maxDiff = 0;
        for( int i=0;  i<=2; i++ ) for( int j=0; j<=2; j++ )
        {   P absDiff = std::fabs(A[i][j] - B[i][j]);
            if( absDiff > maxDiff ) maxDiff = absDiff; }
        return maxDiff;
    }

    bool  areAllRotationElementsSameToEpsilon( const Rotation_& R, P epsilon ) const
    {   return getMaxAbsDifferenceInRotationElements(R) <= epsilon ; }
    bool  areAllRotationElementsSameToMachinePrecision( const Rotation_& R ) const
    {   return areAllRotationElementsSameToEpsilon( R, NTraits<P>::getSignificant() ); }

    /// Like copy constructor but for inverse rotation.  This allows implicit conversion from InverseRotation_ to Rotation_.
    inline Rotation_( const InverseRotation_<P>& );
    /// Like copy assignment but for inverse rotation.
    inline Rotation_& operator=( const InverseRotation_<P>& );

    /// Convert from Rotation_ to InverseRotation_ (no cost). Overrides base class invert().
    const InverseRotation_<P>&  invert() const  { return *reinterpret_cast<const InverseRotation_<P>*>(this); }
    /// Convert from Rotation_ to writable InverseRotation_ (no cost).
#ifndef SWIG
    InverseRotation_<P>&        updInvert()     { return *reinterpret_cast<InverseRotation_<P>*>(this); }
#endif
    /// Transpose, and transpose operators. For an orthogonal matrix like this one, transpose
    /// is the same thing as inversion. These override the base class transpose methods.
    //@{
    const InverseRotation_<P>&  transpose() const  { return invert(); }
    const InverseRotation_<P>&  operator~() const  { return invert(); }
#ifndef SWIG
    InverseRotation_<P>&        updTranspose()     { return updInvert(); }
#endif
    InverseRotation_<P>&        operator~()        { return updInvert(); }
    //@}

    /// In-place composition of Rotation matrices.
    //@{
    inline Rotation_&  operator*=( const Rotation_<P>& R );
    inline Rotation_&  operator/=( const Rotation_<P>& R );
    inline Rotation_&  operator*=( const InverseRotation_<P>& );
    inline Rotation_&  operator/=( const InverseRotation_<P>& );
    //@}

    /// Conversion from Rotation to its base class Mat33.
    /// Note: asMat33 is more efficient than toMat33() (no copy), but you have to know the internal layout.
    //@{
    const Mat<3, 3, P>&  asMat33() const  { return *static_cast<const Mat33*>(this); }
    Mat<3, 3, P>         toMat33() const  { return asMat33(); }
    //@}
#ifndef SWIG
    /// The column and row unit vector types do not necessarily have unit spacing.
    typedef  UnitVec<P,Mat33::RowSpacing>  ColType;
    typedef  UnitRow<P,Mat33::ColSpacing>  RowType;
    const RowType&  row( int i ) const         { return reinterpret_cast<const RowType&>(asMat33()[i]); }
    const ColType&  col( int j ) const         { return reinterpret_cast<const ColType&>(asMat33()(j)); }
    const ColType&  x() const                  { return col(0); }
    const ColType&  y() const                  { return col(1); }
    const ColType&  z() const                  { return col(2); }
    const RowType&  operator[]( int i ) const  { return row(i); }
    const ColType&  operator()( int j ) const  { return col(j); }

    /// Set the Rotation_ matrix directly - but you had better know what you are doing!
    //@{
    Rotation_&  setRotationFromMat33TrustMe( const Mat33& m )
    {   Mat33& R = *this; R=m;  return *this; }
    Rotation_&  setRotationColFromUnitVecTrustMe( int colj, const UnitVec3& uvecj )
    {   Mat33& R = *this; R(colj)=uvecj.asVec3(); return *this; }
    Rotation_&  setRotationFromUnitVecsTrustMe( const UnitVec3& colA, const UnitVec3& colB, const UnitVec3& colC )
    {   Mat33& R = *this; R(0)=colA.asVec3(); R(1)=colB.asVec3(); R(2)=colC.asVec3(); return *this; }
    //@}
#endif
//--------------------------- PAUL CONTINUE FROM HERE --------------------------
public:
//------------------------------------------------------------------------------

    // These are ad hoc routines that don't match the nice API Paul Mitiguy
    // implemented above.


    /// Given cosines and sines (in that order) of three angles, set this
    /// Rotation matrix to the body-fixed 1-2-3 sequence of those angles.
    /// Cost is 18 flops.
    void setRotationToBodyFixedXYZ(const Vec3& c, const Vec3& s) {
        Mat33& R = *this;
        const P s0s1=s[0]*s[1], s2c0=s[2]*c[0], c0c2=c[0]*c[2], nc1= -c[1];

        R = Mat33(     c[1]*c[2]      ,         s[2]*nc1       ,    s[1]  ,
                    s2c0 + s0s1*c[2]   ,     c0c2 - s0s1*s[2]   , s[0]*nc1 ,
                 s[0]*s[2] - s[1]*c0c2 ,  s[0]*c[2] + s[1]*s2c0 , c[0]*c[1] );
    }


    /// Given Euler angles forming a body-fixed 3-2-1 sequence, and the relative
    /// angular velocity vector of B in the parent frame, *BUT EXPRESSED IN
    /// THE BODY FRAME*, return the Euler angle
    /// derivatives. You are dead if q[1] gets near 90 degrees!
    /// See Kane's Spacecraft Dynamics, page 428, body-three: 3-2-1.
    static Vec3 convertAngVelToBodyFixed321Dot(const Vec3& q, const Vec3& w_PB_B) {
        const P s1 = std::sin(q[1]), c1 = std::cos(q[1]);
        const P s2 = std::sin(q[2]), c2 = std::cos(q[2]);
        const P ooc1 = P(1)/c1;
        const P s2oc1 = s2*ooc1, c2oc1 = c2*ooc1;

        const Mat33 E( 0,    s2oc1  ,  c2oc1  ,
                        0,      c2   ,   -s2   ,
                        1,  s1*s2oc1 , s1*c2oc1 );
        return E * w_PB_B;
    }

    /// Inverse of the above routine. Returned angular velocity is B in P,
    /// expressed in *B*: w_PB_B.
    static Vec3 convertBodyFixed321DotToAngVel(const Vec3& q, const Vec3& qd) {
        const P s1 = std::sin(q[1]), c1 = std::cos(q[1]);
        const P s2 = std::sin(q[2]), c2 = std::cos(q[2]);

        const Mat33 Einv(  -s1  ,  0  ,  1 ,
                           c1*s2 ,  c2 ,  0 ,
                           c1*c2 , -s2 ,  0 );
        return Einv*qd;
    }

    // TODO: sherm: is this right? Warning: everything is measured in the
    // *PARENT* frame, but angular velocities and accelerations are
    // expressed in the *BODY* frame.
    // TODO: this is not an efficient way to do this computation.
    static Vec3 convertAngVelDotToBodyFixed321DotDot
        (const Vec3& q, const Vec3& w_PB_B, const Vec3& wdot_PB_B)
    {
        const P s1 = std::sin(q[1]), c1 = std::cos(q[1]);
        const P s2 = std::sin(q[2]), c2 = std::cos(q[2]);
        const P ooc1  = 1/c1;
        const P s2oc1 = s2*ooc1, c2oc1 = c2*ooc1, s1oc1 = s1*ooc1;

        const Mat33 E( 0 ,   s2oc1  ,  c2oc1  ,
                       0 ,     c2   ,   -s2   ,
                       1 , s1*s2oc1 , s1*c2oc1 );
        const Vec3 qdot = E * w_PB_B;

        const P t = qdot[1]*s1oc1;
        const P a = t*s2oc1 + qdot[2]*c2oc1; // d/dt s2oc1
        const P b = t*c2oc1 - qdot[2]*s2oc1; // d/dt c2oc1

        const Mat33 Edot( 0 ,       a           ,         b         ,
                          0 ,   -qdot[2]*s2     ,    -qdot[2]*c2    ,
                          0 , s1*a + qdot[1]*s2 , s1*b + qdot[1]*c2 );

        return E*wdot_PB_B + Edot*w_PB_B;
    }

    /// Given Euler angles q forming a body-fixed X-Y-Z sequence return the
    /// block N_B of the system N matrix such that qdot=N_B(q)*w_PB_B where
    /// w_PB_B is the angular velocity of B in P EXPRESSED IN *B*!!! Note that
    /// N_B=N_P*R_PB. This matrix will be singular if Y (q[1]) gets near 90
    /// degrees!
    /// @note This version is very expensive because it has to calculate sines
    ///       and cosines. If you already have those, use the alternate form
    ///       of this method.
    ///
    /// Cost: about 100 flops for sin/cos plus 12 to calculate N_B.
    /// @see Kane's Spacecraft Dynamics, page 427, body-three: 1-2-3.
    static Mat33 calcNForBodyXYZInBodyFrame(const Vec3& q) {
        // Note: q[0] is not referenced so we won't waste time calculating
        // its cosine and sine here.
        return calcNForBodyXYZInBodyFrame
           (Vec3(0, std::cos(q[1]), std::cos(q[2])),
            Vec3(0, std::sin(q[1]), std::sin(q[2])));
    }

    /// This faster version of calcNForBodyXYZInBodyFrame() assumes you have
    /// already calculated the cosine and sine of the three q's. Note that we
    /// only look at the cosines and sines of q[1] and q[2]; q[0] does not
    /// matter so you don't have to fill in the 0'th element of cq and sq.
    /// Cost is one divide plus 6 flops, say 12 flops.
    static Mat33 calcNForBodyXYZInBodyFrame(const Vec3& cq, const Vec3& sq) {
        const P s1 = sq[1], c1 = cq[1];
        const P s2 = sq[2], c2 = cq[2];
        const P ooc1  = 1/c1;
        const P s2oc1 = s2*ooc1, c2oc1 = c2*ooc1;

        return Mat33(    c2oc1  , -s2oc1  , 0,
                            s2   ,    c2   , 0,
                       -s1*c2oc1 , s1*s2oc1, 1 );
    }

    /// Given Euler angles q forming a body-fixed X-Y-Z (123) sequence return
    /// the block N_P of the system N matrix such that qdot=N_P(q)*w_PB where
    /// w_PB is the angular velocity of B in P expressed in P (not the
    /// convention that Kane uses, where angular velocities are expressed in
    /// the outboard body B). Note that N_P = N_B*~R_PB. This matrix will be
    /// singular if Y (q[1]) gets near 90 degrees!
    ///
    /// @note This version is very expensive because it has to calculate sines
    ///       and cosines. If you already have those, use the alternate form
    ///       of this method.
    ///
    /// Cost: about 100 flops for sin/cos plus 12 to calculate N_P.
    static Mat33 calcNForBodyXYZInParentFrame(const Vec3& q) {
        // Note: q[2] is not referenced so we won't waste time calculating
        // its cosine and sine here.
        return calcNForBodyXYZInParentFrame
           (Vec3(std::cos(q[0]), std::cos(q[1]), 0),
            Vec3(std::sin(q[0]), std::sin(q[1]), 0));
    }

    /// This faster version of calcNForBodyXYZInParentFrame() assumes you have
    /// already calculated the cosine and sine of the three q's. Note that we
    /// only look at the cosines and sines of q[0] and q[1]; q[2] does not
    /// matter so you don't have to fill in the 3rd element of cq and sq.
    /// Cost is one divide plus 6 flops, say 12 flops.
    /// @see Paul Mitiguy
    static Mat33 calcNForBodyXYZInParentFrame(const Vec3& cq, const Vec3& sq) {
        const P s0 = sq[0], c0 = cq[0];
        const P s1 = sq[1], c1 = cq[1];
        const P ooc1  = 1/c1;
        const P s0oc1 = s0*ooc1, c0oc1 = c0*ooc1;

        return Mat33( 1 , s1*s0oc1 , -s1*c0oc1,
                       0 ,    c0    ,    s0,
                       0 ,  -s0oc1  ,  c0oc1 );
    }

    /// This is the fastest way to form the product qdot=N_P*w_PB for a
    /// body-fixed XYZ sequence where angular velocity of child in parent is
    /// expected to be expressed in the parent. Here we assume you have
    /// previously calculated sincos(qx), sincos(qy), and 1/cos(qy).
    /// Cost is 10 flops, faster even than the 15 it would take if you had saved
    /// N_P and then formed the N_P*w_PB product explicitly.
    static Vec3 multiplyByBodyXYZ_N_P(const Vec2& cosxy,
                                       const Vec2& sinxy,
                                       P        oocosy,
                                       const Vec3& w_PB)
    {
        const P s0 = sinxy[0], c0 = cosxy[0];
        const P s1 = sinxy[1];
        const P w0 = w_PB[0], w1 = w_PB[1], w2 = w_PB[2];

        const P t = (s0*w1-c0*w2)*oocosy;
        return Vec3( w0 + t*s1, c0*w1 + s0*w2, -t ); // qdot
    }

    /// This is the fastest way to form the product v_P=~N_P*q=~(~q*N_P);
    /// see the untransposed method multiplyByBodyXYZ_N_P() for information.
    /// Cost is 9 flops.
    static Vec3 multiplyByBodyXYZ_NT_P(const Vec2& cosxy,
                                        const Vec2& sinxy,
                                        P        oocosy,
                                        const Vec3& q)
    {
        const P s0 = sinxy[0], c0 = cosxy[0];
        const P s1 = sinxy[1];
        const P q0 = q[0], q1 = q[1], q2 = q[2];

        const P t = (q0*s1-q2) * oocosy;
        return Vec3( q0, c0*q1 + t*s0, s0*q1 - t*c0 ); // v_P
    }

    /// Calculate first time derivative qdot of body-fixed XYZ Euler angles q
    /// given sines and cosines of the Euler angles and the angular velocity
    /// w_PB of child B in parent P, expressed in P. Cost is 10 flops.
    ///
    /// Theory: calculate qdot=N_P(q)*w_PB using multiplyByBodyXYZ_N_P().
    /// @see multiplyByBodyXYZ_N_P()
    static Vec3 convertAngVelInParentToBodyXYZDot
       (const Vec2& cosxy,  //< cos(qx), cos(qy)
        const Vec2& sinxy,  //< sin(qx), sin(qy)
        P        oocosy, //< 1/cos(qy)
        const Vec3& w_PB)   //< angular velocity of B in P, exp. in P
    {
        return multiplyByBodyXYZ_N_P(cosxy,sinxy,oocosy,w_PB);
    }

    /// Calculate second time derivative qdotdot of body-fixed XYZ Euler
    /// angles q given sines and cosines of the Euler angles, the first
    /// derivative qdot and the angular acceleration b_PB of child B in
    /// parent P, expressed in P. Cost is 22 flops.
    ///
    /// Theory: we have qdot=N_P*w_PB, which we differentiate in P to
    /// get qdotdot=N_P*b_PB + NDot_P*w_PB. Note that NDot_P=NDot_P(q,qdot)
    /// and w_PB=NInv_P*qdot (because N_P is invertible). We can then rewrite
    /// qdotdot=N_P*b_PB + NDot_P*(NInv_P*qdot) which can be calculated very
    /// efficiently. The second term is just an acceleration remainder term
    /// quadratic in qdot.
    static Vec3 convertAngAccInParentToBodyXYZDotDot
       (const Vec2& cosxy,  //< cos(qx), cos(qy)
        const Vec2& sinxy,  //< sin(qx), sin(qy)
        P        oocosy, //< 1/cos(qy)
        const Vec3& qdot,   //< previously calculated BodyXYZDot
        const Vec3& b_PB)   //< angular acceleration, a.k.a. wdot_PB
    {
        const P s1 = sinxy[1], c1 = cosxy[1];
        const P q0 = qdot[0], q1 = qdot[1], q2 = qdot[2];

        // 10 flops
        const Vec3 Nb = multiplyByBodyXYZ_N_P(cosxy,sinxy,oocosy,b_PB);

        const P q1oc1 = q1*oocosy;
        const Vec3 NDotw((q0*s1-q2)*q1oc1,     //   NDot_P*w_PB
                           q0*q2*c1,            // = NDot_P*(NInv_P*qdot)
                          (q2*s1-q0)*q1oc1 );   // (9 flops)

        return Nb + NDotw; // 3 flops
    }


    /// Fastest way to form the product w_PB=NInv_P*qdot. This is never
    /// singular. Cost is 9 flops.
    static Vec3 multiplyByBodyXYZ_NInv_P(const Vec2& cosxy,
                                          const Vec2& sinxy,
                                          const Vec3& qdot)
    {
        const P s0 = sinxy[0], c0 = cosxy[0];
        const P s1 = sinxy[1], c1 = cosxy[1];
        const P q0 = qdot[0], q1 = qdot[1], q2 = qdot[2];
        const P c1q2 = c1*q2;

        return Vec3( q0 + s1*q2,           // w_PB
                      c0*q1 - s0*c1q2,
                      s0*q1 + c0*c1q2 );
    }

    /// Fastest way to form the product q=~NInv_P*v_P=~(~v_P*NInv_P).
    /// This is never singular. Cost is 10 flops.
    static Vec3 multiplyByBodyXYZ_NInvT_P(const Vec2& cosxy,
                                           const Vec2& sinxy,
                                           const Vec3& v_P)
    {
        const P s0 = sinxy[0], c0 = cosxy[0];
        const P s1 = sinxy[1], c1 = cosxy[1];
        const P w0 = v_P[0], w1 = v_P[1], w2 = v_P[2];

        return Vec3( w0,                           // qdot-like
                      c0*w1 + s0*w2,
                      s1*w0 - s0*c1*w1 + c0*c1*w2);
    }

    /// Given Euler angles forming a body-fixed X-Y-Z (123) sequence q, and
    /// their time derivatives qdot, return the block of the NDot matrix such
    /// that qdotdot=N(q)*wdot + NDot(q,u)*w where w is the angular velocity
    /// of B in P EXPRESSED IN *B*!!! This matrix will be singular if Y (q[1])
    /// gets near 90 degrees! See calcNForBodyXYZInBodyFrame() for the matrix
    /// we're differentiating here.
    /// @note This version is very expensive because it has to calculate sines
    ///       and cosines. If you already have those, use the alternate form
    ///       of this method.
    static Mat33 calcNDotForBodyXYZInBodyFrame
       (const Vec3& q, const Vec3& qdot) {
        // Note: q[0] is not referenced so we won't waste time calculating
        // its cosine and sine here.
        return calcNDotForBodyXYZInBodyFrame
           (Vec3(0, std::cos(q[1]), std::cos(q[2])),
            Vec3(0, std::sin(q[1]), std::sin(q[2])),
            qdot);
    }

    /// This faster version of calcNDotForBodyXYZInBodyFrame() assumes you
    /// have already calculated the cosine and sine of the three q's. Note
    /// that we only look at the cosines and sines of q[1] and q[2]; q[0] does
    /// not matter so you don't have to fill in the 0'th element of cq and sq.
    /// Cost is one divide plus 21 flops.
    static Mat33 calcNDotForBodyXYZInBodyFrame
       (const Vec3& cq, const Vec3& sq, const Vec3& qdot)
    {
        const P s1 = sq[1], c1 = cq[1];
        const P s2 = sq[2], c2 = cq[2];
        const P ooc1  = 1/c1;
        const P s2oc1 = s2*ooc1, c2oc1 = c2*ooc1;

        const P t = qdot[1]*s1*ooc1;
        const P a = t*s2oc1 + qdot[2]*c2oc1; // d/dt s2oc1
        const P b = t*c2oc1 - qdot[2]*s2oc1; // d/dt c2oc1

        return Mat33(       b             ,        -a         , 0,
                          qdot[2]*c2       ,    -qdot[2]*s2    , 0,
                      -(s1*b + qdot[1]*c2) , s1*a + qdot[1]*s2 , 0 );
    }

    /// Given Euler angles forming a body-fixed X-Y-Z (123) sequence q, and
    /// their time derivatives qdot, return the block of the NDot matrix such
    /// that qdotdot=N(q)*wdot + NDot(q,u)*w where w is the angular velocity of
    /// B in P expressed in P. This matrix will be singular if Y (q[1]) gets
    /// near 90 degrees! See calcNForBodyXYZInParentFrame() for the matrix
    /// we're differentiating here.
    /// @note This version is very expensive because it has to calculate sines
    ///       and cosines. If you already have those, use the alternate form
    ///       of this method.
    static Mat33 calcNDotForBodyXYZInParentFrame
       (const Vec3& q, const Vec3& qdot) {
        // Note: q[2] is not referenced so we won't waste time calculating
        // its cosine and sine here.
        const P cy = std::cos(q[1]); // cos(y)
        return calcNDotForBodyXYZInParentFrame
           (Vec2(std::cos(q[0]), cy),
            Vec2(std::sin(q[0]), std::sin(q[1])),
            1/cy, qdot);
    }

    /// This faster version of calcNDotForBodyXYZInParentFrame() assumes you
    /// have already calculated the cosine and sine of the three q's. Note that
    /// we only look at the cosines and sines of q[0] and q[1].
    /// Cost is 21 flops.
    static Mat33 calcNDotForBodyXYZInParentFrame
       (const Vec2& cq, const Vec2& sq, P ooc1, const Vec3& qdot) {
        const P s0 = sq[0], c0 = cq[0];
        const P s1 = sq[1], c1 = cq[1];
        const P s0oc1 = s0*ooc1, c0oc1 = c0*ooc1;

        const P t = qdot[1]*s1*ooc1;
        const P a = t*s0oc1 + qdot[0]*c0oc1; // d/dt s0oc1
        const P b = t*c0oc1 - qdot[0]*s0oc1; // d/dt c0oc1

        return Mat33( 0,  s1*a + qdot[1]*s0, -(s1*b + qdot[1]*c0),
                       0,    -qdot[0]*s0    ,     qdot[0]*c0      ,
                       0,        -a         ,         b            );
    }

    /// Inverse of routine calcNForBodyXYZInBodyFrame(). Return the inverse
    /// NInv_B of the N_B block computed above, such that w_PB_B=NInv_B(q)*qdot
    /// where w_PB_B is the angular velocity of B in P EXPRESSED IN *B*!!!
    /// (Kane's convention.) Note that NInv_B=~R_PB*NInv_P. This matrix is
    /// never singular.
    /// @note This version is very expensive because it has to calculate sines
    ///       and cosines. If you already have those, use the alternate form
    ///       of this method.
    static Mat33 calcNInvForBodyXYZInBodyFrame(const Vec3& q) {
        // Note: q[0] is not referenced so we won't waste time calculating
        // its cosine and sine here.
        return calcNInvForBodyXYZInBodyFrame
           (Vec3(0, std::cos(q[1]), std::cos(q[2])),
            Vec3(0, std::sin(q[1]), std::sin(q[2])));
    }

    /// This faster version of calcNInvForBodyXYZInBodyFrame() assumes you have
    /// already calculated the cosine and sine of the three q's. Note that we
    /// only look at the cosines and sines of q[1] and q[2]; q[0] does not
    /// matter so you don't have to fill in the 0'th element of cq and sq.
    /// Cost is 3 flops.
    static Mat33 calcNInvForBodyXYZInBodyFrame
       (const Vec3& cq, const Vec3& sq) {
        const P s1 = sq[1], c1 = cq[1];
        const P s2 = sq[2], c2 = cq[2];

        return Mat33( c1*c2 ,  s2 , 0 ,
                      -c1*s2 ,  c2 , 0 ,
                        s1   ,  0  , 1 );
    }

    /// Inverse of the above routine. Return the inverse NInv_P of the N_P
    /// block computed above, such that w_PB=NInv_P(q)*qdot where w_PB is the
    /// angular velocity of B in P (expressed in P). Note that
    /// NInv_P=R_PB*NInv_B. This matrix is never singular.
    /// @note This version is very expensive because it has to calculate sines
    ///       and cosines. If you already have those, use the alternate form
    ///       of this method.
    static Mat33 calcNInvForBodyXYZInParentFrame(const Vec3& q) {
        // Note: q[0] is not referenced so we won't waste time calculating
        // its cosine and sine here.
        return calcNInvForBodyXYZInParentFrame
           (Vec3(std::cos(q[0]), std::cos(q[1]), 0),
            Vec3(std::sin(q[0]), std::sin(q[1]), 0));
    }

    /// This faster version of calcNInvForBodyXYZInParentFrame() assumes you
    /// have already calculated the cosine and sine of the three q's. Note that
    /// we only look at the cosines and sines of q[0] and q[1]; q[2] does not
    /// matter so you don't have to fill in the 3rd element of cq and sq.
    /// Cost is 3 flops.
    static Mat33 calcNInvForBodyXYZInParentFrame
       (const Vec3& cq, const Vec3& sq) {
        const P s0 = sq[0], c0 = cq[0];
        const P s1 = sq[1], c1 = cq[1];

        return Mat33( 1 ,  0  ,   s1   ,
                       0 ,  c0 , -s0*c1 ,
                       0 ,  s0 ,  c0*c1 );
    }

    /// Given Euler angles forming a body-fixed X-Y-Z (123) sequence, and the
    /// relative angular velocity vector w_PB_B of B in the parent frame,
    /// <em>BUT EXPRESSED IN THE BODY FRAME</em>, return the Euler angle
    /// derivatives. You are dead if q[1] gets near 90 degrees!
    /// @note This version is very expensive because it has to calculate sines
    ///       and cosines. If you already have those, use the alternate form
    ///       of this method.
    /// @see Kane's Spacecraft Dynamics, page 427, body-three: 1-2-3.
    static Vec3 convertAngVelInBodyFrameToBodyXYZDot
       (const Vec3& q, const Vec3& w_PB_B) {
        return convertAngVelInBodyFrameToBodyXYZDot
           (Vec3(0, std::cos(q[1]), std::cos(q[2])),
            Vec3(0, std::sin(q[1]), std::sin(q[2])),
            w_PB_B);
    }

    /// This faster version of convertAngVelInBodyFrameToBodyXYZDot() assumes
    /// you have already calculated the cosine and sine of the three q's. Note
    /// that we only look at the cosines and sines of q[1] and q[2]; q[0] does
    /// not matter so you don't have to fill in the 0'th element of cq and sq.
    /// Cost is XXX.
    //TODO: reimplement
    static Vec3 convertAngVelInBodyFrameToBodyXYZDot
       (const Vec3& cq, const Vec3& sq, const Vec3& w_PB_B)
    {   return calcNForBodyXYZInBodyFrame(cq,sq)*w_PB_B; }

    /// Inverse of the above routine. Returned angular velocity is B in P,
    /// expressed in *B*: w_PB_B.
    /// @note This version is very expensive because it has to calculate sines
    ///       and cosines. If you already have those, use the alternate form
    ///       of this method.
    static Vec3 convertBodyXYZDotToAngVelInBodyFrame
       (const Vec3& q, const Vec3& qdot) {
           return convertBodyXYZDotToAngVelInBodyFrame
                       (Vec3(0, std::cos(q[1]), std::cos(q[2])),
                        Vec3(0, std::sin(q[1]), std::sin(q[2])),
                        qdot);
    }

    /// This faster version of convertBodyXYZDotToAngVelInBodyFrame() assumes
    /// you have already calculated the cosine and sine of the three q's. Note
    /// that we only look at the cosines and sines of q[1] and q[2]; q[0] does
    /// not matter so you don't have to fill in the 0'th element of cq and sq.
    /// Cost is XXX flops.
    // TODO: reimplement
    static Vec3 convertBodyXYZDotToAngVelInBodyFrame
       (const Vec3& cq, const Vec3& sq, const Vec3& qdot)
    {   return calcNInvForBodyXYZInBodyFrame(cq,sq)*qdot; }

    /// TODO: sherm: is this right? Warning: everything is measured in the
    /// *PARENT* frame, but has to be expressed in the *BODY* frame.
    /// @note This version is very expensive because it has to calculate sines
    ///       and cosines. If you already have those, use the alternate form
    ///       of this method.
    static Vec3 convertAngVelDotInBodyFrameToBodyXYZDotDot
        (const Vec3& q, const Vec3& w_PB_B, const Vec3& wdot_PB_B)
    {
        // Note: q[0] is not referenced so we won't waste time calculating
        // its cosine and sine here.
        return convertAngVelDotInBodyFrameToBodyXYZDotDot
                   (Vec3(0, std::cos(q[1]), std::cos(q[2])),
                    Vec3(0, std::sin(q[1]), std::sin(q[2])),
                    w_PB_B, wdot_PB_B);
    }

    /// This faster version of convertAngVelDotInBodyFrameToBodyXYZDotDot()
    /// assumes you have already calculated the cosine and sine of the three
    /// q's. Note that we only look at the cosines and sines of q[1] and q[2];
    /// q[0] does not matter so you don't have to fill in the 0'th element of
    /// cq and sq.
    /// Cost is XXX flops.
    // TODO: reimplement
    static Vec3 convertAngVelDotInBodyFrameToBodyXYZDotDot
       (const Vec3& cq, const Vec3& sq, const Vec3& w_PB_B, const Vec3& wdot_PB_B)
    {
        const Mat33 N      = calcNForBodyXYZInBodyFrame(cq,sq);
        const Vec3  qdot   = N * w_PB_B; // 15 flops
        const Mat33 NDot   = calcNDotForBodyXYZInBodyFrame(cq,sq,qdot);

        return N*wdot_PB_B + NDot*w_PB_B; // 33 flops
    }
#ifndef SWIG
    /// Given a possibly unnormalized quaternion q, calculate the 4x3 matrix
    /// N which maps angular velocity w to quaternion derivatives qdot. We
    /// expect the angular velocity in the parent frame, i.e. w==w_PB_P.
    /// We don't normalize, so N=|q|N' where N' is the normalized version.
    /// Cost is 7 flops.
    static Mat43P calcUnnormalizedNForQuaternion(const Vec4& q) {
        const Vec4 e = q/2;
        const P ne1 = -e[1], ne2 = -e[2], ne3 = -e[3];
        return Mat43P( ne1,  ne2,  ne3,
                       e[0], e[3], ne2,
                       ne3,  e[0], e[1],
                       e[2], ne1,  e[0]);
    }

    /// Given the time derivative qdot of a possibly unnormalized quaternion
    /// q, calculate the 4x3 matrix NDot which is the time derivative of the
    /// matrix N as described in calcUnnormalizedNForQuaternion(). Note that
    /// NDot = d/dt N = d/dt (|q|N') = |q|(d/dt N'), where N' is the normalized
    /// matrix, since the length of the quaternion should be a constant.
    /// Cost is 7 flops.
    static Mat43P calcUnnormalizedNDotForQuaternion(const Vec4& qdot) {
        const Vec4 ed = qdot/2;
        const P ned1 = -ed[1], ned2 = -ed[2], ned3 = -ed[3];
        return Mat43P( ned1,  ned2,  ned3,
                       ed[0], ed[3], ned2,
                       ned3,  ed[0], ed[1],
                       ed[2], ned1,  ed[0]);
    }

    /// Given a (possibly unnormalized) quaternion q, calculate the 3x4 matrix
    /// NInv (= N^-1) which maps quaternion derivatives qdot to angular velocity
    /// w, where the angular velocity is in the parent frame, i.e. w==w_PB_P.
    /// Note: when the quaternion is not normalized, this is not precisely the
    /// (pseudo)inverse of N. inv(N)=inv(N')/|q| but we're returning
    /// |q|*inv(N')=|q|^2*inv(N). That is, NInv*N =|q|^2*I, which is I
    /// if the original q was normalized. (Note: N*NInv != I, not even close.)
    /// Cost is 7 flops.
    static Mat34P calcUnnormalizedNInvForQuaternion(const Vec4& q) {
        const Vec4 e = 2*q;
        const P ne1 = -e[1], ne2 = -e[2], ne3 = -e[3];
        return Mat34P(ne1, e[0], ne3,  e[2],
                      ne2, e[3], e[0], ne1,
                      ne3, ne2,  e[1], e[0]);
    }
#endif

    /// Given a possibly unnormalized quaternion (0th element is the scalar) and the
    /// relative angular velocity vector of B in its parent, expressed
    /// in the *PARENT*, return the quaternion derivatives. This is never singular.
    /// Cost is 27 flops.
    static Vec4 convertAngVelToQuaternionDot(const Vec4& q, const Vec3& w_PB_P) {
        return calcUnnormalizedNForQuaternion(q)*w_PB_P;
    }

    /// Inverse of the above routine. Returned AngVel is expressed in
    /// the *PARENT* frame: w_PB_P.
    /// Cost is 28 flops.
    static Vec3 convertQuaternionDotToAngVel(const Vec4& q, const Vec4& qdot) {
        return calcUnnormalizedNInvForQuaternion(q)*qdot;
    }

    /// We want to differentiate qdot=N(q)*w to get qdotdot=N*b+NDot*w where
    /// b is angular acceleration wdot. Note that NDot=NDot(qdot), but it is
    /// far better to calculate the matrix-vector product NDot(N*w)*w directly
    /// rather than calculate NDot separately. That gives
    /// <pre>NDot*w = -(w^2)/4 * q</pre>
    /// Cost is 41 flops.
    static Vec4 convertAngVelDotToQuaternionDotDot
       (const Vec4& q, const Vec3& w_PB, const Vec3& b_PB)
    {
        const Mat43P N     = calcUnnormalizedNForQuaternion(q); //  7 flops
        const Vec4  Nb    = N*b_PB;                            // 20 flops
        const Vec4  NDotw = P(-.25)*w_PB.normSqr()*q;      // 10 flops
        return Nb + NDotw;                                      //  4 flops
    }

private:
    // This is only for the most trustworthy of callers, that is, methods of
    // the Rotation_ class.  There are a lot of ways for this NOT to be a
    // legitimate rotation matrix -- be careful!!
    // Note that these are supplied in rows.
    Rotation_( const P& xx, const P& xy, const P& xz,
               const P& yx, const P& yy, const P& yz,
               const P& zx, const P& zy, const P& zz )
    :   Mat33( xx,xy,xz, yx,yy,yz, zx,zy,zz ) {}

    // These next methods are highly-efficient power-user methods. Read the
    // code to understand them.
    /* SimTK_SimTKCOMMON_EXPORT */ Rotation_&  setTwoAngleTwoAxesBodyFixedForwardCyclicalRotation(     P cosAngle1, P sinAngle1, const CoordinateAxis& axis1, P cosAngle2, P sinAngle2, const CoordinateAxis& axis2 );
    /* SimTK_SimTKCOMMON_EXPORT */ Rotation_&  setThreeAngleTwoAxesBodyFixedForwardCyclicalRotation(   P cosAngle1, P sinAngle1, const CoordinateAxis& axis1, P cosAngle2, P sinAngle2, const CoordinateAxis& axis2, P cosAngle3, P sinAngle3 );
    /* SimTK_SimTKCOMMON_EXPORT */ Rotation_&  setThreeAngleThreeAxesBodyFixedForwardCyclicalRotation( P cosAngle1, P sinAngle1, const CoordinateAxis& axis1, P cosAngle2, P sinAngle2, const CoordinateAxis& axis2, P cosAngle3, P sinAngle3, const CoordinateAxis& axis3 );

    // These next methods are highly-efficient power-user methods to convert
    // Rotation matrices to orientation angles.  Read the code to understand them.
    /* SimTK_SimTKCOMMON_EXPORT */ Vec2  convertTwoAxesBodyFixedRotationToTwoAngles(     const CoordinateAxis& axis1, const CoordinateAxis& axis2 ) const;
    /* SimTK_SimTKCOMMON_EXPORT */ Vec3  convertTwoAxesBodyFixedRotationToThreeAngles(   const CoordinateAxis& axis1, const CoordinateAxis& axis2 ) const;
    /* SimTK_SimTKCOMMON_EXPORT */ Vec3  convertThreeAxesBodyFixedRotationToThreeAngles( const CoordinateAxis& axis1, const CoordinateAxis& axis2, const CoordinateAxis& axis3 ) const;

//------------------------------------------------------------------------------
// These are obsolete names from a previous release, listed here so that
// users will get a decipherable compilation error. (sherm 091101)
//------------------------------------------------------------------------------
private:
    // REPLACED BY: calcNForBodyXYZInBodyFrame()
    static Mat33 calcQBlockForBodyXYZInBodyFrame(const Vec3& a)
    {   return calcNForBodyXYZInBodyFrame(a); }
    // REPLACED BY: calcNInvForBodyXYZInBodyFrame()
    static Mat33 calcQInvBlockForBodyXYZInBodyFrame(const Vec3& a)
    {   return calcNInvForBodyXYZInBodyFrame(a); }
    // REPLACED BY: calcUnnormalizedNForQuaternion()
    static Mat<4,3,P> calcUnnormalizedQBlockForQuaternion(const Vec4& q)
    {   return calcUnnormalizedNForQuaternion(q); }
    // REPLACED BY: calcUnnormalizedNInvForQuaternion()
    static Mat<3,4,P> calcUnnormalizedQInvBlockForQuaternion(const Vec4& q)
    {   return calcUnnormalizedNInvForQuaternion(q); }
    // REPLACED BY: convertAngVelInBodyFrameToBodyXYZDot
    static Vec3 convertAngVelToBodyFixed123Dot(const Vec3& q, const Vec3& w_PB_B)
    {   return convertAngVelInBodyFrameToBodyXYZDot(q,w_PB_B); }
    // REPLACED BY: convertBodyXYZDotToAngVelInBodyFrame
    static Vec3 convertBodyFixed123DotToAngVel(const Vec3& q, const Vec3& qdot)
    {   return convertBodyXYZDotToAngVelInBodyFrame(q,qdot); }
    // REPLACED BY: convertAngVelDotInBodyFrameToBodyXYZDotDot
    static Vec3 convertAngVelDotToBodyFixed123DotDot
        (const Vec3& q, const Vec3& w_PB_B, const Vec3& wdot_PB_B)
    {   return convertAngVelDotInBodyFrameToBodyXYZDotDot(q,w_PB_B,wdot_PB_B); }

//------------------------------------------------------------------------------
// The following code is obsolete - it is here temporarily for backward
// compatibility (Mitiguy 9/5/2007)
//------------------------------------------------------------------------------
private:
    // These static methods are like constructors with friendlier names.
    static Rotation_ zero() { return Rotation_(); }
    static Rotation_ NaN()  { Rotation_ r;  r.setRotationToNaN();  return r; }

    /// By zero we mean "zero rotation", i.e., an identity matrix.
    Rotation_&  setToZero()            { return setRotationToIdentityMatrix(); }
    Rotation_&  setToIdentityMatrix()  { return setRotationToIdentityMatrix(); }
    Rotation_&  setToNaN()             { return setRotationToNaN(); }
    static Rotation_  trustMe( const Mat33& m )  { return Rotation_(m,true); }

    // One-angle rotations.
    static Rotation_ aboutX( const P& angleInRad ) { return Rotation_( angleInRad, XAxis ); }
    static Rotation_ aboutY( const P& angleInRad ) { return Rotation_( angleInRad, YAxis ); }
    static Rotation_ aboutZ( const P& angleInRad ) { return Rotation_( angleInRad, ZAxis ); }
    static Rotation_ aboutAxis( const P& angleInRad, const UnitVec3& axis ) { return Rotation_(angleInRad,axis); }
    static Rotation_ aboutAxis( const P& angleInRad, const Vec3& axis )     { return Rotation_(angleInRad,axis); }
    void  setToRotationAboutZ( const P& q ) { setRotationFromAngleAboutZ( q ); }

    // Two-angle space-fixed rotations.
    static Rotation_ aboutXThenOldY(const P& xInRad, const P& yInRad) { return Rotation_( SpaceRotationSequence, xInRad, XAxis, yInRad, YAxis ); }
    static Rotation_ aboutYThenOldX(const P& yInRad, const P& xInRad) { return Rotation_( SpaceRotationSequence, yInRad, YAxis, xInRad, XAxis ); }
    static Rotation_ aboutXThenOldZ(const P& xInRad, const P& zInRad) { return Rotation_( SpaceRotationSequence, xInRad, XAxis, zInRad, ZAxis ); }
    static Rotation_ aboutZThenOldX(const P& zInRad, const P& xInRad) { return Rotation_( SpaceRotationSequence, zInRad, ZAxis, xInRad, XAxis ); }
    static Rotation_ aboutYThenOldZ(const P& yInRad, const P& zInRad) { return Rotation_( SpaceRotationSequence, yInRad, YAxis, zInRad, ZAxis ); }
    static Rotation_ aboutZThenOldY(const P& zInRad, const P& yInRad) { return Rotation_( SpaceRotationSequence, zInRad, ZAxis, yInRad, YAxis ); }

    // Two-angle body fixed rotations (reversed space-fixed ones).
    static Rotation_ aboutXThenNewY(const P& xInRad, const P& yInRad) { return Rotation_( BodyRotationSequence, xInRad, XAxis, yInRad, YAxis ); }
    static Rotation_ aboutYThenNewX(const P& yInRad, const P& xInRad) { return aboutXThenOldY(xInRad, yInRad); }
    static Rotation_ aboutXThenNewZ(const P& xInRad, const P& zInRad) { return aboutZThenOldX(zInRad, xInRad); }
    static Rotation_ aboutZThenNewX(const P& zInRad, const P& xInRad) { return aboutXThenOldZ(xInRad, zInRad); }
    static Rotation_ aboutYThenNewZ(const P& yInRad, const P& zInRad) { return aboutZThenOldY(zInRad, yInRad); }
    static Rotation_ aboutZThenNewY(const P& zInRad, const P& yInRad) { return aboutYThenOldZ(yInRad, zInRad); }

    /// Create a Rotation_ matrix by specifying only its z axis.
    /// This will work for any stride UnitVec because there is always an implicit conversion available to the packed form used as the argument.
    explicit Rotation_( const UnitVec3& uvecZ )  { setRotationFromOneAxis(uvecZ,ZAxis); }

    /// Create a Rotation_ matrix by specifying its x axis, and a "y like" axis.
    //  We will take x seriously after normalizing, but use the y only to create z = normalize(x X y),
    //  then y = z X x. Bad things happen if x and y are aligned but we may not catch it.
    Rotation_( const Vec3& x, const Vec3& yish )  { setRotationFromTwoAxes( UnitVec3(x), XAxis, yish, YAxis ); }

    /// Set this Rotation_ to represent the same rotation as the passed-in quaternion.
    void setToQuaternion( const Quaternion& q )  { setRotationFromQuaternion(q); }

    /// Set this Rotation_ to represent a rotation of +q0 about the body frame's Z axis,
    /// followed by a rotation of +q1 about the body frame's NEW Y axis,
    /// followed by a rotation of +q3 about the body frame's NEW X axis.
    /// See Kane, Spacecraft Dynamics, pg. 423, body-three: 3-2-1.
    //  Similarly for BodyFixed123.
    void setToBodyFixed321( const Vec3& v)  { setRotationFromThreeAnglesThreeAxes( BodyRotationSequence, v[0], ZAxis, v[1], YAxis, v[2], XAxis ); }
    void setToBodyFixed123( const Vec3& v)  { setRotationToBodyFixedXYZ(v); }

    /// Convert this Rotation_ matrix to an equivalent (angle,axis) representation:
    /// The returned Vec4 is [angleInRadians, unitVectorX, unitVectorY, unitVectorZ].
    Vec4 convertToAngleAxis() const  { return convertRotationToAngleAxis(); }

    /// Convert this Rotation_ matrix to equivalent quaternion representation.
    Quaternion convertToQuaternion() const  { return convertRotationToQuaternion(); }

    /// Set this Rotation_ to represent a rotation of +q0 about the base frame's X axis,
    /// followed by a rotation of +q1 about the base frame's (unchanged) Y axis.
    void setToSpaceFixed12( const Vec2& q ) { setRotationFromTwoAnglesTwoAxes( SpaceRotationSequence, q[0], XAxis, q[1], YAxis ); }

    /// Convert this Rotation_ matrix to the equivalent 1-2-3 body fixed Euler angle sequence.
    /// Similarly, convert Rotation_ matrix to the equivalent 1-2 body  fixed Euler angle sequence.
    /// Similarly, convert Rotation_ matrix to the equivalent 1-2 space fixed Euler angle sequence.
    Vec3  convertToBodyFixed123() const  { return convertRotationToBodyFixedXYZ(); }
    Vec2  convertToBodyFixed12() const   { return convertRotationToBodyFixedXY(); }
    Vec2  convertToSpaceFixed12() const  { return convertTwoAxesRotationToTwoAngles( SpaceRotationSequence, XAxis, YAxis ); }
};


///-----------------------------------------------------------------------------
///  This InverseRotation class is the inverse of a Rotation
///  See the Rotation class for information.
///-----------------------------------------------------------------------------
template <class P=double>
class InverseRotation_ : public Mat<3,3,P>::TransposeType {
#ifndef SWIG
    typedef P               P;
    typedef Rotation_<P>    RotationP;
    typedef Mat<3,3,P>      Mat33; // not the base type!
    typedef SymMat<3,P>     SymMat33;
    typedef Mat<2,2,P>      Mat22P;
    typedef Mat<3,2,P>      Mat32P;
    typedef Vec<2,P>        Vec2;
    typedef Vec<3,P>        Vec3;
    typedef Vec<4,P>        Vec4;
#endif
public:
    /// This is the type of the underlying 3x3 matrix; note that it will have
    /// unusual row and column spacing since we're viewing it as transposed.
    typedef typename Mat<3,3,P>::TransposeType  BaseMat;

#ifndef SWIG
    /// Note that the unit vectors representing the rows and columns of this
    /// matrix do not necessarily have unit stride.
    //@{
    /// This is the type of a column of this InverseRotation.
    typedef  UnitVec<P,BaseMat::RowSpacing>  ColType;
    /// This is the type of a row of this InverseRotation.
    typedef  UnitRow<P,BaseMat::ColSpacing>  RowType;
    //@}
#endif
    /// You should not ever construct one of these as they should only occur as expression
    /// intermediates resulting from use of the "~" operator on a Rotation.
    /// But if you must, the default will produce an identity rotation.
    InverseRotation_() : BaseMat(1) {}

    /// This is an explicit implementation of the default copy constructor.
    InverseRotation_( const InverseRotation_& R ) : BaseMat(R) {}
    /// This is an explicit implementation of the default copy assignment operator.
    InverseRotation_&  operator=( const InverseRotation_& R )
    {   BaseMat::operator=(R.asMat33());  return *this; }

    /// Assuming this InverseRotation_ is R_AB, and given a symmetric dyadic matrix S_BB expressed
    /// in B, we can reexpress it in A using S_AA=R_AB*S_BB*R_BA. The matrix should be one
    /// that is formed as products of vectors expressed in A, such as inertia, gyration or
    /// covariance matrices. This can be done efficiently exploiting properties of R and S.
    /// Cost is 57 flops.
    /// @see Rotation::reexpressSymMat33()
#ifndef SWIG
    /* SimTK_SimTKCOMMON_EXPORT */ SymMat33 reexpressSymMat33(const SymMat33& S_BB) const;
#endif
    /// We can invert an InverseRotation just by recasting it to a Rotation at zero cost.
    //@{
    const Rotation_<P>&  invert() const {return *reinterpret_cast<const Rotation_<P>*>(this);}
#ifndef SWIG
    Rotation_<P>&        updInvert() {return *reinterpret_cast<Rotation_<P>*>(this);}
#endif
    //@}

    /// Transpose, and transpose operators (override BaseMat versions of transpose).
    /// For an orthogonal matrix like this one transpose is the same as inverse.
    //@{
    const Rotation_<P>&  transpose() const  { return invert(); }
    const Rotation_<P>&  operator~() const  { return invert(); }
#ifndef SWIG
    Rotation_<P>&        updTranspose()     { return updInvert(); }
#endif
    Rotation_<P>&        operator~()        { return updInvert(); }
    //@}
#ifndef SWIG
    /// Access individual rows and columns of this InverseRotation; no cost or
    /// copying since suitably-cast references to the actual data are returned.
    /// There are no writable versions of these methods since changing a single
    /// row or column would violate the contract that these are always legitimate
    /// rotation matrices.
    //@{

    const RowType&  row( int i ) const         { return reinterpret_cast<const RowType&>(asMat33()[i]); }
    const ColType&  col( int j ) const         { return reinterpret_cast<const ColType&>(asMat33()(j)); }
    const ColType&  x() const                  { return col(0); }
    const ColType&  y() const                  { return col(1); }
    const ColType&  z() const                  { return col(2); }
    const RowType&  operator[]( int i ) const  { return row(i); }
    const ColType&  operator()( int j ) const  { return col(j); }

    //@}

    /// Conversion from InverseRotation_ to BaseMat.
    /// Note: asMat33 is more efficient than toMat33() (no copy), but you have to know the internal layout.
    //@{
    const BaseMat&  asMat33() const  { return *static_cast<const BaseMat*>(this); }
    BaseMat         toMat33() const  { return asMat33(); }
    //@}
#endif
};

#ifndef SWIG
/// Write a Rotation matrix to an output stream by writing out its underlying Mat33.
template <class P> SimTK_SimTKCOMMON_EXPORT std::ostream&
operator<<(std::ostream&, const Rotation_<P>&);
/// Write an InverseRotation matrix to an output stream by writing out its underlying Mat33.
template <class P> SimTK_SimTKCOMMON_EXPORT std::ostream&
operator<<(std::ostream&, const InverseRotation_<P>&);

/// Rotating a unit vector leaves it unit length, saving us from having to perform
/// an expensive normalization. So we override the multiply operators here changing
/// the return type to UnitVec or UnitRow.
//@{
template <class P, int S> inline UnitVec<P,1>
operator*(const Rotation_<P>& R, const UnitVec<P,S>& v)        {return UnitVec<P,1>(R.asMat33()* v.asVec3(),  true);}
template <class P, int S> inline UnitRow<P,1>
operator*(const UnitRow<P,S>& r, const Rotation_<P>& R)        {return UnitRow<P,1>(r.asRow3() * R.asMat33(), true);}
template <class P, int S> inline UnitVec<P,1>
operator*(const InverseRotation_<P>& R, const UnitVec<P,S>& v) {return UnitVec<P,1>(R.asMat33()* v.asVec3(),  true);}
template <class P, int S> inline UnitRow<P,1>
operator*(const UnitRow<P,S>& r, const InverseRotation_<P>& R) {return UnitRow<P,1>(r.asRow3() * R.asMat33(), true);}
//@}

// Couldn't implement these Rotation_ methods until InverseRotation_ was defined.
template <class P> inline
Rotation_<P>::Rotation_(const InverseRotation_<P>& R) : Mat<3,3,P>( R.asMat33() ) {}
template <class P> inline Rotation_<P>&
Rotation_<P>::operator=(const InverseRotation_<P>& R)  {static_cast<Mat<3,3,P>&>(*this)  = R.asMat33();    return *this;}
template <class P> inline Rotation_<P>&
Rotation_<P>::operator*=(const Rotation_<P>& R)        {static_cast<Mat<3,3,P>&>(*this) *= R.asMat33();    return *this;}
template <class P> inline Rotation_<P>&
Rotation_<P>::operator/=(const Rotation_<P>& R)        {static_cast<Mat<3,3,P>&>(*this) *= (~R).asMat33(); return *this;}
template <class P> inline Rotation_<P>&
Rotation_<P>::operator*=(const InverseRotation_<P>& R) {static_cast<Mat<3,3,P>&>(*this) *= R.asMat33();    return *this;}
template <class P> inline Rotation_<P>&
Rotation_<P>::operator/=(const InverseRotation_<P>& R) {static_cast<Mat<3,3,P>&>(*this) *= (~R).asMat33(); return *this;}

/// Composition of Rotation matrices via operator*.
//@{
template <class P> inline Rotation_<P>
operator*(const Rotation_<P>&        R1, const Rotation_<P>&        R2)  {return Rotation_<P>(R1) *= R2;}
template <class P> inline Rotation_<P>
operator*(const Rotation_<P>&        R1, const InverseRotation_<P>& R2)  {return Rotation_<P>(R1) *= R2;}
template <class P> inline Rotation_<P>
operator*(const InverseRotation_<P>& R1, const Rotation_<P>&        R2)  {return Rotation_<P>(R1) *= R2;}
template <class P> inline Rotation_<P>
operator*(const InverseRotation_<P>& R1, const InverseRotation_<P>& R2)  {return Rotation_<P>(R1) *= R2;}
//@}

/// Composition of a Rotation matrix and the inverse of another Rotation via operator/, that is
/// R1/R2 == R1*(~R2).
//@{
template <class P> inline Rotation_<P>
operator/( const Rotation_<P>&        R1, const Rotation_<P>&        R2 )  {return Rotation_<P>(R1) /= R2;}
template <class P> inline Rotation_<P>
operator/( const Rotation_<P>&        R1, const InverseRotation&     R2 )  {return Rotation_<P>(R1) /= R2;}
template <class P> inline Rotation_<P>
operator/( const InverseRotation_<P>& R1, const Rotation_<P>&        R2 )  {return Rotation_<P>(R1) /= R2;}
template <class P> inline Rotation_<P>
operator/( const InverseRotation_<P>& R1, const InverseRotation_<P>& R2 )  {return Rotation_<P>(R1) /= R2;}
//@}

#endif
//------------------------------------------------------------------------------
}  // End of namespace SimTK

//--------------------------------------------------------------------------
#endif // SimTK_SimTKCOMMON_ROTATION_H_
//--------------------------------------------------------------------------
