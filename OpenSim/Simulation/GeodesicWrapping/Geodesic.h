#ifndef OPENSIM_GEODESIC_H
#define OPENSIM_GEODESIC_H
/* -------------------------------------------------------------------------- *
 *                           OpenSim: Geodesic.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco, Andreas Scholz                                 *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

namespace OpenSim {

struct GeodesicBoundaryPointFrame {
    /// The vector from the surface frame to point of geodesic.
    SimTK::Vec3 x;
    /// The tangent vector of the geodesic.
    SimTK::Vec3 t;
    /// The normal vector of the surface.
    SimTK::Vec3 N;
    /// The binormal vector, t x N.
    SimTK::Vec3 B;

    GeodesicBoundaryPointFrame() : x(0), t(0), N(0), B(0) {}
};

struct JacobiScalar {
    /// The value of the Jacobi scalar.
    SimTK::Real q;
    /// The first derivative of the Jacobi scalar.
    SimTK::Real dq;
    /// The second derivative of the Jacobi scalar.
    SimTK::Real ddq;

    JacobiScalar() : q(0), dq(0), ddq(0) {}
};

// TODO include in the API?
class OSIMSIMULATION_API Geodesic : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(Geodesic, Object);
public:
//==============================================================================
// METHODS
//==============================================================================

    // CONSTRUCTION AND DESTRUCTION
    Geodesic();
    ~Geodesic() noexcept override;

    Geodesic(const Geodesic&);
    Geodesic& operator=(const Geodesic&);

    Geodesic(Geodesic&& other);
    Geodesic& operator=(Geodesic&& other);

    // GET AND SET
    const PhysicalFrame& getSurfaceFrame() const;
    void setSurfaceFrame(const PhysicalFrame& surfaceFrame);

//    const GeodesicBoundaryPointFrame& getBoundaryPointFrameAtStart() const;
//    const GeodesicBoundaryPointFrame& getBoundaryPointFrameAtEnd() const;
//
//    SimTK::Real getTangentialNormalCurvatureAtStart() const;
//    SimTK::Real getBinormalNormalCurvatureAtStart() const;
//    SimTK::Real getTangentialNormalCurvatureAtEnd() const;
//    SimTK::Real getBinormalNormalCurvatureAtEnd() const;
//
//    SimTK::Real getTangentialGeodesicTorsionAtStart() const;
//    SimTK::Real getTangentialGeodesicTorsionAtEnd() const;
//
//    SimTK::Real getLength() const;
//    SimTK::Real getSpeed() const;
//    SimTK::Real getAcceleration() const;
//
//    const JacobiScalar& getDisplacementJacobiScalarAtStart() const;
//    const JacobiScalar& getDisplacementJacobiScalarAtEnd() const;
//
//    const JacobiScalar& getRotationJacobiScalarAtStart() const;
//    const JacobiScalar& getRotationJacobiScalarAtEnd() const;
//
//
//    void setBoundaryPointFrameAtStart(const GeodesicBoundaryPointFrame& boundaryPointFrameAtStart);
//    void setBoundaryPointFrameAtEnd(const GeodesicBoundaryPointFrame& boundaryPointFrameAtEnd);
//
//    void setNormalCurvatureTanAtStart(SimTK::Real normalCurvatureTanAtStart);
//    void setNormalCurvatureBinAtStart(SimTK::Real normalCurvatureBinAtStart);
//    void setNormalCurvatureTanAtEnd(SimTK::Real normalCurvatureTanAtEnd);
//    void setNormalCurvatureBinAtEnd(SimTK::Real normalCurvatureBinAtEnd);
//
//    void setGeodesicTorsionTanAtStart(SimTK::Real geodesicTorsionTanAtStart);
//    void setGeodesicTorsionTanAtEnd(SimTK::Real geodesicTorsionTanAtEnd);
//
//    void setLength(SimTK::Real length);
//    void setSpeed(SimTK::Real speed);
//    void setAcceleration(SimTK::Real acceleration);
//
//    void setDisplacementJacobiScalarAtStart(const JacobiScalar& displacementJacobiScalarAtStart);
//    void setDisplacementJacobiScalarAtEnd(const JacobiScalar& displacementJacobiScalarAtEnd);
//
//    void setRotationJacobiScalarAtStart(const JacobiScalar& rotationJacobiScalarAtStart);
//    void setRotationJacobiScalarAtEnd(const JacobiScalar& rotationJacobiScalarAtEnd);

private:
    SimTK::ReferencePtr<const PhysicalFrame> m_surfaceFrame;

    GeodesicBoundaryPointFrame m_boundaryPointFrameAtStart;
    GeodesicBoundaryPointFrame m_boundaryPointFrameAtEnd;
    SimTK::Real m_tangentialNormalCurvatureAtStart;
    SimTK::Real m_binormalNormalCurvatureAtStart;
    SimTK::Real m_tangentialNormalCurvatureAtEnd;
    SimTK::Real m_binormalNormalCurvatureAtEnd;
    SimTK::Real m_tangentialGeodesicTorsionAtStart;
    SimTK::Real m_tangentialGeodesicTorsionAtEnd;
    SimTK::Real m_length;
    SimTK::Real m_speed;
    SimTK::Real m_acceleration;
    JacobiScalar m_displacementJacobiScalarAtStart;
    JacobiScalar m_displacementJacobiScalarAtEnd;
    JacobiScalar m_rotationJacobiScalarAtStart;
    JacobiScalar m_rotationJacobiScalarAtEnd;
};

} // namespace OpenSim

#endif // OPENSIM_GEODESIC_H
