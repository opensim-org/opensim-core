/* -------------------------------------------------------------------------- *
 *                    OpenSim:  StationDefinedFrame.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Adam Kewley                                                     *
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

#include "StationDefinedFrame.h"

#include <OpenSim/Common/Assertion.h>
#include <OpenSim/Common/Component.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Property.h>
#include <OpenSim/Simulation/Model/Frame.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Station.h>
#include <Simbody.h>

#include <array>
#include <sstream>
#include <string>
#include <utility>

using OpenSim::Component;
using OpenSim::Exception;
using OpenSim::Frame;
using OpenSim::Property;
using OpenSim::Station;

namespace {

    // helper: returns the base frame that `station` is defined in
    Frame const& FindBaseFrame(Station const& station)
    {
        return station.getParentFrame().findBaseFrame();
    }

    // helper: returns the location of the `Station` w.r.t. its base frame
    SimTK::Vec3 GetLocationInBaseFrame(Station const& station)
    {
        return station.getParentFrame().findTransformInBaseFrame() * station.get_location();
    }

    // helper: tries to parse a given character as a designator for an axis of a 3D coordinate
    //
    // returns `true` if `out` was updated by the parser, or `false` if the character couldn't be parsed
    bool TryParseAsCoordinateAxis(std::string::value_type c, SimTK::CoordinateAxis& out)
    {
        switch (c) {
        case 'x':
        case 'X':
            out = SimTK::CoordinateAxis::XCoordinateAxis{};
            return true;
        case 'y':
        case 'Y':
            out = SimTK::CoordinateAxis::YCoordinateAxis{};
            return true;
        case 'z':
        case 'Z':
            out = SimTK::CoordinateAxis::ZCoordinateAxis{};
            return true;
        default:
            return false;
        }
    }

    // helper: tries to parse the given string as a potentially-signed representation of a
    // 3D coordinate dimension (e.g. "-x" --> SimTK::CoordinateDirection::NegXDirection()`)
    //
    // returns `true` if `out` was updated by the parser, or `false` if the string couldn't be parsed
    bool TryParseAsCoordinateDirection(std::string s, SimTK::CoordinateDirection& out)
    {
        if (s.empty())
        {
            return false;  // cannot parse: input is empty
        }

        // handle and consume sign (direction) prefix (e.g. '+' / '-')
        const bool isNegated = s.front() == '-';
        if (isNegated || s.front() == '+')
        {
            s = s.substr(1);
        }

        if (s.empty())
        {
            return false;  // cannot parse: the input was just a prefix with no axis (e.g. "+")
        }

        // handle axis suffix
        SimTK::CoordinateAxis axis = SimTK::CoordinateAxis::XCoordinateAxis();
        if (!TryParseAsCoordinateAxis(s.front(), axis))
        {
            return false;
        }

        out = SimTK::CoordinateDirection{axis, isNegated ? -1 : 1};
        return true;
    }

    // helper: tries to parse the string value held within `prop` as a coordinate direction, throwing
    // if the parse isn't possible
    SimTK::CoordinateDirection ParseAsCoordinateDirectionOrThrow(
        Component const& owner,
        Property<std::string> const& prop)
    {
        SimTK::CoordinateDirection dir = SimTK::CoordinateAxis::XCoordinateAxis{};
        if (TryParseAsCoordinateDirection(prop.getValue(), dir))
        {
            return dir;
        }
        else
        {
            std::stringstream ss;
            ss << prop.getName() << ": has an invalid value ('" << prop.getValue() << "'): permitted values are -x, +x, -y, +y, -z, or +z";
            OPENSIM_THROW(Exception, owner, std::move(ss).str());
        }
    }

    // helper: returns the (parseable) string equivalent of the given direction
    std::string to_string(SimTK::CoordinateDirection const& dir)
    {
        std::string rv;
        rv += dir.getDirection() > 0 ? '+' : '-';
        rv += std::array<char, 3>{'x', 'y', 'z'}.at(dir.getAxis());
        return rv;
    }
}


OpenSim::StationDefinedFrame::StationDefinedFrame()
{
    constructProperty_ab_axis("+x");
    constructProperty_ab_x_ac_axis("+y");
}

OpenSim::StationDefinedFrame::StationDefinedFrame(
    SimTK::CoordinateDirection abAxis,
    SimTK::CoordinateDirection abXacAxis,
    Station const& pointA,
    Station const& pointB,
    Station const& pointC,
    Station const& originPoint)
{
    constructProperty_ab_axis(to_string(abAxis));
    constructProperty_ab_x_ac_axis(to_string(abXacAxis));
    connectSocket_point_a(pointA);
    connectSocket_point_b(pointB);
    connectSocket_point_c(pointC);
    connectSocket_origin_point(originPoint);
}

const Station& OpenSim::StationDefinedFrame::getPointA() const
{
    return getConnectee<Station>("point_a");
}

const Station& OpenSim::StationDefinedFrame::getPointB() const
{
    return getConnectee<Station>("point_b");
}

const Station& OpenSim::StationDefinedFrame::getPointC() const
{
    return getConnectee<Station>("point_c");
}

const Station& OpenSim::StationDefinedFrame::getOriginPoint() const
{
    return getConnectee<Station>("origin_point");
}

const Frame& OpenSim::StationDefinedFrame::extendFindBaseFrame() const
{
    return FindBaseFrame(getPointA());
}

SimTK::Transform OpenSim::StationDefinedFrame::extendFindTransformInBaseFrame() const
{
    return _transformInBaseFrame;
}

void OpenSim::StationDefinedFrame::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    // parse `ab_axis`
    const SimTK::CoordinateDirection abDirection = ParseAsCoordinateDirectionOrThrow(*this, getProperty_ab_axis());

    // parse `ab_x_ac_axis`
    const SimTK::CoordinateDirection abXacDirection = ParseAsCoordinateDirectionOrThrow(*this, getProperty_ab_x_ac_axis());

    // ensure `ab_axis` is orthogonal to `ab_x_ac_axis`
    if (abDirection.hasSameAxis(abXacDirection)) {
        std::stringstream ss;
        ss << getProperty_ab_axis().getName() << " (" << getProperty_ab_axis().getValue() << ") and " << getProperty_ab_x_ac_axis().getName() << " (" << getProperty_ab_x_ac_axis().getValue() << ") are not orthogonal";
        OPENSIM_THROW_FRMOBJ(Exception, std::move(ss).str());
    }

    // update vector-to-axis mappings so that `extendConnectToModel` knows how
    // computed vectors (e.g. `ab_x_ac_axis`) relate to the frame transform (e.g. +Y)
    _basisVectorToFrameMappings = {
        abDirection,
        abXacDirection,
        SimTK::CoordinateDirection{abDirection.crossProductAxis(abXacDirection), abDirection.crossProductSign(abXacDirection)},
    };
}

void OpenSim::StationDefinedFrame::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    // ensure all of the `Station`'s have the same base frame
    //
    // this is a hard requirement, because we need to know _for certain_ that
    // the relative transform of this frame doesn't change w.r.t. the base
    // frame during integration
    //
    // (e.g. it would cause mayhem if a Joint was defined using a
    // `StationDefinedFrame` that, itself, changes in response to a change in that
    // Joint's coordinates)
    Frame const& pointABaseFrame = FindBaseFrame(getPointA());
    Frame const& pointBBaseFrame = FindBaseFrame(getPointB());
    Frame const& pointCBaseFrame = FindBaseFrame(getPointC());
    Frame const& originPointFrame = FindBaseFrame(getOriginPoint());
    OPENSIM_ASSERT_FRMOBJ_ALWAYS(&pointABaseFrame == &pointBBaseFrame && "`point_b` is defined in a different base frame from `point_a`. All `Station`s (`point_a`, `point_b`, `point_c`, and `origin_point` of a `StationDefinedFrame` must be defined in the same base frame.");
    OPENSIM_ASSERT_FRMOBJ_ALWAYS(&pointABaseFrame == &pointCBaseFrame && "`point_c` is defined in a different base frame from `point_a`. All `Station`s (`point_a`, `point_b`, `point_c`, and `origin_point` of a `StationDefinedFrame` must be defined in the same base frame.");
    OPENSIM_ASSERT_FRMOBJ_ALWAYS(&pointABaseFrame == &originPointFrame && "`origin_point` is defined in a different base frame from `point_a`. All `Station`s (`point_a`, `point_b`, `point_c`, and `origin_point` of a `StationDefinedFrame` must be defined in the same base frame.");
    OPENSIM_ASSERT_FRMOBJ_ALWAYS(dynamic_cast<PhysicalFrame const*>(&pointABaseFrame) && "the base frame of the stations must be a physical frame (e.g. an `OpenSim::Body`, an `OpenSim::PhysicalOffsetFrame`, etc.)");

    // once we know _for certain_ that all of the points can be calculated w.r.t.
    // the same base frame, we can precompute the transform
    _transformInBaseFrame = calcTransformInBaseFrame();
}

void OpenSim::StationDefinedFrame::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    setMobilizedBodyIndex(dynamic_cast<PhysicalFrame const&>(findBaseFrame()).getMobilizedBodyIndex());
}

SimTK::Transform OpenSim::StationDefinedFrame::calcTransformInBaseFrame() const
{
    // get raw input data
    const SimTK::Vec3 pointA = GetLocationInBaseFrame(getPointA());
    const SimTK::Vec3 pointB = GetLocationInBaseFrame(getPointB());
    const SimTK::Vec3 pointC = GetLocationInBaseFrame(getPointC());
    const SimTK::Vec3 originPoint = GetLocationInBaseFrame(getOriginPoint());

    // validate raw input data
    OPENSIM_ASSERT_FRMOBJ_ALWAYS(pointB != pointA && "`point_b` must be at a different location from `point_a` (the three points of a `StationDefinedFrame` must form a triangle)");
    OPENSIM_ASSERT_FRMOBJ_ALWAYS(pointC != pointA && "`point_c` must be at a different location from `point_a` (the three points of a `StationDefinedFrame` must form a triangle)");
    OPENSIM_ASSERT_FRMOBJ_ALWAYS(pointC != pointB && "`point_c` must be at a different location from `point_b` (the three points of a `StationDefinedFrame` must form a triangle)");

    // compute orthonormal basis vectors
    const SimTK::UnitVec3 ab(pointB - pointA);
    const SimTK::UnitVec3 ac(pointC - pointA);
    const SimTK::UnitVec3 ab_x_ac{cross(ab, ac)};
    const SimTK::UnitVec3 ab_x_ab_x_ac{cross(ab, ab_x_ac)};

    // remap them into a 3x3 "change of basis" matrix for each frame axis
    SimTK::Mat33 orientation{};
    orientation.col(_basisVectorToFrameMappings[0].getAxis()) = _basisVectorToFrameMappings[0].getDirection() * SimTK::Vec3(ab);
    orientation.col(_basisVectorToFrameMappings[1].getAxis()) = _basisVectorToFrameMappings[1].getDirection() * SimTK::Vec3(ab_x_ac);
    orientation.col(_basisVectorToFrameMappings[2].getAxis()) = _basisVectorToFrameMappings[2].getDirection() * SimTK::Vec3(ab_x_ab_x_ac);

    // combine with the origin point to create the complete transform in the base frame
    return SimTK::Transform{SimTK::Rotation{orientation}, originPoint};
}

SimTK::Transform OpenSim::StationDefinedFrame::calcTransformInGround(const SimTK::State& state) const
{
    return extendFindBaseFrame().getTransformInGround(state) * _transformInBaseFrame;
}

SimTK::SpatialVec OpenSim::StationDefinedFrame::calcVelocityInGround(const SimTK::State& state) const
{
    // note: this calculation is inspired from the one found in `OpenSim/Simulation/Model/OffsetFrame.h`

    const Frame& baseFrame = findBaseFrame();

    // get the (angular + linear) velocity of the base frame w.r.t. ground
    const SimTK::SpatialVec vbf = baseFrame.getVelocityInGround(state);

    // calculate the rigid _offset_ (not position) of this frame w.r.t. ground
    const SimTK::Vec3 offset = baseFrame.getTransformInGround(state).R() * findTransformInBaseFrame().p();

    return SimTK::SpatialVec{
        // the angular velocity of this frame is the same as its base frame (it's a rigid attachment)
        vbf(0),

        // the linear velocity of this frame is the linear velocity of its base frame, _plus_ the
        // rejection of this frame's offset from the base frame's angular velocity
        //
        // this is to account for the fact that rotation around the base frame will affect the linear
        // velocity of frames that are at an offset away from the rotation axis
        vbf(1) + (vbf(0) % offset),
    };
}

SimTK::SpatialVec OpenSim::StationDefinedFrame::calcAccelerationInGround(const SimTK::State& state) const
{
    // note: this calculation is inspired from the one found in `OpenSim/Simulation/Model/OffsetFrame.h`

    const Frame& baseFrame = findBaseFrame();

    // get the (angular + linear) velocity and acceleration of the base frame w.r.t. ground
    const SimTK::SpatialVec vbf = baseFrame.getVelocityInGround(state);
    const SimTK::SpatialVec abf = baseFrame.getAccelerationInGround(state);

    // calculate the rigid _offset_ (not position) of this frame w.r.t. ground
    const SimTK::Vec3 offset = baseFrame.getTransformInGround(state).R() * findTransformInBaseFrame().p();

    return SimTK::SpatialVec{
        // the angular acceleration of this frame is the same as its base frame (it's a rigid attachment)
        abf(0),

        // the linear acceleration of this frame is:
        //
        // - the linear acceleration of its base frame
        //
        // - plus the rejection of this frame's offset from from the base frame's angular velocity (to
        //   account for the fact that rotational acceleration in the base frame becomes linear acceleration
        //   for any frames attached at an offset that isn't along the rotation axis)
        //
        // - plus the rejection of TODO
        abf(1) + (abf(0) % offset) + (vbf(0) % (vbf(0) % offset)),
    };
}
