/* -------------------------------------------------------------------------- *
 *                 OpenSim:  testStationDefinedFrame.cpp                      *
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

#include <OpenSim/Simulation/Model/StationDefinedFrame.h>

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Station.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>

#include <catch2/catch_all.hpp>
#include <Simbody.h>

#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

using OpenSim::ArrayPtrs;
using OpenSim::Body;
using OpenSim::Joint;
using OpenSim::Model;
using OpenSim::ModelComponent;
using OpenSim::Object;
using OpenSim::PhysicalOffsetFrame;
using OpenSim::Station;
using OpenSim::StationDefinedFrame;
using OpenSim::WeldJoint;

// helper functions
namespace {
    // helper: generic function that uses `adder` to add a `T` created from `args...` to `model`
    template<typename T, typename MemberFunc, typename... Args>
    T& EmplaceGeneric(Model& model, MemberFunc adder, Args&&... args)
    {
        static_assert(std::is_constructible<T, Args&&...>::value, "Args must be able to construct the T");

        auto p = std::make_unique<T>(std::forward<Args>(args)...);
        T* ptr = p.get();
        adder(model, p.release());
        return *ptr;
    }

    // helper: emplaces a `T` within `model`'s model component collection
    template<typename T, typename... Args>
    T& EmplaceModelComponent(Model& model, Args&&... args)
    {
        static_assert(std::is_base_of<ModelComponent, T>::value, "T must inherit from ModelComponent");
        return EmplaceGeneric<T>(model, std::mem_fn(&Model::addModelComponent), std::forward<Args>(args)...);
    }

    // helper: emplaces a `T` within `model`'s bodyset
    template<typename T = Body, typename... Args>
    T& EmplaceBody(Model& model, Args&&... args)
    {
        static_assert(std::is_base_of<Body, T>::value, "T must inherit from Body");
        return EmplaceGeneric<T>(model, std::mem_fn(&Model::addBody), std::forward<Args>(args)...);
    }

    // helper: emplaces a `T` within `model`'s jointset
    template<typename T = Joint, typename... Args>
    T& EmplaceJoint(Model& model, Args&&... args)
    {
        static_assert(std::is_base_of<Joint, T>::value, "T must inherit from Joint");
        return EmplaceGeneric<T>(model, std::mem_fn(&Model::addJoint), std::forward<Args>(args)...);
    }

    // helper: adds a `StationDefinedFrame` to the model that uses 4 stations, defined in
    // ground at the given `*Location`s, and returns that model
    Model CreateModelWithSDFPointsAt(
        std::string const& sdfName,
        SimTK::Vec3 const& pointALocation,
        SimTK::Vec3 const& pointBLocation,
        SimTK::Vec3 const& pointCLocation,
        SimTK::Vec3 const& originLocation)
    {
        Model model;

        auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), pointALocation);
        auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), pointBLocation);
        auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), pointCLocation);
        auto& origin = EmplaceModelComponent<Station>(model, model.getGround(), originLocation);

        // add a `StationDefinedFrame` that uses the stations to the model
        EmplaceModelComponent<StationDefinedFrame>(
            model,
            sdfName,
            SimTK::CoordinateAxis::XCoordinateAxis(),
            SimTK::CoordinateAxis::YCoordinateAxis(),
            p1,
            p2,
            p3,
            origin
        );

        // leave finalization etc. to the caller
        return model;
    }
}

TEST_CASE("StationDefinedFrame_IsRegistered")
{
    // sanity check: `StationDefinedFrame` should be part of the general `Object` registry
    ArrayPtrs<StationDefinedFrame> objs;
    Object::getRegisteredObjectsOfGivenType<StationDefinedFrame>(objs);
    REQUIRE(objs.size() == 1);
}

TEST_CASE("StationDefinedFrame_CanBeDefaultConstructed")
{
    // sanity check: if this doesn't work then something _quite_ strange has happened :>
    REQUIRE_NOTHROW(StationDefinedFrame{});
}

TEST_CASE("StationDefinedFrame_CanCreateAModelContainingAStandaloneStationDefinedFrame")
{
    Model model;

    // add stations to the model
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0, -1.0, 0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0,  1.0, 0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{ 1.0,  0.0, 0.0});
    auto& origin = p1;

    // add a `StationDefinedFrame` that uses the stations to the model
    EmplaceModelComponent<StationDefinedFrame>(
        model,
        "sdf",
        SimTK::CoordinateAxis::XCoordinateAxis(),
        SimTK::CoordinateAxis::YCoordinateAxis(),
        p1,
        p2,
        p3,
        origin
    );

    // the resulting model should finalize etc. fine
    REQUIRE_NOTHROW(model.buildSystem());
    SimTK::State state = model.initializeState();
    model.realizeReport(state);
}

TEST_CASE("StationDefinedFrame_CanCreateModelContainingStationDefinedFrameAsParentOfOffsetFrame")
{
    Model model;

    // add stations to the model
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0, -1.0, 0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0,  1.0, 0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{ 1.0,  0.0, 0.0});
    auto& origin = p1;

    // add a `StationDefinedFrame` that uses the stations to the model
    auto& sdf = EmplaceModelComponent<StationDefinedFrame>(
        model,
        "sdf",
        SimTK::CoordinateAxis::XCoordinateAxis(),
        SimTK::CoordinateAxis::YCoordinateAxis(),
        p1,
        p2,
        p3,
        origin
    );

    EmplaceModelComponent<PhysicalOffsetFrame>(model, sdf, SimTK::Transform{});

    // the model should initialize etc. fine
    REQUIRE_NOTHROW(model.buildSystem());
    SimTK::State state = model.initializeState();
    model.realizeReport(state);
}

TEST_CASE("StationDefinedFrame_CanCreateAModelContainingAStationDefinedFrameAsJointParentFrame")
{
    Model model;

    // add stations to the model
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0, -1.0, 0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0,  1.0, 0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{ 1.0,  0.0, 0.0});
    auto& origin = p1;

    // add a `StationDefinedFrame` that uses the stations to the model
    auto& sdf = EmplaceModelComponent<StationDefinedFrame>(
        model,
        "sdf",
        SimTK::CoordinateAxis::XCoordinateAxis(),
        SimTK::CoordinateAxis::YCoordinateAxis(),
        p1,
        p2,
        p3,
        origin
    );

    // add a `Body` that will act as the child of a `Joint`
    auto& body = EmplaceBody(
        model,
        "name",
        1.0,
        SimTK::Vec3{0.0, 0.0, 0.0},
        SimTK::Inertia{1.0, 1.0, 1.0}
    );

    // add a `Joint` between the `StationDefinedFrame` and the `Body`
    EmplaceJoint<WeldJoint>(model, "weld", sdf, body);

    // the model should initialize etc. fine
    REQUIRE_NOTHROW(model.buildSystem());
    SimTK::State state = model.initializeState();
    model.realizeReport(state);
}

TEST_CASE("StationDefinedFrame_SanityCheckCanUsePOFAsParentOfJointViaOtherOffsetFrames")
{
    // sanity check, kept for regression checks:
    //
    // `StationDefinedFrame`s required changing the system addition and topology-sorting
    // part of `Model` finalization, and it wasn't entirely clear when/where `StationDefinedFrame`
    // would behave differently from a `PhysicalOffsetFrame`
    //
    // this test is almost identical to the `StationDefinedFrame`-based one below, and they both
    // fail because both `PhysicalOffsetFrame`s and `StationDefinedFrame`s aren't added to the
    // system in the correct order by the `Model` implementation

    Model model;

    // add a `PhysicalOffsetFrame`
    auto& pof = EmplaceModelComponent<PhysicalOffsetFrame>(
        model,
        model.getGround(),
        SimTK::Transform{}
    );

    // add the to-be-joined-to body
    auto& body = EmplaceBody(
        model,
        "name",
        1.0,
        SimTK::Vec3{0.0, 0.0, 0.0},
        SimTK::Inertia{1.0, 1.0, 1.0}
    );

    EmplaceJoint<WeldJoint>(model,
        std::string{"weld"},
        pof,
        SimTK::Vec3{1.0, 0.0, 0.0},  // location in parent
        SimTK::Vec3{},               // orientation in parent
        body,
        SimTK::Vec3{0.0, 1.0, 0.0},  // location in child
        SimTK::Vec3{}                // orientation in child
    );

    // fails because `Ground` <-- `PhysicalOffsetFrame` <-- `PhysicalOffsetFrame` <-- `Joint` --> `PhysicalOffsetFrame` --> `Body`
    // isn't handled correctly by OpenSim's graph traversal
    //
    // remove the `REQUIRE` part and uncomment the other lines if you think you've fixed this
    REQUIRE_THROWS(model.buildSystem());
    // SimTK::State state = model.initializeState();
    // model.realizeReport(state);
}

TEST_CASE("StationDefinedFrame_CanCreateModelContainingStationDefinedFrameViaOffsetFrameForJoint")
{
    // i.e. check that the topology:
    //
    // `Ground` <-- `StationDefinedFrame` <-- `PhysicalOffsetFrame` <-- `Joint` --> `PhysicalOffsetFrame` --> `Body`
    //
    // finalizes etc. fine

    Model model;

    // add stations to the model
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0, -1.0, 0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{-1.0,  1.0, 0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{ 1.0,  0.0, 0.0});
    auto& origin = p1;

    // add a `StationDefinedFrame` that uses the stations to the model
    auto& sdf = EmplaceModelComponent<StationDefinedFrame>(
        model,
        "sdf",
        SimTK::CoordinateAxis::XCoordinateAxis(),
        SimTK::CoordinateAxis::YCoordinateAxis(),
        p1,
        p2,
        p3,
        origin
    );

    // add the to-be-joined-to body
    auto& body = EmplaceBody(
        model,
        "name",
        1.0,
        SimTK::Vec3{0.0, 0.0, 0.0},
        SimTK::Inertia{1.0, 1.0, 1.0}
    );

    EmplaceJoint<WeldJoint>(model,
        std::string{"weld"},
        sdf,
        SimTK::Vec3{1.0, 0.0, 0.0},  // location in parent
        SimTK::Vec3{},  // orientation in parent
        body,
        SimTK::Vec3{0.0, 1.0, 0.0},  // location in child
        SimTK::Vec3{}  // orientation in child
    );

    // fails in the same way that `PhysicalOffsetFrame` would (see sanity test above)
    //
    // remove the `REQUIRE` part and uncomment the other lines if you think you've fixed this
    REQUIRE_THROWS(model.buildSystem());
    // SimTK::State state = model.initializeState();
    // model.realizeReport(state);
}

TEST_CASE("StationDefinedFrame_ThrowsAtConnectionFinalizationIfPointAIsAtSameLocationAsPointB")
{
    Model model = CreateModelWithSDFPointsAt(
        "sdf",
        SimTK::Vec3{-1.0, -1.0, 0.0},
        SimTK::Vec3{-1.0, -1.0, 0.0},  // uh oh
        SimTK::Vec3{ 1.0,  0.0, 0.0},
        SimTK::Vec3{ 0.0,  0.0, 0.0}
    );

    // the property values `seem` ok (the implementation can't necessarily be sure where the stations are)
    model.finalizeFromProperties();

    // but, upon connecting everything, it fails because two points are at the same location in a base frame
    REQUIRE_THROWS(model.finalizeConnections());
}

TEST_CASE("StationDefinedFrame_ThrowsAtConnectionFinalizationIfPointAIsAtSameLocationAsPointC")
{
    Model model = CreateModelWithSDFPointsAt(
        "sdf",
        SimTK::Vec3{-1.0,  0.0, 0.0},
        SimTK::Vec3{ 0.0, -1.0, 0.0},
        SimTK::Vec3{-1.0,  0.0, 0.0},  // uh oh
        SimTK::Vec3{ 0.0,  0.0, 0.0}
    );

    // the property values `seem` ok (the implementation can't necessarily be sure where the stations are)
    model.finalizeFromProperties();

    // but, upon connecting everything, it fails because two points are at the same location in a base frame
    REQUIRE_THROWS(model.finalizeConnections());
}

TEST_CASE("StationDefinedFrame_ThrowsAtConnectionFinalizationIfPointBIsAtSameLocationAsPointC")
{
    Model model = CreateModelWithSDFPointsAt(
        "sdf",
        SimTK::Vec3{-1.0,  0.0, 0.0},
        SimTK::Vec3{ 0.0, -1.0, 0.0},
        SimTK::Vec3{ 0.0, -1.0, 0.0},  // uh oh
        SimTK::Vec3{ 0.0,  0.0, 0.0}
    );

    // the property values `seem` ok (the implementation can't necessarily be sure where the stations are)
    model.finalizeFromProperties();

    // but, upon connecting everything, it fails because two points are at the same location in a base frame
    REQUIRE_THROWS(model.finalizeConnections());
}

TEST_CASE("StationDefinedFrame_HasExpectedOriginLocation")
{
    SimTK::Vec3 originLoc = {-2.0, 1.0, 3.0};

    Model model = CreateModelWithSDFPointsAt(
        "sdf",
        SimTK::Vec3{-1.0,  0.0,  0.0},
        SimTK::Vec3{ 0.0, -1.0,  0.0},
        SimTK::Vec3{ 0.0,  0.0, -1.0},
        originLoc
    );

    // the property values `seem` ok (the implementation can't necessarily be sure where the stations are)
    model.buildSystem();
    SimTK::State state = model.initializeState();
    auto* c = model.findComponent<StationDefinedFrame>("sdf");
    REQUIRE(c != nullptr);
    REQUIRE(c->getTransformInGround(state).p() == originLoc);
}
