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
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{1.0, 0.0, 0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 1.0, 0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 0.0, 1.0});
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
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{1.0, 0.0, 0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 1.0, 0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 0.0, 1.0});
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
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{1.0, 0.0, 0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 1.0, 0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 0.0, 1.0});
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
    model.buildSystem();
    SimTK::State state = model.initializeState();
    model.realizeReport(state);
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
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{1.0, 0.0, 0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 1.0, 0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 0.0, 1.0});
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
    model.buildSystem();
    SimTK::State state = model.initializeState();
    model.realizeReport(state);
}

TEST_CASE("StationDefinedFrame_ThrowsAtConnectionFinalizationIfPointAIsAtSameLocationAsPointB")
{
    Model model = CreateModelWithSDFPointsAt(
        "sdf",
        SimTK::Vec3{1.0, 0.0, 0.0},
        SimTK::Vec3{1.0, 0.0, 0.0},  // uh oh
        SimTK::Vec3{0.0, 0.0, 1.0},
        SimTK::Vec3{0.0, 0.0, 0.0}
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
        SimTK::Vec3{1.0, 0.0, 0.0},
        SimTK::Vec3{0.0, 1.0, 0.0},
        SimTK::Vec3{1.0, 0.0, 0.0},  // uh oh
        SimTK::Vec3{0.0, 0.0, 0.0}
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
        SimTK::Vec3{1.0, 0.0, 0.0},
        SimTK::Vec3{0.0, 1.0, 0.0},
        SimTK::Vec3{0.0, 1.0, 0.0},  // uh oh
        SimTK::Vec3{0.0, 0.0, 0.0}
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
        SimTK::Vec3{1.0, 0.0, 0.0},
        SimTK::Vec3{0.0, 1.0, 0.0},
        SimTK::Vec3{0.0, 0.0, 1.0},
        originLoc
    );

    // the property values `seem` ok (the implementation can't necessarily be sure where the stations are)
    model.buildSystem();
    SimTK::State state = model.initializeState();
    auto* c = model.findComponent<StationDefinedFrame>("sdf");
    REQUIRE(c != nullptr);
    REQUIRE(c->getTransformInGround(state).p() == originLoc);
}

TEST_CASE("StationDefinedFrame_OriginLocationCanBePointA")
{
    Model model;

    // add stations to the model
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{1.0, 2.0, 0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 1.0, 0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 0.0, 1.0});
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

    model.buildSystem();
    SimTK::State state = model.initializeState();
    auto* c = model.findComponent<StationDefinedFrame>("sdf");
    REQUIRE(c != nullptr);
    REQUIRE(c->getTransformInGround(state).p() == p1.get_location());
}

TEST_CASE("StationDefinedFrame_OriginLocationCanBePointB")
{
    Model model;

    // add stations to the model
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{1.0, 2.0, 0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 3.0, 0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 0.0, 1.0});
    auto& origin = p2;

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

    model.buildSystem();
    SimTK::State state = model.initializeState();
    auto* c = model.findComponent<StationDefinedFrame>("sdf");
    REQUIRE(c != nullptr);
    REQUIRE(c->getTransformInGround(state).p() == p2.get_location());
}

TEST_CASE("StationDefinedFrame_OriginLocationCanBePointC")
{
    Model model;

    // add stations to the model
    auto& p1 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{1.0, 2.0,  0.0});
    auto& p2 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 3.0,  0.0});
    auto& p3 = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 0.0, -7.0});
    auto& origin = p3;

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

    model.buildSystem();
    SimTK::State state = model.initializeState();
    auto* c = model.findComponent<StationDefinedFrame>("sdf");
    REQUIRE(c != nullptr);
    REQUIRE(c->getTransformInGround(state).p() == p3.get_location());
}

TEST_CASE("StationDefinedFrame_FinalizeConnectionsThrowsIfAPointIsAttachedToADifferentBaseFrame")
{
    Model model;

    auto& body = EmplaceBody(
        model,
        "name",
        1.0,
        SimTK::Vec3{0.0, 0.0, 0.0},
        SimTK::Inertia{1.0, 1.0, 1.0}
    );

    auto& p1InGround = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{1.0, 2.0,  0.0});
    auto& p2InGround = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 3.0,  0.0});
    auto& p3InBody = EmplaceModelComponent<Station>(model, body, SimTK::Vec3{0.0, 0.0, -7.0});  // uh oh
    auto& originInGround = p1InGround;

    // create an _invalid_ `StationDefinedFrame`
    //
    // it's invalid because one of the points has a different base frame
    auto& sdf = EmplaceModelComponent<StationDefinedFrame>(
        model,
        "sdf",
        SimTK::CoordinateAxis::XCoordinateAxis(),
        SimTK::CoordinateAxis::YCoordinateAxis(),
        p1InGround,
        p2InGround,
        p3InBody,
        originInGround
    );

    REQUIRE_THROWS(model.buildSystem());
}

TEST_CASE("StationDefinedFrame_FinalizeConnectionsThrowsIfOriginIsAttachedToADifferentBaseFrame")
{
    Model model;

    auto& body = EmplaceBody(
        model,
        "name",
        1.0,
        SimTK::Vec3{0.0, 0.0, 0.0},
        SimTK::Inertia{1.0, 1.0, 1.0}
    );

    auto& p1InGround = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{1.0, 2.0,  0.0});
    auto& p2InGround = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 3.0,  0.0});
    auto& p3InGround = EmplaceModelComponent<Station>(model, model.getGround(), SimTK::Vec3{0.0, 0.0, -7.0});
    auto& originInBody = EmplaceModelComponent<Station>(model, body, SimTK::Vec3{0.0, 0.0, 0.0});  // uh oh

    // create an _invalid_ `StationDefinedFrame`
    //
    // it's invalid because the origin is defined in a different base frame from the points
    auto& sdf = EmplaceModelComponent<StationDefinedFrame>(
        model,
        "sdf",
        SimTK::CoordinateAxis::XCoordinateAxis(),
        SimTK::CoordinateAxis::YCoordinateAxis(),
        p1InGround,
        p2InGround,
        p3InGround,
        originInBody
    );

    REQUIRE_THROWS(model.buildSystem());
}

TEST_CASE("StationDefinedFrame_CanDefineStationDefinedFrameUsingStationsDefinedInPofsAttachedToSameBaseFrame")
{
    // edge-case, but useful: it should be possible to define a `StationDefinedFrame` that uses `Station`s
    // that are defined in different `OffsetFrame<T>`s that are, in turn, all attached to the same base frame
    //
    // Example: using mesh landmark data. To simplify model-making, a user might want to:
    //
    // - load several meshes from an external source (e.g. a scanner)
    // - use `PhysicalOffsetFrame`s to attach + place all of those meshes in one body
    // - load landmark positions (stations) for each mesh into the model as stations. Those landmarks would
    //   be defined in the mesh's frame (i.e. the `PhysicalOffsetFrame`)
    // - define a `StationDefinedFrame` from those landmarks

    Model model;

    auto& body = EmplaceBody(
        model,
        "name",
        1.0,
        SimTK::Vec3{0.0, 0.0, 0.0},
        SimTK::Inertia{1.0, 1.0, 1.0}
    );

    // create pofs (for now, we're going to ignore the "what if the three points don't, after
    // applying these PoF transforms, form a triangle" logic that's surely going through your
    // head right now)
    auto& pof1 = EmplaceModelComponent<PhysicalOffsetFrame>(model, body, SimTK::Transform{});
    auto& pof2 = EmplaceModelComponent<PhysicalOffsetFrame>(model, body, SimTK::Transform{});
    auto& pof3 = EmplaceModelComponent<PhysicalOffsetFrame>(model, body, SimTK::Transform{});

    // create stations in those pofs/body
    auto& p1InPof1 = EmplaceModelComponent<Station>(model, pof1, SimTK::Vec3{1.0, 2.0,  0.0});
    auto& p2InPof2 = EmplaceModelComponent<Station>(model, pof2, SimTK::Vec3{0.0, 3.0,  0.0});
    auto& p3InPof3 = EmplaceModelComponent<Station>(model, pof3, SimTK::Vec3{0.0, 0.0, -7.0});
    auto& originInBody = EmplaceModelComponent<Station>(model, body, SimTK::Vec3{0.0, 0.0, 0.0});

    // create a `StationDefinedFrame` from the stations
    auto& sdf = EmplaceModelComponent<StationDefinedFrame>(
        model,
        "sdf",
        SimTK::CoordinateAxis::XCoordinateAxis(),
        SimTK::CoordinateAxis::YCoordinateAxis(),
        p1InPof1,
        p2InPof2,
        p3InPof3,
        originInBody
    );

    // should be completely fine
    REQUIRE_NOTHROW(model.buildSystem());

    // ... and the frame of the `StationDefinedFrame` is the base frame (the body)...
    REQUIRE(&sdf.findBaseFrame() == &body);
}

TEST_CASE("StationDefinedFrame_FinalizeConnectionsThrowsIfPofOffsetCausesPointsToCoincide")
{
    // edge-edge-case: if the `Station`s of a `StationDefinedFrame` are attached via different
    // `PhysicalOffsetFrame`s that, after applying the offset transformation, cause points of
    // the frame to coincide (big no no), that should cause the same connection-time exception
    // that co-inciding them on the same frame would've caused

    Model model;

    SimTK::Vec3 pos = {3.0, 0.0, 0.0};  // this'll be negated by the PoF's transform
    auto& identityPof = EmplaceModelComponent<PhysicalOffsetFrame>(model, model.getGround(), SimTK::Transform{});
    auto& repositioningPof = EmplaceModelComponent<PhysicalOffsetFrame>(model, model.getGround(), SimTK::Transform{-pos});

    // at origin
    auto& p1 = EmplaceModelComponent<Station>(model, identityPof, SimTK::Vec3{0.0, 0.0, 0.0});
    // somewhere else
    auto& p2 = EmplaceModelComponent<Station>(model, identityPof, SimTK::Vec3{2.0, 2.0, 0.0});
    // somewhere else else, _but_ will be repositioned to origin by the PoF
    auto& p3 = EmplaceModelComponent<Station>(model, repositioningPof, pos);
    auto& origin = p2;

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

    // throws, because the *base frame location* of `p3` == `p1`
    REQUIRE_THROWS(model.buildSystem());
}

TEST_CASE("StationDefinedFrame_BasicModelWithStationDefinedFrameOsimLoadsFine")
{
    // ensures the implementation can load a very basic `osim` file containing one
    // `StationDefinedFrame` defined in ground

    Model model{"BasicModelWithStationDefinedFrame.osim"};
    REQUIRE_NOTHROW(model.buildSystem());
    SimTK::State state = model.initializeState();
    model.realizeReport(state);
}
