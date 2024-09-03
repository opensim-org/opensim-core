#include <OpenSim/Simulation/Model/ForceProducer.h>

#include <catch2/catch_all.hpp>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/Model/ForceAdapter.h>
#include <OpenSim/Simulation/Model/ForceConsumer.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

#include <algorithm>
#include <cstddef>

using namespace OpenSim;

namespace
{
    // A trivial example of a `ForceProducer` for the purposes of testing.
    class ExampleForceProducer final : public ForceProducer {
        OpenSim_DECLARE_CONCRETE_OBJECT(ExampleForceProducer, ForceProducer);
    public:
        ExampleForceProducer() = default;

        ExampleForceProducer(
            size_t numGeneralizedForcesToProduce,
            size_t numBodySpatialVectorsToProduce,
            size_t numPointForcesToProduce) :

            _numGeneralizedForcesToProduce{numGeneralizedForcesToProduce},
            _numBodySpatialVectorsToProduce{numBodySpatialVectorsToProduce},
            _numPointForcesToProduce{numPointForcesToProduce}
        {}
    private:
        void implProduceForces(
            const SimTK::State& state,
            ForceConsumer& consumer) const final
        {
            for (size_t i = 0; i < _numGeneralizedForcesToProduce; ++i) {
                consumer.consumeGeneralizedForce(state, _dummyCoordinate, static_cast<double>(i));
            }
            for (size_t i = 0; i < _numBodySpatialVectorsToProduce; ++i) {
                const SimTK::Vec3 torque{static_cast<double>(i)};
                const SimTK::Vec3 force{static_cast<double>(i)};
                consumer.consumeBodySpatialVec(state, _dummyBody, SimTK::SpatialVec{torque, force});
            }
            for (size_t i = 0; i < _numPointForcesToProduce; ++i) {
                const SimTK::Vec3 point{static_cast<double>(i)};
                const SimTK::Vec3 force{static_cast<double>(i)};
                consumer.consumePointForce(state, _dummyBody, point, force);
            }
        }

        OpenSim::Coordinate _dummyCoordinate;
        OpenSim::Body _dummyBody;
        size_t _numGeneralizedForcesToProduce = 0;
        size_t _numBodySpatialVectorsToProduce = 0;
        size_t _numPointForcesToProduce = 0;
    };

    // A trivial example of a `ForceConsumer` for the purposes of testing.
    class ExampleForceConsumer final : public ForceConsumer {
    public:
        size_t getNumGeneralizedForcesConsumed() const { return _numGeneralizedForcesConsumed; }
        size_t getNumBodySpatialVectorsConsumed() const { return _numBodySpatialVectorsConsumed; }
        size_t getNumPointForcesConsumed() const { return _numPointForcesConsumed; }

    private:

        void implConsumeGeneralizedForce(const SimTK::State&, const Coordinate&, double) final
        {
            ++_numGeneralizedForcesConsumed;
        }

        void implConsumeBodySpatialVec(const SimTK::State&, const PhysicalFrame&, const SimTK::SpatialVec&) final
        {
            ++_numBodySpatialVectorsConsumed;
        }

        void implConsumePointForce(const SimTK::State&, const PhysicalFrame&, const SimTK::Vec3&, const SimTK::Vec3&) final
        {
            ++_numPointForcesConsumed;
        }

        size_t _numGeneralizedForcesConsumed = 0;
        size_t _numBodySpatialVectorsConsumed = 0;
        size_t _numPointForcesConsumed = 0;
    };

    // A trivial implementation of a `ForceProducer` that can be added to a `Model`
    // for the purposes of comparing side-effects to the `Force` API (see `MockForce`
    // below).
    class MockForceProducer final : public ForceProducer {
        OpenSim_DECLARE_CONCRETE_OBJECT(MockForceProducer, ForceProducer);
    public:
        OpenSim_DECLARE_SOCKET(force_target, OpenSim::PhysicalFrame, "the physical frame that forces should be produced for");
        OpenSim_DECLARE_SOCKET(generalized_force_target, OpenSim::Coordinate, "the coordinate that generalized forces should be produced for");

        explicit MockForceProducer(
            const OpenSim::PhysicalFrame& forceTarget,
            const OpenSim::Coordinate& generalizedForceTarget)
        {
            connectSocket_force_target(forceTarget);
            connectSocket_generalized_force_target(generalizedForceTarget);
        }

    private:
        void implProduceForces(const SimTK::State& state, ForceConsumer& consumer) const final
        {
            // Note: this should be logically equivalent to `MockForce::computeForce` (below), so
            // that the `ForceProducer`'s default `computeForce` implementations can be compared
            // in an end-to-end way.

            consumer.consumeBodySpatialVec(
                state,
                getConnectee<OpenSim::PhysicalFrame>("force_target"),
                SimTK::SpatialVec{SimTK::Vec3{1.0}, SimTK::Vec3{2.0}}
            );

            consumer.consumeBodySpatialVec(
                state,
                getConnectee<OpenSim::PhysicalFrame>("force_target"),
                SimTK::SpatialVec{SimTK::Vec3{-0.5}, SimTK::Vec3{-0.25}}
            );

            consumer.consumeTorque(
                state,
                getConnectee<OpenSim::PhysicalFrame>("force_target"),
                SimTK::Vec3{0.1, 0.25, 0.5}
            );

            consumer.consumeGeneralizedForce(
                state,
                getConnectee<OpenSim::Coordinate>("generalized_force_target"),
                2.0
            );

            consumer.consumePointForce(
                state,
                getConnectee<OpenSim::PhysicalFrame>("force_target"),
                SimTK::Vec3{1.0, 2.0, 3.0},
                SimTK::Vec3{-1.0, -3.0, -9.0}
            );
        }
    };

    // A trivial implementation of a `Force` that can be added to a `Model`
    // for the purposes of comparing side-effects to the `ForceProducer` API
    // (see `MockForceProducer` above).
    class MockForce final : public Force {
        OpenSim_DECLARE_CONCRETE_OBJECT(MockForce, Force);
    public:
        OpenSim_DECLARE_SOCKET(force_target, OpenSim::PhysicalFrame, "the physical frame that forces should be produced for");
        OpenSim_DECLARE_SOCKET(generalized_force_target, OpenSim::Coordinate, "the coordinate that generalized forces should be produced for");

        explicit MockForce(
            const OpenSim::PhysicalFrame& forceTarget,
            const OpenSim::Coordinate& generalizedForceTarget)
        {
            connectSocket_force_target(forceTarget);
            connectSocket_generalized_force_target(generalizedForceTarget);
        }

        void computeForce(
            const SimTK::State& state,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& generalizedForces) const override
        {
            // Note: this should be logically equivalent to `MockForceProducer::implProduceForces`
            // (above), so that the `ForceProducer`'s default `computeForce` implementations can
            // be compared in an end-to-end way.

            // (this is usually how legacy `Force` code adds `SimTK::SpatialVec`s to the body forces)
            bodyForces[getConnectee<OpenSim::PhysicalFrame>("force_target").getMobilizedBodyIndex()] +=
                SimTK::SpatialVec{SimTK::Vec3{1.0}, SimTK::Vec3{2.0}};
            bodyForces[getConnectee<OpenSim::PhysicalFrame>("force_target").getMobilizedBodyIndex()] +=
                SimTK::SpatialVec{SimTK::Vec3{-0.5}, SimTK::Vec3{-0.25}};

            applyTorque(
                state,
                getConnectee<OpenSim::PhysicalFrame>("force_target"),
                SimTK::Vec3{0.1, 0.25, 0.5},
                bodyForces
            );

            applyGeneralizedForce(
                state,
                getConnectee<OpenSim::Coordinate>("generalized_force_target"),
                2.0,
                generalizedForces
            );

            applyForceToPoint(
                state,
                getConnectee<OpenSim::PhysicalFrame>("force_target"),
                SimTK::Vec3{1.0, 2.0, 3.0},
                SimTK::Vec3{-1.0, -3.0, -9.0},
                bodyForces
            );
        }
    };

    // Returns true if the contents of `a` and `b` are equal, that is, they have the same
    // number of elements, and each element in `a` compares equal with the element in `b`
    // at the same position.
    template<typename T>
    bool equals(const SimTK::Vector_<T>& a, const SimTK::Vector_<T>& b)
    {
        if (a.size() != b.size()) {
            return false;
        }
        for (int i = 0; i < a.size(); ++i) {
            if (a[i] != b[i]) {
                return false;
            }
        }
        return true;
    }

    // Represents the vectors of forces the simbody physics engine lets downstream
    // code (e.g. `OpenSim::Force`s) manipulate
    //
    // They're bundled together in this struct for ease of use, initialization, and comparison.
    struct SimbodyEngineForceVectors {

        // Constructs a zero-initialized set of vectors.
        explicit SimbodyEngineForceVectors(const SimTK::SimbodyMatterSubsystem& matter) :
            bodyForces{matter.getNumBodies(), SimTK::SpatialVec{SimTK::Vec3{0.0}, SimTK::Vec3{0.0}}},
            particleForces{matter.getNumParticles(), SimTK::Vec3{0.0}},
            mobilityForces{matter.getNumMobilities(), double{}}
        {}

        SimTK::Vector_<SimTK::SpatialVec> bodyForces;
        SimTK::Vector_<SimTK::Vec3> particleForces;  // unused by OpenSim
        SimTK::Vector mobilityForces;
    };
}

TEST_CASE("ForceProducer (ExampleForceProducer)")
{
    SECTION("Can Default Construct `ExampleForceProducer`")
    {
        // This test is mostly just ensuring that the example components for
        // the test suite behave correctly. That is, inheriting from the
        // `ForceProducer` API should work in trivial cases

        const ExampleForceProducer example;
    }

    SECTION("Passing `ExampleForceConsumer` to default-constructed `ExampleForceProducer` produces no forces")
    {
        // This test is checking that the API lets calling code pass an example
        // consumer into an example producer. It's ensuring that the concrete
        // public API talks to the relevant virtual APIs in this trivial (0-force)
        // case.

        const ExampleForceProducer producer;
        ExampleForceConsumer consumer;
        const SimTK::State state;  // untouched by example classes
        producer.produceForces(state, consumer);

        REQUIRE(consumer.getNumGeneralizedForcesConsumed() == 0);
        REQUIRE(consumer.getNumBodySpatialVectorsConsumed() == 0);
        REQUIRE(consumer.getNumPointForcesConsumed() == 0);
    }

    SECTION("Passing Non-Zero Number of Generalized Forces to `ExampleForceProducer` makes it produce stubbed generalized forces into the `ExampleForceConsumer`")
    {
        // This test is checking that a generalized force produced by an (example)
        // `ForceProducer` will correctly worm its way into an (example) `ForceConsumer`

        const size_t numGeneralizedForcesToProduce = 7;
        const ExampleForceProducer producer{numGeneralizedForcesToProduce, 0, 0};

        ExampleForceConsumer consumer;
        const SimTK::State state;  // untouched by example classes
        producer.produceForces(state, consumer);

        REQUIRE(consumer.getNumGeneralizedForcesConsumed() == numGeneralizedForcesToProduce);
        REQUIRE(consumer.getNumBodySpatialVectorsConsumed() == 0);
        REQUIRE(consumer.getNumPointForcesConsumed() == 0);
    }

    SECTION("Passing Non-Zero Number of Body Spatial Vectors to `ExampleForceProducer` makes it produce stubbed spatial vectors into the `ExampleForceConsumer`")
    {
        // This test is checking that a body spatial vector produced by an (example)
        // `ForceProducer` will correctly worm its way into an example `ForceConsumer`
        const size_t numBodySpatialVectorsToProduce = 9;
        const ExampleForceProducer producer{0, numBodySpatialVectorsToProduce, 0};

        ExampleForceConsumer consumer;
        const SimTK::State state;  // untouched by example classes
        producer.produceForces(state, consumer);

        REQUIRE(consumer.getNumGeneralizedForcesConsumed() == 0);
        REQUIRE(consumer.getNumBodySpatialVectorsConsumed() == numBodySpatialVectorsToProduce);
        REQUIRE(consumer.getNumPointForcesConsumed() == 0);
    }

    SECTION("Passing Non-Zero Number of Point Force Vectors to `ExampleForceProducer` makes it produce stubbed point forces into the `ExampleForceConsumer`")
    {
        // This test is checking that a body spatial vector produced by an (example)
        // `ForceProducer` will correctly worm its way into an example `ForceConsumer`
        const size_t numPointForcesToProduce = 11;
        const ExampleForceProducer producer{0, 0, numPointForcesToProduce};

        ExampleForceConsumer consumer;
        const SimTK::State state;  // untouched by example classes
        producer.produceForces(state, consumer);

        REQUIRE(consumer.getNumGeneralizedForcesConsumed() == 0);
        REQUIRE(consumer.getNumBodySpatialVectorsConsumed() == 0);
        REQUIRE(consumer.getNumPointForcesConsumed() == numPointForcesToProduce);
    }

    SECTION("Setting `produceForces` to `false` Causes `ExampleForceProducer` to Produce No Forces")
    {
        // This test is ensuring that `appliesForce`, which `ForceProducer` inherits from the
        // `Force` base class, is obeyed by the `ForceProducer` API without each downstream
        // class having to check `appliesForce` (`ExampleForceProducer` doesn't check it).

        ExampleForceProducer producer{1, 2, 3};  // produce nonzero number of forces
        producer.set_appliesForce(false);  // from `OpenSim::Force`
        producer.finalizeFromProperties();

        ExampleForceConsumer consumer;
        const SimTK::State state;  // untouched by example classes
        producer.produceForces(state, consumer);

        REQUIRE(consumer.getNumGeneralizedForcesConsumed() == 0);
        REQUIRE(consumer.getNumBodySpatialVectorsConsumed() == 0);
        REQUIRE(consumer.getNumPointForcesConsumed() == 0);
    }

    SECTION("The `ForceProducer` class's default `computeForce` Implementation Works as Expected")
    {
        // The `ForceProducer` base class provides a default implementation of `Force::computeForce`,
        // which should behave identically to it in the case where the downstream code uses the
        // `ForceConsumer` API "identially" (logically speaking) to the `Force` API.
        //
        // For example, when a concrete implementation calls `ForceConsumer::consumePointForce` in
        // its `implProduceForces` implementation during a call to  `ForceProducer::computeForce`,
        // that should have identical side-effects as when a concrete implementation calls
        // `Force::applyForceToPoint` during a call to `Force::computeForce` (assuming the
        // same arguments, conditions, etc.).
        //
        // I.e. "The `ForceProducer::computeForce` API should behave logically identically to the
        //      `Force::computeForce` API. It's just that the `ForceProducer` API allows for
        //      switching consumers' behavior.

        // step 1) build a model with "equivalent" `ForceProducer` and `Force` implementations
        Model model;
        auto* body          = new Body{"body", 1.0, SimTK::Vec3{0.0}, SimTK::Inertia{1.0}};
        auto* joint         = new FreeJoint{"joint", model.getGround(), *body};
        auto* force         = new MockForce{*body, joint->get_coordinates(0)};
        auto* forceProducer = new MockForceProducer{*body, joint->get_coordinates(0)};
        model.addBody(body);
        model.addJoint(joint);
        model.addForce(force);
        model.addForce(forceProducer);
        model.buildSystem();
        model.initializeState();

        // step 2) create zero-initialized force vectors "as if" pretending to be the physics engine
        SimbodyEngineForceVectors blankForceVectors{model.getMatterSubsystem()};
        SimbodyEngineForceVectors forceVectors{model.getMatterSubsystem()};
        SimbodyEngineForceVectors forceProducerVectors{model.getMatterSubsystem()};

        // step 3a) pump one set of the vectors through the `Force` implementation
        {
            ForceAdapter adapter{*force};
            adapter.calcForce(model.getWorkingState(), forceVectors.bodyForces, forceVectors.particleForces, forceVectors.mobilityForces);
        }
        // step 3b) pump the other set of vectors through the `ForceProducer` implementation
        {
            ForceAdapter adapter{*forceProducer};
            adapter.calcForce(model.getWorkingState(), forceProducerVectors.bodyForces, forceProducerVectors.particleForces, forceProducerVectors.mobilityForces);
        }

        // step 4) compare the vector sets, which should be equal if `ForceProducer` behaves the same
        //         as `Force` for typical use-cases
        REQUIRE((!equals(forceVectors.bodyForces, blankForceVectors.bodyForces) && equals(forceVectors.bodyForces, forceProducerVectors.bodyForces)));
        REQUIRE(equals(forceVectors.particleForces, forceProducerVectors.particleForces));  // should be blank (untouched by OpenSim)
        REQUIRE((!equals(forceVectors.bodyForces, blankForceVectors.bodyForces) && equals(forceVectors.mobilityForces, forceProducerVectors.mobilityForces)));
    }
}
