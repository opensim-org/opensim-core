#include <OpenSim/Simulation/Model/ForceProducer.h>

#include <catch2/catch_all.hpp>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/Model/ForceConsumer.h>

#include <cstddef>

using namespace OpenSim;

namespace
{
    // a trivial example of a `ForceProducer` for the purposes of testing
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

    // a trivial example of a `ForceConsumer` for the purposes of testing
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

        // TODO: requires creating `ForceProducer` and `Force` implementations that are "idential"
        //       and ensuring they have the same effect on the `bodyForces` and `mobilityForces`
        //       vectors after calling `Force::computeForce` on either of them.
    }
}
