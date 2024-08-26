#include <OpenSim/Simulation/Model/ForceProducer.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;

namespace
{
    class ExampleForceProducer final : public ForceProducer {
        OpenSim_DECLARE_CONCRETE_OBJECT(ExampleForceProducer, ForceProducer);
    private:
        void implProduceForces(
            const SimTK::State& state,
            ForceConsumer& consumer) const final
        {
            // TODO
        }
    };
}

TEST_CASE("ForceProducer (ExampleForceProducer)")
{
    SECTION("Can Default Construct ExampleForceProducer")
    {
        ExampleForceProducer example;
    }
}
