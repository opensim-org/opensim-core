
#include <OpenSim/OpenSim.h>
#include <OpenSim/Common/TableSource.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/Model/Point.h>

using namespace OpenSim;
using namespace SimTK;

//==============================================================================
//                               ExperimentalMarker
//==============================================================================
/**
ExperimentalMarker is a concrete Point that represents the experimental value of
a Marker and can be used to display that the marker location in the OpenSim
Visualizer.

Unlike its model Marker counterpart, an ExperimentalMarker is a measured value
and not physically attached to the musculoskeletal model. It is assumed that its 
value is its location with respect to the lab (Ground) frame.

The location in Ground value of an ExperimentalMarker is obtained from its
Input<Vec3>("location_in_ground").

@authors Ajay Seth
**/
class ExperimentalMarker : public Point {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExperimentalMarker, Point);
public:
    ExperimentalMarker() { constructProperties(); }

    OpenSim_DECLARE_PROPERTY(radius, double,
        "The radius of the sphere used to display the ExperimentalMarker.");

    OpenSim_DECLARE_PROPERTY(color, SimTK::Vec3,
        "The color of the sphere used to display the ExperimentalMarker.");

    OpenSim_DECLARE_PROPERTY(default_location, SimTK::Vec3,
        "The default location of the ExperimentalMarker in Ground.");

    OpenSim_DECLARE_INPUT(location_in_ground, SimTK::Vec3, SimTK::Stage::Time,
        "Provide ExperimentalMarker location_in_ground the Ground.");

private:
    void constructProperties() {
        constructProperty_default_location(SimTK::Vec3(SimTK::NaN));
        constructProperty_radius(0.005);
        constructProperty_color(SimTK::Vec3(0.9, 0.9, 0.2));
    }

    void extendFinalizeFromProperties() override {
        _locationInGround = get_default_location();
    }

    /* Calculate the location with respect to and expressed in Ground */
    SimTK::Vec3 calcLocationInGround(const SimTK::State& s) const override {
        return getInput<SimTK::Vec3>("location_in_ground").getValue(s);
    }

    SimTK::Vec3 calcVelocityInGround(const SimTK::State& s) const override {
        return SimTK::Vec3(NaN);
    }

    SimTK::Vec3 calcAccelerationInGround(const SimTK::State& s) const override {
        return SimTK::Vec3(NaN);
    }

    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const override
    {
        Super::generateDecorations(fixed, hints, state, appendToThis);
        if (!fixed && hints.get_show_markers()) {
            appendToThis.push_back(
                SimTK::DecorativeSphere(get_radius()).setBodyId(0)
                .setColor(get_color()).setOpacity(0.5)
                .setTransform(getLocationInGround(state)));
            appendToThis.push_back(SimTK::DecorativeText(getName()).setBodyId(0)
                .setTransform(getLocationInGround(state))
                .setScaleFactors(SimTK::Vec3(get_radius()*0.5)));
        }
    }

    mutable SimTK::Vec3 _locationInGround{ SimTK::NaN };
}; // End of class ExperimentalMarker



void previewMarkerData(const std::string& markerDataFile)
{
    Model markerWorld;
    // Load the marker data into a TableSource that has markers
    // as its output which each markers occupying its own channel
    TableSourceVec3* markersSource = new TableSourceVec3(markerDataFile);
    // Add the markersSource Component to the model
    markerWorld.addComponent(markersSource);

    // Get the underlying Table backing the marker Source so we 
    // know how many markers we have and their names
    const auto& markerData = markersSource->getTable();

    // Create an ExperimentMarker Component for every column in the markerData 
    for (size_t i = 0; i < markerData.getNumColumns(); ++i) {
        auto marker = new ExperimentalMarker();
        marker->setName(markerData.getColumnLabel(i));
        marker->set_default_location(markerData.getMatrix()(0, i));
        // markers are owned by the model
        markerWorld.addComponent(marker);
        // the time varying location of the marker comes from the markersSource
        // Component
        marker->updInput("location_in_ground").connect(
            markersSource->getOutput("column").getChannel(markerData.getColumnLabel(i)));
    }

    markerWorld.setUseVisualizer(true);
    SimTK::State& state = markerWorld.initSystem();

    char c;
    std::cout << "press any key to visualize '" << markerDataFile <<
        "' data ..." << std::endl;
    std::cin >> c;

    auto& times = markerData.getIndependentColumn();
    for (size_t j = 0; j < times.size(); ++j) {
        std::cout << "time: " << times[j] << "s" << std::endl;
        state.setTime(times[j]);
        markerWorld.realizePosition(state);
        markerWorld.getVisualizer().show(state);
    }
}

int main() {
    previewMarkerData("futureMarkerPreviewData.trc");

return 0;
}



