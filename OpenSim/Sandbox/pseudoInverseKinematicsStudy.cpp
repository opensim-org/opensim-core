
#include <OpenSim/OpenSim.h>

main() {
    // A study is the top level component that contains the model and other 
    // computational components.
    Study inverseKinematicsStudy;

    Model model("subject_01.osim");
    inverseKinematicsStudy.addComponent(model);
    
    // A data Source component wraps a TimeSeriesTables and access rows by time
    // Each column is given its own Output by name unless specified otherwise
    Source markers("subject01_trial2.trc");
    // Source markers("subject01_trial2.trc", Vec2(0, 5)); // by range
    // Source markers("subject01_trial2.trc", {"aa, "ai", ac"}); //by name
    inverseKinematicsStudy.addComponent(markers);
    
    // InverseKinematicsSolver is wrapped by a component (or one itself?)
    // dependency on model, and marker inputs (outputs of a source) wired
    // upon connect
    InverseKinematics ik(model, markers);
    inverseKinematicsStudy.addComponent(ik);

    // Now handle IK Outputs
    // Extract the outputs from the model and the state via a reporter
    // Do not need to know type BUT cannot combine different types
    // Coordinates are doubles and modelMarkers are Vec3's
    OutputReporter coordinateReporter();
    OutputReporter modelMarkerReporter();
    // Output coordinates from IK
    for(const auto& coord: mode.getCoordinates())
        coordinateReporter.addOutput(coord.getOutput("value"));
    inverseKinematicsStudy.addComponent(coordinateReporter);
    // Output model marker locations from IK
    for (const auto& marker : mode.getMarkers())
        modelMarkerReporter.addOutputs(marker.getOutput("location"));
    inverseKinematicsStudy.addComponent(modelMarkerReporter);

    State& state = inverseKinematicsStudy.initSystem();

    // March through time to solve for the state's coordinates
    for (double t : markers.getTimes()) {
        state.setTime(t);
        ik.track(state);
        inverseKinematicsStudy.realizeReport(state);
    }

    // write results to file
    FileAdapter fileAdapter;
    fileAdapter.write("s01_tr2_IK.mot", coordinateReporter.getReport());
    fileAdapter.write("s01_tr2_markers.trc", modelMarkerReporter.getReport());
}
