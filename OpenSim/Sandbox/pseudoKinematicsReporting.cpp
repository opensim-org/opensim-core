
#include <OpenSim/OpenSim.h>

main() {
    // A study is the top level component that contains the model and other 
    // computational components. 
    Study kinematicsStudy;

    Model model("subject_01.osim");
    // previous simulation result with the same model
    StatesTrajectory states("states.ost");

    kinematicsStudy.addComponent(model);

    OutputReporter bodyKinematics;
    // Replace exactly what body kinematics analysis does
    for (const auto& body : model.getComponents<Body>()) {
        bodyKinematics.addOutput(body.getOutput("transfom"));
    }
    kinematicsStudy.addReporter(bodyKinematics);


    // Print out a list of all the available outputs from Frames in the model
    for (const auto& frame : model.getComponents<Frame>()) {
        for (const auto& output : frame.getOutputs()) {
            cout << "Frame " frame.getName();
            cout << " output: " << output.getName() << endl;
        }
    }

    OutputReporter pointKinematics;
    // any Frame in the model can give its transform(location, rotation) w.r.t ground
    pointKinematics.addOutput("subject_01/joints/knee_r/femur_r_offset/location");
    pointKinematics.addOutput("subject_01/markers/r_medial_epicondyle/location");
    pointKinematics.addOutput("subject_01/markers/r_lateral_epicondyle/location");
    // Add reporters to the Study a NOT a Model
    kinematicsStudy.addReporter(pointKinematics);

    // initialize the study and the underlying system used to solve for unknowns
    kinematicsStudy.initSystem();
    // realize the study to the reporting stage (time->pos->vel->acc->report)
    for (const auto& state : states)
        kinematicsStudy.realizeReport(state);

    // in memory results to be used for further processing
    const auto& bodyResults = bodyKinematics.getReport();
    // or simply write it to file
    FileAdapter fileAdapter;
    fileAdapter.write("bodyKinematics.sto", bodyResults);
    fileAdapter.write("pointKinematics.trc", pointKinematicsReporter.getReport());
}
