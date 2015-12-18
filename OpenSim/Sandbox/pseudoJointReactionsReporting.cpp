
#include <OpenSim/OpenSim.h>

main() {
    // A study is the top level component that contains the model and other 
    // computational components. 
    Study jointReactionsStudy;

    Model model("subject_02.osim");
    StatesTrajectory states("states.ost");
    // Read-in the external loads from a source
    FileAdapter fileAdpater;"
    auto grfs = filedapter.read("subject_02_grfs.mot", "forces");

    // External force created from table, with force/torque/point column labels,
    // to which frame (PhysicalFrame) the force is applied to and expressed in.
    // ExternalForce(table, forceColumn, torqueColumn, pointColumn, appliedTo, expressedIn)
    ExternalForce rGrf(grfs, "force_r", "torque_r", "point_r", "foot_r", "ground");
    ExternalForce lGrf(grfs, "force_l", "torque_l", "point_l", "foot_l", "ground");
    model.addForce(rGrf);
    model.addForce(lGrf);
    // Access joint by name
    const Joint& rightKnee = model.getComponent<Joint>("knee_r");
    const PhysicalFrame& parent = rightKnee.getParentFrame();
    const PhysicalFrame& child = rightKnee.getChildFrame();

    // Use operator to reexpress a vector input in one frame to another frame
    // A reexpress operator reexpress
    ReexpressOperator reexpressor(parent.getName(), ground);
    // By default reaction force is that on the child and expressed in the parent, 
    // therefore, rightKnee output returns force on tibia w.r.t. the femur
    reexpressor.setInput(rightKnee.getOutput("reaction_force"));
    OutputReporter jointReactionsReporter;
    // second argument allows reporter to relabel or alias the output in the reporter
    jointReactionsReporter.addOuput(reexpressor.getOutput("reexpressed"), "knee_reaction_force");

    // put together the study
    jointReactionsStudy.addComponent(model);
    jointReactionsStudy.addComponent(reexpressor);
    jointReactionsStudy.addReporter(jointReactionsReporter);

    // initialize the model and underlying system
    jointReactionsStudy.initSystem();
    // realize the model to the reporting stage for each state of interest
    for (const auto& state : states)
        jointReactionsStudy.realizeReport(state);

    // access results to pass on for further analysis
    auto results = jointReactionsReporter.getResults();
    
    /** Current Interface for specifying meta data 
    DependentsMetaData forcesMetaData = results.getDependentsMetaData();
    // define meta data to associate reference frames with joint reaction forces
    // in what frame is the spatial force applied to?
    ValueArray<string> appliedToFrame(results.getNumCols());
    // in what frame is spatial force (torque and point of application) expressed?
    ValueArray<string> expressedinFrame(results.getNumCols());

    size_t index = forcesMetaData.getIndexForLabel("knee_reaction_force");
    appliedToFrame[index] = child.getName();
    expressedinFrame[index] = parent.getName();
    forcesMetaData.setValueArrayForKey("applied_to_frame", appliedToFrame);
    forcesMetaData.setValueArrayForKey("exprressed_in_frame", expressedinFrame);

    // update the meta data of reaction forces table
    results.setDependentsMetaData(forcesMetaData);
    */

    // Proposed interface for specifying meta data
    // For indepenent column it could be updIndependentColumnMetaData, but
    // for the main block of dependent data we could lose the Dependents qualifier
    MetaData& forcesMetaData = results.updMetaData(); 
    // Add field with a default value with each dependent data column
    // is templated by type but defaults to string.
    forcesMetaData.addField("applied_to_frame", "");
    // By default external forces are expressed in ground
    forcesMetaData.addField("exprressed_in_frame", "ground");
    // Update the meta data associated with the reaction force we reexpressed
    forcesMetaData["knee_reaction_force"].updFieldValue("applied_to_frame") = child.getName();
    forcesMetaData["knee_reaction_force"].updFieldValue("exprressed_in_frame") = parent.getName();

    // write it to file
    fileAdapter.write("reactionForces.sto", results);
}
