
#include "InverseMuscleSolver.h"

using namespace OpenSim;

InverseMuscleSolver::InverseMuscleSolver() {
    constructProperties();
}

InverseMuscleSolver::InverseMuscleSolver(const std::string& setupFilePath) :
        Object(setupFilePath) {
    constructProperties();
}

void InverseMuscleSolver::constructProperties() {
    constructProperty_model_file("");
    constructProperty_kinematics_file("");
    constructProperty_lowpass_cutoff_frequency_for_joint_moments(-1);
    constructProperty_create_reserve_actuators(-1);
}

void InverseMuscleSolver::loadModelAndKinematicsData(Model& model,
        TimeSeriesTable& kinematics) const {
    using SimTK::Pathname;
    // Get the directory containing the setup file.
    std::string setupDir;
    {
        bool dontApplySearchPath;
        std::string fileName, extension;
        Pathname::deconstructPathname(getDocumentFileName(),
                dontApplySearchPath, setupDir, fileName, extension);
    }
    OPENSIM_THROW_IF_FRMOBJ(!get_model_file().empty() && _model, Exception,
            "A model has been specified via the model_file "
            "property AND via setModel(); only one of these two mechanisms "
            "can be used. Consider setting the model_file "
            "property to an empty string.");
    if (!get_model_file().empty()) {
        // The model file path may be relative to the setup dir.
        std::string modelFilePath =
                Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(
                        setupDir, get_model_file());
        model = Model(modelFilePath);
        model.finalizeFromProperties();
    } else if (_model) {
        // The user called setModel().
        model = Model(*_model.get());
    } else {
        OPENSIM_THROW_FRMOBJ(Exception, "No model specified.");
    }

    OPENSIM_THROW_IF_FRMOBJ(!get_kinematics_file().empty() && _kinematics,
            Exception,
            "Kinematics have been specified via the kinematics_file "
            "property AND via setKinematicsData(); only one of these two "
            "mechanisms can be used. Consider setting the kinematics_file "
            "property to an empty string.");
    if (!get_kinematics_file().empty()) {
        // The kinematics file path may be relative to the setup dir.
        std::string kinematicsFilePath =
                Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(
                        setupDir, get_kinematics_file());
        DataAdapter::OutputTables tables =
                FileAdapter::readFile(kinematicsFilePath);

        // There should only be one table.
        OPENSIM_THROW_IF_FRMOBJ(tables.size() != 1, Exception,
                "Expected kinematics file '" + get_kinematics_file() +
                "' to contain 1 table, but it contained " +
                std::to_string(tables.size()) + " tables.");

        // Get the first table.
        auto* firstTable =
                dynamic_cast<TimeSeriesTable*>(tables.begin()->second.get());
        OPENSIM_THROW_IF_FRMOBJ(!firstTable, Exception,
                "Expected kinematics file to contain a (scalar) "
                "TimeSeriesTable, but it contains a different type of table.");

        kinematics = *firstTable;
    } else if (_kinematics) {
        // The user called setKinematicsData().
        kinematics = TimeSeriesTable(*_kinematics.get());
    } else {
        OPENSIM_THROW_FRMOBJ(Exception, "No kinematics data specified.");
    }
    OPENSIM_THROW_IF_FRMOBJ(kinematics.getNumRows() == 0, Exception,
            "The provided kinematics table has no rows.");
}
