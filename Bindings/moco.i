%newobject *::clone;

/* To recognize SimTK::RowVector in header files (TODO: move to simbody.i) */

%include <OpenSim/Moco/osimMocoDLL.h>

%include <OpenSim/Moco/About.h>

%include <OpenSim/Moco/MocoScaleFactor.h>
%include <OpenSim/Moco/MocoBounds.h>
%include <OpenSim/Moco/MocoVariableInfo.h>

namespace OpenSim {
    %ignore MocoGoal::IntegrandInput;
    %ignore MocoGoal::calcIntegrand;
    %ignore MocoGoal::GoalInput;
    %ignore MocoGoal::calcGoal;
}
%include <OpenSim/Moco/MocoGoal/MocoGoal.h>
%template(SetMocoWeight) OpenSim::Set<OpenSim::MocoWeight, OpenSim::Object>;
%include <OpenSim/Moco/MocoWeightSet.h>
%include <OpenSim/Moco/MocoGoal/MocoStateTrackingGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoMarkerTrackingGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoMarkerFinalGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoControlGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoControlTrackingGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoContactTrackingGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoContactImpulseTrackingGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoInitialActivationGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoJointReactionGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoSumSquaredStateGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoOrientationTrackingGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoTranslationTrackingGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoAccelerationTrackingGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoAngularVelocityTrackingGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoOutputGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoPeriodicityGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoInitialForceEquilibriumDGFGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoInitialVelocityEquilibriumDGFGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoStepTimeAsymmetryGoal.h>
%include <OpenSim/Moco/MocoGoal/MocoStepLengthAsymmetryGoal.h>
%include <OpenSim/Moco/MocoConstraintInfo.h>

%template(StdVectorMocoBounds) std::vector<OpenSim::MocoBounds>;

%ignore OpenSim::MocoMultibodyConstraint::getKinematicLevels;
%ignore OpenSim::MocoProblemRep::getMultiplierInfos;

%include <OpenSim/Moco/MocoConstraint.h>

%include <OpenSim/Moco/MocoControlBoundConstraint.h>
%include <OpenSim/Moco/MocoFrameDistanceConstraint.h>
%include <OpenSim/Moco/MocoOutputConstraint.h>

%include <OpenSim/Moco/MocoProblemRep.h>

// MocoProblemRep() is not copyable, but by default, SWIG tries to make a copy
// when wrapping createRep().
%rename(createRep) OpenSim::MocoProblem::createRepHeap;

namespace OpenSim {
    %ignore MocoPhase::setModel(Model);
    %ignore MocoPhase::setModel(std::unique_ptr<Model>);
    %ignore MocoProblem::setModel(Model);
    %ignore MocoProblem::setModel(std::unique_ptr<Model>);
}

%extend OpenSim::MocoPhase {
    void setModel(Model* model) {
        $self->setModel(std::unique_ptr<Model>(model));
    }
    void addParameter(MocoParameter* ptr) {
        $self->addParameter(std::unique_ptr<MocoParameter>(ptr));
    }
    void addGoal(MocoGoal* ptr) {
        $self->addGoal(std::unique_ptr<MocoGoal>(ptr));
    }
    void addPathConstraint(MocoPathConstraint* ptr) {
        $self->addPathConstraint(std::unique_ptr<MocoPathConstraint>(ptr));
    }
}

%extend OpenSim::MocoProblem {
    void setModel(Model* model) {
        $self->setModel(std::unique_ptr<Model>(model));
    }
    void addParameter(MocoParameter* ptr) {
        $self->addParameter(std::unique_ptr<MocoParameter>(ptr));
    }
    void addGoal(MocoGoal* ptr) {
        $self->addGoal(std::unique_ptr<MocoGoal>(ptr));
    }
    void addPathConstraint(MocoPathConstraint* ptr) {
        $self->addPathConstraint(std::unique_ptr<MocoPathConstraint>(ptr));
    }
}

%include <OpenSim/Moco/MocoProblem.h>
%include <OpenSim/Moco/MocoParameter.h>

// Workaround for SWIG not supporting inherited constructors.
%define EXPOSE_BOUNDS_CONSTRUCTORS_HELPER(NAME)
%extend OpenSim::NAME {
    NAME() { return new NAME(); }
    NAME(double value) { return new NAME(value); }
    NAME(double lower, double upper) { return new NAME(lower, upper); }
};
%enddef
EXPOSE_BOUNDS_CONSTRUCTORS_HELPER(MocoInitialBounds);
EXPOSE_BOUNDS_CONSTRUCTORS_HELPER(MocoFinalBounds);


// SWIG does not support initializer_list, but we can use Java arrays to
// achieve similar syntax in MATLAB.
%ignore OpenSim::MocoTrajectory::setTime(std::initializer_list<double>);
%ignore OpenSim::MocoTrajectory::setState(const std::string&,
        std::initializer_list<double>);
%ignore OpenSim::MocoTrajectory::setControl(const std::string&,
        std::initializer_list<double>);
%ignore OpenSim::MocoTrajectory::setMultiplier(const std::string&,
        std::initializer_list<double>);
%ignore OpenSim::MocoTrajectory::setDerivative(const std::string&,
        std::initializer_list<double>);

%include <OpenSim/Moco/MocoTrajectory.h>

%include <OpenSim/Moco/MocoSolver.h>
%include <OpenSim/Moco/MocoDirectCollocationSolver.h>


namespace OpenSim {
    %ignore MocoTropterSolver::MocoTropterSolver(const MocoProblem&);
}
%include <OpenSim/Moco/MocoTropterSolver.h>
%include <OpenSim/Moco/MocoCasADiSolver/MocoCasADiSolver.h>
%include <OpenSim/Moco/MocoStudy.h>
%include <OpenSim/Moco/MocoStudyFactory.h>

%include <OpenSim/Moco/MocoTool.h>
%include <OpenSim/Moco/MocoInverse.h>
%include <OpenSim/Moco/MocoTrack.h>

%include <OpenSim/Moco/MocoUtilities.h>
%template(analyzeMocoTrajectory) OpenSim::analyzeMocoTrajectory<double>;
%template(analyzeMocoTrajectoryVec3) OpenSim::analyzeMocoTrajectory<SimTK::Vec3>;
%template(analyzeMocoTrajectorySpatialVec) OpenSim::analyzeMocoTrajectory<SimTK::SpatialVec>;

%include <OpenSim/Moco/ModelOperatorsDGF.h>
