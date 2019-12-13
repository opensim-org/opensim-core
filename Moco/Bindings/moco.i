%newobject *::clone;

/* To recognize SimTK::RowVector in header files (TODO: move to simbody.i) */
typedef SimTK::RowVector_<double> RowVector;

%include <Moco/osimMocoDLL.h>

%include <Moco/About.h>

%include <Moco/Common/TableProcessor.h>


namespace OpenSim {
    %ignore ModelProcessor::setModel(std::unique_ptr<Model>);
}

%extend OpenSim::ModelProcessor {
    void setModel(Model* model) {
        $self->setModel(std::unique_ptr<Model>(model));
    }
};
%include <Moco/ModelProcessor.h>

namespace OpenSim {
    %ignore MocoGoal::GoalInput;
    %ignore MocoGoal::calcGoal;
}
%include <Moco/MocoGoal/MocoGoal.h>
%template(SetMocoWeight) OpenSim::Set<OpenSim::MocoWeight, OpenSim::Object>;
%include <Moco/MocoWeightSet.h>
%include <Moco/MocoGoal/MocoStateTrackingGoal.h>
%include <Moco/MocoGoal/MocoMarkerTrackingGoal.h>
%include <Moco/MocoGoal/MocoMarkerFinalGoal.h>
%include <Moco/MocoGoal/MocoControlGoal.h>
%include <Moco/MocoGoal/MocoInitialActivationGoal.h>
%include <Moco/MocoGoal/MocoJointReactionGoal.h>
%include <Moco/MocoGoal/MocoSumSquaredStateGoal.h>
%include <Moco/MocoGoal/MocoOrientationTrackingGoal.h>
%include <Moco/MocoGoal/MocoTranslationTrackingGoal.h>
%include <Moco/MocoGoal/MocoOutputGoal.h>
%include <Moco/MocoGoal/MocoPeriodicityGoal.h>
%include <Moco/MocoGoal/MocoInitialForceEquilibriumGoal.h>
%include <Moco/MocoGoal/MocoInitialVelocityEquilibriumDGFGoal.h>


// %template(MocoBoundsVector) std::vector<OpenSim::MocoBounds>;

%include <Moco/MocoBounds.h>
%include <Moco/MocoVariableInfo.h>

// %template(MocoVariableInfoVector) std::vector<OpenSim::MocoVariableInfo>;

%ignore OpenSim::MocoMultibodyConstraint::getKinematicLevels;
%ignore OpenSim::MocoConstraintInfo::getBounds;
%ignore OpenSim::MocoConstraintInfo::setBounds;
%ignore OpenSim::MocoProblemRep::getMultiplierInfos;

%include <Moco/MocoConstraint.h>

%include <Moco/MocoControlBoundConstraint.h>
%include <Moco/MocoFrameDistanceConstraint.h>

// unique_ptr
// ----------
// https://stackoverflow.com/questions/27693812/how-to-handle-unique-ptrs-with-swig
namespace std {
%feature("novaluewrapper") unique_ptr;
template <typename Type>
struct unique_ptr {
    typedef Type* pointer;
    explicit unique_ptr( pointer Ptr );
    unique_ptr (unique_ptr&& Right);
    template<class Type2, Class Del2> unique_ptr( unique_ptr<Type2, Del2>&& Right );
    unique_ptr( const unique_ptr& Right) = delete;
    pointer operator-> () const;
    pointer release ();
    void reset (pointer __p=pointer());
    void swap (unique_ptr &__u);
    pointer get () const;
    operator bool () const;
    ~unique_ptr();
};
}

// https://github.com/swig/swig/blob/master/Lib/python/std_auto_ptr.i
#if SWIGPYTHON
%define moco_unique_ptr(TYPE)
    %template() std::unique_ptr<TYPE>;
    %newobject std::unique_ptr<TYPE>::release;
    %typemap(out) std::unique_ptr<TYPE> %{
        %set_output(SWIG_NewPointerObj($1.release(), $descriptor(TYPE *), SWIG_POINTER_OWN | %newpointer_flags));
    %}
%enddef
#endif

// https://github.com/swig/swig/blob/master/Lib/java/std_auto_ptr.i
#if SWIGJAVA
%define moco_unique_ptr(TYPE)
    %template() std::unique_ptr<TYPE>;
    %typemap(jni) std::unique_ptr<TYPE> "jlong"
    %typemap(jtype) std::unique_ptr<TYPE> "long"
    %typemap(jstype) std::unique_ptr<TYPE> "$typemap(jstype, TYPE)"
    %typemap(out) std::unique_ptr<TYPE> %{
        jlong lpp = 0;
        *(TYPE**) &lpp = $1.release();
        $result = lpp;
    %}
    %typemap(javaout) std::unique_ptr<TYPE> {
        long cPtr = $jnicall;
        return (cPtr == 0) ? null : new $typemap(jstype, TYPE)(cPtr, true);
    }
%enddef
#endif

moco_unique_ptr(OpenSim::MocoProblemRep);

%include <Moco/MocoProblemRep.h>

// MocoProblemRep() is not copyable, but by default, SWIG tries to make a copy
// when wrapping createRep().
%rename(createRep) OpenSim::MocoProblem::createRepHeap;

namespace OpenSim {
    %ignore ModelProcessor::setModel(std::unique_ptr<Model>);
    %ignore MocoPhase::setModel(Model);
    %ignore MocoPhase::setModel(std::unique_ptr<Model>);
    %ignore MocoProblem::setModel(Model);
    %ignore MocoProblem::setModel(std::unique_ptr<Model>);
}

%extend OpenSim::ModelProcessor {
    void setModel(Model* model) {
        $self->setModel(std::unique_ptr<Model>(model));
    }
};

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

%include <Moco/MocoProblem.h>
%include <Moco/MocoParameter.h>

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

%include <Moco/MocoTrajectory.h>

%include <Moco/MocoSolver.h>
%include <Moco/MocoDirectCollocationSolver.h>


namespace OpenSim {
    %ignore MocoTropterSolver::MocoTropterSolver(const MocoProblem&);
}
%include <Moco/MocoTropterSolver.h>
%include <Moco/MocoCasADiSolver/MocoCasADiSolver.h>
%include <Moco/MocoStudy.h>

%include <Moco/MocoTool.h>
%include <Moco/MocoInverse.h>
%include <Moco/MocoTrack.h>

%include <Moco/Components/ActivationCoordinateActuator.h>
%include <Moco/Components/DeGrooteFregly2016Muscle.h>
moco_unique_ptr(OpenSim::PositionMotion);
%include <Moco/Components/PositionMotion.h>

%include <Moco/MocoUtilities.h>
%template(analyze) OpenSim::analyze<double>;
%template(analyzeVec3) OpenSim::analyze<SimTK::Vec3>;
%template(analyzeSpatialVec) OpenSim::analyze<SimTK::SpatialVec>;

%include <Moco/Components/ModelFactory.h>
%include <Moco/Components/SmoothSphereHalfSpaceForce.h>
%include <Moco/Components/MultivariatePolynomialFunction.h>

%include <Moco/ModelOperators.h>