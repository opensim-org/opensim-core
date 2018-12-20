%newobject *::clone;

/* To recognize SimTK::RowVector in header files (TODO: move to simbody.i) */
typedef SimTK::RowVector_<double> RowVector;

%include <Muscollo/osimMuscolloDLL.h>
%include <Muscollo/MucoCost.h>
%include <Muscollo/MucoWeightSet.h>
%include <Muscollo/MucoStateTrackingCost.h>
%include <Muscollo/MucoMarkerTrackingCost.h>
%include <Muscollo/MucoMarkerEndpointCost.h>
%include <Muscollo/MucoControlCost.h>
%include <Muscollo/MucoJointReactionNormCost.h>


// %template(MucoBoundsVector) std::vector<OpenSim::MucoBounds>;

%include <Muscollo/MucoBounds.h>
%include <Muscollo/MucoVariableInfo.h>

// %template(MucoVariableInfoVector) std::vector<OpenSim::MucoVariableInfo>;

%ignore OpenSim::MucoMultibodyConstraint::getKinematicLevels;
%ignore OpenSim::MucoConstraintInfo::getBounds;
%ignore OpenSim::MucoConstraintInfo::setBounds;
%ignore OpenSim::MucoProblemRep::getMultiplierInfos;

%include <Muscollo/MucoConstraint.h>

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
%define muscollo_unique_ptr(TYPE)
    %template() std::unique_ptr<TYPE>;
    %newobject std::unique_ptr<TYPE>::release;
    %typemap(out) std::unique_ptr<TYPE> %{
        %set_output(SWIG_NewPointerObj($1.release(), $descriptor(TYPE *), SWIG_POINTER_OWN | %newpointer_flags));
    %}
%enddef
#endif

// https://github.com/swig/swig/blob/master/Lib/java/std_auto_ptr.i
#if SWIGJAVA
%define muscollo_unique_ptr(TYPE)
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

muscollo_unique_ptr(OpenSim::MucoProblemRep);

%include <Muscollo/MucoProblemRep.h>

// MucoProblemRep() is not copyable, but by default, SWIG tries to make a copy
// when wrapping createRep().
%rename(createRep) OpenSim::MucoProblem::createRepHeap;

namespace OpenSim {
    %ignore MucoPhase::setModel(Model);
    %ignore MucoPhase::setModel(std::unique_ptr<Model>);
    %ignore MucoProblem::setModel(Model);
    %ignore MucoProblem::setModel(std::unique_ptr<Model>);
}

%extend OpenSim::MucoPhase {
    void setModel(Model* model) {
        $self->setModel(std::unique_ptr<Model>(model));
    }
    void addParameter(MucoParameter* ptr) {
        $self->addParameter(std::unique_ptr<MucoParameter>(ptr));
    }
    void addCost(MucoCost* ptr) {
        $self->addCost(std::unique_ptr<MucoCost>(ptr));
    }
    void addPathConstraint(MucoPathConstraint* ptr) {
        $self->addPathConstraint(std::unique_ptr<MucoPathConstraint>(ptr));
    }
}

%extend OpenSim::MucoProblem {
    void setModel(Model* model) {
        $self->setModel(std::unique_ptr<Model>(model));
    }
    void addParameter(MucoParameter* ptr) {
        $self->addParameter(std::unique_ptr<MucoParameter>(ptr));
    }
    void addCost(MucoCost* ptr) {
        $self->addCost(std::unique_ptr<MucoCost>(ptr));
    }
    void addPathConstraint(MucoPathConstraint* ptr) {
        $self->addPathConstraint(std::unique_ptr<MucoPathConstraint>(ptr));
    }
}

%include <Muscollo/MucoProblem.h>
%include <Muscollo/MucoParameter.h>

// Workaround for SWIG not supporting inherited constructors.
%define EXPOSE_BOUNDS_CONSTRUCTORS_HELPER(NAME)
%extend OpenSim::NAME {
    NAME() { return new NAME(); }
    NAME(double value) { return new NAME(value); }
    NAME(double lower, double upper) { return new NAME(lower, upper); }
};
%enddef
EXPOSE_BOUNDS_CONSTRUCTORS_HELPER(MucoInitialBounds);
EXPOSE_BOUNDS_CONSTRUCTORS_HELPER(MucoFinalBounds);


// SWIG does not support initializer_list, but we can use Java arrays to
// achieve similar syntax in MATLAB.
%ignore OpenSim::MucoIterate::setTime(std::initializer_list<double>);
%ignore OpenSim::MucoIterate::setState(const std::string&,
        std::initializer_list<double>);
%ignore OpenSim::MucoIterate::setControl(const std::string&,
        std::initializer_list<double>);
%ignore OpenSim::MucoIterate::setMultiplier(const std::string&,
        std::initializer_list<double>);

%include <Muscollo/MucoIterate.h>

%include <Muscollo/MucoSolver.h>


namespace OpenSim {
    %ignore MucoTropterSolver::MucoTropterSolver(const MucoProblem&);
}
%include <Muscollo/MucoTropterSolver.h>
%include <Muscollo/MucoTool.h>

%include <Muscollo/ActivationCoordinateActuator.h>
%include <Muscollo/MuscolloUtilities.h>
