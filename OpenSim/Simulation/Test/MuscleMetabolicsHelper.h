

#ifdef USE_ACTIVATION_DYNAMICS_MODEL
//==============================================================================
//                              EXCITATION GETTER
//==============================================================================
// Supplies muscle excitation to the ZerothOrderMuscleActivationDynamics model
// used by UmbergerMuscle.
class MyExcitationGetter : public MuscleActivationDynamics::ExcitationGetter {
public:
    MyExcitationGetter(Muscle& muscle) : _muscle(muscle) {}

    double getExcitation(const SimTK::State& s) const override
    { return _muscle.getControl(s); }

private:
    const Muscle& _muscle;
};
#endif


//==============================================================================
//                               UMBERGER MUSCLE
//==============================================================================
// Muscle model used by Umberger et al., which is a slightly modified version of
// the contractile element described by van Soest and Bobbert.
class UmbergerMuscle : public Muscle {
OpenSim_DECLARE_CONCRETE_OBJECT(UmbergerMuscle, Muscle);
public:
    OpenSim_DECLARE_PROPERTY(width, double,
        "Normalized width of the active-force-length curve.");
    OpenSim_DECLARE_PROPERTY(Arel, double,
        "Arel = A_Hill * fiberActiveForceLengthMultiplier / maxIsometricForce");
    OpenSim_DECLARE_PROPERTY(Brel, double,
        "Brel = B_Hill / optimalFiberLength");
    OpenSim_DECLARE_PROPERTY(FmaxEccentric, double,
        "Asymptote on the eccentric side of the force-velocity curve.");

    #ifdef USE_ACTIVATION_DYNAMICS_MODEL
    OpenSim_DECLARE_UNNAMED_PROPERTY(ZerothOrderMuscleActivationDynamics,
        "Activation dynamic model that simply sets activation to excitation.");
    #endif

    //--------------------------------------------------------------------------
    // CONSTRUCTOR
    //--------------------------------------------------------------------------
    UmbergerMuscle(const std::string& muscleName, double maxIsometricForce,
                   double optimalFiberLength, double width, double Arel,
                   double Brel, double FmaxEccentric)
    {
        setName(muscleName);
        setMaxIsometricForce(maxIsometricForce);
        setOptimalFiberLength(optimalFiberLength);
        setMaxContractionVelocity(Brel/Arel);

        constructProperty_width(width);
        constructProperty_Arel(Arel);
        constructProperty_Brel(Brel);
        constructProperty_FmaxEccentric(FmaxEccentric);

        #ifdef USE_ACTIVATION_DYNAMICS_MODEL
        constructProperty_ZerothOrderMuscleActivationDynamics(
            ZerothOrderMuscleActivationDynamics());
        upd_ZerothOrderMuscleActivationDynamics().setExcitationGetter(
            new MyExcitationGetter(*this));
        #endif
    }

    //--------------------------------------------------------------------------
    // SET MUSCLE STATES
    //--------------------------------------------------------------------------
    void setFiberLength(SimTK::State& s, double fiberLength) const
    {
        setStateVariableValue(s, stateName_fiberLength, fiberLength);
        markCacheVariableInvalid(s, "lengthInfo");
        markCacheVariableInvalid(s, "velInfo");
        markCacheVariableInvalid(s, "dynamicsInfo");
    }

    void setNormFiberVelocity(SimTK::State& s, double normFiberVelocity) const
    {
        setStateVariableValue(s, stateName_fiberVelocity, normFiberVelocity *
                         getMaxContractionVelocity() * getOptimalFiberLength());
        markCacheVariableInvalid(s, "velInfo");
        markCacheVariableInvalid(s, "dynamicsInfo");
    }

    //--------------------------------------------------------------------------
    // MODELCOMPONENT INTERFACE
    //--------------------------------------------------------------------------
    void extendConnectToModel(Model& model) override
    {
        Super::extendConnectToModel(model);
        #ifdef USE_ACTIVATION_DYNAMICS_MODEL
        ZerothOrderMuscleActivationDynamics &zomad =
            upd_ZerothOrderMuscleActivationDynamics();
        includeAsSubComponent(&zomad);
        #endif
    }

    void extendAddToSystem(SimTK::MultibodySystem& system) const override
    {
        Super::extendAddToSystem(system);
        addStateVariable(stateName_fiberLength);
        addStateVariable(stateName_fiberVelocity);
    }

    void extendInitStateFromProperties(SimTK::State& s) const override
    {
        Super::extendInitStateFromProperties(s);
        setFiberLength(s, getOptimalFiberLength());
    }

    void extendSetPropertiesFromState(const SimTK::State& s) override
    {
        Super::extendSetPropertiesFromState(s);
        setOptimalFiberLength(getFiberLength(s));
    }

    void computeStateVariableDerivatives(const SimTK::State& s) const override
    {
        // This implementation is not intended for use in dynamic simulations.
        const int n = getNumStateVariables();
        setStateVariableDerivativeValue(s, stateName_fiberLength, 0.0);
        setStateVariableDerivativeValue(s, stateName_fiberVelocity, 0.0);
    }

    //--------------------------------------------------------------------------
    // MUSCLE INTERFACE
    //--------------------------------------------------------------------------
    void computeInitialFiberEquilibrium(SimTK::State& s) const override {}

    void setActivation(SimTK::State& s, double activation) const override
    {
        #ifdef USE_ACTIVATION_DYNAMICS_MODEL
        get_ZerothOrderMuscleActivationDynamics().setActivation(s, activation);
        #else
        setExcitation(s, activation);
        #endif
    }

    double computeActuation(const SimTK::State& s) const override
    {
        const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
        setActuation(s, mdi.tendonForce);
        return mdi.tendonForce;
    }

    // Calculate position-level variables.
    void calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli)
        const override
    {
        mli.fiberLength            = getStateVariableValue(s, stateName_fiberLength);
        mli.fiberLengthAlongTendon = mli.fiberLength;
        mli.normFiberLength        = mli.fiberLength / getOptimalFiberLength();
        mli.tendonLength           = 0;
        mli.normTendonLength       = 0;
        mli.tendonStrain           = 0;
        mli.pennationAngle         = 0;
        mli.cosPennationAngle      = 1;
        mli.sinPennationAngle      = 0;
        mli.fiberPassiveForceLengthMultiplier = 0;

        // The fiberActiveForceLengthMultiplier (referred to as 'Fisom' in [3])
        // is the proportion of maxIsometricForce that would be delivered
        // isometrically at maximal activation. Fisom=1 if Lce=Lceopt.
        if (mli.fiberLength < (1 - get_width()) * getOptimalFiberLength() ||
            mli.fiberLength > (1 + get_width()) * getOptimalFiberLength())
            mli.fiberActiveForceLengthMultiplier = 0;
        else {
            double c  = -1.0 / (get_width() * get_width());
            double t1 = mli.fiberLength / getOptimalFiberLength();
            mli.fiberActiveForceLengthMultiplier = c*t1*(t1-2) + c + 1;
        }
    }

    // Calculate velocity-level variables.
    void calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi)
        const override
    {
        fvi.fiberVelocity = getStateVariableValue(s, stateName_fiberVelocity);
        fvi.fiberVelocityAlongTendon = fvi.fiberVelocity;
        fvi.normFiberVelocity        = fvi.fiberVelocity /
                        (getMaxContractionVelocity() * getOptimalFiberLength());

        fvi.pennationAngularVelocity     = 0;
        fvi.tendonVelocity               = 0;
        fvi.normTendonVelocity           = 0;
        fvi.fiberForceVelocityMultiplier = 1;
    }

    // Calculate dynamics-level variables.
    void calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi)
        const override
    {
        #ifdef USE_ACTIVATION_DYNAMICS_MODEL
        mdi.activation =
            get_ZerothOrderMuscleActivationDynamics().getActivation(s);
        #else
        mdi.activation = getExcitation(s);
        #endif

        // These expressions were obtained by solving the 'Vce' equations in [3]
        // for force F, then applying the modifications described in [1].
        // Negative fiber velocity corresponds to concentric contraction.
        double ArelStar = pow(mdi.activation,-0.3) * get_Arel();
        if (getFiberVelocity(s) <= 0) {
            double v  = max(getFiberVelocity(s),
                        -getMaxContractionVelocity() * getOptimalFiberLength());
            double t1 = get_Brel() * getOptimalFiberLength();
            mdi.fiberForce = (t1*getActiveForceLengthMultiplier(s) + ArelStar*v)
                             / (t1 - v);
        } else {
            double c2 = -get_FmaxEccentric() / mdi.activation;
            double c3 = (get_FmaxEccentric()-1) * get_Brel() / (mdi.activation *
                            2 * (getActiveForceLengthMultiplier(s) + ArelStar));
            double c1 = (get_FmaxEccentric()-1) * c3 / mdi.activation;
            mdi.fiberForce = -(getOptimalFiberLength() * (c1 + c2*c3)
                               + c2*getFiberVelocity(s)) /
                             (getFiberVelocity(s) + c3*getOptimalFiberLength());
        }
        mdi.fiberForce *= getMaxIsometricForce() * mdi.activation;

        mdi.fiberForceAlongTendon = mdi.fiberForce;
        mdi.normFiberForce        = mdi.fiberForce / getMaxIsometricForce();
        mdi.activeFiberForce      = mdi.fiberForce;
        mdi.passiveFiberForce     = 0;
        mdi.tendonForce           = mdi.fiberForce;
        mdi.normTendonForce       = mdi.normFiberForce;
        mdi.fiberStiffness        = 0;
        mdi.fiberStiffnessAlongTendon = 0;
        mdi.tendonStiffness       = 0;
        mdi.muscleStiffness       = 0;
        mdi.fiberActivePower      = 0;
        mdi.fiberPassivePower     = 0;
        mdi.tendonPower           = 0;
        mdi.musclePower           = 0;
    }

private:
    static const std::string stateName_fiberLength;
    static const std::string stateName_fiberVelocity;
};
const std::string UmbergerMuscle::stateName_fiberLength   = "fiber_length";
const std::string UmbergerMuscle::stateName_fiberVelocity = "fiber_velocity";

//==============================================================================
//                    CONSTANT-EXCITATION MUSCLE CONTROLLER
//==============================================================================
// Simple controller to maintain all muscle excitations at the same constant
// value. The metabolic probes depend on both excitation and activation.
class ConstantExcitationMuscleController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(ConstantExcitationMuscleController, Controller);
public:
    ConstantExcitationMuscleController(double u) : _u(u) {}

    void computeControls(const SimTK::State& s, SimTK::Vector &controls) const
        override
    {
        for (int i=0; i<_model->getMuscles().getSize(); ++i)
            controls[i] = _u;
    }

    void setConstantExcitation(double u) { _u = u; }

private:
    double _u;
};


Model createSlidingBlockWithMuscle(const std::string& muscleName,
                                   double  maxIsometricForce,
                                   double  optimalFiberLength,
                                   double  width,
                                   double  Arel,
                                   double  Brel,
                                   double  FmaxEccentric) {
    Model model;
    model.setName("testModel_"+muscleName);
    Ground& ground = model.updGround();

    // Create block. The length and velocity of the muscle will be specified, so
    // the properties of the block are irrelevant.
    const double blockMass       = 1.0;
    const double blockSideLength = 0.1;
    Inertia blockInertia = blockMass * Inertia::brick(Vec3(blockSideLength/2));
    OpenSim::Body *block = new OpenSim::Body("block", blockMass, Vec3(0),
                                             blockInertia);

    // Create slider joint between ground and block.
    SliderJoint* prismatic = new SliderJoint("prismatic", ground, Vec3(0), Vec3(0),
                                                *block, Vec3(0), Vec3(0));
    CoordinateSet& prisCoordSet = prismatic->upd_CoordinateSet();
    prisCoordSet[0].setName("xTranslation");
    model.addBody(block);
    model.addJoint(prismatic);

    // Create muscle attached to ground and block.
    UmbergerMuscle* muscle = new UmbergerMuscle(muscleName, maxIsometricForce,
        optimalFiberLength, width, Arel, Brel, FmaxEccentric);
    muscle->addNewPathPoint("muscle-ground", ground, Vec3(0));
    muscle->addNewPathPoint("muscle-block",  *block, Vec3(0));
    model.addForce(muscle);

    // Attach muscle controller.
    const double constantActivation = 1.0;
    ConstantExcitationMuscleController* controller =
        new ConstantExcitationMuscleController(constantActivation);
    controller->setActuators(model.updActuators());
    model.addController(controller);
    
    return model;
}
