// osimSimulation
%include <OpenSim/Simulation/osimSimulationDLL.h>

%typedef SimTK::DecorativeGeometry::Representation VisualRepresentation;

%include <OpenSim/Common/Component.h>
%include <OpenSim/Simulation/Model/Appearance.h>
%include <OpenSim/Simulation/Model/Geometry.h>
%include <OpenSim/Simulation/Model/ModelComponent.h>
%include <OpenSim/Simulation/Model/ModelComponentSet.h>
%include <OpenSim/Simulation/Model/ComponentSet.h>

%include <OpenSim/Simulation/Solver.h>
%include <OpenSim/Simulation/InverseDynamicsSolver.h>
%include <OpenSim/Simulation/MomentArmSolver.h>

%include <OpenSim/Simulation/Model/Frame.h>

%include <OpenSim/Simulation/Model/PhysicalFrame.h>
%include <OpenSim/Simulation/Model/Ground.h>
%include <OpenSim/Simulation/SimbodyEngine/Body.h>
%include <OpenSim/Simulation/SimbodyEngine/Joint.h>
%include <OpenSim/Simulation/SimbodyEngine/Constraint.h>
%include <OpenSim/Simulation/Model/Force.h>
%include <OpenSim/Simulation/Control/Controller.h>
%include <OpenSim/Simulation/Model/ContactGeometry.h>
%include <OpenSim/Simulation/Model/Analysis.h>

%include <OpenSim/Simulation/Model/Probe.h>
%template(SetProbes) OpenSim::Set<OpenSim::Probe>;
%template(ModelComponentSetProbes) OpenSim::ModelComponentSet<OpenSim::Probe>;
%include <OpenSim/Simulation/Model/ProbeSet.h>
%include <OpenSim/Simulation/Model/SystemEnergyProbe.h>
%include <OpenSim/Simulation/Model/JointInternalPowerProbe.h>
%include <OpenSim/Simulation/Model/ActuatorPowerProbe.h>
%include <OpenSim/Simulation/Model/ActuatorForceProbe.h>
%include <OpenSim/Simulation/Model/MuscleActiveFiberPowerProbe.h>
%include <OpenSim/Simulation/Model/Bhargava2004MuscleMetabolicsProbe.h>
%include <OpenSim/Simulation/Model/Umberger2010MuscleMetabolicsProbe.h>
%include <OpenSim/Simulation/Model/ModelDisplayHints.h>
%include <OpenSim/Simulation/Model/ModelVisualPreferences.h>
%include <OpenSim/Simulation/Model/ModelVisualizer.h>
%include <OpenSim/Simulation/Model/Model.h>

// Iterators.
// TODO rename to singular form.
%template(FrameList)          OpenSim::ComponentList<OpenSim::Frame>;
%template(BodyList)           OpenSim::ComponentList<OpenSim::Body>;
%template(MuscleList)         OpenSim::ComponentList<OpenSim::Muscle>;
%template(ModelComponentList) OpenSim::ComponentList<OpenSim::ModelComponent>;
%template(JointList)          OpenSim::ComponentList<OpenSim::Joint>;

%template(getComponentsList) 
    OpenSim::Component::getComponentList<OpenSim::Component>;
%template(getFrameList) 
    OpenSim::Component::getComponentList<OpenSim::Frame>;
%template(getBodyList) 
    OpenSim::Component::getComponentList<OpenSim::Body>;
%template(getMuscleList) 
    OpenSim::Component::getComponentList<OpenSim::Muscle>;
%template(getModelComponentList) 
    OpenSim::Component::getComponentList<OpenSim::ModelComponent>;
%template(getJointList) 
    OpenSim::Component::getComponentList<OpenSim::Joint>;

%template(FrameIterator) 
    OpenSim::ComponentListIterator<OpenSim::Frame>;
%template(BodyIterator) 
    OpenSim::ComponentListIterator<OpenSim::Body>;
%template(MuscleIterator) 
    OpenSim::ComponentListIterator<OpenSim::Muscle>;
%template(ModelComponentIterator) 
    OpenSim::ComponentListIterator<OpenSim::ModelComponent>;
%template(JointIterator) 
    OpenSim::ComponentListIterator<OpenSim::Joint>;

