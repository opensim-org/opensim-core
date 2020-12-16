OpenSim Core
============
![continuous-integration](https://github.com/opensim-org/opensim-core/workflows/continuous-integration/badge.svg)[![ZenHub][zenhub_image]][zenhub]

**NOTE: This repository cannot be used to build OpenSim 3.x or earlier. For OpenSim 3.x, see [here](http://simtk-confluence.stanford.edu:8080/display/OpenSim/Building+OpenSim+from+Source).**

OpenSim is software that lets users develop models of musculoskeletal
structures and create dynamic simulations of movement, such as this one:

![Simulation of human running by Sam Hamner (doi:
10.1016/j.jbiomech.2010.06.025)][running_gif]

More information can be found at our websites:

* [OpenSim website](http://opensim.stanford.edu); in particular, the [support
  page](http://opensim.stanford.edu/support/index.html).
* [SimTK project website](https://simtk.org/home/opensim)

This repository contains the source code for OpenSim's C++ libraries, C++
examples, command-line applications (inverse kinematics, computed muscle
control, etc.), and Java and Python wrapping. This repository does *not*
include source code for the OpenSim GUI.


Table of contents
-----------------
- [Simple Example](#simple-example)
- [Building from the source code](#building-from-the-source-code)
  - [On Windows using Microsoft Visual Studio](#on-windows-using-visual-studio).
  - [On Mac OSX using Xcode](#on-mac-osx-using-xcode).
  - [On Ubuntu using Unix Makefiles](#on-ubuntu-using-unix-makefiles).


Simple example
--------------
Let's simulate a simple arm whose elbow is actuated by a muscle, using
the C++ interface 

<details open>
<summary>C++</summary>

```cpp
#include <OpenSim/OpenSim.h>
using namespace SimTK;
using namespace OpenSim;

int main() {
    Model model;
    model.setName("bicep_curl");
    model.setUseVisualizer(true);

    // Create two links, each with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia of zero.
    OpenSim::Body* humerus = new OpenSim::Body("humerus", 1, Vec3(0), Inertia(0));
    OpenSim::Body* radius  = new OpenSim::Body("radius",  1, Vec3(0), Inertia(0));

    // Connect the bodies with pin joints. Assume each body is 1 m long.
    PinJoint* shoulder = new PinJoint("shoulder",
            // Parent body, location in parent, orientation in parent.
            model.getGround(), Vec3(0), Vec3(0),
            // Child body, location in child, orientation in child.
            *humerus, Vec3(0, 1, 0), Vec3(0));
    PinJoint* elbow = new PinJoint("elbow",
            *humerus, Vec3(0), Vec3(0), *radius, Vec3(0, 1, 0), Vec3(0));

    // Add a muscle that flexes the elbow.
    Millard2012EquilibriumMuscle* biceps = new
        Millard2012EquilibriumMuscle("biceps", 200, 0.6, 0.55, 0);
    biceps->addNewPathPoint("origin",    *humerus, Vec3(0, 0.8, 0));
    biceps->addNewPathPoint("insertion", *radius,  Vec3(0, 0.7, 0));

    // Add a controller that specifies the excitation of the muscle.
    PrescribedController* brain = new PrescribedController();
    brain->addActuator(*biceps);
    // Muscle excitation is 0.3 for the first 0.5 seconds, then increases to 1.
    brain->prescribeControlForActuator("biceps",
            new StepFunction(0.5, 3, 0.3, 1));

    // Add components to the model.
    model.addBody(humerus);    model.addBody(radius);
    model.addJoint(shoulder);  model.addJoint(elbow);
    model.addForce(biceps);
    model.addController(brain);

    // Add a console reporter to print the muscle fiber force and elbow angle.
    ConsoleReporter* reporter = new ConsoleReporter();
    reporter->set_report_time_interval(1.0);
    reporter->addToReport(biceps->getOutput("fiber_force"));
    reporter->addToReport(
        elbow->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"),
        "elbow_angle");
    model.addComponent(reporter);

    // Add display geometry.
    Ellipsoid bodyGeometry(0.1, 0.5, 0.1);
    bodyGeometry.setColor(Gray);
    // Attach an ellipsoid to a frame located at the center of each body.
    PhysicalOffsetFrame* humerusCenter = new PhysicalOffsetFrame(
        "humerusCenter", *humerus, Transform(Vec3(0, 0.5, 0)));
    humerus->addComponent(humerusCenter);
    humerusCenter->attachGeometry(bodyGeometry.clone());
    PhysicalOffsetFrame* radiusCenter = new PhysicalOffsetFrame(
        "radiusCenter", *radius, Transform(Vec3(0, 0.5, 0)));
    radius->addComponent(radiusCenter);
    radiusCenter->attachGeometry(bodyGeometry.clone());

    // Configure the model.
    State& state = model.initSystem();
    // Fix the shoulder at its default angle and begin with the elbow flexed.
    shoulder->getCoordinate().setLocked(state, true);
    elbow->getCoordinate().setValue(state, 0.5 * Pi);
    model.equilibrateMuscles(state);

    // Configure the visualizer.
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(viz.SolidColor);
    viz.setBackgroundColor(White);

    // Simulate.
    simulate(model, state, 10.0);

    return 0;
};
```
</details>


This code produces the following animation:

![Simulation of an arm actuated by a muscle][simple_example_gif]

and prints the following information to the console:
```
[reporter]
              | /forceset/bice|               | 
          time| ps|fiber_force|    elbow_angle| 
--------------| --------------| --------------| 
           0.0|       1.180969|      1.5707963| 
           1.0|       57.27509|     0.77066412| 
           2.0|      19.219591|      1.5679832| 
           3.0|      56.155742|      1.4422429| 
           4.0|      33.436111|      1.5084227| 
           5.0|      32.678114|       1.517973| 
           6.0|      37.605448|      1.5022219| 
           7.0|      36.417485|      1.5072158| 
           8.0|      34.419941|      1.5079513| 
           9.0|      34.661339|      1.5067137| 
          10.0|      35.896608|      1.5071069| 
```

Expand to see Python and Matlab versions of the above example example:

<details>
<summary>Python</summary>
 
```py
import opensim as osim 

arm = osim.Model()
arm.setName("bicep_curl")
arm.setUseVisualizer(True)

# ---------------------------------------------------------------------------
# Create two links, each with a mass of 1 kg, centre of mass at the body's
# origin, and moments and products of inertia of zero.
# ---------------------------------------------------------------------------

humerus = osim.Body("humerus",
                    1.0,
                    osim.Vec3(0),
                    osim.Inertia(0, 0, 0))
radius = osim.Body("radius",
                   1.0,
                   osim.Vec3(0),
                   osim.Inertia(0, 0, 0))

# ---------------------------------------------------------------------------
# Connect the bodies with pin joints. Assume each body is 1m long.
# ---------------------------------------------------------------------------

shoulder = osim.PinJoint("shoulder",
                         arm.getGround(), # PhysicalFrame
                         osim.Vec3(0),
                         osim.Vec3(0),
                         humerus, # PhysicalFrame
                         osim.Vec3(0, 1, 0),
                         osim.Vec3(0))

elbow = osim.PinJoint("elbow",
                      humerus, # PhysicalFrame
                      osim.Vec3(0),
                      osim.Vec3(0),
                      radius, # PhysicalFrame
                      osim.Vec3(0, 1, 0),
                      osim.Vec3(0))

# ---------------------------------------------------------------------------
# Add a muscle that flexes the elbow (actuator for robotics people).
# ---------------------------------------------------------------------------

biceps = osim.Millard2012EquilibriumMuscle("biceps",  # Muscle name
                                           200.0,  # Max isometric force
                                           0.6,  # Optimal fibre length
                                           0.55,  # Tendon slack length
                                           0.0)  # Pennation angle
biceps.addNewPathPoint("origin",
                       humerus,
                       osim.Vec3(0, 0.8, 0))

biceps.addNewPathPoint("insertion",
                       radius,
                       osim.Vec3(0, 0.7, 0))

# ---------------------------------------------------------------------------
# Add a controller that specifies the excitation of the muscle.
# ---------------------------------------------------------------------------

brain = osim.PrescribedController()
brain.addActuator(biceps)
brain.prescribeControlForActuator("biceps",
                                  osim.StepFunction(0.5, 3.0, 0.3, 1.0))

# ---------------------------------------------------------------------------
# Build model with components created above.
# ---------------------------------------------------------------------------

arm.addBody(humerus)
arm.addBody(radius)
arm.addJoint(shoulder) # Now required in OpenSim4.0
arm.addJoint(elbow)
arm.addForce(biceps)
arm.addController(brain)

# ---------------------------------------------------------------------------
# Add a console reporter to print the muscle fibre force and elbow angle.
# ---------------------------------------------------------------------------

# We want to write our simulation results to the console.
reporter = osim.ConsoleReporter()
reporter.set_report_time_interval(1.0)
reporter.addToReport(biceps.getOutput("fiber_force"))
elbow_coord = elbow.getCoordinate().getOutput("value")
reporter.addToReport(elbow_coord, "elbow_angle")
arm.addComponent(reporter)

# ---------------------------------------------------------------------------
# Add display geometry. 
# ---------------------------------------------------------------------------

bodyGeometry = osim.Ellipsoid(0.1, 0.5, 0.1)
bodyGeometry.setColor(osim.Vec3(0.5)) # Gray
humerusCenter = osim.PhysicalOffsetFrame()
humerusCenter.setName("humerusCenter")
humerusCenter.setParentFrame(humerus)
humerusCenter.setOffsetTransform(osim.Transform(osim.Vec3(0, 0.5, 0)))
humerus.addComponent(humerusCenter)
humerusCenter.attachGeometry(bodyGeometry.clone())

radiusCenter = osim.PhysicalOffsetFrame()
radiusCenter.setName("radiusCenter")
radiusCenter.setParentFrame(radius)
radiusCenter.setOffsetTransform(osim.Transform(osim.Vec3(0, 0.5, 0)))
radius.addComponent(radiusCenter)
radiusCenter.attachGeometry(bodyGeometry.clone())

# ---------------------------------------------------------------------------
# Configure the model.
# ---------------------------------------------------------------------------

state = arm.initSystem()
# Fix the shoulder at its default angle and begin with the elbow flexed.
shoulder.getCoordinate().setLocked(state, True)
elbow.getCoordinate().setValue(state, 0.5 * osim.SimTK_PI)
arm.equilibrateMuscles(state)

# ---------------------------------------------------------------------------
# Configure the visualizer.
# ---------------------------------------------------------------------------

viz = arm.updVisualizer().updSimbodyVisualizer()
viz.setBackgroundColor(osim.Vec3(0)) # white
viz.setGroundHeight(-2)

# ---------------------------------------------------------------------------
# Simulate.
# ---------------------------------------------------------------------------

manager = osim.Manager(arm)
state.setTime(0)
manager.initialize(state)
state = manager.integrate(10.0)
```

</details>
<details>
<summary>Matlab</summary>

``` Matlab
%% Import Java libraries
import org.opensim.modeling.*

arm = Model();
arm.setName('bicep_curl');
arm.setUseVisualizer(true);

% ---------------------------------------------------------------------------
% Create two links, each with a mass of 1 kg, centre of mass at the body's
% origin, and moments and products of inertia of zero.
% ---------------------------------------------------------------------------

humerus = Body('humerus',...
                    1.0,...
                    Vec3(0),...
                    Inertia(0, 0, 0));
radius = Body('radius',...
                   1.0,...
                   Vec3(0),...
                   Inertia(0, 0, 0));

% ---------------------------------------------------------------------------
% Connect the bodies with pin joints. Assume each body is 1m long.
% ---------------------------------------------------------------------------

shoulder = PinJoint('shoulder',...
                         arm.getGround(),... % PhysicalFrame
                         Vec3(0),...
                         Vec3(0),...
                         humerus,... % PhysicalFrame
                         Vec3(0, 1, 0),...
                         Vec3(0));

elbow = PinJoint('elbow',...
                      humerus,... % PhysicalFrame
                      Vec3(0),...
                      Vec3(0),...
                      radius,... % PhysicalFrame
                      Vec3(0, 1, 0),...
                      Vec3(0));

% ---------------------------------------------------------------------------
% Add a muscle that flexes the elbow (actuator for robotics people).
% ---------------------------------------------------------------------------

biceps = Millard2012EquilibriumMuscle('biceps',...  % Muscle name
                                           200.0,...  % Max isometric force
                                           0.6,...  % Optimal fibre length
                                           0.55,...  % Tendon slack length
                                           0.0);  % Pennation angle
biceps.addNewPathPoint('origin',...
                       humerus,...
                       Vec3(0, 0.8, 0));

biceps.addNewPathPoint('insertion',...
                       radius,...
                       Vec3(0, 0.7, 0));

% ---------------------------------------------------------------------------
% Add a controller that specifies the excitation of the muscle.
% ---------------------------------------------------------------------------

brain = PrescribedController();
brain.addActuator(biceps);
brain.prescribeControlForActuator('biceps',...
                                  StepFunction(0.5, 3.0, 0.3, 1.0));

% ---------------------------------------------------------------------------
% Build model with components created above.
% ---------------------------------------------------------------------------

arm.addBody(humerus);
arm.addBody(radius);
arm.addJoint(shoulder); % Now required in OpenSim4.0
arm.addJoint(elbow);
arm.addForce(biceps);
arm.addController(brain);

% ---------------------------------------------------------------------------
% Add a console reporter to print the muscle fibre force and elbow angle.
% ---------------------------------------------------------------------------

% We want to write our simulation results to the console.
reporter = ConsoleReporter();
reporter.set_report_time_interval(1.0);
reporter.addToReport(biceps.getOutput('fiber_force'));
elbow_coord = elbow.getCoordinate().getOutput('value');
reporter.addToReport(elbow_coord, 'elbow_angle');
arm.addComponent(reporter);

% ---------------------------------------------------------------------------
% Add display geometry. 
% ---------------------------------------------------------------------------

bodyGeometry = Ellipsoid(0.1, 0.5, 0.1);
bodyGeometry.setColor(Vec3(0.5)); % Gray
humerusCenter = PhysicalOffsetFrame();
humerusCenter.setName('humerusCenter');
humerusCenter.setParentFrame(humerus);
humerusCenter.setOffsetTransform(Transform(Vec3(0, 0.5, 0)));
humerus.addComponent(humerusCenter);
humerusCenter.attachGeometry(bodyGeometry.clone());

radiusCenter = PhysicalOffsetFrame();
radiusCenter.setName('radiusCenter');
radiusCenter.setParentFrame(radius);
radiusCenter.setOffsetTransform(Transform(Vec3(0, 0.5, 0)));
radius.addComponent(radiusCenter);
radiusCenter.attachGeometry(bodyGeometry.clone());

% ---------------------------------------------------------------------------
% Configure the model.
% ---------------------------------------------------------------------------

state = arm.initSystem();
% Fix the shoulder at its default angle and begin with the elbow flexed.
shoulder.getCoordinate().setLocked(state, true);
elbow.getCoordinate().setValue(state, 0.5 * pi);
arm.equilibrateMuscles(state);

% ---------------------------------------------------------------------------
% Configure the visualizer
% ---------------------------------------------------------------------------

viz = arm.updVisualizer().updSimbodyVisualizer();
viz.setBackgroundColor(Vec3(0)); % white
viz.setGroundHeight(-2)

% ---------------------------------------------------------------------------
% Simulate.
% ---------------------------------------------------------------------------

manager = Manager(arm);
state.setTime(0);
manager.initialize(state);
state = manager.integrate(10.0);
```

</details>

Building from the source code
-----------------------------

**NOTE**: On all platforms (Windows, OSX, Linux), you should
build all OpenSim dependencies (Simbody, BTK, etc) with the
same *CMAKE_BUILD_TYPE* (Linux) / *CONFIGURATION*
(MSVC/Xcode) (e.g., Release, Debug) as OpenSim. Failing to
do so *may* result in mysterious runtime errors like
segfaults.

We support a few ways of building OpenSim:

1. [On Windows using Microsoft Visual Studio](#on-windows-using-visual-studio). In a rush? Use [these instructions](#for-the-impatient-windows).
2. [On Mac OSX using Xcode](#on-mac-osx-using-xcode). Need extended instructions? Use [these instructions](#extended-instructions-for-osx).
3. [On Ubuntu using Unix Makefiles](#on-ubuntu-using-unix-makefiles). In a rush? Use [these instructions](#for-the-impatient-ubuntu).


On Windows using Visual Studio
------------------------------

#### Get the dependencies

* **operating system**: Windows 7, 8, or 10.
* **cross-platform build system**:
  [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.2
* **compiler / IDE**: Visual Studio [2015](https://www.visualstudio.com/vs/older-downloads/) or [2017](https://www.visualstudio.com/) (2017 requires CMake >= 3.9).
    * The *Community* variant is sufficient and is free for everyone.
    * Visual Studio 2015 and 2017 do not install C++ support by default.
      * **2015**: During the installation you must select
        *Custom*, and check *Programming Languages > Visual C++ > Common Tools
        for Visual C++ 2015*.
        You can uncheck all other boxes. If you have already installed
        Visual Studio without C++ support, simply re-run the installer and
        select *Modify*. Alternatively, go to *File > New > Project...* in
        Visual Studio, select *Visual C++*, and click
        *Install Visual C++ 2015 Tools for Windows Desktop*.
      * **2017**: During the installation, select the workload
        *Desktop Development with C++*.
      * If Visual Studio is installed without C++ support, CMake will report
        the following errors:
        ```
        The C compiler identification is unknown
        The CXX compiler identification is unknown
        ```    
* **physics engine**: Simbody >= 3.7. Two options:
    * Let OpenSim get this for you using superbuild (see below).
    * [Build on your own](
      https://github.com/simbody/simbody#windows-using-visual-studio).
* **C3D file support**: Biomechanical-ToolKit Core. Two options:
    * Let OpenSim get this for you using superbuild (see below).
    * [Build on your own](https://github.com/klshrinidhi/BTKCore).
* **command-line argument parsing**: docopt.cpp. Two options:
    * Let OpenSim get this for you using superbuild (see below); much easier!
    * [Build on your own](https://github.com/docopt/docopt.cpp) (no instructions).
* **logging**: spdlog. Two options:
    * Let OpenSim get this for you using superbuild (see below); much easier!
    * [Build on your own](https://github.com/gabime/spdlog).
* **Moco optimal control solvers**: Solving optimal control problems with 
    OpenSim's Moco module requires either CasADi or Tropter. Use superbuild for
    all these dependencies (except for Tropter, which is part of this
    repository).
    * **MocoCasADiSolver** (optional): [CasADi](https://web.casadi.org) (LGPL).
    * **MocoTropterSolver** (optional): Tropter.
        * **matrix library**: [Eigen](http://eigen.tuxfamily.org) >= 3.3.7
        * **sparse matrix algorithms**: [ColPack](https://github.com/CSCsw/ColPack).
        * **automatic differentiation**: [ADOL-C](https://github.com/coin-or/ADOL-C) >= 2.6.3.
    * **nonlinear optimizer** (required if building with CasADi or Tropter): 
        [IPOPT](https://github.com/coin-or/Ipopt) >= 3.12.8.
* **API documentation** (optional):
  [Doxygen](http://www.stack.nl/~dimitri/doxygen/download.html) >= 1.8.6
* **version control** (optional): git. There are many options:
    * [Git for Windows](http://msysgit.github.io/), most advanced;
    * [TortoiseGit](https://code.google.com/p/tortoisegit/wiki/Download),
      intermediate; good for TortoiseSVN users;
    * [GitHub for Windows](https://windows.github.com/), easiest.
* **Bindings** (optional): [SWIG](http://www.swig.org/) 3.0.8
    * **MATLAB scripting** (optional): [Java development kit][java] >= 1.7.
        * Note: Older versions of MATLAB may use an older version of JVM. Run
                'ver' in MATLAB to check MATLAB's JVM version (must be >= 1.7).
        * Note: Java development kit >= 9 requires CMake >= 3.10.
    * **Python scripting** (optional): Python 2 >= 2.7 or Python 3 >= 3.5
        * [Anaconda](https://www.anaconda.com/distribution/)
        * Must provide the NumPy package; this should come with Anaconda.
    * The choice between 32-bit/64-bit must be the same between Java, Python,
      and OpenSim.

#### Download the OpenSim-Core source code

* Method 1: If you want to get going quickly, download the source code from
  https://github.com/opensim-org/opensim-core/releases, for the version of
  OpenSim you want. We'll assume you unzipped the source code into
  `C:/opensim-core-source`.
* Method 2: If you plan on updating your OpenSim installation or you want to
  contribute back to the project, clone the opensim-core git repository into
  `C:/opensim-core-source`. If using TortoiseGit, open Windows Explorer,
  right-click in the window, select **Git Clone...**, and provide the
  following:
    * **URL**: `https://github.com/opensim-org/opensim-core.git`.
    * **Directory**: `C:/opensim-core-source`.

  If using a Git Bash or Git Shell, run the following:

        $ git clone https://github.com/opensim-org/opensim-core.git C:/opensim-core-source

  This will give you a bleeding-edge version of OpenSim-Core.

#### [Optional] Superbuild: download and build OpenSim dependencies
1. Open the CMake GUI.
2. In the field **Where is the source code**, specify
   `C:/opensim-core-source/dependencies`.
3. In the field **Where to build the binaries**, specify a directory under
   which to build dependencies. Let's say this is
   `C:/opensim-core-dependencies-build`.
4. Click the **Configure** button.
    1. Visual Studio 2015: Choose the *Visual Studio 14 2015* generator (may appear as *Visual Studio 14*).
    2. Visual Studio 2017: Choose the *Visual Studio 15 2017* generator.
    3. To build as 64-bit, select the generator with *Win64* in the name.
    4. Click **Finish**.
5. Where do you want to install OpenSim dependencies on your computer? Set this
   by changing the `CMAKE_INSTALL_PREFIX` variable. Let's say this is
   `C:/opensim-core-dependencies-install`.
6. Variables named `SUPERBUILD_<dependency-name>` allow you to selectively
   download dependencies. By default, all dependencies are downloaded,
   configured and built.
7. Click the **Configure** button again. Then, click **Generate** to make
   Visual Studio project files in the build directory.
8. Go to the build directory you specified in step 3 using the command:

        cd C:/opensim-core-dependencies-build

9. Use CMake to download, compile and install the dependencies:

        cmake --build . --config RelWithDebInfo

   Alternative values for `--config` in this command are:

   * **Debug**: debugger symbols; no optimizations (more than 10x slower).
     Library names end with `_d`.
   * **Release**: no debugger symbols; optimized.
   * **RelWithDebInfo**: debugger symbols; optimized. Bigger but not slower
     than Release; choose this if unsure.
   * **MinSizeRel**: minimum size; optimized.

   You must run this command for each of the configurations you plan to use
   with OpenSim (see below). You should run this command for the release
   configuration *last* to ensure that you use the release version of the
   command-line applications instead of the slow debug versions.

   * Note: Superbuild attempts to determine when dependencies are out of date
     but is not always successful. It is therefore recommended to build all
     dependencies from scratch when updating your installation.
10. If you like, you can now remove the directory used for building
    dependencies (`c:/opensim-core-dependencies-build`).

#### Configure and generate project files

1. Open the CMake GUI.
2. In the field **Where is the source code**, specify `C:/opensim-core-source`.
3. In the field **Where to build the binaries**, specify something like
   `C:/opensim-core-build`, or some other path that is not inside your source
   directory. This is *not* where we are installing OpenSim-Core; see below.
4. Click the **Configure** button.
    1. Visual Studio 2015: Choose the *Visual Studio 14* or *Visual Studio 14 2015* generator.
    2. Visual Studio 2017: Choose the *Visual Studio 15 2017* generator.
    3. To build as 64-bit, select the generator with *Win64* in the name.
       The choice between 32-bit/64-bit must be the same across all dependencies.
    4. Click **Finish**.
5. Where do you want to install OpenSim-Core on your computer? Set this by
   changing the `CMAKE_INSTALL_PREFIX` variable. We'll assume you set it to
   `C:/opensim-core`. If you choose a different installation location, make
   sure to use *yours* where we use `C:/opensim-core` below.
6. Tell CMake where to find dependencies. This depends on how you got them.
    * Superbuild: Set the variable `OPENSIM_DEPENDENCIES_DIR` to the root
      directory you specified with superbuild for installation of dependencies.
      In our example, it would be `c:/opensim-core-dependencies-install`.
    * Obtained on your own:
        1. Simbody: Set the `SIMBODY_HOME` variable to where you installed
           Simbody (e.g., `C:/Simbody`).
        2. BTK: Set the variable `BTK_DIR` to the directory containing
           `BTKConfig.cmake`. If the root directory of your BTK installation is
           `C:/BTKCore-install`, then set this variable to
           `C:/BTKCore-install/share/btk-0.4dev`.
        3. docopt.cpp: Set the variable `docopt_DIR` to the directory
           containing `docopt-config.cmake`. If the root directory of your
           docopt.cpp installation is `C:/docopt.cpp-install`, then set this
           variable to `C:/docopt.cpp-install/lib/cmake`.
        4. spdlog: Set the variable `spdlog_DIR` to the directory containing
           `spdlogConfig.cmake`. If the root directory of your spdlog 
           installation is `C:/spdlog-install`, then set this variable to
           `C:/spdlog-install/lib/spdlog/cmake`.
7. Set the remaining configuration options.
    * `BUILD_API_EXAMPLES` to compile C++ API examples.
    * `BUILD_TESTING` to ensure that OpenSim works correctly. The tests take a
      while to build; if you want to build OpenSim quickly, you can turn this
      off.
    * `BUILD_JAVA_WRAPPING` if you want to access OpenSim through MATLAB or
      Java; see dependencies above.
    * `BUILD_PYTHON_WRAPPING` if you want to access OpenSim through Python; see
      dependencies above. CMake sets `PYTHON_*` variables to tell you the
      Python version used when building the wrappers.
    * `OPENSIM_PYTHON_VERSION` to choose if the Python wrapping is built for
      Python 2 or Python 3.
    * `BUILD_API_ONLY` if you don't want to build the command-line applications.
    * `OPENSIM_WITH_CASADI` if you want support for MocoCasADiSolver.
    * `OPENSIM_WITH_TROPTER` if you want support for MocoTropterSolver.
8. Click the **Configure** button again. Then, click **Generate** to make
   Visual Studio project files in the build directory.

#### Build and install

1. Open `C:/opensim-core-build/OpenSim.sln` in Visual Studio.
2. Select your desired *Solution configuration* from the drop-down at the top.
    * **Debug**: debugger symbols; no optimizations (more than 10x slower).
      Library names end with `_d`.
    * **Release**: no debugger symbols; optimized.
    * **RelWithDebInfo**: debugger symbols; optimized. Bigger but not slower
      than Release; choose this if unsure.
    * **MinSizeRel**: minimum size; optimized.

    You at least want release libraries (the last 3 count as release), but you
    can have debug libraries coexist with them. To do this, go through the
    installation process twice, once for each of the two configurations. You
    should install the release configuration *last* to ensure that you use the
    release version of the command-line applications instead of the slow debug
    versions.
3. Build the API documentation. This is optional, and you can only do this if
   you have Doxygen. Build the documentation by right-clicking **doxygen** and
   selecting **Build**.
4. Build the libraries, etc. by right-clicking  **ALL_BUILD** and selecting
   **Build**.
5. Run the tests by right-clicking **RUN_TESTS_PARALLEL** and selecting
   **Build**.
6. Install OpenSim-Core by right-clicking **INSTALL** and selecting **Build**.

#### Set environment variables

In order to use the OpenSim-Core command-line applications or use OpenSim-Core
libraries in your own application, you must add the OpenSim-Core `bin/`
directory to your `PATH` environment variable.

1. In the Windows toolbar (Windows 10), Start screen (Windows 8) or Start menu
   (Windows 7), search `environment`.
2. Select **Edit the system environment variables**.
3. Click **Environment Variables...**.
4. Under **System variables**, click **Path**, then click **Edit**.
5. Add **C:/opensim-core/bin;** to the front of of the text field. Don't forget
   the semicolon!

#### For the impatient (Windows)

* Get **Visual Studio Community**
  [2015](https://www.visualstudio.com/vs/older-downloads/) or
  [2017](https://www.visualstudio.com/en-us/downloads/download-visual-studio-vs.aspx).
  * 2015: Choose *Custom installation*, then choose
    *Programming Languages* -> *Visual C++*.
  * 2017: Choose the workload *Desktop Development with C++*.
* Get **git** from [here](https://git-scm.com/downloads).
  * Choose *Use Git from the Windows Command Prompt*.
* Get **CMake** from [here](https://cmake.org/download/).
  * Choose *Add CMake to the system PATH for all users*.
* Get **Chocolatey** from [here](https://chocolatey.org/).
* In **PowerShell**, *run as Administrator*:

  ```powershell
  choco install python2 jdk8 swig
  ```
* In **PowerShell** (if using Visual Studio 2017, replace
  *14 2015* with *15 2017*):

  ```powershell
  git clone https://github.com/opensim-org/opensim-core.git
  mkdir opensim_dependencies_build
  cd .\opensim_dependencies_build
  cmake ..\opensim-core\dependencies                             `
        -G"Visual Studio 14 2015 Win64"                          `
        -DCMAKE_INSTALL_PREFIX="..\opensim_dependencies_install"
  cmake --build . --config RelWithDebInfo -- /maxcpucount:8
  cd ..
  mkdir opensim_build
  cd .\opensim_build
  cmake ..\opensim-core                                              `
        -G"Visual Studio 14 2015 Win64"                              `
        -DCMAKE_INSTALL_PREFIX="..\opensim_install"                  `
        -DOPENSIM_DEPENDENCIES_DIR="..\opensim_dependencies_install" `
        -DBUILD_JAVA_WRAPPING=ON                                     `
        -DBUILD_PYTHON_WRAPPING=ON                                   `
        -DWITH_BTK=ON                                  
  cmake --build . --config RelWithDebInfo -- /maxcpucount:8
  ctest --build-config RelWithDebInfo --parallel 8
  cmake --build . --config RelWithDebInfo --target install -- /maxcpucount:8
  ```

Note: Please add `<FULL-DIR>\opensim_install\bin` to your PATH variable as per [these instructions](#set-environment-variables).  
Example: If `opensim_install` is in `C:`, add `C:\opensim_install\bin` to your PATH.  

On Mac OSX using Xcode
======================

### For Mac OSX 10.11 El Capitan
Get **Xcode** from the App store. Open **Xcode** and *Agree* to license agreement. To *Agree* to to the license agreement, you may need to type in **Terminal**:
```shell
sudo xcodebuild -license
```
If you already have **Xcode**, update it to 7.3, or the latest version.

Then, in **Terminal**, copy and paste commands below, line by line, one at a time. The first line installs the Homebrew package manager; omit this line if you already have Homebrew. Be sure the output doesn't contain errors.
```shell
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
brew install cmake swig gcc pkgconfig autoconf libtool automake wget doxygen
brew cask install java
git clone https://github.com/opensim-org/opensim-core.git
mkdir opensim_dependencies_build
cd opensim_dependencies_build
cmake ../opensim-core/dependencies \
      -DCMAKE_INSTALL_PREFIX="~/opensim_dependencies_install" \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j8
cd ..
mkdir opensim_build
cd opensim_build
cmake ../opensim-core \
      -DCMAKE_INSTALL_PREFIX="~/opensim_install" \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DBUILD_PYTHON_WRAPPING=ON \
      -DBUILD_JAVA_WRAPPING=ON \
      -DOPENSIM_DEPENDENCIES_DIR="~/opensim_dependencies_install" \
      -DWITH_BTK=ON
make -j8
ctest -j8
```


### Extended Instructions for OSX

#### Get the dependencies

* **operating system**: Mac OSX 10.11 El Capitan or newer.
* **cross-platform build system**:
  [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.2
* **compiler / IDE**: [Xcode](https://developer.apple.com/xcode/) >= 7.3 (the latest version), through
  the Mac App Store.
* **physics engine**: Simbody >= 3.7. Two options:
  * Let OpenSim get this for you using superbuild (see below).
  * [Build on your own](https://github.com/simbody/simbody#installing).
* **C3D file support**: Biomechanical-ToolKit Core. Two options:
  * Let OpenSim get this for you using superbuild (see below).
  * [Build on your own](https://github.com/klshrinidhi/BTKCore).
* **command-line argument parsing**: docopt.cpp. Two options:
    * Let OpenSim get this for you using superbuild (see below); much easier!
    * [Build on your own](https://github.com/docopt/docopt.cpp) (no instructions).
* **logging**: spdlog. Two options:
    * Let OpenSim get this for you using superbuild (see below); much easier!
    * [Build on your own](https://github.com/gabime/spdlog).
* **Moco optimal control solvers**: Solving optimal control problems with 
    OpenSim's Moco module requires either CasADi or Tropter. Use superbuild for
    all these dependencies (except for Tropter, which is part of this
    repository).
    * **MocoCasADiSolver** (optional): [CasADi](https://web.casadi.org) (LGPL).
    * **MocoTropterSolver** (optional): Tropter.
        * **matrix library**: [Eigen](http://eigen.tuxfamily.org) >= 3.3.7
        * **sparse matrix algorithms**: [ColPack](https://github.com/CSCsw/ColPack).
        * **automatic differentiation**: [ADOL-C](https://github.com/coin-or/ADOL-C) >= 2.6.3.
    * **nonlinear optimizer** (required if building with CasADi or Tropter): 
        [IPOPT](https://github.com/coin-or/Ipopt) >= 3.12.8.
* **API documentation** (optional):
  [Doxygen](http://www.stack.nl/~dimitri/doxygen/download.html) >= 1.8.6
* **version control** (optional): git.
    * Xcode Command Line Tools gives you git on the command line.
    * [GitHub for Mac](https://mac.github.com), for a simple-to-use GUI.
* **Bindings** (optional): [SWIG](http://www.swig.org/) 3.0.8
    * **MATLAB scripting** (optional): [Java development kit][java] >= 1.7.
        * Note: Older versions of MATLAB may use an older version of JVM. Run
                'ver' in MATLAB to check MATLAB's JVM version (must be >= 1.7).
        * Note: Java development kit >= 9 requires CMake >= 3.10.
    * **Python scripting** (optional): Python 2 >= 2.7 or Python 3 >= 3.5
        * Mac OSX comes with Python, but you could also use:
        * [`brew install python`](http://brew.sh),
        * [Anaconda](https://www.anaconda.com/distribution/)
        * Must provide the NumPy package; this should come with Anaconda.

You can get most of these dependencies using [Homebrew](http://brew.sh):

    $ brew install cmake doxygen swig

#### Download the OpenSim-Core source code

* Method 1; If you want to get going quickly, download the source code from
  https://github.com/opensim-org/opensim-core/releases, for the version of
  OpenSim you want. We'll assume you unzipped the source code into
  `~/opensim-core-source`.
* Method 2: If you plan on updating your OpenSim installation or you want to
  contribute back to the project, clone the opensim-core git repository into
  `~/opensim-core-source`. Run the following in a terminal, or
  find a way to run the equivalent commands in a GUI client:

        $ git clone https://github.com/opensim-org/opensim-core.git ~/opensim-core-source

  This will give you a bleeding-edge version of OpenSim-Core.

#### [Optional] Superbuild: download and build OpenSim dependencies
1. Open the CMake GUI.
2. In the field **Where is the source code**, specify
   `~/opensim-core-source/dependencies`.
3. In the field **Where to build the binaries**, specify a directory under
   which to build dependencies. Let's say this is
   `~/opensim-core-dependencies-build`.
4. Click the **Configure** button. Choose **Xcode**. Click **Finish**.
5. Where do you want to install OpenSim dependencies on your computer? Set this
   by changing the `CMAKE_INSTALL_PREFIX` variable. Let's say this is
   `~/opensim-core-dependencies-install`.
6. Variables named `SUPERBUILD_<dependency-name>` allow you to selectively
   download dependencies. By default, all dependencies are downloaded,
   configured and built.
7. Click the **Configure** button again. Then, click **Generate** to make Xcode
   files in the build directory.
8. Open `~/opensim-core-dependencies/build/OpenSimDependencies.xcodeproj` in
   Xcode.
9. Choose your **Build Configuration** for the **ALL_BUILD** Scheme by pressing
   `Command-Shift ,` (or, `Command-LessThan`), or navigating to **Product ->
   Scheme -> Edit Scheme...**; and changing the **Build Configuration** field.
    * **Debug**: debugger symbols; no optimizations (more than 10x slower).
    Library names end with `_d`.
    * **Release**: no debugger symbols; optimized.
    * **RelWithDebInfo**: debugger symbols; optimized. Bigger but not slower
    than Release; choose this if unsure.
    * **MinSizeRel**: minimum size; optimized.

    You must build each of the configurations you plan to use with OpenSim (see
    below). You should install the release configuration *last* to ensure that
    you use the release version of the command-line applications instead of the
    slow debug versions.
10. Compile. Run the Scheme **ALL_BUILD** by clicking the play button in the
   upper left. If necessary, change the build configuration (previous step) and
   run **ALL_BUILD** again.

    * Note: Superbuild attempts to determine when dependencies are out of date
      but is not always successful. It is therefore recommended to build all
      dependencies from scratch when updating your installation.

#### Configure and generate project files

1. Open the CMake GUI.
2. In the field **Where is the source code**, specify `~/opensim-core-source`.
3. In the field **Where to build the binaries**, specify something like
   `~/opensim-core-build`, or some other path that is not inside your source
   directory. This is *not* where we are installing OpenSim-Core; see below.
4. Click the **Configure** button. Choose **Xcode**. Click **Finish**.
5. Where do you want to install OpenSim-Core on your computer? Set this by
   changing the `CMAKE_INSTALL_PREFIX` variable. We'll assume you set it to
   `~/opensim-core`. If you choose a different installation location, make
   sure to use *yours* where we use `~/opensim-core` below. You should *not*
   use `/usr/`, `/usr/local/`, etc. (because our installation does not yet
   conform to the [FHS](http://www.pathname.com/fhs/)).
6. Tell CMake where to find dependencies. This depends on how you got them.
    * Superbuild: Set the variable `OPENSIM_DEPENDENCIES_DIR` to the root
      directory you specified with superbuild for installation of dependencies.
      In our example, it would be `~/opensim-core-dependencies-install`.
    * Obtained on your own:
        1. Simbody: Set the `SIMBODY_HOME` variable to where you installed
           Simbody (e.g., `~/simbody`). If you installed Simbody using `brew`,
           then CMake will find Simbody automatically.
        2. BTK: Set the `BTK_DIR` variable to the directory containing
           `BTKConfig.cmake`. If you installed BTK in `~/BTKCore-install`, then
           set `BTK_DIR` to `~/BTKCore-install/share/btk-0.4dev`
        3. docopt.cpp: Set the variable `docopt_DIR` to the directory
           containing `docopt-config.cmake`. If the root directory of your
           docopt.cpp installation is `~/docopt.cpp-install`, then set this
           variable to `~/docopt.cpp-install/lib/cmake`.
        4. spdlog: Set the variable `spdlog_DIR` to the directory containing
           `spdlogConfig.cmake`. If the root directory of your spdlog 
           installation is `~/spdlog-install`, then set this variable to
           `~/spdlog-install/lib/spdlog/cmake`.
7. Set the remaining configuration options.
    * `BUILD_API_EXAMPLES` to compile C++ API examples.
    * `BUILD_TESTING` to ensure that OpenSim works correctly. The tests take a
      while to build; if you want to build OpenSim quickly, you can turn this
      off.
    * `BUILD_JAVA_WRAPPING` if you want to access OpenSim through MATLAB or
      Java; see dependencies above.
    * `BUILD_PYTHON_WRAPPING` if you want to access OpenSim through Python; see
      dependencies above. CMake sets `PYTHON_*` variables to tell you the
      Python version used when building the wrappers.
    * `OPENSIM_PYTHON_VERSION` to choose if the Python wrapping is built for
      Python 2 or Python 3.
    * `BUILD_API_ONLY` if you don't want to build the command-line applications.
8. Click the **Configure** button again. Then, click **Generate** to create
   Xcode project files in the build directory.

#### Build and install

1. Open `~/opensim-core-build/OpenSim.xcodeproj` in Xcode.
2. Choose your **Build Configuration** for the **ALL_BUILD** Scheme by pressing
   `Command-Shift ,` (or, `Command-LessThan`), or navigating to **Product ->
   Scheme -> Edit Scheme...**; and changing the **Build Configuration** field.
    * **Debug**: debugger symbols; no optimizations (more than 10x slower).
    Library names end with `_d`.
    * **Release**: no debugger symbols; optimized.
    * **RelWithDebInfo**: debugger symbols; optimized. Bigger but not slower
    than Release; choose this if unsure.
    * **MinSizeRel**: minimum size; optimized.

    You at least want release libraries (the last 3 count as release), but you
    can have debug libraries coexist with them. To do this, go through the
    installation process twice, once for each of the two configurations. You
    should install the release configuration *last* to ensure that you use the
    release version of the command-line applications instead of the slow debug
    versions.

3. Compile. Run the Scheme **ALL_BUILD** by clicking the play button in the
   upper left.
4. Test. Click on **ALL_BUILD** in the upper left, and select
   **RUN_TESTS_PARALLEL**. Change the **Build Configuration** of this Scheme to
   the same as you used for **ALL_BUILD** (using the same instructions as
   above). Click the play button.
5. Build the API documentation. This is optional, and you can only do this if
   you have Doxygen. Click on the current Scheme (**RUN_TESTS_PARALLEL**) and
   select **doxygen**. Click the play button.
6. Install. Click on the current Scheme (**RUN_TESTS_PARALLEL** or
   **doxygen**), and select **install**. Click the play button.

#### Set environment variables

1. **Executables**. If you want to run OpenSim-Core's executables from
   anywhere on your computer, you must update your PATH. Open a terminal and
   type:

        $ echo 'export PATH=~/opensim-core/bin:$PATH' >> ~/.bash_profile

Your changes will only take effect in new terminal windows.


On Ubuntu using Unix Makefiles
==============================

#### Get the dependencies

Most dependencies can be obtained via the Ubuntu software repositories;
especially if you are using Ubuntu 16.04 or later. On each line below, we show
the Ubuntu package names for the dependencies. You can find instructions for
specific Ubuntu versions under 'For the impatient' below.

* **cross-platform build system**:
  [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.2;
  `cmake-gui`.
* **compiler**: [gcc](http://gcc.gnu.org) >= 4.9; `g++-4.9`, or
  [Clang](http://clang.llvm.org) >= 3.4; `clang-3.4`.
* **physics engine**: Simbody >= 3.7. Two options:
  * Let OpenSim get this for you using superbuild (see below).
  * [Build on your own](https://github.com/simbody/simbody#installing).
* **C3D file support**: Biomechanical-ToolKit Core. Two options:
  * Let OpenSim get this for you using superbuild (see below).
  * [Build on your own](https://github.com/klshrinidhi/BTKCore).
* **command-line argument parsing**: docopt.cpp. Two options:
    * Let OpenSim get this for you using superbuild (see below); much easier!
    * [Build on your own](https://github.com/docopt/docopt.cpp) (no instructions).
* **logging**: spdlog. Two options:
    * Let OpenSim get this for you using superbuild (see below); much easier!
    * [Build on your own](https://github.com/gabime/spdlog).
* **Moco optimal control solvers**: Solving optimal control problems with 
    OpenSim's Moco module requires either CasADi or Tropter. Use superbuild for
    all these dependencies (except for Tropter, which is part of this
    repository).
    * **MocoCasADiSolver** (optional): [CasADi](https://web.casadi.org) (LGPL).
    * **MocoTropterSolver** (optional): Tropter.
        * **matrix library**: [Eigen](http://eigen.tuxfamily.org) >= 3.3.7
        * **sparse matrix algorithms**: [ColPack](https://github.com/CSCsw/ColPack).
        * **automatic differentiation**: [ADOL-C](https://github.com/coin-or/ADOL-C) >= 2.6.3.
    * **nonlinear optimizer** (required if building with CasADi or Tropter): 
        [IPOPT](https://github.com/coin-or/Ipopt) >= 3.12.8.
* **API documentation** (optional):
  [Doxygen](http://www.stack.nl/~dimitri/doxygen/download.html) >= 1.8.6;
  `doxygen`.
* **version control** (optional): git; `git`.
* **Bindings** (optional): [SWIG](http://www.swig.org/) 3.0.8; `swig`.
    * **MATLAB scripting** (optional): [Java development kit][java] >= 1.7;
      `openjdk-7-jdk`.
        * Note: Older versions of MATLAB may use an older version of JVM. Run
                'ver' in MATLAB to check MATLAB's JVM version (must be >= 1.7).
        * Note: Java development kit >= 9 requires CMake >= 3.10.
    * **Python scripting** (optional): Python 2 >= 2.7 or Python 3 >= 3.5; `python-dev`.
        * Must provide the NumPy package; `python-numpy`.

For example, you could get the required dependencies (except Simbody) via:

    $ sudo apt-get install cmake-gui g++-4.9

And you could get all the optional dependencies via:

    $ sudo apt-get install doxygen git swig openjdk-7-jdk python-dev python-numpy wget build-essential libtool autoconf pkg-config gfortran

#### Download the OpenSim-Core source code

* Method 1; If you want to get going quickly, download the source code from
  https://github.com/opensim-org/opensim-core/releases, for the version of
  OpenSim you want. We'll assume you unzipped the source code into
  `~/opensim-core-source`.
* Method 2: If you plan on updating your OpenSim installation or you want to
  contribute back to the project, clone the opensim-core git repository into
  `C:/opensim-core-source`. Run the following in a terminal:

        $ git clone https://github.com/opensim-org/opensim-core.git ~/opensim-core-source

  This will give you a bleeding-edge version of OpenSim-Core.

#### [Optional] Superbuild: download and build OpenSim dependencies
1. Open the CMake GUI.
2. In the field **Where is the source code**, specify
   `~/opensim-core-source/dependencies`.
3. In the field **Where to build the binaries**, specify a directory under
   which to build dependencies. Let's say this is
   `~/opensim-core-dependencies-build`.
4. Click the **Configure** button. Choose *Unix Makefiles*. Click **Finish**.
5. Where do you want to install OpenSim dependencies on your computer? Set this
   by changing the `CMAKE_INSTALL_PREFIX` variable. Let's say this is
   `~/opensim-core-dependencies-install`.
6. Variables named `SUPERBUILD_<dependency-name>` allow you to selectively
   download dependencies. By default, all dependencies are downloaded,
   configured and built.
7. Choose your build type by setting `CMAKE_BUILD_TYPE` to one of the following:
    * **Debug**: debugger symbols; no optimizations (more than 10x slower).
    Library names end with `_d`.
    * **Release**: no debugger symbols; optimized.
    * **RelWithDebInfo**: debugger symbols; optimized. Bigger but not slower
    than Release; choose this if unsure.
    * **MinSizeRel**: minimum size; optimized.

    You must perform the superbuild procedure for each of the
    build types you plan to use with OpenSim (see below). You might want to
    use different build directories for each build type, though you can use
    the same install directory for all build types. You should install the
    release build type *last* to ensure that you use the release version of
    the command-line applications instead of the slow debug versions.
8. Click the **Configure** button again. Then, click **Generate** to make Unix
   Makefiles in the build directory.
9. Open a terminal and navigate to the build directory.

        $ cd ~/opensim-core-dependencies-build

3. Compile. Use the `-jn` flag to build using `n` concurrent jobs (potentially
   in parallel); this will greatly speed up your build. For example:

        $ make -j8

    * Note: Superbuild attempts to determine when dependencies are out of date
      but is not always successful. It is therefore recommended to build all
      dependencies from scratch when updating your installation.
11. If necessary, repeat this whole procedure for other build types.

#### Configure and generate project files

1. Open the CMake GUI.
2. In the field **Where is the source code**, specify `~/opensim-core-source`.
3. In the field **Where to build the binaries**, specify something like
   `~/opensim-core-build`, or some other path that is not inside your source
   directory. This is *not* where we are installing OpenSim-Core; see below.
4. Click the **Configure** button. Choose *Unix Makefiles*. Click **Finish**.
5. Where do you want to install OpenSim-Core on your computer? Set this by
   changing the `CMAKE_INSTALL_PREFIX` variable. We'll assume you set it to
   `~/opensim-core`. If you choose a different installation location, make
   sure to use *yours* where we use `~/opensim-core` below. You should *not*
   use `/usr/`, `/usr/local/`, etc. (because our installation does not yet
   conform to the [FHS](http://www.pathname.com/fhs/)), but
   [`/opt/`](http://www.tldp.org/LDP/Linux-Filesystem-Hierarchy/html/opt.html)
   is okay.
6. Tell CMake where to find dependencies. This depends on how you got them.
    * Superbuild: Set the variable `OPENSIM_DEPENDENCIES_DIR` to the root
      directory you specified with superbuild for installation of dependencies.
      In our example, it would be `~/opensim-core-dependencies-install`.
    * Obatained on your own:
        1. Simbody: Set the `SIMBODY_HOME` variable to where you installed
           Simbody (e.g., `~/simbody`).
        2. BTK: Set the `BTK_DIR` variable to the directory containing
           `BTKConfig.cmake`. If you installed BTK in `~/BTK-install`, then set
           `BTK-DIR` to `~/BTK-install/share/btk-0.4dev`.
        3. docopt.cpp: Set the variable `docopt_DIR` to the directory
           containing `docopt-config.cmake`. If the root directory of your
           docopt.cpp installation is `~/docopt.cpp-install`, then set this
           variable to `~/docopt.cpp-install/lib/cmake`.
        4. spdlog: Set the variable `spdlog_DIR` to the directory containing
           `spdlogConfig.cmake`. If the root directory of your spdlog 
           installation is `~/spdlog-install`, then set this variable to
           `~/spdlog-install/lib/spdlog/cmake`.
7. Choose your build type by setting `CMAKE_BUILD_TYPE` to one of the following:
    * **Debug**: debugger symbols; no optimizations (more than 10x slower).
    Library names end with `_d`.
    * **Release**: no debugger symbols; optimized.
    * **RelWithDebInfo**: debugger symbols; optimized. Bigger but not slower
    than Release; choose this if unsure.
    * **MinSizeRel**: minimum size; optimized.

    You at least want release libraries (the last 3 count as release), but you
    can have debug libraries coexist with them. To do this, go through the
    installation process twice, once for each of the two build types. It is
    typical to use a different build directory for each build type (e.g.,
    `~/opensim-core-build-debug` and `~/opensim-core-build-release`). You
    should install the release build type *last* to ensure that you use the
    release version of the command-line applications instead of the slow debug
    versions.
8. Set the remaining configuration options.
    * `BUILD_API_EXAMPLES` to compile C++ API examples.
    * `BUILD_TESTING` to ensure that OpenSim works correctly. The tests take a
      while to build; if you want to build OpenSim quickly, you can turn this
      off.
    * `BUILD_JAVA_WRAPPING` if you want to access OpenSim through MATLAB or
      Java; see dependencies above.
    * `BUILD_PYTHON_WRAPPING` if you want to access OpenSim through Python; see
      dependencies above.
    * `OPENSIM_PYTHON_VERSION` to choose if the Python wrapping is built for
      Python 2 or Python 3.
    * `BUILD_API_ONLY` if you don't want to build the command-line applications.
    * `OPENSIM_COPY_DEPENDENCIES` to decide if Simbody and BTK are copied into
      the OpenSim installation; you want this off if you're installing OpenSim
      into `/usr/` or `/usr/local/`.
9. Click the **Configure** button again. Then, click **Generate** to create
   Makefiles in the build directory.

#### Build and install

1. Open a terminal and navigate to the build directory.

        $ cd ~/opensim-core-build

2. Build the API documentation. This is optional, and you can only do this if
   you have Doxygen.

        $ make doxygen

3. Compile. Use the `-jn` flag to build using `n` concurrent jobs (potentially
   in parallel); this will greatly speed up your build. For example:

        $ make -j8

4. Run the tests.

        $ ctest -j8

5. Install (to `~/opensim-core`).

        $ make -j8 install

#### Set environment variables

1. **Executables**. Add OpenSim-Core's executables to the path so you can
   access them from any directory on your computer.

        $ echo 'export PATH=~/opensim-core/bin:$PATH' >> ~/.bashrc

Your changes will only take effect in new terminal windows.

#### For the impatient (Ubuntu)
##### Ubuntu 14.04 Trusty Tahr
In **Terminal** --
```shell
sudo add-apt-repository --yes ppa:george-edison55/cmake-3.x
sudo apt-add-repository --yes ppa:fenics-packages/fenics-exp
sudo apt-get update
sudo apt-get --yes install git cmake cmake-curses-gui clang-3.6 \
                           freeglut3-dev libxi-dev libxmu-dev \
                           liblapack-dev swig3.0 python-dev \
                           openjdk-7-jdk
sudo rm -f /usr/bin/cc /usr/bin/c++
sudo ln -s /usr/bin/clang-3.6 /usr/bin/cc
sudo ln -s /usr/bin/clang++-3.6 /usr/bin/c++
export JAVA_HOME=/usr/lib/jvm/java-7-openjdk-amd64
git clone https://github.com/opensim-org/opensim-core.git
mkdir opensim_dependencies_build
cd opensim_dependencies_build
cmake ../opensim-core/dependencies/ \
      -DCMAKE_INSTALL_PREFIX='~/opensim_dependencies_install' \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j8
cd ..
mkdir opensim_build
cd opensim_build
cmake ../opensim-core \
      -DCMAKE_INSTALL_PREFIX="~/opensim_install" \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DOPENSIM_DEPENDENCIES_DIR="~/opensim_dependencies_install" \
      -DBUILD_PYTHON_WRAPPING=ON \
      -DBUILD_JAVA_WRAPPING=ON \
      -DWITH_BTK=ON
make -j8
ctest -j8
make -j8 install
 ```
##### Ubuntu 15.10 Wily Werewolf
In **Terminal** --
```shell
sudo apt-add-repository --yes ppa:fenics-packages/fenics
sudo apt-get update
sudo apt-get --yes install git cmake cmake-curses-gui \
                           freeglut3-dev libxi-dev libxmu-dev \
                           liblapack-dev swig3.0 python-dev \
                           openjdk-8-jdk
export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
git clone https://github.com/opensim-org/opensim-core.git
mkdir opensim_dependencies_build
cd opensim_dependencies_build
cmake ../opensim-core/dependencies/ \
      -DCMAKE_INSTALL_PREFIX='~/opensim_dependencies_install' \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j8
cd ..
mkdir opensim_build
cd opensim_build
cmake ../opensim-core \
      -DCMAKE_INSTALL_PREFIX="~/opensim_install" \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DOPENSIM_DEPENDENCIES_DIR="~/opensim_dependencies_install" \
      -DBUILD_PYTHON_WRAPPING=ON \
      -DBUILD_JAVA_WRAPPING=ON \
      -DWITH_BTK=ON
make -j8
ctest -j8
make -j8 install
```
##### Ubuntu 16.04 Xenial Xerus AND Ubuntu 16.10 Yakkety Yak
In **Terminal** --
```shell
sudo apt-get update
sudo apt-get --yes install git cmake cmake-curses-gui \
                           freeglut3-dev libxi-dev libxmu-dev \
                           liblapack-dev swig python-dev \
                           openjdk-8-jdk
export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
git clone https://github.com/opensim-org/opensim-core.git
mkdir opensim_dependencies_build
cd opensim_dependencies_build
cmake ../opensim-core/dependencies/ \
      -DCMAKE_INSTALL_PREFIX='~/opensim_dependencies_install' \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j8
cd ..
mkdir opensim_build
cd opensim_build
cmake ../opensim-core \
      -DCMAKE_INSTALL_PREFIX="~/opensim_install" \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DOPENSIM_DEPENDENCIES_DIR="~/opensim_dependencies_install" \
      -DBUILD_PYTHON_WRAPPING=ON \
      -DBUILD_JAVA_WRAPPING=ON \
      -DWITH_BTK=ON
make -j8
ctest -j8
make -j8 install
```
Note: You may need to add `<FULL-DIR>/opensim_install/bin` to your PATH variable as per [these instructions](#set-environment-variables-2).  
Example: If opensim_install is in your home directory:

        $ echo 'export PATH=~/opensim_install/bin:$PATH' >> ~/.bashrc


[buildstatus_image_travis]: https://travis-ci.org/opensim-org/opensim-core.svg?branch=master
[travisci]: https://travis-ci.org/opensim-org/opensim-core
[buildstatus_image_appveyor]: https://ci.appveyor.com/api/projects/status/i4wxnmx9jlk69kge/branch/master?svg=true
[appveyorci]: https://ci.appveyor.com/project/opensim-org/opensim-core/branch/master
[zenhub_image]: https://dxssrr2j0sq4w.cloudfront.net/3.2.0/img/external/zenhub-badge.png
[zenhub]: https://zenhub.com

[running_gif]: doc/images/opensim_running.gif
[simple_example_gif]: doc/images/opensim_double_pendulum_muscle.gif
[java]: http://www.oracle.com/technetwork/java/javase/downloads/index.html
