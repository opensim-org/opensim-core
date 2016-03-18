OpenSim Core [![Travis][buildstatus_image_travis]][travisci] [![Appveyor][buildstatus_image_appveyor]][appveyorci]
============

**NOTE: This repository contains OpenSim 4.0 development and cannot be used to build OpenSim 3.x or earlier. For OpenSim 3.x, see [here](http://simtk-confluence.stanford.edu:8080/display/OpenSim/Building+OpenSim+from+Source).**

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


Simple example
--------------
Let's simulate a simple arm whose elbow is actuated by a muscle:
```cpp
#include <OpenSim/OpenSim.h>
using namespace SimTK;
using namespace OpenSim; using OpenSim::Body;
int main() {
    Model model; model.setUseVisualizer(true);
    // Two links, with mass of 1 kg, center of mass at the
    // origin of the body's frame, and moments/products of inertia of zero.
    OpenSim::Body* link1 = new OpenSim::Body("humerus", 1, Vec3(0), Inertia(0));
    OpenSim::Body* link2 = new OpenSim::Body("radius", 1, Vec3(0), Inertia(0));

    // Joints that connect the bodies together.
    PinJoint* joint1 = new PinJoint("shoulder",
            // Parent body, location in parent, orientation in parent.
            model.getGround(), Vec3(0), Vec3(0),
            // Child body, location in child, orientation in child.
            *link1, Vec3(0, 1, 0), Vec3(0));
    PinJoint* joint2 = new PinJoint("elbow",
            *link1, Vec3(0), Vec3(0), *link2, Vec3(0, 1, 0), Vec3(0));

    // Add an actuator that crosses the elbow, and a joint stop.
    Millard2012EquilibriumMuscle* muscle = new
        Millard2012EquilibriumMuscle("biceps", 200, 0.6, 0.55, 0);
    muscle->addNewPathPoint("point1", *link1, Vec3(0, 0.8, 0));
    muscle->addNewPathPoint("point2", *link2, Vec3(0, 0.7, 0));

    // A controller that specifies the excitation of the biceps muscle.
    PrescribedController* brain = new PrescribedController();
    brain->addActuator(*muscle);
    // Muscle excitation is 0.3 for the first 0.5 seconds, and 1.0 thereafter.
    brain->prescribeControlForActuator("biceps",
            new StepFunction(0.5, 3, 0.3, 1));

    // Add bodies and joints to the model.
    model.addBody(link1); model.addBody(link2);
    model.addJoint(joint1); model.addJoint(joint2);
    model.addForce(muscle);
    model.addController(brain);

    // Configure the model.
    State& state = model.initSystem();
    // Fix shoulder joint, flex elbow joint.
    model.updCoordinateSet()[0].setLocked(state, true);
    model.updCoordinateSet()[1].setValue(state, 0.5 * Pi);
    model.equilibrateMuscles(state);

    // Add display geometry.
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundColor(Vec3(1, 1, 1));
    // Ellipsoids: 0.5 m radius along y axis, centered 0.5 m up along y axis.
    DecorativeEllipsoid geom(Vec3(0.1, 0.5, 0.1)); Vec3 center(0, 0.5, 0);
    viz.addDecoration(link1->getMobilizedBodyIndex(), Transform(center), geom);
    viz.addDecoration(link2->getMobilizedBodyIndex(), Transform(center), geom);

    // Simulate.
    RungeKuttaMersonIntegrator integrator(model.getSystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0); manager.setFinalTime(10.0);
    manager.integrate(state);
};
```

This code produces the following animation:

![Simulation of an arm actuated by a muscle][simple_example_gif]

---


Building from the source code
-----------------------------

We support a few ways of building OpenSim:

1. [On Windows using Microsoft Visual Studio](#on-windows-using-visual-studio). In a rush? Use [these instructions](#for-the-impatient-windows). 
2. [On Mac OSX using Xcode](#on-mac-osx-using-xcode). In a rush? Use [these instructions](#for-the-impatient-mac-os-x).
3. [On Ubuntu using Unix Makefiles](#on-ubuntu-using-unix-makefiles). In a rush? Use [these instructions](#for-the-impatient-ubuntu).


On Windows using Visual Studio
------------------------------

#### Get the dependencies

* **operating system**: Windows 7 or 8.
* **cross-platform build system**:
  [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.1.3
* **compiler / IDE**: [Visual Studio 2015](https://www.visualstudio.com/).
    * *Visual Studio Community 2015* is sufficient and is free for everyone.
        If you want to use *Visual Studio Enterprise 2015*, you may be able
        to get it for free at [Dreamspark](https://www.dreamspark.com) if
        you are at an academic institution.
    * Visual Studio 2015 does not install C++
      support by default. During the installation you must select
      *Custom*, and check *Programming Languages > Visual C++ > Common Tools
      for Visual C++ 2015*.
      You can uncheck all other boxes. If you have already installed Visual
      Studio without C++ support, simply re-run the installer and select *Modify*.
* **physics engine**: Simbody >= 3.6. Two options:
    * Let OpenSim get this for you using superbuild (see below).
    * [Build on your own](
      https://github.com/simbody/simbody#windows-using-visual-studio).
* **C3D file support**: Biomechanical-ToolKit Core. Two options:
    * Let OpenSim get this for you using superbuild (see below).
    * [Build on your own](https://github.com/klshrinidhi/BTKCore).
* **API documentation** (optional):
  [Doxygen](http://www.stack.nl/~dimitri/doxygen/download.html) >= 1.8.6
* **version control** (optional): git. There are many options:
    * [Git for Windows](http://msysgit.github.io/), most advanced;
    * [TortoiseGit](https://code.google.com/p/tortoisegit/wiki/Download),
      intermediate; good for TortoiseSVN users;
    * [GitHub for Windows](https://windows.github.com/), easiest.
* **Bindings** (optional): [SWIG](http://www.swig.org/) 3.0.5
    * **MATLAB scripting** (optional): [Java development kit][java] 1.7.
    * **python scripting** (optional):
        * [Enthought Canopy](https://www.enthought.com/products/canopy/), or
        * [Anaconda](https://store.continuum.io/cshop/anaconda/)
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
    1. Choose the *Visual Studio 14* generator (for Visual Studio 2015). To
       build as 64-bit, select *Visual Studio 14 Win64*.
    2. Click **Finish**.
5. Where do you want to install OpenSim dependencies on your computer? Set this
   by changing the `CMAKE_INSTALL_PREFIX` variable. Let's say this is
   `C:/opensim-core-dependencies-install`.
6. Variables named `SUPERBUILD_<dependency-name>` allow you to selectively
   download dependencies. By default, all dependencies are downloaded,
   configured and built.
7. Click the **Configure** button again. Then, click **Generate** to make
   Visual Studio project files in the build directory.
9. Go to the build directory you specified in step 3 using the command:

        cd C:/opensim-core-dependencies-build

10. Use CMake to download, compile and install the dependencies:

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
11. If you like, you can now remove the directory used for building
    dependencies (`c:/opensim-core-dependencies-build`).

#### Configure and generate project files

1. Open the CMake GUI.
2. In the field **Where is the source code**, specify `C:/opensim-core-source`.
3. In the field **Where to build the binaries**, specify something like
   `C:/opensim-core-build`, or some other path that is not inside your source
   directory. This is *not* where we are installing OpenSim-Core; see below.
4. Click the **Configure** button.
    1. Choose the *Visual Studio 14* generator (for Visual Studio 2015). To
       build as 64-bit, select *Visual Studio 14 Win64*. The choice between
       32-bit/64-bit must be the same across all dependencies.
    2. Click **Finish**.
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
7. Set the remaining configuration options.
    * `BUILD_EXAMPLES` to compile C++ API examples.
    * `BUILD_TESTING` to ensure that OpenSim works correctly. The tests take a
      while to build; if you want to build OpenSim quickly, you can turn this
      off.
    * `BUILD_JAVA_WRAPPING` if you want to access OpenSim through MATLAB or
      Java; see dependencies above.
    * `BUILD_PYTHON_WRAPPING` if you want to access OpenSim through Python; see
      dependencies above. CMake sets `PYTHON_*` variables to tell you the
      Python it will use for building the wrappers.
    * `BUILD_API_ONLY` if you don't want to build the command-line applications.
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

* Get **Visual Studio Community** from [here](https://www.visualstudio.com/en-us/downloads/download-visual-studio-vs.aspx).
 * Choose *Custom' installation*.
 * Choose *Programming Languages* -> *Visual C++*.
* Get **git** from [here](https://git-scm.com/downloads).
 * Choose *Use Git from the Windows Command Prompt*.
* Get **CMake** from [here](https://cmake.org/download/).
 * Choose *Add CMake to the system PATH for all users*.
* Get **Chocolatey** from [here](https://chocolatey.org/).
* In **PowerShell**, *run as Administrator* --

 ```powershell
 choco install python
 choco install jdk8
 choco install swig
 ```
* In **PowerShell** --

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
ctest -C RelWithDebInfo --parallel 8
```

On Mac OSX using Xcode
======================

#### Get the dependencies

* **operating system**: Mac OSX 10.8 or later.
* **cross-platform build system**:
  [CMake](http://www.cmake.org/cmake/resources/software.html) >= 2.8.8
* **compiler / IDE**: [Xcode](https://developer.apple.com/xcode/) >= 5, through
  the Mac App Store.
* **physics engine**: Simbody >= 3.6. Two options:
  * Let OpenSim get this for you using superbuild (see below).
  * [Build on your own](https://github.com/simbody/simbody#installing).
* **C3D file support**: Biomechanical-ToolKit Core. Two options:
  * Let OpenSim get this for you using superbuild (see below).
  * [Build on your own](https://github.com/klshrinidhi/BTKCore).
* **API documentation** (optional):
  [Doxygen](http://www.stack.nl/~dimitri/doxygen/download.html) >= 1.8.6
* **version control** (optional): git.
    * Xcode Command Line Tools gives you git on the command line.
    * [GitHub for Mac](https://mac.github.com), for a simple-to-use GUI.
* **Bindings** (optional): [SWIG](http://www.swig.org/) 3.0.5
    * **MATLAB scripting** (optional): [Java development kit][java] 1.7.
    * **python scripting** (optional):
        * Mac OSX comes with python, but you could also use:
        * [`brew install python`](http://brew.sh),
        * [Enthought Canopy](https://www.enthought.com/products/canopy/), or
        * [Anaconda](https://store.continuum.io/cshop/anaconda/)

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
7. Set the remaining configuration options.
    * `BUILD_EXAMPLES` to compile C++ API examples.
    * `BUILD_TESTING` to ensure that OpenSim works correctly. The tests take a
      while to build; if you want to build OpenSim quickly, you can turn this
      off.
    * `BUILD_JAVA_WRAPPING` if you want to access OpenSim through MATLAB or
      Java; see dependencies above.
    * `BUILD_PYTHON_WRAPPING` if you want to access OpenSim through Python; see
      dependencies above. CMake sets `PYTHON_*` variables to tell you the
      Python it will use for building the wrappers. (If you installed python
      with homebrew, [CMake will not find the homebrew python libraries on its
      own](https://github.com/Homebrew/homebrew/issues/25118); you must set the
      CMake variable `PYTHON_LIBRARIES` manually. Use `'$(python-config
      --prefix)/lib/libpython2.7.dylib'` in bash to get the correct value.)
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
   anywhere on your computer, you must update your PATH. *Note* some of the
   names of OpenSim-Core executables conflict with some UNIX commands (e.g.,
   `id`). To give preference to OpenSim-Core's executables, we must *prepend*
   OpenSim-Core's `bin/` directory to the path. Open a terminal and type:

        $ echo 'export PATH=~/opensim-core/bin:$PATH' >> ~/.bash_profile

2. **Libraries**. Hopefully you can skip this step. This step is required if:
  1. You are using CMake version 2.8.11 or older.
  2. You plan on building C++ executables or libraries on top of OpenSim, *and*
     you plan to "install" them in the CMake sense of the word (that is, you're
     not going to simply use them from your project's build directory).
  3. You plan to use the Java or MATLAB scripting.

  If any of these are true, then you must add OpenSim-Core libraries to your
  linker path. Open a terminal and type:

          $ echo 'export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:~/opensim-core/lib' >> ~/.bash_profile

Your changes will only take effect in new terminal windows.

#### For the impatient (Mac OS X)
##### Mac OS X 10.10 Yosemite and OS X 10.11 El Capitan
Get **Xcode** from the App store. Open **Xcode** and *Agree* to license agreement.
In **Terminal** --
```shell
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
brew install cmake swig
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
On Ubuntu using Unix Makefiles
==============================

#### Get the dependencies

Most dependencies can be obtained via the Ubuntu software repositories. On each
line below, we show the corresponding package.

* **operating system**: Ubuntu 13.10 or later.
* **cross-platform build system**:
  [CMake](http://www.cmake.org/cmake/resources/software.html) >= 2.8.8;
  `cmake-gui`. Ubuntu 12.04 only has 2.8.6 available; download from the website
  or from this [third party
  PPA](https://launchpad.net/~robotology/+archive/ubuntu/ppa).
* **compiler**: [gcc](http://gcc.gnu.org) >= 4.8; `g++-4.8`, or
  [Clang](http://clang.llvm.org) >= 3.4; `clang-3.4`.
* **physics engine**: Simbody >= 3.6. Two options:
  * Let OpenSim get this for you using superbuild (see below).
  * [Build on your own](https://github.com/simbody/simbody#installing).
* **C3D file support**: Biomechanical-ToolKit Core. Two options:
  * Let OpenSim get this for you using superbuild (see below).
  * [Build on your own](https://github.com/klshrinidhi/BTKCore).
* **API documentation** (optional):
  [Doxygen](http://www.stack.nl/~dimitri/doxygen/download.html) >= 1.8.6;
  `doxygen`.
* **version control** (optional): git; `git`.
* **Bindings** (optional): [SWIG](http://www.swig.org/) 3.0.5; must get from SWIG website.
    * **MATLAB scripting** (optional): [Java development kit][java] >= 1.7;
      `openjdk-6-jdk` or `openjdk-7-jdk`.
    * **python scripting** (optional): `python-dev`.

For example, you could get the required dependencies (except Simbody) via:

    $ sudo apt-get install cmake-gui g++-4.8

And you could get all the optional dependencies via:

    $ sudo apt-get install doxygen git swig openjdk-7-jdk python-dev

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
    * `BUILD_EXAMPLES` to compile C++ API examples.
    * `BUILD_TESTING` to ensure that OpenSim works correctly. The tests take a
      while to build; if you want to build OpenSim quickly, you can turn this
      off.
    * `BUILD_JAVA_WRAPPING` if you want to access OpenSim through MATLAB or
      Java; see dependencies above.
    * `BUILD_PYTHON_WRAPPING` if you want to access OpenSim through Python; see
      dependencies above.
    * `BUILD_API_ONLY` if you don't want to build the command-line applications.
    * `OPENSIM_COPY_SIMBODY` to decide if Simbody headers and libraries should
      be installed inside OpenSim; you want this off if you're installing
      Simbody and OpenSim into `/usr/` or `/usr/local/`.
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

1. **Executables**. Add OpenSim-Core's executables to the path so you can access them from any
   directory on your computer. NOTE that some of the names of OpenSim-Core
   executables conflict with some UNIX commands (e.g., `id`). To give
   preference to OpenSim-Core's executables, we must *prepend* OpenSim-Core's
   `bin/` directory to the path.

        $ echo 'export PATH=~/opensim-core/bin:$PATH' >> ~/.bashrc

2. **Libraries**. Allow executables to find OpenSim-Core libraries by
  adding the OpenSim-Core
   `lib/` directory to your linker path.

        $ echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/opensim-core/lib' >> ~/.bashrc

Your changes will only take effect in new terminal windows.


[buildstatus_image_travis]: https://travis-ci.org/opensim-org/opensim-core.svg?branch=master
[travisci]: https://travis-ci.org/opensim-org/opensim-core
[buildstatus_image_appveyor]: https://ci.appveyor.com/api/projects/status/i4wxnmx9jlk69kge/branch/master?svg=true
[appveyorci]: https://ci.appveyor.com/project/opensim-org/opensim-core/branch/master
[running_gif]: doc/images/opensim_running.gif
[simple_example_gif]: doc/images/opensim_double_pendulum_muscle.gif
[java]: http://www.oracle.com/technetwork/java/javasebusiness/downloads/java-archive-downloads-javase6-419409.html

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
```
