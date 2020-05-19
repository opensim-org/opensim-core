Guidelines for Developers of OpenSim-Core
=========================================
OpenSim is a community resource that is housed in the opensim-core repository.

This specific document addresses issues that come up during development and maintenance of the OpenSim code base, that may not be relevant to end users because it's not in release code yet or of historical relevance only but would be
valuable to new developers or advanced users. For general contribution guidelines please consult [CONTRIBUTING.md](https://github.com/opensim-org/opensim-core/blob/master/CONTRIBUTING.md)

Contents:

- [Backward Compatibility of File Formats](#backward-compatibility-of-file-formats)
- [CMake options for packaging a binary distribution](#cmake-options-for-packaging-a-binary-distribution)
- [Adding dependencies](#adding-dependencies)


Backward Compatibility of File Formats
--------------------------------------
OpenSim stores models and analysis-tools/objects in xml files (with extension .osim or .xml). Since the first release of OpenSim in 2007, these files have mostly survived many major format changes which is quite important considering that thousands of these files are around both in earlier OpenSim distributions and also in models created by users and/or collaborators. This section describes how backward compatibility is maintained so that if a format change is introduced, users do not lose valuable work and/or models in the process. In what follows we use the term serialization to mean writing and deserialization to mean reading.

One key piece of information that is kept with each file is a version number (usually in the header section e.g. `<OpenSimDocument Version="NNNNN">`. The same is true for .sto files that contain a header as well, though we will not discuss .sto files here. The version number encodes what the code was expecting the format to be when the file was produced/written. Keeping this number consistent with the content of the file is the fundamental invariant that needs to be maintained. In code, the version number is maintained in XMLDocument.cpp and is updated as version numbers are incremented. The actual numbers do not matter much, as long as they are monotonically increasing. OpenSim 3.2 shipped with OpenSimDocument Version number 30000. OpenSim 4.0 development started at version 30500 and will ship with 40000).

Deserialization of Objects and their Properties (primitives that are read-from/written-to files) is mostly transparent to users/developers. Once you use the property macros (e.g. ` OpenSim_DECLARE_UNNAMED_PROPERTY`) in the header and perform the necessary wiring/calls, these properties know how to serialize themselves to XML elements and back. This serialization/deserialization, however, assumes a fixed XML layout/schema. If this assumption fails (because of a property name change, layout change or addition of new properties) then deserialization will not work out of the box and the developer making the change is responsible for changing the version number and the deserialization code to be in sync with the latest code. This is very important since if the developer does not update the version number then the code and the XML files will not be in sync and it will be very hard to go back to find out what changes should have been made.

Keep in mind that deserialization methods are practically called during the construction of the top level object/Model so typically you cannot rely on having any member variables or pointers populated unless done by default constructors, instead deserialization methods operate on the XML structure only. This helps isolate the backward compatibility code from the rest of the modeling/simulation or initialization functions.

Deserialization works by making the following calls:
 1. A user constructs an object that is a subclass of `OpenSim::Object` by using a constructor that takes the file name of a .osim or .xml file (only some classes have such a constructor). The construction call sequence ends with calling `OpenSim::Object`'s constructor with the file name as an argument. This constructor invokes code that parses the XML file.
 2. The parsing code looks for XML tags that correspond to names of subclasses of `OpenSim::Object`; when it finds one of the them it performs the next 2 steps.
 3. An Object of the appropriate type is instantiated from the registry of available types using a lookup by the XML tag.
 4. The method `updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber) ` is invoked on the object and is passed the XML element corresponding to the object along with the version number from the XML document/file. That’s exactly the hook to deserialization that developers can use to correct for deserialization changes by manipulating the passed in `node` to match what the latest code expects.


If an object has never undergone format changes then it should not implement the method `updateFromXMLNode()` altogether.
If the function needs to be implemented, it would have the following form:
```cpp

void XXX::updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber)
{
       // XMLDocument::getLatestVersion() is a number that changes with upgrades
       // Guarding with this condition makes sure that files already converted
       // do not get penalized or converted again.
       if ( versionNumber < XMLDocument::getLatestVersion()) {
              if (versionNumber <= 20301) {
		         // convert node from version 20301 or prior to the next version
		          ……
              }
              if (versionNumber < 30500) {
	             // Convert versions before 30500
              }
        }
        // At this point of the code, node is on the latest XML format
        // Call base class, now assuming node has been corrected for current version
        // This call will end up being made on Object, which will do the actual population of Property values
        Super::updateFromXMLNode(node, versionNumber);
}
```

CMake options for packaging a binary distribution
-------------------------------------------------

When packaging opensim-core for distribution, it is important to set certain
CMake options correctly so that the distribution contains the necessary files
from dependencies. The variables to set depend on how opensim-core is being
distributed. Currently, opensim-core binaries are distributed through the
OpenSim GUI distribution. In this case, the following settings should be used:

    WITH_BTK=ON                                 non-default
    OPENSIM_COPY_DEPENDENCIES=ON                default
    OPENSIM_PYTHON_STANDALONE=OFF               default
    on Windows: OPENSIM_INSTALL_UNIX_FHS=OFF    default
    on UNIX:    OPENSIM_INSTALL_UNIX_FHS=ON     default
    OPENSIM_SIMBODY_DOXYGEN_LOCATION=https://simtk.org/api_docs/simbody/<version>/

The last variable causes OpenSim's doxygen documentation to link to Simbody's
documentation online.

The layout of the distribution on Windows is as follows:

  - `bin/` OpenSim, SimTK, and BTK DLLs, opensim-cmd.exe, simbody-visualizer.exe
  - `cmake/` OpenSimConfig.cmake, etc.
  - `sdk/`
    - `APIExamples/`: C++ examples.
    - `doc/` API doxygen documentation.
    - `include/` OpenSim (and Lepton) headers.
    - `Java/` Source files for Java interface, and org-opensim-modeling.jar.
    - `lib/` OpenSim "import" libraries, used during linking.
    - `Scripts/` MATLAB and Python examples/utilities.
    - `Simbody/` A copy of the Simbody installation.
      - `bin/` SimTK DLLs.
      - `cmake/` SimbodyConfig.cmake, etc.
      - `include/` Simbody headers.
      - `lib/` SimTK "import" libraries, used during linking.
    - `python/` OpenSim Python bindings.
    - `OpenSim_buildinfo.txt` Describes the compiler used to build OpenSim.

The layout of the distribution on macOS (and Linux) is as follows:

  - `bin/` opensim-cmd
  - `etc/OpenSim_buildinfo.txt` Describes the compiler used to build OpenSim.
  - `include/`
    - `OpenSim/` OpenSim (and Lepton) headers.
    - `simbody/` Simbody headers.
  - `lib/` (on some Linux variants, `lib/<arch>/`) OpenSim, SimTK, and BTK  shared libraries.
    - `cmake/` OpenSimConfig.cmake, SimbodyConfig.cmake, etc.
    - `python2.7/site-packages/` OpenSim Python bindings.
  - `libexec/simbody/simbody-visualizer`
  - `share/`
    - `doc/OpenSim/` API doxygen documentation.
      - `APIExamles/` C++ examples.
      - `Scripts/` MATLAB and Python examples/utilities.
    - `java/org-opensim-modeling.jar`
    - `OpenSim/java/` Source files for Java interface.

This layout is intended to follow the UNIX Filesystem Hierarchy Standard.

We hope to distribute opensim-core binaries through common package managers
(we already have some progress for Ubuntu, macOS/Homebrew, and Conda). In these
cases, the dependencies should be installed via their own packages so that the
OpenSim installation does not need to contain the dependencies.

    WITH_BTK=ON                                 non-default
    OPENSIM_COPY_DEPENDENCIES=OFF               non-default
    OPENSIM_PYTHON_STANDALONE=OFF               default
    on Windows: OPENSIM_INSTALL_UNIX_FHS=OFF    default
    on UNIX:    OPENSIM_INSTALL_UNIX_FHS=ON     default

Adding dependencies
-------------------
OpenSim depends on multiple C/C++ software libraries. Developers adding new
dependencies to OpenSim should ensure the following scenarios are supported on
Windows, macOS, and Linux:

- Building OpenSim from source
- Building C++ plug-ins or extensions using OpenSim's binary software
  development kit (SDK).
- Using OpenSim's Python package

Some dependencies may be used as static libraries or dynamic/shared
libraries; typically, we use dynamic libraries, but using static libraries is
permitted. Most of the following considerations are for dynamic libraries only;
we use the terms "dynamic" (Windows) and "shared" (UNIX) somewhat 
interchangeably.

There are two types of dependencies: those exposed through OpenSim's API (e.g.,
Simbody), and those used internally by OpenSim (e.g., BTK and ezc3d):

- **private**: OpenSim's binary distribution need only contain the 
  dependency's dynamic libraries.
- **public**: OpenSim's binary distribution must contain the dependency's
  dynamic libraries, CMake Config files, and header files, and Windows library
  (.lib files).

When distributing
OpenSim, various parts of these dependencies must be copied 

### RPATH on UNIX systems

UNIX systems (macOS and Linux) provide mechanisms for shared libraries and
executables to contain file system paths to help locate the shared libraries
they need. This information is called an RPATH. By properly embedding RPATHs
in OpenSim's shared libraries and executables, we avoid requiring the user from
setting the DYLD_LIBRARY_PATH environment variable.

Windows does not support RPATHs; it is for this reason that Windows users 
of OpenSim must set their Windows PATH environment variable to include OpenSim's
`bin` directory.

### OpenSim's Python package.

OpenSim's Python package `opensim` is built if the CMake option
`BUILD_PYTHON_WRAPPING` is `on`.
The Python package may be used from OpenSim's installation or can be installed
into a user's Python package directory (`OPENSIM_PYTHON_STANDALONE=ON`).

In the former case, the Python package depends on OpenSim's libraries (and the
libraries that OpenSim depends on) that exist outside of the Python package
itself. For the Python package to find these OpenSim libraries, on Windows,
users must set their `PATH` environment variable to include the OpenSim
installation's `bin` directory. On macOS and Linux, we set the RPATH of the
shared libraries in the Python package (e.g., `_common.so`) to use relative
paths that point to the OpenSim distribution's `lib` directory.

In the latter case, all libraries that the Python package depends on are copied
directly into the Python package.

TODO I think I have this kinda confused...
and Linux,

- OpenSimInstallDependencyLibraries: To 

### End-user of opensim-core.
### C++ projects that depend on opensim-core.

On Windows, building C++ code that depends on OpenSim requires that
osimCommon.lib files are available.

TODO copying dependencies.

### CMake macros 

OpenSim provides multiple CMake macros to aid with distributing dynamic/shared
libraries from dependencies. 

- **OpenSimCopyDependencyDLLsForWin()** 
- **OpenSimInstallDependencyLibraries()**

See `cmake/OpenSimMacros.cmake` for documentation.

### Steps for adding a dependency to this repository

1. Ensure the dependency can be built and installed using CMake, and that the
   dependency uses modern CMake practices to export its targets in its Package
   Configuration file; see
   https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html.
2. Ensure the layout of the dependency's installation conforms to OpenSim's
   layout as described in the section "CMake options for packaging a binary
   distribution" above.
3. Ensure the dependency provides a open-source license that is consistent with
   OpenSim's Apache License 2.0. If the license is not as permissive as 
   Apache License 2.0, the dependency must be optional.
4. In `README.md`, add a description of the dependency to each 
   "Get the dependencies" section.
5. Add to `dependencies/CMakeLists.txt` a new call to `AddDependency()` to 
   download, build, and install the dependency. Pass any necessary CMake flags,
   and attempt to reduce the time required to build the dependency by disabling
   examples, etc.
6. If the dependency is optional, add a CMake option to `CMakeLists.txt` to
   control whether the dependency is used (e.g., `OPENSIM_C3D_PARSER`).
7. Add to `CMakeLists.txt` a call to `find_package()`. Provide a hint (`HINTS`)
   that will help `find_package()` find the dependency if it was built with the
   superbuild (`dependencies/CMakeLists.txt`).
8. Copy shared libraries into the installed OpenSim Python package,
   if `BUILD_PYTHON_WRAPPING` and `OPENSIM_PYTHON_STANDALONE`). If these
   options are true, then we guarantee for the user that they can use the
   installed OpenSim Python package without relying on any other files. Use
   the CMake macro `OpenSimInstallDependencyLibraries()` to facilitate this.
9. Install the dependency's Windows library (.lib) files in OpenSim's 
   `CMAKE_INSTALL_LIBDIR`. These files are needed to build C++ projects that 
   depend on OpenSim. TODO where does this happen?

Perform the additional steps for a **public** dependency:

1. In `cmake/OpenSimConfig.cmake.in`, add a section for finding the dependency,
   using either `find_package()` or `find_dependency()`, depending on the value
   of `OPENSIM_COPY_DEPENDENCIES`. See how `cmake/OpenSimConfig.cmake.in` 
   handles Simbody for an example.


### Miscellaneous notes

- All dependencies should be built with the same CMAKE_BUILD_TYPE (e.g.,
Release, RelWithDebInfo, Debug) as OpenSim. This must be enforced by users
building OpenSim and its dependencies, not through CMakeLists.txt.
- Dependencies should be optional, if possible. Dependencies make OpenSim
much harder to build for those new to the project, and we can make the lives of
newcomers easier by reducing the number of required dependencies.
