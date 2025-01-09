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

    OPENSIM_C3D_PARSER=ezc3d                    non-default
    OPENSIM_COPY_DEPENDENCIES=ON                default
    OPENSIM_PYTHON_STANDALONE=OFF               default
    OPENSIM_INSTALL_UNIX_FHS=OFF                default on Windows, not on UNIX
    OPENSIM_SIMBODY_DOXYGEN_LOCATION=https://simtk.org/api_docs/simbody/<version>/

The last variable causes OpenSim's doxygen documentation to link to Simbody's
documentation online.

The layout of the distribution on Windows is as follows:

  - `bin/` OpenSim and SimTK DLLs, opensim-cmd.exe, simbody-visualizer.exe
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

The layout of the distribution on macOS (and Linux), 
if `OPENSIM_INSTALL_UNIX_FHS` is `ON`, as follows:

  - `bin/` opensim-cmd
  - `etc/OpenSim_buildinfo.txt` Describes the compiler used to build OpenSim.
  - `include/`
    - `OpenSim/` OpenSim (and Lepton) headers.
    - `simbody/` Simbody headers.
  - `lib/` (on some Linux variants, `lib/<arch>/`) OpenSim and SimTK shared libraries.
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

    OPENSIM_C3D_PARSER=ezc3d                    non-default
    OPENSIM_COPY_DEPENDENCIES=OFF               non-default
    OPENSIM_PYTHON_STANDALONE=OFF               default
    OPENSIM_INSTALL_UNIX_FHS=ON                 default on UNIX, not on Windows


Adding dependencies
-------------------
OpenSim depends on multiple C/C++ software libraries. These libraries can be
copied into this repository (Catch2) or built separately. This section mostly 
pertains to dependencies built separately. The number of dependencies should be
reduced to avoid unnecessarily complicating the process of building OpenSim.
Whether a dependency should be copied into the repository or built separately
is a complex decision, and depends on the size of the dependency and the 
difficulty of building the dependency separately.

Developers adding new dependencies to OpenSim should ensure the following
scenarios are supported on Windows, macOS, and Linux:

- Building OpenSim from source
- Building C++ plug-ins or extensions using OpenSim's binary software
  development kit (SDK).
- Using OpenSim through Matlab
- Using OpenSim's Python package (this requires special consideration because
  the Python package is often installed outside of OpenSim's installation).

Some dependencies may be used as static libraries or dynamic/shared
libraries; typically, we use dynamic libraries, but using static libraries is
permitted. Most of the following considerations are for dynamic libraries only;
we use the terms "dynamic" (Windows) and "shared" (UNIX) somewhat 
interchangeably.

There are two types of dependencies: public, or those exposed through OpenSim's 
API (e.g., Simbody), and private, or those used internally by OpenSim 
(e.g., ezc3d):

- **private**: OpenSim's binary distribution need only contain the 
  dependency's dynamic libraries.
- **public**: OpenSim's binary distribution must contain the dependency's
  dynamic libraries, CMake Config files, and header files, and Windows library
  (.lib files).

### RPATH on UNIX systems

UNIX systems (macOS and Linux) provide mechanisms for shared libraries and
executables to contain file system paths to help locate the shared libraries
they need. This information is called an RPATH. By properly embedding RPATHs
in OpenSim's shared libraries and executables, we avoid requiring the user from
setting the DYLD_LIBRARY_PATH environment variable.
See https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling 
for details.

OpenSim provides the following CMake macros to set RPATHs:
- `OpenSimAddInstallRPATH()`
- `OpenSimAddInstallRPATHSelf()`
- `OpenSimAddInstallRPATHSimbody()`

See `cmake/OpenSimMacros.cmake` for details.

Windows does not support RPATHs; it is for this reason that Windows users 
of OpenSim must set their Windows PATH environment variable to include OpenSim's
`bin` directory.

### Steps for adding a dependency to this repository

1. Ensure the dependency can be built and installed using CMake, and that the
   dependency uses modern CMake practices to export its targets in its Package
   Configuration file; see
   https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html.
   Note: it's preferable that the dependency provides a CMake *Config* file 
   instead of a *Module* file.
2. Ensure the dependency provides an open-source license that is consistent with
   OpenSim's Apache License 2.0. If the license is not as permissive as 
   Apache License 2.0, the dependency must be optional and disabled by default
   (see https://choosealicense.com).
3. Ensure the layout of the dependency's installation conforms to OpenSim's
   layout as described in the section "CMake options for packaging a binary
   distribution" above.
4. This step is for Linux and macOS. Ensure the dependency specifies an RPATH 
   for its shared libraries (see description of RPATH above).
5. In `README.md`, add a description of the dependency to each 
   "Get the dependencies" section.
6. Add to `dependencies/CMakeLists.txt` a new call to `AddDependency()` to 
   download, build, and install the dependency. Pass any necessary CMake flags,
   and attempt to reduce the time required to build the dependency by disabling
   examples, etc.
7. If the dependency is optional, add a CMake option to `CMakeLists.txt` to
   control whether the dependency is used (e.g., `OPENSIM_C3D_PARSER`).
8. Add to `CMakeLists.txt` a call to `find_package()`. Provide a hint (`HINTS`)
   that will help `find_package()` find the dependency if it was built with the
   superbuild (`dependencies/CMakeLists.txt`).
9. If linking to the dependency using dynamic (shared) libraries,
   copy the dynamic libraries.
   1. Windows: Copy the dependency's dynamic libraries (DLLs) 
      into OpenSim's build directory; this must be done so tests/examples that are
      run while building OpenSim can find the necessary DLLs.
      If `OPENSIM_COPY_DEPENDENCIES` is `ON`, ensure the DLLs 
      will be installed in OpenSim's installation. Use the CMake macro
      `OpenSimCopyDependencyDLLsForWin()` to achieve copying into the build 
      directory and the installation.
   2. UNIX: The shared libraries need not be copied into the build directory,
      but must be copied into OpenSim's installation if 
      `OPENSIM_COPY_DEPENDENCIES` is `ON`. 
10. Copy shared libraries into the installed OpenSim Python package,
   if `BUILD_PYTHON_WRAPPING` and `OPENSIM_PYTHON_STANDALONE` are `ON`. If these
   options are on, then we want the user to be able to use the
   installed OpenSim Python package without relying on any other OpenSim files. 
   Use the CMake macro `OpenSimInstallDependencyLibraries()` to facilitate this.
11. Add tests to ensure the functionality of the dependency can be accessed
   through the Java, Matlab, and Python interfaces, if such functionality should
   be exposed.
   
Note: the `OPENSIM_PYTHON_STANDALONE=ON` setting is experimental and is not
tested regularly.

Perform the additional steps for a **public** dependency:

1. In `cmake/OpenSimConfig.cmake.in`, add a section for finding the dependency,
   using either `find_package()` or `find_dependency()`, depending on the value
   of `OPENSIM_COPY_DEPENDENCIES`. See how `cmake/OpenSimConfig.cmake.in` 
   handles Simbody for an example.
2. If `OPENSIM_COPY_DEPENDENCIES` is `ON`, ensure the dependency's headers 
   and CMake Configuration files are copied into OpenSim's installation. On
   Windows, ensure library (.lib) files are installed. Ideally, the dependency's
   installation layout matches OpenSim's, so you can just copy the whole project
   into OpenSim's installation.


### Miscellaneous notes

- All dependencies should be built with the same CMAKE_BUILD_TYPE (e.g.,
Release, RelWithDebInfo, Debug) as OpenSim. This must be enforced by users
building OpenSim and its dependencies, not through CMakeLists.txt.
- Dependencies should be optional, if possible. Dependencies make OpenSim
much harder to build for those new to the project, and we can make the lives of
newcomers easier by reducing the number of required dependencies.
- For an example of adding a dependency to OpenSim, refer to the pull request
that introduced ezc3d: https://github.com/opensim-org/opensim-core/pull/2728/files
