# OpenSim Core

[![continuous-integration](https://github.com/opensim-org/opensim-core/workflows/continuous-integration/badge.svg)](https://github.com/opensim-org/opensim-core/actions) [![GitHub release (latest by date including pre-releases)](https://img.shields.io/github/v/release/opensim-org/opensim-core?include_prereleases)](https://github.com/opensim-org/opensim-core/releases) [![License](https://img.shields.io/hexpm/l/apa)](https://github.com/opensim-org/opensim-core/blob/master/LICENSE.txt) [![platforms](https://img.shields.io/badge/platform-windows%20%7C%20macos%20%7C%20linux-lightgrey)](https://github.com/opensim-org/opensim-core/wiki/Build-Instructions) [![ZenHub][zenhub_image]][zenhub]  

**NOTE: This repository contains the source code of OpenSim 4.x. For OpenSim 3.x or earlier, see [this link](https://simtk-confluence.stanford.edu:8443/display/OpenSim/Building+OpenSim+from+Source).**

OpenSim is software that lets users develop models of musculoskeletal structures and create dynamic simulations of movement, such as this one:

![Simulation of human running by Sam Hamner (doi:10.1016/j.jbiomech.2010.06.025)][running_gif]

More information can be found at our websites:

* [OpenSim website](http://opensim.stanford.edu), in particular the [support page](http://opensim.stanford.edu/support/index.html).
* [SimTK project website](https://simtk.org/home/opensim).

This repository contains:

 - OpenSim's C++ libraries.
 - OpenSim's C++ examples.
 - OpenSim's command-line applications (inverse kinematics, computed muscle control, etc.).
 - OpenSim's Java and Python wrapping.
 
This repository does *not* include source code for the OpenSim GUI. The source code for the Opensim GUI can be found [here](https://github.com/opensim-org/opensim-gui).

## Documentation

OpenSim's documentation can be found in our [Documentation](https://simtk-confluence.stanford.edu:8443/display/OpenSim/Documentation) website. 
OpenSim's C++ API reference can be found [here](https://simtk.org/api_docs/opensim/api_docs/).

## Examples and Tutorials

A simple example of an elbow simulation in C++, Python and Matlab can be found in the [OpenSim API Example](https://github.com/opensim-org/opensim-core/wiki/OpenSim-API-Example) page of this repository's wiki.

Examples and Tutorials for OpenSim can be found in [this](https://simtk-confluence.stanford.edu:8443/display/OpenSim/Examples+and+Tutorials) website. These tutorials move from introductory to more advanced, so you can learn OpenSim in a progressive way. Additional OpenSim-based tutorials, homework problems, and project ideas are available on the [Biomechanics of Movement classroom site](https://simtk-confluence-homeworks.stanford.edu:8443/pages/viewpage.action?pageId=5537857). 

## Releases [![GitHub release (latest by date including pre-releases)](https://img.shields.io/github/v/release/opensim-org/opensim-core?include_prereleases)](https://github.com/opensim-org/opensim-core/releases)

This repository contains releases for OpenSim 4.x. You can find all of the OpenSim's releases in the [Releases](https://github.com/opensim-org/opensim-core/releases) page of this repository.

## Build instructions [![platforms](https://img.shields.io/badge/platform-windows%20%7C%20macos%20%7C%20linux-lightgrey)](https://github.com/opensim-org/opensim-core/wiki/Build-Instructions)

We provide scripts to build OpenSim on Windows, macOS and Linux (Ubuntu and Debian). The instructions to download and execute the scripts can be found in the [Build Instructions](https://github.com/opensim-org/opensim-core/wiki/Build-Instructions) page of this repository's wiki.

## License [![License](https://img.shields.io/hexpm/l/apa)](https://github.com/opensim-org/opensim-core/blob/master/LICENSE.txt)

Licensed under the Apache License, Version 2.0.  See full text of the [Apache License, Version 2.0](https://github.com/opensim-org/opensim-core/blob/master/LICENSE.txt). 



[buildstatus_image_travis]: https://travis-ci.org/opensim-org/opensim-core.svg?branch=master
[travisci]: https://travis-ci.org/opensim-org/opensim-core
[buildstatus_image_appveyor]: https://ci.appveyor.com/api/projects/status/i4wxnmx9jlk69kge/branch/master?svg=true
[appveyorci]: https://ci.appveyor.com/project/opensim-org/opensim-core/branch/master
[zenhub_image]: https://img.shields.io/badge/Shipping%20faster%20with-ZenHub-blueviolet
[zenhub]: https://zenhub.com

[running_gif]: doc/images/opensim_running.gif
[simple_example_gif]: doc/images/opensim_double_pendulum_muscle.gif
[java]: http://www.oracle.com/technetwork/java/javase/downloads/index.html
