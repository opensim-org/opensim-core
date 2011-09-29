
                                 OPENSIM
 

  What is it? 
  -----------
  
  OpenSim is an open-source software system that lets users develop models of musculoskeletal
  structures and create dynamic simulations of movement. 

  why?
  -----

  The software provides a platform on which the biomechanics community can build a library   
  of simulations that can be exchanged, tested, analyzed, and improved through
  multi-institutional collaboration. 

  The underlying software is written in ANSI C++, and the graphical user interface (GUI) is 
  written in Java. OpenSim technology makes it possible to develop customized controllers,
  analyses, contact models, and muscle models among other things. These plugins can be shared
  without the need to alter or compile source code. Users can analyze existing models and
  simulations and develop new models and simulations from within the GUI.

  The Latest Version
  ------------------

  Details of the latest version can be found on the simtk.org site.
  Project web site <https://simtk.org/home/opensim>.


  Licensing
  ---------

  This software is licensed under the terms you may find in the file 
  named "Copyright.txt" in this directory.
  
 Cross-platform Instructions
 ---------------------------------------------
The release of OpenSim on Linux and MAC is only in the form of API/Libraries and headers, for GUI use we recommend using a Virtual Machine that runs windows.

To use the command line utilities on these systems you need to 
1. Add the directory bin (under the directory where you untar the distribution) to your path (environment variable PATH)
2. Add the same directory to the dynamic libraries path (this is env. var LD_LIBRARY_PATH on Linux, DYLD_LIBRARYPATH on Mac).

The libraries are built with gcc 4.4.1 32 bit (Linux Ubuntu 10.10, Mac OSX 10.6) . 

To build the API examples please follow the developer's guide steps, use CMake (which is cross platform Make system).


  
                                          The OpenSim Project
  
