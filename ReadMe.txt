
                                OPENSIM
 

What is it? 
----------- 
OpenSim is software that lets users develop models of musculoskeletal
structures and create dynamic simulations of movement. 

More information can be found at our websites
OpenSim website <http://opensim.stanford.edu>
Simtk project website <https://simtk.org/home/opensim>


Licensing
---------
This OpenSim API is licensed under the terms you may find in the file 
named NOTICE in this directory.

Models and examples distributed with our source code often have their 
own custom licenses. 
  

Build Instructions
------------------
Building from source is difficult and we have limited resource to support 
it. Instructions can be found online at:
http://simtk-confluence.stanford.edu:8080/display/OpenSim/Building+OpenSim+from+Source


Cross-platform Instructions
---------------------------------------------
The release of OpenSim on Linux and MAC is only in the form of API/Libraries and headers, for GUI use we recommend using a Virtual Machine that runs windows.

To use the command line utilities on these systems you need to 
1. Add the directory bin (under the directory where you untar the distribution) to your path (environment variable PATH)
2. Add the same directory to the dynamic libraries path (this is env. var LD_LIBRARY_PATH on Linux, DYLD_LIBRARY_PATH on Mac).

The libraries are built with gcc 4.6.3 64 bit (Linux Ubuntu 12.04, Mac OSX 10.6) . 

To build the API examples please follow the developer's guide steps, use CMake (which is cross platform Make system).


  
                                          The OpenSim Project
  
