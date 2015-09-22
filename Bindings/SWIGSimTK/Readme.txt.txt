This folder contains headers from Simbody installation annotated with #ifdef SWIG
as needed to make it possible for SWIG to process them and generate wrappers of 
the corresponding classes. The files in this directory do not need to be distributed
or updated to the very latest version of these headers on the Simbody side of the code 
base as long as the methods exposed do not change.

If SWIG ever evolves to handle nested classes which are heavily used in these headers,
these files could be removed and the files from the Simbody codebase can be used directly.
