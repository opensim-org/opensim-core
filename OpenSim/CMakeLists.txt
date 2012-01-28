#-----------------------------------------------------------------------------

SUBDIRS(Common Simulation Actuators Analyses Models Utilities Tools Java Examples Tests doc)

INSTALL_FILES(/sdk/include/OpenSim .h OpenSim.h)

INSTALL(DIRECTORY Plugin DESTINATION  sdk/APIExamples  PATTERN ".svn" EXCLUDE)
