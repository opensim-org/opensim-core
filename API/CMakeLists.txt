SUBDIRS(doc)
#-----------------------------------------------------------------------------
IF(WIN32)
INSTALL(DIRECTORY examples DESTINATION  sdk  PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY templates DESTINATION sdk PATTERN ".svn" EXCLUDE)
ENDIF(WIN32)
# Create a plugins directory so that users know where to place theirs.
INSTALL_FILES(/plugins/ FILES "readme.txt")
