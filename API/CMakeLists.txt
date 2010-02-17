SUBDIRS(doc)
#-----------------------------------------------------------------------------
IF(INSTALL_DOC)
INSTALL(DIRECTORY examples DESTINATION  sdk  PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY templates DESTINATION sdk PATTERN ".svn" EXCLUDE)
ENDIF(INSTALL_DOC)
# Create a plugins directory so that users know where to place theirs.
INSTALL_FILES(/sdk/ FILES "readme.txt")
