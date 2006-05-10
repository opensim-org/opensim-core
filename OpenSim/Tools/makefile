# Makefile for compiling rdTools.

#=============================
# DIRECTORIES
#=============================

#=============================
# INCLUDE SEARCH PATHS
#=============================
INC	= -I $(IncDev) -I $(IncRel)


#=============================
# LIBRARIES
#=============================
LIB	= -L$(LibDev) -L$(LibRel)

#=============================
# COMPILERS
#=============================
PURIFY =
cc	=	$(PURIFY) cc
CC =	$(PURIFY) CC
FF	=	f77
AR	=	ar

#=============================
# FLAGS
#=============================
CCFLG		=	-g -DUNIX -LANG:std -DRDTOOLS_EXPORTS -woff 3303
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJ1	=	rdIO.o rdMath.o rdMemory.o rdMtx.o rdObject.o rdVisibleObject.o
OBJ2	=	rdProperty.o \
			rdPropertyBool.o rdPropertyBoolArray.o \
			rdPropertyDbl.o rdPropertyDblArray.o \
			rdPropertyInt.o rdPropertyIntArray.o \
			rdPropertyObj.o rdPropertyObjArray.o \
			rdPropertyStr.o rdPropertyStrArray.o
OBJ3	=	rdBodyConstraint.o rdPointConstraint.o rdSignal.o rdSpline.o
OBJ4	=	rdStateVector.o rdStorage.o rdException.o
OBJ5	=	rdXMLNode.o rdXMLDocument.o
OBJ6	=	rdLine.o rdPlane.o	
OBJ7	=	
OBJ8	=	rdPropertySet.o
OBJ9	=	rdFunction.o rdFunctionSet.o rdGCVSpline.o rdGCVSplineSet.o
OBJ10	=	rdVectorFunction.o gcvspl.o rd1to3VectorGCVSpline.o
OBJ11	= 	rdTransform.o rdVisibleProperties.o
OBJ12	=	rdMaterial.o rdMaterialManager.o rdMaterialSet.o

OBJS	=	$(OBJ1) $(OBJ2) $(OBJ3) $(OBJ4) $(OBJ5) $(OBJ6) $(OBJ7) $(OBJ8) \
			$(OBJ9) $(OBJ10) $(OBJ11) $(OBJ12)
#OBJS	=	$(OBJ1) $(OBJ2) $(OBJ3) $(OBJ4) $(OBJ5) $(OBJ6) $(OBJ7) $(OBJ8) \
#			$(OBJ9) $(OBJ10) $(OBJ11) $(OBJ12)


#=============================
# TARGETS
#=============================
S1	=	$(LibDev)/librdTools.so

#=============================
# RULES
#=============================
all : $(S1)

clean :
	rm *.o $(S1)


.SUFFIXES : .cpp


$(S1) : $(OBJS)
	$(CC) $(CCFLG) -shared -o $(S1) $(OBJS)


.cpp.o:
	$(CC) $(CCFLG) $(INC) -c $< 

.c.o:
	$(CC) $(CCFLG) $(INC) -c $<

.f.o:
	$(FF) $(FFFLG) $(INC) -c $<

