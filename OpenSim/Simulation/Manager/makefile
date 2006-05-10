# Makefile for compiling rdManager.

#=============================
# DIRECTORIES
#=============================


#=============================
# INCLUDE SEARCH PATHS
#=============================
INC	= -I$(IncDev) -I$(IncRel)

#=============================
# LIBRARIES
#=============================
LIB	= -L$(LibDev) -L$(LibRel) -lrdCntr -lrdIntegrator -lrdTools -lrdModel

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
CCFLG		=	-g -LANG:std -woff 3303 -DUNIX -DRDMANAGER_EXPORTS
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJS1	=	rdManager.o

#=============================
# TARGETS
#=============================
S1	=	$(LibDev)/librdManager.so

#=============================
# RULES
#=============================
all : $(S1)

clean :
	rm *.o $(S1)


$(S1) : $(OBJS1)
	$(CC) $(CCFLG) -shared -o $(S1) $(OBJS1) $(LIB)


rdManager.o : rdManager.cpp rdManager.h
	$(CC) $(CCFLG) $(INC) -c -o rdManager.o rdManager.cpp
