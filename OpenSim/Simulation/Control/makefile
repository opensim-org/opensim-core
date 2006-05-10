# Makefile for compiling the rdControl library.

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
LIB	= -L$(LibDev) -L$(LibRel) -lrdTools

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
CCFLG		=	-g -LANG:std -DUNIX -DRDCNTR_EXPORTS
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJS1	=	rdControlSet.o rdControl.o rdControlLinear.o

#=============================
# TARGETS
#=============================
S1	=	$(LibDev)/librdControl.so

#=============================
# RULES
#=============================
all : $(S1)

clean :
	rm *.o $(S1)


$(S1) : $(OBJS1)
	$(CC) $(CCFLG) -shared -o $(S1) $(OBJS1) $(LIB)


rdControlSet.o : rdControlSet.cpp rdControlSet.h rdControl.h
	$(CC) $(CCFLG) $(INC) -c -o rdControlSet.o rdControlSet.cpp

rdControl.o : rdControl.cpp rdControl.h
	$(CC) $(CCFLG) $(INC) -c -o rdControl.o rdControl.cpp

rdControlStep.o : rdControlStep.cpp rdControlStep.h
	$(CC) $(CCFLG) $(INC) -c -o rdControlStep.o rdControlStep.cpp

rdControlLinear.o : rdControlLinear.cpp rdControlLinear.h
	$(CC) $(CCFLG) $(INC) -c -o rdControlLinear.o rdControlLinear.cpp
