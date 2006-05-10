# Makefile for compiling the sequential quadratic programming library.

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
LIB	= -L$(LibDev) -L$(LibRel) -lrdMath -lparamopt -lcfsqp -lm -lftn

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
CCFLG		=	-g -LANG:std -DUNIX -DRDSQP_EXPORTS
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJS1	=	rdFSQP.o rdSQP.o rdOptimizationTarget.o nrODS.o


#=============================
# TARGETS
#=============================
S1	=	$(LibDev)/librdSQP.so

#=============================
# RULES
#=============================
all : $(S1)

clean :
	rm *.o $(S1)


$(S1) : $(OBJS1)
	$(CC) $(CCFLG) -shared -o $(S1) $(OBJS1)


rdSQP.o : rdSQP.cpp rdSQP.h rdOptimizationTarget.h nrODS.h
	$(CC) $(CCFLG) $(INC) -c -o rdSQP.o rdSQP.cpp

rdFSQP.o : rdFSQP.cpp rdFSQP.h rdOptimizationTarget.h
	$(CC) $(CCFLG) $(INC) -c -o rdFSQP.o rdFSQP.cpp

rdOptimizationTarget.o : rdOptimizationTarget.cpp rdOptimizationTarget.h
	$(CC) $(CCFLG) $(INC) -c -o rdOptimizationTarget.o rdOptimizationTarget.cpp

nrODS.o : nrODS.cpp nrODS.h
	$(CC) $(CCFLG) $(INC) -c -o nrODS.o nrODS.cpp
