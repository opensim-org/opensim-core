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
LIB1	=	-lrdBlock -lsuAnalyses -lrdActuators -lrdSimulation
LIB2	=	-lrdTools -lrdSQP -lrdSecurity -lxerces-c1_7_0 -lm
LIB	=	-L$(LibDev) -L$(LibRel) $(LIB1) $(LIB2)

#=============================
# COMPILERS
#=============================
PURIFY = purify
cc	=	$(PURIFY) cc
CC =	$(PURIFY) CC
FF	=	f77
AR	=	ar

#=============================
# FLAGS
#=============================
CCFLG		=	-g -DUNIX -DRDTOOLS_EXPORTS -woff 3303
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJ1	=	fallingBlock.o

OBJS	=	$(OBJ1)


#=============================
# TARGETS
#=============================
E1	=	$(Bin)/fallingBlock.exe

#=============================
# RULES
#=============================
all : $(E1)

clean :
	rm *.o $(E1)


.SUFFIXES : .c .cpp


$(E1) : $(OBJS)
	$(CC) $(CCFLG) -o $(E1) $(OBJS) $(LIB)


.c.o:
	$(cc) $(CCFLG) $(INC) -c $<

.f.o:
	$(FF) $(FFFLG) $(INC) -c $<

.cpp.o:
	$(CC) $(CCFLG) $(INC) -c $< 

