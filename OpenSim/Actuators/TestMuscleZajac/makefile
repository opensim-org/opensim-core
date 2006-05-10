#Makefile for compiling a program for executing a forward integration of the
# UT Optimal Walking solution.

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
LIB1	=	-lrdSimulation -lrdActuators
LIB2	=	-lrdTools -lrdSecurity -lxerces-c
LIB3	=	-lm
LIB	=	-v -L$(LibDev) -L$(LibRel) $(LIB1) $(LIB2) $(LIB3)


#=============================
# COMPILERS
#=============================
#PURIFY = purify -chain-length="20"
PURIFY = 
cc	=	$(PURIFY) cc
CC	=	$(PURIFY) CC
FF	=	$(PURIFY) f77
AR	=	ar

#=============================
# FLAGS
#=============================
CCFLG		=	-g -LANG:std -woff 3303 -DUNIX
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJ100	=	testMuscleZajac.o musmodel.o

#=============================
# TARGETS
#=============================
E100	=	./test.exe

#=============================
# RULES
#=============================
all : $(E100)

clean :
	rm *.o $(E100)


$(E100) : $(OBJ100)
	$(CC) $(CCFLG) -o $(E100) $(OBJ100) $(LIB) -lftn

testMuscleZajac.o : testMuscleZajac.cpp
	$(CC) $(CCFLG) $(INC) -c -o testMuscleZajac.o testMuscleZajac.cpp

#w23moddat.o : w23moddat.f
#	$(FF) $(FFFLG) -c -o w23moddat.o w23moddat.f

musmodel.o : musmodel.f
	$(FF) $(FFFLG) -c -o musmodel.o musmodel.f
