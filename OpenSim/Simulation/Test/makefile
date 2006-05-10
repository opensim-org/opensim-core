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
LIB	= -L$(LibDev)  -L$(LibRel) -lrdActuators -lrdSimulation \
			-lrdTools -lxerces-c -lm

#=============================
# COMPILERS
#=============================
PURIFY = purify -chain-length="20"
cc	=	$(PURIFY) cc
CC =	$(PURIFY) CC
FF	=	f77
AR	=	ar

#=============================
# FLAGS
#=============================
CCFLG		=	-g -LANG:std -DUNIX -DRDTOOLS_EXPORTS -woff 3303
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJ1	=	simulationTest.o

OBJS	=	$(OBJ1) $(OBJ2) $(OBJ3) $(OBJ4) $(OBJ5) $(OBJ6) $(OBJ7)


#=============================
# TARGETS
#=============================
E1	=	$(Bin)/testSimulation.exe

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

