
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
LIB	= -L$(LibDev) -L$(LibRel) -lrdSimulation -lrdTools

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
CCFLG		=	-g -LANG:std -woff 3303 -DUNIX -DSUACTUATORS_EXPORTS
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS	
#=============================
OBJ1	=	rdActuators.o rdGeneralizedForceAtv.o rdMuscleZajac.o 
OBJ2	=	rdSetPoint.o rdLinearSetPoint.o rdPolynomialSetPoint.o
OBJ	=	$(OBJ1) $(OBJ2)

#=============================
# TARGETS
#=============================
S1	=	$(LibDev)/librdActuators.so

#=============================
# RULES
#=============================
all : $(S1)

clean :
	rm *.o $(S1)


.SUFFIXES : .cpp


$(S1) : $(OBJ)
	$(CC) $(CCFLG) -shared -o $(S1) $(OBJ) $(LIB)

.cpp.o:
	$(CC) $(CCFLG) $(INC) -c -o $*.o $< 

.c.o:
	$(CC) $(CCFLG) $(INC) -c -o $*.o $<

.f.o:
	$(FF) $(FFFLG) $(INC) -c -o $*.o $<


