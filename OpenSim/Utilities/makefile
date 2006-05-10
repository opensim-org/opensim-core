# Makefile for compiling a program for computing segmental powers.

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
LIB1	=	-lrdTools -lxerces-c1_7_0
LIB2	=
LIB3	=	
LIB	=	-L$(LibDev) -L$(LibRel) $(LIB1) $(LIB2) $(LIB3)


#=============================
# COMPILERS
#=============================
PURIFY = 
cc	=	$(PURIFY) cc
CC =	$(PURIFY) CC
FF	=	$(PURIFY) f77
AR	=	ar

#=============================
# FLAGS
#=============================
CCFLG		=	-g -woff 3303 -DUNIX
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJ100	=	storageAdd.o
OBJ110	=	storageSubtract.o
OBJ120	=	storageMultiply.o
OBJ200	=	storageIntegrate.o
OBJ220	=	suMakeGaitCycleM.o

#=============================
# TARGETS
#=============================
E100	=	${Bin}/storageAdd
E110	=	${Bin}/storageSubtract
E120	=	${Bin}/storageMultiply
E200	=	${Bin}/storageIntegrate
E220	=	${Bin}/makeGaitCycleM

#=============================
# RULES
#=============================
all : $(E100) $(E110) $(E120) $(E200) $(E220)

clean :
	rm *.o $(E100) $(E110) $(E120) $(E200) $(E220)


$(E100) : $(OBJ100)
	$(CC) $(CCFLG) -o $(E100) $(OBJ100) $(LIB)

$(E110) : $(OBJ110)
	$(CC) $(CCFLG) -o $(E110) $(OBJ110) $(LIB)

$(E120) : $(OBJ120)
	$(CC) $(CCFLG) -o $(E120) $(OBJ120) $(LIB)

$(E200) : $(OBJ200)
	$(CC) $(CCFLG) -o $(E200) $(OBJ200) $(LIB)

$(E220) : $(OBJ220)
	$(CC) $(CCFLG) -o $(E220) $(OBJ220) $(LIB)

storageAdd.o : storageAdd.cpp
	$(CC) $(CCFLG) $(INC) -c -o storageAdd.o storageAdd.cpp

storageSubtract.o : storageSubtract.cpp
	$(CC) $(CCFLG) $(INC) -c -o storageSubtract.o storageSubtract.cpp

storageMultiply.o : storageMultiply.cpp
	$(CC) $(CCFLG) $(INC) -c -o storageMultiply.o storageMultiply.cpp

storageIntegrate.o : storageIntegrate.cpp
	$(CC) $(CCFLG) $(INC) -c -o storageIntegrate.o storageIntegrate.cpp

suMakeGaitCycleM.o : suMakeGaitCycleM.cpp
	$(CC) $(CCFLG) $(INC) -c -o suMakeGaitCycleM.o suMakeGaitCycleM.cpp

