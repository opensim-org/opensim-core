# Makefile for compiling the math routines.

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
CCFLG		=	-g -LANG:std -woff 3303 -DUNIX -DRDMODEL_EXPORTS
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJS1	=	rdModel.o rdMuscle.o rdSprings.o rdBody.o rdAnalysis.o
OBJS2	=	rdActuatorSet.o rdActuator.o
OBJS3	=	rdContactForceSet.o rdContactForce.o
OBJS4	=	
OBJS5	=	rdCallback.o rdIntegCallback.o rdDerivCallback.o
OBJS6	=	rdCallbackSet.o rdIntegCallbackSet.o
OBJS7	=	rdGeneralizedForce.o rdForce.o
OBJS	=	$(OBJS1) $(OBJS2) $(OBJS3) $(OBJS4) $(OBJS5) $(OBJS6) $(OBJS7)

#=============================
# TARGETS
#=============================
S1	=	$(LibDev)/librdModel.so

#=============================
# RULES
#=============================
all : $(S1)

clean :
	rm *.o so_locations $(S1)


$(S1) : $(OBJS)
	$(CC) $(CCFLG) -shared -o $(S1) $(OBJS)


rdModel.o : rdModel.cpp rdModel.h rdBody.h
	$(CC) $(CCFLG) $(INC) -c -o rdModel.o rdModel.cpp

rdMuscle.o : rdMuscle.cpp rdMuscle.h rdModel.h
	$(CC) $(CCFLG) $(INC) -c -o rdMuscle.o rdMuscle.cpp

rdSprings.o : rdSprings.cpp rdSprings.h
	$(CC) $(CCFLG) $(INC) -c -o rdSprings.o rdSprings.cpp

rdBody.o : rdBody.cpp rdBody.h
	$(CC) $(CCFLG) $(INC) -c -o rdBody.o rdBody.cpp

rdAnalysis.o : rdAnalysis.cpp rdAnalysis.h
	$(CC) $(CCFLG) $(INC) -c -o rdAnalysis.o rdAnalysis.cpp

rdController.o : rdController.cpp rdController.h
	$(CC) $(CCFLG) $(INC) -c -o rdController.o rdController.cpp

rdActuatorSet.o : rdActuatorSet.cpp rdActuatorSet.h
	$(CC) $(CCFLG) $(INC) -c -o rdActuatorSet.o rdActuatorSet.cpp

rdActuator.o : rdActuator.cpp rdActuator.h
	$(CC) $(CCFLG) $(INC) -c -o rdActuator.o rdActuator.cpp

rdGeneralizedForceAtv.o : rdGeneralizedForceAtv.cpp rdGeneralizedForceAtv.h
	$(CC) $(CCFLG) $(INC) -c -o rdGeneralizedForceAtv.o rdGeneralizedForceAtv.cpp

rdGeneralizedForce.o : rdGeneralizedForce.cpp rdGeneralizedForce.h
	$(CC) $(CCFLG) $(INC) -c -o rdGeneralizedForce.o rdGeneralizedForce.cpp

rdForce.o : rdForce.cpp rdForce.h
	$(CC) $(CCFLG) $(INC) -c -o rdForce.o rdForce.cpp

rdContactForceSet.o : rdContactForceSet.cpp rdContactForceSet.h
	$(CC) $(CCFLG) $(INC) -c -o rdContactForceSet.o rdContactForceSet.cpp

rdContactForce.o : rdContactForce.cpp rdContactForce.h
	$(CC) $(CCFLG) $(INC) -c -o rdContactForce.o rdContactForce.cpp

rdSetPoint.o : rdSetPoint.cpp rdSetPoint.h
	$(CC) $(CCFLG) $(INC) -c -o rdSetPoint.o rdSetPoint.cpp

rdLinearSetPoint.o : rdLinearSetPoint.cpp rdLinearSetPoint.h
	$(CC) $(CCFLG) $(INC) -c -o rdLinearSetPoint.o rdLinearSetPoint.cpp

rdPolynomialSetPoint.o : rdPolynomialSetPoint.cpp rdPolynomialSetPoint.h
	$(CC) $(CCFLG) $(INC) -c -o rdPolynomialSetPoint.o rdPolynomialSetPoint.cpp

rdCallback.o : rdCallback.cpp rdCallback.h
	$(CC) $(CCFLG) $(INC) -c -o rdCallback.o rdCallback.cpp

rdIntegCallback.o : rdIntegCallback.cpp rdIntegCallback.h
	$(CC) $(CCFLG) $(INC) -c -o rdIntegCallback.o rdIntegCallback.cpp

rdDerivCallback.o : rdDerivCallback.cpp rdDerivCallback.h
	$(CC) $(CCFLG) $(INC) -c -o rdDerivCallback.o rdDerivCallback.cpp

rdCallbackSet.o : rdCallbackSet.cpp rdCallbackSet.h
	$(CC) $(CCFLG) $(INC) -c -o rdCallbackSet.o rdCallbackSet.cpp

rdIntegCallbackSet.o : rdIntegCallbackSet.cpp rdIntegCallbackSet.h
	$(CC) $(CCFLG) $(INC) -c -o rdIntegCallbackSet.o rdIntegCallbackSet.cpp


