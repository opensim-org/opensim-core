# Makefile for compiling suAnalyses.

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
LIB	= -L$(LibDev) -L$(LibRel) -lrdSimulation -lrdTools -lrdSecurity

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
#CCFLG		=	-O2 -LANG:std -woff 3303 -DUNIX -DSUANALYSES_EXPORTS
#FFFLG		=	-O2
CCFLG		=	-g -LANG:std -woff 3303 -DUNIX -DSUANALYSES_EXPORTS
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS	
#=============================
OBJ1	=	suAnalyses.o suKinematics.o suIndAcc.o suGeneralizedForces.o suActuatorGeneralizedForces.o
OBJ2	=	suBodyKinematics.o suBodyIndAcc.o suBodyPointIndAcc.o suBodyIndAccCOM.o
OBJ3	=  suBodyIndPowers.o suPointKinematics.o suActuation.o suContact.o
#OBJ4	=	suDecomp.o suDecompInteg.o suDecompTarget.o suDecompHardUTWalk.o
#OBJ4	=	suDecomp.o suDecompTarget.o suDecompHardUTWalk.o
OBJ4	=	suDecomp.o suDecompTarget.o
OBJ5	=	suActuatorPerturbation.o suActuatorPerturbationIndependent.o
OBJ6	=	suGeneralizedForcePerturbation.o suForceApplier.o
OBJ7	=  suIndContactPowers.o suDecompTaylor.o
OBJS	=	$(OBJ1) $(OBJ2) $(OBJ3) $(OBJ4) $(OBJ5) $(OBJ6) $(OBJ7)

#=============================
# TARGETS
#=============================
S1	=	$(LibDev)/libsuAnalyses.so

#=============================
# RULES
#=============================
all : $(S1)

clean :
	rm *.o $(S1)


$(S1) : $(OBJS)
	$(CC) $(CCFLG) -shared -o $(S1) $(OBJS) $(LIB)


suAnalyses.o : suAnalyses.cpp suAnalyses.h
	$(CC) $(CCFLG) $(INC) -c -o suAnalyses.o suAnalyses.cpp

suKinematics.o : suKinematics.cpp suKinematics.h
	$(CC) $(CCFLG) $(INC) -c -o suKinematics.o suKinematics.cpp

suActuation.o : suActuation.cpp suActuation.h
	$(CC) $(CCFLG) $(INC) -c -o suActuation.o suActuation.cpp

suBodyKinematics.o : suBodyKinematics.cpp suBodyKinematics.h
	$(CC) $(CCFLG) $(INC) -c -o suBodyKinematics.o suBodyKinematics.cpp

suPointKinematics.o : suPointKinematics.cpp suPointKinematics.h
	$(CC) $(CCFLG) $(INC) -c -o suPointKinematics.o suPointKinematics.cpp

suContact.o : suContact.cpp suContact.h
	$(CC) $(CCFLG) $(INC) -c -o suContact.o suContact.cpp

suIndAcc.o : suIndAcc.cpp suIndAcc.h
	$(CC) $(CCFLG) $(INC) -c -o suIndAcc.o suIndAcc.cpp

suIndContactPowers.o : suIndContactPowers.cpp suIndContactPowers.h
	$(CC) $(CCFLG) $(INC) -c -o suIndContactPowers.o suIndContactPowers.cpp

suBodyIndAcc.o : suBodyIndAcc.cpp suBodyIndAcc.h
	$(CC) $(CCFLG) $(INC) -c -o suBodyIndAcc.o suBodyIndAcc.cpp

suBodyPointIndAcc.o : suBodyPointIndAcc.cpp suBodyPointIndAcc.h
	$(CC) $(CCFLG) $(INC) -c -o suBodyPointIndAcc.o suBodyPointIndAcc.cpp

suBodyIndPowers.o : suBodyIndPowers.cpp suBodyIndPowers.h
	$(CC) $(CCFLG) $(INC) -c -o suBodyIndPowers.o suBodyIndPowers.cpp

suBodyIndAccCOM.o : suBodyIndAccCOM.cpp suBodyIndAccCOM.h
	$(CC) $(CCFLG) $(INC) -c -o suBodyIndAccCOM.o suBodyIndAccCOM.cpp

suDecomp.o : suDecomp.cpp suDecomp.h
	$(CC) $(CCFLG) $(INC) -c -o suDecomp.o suDecomp.cpp
	
suDecompTarget.o : suDecompTarget.cpp suDecompTarget.h
	$(CC) $(CCFLG) $(INC) -c -o suDecompTarget.o suDecompTarget.cpp

suDecompHardUTWalk.o : suDecompHardUTWalk.cpp suDecompHardUTWalk.h
	$(CC) $(CCFLG) $(INC) -c -o suDecompHardUTWalk.o suDecompHardUTWalk.cpp

#suDecompInteg.o : suDecompInteg.cpp suDecompInteg.h
#	$(CC) $(CCFLG) $(INC) -c -o suDecompInteg.o suDecompInteg.cpp

suDecompTaylor.o : suDecompTaylor.cpp suDecompTaylor.h
	$(CC) $(CCFLG) $(INC) -c -o suDecompTaylor.o suDecompTaylor.cpp

suActuatorPerturbation.o : suActuatorPerturbation.cpp suActuatorPerturbation.h
	$(CC) $(CCFLG) $(INC) -c -o suActuatorPerturbation.o \
	suActuatorPerturbation.cpp

suActuatorPerturbationIndependent.o : suActuatorPerturbationIndependent.cpp suActuatorPerturbationIndependent.h
	$(CC) $(CCFLG) $(INC) -c -o suActuatorPerturbationIndependent.o \
	suActuatorPerturbationIndependent.cpp

suGeneralizedForces.o : suGeneralizedForces.cpp suGeneralizedForces.h
	$(CC) $(CCFLG) $(INC) -c -o suGeneralizedForces.o suGeneralizedForces.cpp

suActuatorGeneralizedForces.o : suActuatorGeneralizedForces.cpp suActuatorGeneralizedForces.h
	$(CC) $(CCFLG) $(INC) -c -o suActuatorGeneralizedForces.o suActuatorGeneralizedForces.cpp

suGeneralizedForcePerturbation.o : suGeneralizedForcePerturbation.cpp suGeneralizedForcePerturbation.h
	$(CC) $(CCFLG) $(INC) -c -o suGeneralizedForcePerturbation.o suGeneralizedForcePerturbation.cpp

suForceApplier.o : suForceApplier.cpp suForceApplier.h
	$(CC) $(CCFLG) $(INC) -c -o suForceApplier.o suForceApplier.cpp
