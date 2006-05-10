# Makefile for compiling rdSimulation.

#=============================
# DIRECTORIES
#=============================
Mdl = Model/
Ctr = Control/
Int = Integrator/
Mgr = Manager/

#=============================
# INCLUDE SEARCH PATHS
#=============================
INC	= -I $(IncDev) -I $(IncRel)

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
CCFLG		=	-g -LANG:std -DUNIX -DRDSIMULATION_EXPORTS -woff 3303
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJ1	=	$(Mdl)rdModel.o $(Mdl)rdMuscle.o $(Mdl)rdSprings.o $(Mdl)rdBody.o
OBJ2	=	$(Mdl)rdAnalysis.o $(Mdl)rdAnalysisSet.o
OBJ3	=	$(Mdl)rdActuatorSet.o $(Mdl)rdActuator.o
OBJ4	=	$(Mdl)rdContactForceSet.o $(Mdl)rdContactForce.o
OBJ6	=	$(Mdl)rdCallback.o $(Mdl)rdIntegCallback.o $(Mdl)rdDerivCallback.o
OBJ7	=	$(Mdl)rdCallbackSet.o $(Mdl)rdIntegCallbackSet.o \
			$(Mdl)rdDerivCallbackSet.o
OBJ8	=	$(Mdl)rdGeneralizedForce.o $(Mdl)rdForce.o
OBJ9	=	$(Ctr)rdControlSet.o $(Ctr)rdControl.o $(Ctr)rdController.o
OBJ10	=	$(Ctr)rdControlConstant.o $(Ctr)rdControlLinear.o \
			$(Ctr)rdControlLinearNode.o
OBJ11	=	$(Int)rdIntegRKF.o $(Int)rdRKF.o
OBJ12	=	$(Mgr)rdManager.o

OBJS	=	$(OBJ1) $(OBJ2) $(OBJ3) $(OBJ4) $(OBJ6) $(OBJ7) $(OBJ8) \
			$(OBJ9) $(OBJ10) $(OBJ11) $(OBJ12) 


#=============================
# TARGETS
#=============================
S1	=	$(LibDev)/librdSimulation.so

#=============================
# RULES
#=============================
all : $(S1) rdSDFast rdTestSuite

clean :
	rm so_locations $(S1) $(Mdl)*.o $(Ctr)*.o $(Int)*.o $(Mgr)*.o
	cd SDFast; make clean; cd ..
	cd TestSuite; make clean; cd ..

rdSDFast :
	cd SDFast; make; cd ..

rdTestSuite :
	cd TestSuite; make; cd ..


.SUFFIXES : .cpp


$(S1) : $(OBJS)
	$(CC) $(CCFLG) -shared -o $(S1) $(OBJS) $(LIB)

.cpp.o:
	$(CC) $(CCFLG) $(INC) -c -o $*.o $< 

.c.o:
	$(CC) $(CCFLG) $(INC) -c -o $*.o $<

.f.o:
	$(FF) $(FFFLG) $(INC) -c -o $*.o $<

