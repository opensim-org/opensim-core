# Makefile for compiling library rdSDFast

#=============================
# DIRECTORIES
#=============================
D_Model = ../Model

#=============================
# INCLUDE SEARCH PATHS
#=============================
INC	= -I$(IncDev) -I$(IncRel)

#=============================
# LIBRARIES
#=============================
LIB1	=	-lrdSimulation -lrdTools
LIB2	=	-lftn
LIB	=	-L$(LibDev) -L$(LibRel) $(LIB1) $(LIB2)


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
CCFLG		=	-g -LANG:std -woff 3303 -DUNIX -DRDSDFAST_EXPORTS
FFFLG		=	-g -DUNIX
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJS10	=	rdActuatedModel_SDFast.o rdActuatedModelIntegCallback.o \
				rdSDFast.o
OBJS20	=	sdlib.o sdtransform.o sdfast_f2c.o
OBJS100	=	$(OBJS10) $(OBJS20)

#=============================
# TARGETS
#=============================
S100	=	$(LibDev)/librdSDFast.so

#=============================
# RULES
#=============================
all : $(S100)

clean :
	rm *.o so_locations $(S100)


$(S100) : $(OBJS100)
	$(CC) $(CCFLG) -shared -o $(S100) $(OBJS100) $(LIB)


rdActuatedModel_SDFast.o : rdActuatedModel_SDFast.cpp rdActuatedModel_SDFast.h
	$(CC) $(CCFLG) $(INC) -c -o rdActuatedModel_SDFast.o \
	rdActuatedModel_SDFast.cpp

rdActuatedModelIntegCallback.o : rdActuatedModelIntegCallback.cpp rdActuatedModelIntegCallback.h
	$(CC) $(CCFLG) $(INC) -c -o rdActuatedModelIntegCallback.o \
	rdActuatedModelIntegCallback.cpp

rdSDFast.o : rdSDFast.cpp rdSDFast.h
	$(CC) $(CCFLG) $(INC) -c -o rdSDFast.o rdSDFast.cpp

sdlib.o : sdlib.c
	$(cc) $(CCFLG) $(INC) -c -o sdlib.o sdlib.c

sdtransform.o : sdtransform.f
	$(FF) $(FFFLG) $(INC) -c -o sdtransform.o sdtransform.f

sdfast_f2c.o : sdfast_f2c.c
	$(cc) $(CCFLG) $(INC) -c -o sdfast_f2c.o sdfast_f2c.c

