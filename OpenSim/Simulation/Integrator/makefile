# Makefile for compiling rdIntegrator.

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
CCFLG		=	-g -LANG:std -woff 3303 -DUNIX -DRDINTEGRATOR_EXPORTS
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJS1	=	rdIntegRKF.o rdRKF.o

#=============================
# TARGETS
#=============================
S1	=	$(LibDev)/librdIntegrator.so

#=============================
# RULES
#=============================
all : $(S1)

clean :
	rm *.o $(S1)


$(S1) : $(OBJS1)
	$(CC) $(CCFLG) -shared -o $(S1) $(OBJS1)


rdIntegRKF.o : rdIntegRKF.cpp rdIntegRKF.h rdRKF.h
	$(CC) $(CCFLG) $(INC) -c -o rdIntegRKF.o rdIntegRKF.cpp

rdRKF.o : rdRKF.cpp rdRKF.h
	$(CC) $(CCFLG) $(INC) -c -o rdRKF.o rdRKF.cpp
