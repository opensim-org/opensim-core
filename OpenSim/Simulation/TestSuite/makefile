# Makefile for compiling rdModelTestSuite

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
LIB1	=	-L$(LibDev) -L$(LibRel) -lrdSimulation -lrdTools


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
CCFLG		=	-g -LANG:std -DUNIX -DRDFAST_EXPORTS
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJS10	=	rdModelTestSuite.o
OBJS100	=	$(OBJS10)

#=============================
# TARGETS
#=============================
S100	=	$(LibDev)/librdModelTestSuite.so

#=============================
# RULES
#=============================
all : $(S100)

clean :
	rm *.o $(S100)


$(S100) : $(OBJS100)
	$(CC) $(CCFLG) -shared -o $(S100) $(OBJS100) $(LIB1)


rdModelTestSuite.o : rdModelTestSuite.cpp rdModelTestSuite.h
	$(CC) $(CCFLG) $(INC) -c -o rdModelTestSuite.o rdModelTestSuite.cpp

