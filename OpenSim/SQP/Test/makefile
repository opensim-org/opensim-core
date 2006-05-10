# Makefile for compiling the math routines.

#=============================
# DIRECTORIES
#=============================


#=============================
# INCLUDE SEARCH PATHS
#=============================
INC	= -I$(LibDev) -I$(LibRel)

#=============================
# LIBRARIES
#=============================
LIB	= -L$(LibDev) -I$(LibRel) -lrdSQP -lrdMath -lcfsqp -lm -lftn

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
CCFLG		=	-g -DUNIX -DRDSQP_EXPORTS
FFFLG		=	-g
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJS3 =	quadTarget.o
OBJS4 =	testSQP.o $(OBJS3)
OBJS5 =	testvf.o
OBJS6 =	testFSQP1.o $(OBJS3)
OBJS7 =	rosenTarget.o rosen.o


#=============================
# TARGETS
#=============================
E4	=	testSQP
E5 = 	testvf
E6 =	testFSQP
E7 =	rosen

#=============================
# RULES
#=============================
all : $(E6)

clean :
	rm *.o $(E4) $(E5) $(E6) $(E7)


$(E4) : $(OBJS4)
	$(CC) $(CCFLG) -o $(E4) $(OBJS4) $(LIB)

$(E5) : $(OBJS2) $(OBJS5)
	$(FF) $(FFFLG) -o $(E5) $(OBJS2) $(OBJS5)

$(E6) : $(OBJS6)
	$(CC) $(CCFLG) -o $(E6) $(OBJS6) $(LIB)

$(E7) : $(OBJS7) $(LibDev)/librdSQP.a $(LibDev)/librdMath.a
	$(CC) $(CCFLG) -o $(E7) $(OBJS7) $(LIB)


rosen.o : rosen.cpp
	$(CC) $(CCFLG) $(INC) -c -o rosen.o rosen.cpp

testFSQP1.o : testFSQP1.cpp quadTarget.h
	$(CC) $(CCFLG) $(INC) -c -o testFSQP1.o testFSQP1.cpp

testFSQP.o : testFSQP.cpp quadTarget.h
	$(CC) $(CCFLG) $(INC) -c -o testFSQP.o testFSQP.cpp

testvf.o : testvf.f
	$(FF) $(FFFLG) $(INC) -c -o testvf.o testvf.f

testSQP.o : testSQP.c quadTarget.h
	$(CC) $(CCFLG) $(INC) -c -o testSQP.o testSQP.c

rosenTarget.o : rosenTarget.c rosenTarget.h
	$(CC) $(CCFLG) $(INC) -c -o rosenTarget.o rosenTarget.c

quadTarget.o : quadTarget.c quadTarget.h
	$(CC) $(CCFLG) $(INC) -c -o quadTarget.o quadTarget.c
