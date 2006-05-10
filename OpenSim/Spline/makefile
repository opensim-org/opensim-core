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
LIB	=	-L$(LibDev) -L$(LibRel) -lsuSpline

#=============================
# COMPILERS
#=============================
CC	=	CC
FF	=	f77
AR	=	ar

#=============================
# FLAGS
#=============================
CCFLG		=	-O2
FFFLG		=	-O2
ARFLG		=	-rsv

#=============================
# OBJECTS
#=============================
OBJS1	=	spline.o
OBJS2	=	mainTest.o

#=============================
# TARGETS
#=============================
T1	=	$(LibDev)/libsuSpline.a
T2	=	mainTest.exe

#=============================
# RULES
#=============================
T : $(T1) $(T2)

clean :
	rm $(OBJS1)
	rm $(T1)
	rm $(OBJS2)
	rm $(T2)

$(T1) : $(OBJS1)
	$(AR) $(ARFLG) $(T1) $(OBJS1)

$(T2) : $(OBJS2)
	$(FF) $(FFFLG) -o $(T2) $(OBJS2) $(LIB)


spline.o : spline.c spline.h
	$(CC) $(CCFLG) $(INC) -c -o spline.o spline.c

mainTest.o : mainTest.f
	$(FF) $(FFFLG) $(INC) -c -o mainTest.o mainTest.f
