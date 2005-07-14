COPTS = -o32
all:
	cc -c $(COPTS) cccp.c -o cccp.o
	cc -c $(COPTS) alloca.c -o alloca.o
	yacc cexp.y
	cc -c $(COPTS) y.tab.c -o y.tab.o
	cc -c $(COPTS) obstack.c -o obstack.o
	cc -c $(COPTS) version.c -o version.o
	ar q libacpp.a *.o

clean:
	rm libacpp.a *.o
