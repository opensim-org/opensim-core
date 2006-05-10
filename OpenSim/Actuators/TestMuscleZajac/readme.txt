11/19 

I managed to compile the thing and added printing commands in testMuscleZajac
and rdMuscleZajac
It does not work anymore when I try to link in the fortran musmodel.f
I changed the makefile -> no change commented the original lines

I defined mmfdot_ (this should be done through th *.h) but even when definig it
in the testMusclZajac it does not work.

To be continued



11/20

Linked in the f-code that seems to work now problem was that for building teh excecutable in the makefile the fortramn was not immediately linked in therefore in the makefile the -lftn was inserted

Tried to include the mmintl_ for this I had to add this in the utwalking.h
Changed the definition of the ComputeMuscleForce () adding teh argument list

Excecutable runs but returns error related to the musmodel.f 
Be carefull we use th musmodel.f in the projects file

Now goes through

To be continued


11/23
worked my way through, most important was computelength effect and computevelocitryeffect

up to
lijn 1122 in rdMuscleZajac now OK 
mszero still wrong

11/24
Looks like the looping in mslm is OK mow although it starts of with a slightly different value for a and therefore c
also the solution is slightly different, but why is the change in teh fortran code coming from?

11/28
line 949 rdMuscleZajac imposed that  -lm is similar to the fortran value so overriding the above difference

If so the result for musfrc is equal

was unable to link to rdMuscle::DADT therefore created a routine within rdMuscleZajac

With the exception of line 949 it seems to work

Talked to Clay. The parameters in fortran passed through teh argument list can be changed in the routine and passed back to the initial routine. This can be solved by C by passing te reference to the variable rather than the actual variable. This works and the results produced are identical

changes the excitation and activation -> OK
changes the length nl lom+0.1 ->OK
changes the length nl lom-0.2 ->OK
changes in the velocity nl *2 -> OK
changes the length nl lom+0.5 ->OK
changes the length nl lom-0.5 ->OK
