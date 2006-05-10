// testMuscleZajac.cpp


// INCLUDES
#include <iostream>
#include <rdMuscleZajac.h>
#include <utwalking.h>

double fom,lom,tsl,pen,tact,tdeact,Vmax,fmus,actsv,actlen;
double exc,act,aforce,alength,avelocity;
double aX[2],aY[2];
double fdot,edot,atvdot;
int nmus;
double mass,slow;


int main(int argc,char *argv[])
{

// Generic parameters
        printf("initial program\n\n");

         tact=0.007667;
	 tdeact=0.014;
	 Vmax=10; 

	//Muscle specific
	// Parameters that need to be initailised for every muscle are
	// optimal fiber length (lom)
	// tendon slack length
	// Max force
	// pennation angle
	// In the present example the values for rectus femoris are used as 			//reference

	fom=779.00;
	//fom=200000;
	lom=0.084;
	tsl=0.346;
	pen=5.0;
	exc=0.4;
	act=0.9;

	printf("PRINTING FORCE%f\n",fom);
	printf("PRINTING Length%f\n",lom);
	printf("PRINTING TSL%f\n",tsl);
	printf("PRINTING Pen%f\n",pen);
	printf("PRINTING EXc%f\n",exc);
	printf("PRINTING Act%f\n",act);
	

// CALL THE FORTRAN MUSCLE MODEL

	
	actsv=0.12*2;
	actlen=lom-0.5;
	nmus=1;
	slow=0.5;
	mass=1;
	nmus=1;
	mmintl_(&nmus,&fom,&lom,&tsl,&pen,&slow,&mass);
	fmus = musfrc_(&nmus,&act,&actlen,&actsv);
	printf("PRINTING in testMuscleZajac after musfrc%f\n",fmus);
	mmatvdot_(&nmus,&exc,&act,&tact,&tdeact,&atvdot);
        printf("PRINTING after atvdot%f\n",atvdot);	
	mmfdot_(&nmus,&actlen,&actsv,&fmus,&act,&exc,&fdot,&edot);
        printf("PRINTING after mmfdot%f\n",fdot);	
	

	//Call the C++
	printf("rdMuscleZajac muscle\n");
	rdMuscleZajac muscle;

 	// ACCESS INFO FOR THE INDIVIDUAL ACTUATOR
	muscle.setOptimalForce(fom);
	muscle.setOptimalFiberLength(lom);
	muscle.setOptimalPennationAngle(pen);
	muscle.setTendonSlackLength(tsl);
	muscle.setMaxShorteningVelocity(Vmax);
	muscle.setRiseTime(tact);
	muscle.setFallTime(tdeact);
	aX[0]=exc;
	
	
	
	//In next step muscle force will need to be calculated
	
	// THE FOLLOWING SECTION ALLOWS THE CALCULATION OF ACTIVE FORCE
	//alength is set to lom for the moment
	aforce=fom;
	alength=actlen;
	avelocity=actsv;


	
	printf("Before ComputeMsForce");
	printf("PRINTING Length before ComputeMusscleForce %f\n",alength);
	aforce=muscle.ComputeMuscleForce(act,alength,avelocity);
	printf("PRINTING FORCE after ComputeMuscleForce %f\n",aforce);

	aY[0]=act;
	aY[1]=aforce;
  	muscle.setControls(aX);
    	muscle.setStates(aY);
	muscle.setShorteningVelocity(avelocity);
	muscle.setActuatorLength(alength);


	// COMPUTE STATE DERIVATIVES this calls the ComputeDFDT
        muscle.computeStateDerivatives(aY);
	
	printf("PRINTING after computeStateDerivativesn\n");	

	
return(0);
}
