// testMuscleZajac.cpp
/*
* Copyright (c)  2005, Stanford University, All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


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
