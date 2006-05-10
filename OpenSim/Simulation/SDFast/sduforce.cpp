// sduforce.c
// AUTHORS: Frank C. Anderson

#include "SDFast.h"




using namespace OpenSim;
extern "C" {

//_____________________________________________________________________________
/**
 * Apply user-supplied forces to an SDFast dynamic model.
 * This routine is intended to take calls to sduforce originating from
 * SDFast and redirect them through an SDFast model by calling
 * SDFast::SDUForce().  SDFast::SDUForce() provides an entry point
 * to the C++ object-oriented model classes.  In general, direct calls to
 * sduforce() should be avoided.
 */
int sduforce(double t, double q[], double u[])
{
	SDFast::SDUForce();

   return(1);
}


}
