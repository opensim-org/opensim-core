// This may look kind of scary but it's just a way to simplify the number of lines of code needed in SdfastEngine
// to handle the SD/Fast function pointers.  Basically we define a macro here, OPENSIM_FOR_ALL_SDFAST_FUNCTIONS, which
// is like an iterator that applies any macro you give it to all of the functions we want to handle pointers for.
// This way if a function pointer is desired for a new SD/Fast function, it only needs to be added to the list below.
// The format is
//
//		macro( <function name>, <function's return type>, <functions arguments surrounded by parentheses> )
//
// See SdfastEngine.{h,cpp} for how this FOR_ALL macro is used.

#define OPENSIM_FOR_ALL_SDFAST_FUNCTIONS(macro) \
	macro(sdinit, void, ()); \
	macro(sdgrav, void, (double aGrav[3])); \
	macro(sdgetgrav, void, (double rGrav[3])); \
	macro(sdstate, void, (double t,double *q,double *u)); \
	macro(sdassemble, void, (double time,double *state,int *lock,double tol, int maxevals,int *fcnt,int *err )); \
	macro(sdinitvel, void, (double time, double state[100], int lock[50], double tol, int maxevals, int *fcnt, int *err)); \
	macro(sdbodyt, void, (int aBody,double aTorque[3])); \
	macro(sdpointf, void, (int aBody,double aPoint[3],double aForce[3])); \
	macro(sdhinget, void, (int aJoint,int aAxis,double aF)); \
	macro(sdpres, void, (int aJoint,int aAxis,int aPrescribed)); \
	macro(sdderiv, void, (double *dqdt,double *dudt)); \
	macro(sdgetht, void, (int aJoint,int aAxis,double *aForce)); \
	macro(sdreac, void, (double forces[][3],double torques[][3])); \
	macro(sdequivht, void, (double force[])); \
	macro(sdmassmat, void, (double *rI)); \
	macro(sdrel2cart, void, (int coord,int body,double pt[3],double lin[3],double rot[3])); \
	macro(sdpos, void, (int aBody,double aPoint[3],double rPos[3])); \
	macro(sdvel, void, (int aBody,double aPoint[3],double rVel[3])); \
	macro(sdacc, void, (int aBody,double aPoint[3],double rAcc[3])); \
	macro(sdorient, void, (int aBody,double rDirCos[3][3])); \
	macro(sdangvel, void, (int aBody,double rAngVel[3])); \
	macro(sdangacc, void, (int aBody,double rAngAcc[3])); \
	macro(sdtrans, void, (int aBody1,double aVec1[3],int aBody2,double rVec2[3])); \
	macro(sdcomptrq, void, (double *aUDot,double *rF)); \
	macro(sdang2st, void, (double *qang,double *q)); \
	macro(sdst2ang, void, (double *q,double *qang)); \
	macro(sddc2ang, void, (double aDirCos[3][3],double *rE1,double *rE2,double *rE3)); \
	macro(sdang2dc, void, (double aE1,double aE2,double aE3,double rDirCos[3][3])); \
	macro(sddc2quat, void, (double aDirCos[3][3],double *rQ1,double *rQ2,double *rQ3,double *rQ4)); \
	macro(sdquat2dc, void, (double aQ1,double aQ2,double aQ3,double aQ4,double rDirCos[3][3])); \
	macro(sdbtj, void, (int body,double btj[3])); \
	macro(sdgetbtj, void, (int body,double btj[3])); \
	macro(sditj, void, (int body,double itj[3])); \
	macro(sdgetitj, void, (int body,double itj[3])); \
	macro(sdpin, void, (int body,int pinNumber,double pin[3])); \
	macro(sdgetpin, void, (int body,int pinNumber,double pin[3])); \
	macro(sdinfo, void, (int info[50])); \
	macro(sdjnt, void, (int joint,int info[50],int slider[6])); \
	macro(sdsys, void, (double *aM,double rCOM[3],double rI[3][3])); \
	macro(sdmass, void, (int aBody,double aMass)); \
	macro(sdgetmass, void, (int aBody,double *rMass)); \
	macro(sdiner, void, (int aBody,double aInertia[3][3])); \
	macro(sdgetiner, void, (int aBody,double rInertia[3][3])); \
	macro(sdgetpres, void, (int aJoint, int aAxis, int *rPresout)); \
	macro(sdprespos, void, (int aJoint, int aAxis, double aPrval)); \
	macro(sdpresvel, void, (int aJoint, int aAxis, double aPrval)); \
	macro(sdpresacc, void, (int aJoint, int aAxis, double aPrval)); \
	macro(init_sdm, void, ()); \
	macro(setJointConstraintFunctions, int, (CoordinateSet *aCoordinateSet)); \
	macro(setCoordinateInitialValues, int, (CoordinateSet *aCoordinateSet)); \
	macro(compute_constrained_coords, void, (double *y)); \
	macro(setSdfastEngineInstance, void, (SdfastEngine *aEngine));
