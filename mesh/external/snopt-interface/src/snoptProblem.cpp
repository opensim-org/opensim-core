#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "snopt.h"
#include "snoptProblem.hpp"

using namespace std;

static int pID = 7;
int snUnitS = 6;

void summaryOff()
{
  snUnitS = 0;
}

void summaryOn()
{
  snUnitS = 6;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblem::snoptProblem() : isumm(snUnitS), iprint(0)
{
  int len = 0;
  init2zero();
  sprintf(Prob, "%8s", "        " );

  allocI( 500 );
  allocR( 500 );

  probID = pID++;
  f_sninit( "", len, iprint, isumm, iw, leniw, rw, lenrw );
  initCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblem::snoptProblem( const char *name ) :
  isumm(snUnitS), iprint(0)
{
  int len = 0;
  init2zero();

  sprintf(Prob, "%8s", name );

  allocI( 500 );
  allocR( 500 );

  probID = pID++;
  iprint = probID;

  f_sninit( "", len, iprint, isumm, iw, leniw, rw, lenrw );
  initCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblem::snoptProblem( const char *name, const char *prtfile ) :
  isumm(snUnitS)
{
  int len = strlen(prtfile);
  init2zero();

  sprintf(Prob, "%8s", name );

  allocI( 500 );
  allocR( 500 );

  probID = pID++;
  iprint = probID;

  f_sninit( prtfile, len, iprint, isumm, iw, leniw, rw, lenrw );
  initCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblem::~snoptProblem()
{
  f_snend( iprint );

  delete []rw;  delete []iw;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::init2zero()
{
  inform = 0;

  initCalled = 0; memCalled = 0; allocA = 0; allocG = 0;

  leniw = 0; lenrw = 0;
  iw    = 0; rw    = 0;

  leniu = 0; lenru = 0;
  iu    = 0; ru    = 0;

  snLog = 0; snLog2 = 0;
  sqLog = 0; snSTOP = 0;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::allocI( int aleniw )
{
  // Reset work array lengths.
  // Allocate new memory for work arrays.
  leniw = aleniw;
  iw    = new int[leniw];
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::allocR( int alenrw )
{
  // Reset work array lengths.
  // Allocate new memory for work arrays.
  lenrw = alenrw;
  rw    = new double[lenrw];
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::reallocI( int aleniw )
{
  int  tleniw = leniw;
  int    *tiw = iw;

  // Allocate new memory
  allocI( aleniw );

  // Copy old workspace into new.
  int mleniw = leniw < tleniw ? leniw : tleniw;
  memcpy( iw, tiw, mleniw*sizeof(int));

  // Delete temporary work arrays
  delete []tiw;

  setIntParameter((char*)"Total int workspace", leniw );
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::reallocR( int alenrw )
{
  int  tlenrw = lenrw;
  double *trw = rw;

  // Allocate new memory
  allocR( alenrw );

  // Copy old workspace into new.
  int mlenrw = lenrw < tlenrw ? lenrw : tlenrw;
  memcpy( rw, trw, mlenrw*sizeof(double));

  // Delete temporary work arrays
  delete []trw;

  setIntParameter((char*)"Total real workspace   ", lenrw );
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::setProbName( const char *name )
{
  sprintf(Prob, "%8s", name );
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::setPrintFile( const char *prtname )
{
  assert( initCalled == 1 );

  int len = strlen(prtname);

  iprint = probID;
  f_snsetprint( prtname, len, iprint, iw, leniw, rw, lenrw );
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblem::setSpecsFile( const char *specname )
{
  assert( initCalled == 1 );

  int len = strlen(specname);
  f_snspec( specname, len, &inform, iw, leniw, rw, lenrw );
  if( inform != 101 ){
    printf("Warning: unable to find specs file %s \n", specname);
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblem::setParameter( const char *stropt )
{
  assert( initCalled == 1 );

  int errors, stropt_len = strlen(stropt);
  f_snset( stropt, stropt_len, &errors, iw, leniw, rw, lenrw );
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblem::getParameter( const char *stroptin, char *stroptout )
{
  assert( initCalled == 1 );

  int errors;
  int stroptin_len  = strlen(stroptin);
  int stroptout_len = strlen(stroptout);

  f_sngetc( stroptin, stroptin_len, stroptout, stroptout_len,
	    &errors, iw, leniw, rw, lenrw );
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblem::setIntParameter( const char *stropt, int opt )
{
  assert( initCalled == 1 );

  int errors, stropt_len = strlen(stropt);

  f_snseti( stropt, stropt_len, opt, &errors,
	    iw, leniw, rw, lenrw );
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblem::getIntParameter( const char *stropt, int &opt )
{
  assert( initCalled == 1 );

  int errors, stropt_len = strlen(stropt);
  f_sngeti( stropt, stropt_len, &opt, &errors,
	    iw, leniw, rw, lenrw );
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblem::setRealParameter( const char *stropt, double opt )
{
  assert( initCalled == 1 );

  int errors, stropt_len = strlen(stropt);
  f_snsetr( stropt, stropt_len, opt, &errors,
	    iw, leniw, rw, lenrw );
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblem::getRealParameter( const char *stropt, double &opt )
{
  assert( initCalled == 1 );

  int errors, stropt_len = strlen(stropt);
  f_sngetr( stropt, stropt_len, &opt, &errors,
	    iw, leniw, rw, lenrw );
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::setUserI( int *aiu, int aleniu )
{
  leniu = aleniu;
  iu    = aiu;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::setUserR( double *aru, int alenru )
{
  lenru = alenru;
  ru    = aru;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::setUserspace  ( int*aiu,     int aleniu,
				   double *aru, int alenru )
{
  leniu = aleniu;
  iu    = aiu;

  lenru = alenru;
  ru    = aru;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::setLog( isnLog asnLog, isnLog2 asnLog2, isqLog asqLog )
{
  snLog  = asnLog;
  snLog2 = asnLog2;
  sqLog  = asqLog;
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::setSTOP( isnSTOP asnSTOP )
{
  snSTOP = asnSTOP;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblem::errMsgExit( const char *var )
{
  cerr << endl << "XXX: Input error: " << var << " must be set prior to call to "
       << endl << "     snoptProblem::solve() or snoptProblem::computeJac()!\n";
  return 99;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemA::snoptProblemA()
{
  init2zero();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemA::snoptProblemA( const char *name ) : snoptProblem(name)
{
  init2zero();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemA::snoptProblemA( const char *name, const char *prtfile ) :
  snoptProblem(name, prtfile)
{
  init2zero();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemA::~snoptProblemA()
{
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemA::init2zero()
{
  // Data that must be set by user.
  jacComputed = 0;

  n      =  0;  neF   = 0;
  ObjRow = -1; ObjAdd = 0;

  x = 0; xlow = 0; xupp = 0; xmul = 0;
  F = 0; Flow = 0; Fupp = 0; Fmul = 0;

  xstate  = 0; Fstate  = 0;

  usrfunA = 0;

  lenA  =  0; lenG  =  0;
  neA   = -1; neG   = -1;
  iAfun =  0; jAvar =  0; A   = 0;
  iGfun =  0; jGvar =  0;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemA::userDataSet()
{
  int errors = 0;

  if ( n    == 0)  errors = errMsgExit( "n"  );
  if ( neF  == 0)  errors = errMsgExit( "neF");

  if ( x    == 0 ) errors = errMsgExit( "x"    );
  if ( xlow == 0 ) errors = errMsgExit( "xlow" );
  if ( xupp == 0 ) errors = errMsgExit( "xupp" );
  if ( xmul == 0 ) errors = errMsgExit( "xmul" );

  if ( F    == 0 ) errors = errMsgExit( "F"    );
  if ( Flow == 0 ) errors = errMsgExit( "Flow" );
  if ( Fupp == 0 ) errors = errMsgExit( "Fupp" );
  if ( Fmul == 0 ) errors = errMsgExit( "Fmul" );

  if ( usrfunA ==  0 ) errors = errMsgExit( "usrfunA" );

  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemA::setWorkspace()
{
  assert( initCalled == 1 );

  int miniw, minrw;
  int memGuess = 0;

  if ( neA < 0 ) {
    neA      = n*neF;
    memGuess = 1;
  }
  if ( neG < 0 ) {
    neG      = n*neF;
    memGuess = 1;
  }

  f_snmema( &inform, neF, n, neA, neG, &miniw, &minrw,
	    iw, leniw, rw, lenrw );

  if ( miniw > leniw ) { reallocI ( miniw ); }
  if ( minrw > lenrw ) { reallocR ( minrw ); }

  memCalled = 1;

  if ( memGuess == 1 ) {
    computeJac( neA, neG );

    assert ( neA >= 0 );
    assert ( neG >= 0 );

    f_snmema( &inform, neF, n, neA, neG, &miniw, &minrw,
	      iw, leniw, rw, lenrw );

    if ( miniw > leniw ) { reallocI ( miniw ); }
    if ( minrw > lenrw ) { reallocR ( minrw ); }
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemA::computeJac(int &aneA, int &aneG)
{
  assert( initCalled == 1 );
  aneA = 0; aneG = 0;

  if ( lenA <= 0 ) {
    lenA = n*neF;
    iAfun  = new int[lenA];
    jAvar  = new int[lenA];
    A      = new double[lenA];
    allocA = 1;
  }

  if ( lenG <= 0 ) {
    lenG   = n*neF;
    iGfun  = new int[lenG];
    jGvar  = new int[lenG];
    allocG = 1;
  }

  if (userDataSet() != 0 ) { return 99; }

  int miniw, minrw;

  if ( memCalled == 0 ) { setWorkspace(); }

  f_snjac( &inform, neF, n, usrfunA, x, xlow, xupp,
	   iAfun, jAvar, lenA, &neA, A,
	   iGfun, jGvar, lenG, &neG,
	   &miniw, &minrw, iu, leniu, ru, lenru,
	   iw, leniw, rw, lenrw );

  aneA = neA; aneG = neG;
  jacComputed = 1;

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemA::solve( int starttype )
{
  assert( initCalled == 1 );

  //Ensures all user data initialized.
  if (userDataSet() != 0 ) { return 99; }

  int nS, nInf, miniw, minrw, snObjRow;
  double sInf;

  if ( memCalled == 0 ) { setWorkspace(); }

  if ( jacComputed == 0 ) {
    for ( int i = 0; i < lenA; i++ ) {
      iAfun[i]++; jAvar[i]++;
    }
    for ( int i = 0; i < lenG; i++ ) {
      iGfun[i]++; jGvar[i]++;
    }
  }
  snObjRow = ObjRow+1;

  f_snkera ( starttype, Prob,
	     neF, n, ObjAdd, snObjRow, usrfunA,
	     snLog, snLog2, sqLog, snSTOP,
	     iAfun, jAvar, neA, A,
	     iGfun, jGvar, neG,
	     xlow, xupp, Flow, Fupp, x, xstate,
	     xmul, F, Fstate, Fmul,
	     &inform, &nS, &nInf, &sInf,
	     &miniw, &minrw, iu, leniu, ru, lenru,
	     iw, leniw, rw, lenrw );

  if ( jacComputed == 0 ) {
    for ( int i = 0; i < lenA; i++ ) {
      iAfun[i]--; jAvar[i]--;
    }
    for ( int i = 0; i < lenG; i++ ) {
      iGfun[i]--; jGvar[i]--;
    }
  }

  if ( allocA == 1 ) {
    delete []iAfun;
    delete []jAvar;
    delete []A;
  }

  if ( allocG == 1 ) {
    delete []iGfun;
    delete []jGvar;
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemA::setProblemSize( int an, int aneF )
{
  n   = an;
  neF = aneF;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemA::setObjective( int aObjRow, double aObjAdd )
{
  ObjRow = aObjRow;
  ObjAdd = aObjAdd;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemA::setA( int lenA0, int neA0, int *iAfun0, int *jAvar0,
			  double *A0 )
{
  lenA  = lenA0;  iAfun = iAfun0;  jAvar = jAvar0;  A = A0;
  jacComputed = 0;  neA = neA0;
  allocA = 0;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemA::setG( int lenG0, int neG0, int *iGfun0, int *jGvar0 )
{
  lenG  = lenG0;  iGfun = iGfun0;  jGvar = jGvar0;
  jacComputed = 0;  neG = neG0;
  allocG = 0;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemA::setX( double *ax, double *axlow, double *axupp,
                         double *axmul, int *axstate )
{
  x      = ax;
  xlow   = axlow;
  xupp   = axupp;
  xmul   = axmul;
  xstate = axstate;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemA::setF( double *aF, double *aFlow, double *aFupp,
                         double *aFmul, int *aFstate )
{
  F      = aF;
  Flow   = aFlow;
  Fupp   = aFupp;
  Fmul   = aFmul;
  Fstate = aFstate;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemA::setUserFun( snFunA ausrfun )
{
  usrfunA = ausrfun;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemC::snoptProblemC()
{
  init2zero();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemC::snoptProblemC( const char *name ) : snoptProblem(name)
{
  init2zero();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemC::snoptProblemC( const char *name, const char *prtfile ) :
  snoptProblem(name, prtfile)
{
  init2zero();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemC::~snoptProblemC()
{
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemC::init2zero()
{
  m      =  0; n     = 0; ne    = 0; iObj   = -1;
  nnCon  =  0; nnObj = 0; nnJac = 0; ObjAdd =  0;
  negCon = -1;

  hs  = 0; x  = 0;
  bl  = 0; bu = 0;
  pi  = 0; rc = 0;

  Jval = 0; indJ = 0; locJ = 0;

  usrfunC = 0;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemC::userDataSet()
{
  int errors = 0;

  if ( n    == 0)  errors = errMsgExit( "n"     );
  if ( m    == 0)  errors = errMsgExit( "m"     );

  if ( ne   == 0)  errors = errMsgExit( "ne"    );

  if ( hs   == 0 ) errors = errMsgExit( "hs"    );
  if ( x    == 0 ) errors = errMsgExit( "x"     );
  if ( bl   == 0 ) errors = errMsgExit( "bl"    );
  if ( bu   == 0 ) errors = errMsgExit( "bu"    );
  if ( pi   == 0 ) errors = errMsgExit( "pi"    );
  if ( rc   == 0 ) errors = errMsgExit( "rc"    );

  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemC::setWorkspace()
{
  assert( initCalled == 1 );

  int miniw, minrw;
  int memGuess = 0;
  if ( negCon < 0 ) {
    negCon   = ne; //nnCon*nnJac;
    memGuess = 1;
  }

  f_snmem ( &inform, m, n, ne, negCon, nnCon, nnObj, nnJac,
	    &miniw, &minrw, iw, leniw, rw, lenrw );

  if ( miniw > leniw ) { reallocI ( miniw ); }
  if ( minrw > lenrw ) { reallocR ( minrw ); }

  memCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemC::solve( int starttype )
{
  assert( initCalled == 1 );

  //Ensures all user data initialized.

  if (usrfunC ==  0) {
    errMsgExit( "usrfunC" );
    return 99;
  }
  if (userDataSet() != 0 ) { return 99; }

  int sniObj, nS, nInf, miniw, minrw;
  double sInf, Obj;

  if ( memCalled == 0 ) { setWorkspace(); }

  for ( int i = 0; i < ne; i++ ) {
    indJ[i]++;
  }
  for ( int i = 0; i <= n; i++ ) {
    locJ[i]++;
  }

  sniObj = iObj+1;

  f_snkerc( starttype, Prob,
	    m, n, ne, nnCon, nnObj, nnJac,
	    sniObj, ObjAdd, usrfunC,
	    snLog, snLog2, sqLog, snSTOP,
	    Jval, indJ, locJ,
	    bl, bu, hs, x, pi, rc,
	    &inform, &nS, &nInf, &sInf, &Obj,
	    &miniw, &minrw, iu, leniu, ru, lenru,
	    iw, leniw, rw, lenrw );

  for ( int i = 0; i < ne; i++ ) {
    indJ[i]--;
  }
  for ( int i = 0; i <= n; i++ ) {
    locJ[i]--;
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemC::setProblemSize( int am, int an, int annCon,
				    int annJac, int annObj)
{
  m = am;  n = an;
  nnCon = annCon;
  nnJac = annJac;
  nnObj = annObj;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemC::setObjective( int aiObj, double aObjAdd )
{
  iObj   = aiObj;
  ObjAdd = aObjAdd;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemC::setJ( int ane, double *aJval, int *aindJ, int *alocJ )
{
  ne   = ane;
  Jval = aJval;
  indJ = aindJ;
  locJ = alocJ;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemC::setX( double *abl, double *abu, double *ax,
			  double *api, double *arc, int *ahs)
{
  x  = ax;  hs = ahs;
  bl = abl; bu = abu;
  pi = api; rc = arc;
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemC::setUserFun( snFunC ausrfun )
{
  usrfunC = ausrfun;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemB::snoptProblemB() : snoptProblemC()
{
  init2zero();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemB::snoptProblemB( const char *name ) : snoptProblemC(name)
{
  init2zero();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemB::snoptProblemB( const char *name, const char *prtfile ) :
  snoptProblemC(name,prtfile)
{
  init2zero();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemB::~snoptProblemB()
{
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemB::init2zero()
{
  funobj = 0; funcon = 0;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemB::userDataSet()
{
  int errors = 0;

  errors = snoptProblemC::userDataSet();
  if ( funobj ==  0 ) errors = errMsgExit( "funobj" );
  if ( funcon ==  0 ) errors = errMsgExit( "funcon" );

  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemB::solve( int starttype )
{
  assert( initCalled == 1 );

  //Ensures all user data initialized.
  if (userDataSet() != 0 ) { return 99; }

  int sniObj, nS, nInf, miniw, minrw;
  double sInf, Obj;

  if ( memCalled == 0 ) { setWorkspace(); }

  for ( int i = 0; i < ne; i++ ) {
    indJ[i]++;
  }
  for ( int i = 0; i <= n; i++ ) {
    locJ[i]++;
  }
  sniObj = iObj+1;

  f_snkerb( starttype, Prob,
	    m, n, ne, nnCon, nnObj, nnJac,
	    sniObj, ObjAdd, funcon, funobj,
	    snLog, snLog2, sqLog, snSTOP,
	    Jval, indJ, locJ,
	    bl, bu, hs, x, pi, rc,
	    &inform, &nS, &nInf, &sInf, &Obj,
	    &miniw, &minrw, iu, leniu, ru, lenru,
	    iw, leniw, rw, lenrw );

  for ( int i = 0; i < ne; i++ ) {
    indJ[i]--;
  }
  for ( int i = 0; i <= n; i++ ) {
    locJ[i]--;
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemB::setFuncon( snConB afuncon )
{
  funcon = afuncon;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemB::setFunobj( snObjB afunobj )
{
  funobj = afunobj;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
