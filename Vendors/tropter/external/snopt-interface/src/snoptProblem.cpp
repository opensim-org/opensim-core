#include <assert.h>
#include <cstring>
#include <iostream>
#include <stdio.h>
#include "snopt.h"
#include "snoptProblem.hpp"

using namespace std;

static const char *snversion =
  " ==============================\n\
   SNOPT C++ interface  2.2.0   ";
//  123456789|123456789|123456789|


static const char *sqversion =
  " ==============================\n\
   SQOPT C++ interface  2.2.0   ";
//  123456789|123456789|123456789|


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblem::snoptProblem(const char *name, int *aiw, int aleniw, double *arw, int alenrw) {
  init2zero();
  sprintf(Prob, "%8s", name);

  if (aleniw >= 500 && alenrw >= 500) {
    if (aiw != 0 && arw != 0) {
      leniw = aleniw;
      lenrw = alenrw;
      iw    = aiw;
      rw    = arw;
      userWork  = 1;
    }
    else {
      allocI(aleniw);
      allocR(alenrw);
    }
    memCalled = 1;
  }
  else {
    allocI(500);
    allocR(500);
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblem::~snoptProblem() {
  f_snend(iw, leniw, rw, lenrw);

  if (userWork == 0) {
    delete []rw;  delete []iw;
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::init2zero() {
  initCalled = 0; memCalled = 0; userWork = 0;

  leniw = 0; lenrw = 0;
  iw    = 0; rw    = 0;

  leniu = 0; lenru = 0;
  iu    = 0; ru    = 0;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::allocI(int aleniw) {
  // Reset work array lengths.
  // Allocate new memory for work arrays.
  assert(aleniw >= 500);
  leniw = aleniw;
  iw    = new int[leniw];
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::allocR(int alenrw) {
  // Reset work array lengths.
  // Allocate new memory for work arrays.
  assert(alenrw >= 500);
  lenrw = alenrw;
  rw    = new double[lenrw];
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::reallocI(int aleniw) {
  int  tleniw = leniw;
  int    *tiw = iw;

  // Allocate new memory
  allocI(aleniw);

  // Copy old workspace into new.
  int mleniw = leniw < tleniw ? leniw : tleniw;
  memcpy(iw, tiw, mleniw*sizeof(int));

  // Delete temporary work arrays
  delete []tiw;

  setIntParameter((char*)"Total int workspace", leniw);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::reallocR(int alenrw) {
  int  tlenrw = lenrw;
  double *trw = rw;

  // Allocate new memory
  allocR(alenrw);

  // Copy old workspace into new.
  int mlenrw = lenrw < tlenrw ? lenrw : tlenrw;
  memcpy(rw, trw, mlenrw*sizeof(double));

  // Delete temporary work arrays
  delete []trw;

  setIntParameter((char*)"Total real workspace   ", lenrw);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::setProbName(const char *name) {
  sprintf(Prob, "%8s", name);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::setPrintFile(const char *prtname) {
  assert(initCalled == 1);

  int len = strlen(prtname);
  f_snsetprintf(prtname, len, iw, leniw, rw, lenrw);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::setPrintFile(const char *prtname, int iprint) {
  assert(initCalled == 1);

  int len = strlen(prtname);
  f_snsetprint(prtname, len, iprint, iw, leniw, rw, lenrw);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblem::setParameter(const char *stropt) {
  assert(initCalled == 1);

  int errors, stropt_len = strlen(stropt);
  f_snset(stropt, stropt_len, &errors, iw, leniw, rw, lenrw);
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblem::setIntParameter(const char *stropt, int opt) {
  assert(initCalled == 1);

  int errors, stropt_len = strlen(stropt);

  f_snseti(stropt, stropt_len, opt, &errors,
	    iw, leniw, rw, lenrw);
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblem::getIntParameter(const char *stropt, int &opt) {
  assert(initCalled == 1);

  int errors, stropt_len = strlen(stropt);
  f_sngeti(stropt, stropt_len, &opt, &errors,
	    iw, leniw, rw, lenrw);
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblem::setRealParameter(const char *stropt, double opt) {
  assert(initCalled == 1);

  int errors, stropt_len = strlen(stropt);
  f_snsetr(stropt, stropt_len, opt, &errors,
	    iw, leniw, rw, lenrw);
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblem::getRealParameter(const char *stropt, double &opt) {
  assert(initCalled == 1);

  int errors, stropt_len = strlen(stropt);
  f_sngetr(stropt, stropt_len, &opt, &errors,
	    iw, leniw, rw, lenrw);
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::setUserI(int *aiu, int aleniu) {
  leniu = aleniu;
  iu    = aiu;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::setUserR(double *aru, int alenru) {
  lenru = alenru;
  ru    = aru;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblem::setUserspace  (int*aiu,     int aleniu,
				   double *aru, int alenru) {
  leniu = aleniu;
  iu    = aiu;

  lenru = alenru;
  ru    = aru;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemABC::snoptProblemABC(const char *name, int *aiw, int aleniw,
				 double *arw, int alenrw) :
  snoptProblem(name, aiw, aleniw, arw, alenrw) {
  init2zero();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemABC::~snoptProblemABC() {
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemABC::init2zero() {
  snLog  = 0; snLog2 = 0;
  sqLog  = 0; snSTOP = 0;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemABC::initialize(const char*prtfile, int summOn) {
  int len = strlen(prtfile);

  if (summOn != 0) {
    std::cout << snversion;
  }

  f_sninitf(prtfile, len, summOn, iw, leniw, rw, lenrw);
  initCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemABC::initialize(const char*prtfile, int iprint,
				 const char*sumfile, int isumm) {
  int plen, slen;
  plen = strlen(prtfile);
  slen = strlen(prtfile);

  if (isumm != 0) {
    std::cout << snversion;
  }

  f_sninit(prtfile, plen, iprint, sumfile, slen, isumm,
	   iw, leniw, rw, lenrw);
  initCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemABC::setSpecsFile(const char *specname) {
  assert(initCalled == 1);

  int inform, len = strlen(specname);
  f_snspecf(specname, len, &inform, iw, leniw, rw, lenrw);
  if (inform != 101){
    std::cerr << "Warning: unable to find specs file " <<
      specname << std::endl;
  }
  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemABC::setSpecsFile(const char *specname, int ispecs) {
  assert(initCalled == 1);

  int inform, len = strlen(specname);
  f_snspec(specname, len, ispecs, &inform, iw, leniw, rw, lenrw);
  if(inform != 101){
    std::cerr << "Warning: unable to find specs file " <<
      specname << std::endl;
  }
  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemABC::setLog(isnLog asnLog, isnLog2 asnLog2,
			     isqLog asqLog) {
  snLog  = asnLog;
  snLog2 = asnLog2;
  sqLog  = asqLog;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemABC::setSTOP(isnSTOP asnSTOP) {
  snSTOP = asnSTOP;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemA::snoptProblemA(const char *name,
			     int *aiw, int aleniw, double *arw, int alenrw) :
  snoptProblemABC(name,aiw,aleniw,arw,alenrw) {
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemA::~snoptProblemA() {
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemA::setWorkspace(int neF, int n, int neA, int neG) {
  assert(initCalled == 1);

  int inform, miniw, minrw;

  f_snmema(&inform, neF, n, neA, neG, &miniw, &minrw,
	    iw, leniw, rw, lenrw);

  if (miniw > leniw) { reallocI (miniw); }
  if (minrw > lenrw) { reallocR (minrw); }

  memCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemA::computeJac(int neF, int n, snFunA usrfunA,
			      double *x, double *xlow, double*xupp,
			      int *&iAfun, int *&jAvar, double *&A, int &neA,
			      int *&iGfun, int *&jGvar, int &neG) {
  assert(initCalled == 1);

  int inform, lenA, lenG, miniw, minrw;

  lenA  = n*neF;
  lenG  = n*neF;

  if (memCalled == 0) { setWorkspace(neF, n, lenA, lenG); }

  f_snjac(&inform, neF, n, usrfunA, x, xlow, xupp,
	  iAfun, jAvar, lenA, &neA, A,
	  iGfun, jGvar, lenG, &neG,
	  &miniw, &minrw, iu, leniu, ru, lenru,
	  iw, leniw, rw, lenrw);

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemA::solve(int starttype, int neF, int n, double ObjAdd,
			 int ObjRow, snFunA usrfunA,
			 double *xlow, double *xupp, double *Flow, double *Fupp,
			 double *x, int *xstate, double *xmul,
			 double *F, int *Fstate, double *Fmul,
			 int &nS, int &nInf, double &sInf) {
  assert(initCalled == 1);

  int inform, miniw, minrw, neA, neG, snObjRow;
  int *iAfun, *jAvar, *iGfun, *jGvar;
  double *A;

  iAfun  = new int[n*neF];
  jAvar  = new int[n*neF];
  A      = new double[n*neF];

  iGfun  = new int[n*neF];
  jGvar  = new int[n*neF];

  // computeJac will check for memCalled.
  computeJac(neF, n, usrfunA, x, xlow, xupp,
	     iAfun, jAvar, A, neA,
	     iGfun, jGvar, neG);

  snObjRow = ObjRow+1;

  f_snkera(starttype, Prob, neF, n, ObjAdd, snObjRow, usrfunA,
	   snLog, snLog2, sqLog, snSTOP,
	   iAfun, jAvar, neA, A, iGfun, jGvar, neG,
	   xlow, xupp, Flow, Fupp,
	   x, xstate, xmul, F, Fstate, Fmul,
	   &inform, &nS, &nInf, &sInf,
	   &miniw, &minrw,
	   iu, leniu, ru, lenru,
	   iw, leniw, rw, lenrw);

  delete []iAfun;
  delete []jAvar;
  delete []A;

  delete []iGfun;
  delete []jGvar;

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemA::solve(int starttype, int neF, int n, double ObjAdd,
			 int ObjRow, snFunA usrfunA,
			 int *iAfun, int *jAvar, double *A, int neA,
			 int *iGfun, int *jGvar, int neG,
			 double *xlow, double *xupp, double *Flow, double *Fupp,
			 double *x, int *xstate, double *xmul,
			 double *F, int *Fstate, double *Fmul,
			 int &nS, int &nInf, double &sInf) {
  assert(initCalled == 1);

  int inform, miniw, minrw, snObjRow;

  if (memCalled == 0) { setWorkspace(neF, n, neA, neG); }

  for (int i = 0; i < neA; i++) {
    iAfun[i]++; jAvar[i]++;
  }
  for (int i = 0; i < neG; i++) {
    iGfun[i]++; jGvar[i]++;
  }

  snObjRow = ObjRow+1;

  f_snkera(starttype, Prob, neF, n, ObjAdd, snObjRow, usrfunA,
	   snLog, snLog2, sqLog, snSTOP,
	   iAfun, jAvar, neA, A, iGfun, jGvar, neG,
	   xlow, xupp, Flow, Fupp,
	   x, xstate, xmul, F, Fstate, Fmul,
	   &inform, &nS, &nInf, &sInf,
	   &miniw, &minrw,
	   iu, leniu, ru, lenru,
	   iw, leniw, rw, lenrw);

  for (int i = 0; i < neA; i++) {
    iAfun[i]--; jAvar[i]--;
  }
  for (int i = 0; i < neG; i++) {
    iGfun[i]--; jGvar[i]--;
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemC::snoptProblemC(const char *name,
			     int *aiw, int aleniw, double *arw, int alenrw) :
  snoptProblemABC(name,aiw,aleniw,arw,alenrw) {
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemC::~snoptProblemC() {
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void snoptProblemC::setWorkspace(int m, int n, int ne, int negCon,
				 int nnCon, int nnObj, int nnJac) {
  assert(initCalled == 1);

  int inform, miniw, minrw;

  if (negCon < 0) {
    negCon   = nnCon*nnJac;
  }

  f_snmem (&inform, m, n, ne, negCon, nnCon, nnObj, nnJac,
	   &miniw, &minrw, iw, leniw, rw, lenrw);

  if (miniw > leniw) { reallocI (miniw); }
  if (minrw > lenrw) { reallocR (minrw); }

  memCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemC::solve(int starttype, int m, int n, int ne, int negCon,
			 int nnCon, int nnObj, int nnJac,
			 int iObj, double ObjAdd, snFunC usrfunC,
			 double *Jval, int *indJ, int *locJ,
			 double *bl, double *bu, int *hs,
			 double *x, double *pi, double *rc,
			 int &nS, int &nInf, double &sInf, double &objective) {
  assert(initCalled == 1);

  int inform, sniObj, miniw, minrw;

  if (memCalled == 0) {
    setWorkspace(m, n, ne, negCon, nnCon, nnObj, nnJac);
  }

  for (int i = 0; i < ne; i++) {
    indJ[i]++;
  }
  for (int i = 0; i <= n; i++) {
    locJ[i]++;
  }

  sniObj = iObj+1;

  f_snkerc(starttype, Prob,
	    m, n, ne, nnCon, nnObj, nnJac,
	    sniObj, ObjAdd, usrfunC,
	    snLog, snLog2, sqLog, snSTOP,
	    Jval, indJ, locJ,
	    bl, bu, hs, x, pi, rc,
	    &inform, &nS, &nInf, &sInf, &objective,
	    &miniw, &minrw, iu, leniu, ru, lenru,
	    iw, leniw, rw, lenrw);

  for (int i = 0; i < ne; i++) {
    indJ[i]--;
  }
  for (int i = 0; i <= n; i++) {
    locJ[i]--;
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemB::snoptProblemB(const char *name,
			     int *aiw, int aleniw, double *arw, int alenrw) :
  snoptProblemC(name,aiw,aleniw,arw,alenrw) {
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
snoptProblemB::~snoptProblemB() {
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int snoptProblemB::solve(int starttype, int m, int n, int ne, int negCon,
			 int nnCon, int nnObj, int nnJac,
			 int iObj, double ObjAdd,
			 snConB funcon, snObjB funobj,
			 double *Jval, int *indJ, int *locJ,
			 double *bl, double *bu, int *hs,
			 double *x, double *pi, double *rc,
			 int &nS, int &nInf, double &sInf, double &objective) {
  assert(initCalled == 1);

  int inform, sniObj, miniw, minrw;

  if (memCalled == 0) { setWorkspace(m, n, ne, negCon, nnCon, nnObj, nnJac); }

  for (int i = 0; i < ne; i++) {
    indJ[i]++;
  }
  for (int i = 0; i <= n; i++) {
    locJ[i]++;
  }
  sniObj = iObj+1;

  f_snkerb(starttype, Prob,
	    m, n, ne, nnCon, nnObj, nnJac,
	    sniObj, ObjAdd, funcon, funobj,
	    snLog, snLog2, sqLog, snSTOP,
	    Jval, indJ, locJ,
	    bl, bu, hs, x, pi, rc,
	    &inform, &nS, &nInf, &sInf, &objective,
	    &miniw, &minrw, iu, leniu, ru, lenru,
	    iw, leniw, rw, lenrw);

  for (int i = 0; i < ne; i++) {
    indJ[i]--;
  }
  for (int i = 0; i <= n; i++) {
    locJ[i]--;
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
sqoptProblem::sqoptProblem(const char *name,
			     int *aiw, int aleniw, double *arw, int alenrw) :
  snoptProblem(name,aiw,aleniw,arw,alenrw) {
  init2zero();
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
sqoptProblem::~sqoptProblem() {
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void sqoptProblem::init2zero() {
  sqLog = 0;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void sqoptProblem::initialize(const char*prtfile, int summOn) {
  int len = strlen(prtfile);

  if (summOn != 0) {
    std::cout << sqversion;
  }

  f_sqinitf(prtfile, len, summOn, iw, leniw, rw, lenrw);
  initCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void sqoptProblem::initialize(const char*prtfile, int iprint,
			      const char*sumfile, int isumm) {
  int plen, slen;
  plen = strlen(prtfile);
  slen = strlen(prtfile);

  if (isumm != 0) {
    std::cout << sqversion;
  }

  f_sqinit(prtfile, plen, iprint, sumfile, slen, isumm,
	   iw, leniw, rw, lenrw);
  initCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int sqoptProblem::setSpecsFile(const char *specname) {
  assert(initCalled == 1);

  int inform, len = strlen(specname);
  f_sqspecf(specname, len, &inform, iw, leniw, rw, lenrw);
  if(inform != 101){
    std::cerr << "Warning: unable to find specs file " <<
      specname << std::endl;
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int sqoptProblem::setSpecsFile(const char *specname, int ispecs) {
  assert(initCalled == 1);

  int inform, len = strlen(specname);
  f_sqspec(specname, len, ispecs, &inform, iw, leniw, rw, lenrw);
  if(inform != 101){
    std::cerr << "Warning: unable to find specs file " <<
      specname << std::endl;
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void sqoptProblem::setLog(isqLog asqLog) {
  sqLog  = asqLog;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
void sqoptProblem::setWorkspace(int m, int n, int neA, int ncObj, int nnH) {
  assert(initCalled == 1);

  int inform, miniw, minrw;

  f_sqmem(&inform, m, n, neA, ncObj, nnH,
	   &miniw, &minrw, iw, leniw, rw, lenrw);

  if (miniw > leniw) { reallocI (miniw); }
  if (minrw > lenrw) { reallocR (minrw); }

  memCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
int sqoptProblem::solve(int starttype, sqFunHx qpHx,
			int m, int n, int neA, int ncObj, int nnH,
			int iObj, double ObjAdd,
			double *A, int *indA, int *locA,
			double *bl, double *bu, double *cObj,
			int *eType, int *hs, double *x, double *pi, double *rc,
			int &nS, int &nInf, double &sInf, double &objective) {
  assert(initCalled == 1);

  int inform, sniObj, miniw, minrw;

  if (memCalled == 0) { setWorkspace(m, n, neA, ncObj, nnH); }

  for (int i = 0; i < neA; i++) {
    indA[i]++;
  }
  for (int i = 0; i <= n; i++) {
    locA[i]++;
  }

  sniObj = iObj+1;

  f_snkerq(starttype, qpHx, sqLog,
	   m, n, neA, ncObj, nnH,
	   sniObj, ObjAdd, Prob,
	   A, indA, locA, bl, bu, cObj,
	   eType, hs, x, pi, rc,
	   &inform, &nS, &nInf, &sInf, &objective,
	   &miniw, &minrw, iu, leniu, ru, lenru,
	   iw, leniw, rw, lenrw);

  for (int i = 0; i < neA; i++) {
    indA[i]--;
  }
  for (int i = 0; i <= n; i++) {
    locA[i]--;
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
