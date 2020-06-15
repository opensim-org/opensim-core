#include "sqopt_cwrap.h"

static char *sqversion =
  " ==============================\n\
    SQOPT  C interface  2.2.0   ";
//  123456789|123456789|123456789|

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void sqInitX(sqProblem* prob, char* name,
	     char* prtfile, int iprint, char *sumfile, int isumm) {
  /*
   * sqInitX - call sqInit to initialize workspace for SQOPT
   *
   * On entry:
   *   prob     is the sqProblem struct
   *   name     is the name of the problem
   *   prtfile  is the name of the output print file
   *   iprint   is the Fortran file unit number to use for prtfile
   *            (iPrint == 0 if no print output)
   *   sumfile  is the name of the summary output file
   *            (iSumm == 0 if no summary output)
   *   isumm    is the Fortran file unit number to use for sumfile
   *
   * On exit:
   *   Internal workspace for SQOPT is initialized
   */
  sqInitXW(prob, name, prtfile, iprint, sumfile, isumm, 0, 0, 0, 0);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void sqInitXW(sqProblem* prob, char* name,
	      char* prtfile, int iprint, char *sumfile, int isumm,
	      int *iw, int leniw, double *rw, int lenrw) {
  /*
   * sqInitXW - call sqInit to initialize workspace for SQOPT
   *
   * On entry:
   *   prob     is the sqProblem struct
   *   name     is the name of the problem
   *   prtfile  is the name of the output print file
   *   iprint   is the Fortran file unit number to use for prtfile
   *            (iPrint == 0 if no print output)
   *   sumfile  is the name of the summary output file
   *            (iSumm == 0 if no summary output)
   *   isumm    is the Fortran file unit number to use for sumfile
   *
   *   iw, leniw
   *   rw, lenrw
   *            are the integer and real workspaces and their lengths.
   *            If leniw/lenrw < 500 or iw/rw is null, the workspace will
   *            be allocated automatically.
   *
   * On exit:
   *   Internal workspace for SQOPT is initialized
   */
  int plen, slen;

  init2zeroQ(prob);
  if (leniw >= 500 && lenrw >= 500) {
    if (iw != 0 && rw != 0) {
      prob->leniw = leniw;
      prob->iw    = iw;

      prob->lenrw = lenrw;
      prob->rw    = rw;

      prob->userWork = 1;
    } else {
      allocIQ(prob,leniw);
      allocRQ(prob,lenrw);
    }
    prob->memCalled = 1;

  } else {
    allocIQ(prob,500);
    allocRQ(prob,500);
  }

  prob->name   = name;

  plen = strlen(prtfile);
  slen = strlen(sumfile);

  if (isumm != 0) {
    printf("%s",sqversion);
  }

  f_sqinit(prtfile, plen, iprint, sumfile, slen, isumm,
	   prob->iw, prob->leniw, prob->rw, prob->lenrw);
  prob->initCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void sqInit(sqProblem* prob, char* name, char* prtfile, int summOn) {
  /*
   * sqInit - call snInit to initialize workspace for SQOPT
   * On entry:
   *   prob     is the sqProblem struct
   *   name     is the name of the problem
   *   prtfile  is the name of the output print file
   *            (empty string for no print file)
   *   summOn   is an integer indicating whether summary output
   *            (to screen) should be turned on (!= 0) or off (== 0)
   *
   * On exit:
   *   Internal workspace for SQOPT is initialized
   */
  sqInitW(prob, name, prtfile, summOn, 0, 0, 0, 0);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void sqInitW(sqProblem* prob, char* name, char* prtfile, int summOn,
	     int *iw, int leniw, double *rw, int lenrw) {
  /*
   * sqInitW - call snInit to initialize workspace for SQOPT
   * On entry:
   *   prob     is the sqProblem struct
   *   name     is the name of the problem
   *   prtfile  is the name of the output print file
   *            (empty string for no print file)
   *   summOn   is an integer indicating whether summary output
   *            (to screen) should be turned on (!= 0) or off (== 0)
   *
   *   iw, leniw
   *   rw, lenrw
   *            are the integer and real workspaces and their lengths.
   *            If leniw/lenrw < 500 or iw/rw is null, the workspace will
   *            be allocated automatically.
   *
   * On exit:
   *   Internal workspace for SQOPT is initialized
   */
  int len;

  init2zeroQ(prob);

  if (leniw >= 500 && lenrw >= 500) {
    if (iw != 0 && rw != 0) {
      prob->leniw = leniw;
      prob->iw    = iw;

      prob->lenrw = lenrw;
      prob->rw    = rw;

      prob->userWork = 1;
    } else {
      allocIQ(prob,leniw);
      allocRQ(prob,lenrw);
    }
    prob->memCalled = 1;

  } else {
    allocIQ(prob,500);
    allocRQ(prob,500);
  }

  prob->name   = name;

  len = strlen(prtfile);

  if (summOn != 0) {
    printf("%s",sqversion);
  }

  f_sqinitf(prtfile, len, summOn,
	    prob->iw, prob->leniw, prob->rw, prob->lenrw);
  prob->initCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void init2zeroQ(sqProblem* prob) {
  prob->name       = NULL;

  prob->memCalled  = 0;
  prob->initCalled = 0;
  prob->userWork   = 0;

  prob->sqLog      = NULL;

  prob->leniw      = 0;
  prob->lenrw      = 0;
  prob->iw         = NULL;
  prob->rw         = NULL;

  prob->leniu      = 0;
  prob->lenru      = 0;
  prob->iu         = NULL;
  prob->ru         = NULL;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void allocIQ(sqProblem* prob, int len) {
  prob->leniw = len;
  prob->iw    = malloc(sizeof(int)*len);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void allocRQ(sqProblem* prob, int len) {
  prob->lenrw = len;
  prob->rw    = malloc(sizeof(double)*len);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void reallocIQ(sqProblem* prob, int len) {
  prob->leniw = len;
  prob->iw    = (int*)realloc(prob->iw, sizeof(int)*prob->leniw);

  sqSetIntParameter(prob, (char*)"Total int workspace", prob->leniw);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void reallocRQ(sqProblem* prob, int len) {
  prob->lenrw = len;
  prob->rw = (double*)realloc(prob->rw, sizeof(double)*prob->lenrw);

  sqSetIntParameter(prob, (char*)"Total real workspace", prob->lenrw);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void sqPrintfileX(sqProblem* prob, char *prtname, int iprint) {
  int len = strlen(prtname);

  assert(prob->initCalled == 1);
  f_sqsetprint(prtname, len, iprint,
		prob->iw, prob->leniw, prob->rw, prob->lenrw);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void sqPrintfile(sqProblem* prob, char *prtname) {
  int len = strlen(prtname);

  assert(prob->initCalled == 1);
  f_sqsetprintf(prtname, len,
		prob->iw, prob->leniw, prob->rw, prob->lenrw);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int sqSpecsfileX(sqProblem* prob, char *spcname, int ispecs) {
  int inform;
  int len = strlen(spcname);

  assert(prob->initCalled == 1);
  f_sqspec(spcname, len, ispecs, &inform,
	   prob->iw, prob->leniw, prob->rw, prob->lenrw);

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int sqSpecsfile(sqProblem* prob, char *spcname) {
  int inform;
  int len = strlen(spcname);

  assert(prob->initCalled == 1);
  f_sqspecf(spcname, len, &inform,
	    prob->iw, prob->leniw, prob->rw, prob->lenrw);

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int sqSetParameter(sqProblem* prob, char stropt[]) {
  int errors, len = strlen(stropt);

  assert(prob->initCalled == 1);
  f_sqset(stropt, len, &errors,
	   prob->iw, prob->leniw, prob->rw, prob->lenrw);

  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int sqSetIntParameter(sqProblem* prob, char stropt[], int opt) {
  int errors, len = strlen(stropt);

  assert(prob->initCalled == 1);
  f_sqseti (stropt, len, opt, &errors,
	     prob->iw, prob->leniw, prob->rw, prob->lenrw);

  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int sqGetIntParameter(sqProblem* prob, char stropt[], int *opt) {
  int errors, len = strlen(stropt);

  assert(prob->initCalled == 1);
  f_sqgeti(stropt, len, opt, &errors,
	    prob->iw, prob->leniw, prob->rw, prob->lenrw);
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int sqSetRealParameter(sqProblem* prob, char stropt[], double opt) {
  int errors, len = strlen(stropt);

  assert(prob->initCalled == 1);
  f_sqsetr (stropt, len, opt, &errors,
	     prob->iw, prob->leniw, prob->rw, prob->lenrw);
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int sqGetRealParameter(sqProblem* prob, char stropt[], double *opt) {
  int errors, len = strlen(stropt);

  assert(prob->initCalled == 1);
  f_sqgetr (stropt, len, opt, &errors,
	     prob->iw, prob->leniw, prob->rw, prob->lenrw);
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void sqUserI(sqProblem* prob, int *iu, int leniu) {
  prob->iu    = iu;
  prob->leniu = leniu;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void sqUserR(sqProblem* prob, double *ru, int lenru) {
  prob->ru    = ru;
  prob->lenru = lenru;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void sqUserspace(sqProblem* prob, int *iu, int leniu, double *ru, int lenru) {
  prob->iu    = iu;
  prob->leniu = leniu;

  prob->ru    = ru;
  prob->lenru = lenru;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void sqWorkspace(sqProblem* prob, int m, int n,
		 int neA, int ncObj, int nnH) {
  int miniw, minrw, inform;

  assert(prob->initCalled == 1);

  f_sqmem(&inform, m, n, neA, ncObj, nnH,
	  &miniw, &minrw,
	  prob->iw, prob->leniw, prob->rw, prob->lenrw);

  if (miniw > prob->leniw) { reallocIQ(prob, miniw); }
  if (minrw > prob->lenrw) { reallocRQ(prob, minrw); }

  prob->memCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void sqSetLog(sqProblem* prob, isqLog sqLog) {
  prob->sqLog  = sqLog;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int solveQ(sqProblem* prob, int start, sqFunHx qpHx,
	   int m, int n, int neA, int ncObj, int nnH,
	   int iObj, double ObjAdd,
	   double *valA, int *indA, int *locA,
	   double *bl, double *bu, double *cObj,
	   int *eType, int *hs, double *x, double *pi, double *rc,
	   double* objective,
	   int* nS, int* nInf, double* sInf) {

  int i, inform, iiObj, miniw, minrw;

  assert(prob->initCalled == 1);

  if (prob->memCalled == 0) {
    sqWorkspace(prob, m, n, neA, ncObj, nnH);
  }

  for (i = 0; i < neA; i++) {
    indA[i]++;
  }
  for (i = 0; i <= n; i++) {
    locA[i]++;
  }
  iiObj = iObj+1;

  f_snkerq(start, qpHx, prob->sqLog,
	   m, n, neA, ncObj, nnH,
	   iiObj, ObjAdd, prob->name,
	   valA, indA, locA,
	   bl, bu, cObj, eType, hs, x, pi, rc,
	   &inform, nS, nInf, sInf, objective,
	   &miniw, &minrw,
	   prob->iu, prob->leniu, prob->ru, prob->lenru,
	   prob->iw, prob->leniw, prob->rw, prob->lenrw);

  for (i = 0; i < neA; i++) {
    indA[i]--;
  }
  for (i = 0; i <= n; i++) {
    locA[i]--;
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int sqopt(sqProblem* prob, int start, sqFunHx qpHx,
	  int m, int n, int neA, int ncObj, int nnH,
	  int iObj, double ObjAdd,
	  double *valA, int *indA, int *locA,
	  double *bl, double *bu, double *cObj,
	  int *eType, int *hs, double *x, double *pi, double *rc,
	  double* objective,
	  int* nS, int* nInf, double* sInf) {

  int i, inform, iiObj, miniw, minrw;

  assert(prob->initCalled == 1);

  if (prob->memCalled == 0) {
    sqWorkspace(prob, m, n, neA, ncObj, nnH);
  }

  for (i = 0; i < neA; i++) {
    indA[i]++;
  }
  for (i = 0; i <= n; i++) {
    locA[i]++;
  }
  iiObj = iObj+1;

  f_snkerq(start, qpHx, prob->sqLog,
	   m, n, neA, ncObj, nnH,
	   iiObj, ObjAdd, prob->name,
	   valA, indA, locA,
	   bl, bu, cObj, eType, hs, x, pi, rc,
	   &inform, nS, nInf, sInf, objective,
	   &miniw, &minrw,
	   prob->iu, prob->leniu, prob->ru, prob->lenru,
	   prob->iw, prob->leniw, prob->rw, prob->lenrw);

  for (i = 0; i < neA; i++) {
    indA[i]--;
  }
  for (i = 0; i <= n; i++) {
    locA[i]--;
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void deleteSQOPT(sqProblem* prob) {
  f_sqend(prob->iw, prob->leniw, prob->rw, prob->lenrw);

  if (prob->userWork == 0) {
    free(prob->iw);
    free(prob->rw);
  }

  init2zeroQ(prob);
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
