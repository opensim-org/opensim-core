#include "snopt_cwrap.h"

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void snInit ( snProblem* prob, char* name, char* prtfile, int iprint, int isumm )
{
  int leniw, lenrw, len;

  init2zero ( prob );

  leniw = 500;
  lenrw = 500;

  allocI( prob, leniw );
  allocR( prob, lenrw );

  prob->iprint = iprint;
  prob->isumm  = isumm;

  prob->name   = name;

  len = strlen(prtfile);

  f_sninit ( prtfile, len, prob->iprint, prob->isumm,
	     prob->iw, prob->leniw,
	     prob->rw, prob->lenrw );
  prob->initCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void init2zero ( snProblem* prob )
{
  prob->name      = NULL;

  prob->iprint     = 0;
  prob->isumm      = 6;
  prob->memCalled  = 0;
  prob->initCalled = 0;
  prob->sizeCalled = 0;

  prob->m         =  0;
  prob->n         =  0;
  prob->ne        =  0;
  prob->negCon    = -1;
  prob->nnCon     =  0;
  prob->nnObj     =  0;
  prob->nnJac     =  0;
  prob->iObj      = -1;
  prob->ObjAdd    =  0;

  prob->valJ      = NULL;
  prob->indJ      = NULL;
  prob->locJ      = NULL;

  prob->bl        = NULL;
  prob->bu        = NULL;
  prob->x         = NULL;
  prob->pi        = NULL;
  prob->rc        = NULL;

  prob->usrfun    = NULL;
  prob->funobj    = NULL;
  prob->funcon    = NULL;

  prob->snLog     = NULL;
  prob->snLog2    = NULL;
  prob->sqLog     = NULL;
  prob->snSTOP    = NULL;

  prob->leniw     = 0;
  prob->lenrw     = 0;
  prob->iw        = NULL;
  prob->rw        = NULL;

  prob->leniu     = 0;
  prob->lenru     = 0;
  prob->iu        = NULL;
  prob->ru        = NULL;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void allocI ( snProblem* prob, int len )
{
  prob->leniw = len;
  prob->iw    = malloc( sizeof(int)*len );
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void allocR ( snProblem* prob, int len )
{
  prob->lenrw = len;
  prob->rw    = malloc( sizeof(double)*len );
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void reallocI ( snProblem* prob, int len )
{
  prob->leniw = len;
  prob->iw    = (int*)realloc( prob->iw, sizeof(int)*prob->leniw );

  setIntParameter(prob, (char*)"Total int workspace", prob->leniw );
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void reallocR ( snProblem* prob, int len )
{
  prob->lenrw = len;
  prob->rw = (double*)realloc( prob->rw, sizeof(double)*prob->lenrw );

  setIntParameter(prob, (char*)"Total real workspace", prob->lenrw );
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setProbName ( snProblem* prob, char *name )
{
  prob->name = name;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setPrintfile ( snProblem* prob, char *prtname )
{
  int len = strlen(prtname);

  assert( prob->initCalled == 1 );
  f_snsetprint( prtname, len, prob->iprint,
		prob->iw, prob->leniw, prob->rw, prob->lenrw );
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int setSpecsfile ( snProblem* prob, char *spcname )
{
  int inform;
  int len = strlen(spcname);

  assert( prob->initCalled == 1 );
  f_snspec( spcname, len, &inform,
	    prob->iw, prob->leniw, prob->rw, prob->lenrw );

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int setParameter ( snProblem* prob, char stropt[] )
{
  int errors, len = strlen(stropt);

  assert( prob->initCalled == 1 );
  f_snset( stropt, len, &errors,
	   prob->iw, prob->leniw, prob->rw, prob->lenrw );

  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int getParameter ( snProblem* prob, char stropt[], char strout[] )
{
  int errors;
  int inlen  = strlen(stropt);
  int outlen = strlen(strout);

  assert( prob->initCalled == 1 );
  f_sngetc( stropt, inlen, strout, outlen, &errors,
	    prob->iw, prob->leniw, prob->rw, prob->lenrw );
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int setIntParameter ( snProblem* prob, char stropt[], int opt )
{
  int errors, len = strlen(stropt);

  assert( prob->initCalled == 1 );
  f_snseti ( stropt, len, opt, &errors,
	     prob->iw, prob->leniw, prob->rw, prob->lenrw );

  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int getIntParameter ( snProblem* prob, char stropt[], int opt )
{
  int errors, len = strlen(stropt);

  assert( prob->initCalled == 1 );
  f_sngeti( stropt, len, &opt, &errors,
	    prob->iw, prob->leniw, prob->rw, prob->lenrw );
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int setRealParameter ( snProblem* prob, char stropt[], double opt )
{
  int errors, len = strlen(stropt);

  assert( prob->initCalled == 1 );
  f_snsetr ( stropt, len, opt, &errors,
	     prob->iw, prob->leniw, prob->rw, prob->lenrw );
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int getRealParameter ( snProblem* prob, char stropt[], double opt )
{
  int errors, len = strlen(stropt);

  assert( prob->initCalled == 1 );
  f_sngetr ( stropt, len, &opt, &errors,
	     prob->iw, prob->leniw, prob->rw, prob->lenrw );
  return errors;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setUserI ( snProblem* prob, int *iu, int leniu )
{
  prob->iu    = iu;
  prob->leniu = leniu;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setUserR ( snProblem* prob, double *ru, int lenru )
{
  prob->ru    = ru;
  prob->lenru = lenru;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setUserspace ( snProblem* prob, int *iu, int leniu,
		    double *ru, int lenru )
{
  prob->iu    = iu;
  prob->leniu = leniu;

  prob->ru    = ru;
  prob->lenru = lenru;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setWorkspace( snProblem* prob )
{
  int miniw, minrw, inform;
  int memGuess = 0;

  assert( prob->initCalled == 1 );
  assert( prob->sizeCalled == 1 );

  if ( prob->negCon < 0 ) {
    prob->negCon = prob->nnCon*prob->nnJac;
    memGuess = 1;
  }

  f_snmem ( &inform, prob->m, prob->n, prob->ne,
	    prob->negCon, prob->nnCon, prob->nnObj,
	    prob->nnJac, &miniw, &minrw,
	    prob->iw, prob->leniw, prob->rw, prob->lenrw );

  if ( miniw > prob->leniw ) { reallocI(prob, miniw); }
  if ( minrw > prob->lenrw ) { reallocR(prob, minrw); }

  prob->memCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setProblemSize ( snProblem* prob, int m, int n, int ne,
		      int nnCon, int nnJac, int nnObj )
{
  int nb = n + m;

  prob->m      =  m;
  prob->n      =  n;
  prob->ne     =  ne;
  prob->nnCon  = nnCon;
  prob->nnObj  = nnObj;
  prob->nnJac  = nnJac;

  prob->bl = calloc( nb, sizeof(double) );
  prob->bu = calloc( nb, sizeof(double) );

  prob->hs = calloc( nb, sizeof(int   ) );
  prob->x  = calloc( nb, sizeof(double) );
  prob->pi = calloc(  m, sizeof(double) );
  prob->rc = calloc( nb, sizeof(double) );

  prob->indJ = calloc(  ne, sizeof(int   ) );
  prob->locJ = calloc( n+1, sizeof(int   ) );
  prob->valJ = calloc(  ne, sizeof(double) );

  prob->sizeCalled = 1;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setProblemData( snProblem *prob, double *bl, double *bu, int *hs,
		     double *x, int *indJ, int *locJ, double *valJ ) {

  int i;
  assert( prob->sizeCalled == 1 );

  // Copy problem data.
  for (i = 0; i < prob->m+prob->n; i++) {
    prob->bl[i] = bl[i];
    prob->bu[i] = bu[i];
    prob->x[i]  =  x[i];
    prob->hs[i] = hs[i];
  }

  for (i = 0; i < prob->ne; i++) {
    prob->indJ[i] = indJ[i];
    prob->valJ[i] = valJ[i];
  }

  for (i = 0; i < prob->n+1; i++) {
    prob->locJ[i] = locJ[i];
  }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setObjective ( snProblem* prob, int iObj, double ObjAdd)
{
  prob->iObj   = iObj;
  prob->ObjAdd = ObjAdd;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setUserfun ( snProblem* prob, snFunC func )
{
  prob->usrfun = func;
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setFuncon ( snProblem* prob, snConB func )
{
  prob->funcon = func;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setFunobj ( snProblem* prob, snObjB func )
{
  prob->funobj = func;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setLog ( snProblem* prob, isnLog snLog, isnLog2 snLog2, isqLog sqLog )
{
  prob->snLog  = snLog;
  prob->snLog2 = snLog2;
  prob->sqLog  = sqLog;
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void setSTOP ( snProblem* prob, isnSTOP snSTOP )
{
  prob->snSTOP = snSTOP;
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int solveB ( snProblem* prob, int start, double* objective )
{
  int i, inform, iObj, miniw, minrw, nInf, nS;
  double sInf;

  assert( prob->initCalled == 1 );
  assert( prob->sizeCalled == 1 );
  assert( prob->funcon     != NULL);
  assert( prob->funobj     != NULL);

  if ( prob->memCalled == 0 ) { setWorkspace ( prob ); }

  for ( i = 0; i < prob->ne; i++ ) {
    prob->indJ[i]++;
  }
  for ( i = 0; i <= prob->n; i++ ) {
    prob->locJ[i]++;
  }

  iObj = prob->iObj+1;

  f_snkerb ( start, prob->name, prob->m, prob->n, prob->ne,
	     prob->nnCon, prob->nnObj, prob->nnJac, iObj,
	     prob->ObjAdd,
	     prob->funcon, prob->funobj,
	     prob->snLog, prob->snLog2, prob->sqLog, prob->snSTOP,
	     prob->valJ, prob->indJ, prob->locJ,
	     prob->bl, prob->bu, prob->hs, prob->x, prob->pi, prob->rc,
	     &inform, &nS, &nInf, &sInf, objective,
	     &miniw, &minrw,
	     prob->iu, prob->leniu, prob->ru, prob->lenru,
	     prob->iw, prob->leniw, prob->rw, prob->lenrw );

  for ( i = 0; i < prob->ne; i++ ) {
    prob->indJ[i]--;
  }
  for ( i = 0; i <= prob->n; i++ ) {
    prob->locJ[i]--;
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

int solveC ( snProblem* prob, int start, double* objective )
{
  int i, inform, iObj, miniw, minrw, nInf, nS;
  double sInf;

  assert( prob->initCalled == 1 );
  assert( prob->sizeCalled == 1 );
  assert( prob->usrfun     != NULL);


  if ( prob->memCalled == 0 ) { setWorkspace ( prob ); }

  for ( i = 0; i < prob->ne; i++ ) {
    prob->indJ[i]++;
  }
  for ( i = 0; i <= prob->n; i++ ) {
    prob->locJ[i]++;
  }
  iObj = prob->iObj+1;

  f_snkerc( start, prob->name, prob->m, prob->n, prob->ne,
	    prob->nnCon, prob->nnObj, prob->nnJac, iObj,
	    prob->ObjAdd,
	    prob->usrfun,
	    prob->snLog, prob->snLog2, prob->sqLog, prob->snSTOP,
	    prob->valJ, prob->indJ, prob->locJ,
	    prob->bl, prob->bu, prob->hs, prob->x, prob->pi, prob->rc,
	    &inform, &nS, &nInf, &sInf, objective,
	    &miniw, &minrw,
	    prob->iu, prob->leniu, prob->ru, prob->lenru,
	    prob->iw, prob->leniw, prob->rw, prob->lenrw );

  for ( i = 0; i < prob->ne; i++ ) {
    prob->indJ[i]--;
  }
  for ( i = 0; i <= prob->n; i++ ) {
    prob->locJ[i]--;
  }

  return inform;
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

void deleteSNOPT ( snProblem* prob )
{
  f_snend ( prob->iprint );

  free(prob->iw);
  free(prob->rw);

  free(prob->locJ);
  free(prob->indJ);
  free(prob->valJ);

  free(prob->hs);
  free(prob->bl);
  free(prob->bu);
  free(prob->x);
  free(prob->pi);
  free(prob->rc);

  init2zero( prob );
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
