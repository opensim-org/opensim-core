#ifndef SNOPT_C_H
#define SNOPT_C_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "snopt.h"

/* File: snopt_c.h
 *   C interface for SNOPT.
 *
 * 10 Jul 2014: First version (based on cwrap).
 */
typedef struct {
  char   *name;

  int    iprint;
  int    isumm;
  int    memCalled;
  int    initCalled;
  int    sizeCalled;

  int    m;
  int    n;
  int    ne;
  int    negCon;
  int    nnCon;
  int    nnObj;
  int    nnJac;
  int    iObj;
  double ObjAdd;

  double *valJ;
  int    *indJ;
  int    *locJ;

  double *bl;
  double *bu;
  int    *hs;
  double  *x;
  double *pi;
  double *rc;

  snFunC usrfun;
  snObjB funobj;
  snConB funcon;

  isnSTOP snSTOP;
  isnLog  snLog;
  isnLog2 snLog2;
  isqLog  sqLog;

  int    lenrw, leniw;
  int    *iw;
  double *rw;

  int    lenru, leniu;
  int    *iu;
  double *ru;

} snProblem;

void snInit         ( snProblem* prob, char* name, char* prtfile, int iprint, int isumm );
void init2zero      ( snProblem* prob );

void allocI         ( snProblem* prob, int len );
void allocR         ( snProblem* prob, int len );
void reallocI       ( snProblem* prob, int len );
void reallocR       ( snProblem* prob, int len );

void setProbName    ( snProblem* prob, char* name );
void setPrintfile   ( snProblem* prob, char* prtname );
int  setSpecsfile   ( snProblem* prob, char* spcname );

int setParameter    ( snProblem* prob, char stropt[] );
int getParameter    ( snProblem* prob, char stropt[], char strout[] );
int setIntParameter ( snProblem* prob, char stropt[], int opt );
int getIntParameter ( snProblem* prob, char stropt[], int opt );
int setRealParameter( snProblem* prob, char stropt[], double opt );
int getRealParameter( snProblem* prob, char stropt[], double opt );

void setUserI       ( snProblem* prob, int *iu, int leniu );
void setUserR       ( snProblem* prob, double *ru, int lenru );
void setUserspace   ( snProblem* prob, int *iu, int leniu,
		      double *ru, int lenru );

void setLog         ( snProblem* prob, isnLog snLog, isnLog2 snLog2, isqLog sqLog );
void setSTOP        ( snProblem* prob, isnSTOP snSTOP );

void setWorkspace   ( snProblem* prob );

void setProblemSize ( snProblem* prob, int m, int n, int ne,
		      int nnCon, int nnJac, int nnObj );
void setObjective   ( snProblem* prob, int iObj, double ObjAdd);

void setProblemData( snProblem *prob, double *bl, double *bu,
		     int *hs, double *x, int *indJ, int *locJ, double *valJ );

void setUserfun     ( snProblem* prob, snFunC func );
void setFuncon      ( snProblem* prob, snConB func );
void setFunobj      ( snProblem* prob, snObjB func );

int solveB          ( snProblem* prob, int start, double *objective );
int solveC          ( snProblem* prob, int start, double *objective );

void deleteSNOPT    ( snProblem* prob );

#endif /* SNOPT_C_H */
