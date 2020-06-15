#ifndef SNOPTPROBLEM_H
#define SNOPTPROBLEM_H

#include "snopt.h"

/* File snoptProblem.hpp
 *   C++ interface for SNOPT and SQOPT
 *
 */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

class snoptProblem {
protected:
  // default constructor
  snoptProblem(const char*name, int *iw, int aleniw, double *rw, int alenrw);

  // delegated constructors
  snoptProblem() :
    snoptProblem("        ", 0, 0, 0, 0) {};
  snoptProblem(int aleniw, int alenrw) :
    snoptProblem("        ", 0, aleniw, 0, alenrw) {};
  snoptProblem(int *aiw, int aleniw, double *arw, int alenrw) :
    snoptProblem("        ", aiw, aleniw, arw, alenrw) {} ;
  snoptProblem(const char*name) :
    snoptProblem(name, 0, 0, 0, 0) {};
  snoptProblem(const char*name, int aleniw, int alenrw) :
    snoptProblem(name, 0, aleniw, 0, alenrw) {};

  ~snoptProblem();

  void init2zero();

  char    Prob[30];

  int     initCalled, memCalled, userWork;

  int     leniw, lenrw;
  double *rw;
  int    *iw;

  int     lenru, leniu;
  double *ru;
  int    *iu;

  void allocI    (int leniw);
  void allocR    (int lenrw);
  void reallocI  (int leniw);
  void reallocR  (int lenrw);

public:
  void setProbName    (const char *Prob);
  void setPrintFile   (const char *prtname);
  void setPrintFile   (const char *prtname, int iprint);

  int getIntParameter (const char *stropt,   int    &opt);
  int getRealParameter(const char *stropt,   double &opt);
  int setParameter    (const char *stroptin);
  int setIntParameter (const char *stropt,   int     opt);
  int setRealParameter(const char *stropt,   double  opt);

  void setUserI       (int    *iu, int leniu);
  void setUserR       (double *ru, int lenru);
  void setUserspace   (int    *iu, int leniu, double *ru, int lenru);
};

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

class snoptProblemABC : public snoptProblem  {
protected:
  // default constructor
  snoptProblemABC(const char*name, int *iw, int aleniw, double *rw, int alenrw);

  // delegated constructors
  snoptProblemABC() :
    snoptProblemABC("        ", 0, 0, 0, 0) {};
  snoptProblemABC(const char*name) :
    snoptProblemABC(name, 0, 0, 0, 0) {};
  snoptProblemABC(int aleniw, int alenrw) :
    snoptProblemABC("        ", 0, aleniw, 0, alenrw) {};
  snoptProblemABC(int *aiw, int aleniw, double *arw, int alenrw) :
    snoptProblemABC("        ", aiw, aleniw, arw, alenrw) {};
  snoptProblemABC(const char*name, int aleniw, int alenrw) :
    snoptProblemABC(name, 0, aleniw, 0, alenrw) {};

  ~snoptProblemABC();

  void init2zero();

  isnLog  snLog;
  isnLog2 snLog2;
  isqLog  sqLog;
  isnSTOP snSTOP;

public:
  void initialize     (const char *prtfile, int summOn);
  void initialize     (const char *prtfile, int iprint, const char *sumfile, int isumm);

  int  setSpecsFile   (const char *specname);
  int  setSpecsFile   (const char *specname, int ispecs);

  void setLog         (isnLog snLog, isnLog2 snLog2, isqLog sqLog);
  void setSTOP        (isnSTOP snSTOP);
};

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

class snoptProblemA : public snoptProblemABC {
public:
  // default constructor
  snoptProblemA(const char*name, int *iw, int aleniw, double *rw, int alenrw);

  // delegated constructors
  snoptProblemA() :
    snoptProblemA("        ", 0, 0, 0, 0) {};
  snoptProblemA(const char*name) :
    snoptProblemA(name, 0, 0, 0, 0) {};
  snoptProblemA(int aleniw, int alenrw) :
    snoptProblemA("        ", 0, aleniw, 0, alenrw) {};
  snoptProblemA(int *aiw, int aleniw, double *arw, int alenrw) :
    snoptProblemA("        ", aiw, aleniw, arw, alenrw) {};
  snoptProblemA(const char*name, int aleniw, int alenrw) :
    snoptProblemA(name, 0, aleniw, 0, alenrw) {};

  ~snoptProblemA();

  void setWorkspace(int neF, int n, int neA, int neG);
  int  computeJac(int neF, int n, snFunA usrfunA,
		  double *x, double *xlow, double*xupp,
		  int *&iAfun, int *&jAvar, double *&A, int &neA,
		  int *&iGfun, int *&jGvar, int &neG);

  int  solve(int starttype, int nF, int n, double ObjAdd,
	     int ObjRow, snFunA usrfunA,
	     double *xlow, double *xupp, double *Flow, double *Fupp,
	     double *x, int *xstate, double *xmul,
	     double *F, int *Fstate, double *Fmul,
	     int &nS, int &nInf, double &sInf);

  int  solve(int starttype, int nF, int n, double ObjAdd,
	     int ObjRow, snFunA usrfunA,
	     int *iAfun, int *jAvar, double *A, int neA,
	     int *iGfun, int *jGvar, int neG,
	     double *xlow, double *xupp, double *Flow, double *Fupp,
	     double *x, int *xstate, double *xmul,
	     double *F, int *Fstate, double *Fmul,
	     int &nS, int &nInf, double &sInf);
};

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

class snoptProblemC : public snoptProblemABC {
public:
  // default constructor
  snoptProblemC(const char*name, int *iw, int aleniw, double *rw, int alenrw);

  // delegated constructors
  snoptProblemC() :
    snoptProblemC("        ", 0, 0, 0, 0) {};
  snoptProblemC(const char*name) :
    snoptProblemC(name, 0, 0, 0, 0) {};
  snoptProblemC(int aleniw, int alenrw) :
    snoptProblemC("        ", 0, aleniw, 0, alenrw) {};
  snoptProblemC(int *aiw, int aleniw, double *arw, int alenrw) :
    snoptProblemC("        ", aiw, aleniw, arw, alenrw) {};
  snoptProblemC(const char*name, int aleniw, int alenrw) :
    snoptProblemC(name, 0, aleniw, 0, alenrw) {};

  ~snoptProblemC();

  void setWorkspace(int m, int n, int ne, int negCon,
		    int nnCon, int nnObj, int nnJac);

  int solve(int starttype, int m, int n, int ne, int negCon,
	    int nnCon, int nnObj, int nnJac,
	    int iObj, double ObjAdd, snFunC usrfunC,
	    double *Jval, int *indJ, int *locJ,
	    double *bl, double *bu, int *hs,
	    double *x, double *pi, double *rc,
	    int &nS, int &nInf, double &sInf, double &objective);
};

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

class snoptProblemB : public snoptProblemC {
public:
  // default constructor
  snoptProblemB(const char*name, int *aiw, int aleniw, double *arw, int alenrw);

  // delegated constructors
  snoptProblemB() :
    snoptProblemB("        ", 0, 0, 0, 0) {};
  snoptProblemB(const char*name) :
    snoptProblemB(name, 0, 0, 0, 0) {};
  snoptProblemB(int aleniw, int alenrw) :
    snoptProblemB("        ", 0, aleniw, 0, alenrw) {};
  snoptProblemB(int *aiw, int aleniw, double *arw, int alenrw) :
    snoptProblemB("        ", aiw, aleniw, arw, alenrw) {};
  snoptProblemB(const char*name, int aleniw, int alenrw) :
    snoptProblemB(name, 0, aleniw, 0, alenrw) {};

  ~snoptProblemB();

  int solve(int starttype, int m, int n, int ne, int negCon,
	    int nnCon, int nnObj, int nnJac,
	    int iObj, double ObjAdd,
	    snConB funcon, snObjB funobj,
	    double *Jval, int *indJ, int *locJ,
	    double *bl, double *bu, int *hs,
	    double *x, double *pi, double *rc,
	    int &nS, int &nInf, double &sInf, double &objective);
};

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

class sqoptProblem : public snoptProblem {
private:
  isqLog  sqLog;
  void init2zero();

public:
  // default constructor
  sqoptProblem(const char*name, int *iw, int aleniw, double *rw, int alenrw);

  // delegated constructors
  sqoptProblem() :
    sqoptProblem("        ", 0, 0, 0, 0) {};
  sqoptProblem(const char*name) :
    sqoptProblem(name, 0, 0, 0, 0) {};
  sqoptProblem(int aleniw, int alenrw) :
    sqoptProblem("        ", 0, aleniw, 0, alenrw) {};
  sqoptProblem(int *aiw, int aleniw, double *arw, int alenrw) :
    sqoptProblem("        ", aiw, aleniw, arw, alenrw) {};
  sqoptProblem(const char*name, int aleniw, int alenrw) :
    sqoptProblem(name, 0, aleniw, 0, alenrw) {};

  ~sqoptProblem();

  void initialize  (const char *prtfile, int summOn);
  void initialize  (const char *prtfile, int iprint, const char *sumfile, int isumm);

  int  setSpecsFile(const char *specname);
  int  setSpecsFile(const char *specname, int ispecs);

  void setLog      (isqLog sqLog);
  void setWorkspace(int m, int n, int neA, int ncObj, int nnH);

  int solve(int starttype, sqFunHx qpHx,
	    int m, int n, int neA, int ncObj, int nnH,
	    int iObj, double ObjAdd,
	    double *A, int *indA, int *locA,
	    double *bl, double *bu, double *cObj,
	    int *eType, int *hs, double *x, double *pi, double *rc,
	    int &nS, int &nInf, double &sInf, double &objective);
};

#endif /* SNOPTPROBLEM_H */
