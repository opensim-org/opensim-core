/*
Generated 07-Aug-2006 15:55:31 by SD/FAST, Kane's formulation
(sdfast B.2.8 #30123) on machine ID unknown
Copyright (c) 1990-1997 Symbolic Dynamics, Inc.
Copyright (c) 1990-1997 Parametric Technology Corp.
RESTRICTED RIGHTS LEGEND: Use, duplication, or disclosure by the U.S.
Government is subject to restrictions as set forth in subparagraph
(c)(1)(ii) of the Rights in Technical Data and Computer Software
clause at DFARS 52.227-7013 and similar clauses in the FAR and NASA
FAR Supplement.  Symbolic Dynamics, Inc., Mountain View, CA 94041
*/
#include <math.h>

/* These routines are passed to sdroot. */

void sdposfunc(double vars[50],
    double param[1],
    double resid[50])
{
    int i;
    double pos[50],vel[50];

    for (i = 0; i < 50; i++) {
        vel[i] = 0.;
    }
    sdang2st(vars,pos);
    sdstate(param[0],pos,vel);
    sdumotion(param[0],pos,vel);
    sdperr(resid);
}

void sdvelfunc(double vars[50],
    double param[51],
    double resid[50])
{

    sdstate(param[50],param,vars);
    sdumotion(param[50],param,vars);
    sdverr(resid);
}

void sdstatfunc(double vars[50],
    double param[51],
    double resid[100])
{
    double pos[50],qdotdum[50];

    sdang2st(vars,pos);
    sdstate(param[50],pos,param);
    sdumotion(param[50],pos,param);
    sduforce(param[50],pos,param);
    sdperr(resid);
    sdderiv(qdotdum,&resid[50]);
}

void sdstdyfunc(double vars[100],
    double param[1],
    double resid[150])
{
    double pos[50],qdotdum[50];

    sdang2st(vars,pos);
    sdstate(param[0],pos,&vars[50]);
    sdumotion(param[0],pos,&vars[50]);
    sduforce(param[0],pos,&vars[50]);
    sdperr(resid);
    sdverr(&resid[50]);
    sdderiv(qdotdum,&resid[100]);
}

/* This routine is passed to the integrator. */

void sdmotfunc(double time,
    double state[100],
    double dstate[100],
    double param[1],
    int *status)
{
    double err[50];
    int i;

    sdstate(time,state,&state[50]);
    sdumotion(time,state,&state[50]);
    sduforce(time,state,&state[50]);
    sdderiv(dstate,&dstate[50]);
    *status = 1;
    sdverr(err);
    for (i = 0; i < 50; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    sdperr(err);
    for (i = 0; i < 50; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    *status = 0;
}

/* This routine performs assembly analysis. */

void sdassemble(double time,
    double state[100],
    int lock[50],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double perrs[50],param[1];
    int i;
    double jw[2500],dw[20000],rw[800];
    int iw[400],rooterr;

    sdgentime(&i);
    if (i != 155528) {
        sdseterr(50,42);
    }
    param[0] = time;
    sdroot(sdposfunc,state,param,50,50,0,lock,tol,tol,maxevals,
      jw,dw,rw,iw,perrs,fcnt,&rooterr);
    sdposfunc(state,param,perrs);
    *fcnt = *fcnt+1;
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs initial velocity analysis. */

void sdinitvel(double time,
    double state[100],
    int lock[50],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double verrs[50],param[51];
    int i;
    double jw[2500],dw[20000],rw[800];
    int iw[400],rooterr;

    sdgentime(&i);
    if (i != 155528) {
        sdseterr(51,42);
    }
    for (i = 0; i < 50; i++) {
        param[i] = state[i];
    }
    param[50] = time;
    sdroot(sdvelfunc,&state[50],param,50,50,0,lock,tol,tol,maxevals,
      jw,dw,rw,iw,verrs,fcnt,&rooterr);
    sdvelfunc(&state[50],param,verrs);
    *fcnt = *fcnt+1;
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs static analysis. */

void sdstatic(double time,
    double state[100],
    int lock[50],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[100],param[51],jw[5000],dw[45000],rw[1150];
    int iw[600],rooterr,i;

    sdgentime(&i);
    if (i != 155528) {
        sdseterr(52,42);
    }
    for (i = 0; i < 50; i++) {
        param[i] = state[50+i];
    }
    param[50] = time;
    sdroot(sdstatfunc,state,param,100,50,50,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    sdstatfunc(state,param,resid);
    *fcnt = *fcnt+1;
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs steady motion analysis. */

void sdsteady(double time,
    double state[100],
    int lock[100],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[150],param[1];
    double jw[15000],dw[125000],rw[1950];
    int iw[1000],rooterr,i;

    sdgentime(&i);
    if (i != 155528) {
        sdseterr(53,42);
    }
    param[0] = time;
    sdroot(sdstdyfunc,state,param,150,100,50,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    sdstdyfunc(state,param,resid);
    *fcnt = *fcnt+1;
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs state integration. */

void sdmotion(double *time,
    double state[100],
    double dstate[100],
    double dt,
    double ctol,
    double tol,
    int *flag,
    int *err)
{
    static double step;
    double work[600],ttime,param[1];
    int vintgerr,which,ferr,i;

    sdgentime(&i);
    if (i != 155528) {
        sdseterr(54,42);
    }
    param[0] = ctol;
    ttime = *time;
    if (*flag != 0) {
        sdmotfunc(ttime,state,dstate,param,&ferr);
        step = dt;
        *flag = 0;
    }
    if (step <= 0.) {
        step = dt;
    }
    sdvinteg(sdmotfunc,&ttime,state,dstate,param,dt,&step,100,tol,work,&
      vintgerr,&which);
    *time = ttime;
    *err = vintgerr;
}

/* This routine performs state integration with a fixed-step integrator. */

void sdfmotion(double *time,
    double state[100],
    double dstate[100],
    double dt,
    double ctol,
    int *flag,
    double *errest,
    int *err)
{
    double work[400],ttime,param[1];
    int ferr,i;

    sdgentime(&i);
    if (i != 155528) {
        sdseterr(55,42);
    }
    param[0] = ctol;
    *err = 0;
    ttime = *time;
    if (*flag != 0) {
        sdmotfunc(ttime,state,dstate,param,&ferr);
        *flag = 0;
    }
    sdfinteg(sdmotfunc,&ttime,state,dstate,param,dt,100,work,errest,&ferr);
    if (ferr != 0) {
        *err = 1;
    }
    *time = ttime;
}
