// SdfastStumps.c
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "sdfast.h"

//=============================================================================
// STATICS
//=============================================================================

void init_sdm() { }

void sdinit() { }

void sdinfo(int aInfo[]) { }

void sdderiv(double *dqdt,double *dudt) { }

void sdgetmass(int aBody,double *rMass) { }

void sdmass(int aBody,double aMass) { }

void sdiner(int aBody,double aInertia[3][3]) { }

void sdgetiner(int aBody,double rInertia[3][3]) { }

void sdgetbtj(int joint,double btjout[3]) { }

void sdgetitj(int joint,double itjout[3]) { }

void sdpos(int aBody,double aPoint[3],double rPos[3]) { }

void sdvel(int aBody,double aPoint[3],double rVel[3]) { }

void sdacc(int aBody,double aPoint[3],double rAcc[3]) { }

void sdorient(int aBody,double rDirCos[3][3]) { }

void sdassemble(double time,double *state,int *lock,double tol,
					 int maxevals,int *fcnt,int *err ) { }

void sdinitvel(double time, double state[100], int lock[50], double tol,
					int maxevals, int *fcnt, int *err) { }

void sdpres(int joint, int axis, int presin) { }

void sdgetpres(int joint, int axis, int *presout) { }

void sdstate(double timein, double qin[], double uin[]) { }

void sdtrans(int frbod, double ivec[], int tobod, double ovec[]) { }

void sdpointf(int body, double point[], double force[]) { }

void sddc2ang(double dircos[][3], double *a1, double *a2, double *a3) { }

void sdangvel(int body, double avel[]) { }

void sdangacc(int body, double aacc[]) { }

void sdbodyt(int body, double torque[]) { }

void sdhinget(int joint, int axis, double torque) { }

void sdgetht(int joint, int axis, double *torque) { }

void sdcomptrq(double udotin[], double trqout[]) { }

void sdreac(double force[][3], double torque[][3]) { }

void sdmassmat(double *rI) {}

void sdst2ang(double *q, double *qang) { }

void sdang2st(double *qang, double *q) { }

void sdang2dc(double aE1, double aE2, double aE3, double rDirCos[3][3]) { }

void sddc2quat(double aDirCos[3][3], double *rQ1, double *rQ2, double *rQ3, double *rQ4) { }

void sdquat2dc(double aQ1, double aQ2, double aQ3, double aQ4, double rDirCos[3][3]) { }

void sdbtj(int body,double btj[3]) { }

void sditj(int body,double itj[3]) { }

void compute_constrained_coords(double *y) { }

void sdjnt(int joint, int info[50], int tran[6]) { }

void sdgrav(double gravin[3]) { }

void scaleConstraints(int SdfastBodyIndex, double scaleFactor[3]) { }

void sdgetgrav(double gravout[3]) { }
