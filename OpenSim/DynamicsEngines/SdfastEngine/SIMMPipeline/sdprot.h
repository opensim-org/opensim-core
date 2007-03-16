#ifndef __sdprot_h__
#define __sdprot_h__

// sdprot.h -- function prototypes for sdlib.c and <model>_dyn/sar
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

void sdseterr(int routine, int errnum);
void sdserialno(int *serno);
//void sdldudcomp(int n, int na, int map[], double tol, double ld[], double sum[], double m[], double l[], double d[]);
void sdfsmult();
void sdvcross(double ivec1[], double ivec2[], double ovec[]);
void sdang2st(double stang[], double st[]);
void sdstate(double timein, double qin[], double uin[]);
void sdgentime(int *gentm);
void sdgetmass(int body, double *massout);
void sdgetiner(int body, double inerout[3][3]);
int sdchkjaxis(int routine, int jnum, int axnum);
int sdchkjpin(int routine, int jnum, int pinno);
int sdchkbnum(int routine, int num);
int sdchkjnum(int routine, int jnum);
int sdchkucnum(int routine, int ucnum);
void sdperr(double errs[]);
void sdverr(double errs[]);
void sdderiv(double oqdot[], double oudot[]);
void sdst2ang(double st[], double stang[]);
void sdbtj(int joint, double btjin[]);
//void sdvinteg(int (*func)(), double *time, double st[], double dst[], double param[], double dt, double *step, int neqin, double tol, double work[], int *err, int *which);
//void sdfinteg(int (*func)(), double *time, double st[], double dst[], double param[], double step,  int neq, double work[], double *errest, int *status);
//void sdroot(int (*func)(), double vars[], double param[], int nfunc, int nvar, int ndesin, int lock[], double rtol, double dtol, int maxeval, double jw[], double dw[], double rw[], int iw[], double fret[], int *fcnt, int *err);
//void sdldubsl(int n, int na, int map[], double l[], double b[], double x[]);
void sdldubsd(int n, int na, int map[], double d[], double b[], double x[]);
//void sdldubslv(int n, int na, int map[], double work[], double l[], double d[], double b[], double x[]);
//void sdqrdcomp(int nr, int nc, int nra, int nca, int mapr[], int mapc[], double w[], double qraux[], int jpvt[]);
//void sdqrbslv(int nr, int nc, int nra, int nca, int mapr[], int mapc[], double tol, double work[], int iwork[], double w[], double qraux[], int jpvt[], double b[], double x[], int *rank);
void sdvcopy(double ivec[], double vec[]);
void sdvset(double sclr1, double sclr2, double sclr3, double ovec[]);
void sdvadd(double ivec1[], double ivec2[], double ovec[]);

// FUNCTIONS DON'T EXIST!!
#if 1
void sduperr(double t, double q[], double errors[]);
void sduverr(double t, double q[], double u[], double errors[]);
void sduaerr(double t, double q[], double u[], double udot[], double errors[]);
void sduconsfrc(double t, double q[], double u[], double mults[]);
int sdumotion(double t, double q[], double u[]);
int sduforce(double t, double q[], double u[]);
#endif

#endif
