/*
Generated 23-Jul-2009 11:00:02 by SD/FAST, Order(N) formulation
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

/* These routines are passed to gymnistroot. */

void gymnistposfunc(double vars[54],
    double param[1],
    double resid[1])
{
    int i;
    double pos[55],vel[54];

    for (i = 0; i < 54; i++) {
        vel[i] = 0.;
    }
    gymnistang2st(vars,pos);
    gymniststate(param[0],pos,vel);
    gymnistperr(resid);
}

void gymnistvelfunc(double vars[54],
    double param[56],
    double resid[1])
{

    gymniststate(param[55],param,vars);
    gymnistverr(resid);
}

void gymniststatfunc(double vars[54],
    double param[55],
    double resid[54])
{
    double pos[55],qdotdum[55];

    gymnistang2st(vars,pos);
    gymniststate(param[54],pos,param);
    gymnistuforce(param[54],pos,param);
    gymnistperr(resid);
    gymnistderiv(qdotdum,&resid[0]);
}

void gymniststdyfunc(double vars[108],
    double param[1],
    double resid[54])
{
    double pos[55],qdotdum[55];

    gymnistang2st(vars,pos);
    gymniststate(param[0],pos,&vars[54]);
    gymnistuforce(param[0],pos,&vars[54]);
    gymnistperr(resid);
    gymnistverr(&resid[0]);
    gymnistderiv(qdotdum,&resid[0]);
}

/* This routine is passed to the integrator. */

void gymnistmotfunc(double time,
    double state[109],
    double dstate[109],
    double param[1],
    int *status)
{

    gymniststate(time,state,&state[55]);
    gymnistuforce(time,state,&state[55]);
    gymnistderiv(dstate,&dstate[55]);
    *status = 0;
}

/* This routine performs assembly analysis. */

void gymnistassemble(double time,
    double state[109],
    int lock[54],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double perrs[1],param[1];
    int i;

    gymnistgentime(&i);
    if (i != 110000) {
        gymnistseterr(50,42);
    }
    param[0] = time;
    gymnistst2ang(state,state);
    *err = 0;
    *fcnt = 0;
    gymnistposfunc(state,param,perrs);
    *fcnt = *fcnt+1;
    gymnistang2st(state,state);
}

/* This routine performs initial velocity analysis. */

void gymnistinitvel(double time,
    double state[109],
    int lock[54],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double verrs[1],param[56];
    int i;

    gymnistgentime(&i);
    if (i != 110000) {
        gymnistseterr(51,42);
    }
    for (i = 0; i < 55; i++) {
        param[i] = state[i];
    }
    param[55] = time;
    *err = 0;
    *fcnt = 0;
    gymnistvelfunc(&state[55],param,verrs);
    *fcnt = *fcnt+1;
}

/* This routine performs static analysis. */

void gymniststatic(double time,
    double state[109],
    int lock[54],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[54],param[55],jw[2916],dw[23328],rw[864];
    int iw[432],rooterr,i;

    gymnistgentime(&i);
    if (i != 110000) {
        gymnistseterr(52,42);
    }
    for (i = 0; i < 54; i++) {
        param[i] = state[55+i];
    }
    param[54] = time;
    gymnistst2ang(state,state);
    gymnistroot(gymniststatfunc,state,param,54,54,54,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    gymniststatfunc(state,param,resid);
    *fcnt = *fcnt+1;
    gymnistang2st(state,state);
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

void gymniststeady(double time,
    double state[109],
    int lock[108],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[54],param[1],vars[108];
    double jw[5832],dw[52488],rw[1350];
    int iw[648],rooterr,i;

    gymnistgentime(&i);
    if (i != 110000) {
        gymnistseterr(53,42);
    }
    param[0] = time;
    gymnistst2ang(state,vars);
    for (i = 0; i < 54; i++) {
        vars[54+i] = state[55+i];
    }
    gymnistroot(gymniststdyfunc,vars,param,54,108,54,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    gymniststdyfunc(vars,param,resid);
    *fcnt = *fcnt+1;
    gymnistang2st(vars,state);
    for (i = 0; i < 54; i++) {
        state[55+i] = vars[54+i];
    }
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

void gymnistmotion(double *time,
    double state[109],
    double dstate[109],
    double dt,
    double ctol,
    double tol,
    int *flag,
    int *err)
{
    static double step;
    double work[654],ttime,param[1];
    int vintgerr,which,ferr,i;

    gymnistgentime(&i);
    if (i != 110000) {
        gymnistseterr(54,42);
    }
    param[0] = ctol;
    ttime = *time;
    if (*flag != 0) {
        gymnistmotfunc(ttime,state,dstate,param,&ferr);
        step = dt;
        *flag = 0;
    }
    if (step <= 0.) {
        step = dt;
    }
    gymnistvinteg(gymnistmotfunc,&ttime,state,dstate,param,dt,&step,109,tol,
      work,&vintgerr,&which);
    *time = ttime;
    *err = vintgerr;
}

/* This routine performs state integration with a fixed-step integrator. */

void gymnistfmotion(double *time,
    double state[109],
    double dstate[109],
    double dt,
    double ctol,
    int *flag,
    double *errest,
    int *err)
{
    double work[436],ttime,param[1];
    int ferr,i;

    gymnistgentime(&i);
    if (i != 110000) {
        gymnistseterr(55,42);
    }
    param[0] = ctol;
    *err = 0;
    ttime = *time;
    if (*flag != 0) {
        gymnistmotfunc(ttime,state,dstate,param,&ferr);
        *flag = 0;
    }
    gymnistfinteg(gymnistmotfunc,&ttime,state,dstate,param,dt,109,work,errest,&
      ferr);
    if (ferr != 0) {
        *err = 1;
    }
    *time = ttime;
}
