//
//	SD/Fast headers
//
//
#ifndef gymnistFASTDEFS
#define gymnistFASTDEFS
extern "C" {
#include "MotionSynthesisLib/RigidBody/SDFAST_implementation/SDFAST_common.h"
//	SD/Fast routines
extern int gymnistinit();
extern int gymnistprinterr(FILE*);
extern void gymnisterror(int *routine, int* errnum);
extern void gymnistclearerr();
extern int gymniststatic(double time,double state[],int lock[],double ctol,double tol,
int maxevals,int* fcnt,int *err);
extern int gymnistpointf(int body,sdvector3 point,sdvector3 force);
extern int gymnisttrans(int frbod,sdvector3 ivec,int tobod,sdvector3 ovec);
extern int gymnisthinget(int joint,int axis,double torque);
extern void gymnistbodyt(int body,sdvector3 torque);
extern void gymnistangvel(int body,sdvector3 avel);
extern void gymnistbtj(int joint, sdvector3 btjin);
extern void gymnistgetbtj(int joint, sdvector3 btjout);
extern void gymnistitj(int joint, sdvector3 itjin);
extern void gymnistgetitj(int joint, sdvector3 itjout);
extern void gymnistvel(int body, sdvector3 pt, sdvector3 velo);
extern int gymnistpos(int body,sdvector3 pt,sdvector3 loc);
extern int gymnistorient(int body,double mat[3][3]);
extern int gymnistmotion(double* time,double state[],double dstate[],
double dt,double ctol,double tol,int* flag,int* err);
extern void gymnistfmotion(double *time,double state[],double dstate[],
double dt,double ctol,int *flag,double *errest,int *err);
extern int gymnistmass(int body,double massin);
extern int gymnistgetmass(int body, double *massout);
extern int gymnistiner(int body,double *inerin);
extern int gymnistgetiner(int body, double *inerout);
extern int gymnistreac(sdvector3 force[],sdvector3 torque[]);
extern int gymnistacc(int body,sdvector3 pt, sdvector3 accel);
extern int gymnistgetgrav(sdvector3 gravout);
extern int gymnistinfo(int info[50]);						// get info
extern int gymnistindx(int joint, int axis);
extern void gymnistuforce(double t, double q[], double u[]);	// user provides this
extern void gymniststate(double timein, double qin[], double uin[]);
	extern void gymnistmassmat(double mmat[54][54]);
}
#endif // SDFASTDEFS
