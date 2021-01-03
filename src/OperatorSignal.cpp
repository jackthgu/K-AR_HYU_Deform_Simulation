#include "stdafx.h"
#include "BaseLib/math/mathclass.h"
#include "BaseLib/math/OperatorStitch.h"
#include "BaseLib/math/Operator.h"
#include "BaseLib/math/nr/nr.h"
#include "OperatorSignal.h"
#include "BaseLib/math/Operator.h"
#include "BaseLib/math/BSpline.h"
namespace v
{
	void sampling(vectorn& sample, matrixn const& input, m_real time)
	{
		//!< 0 <=time<= input.rows()-1

		int a;
		m_real t;

		a=(int)floor(time);
		t=time-(m_real)a;

		if(t<0.005)
			sample=input.row(a);
		else if(t>0.995)
			sample=input.row(a+1);
		else
		{
			if(a<0)
				//sample.op2(v2::interpolate(t-1.0), input.row(a+1), input.row(a+2));
				v::interpolate(sample,t-1.0, input.row(a+1), input.row(a+2));
			else if(a+1>=input.rows())
			//	sample.op2(v2::interpolate(t+1.0), input.row(a-1), input.row(a));
				v::interpolate(sample,t+1.0, input.row(a-1), input.row(a));
			else
			//	sample.op2(v2::interpolate(t), input.row(a), input.row(a+1));
				v::interpolate(sample,t, input.row(a), input.row(a+1));
		}
	}
}


void v0::alignAngles::calc(vectorn& c) const
{
	alignAngle(mRefAngle, c[0]);
	for(int i=1; i<c.size(); i++)
		alignAngle(c[i-1], c[i]);
}

void v0::alignAngles::projectAngle(m_real& angle)
{
	// represent angle in [-pi, pi]
	while(angle>M_PI+FERR)
		angle-=2.0*M_PI;
	while(angle<-1.0*M_PI-FERR)
		angle+=2.0*M_PI;

}
void v0::alignAngles::alignAngle(m_real prev, m_real& next)
{
	// next-prev는 [-PI,PI]에 있다고 보장을 못한다. 따라서 이범위에 들어오는 next-prev를 구한다.

	m_real delta=next-prev;

	projectAngle(delta);

	// 다시 원래의 prev범위로 되돌린다.
	next=prev+delta;
}


void v1::inversePiecewiseLinearFunction::calc(vectorn& out, const vectorn &inputFunction) const
{
	m_real minTime=inputFunction[0];
	m_real maxTime=inputFunction[inputFunction.size()-1];

	ASSERT(isSimilar(minTime,0.0));

	// 길이가 정수가 되도록 rounding한다. 1을 더하는 이유는, 정수로 바꾸면 연속된 동작세그먼트간 1프레임 오버랩이 생기기 때문.
	int nDesiredLen=ROUND(maxTime);

	vectorn in(inputFunction);
	in*=1.0/maxTime*((m_real)nDesiredLen);
	
	ASSERT(nDesiredLen>=0);
	
	out.setSize(nDesiredLen+1);

	int curInterval=0;
	for(int i=0; i<=nDesiredLen; i++)
	{
		m_real t=m_real (i);
		int j=curInterval+1;
		for(; j<in.size(); j++)
		{
			if(in[j]>=t-FERR)
				break;
		}

		curInterval=j-1;

		m_real frac=((m_real)i-in[curInterval])/(in[j]-in[curInterval]);

		out[i]=m_real(curInterval)+frac;
	}
}


void m1::timewarpingLinear::calc(matrixn& out, const matrixn& in) const
{
	out.setSize(timewarpFunction.size(), in.cols());

	for(int i=0; i<timewarpFunction.size(); i++)
	{
		vectornView o_i=out.row(i);
		v::sampling(o_i, in, timewarpFunction[i]);
	}
}


void m1::upsampling::calc(matrixn& c, const matrixn& a) const/// c.op(a)
{
	UniformSpline spl(a, m_nStart, m_nEnd);
	spl.getCurve(c, m_nXn);
}

void m1::inpaint0::calc(matrixn& c, const matrixn& a) const 
{
	vectorn w(a.rows()-2);
	w.linspace(1.0, 0.0);

	c.setSameSize(a);

	c.row(0)=a.row(0);
	c.row(c.rows()-1)=a.row(a.rows()-1);
	
	for(int i=0; i<w.size(); i++)
		for(int dim=0; dim<c.cols(); dim++)
			c[i+1][dim]=w[i]*a[1][dim]+(1.0-w[i])*a[a.rows()-2][dim];
}

void m1::inpaint::calc(matrixn& c, const matrixn& a) const 
{
	/* LAGRANGIAN MULTIPLIER version*/
	//    Augmented* x= d , w=(X, lamda)
	//   (H A^T) (X    )= (d)
	//   (A 0  ) (lamda)
	c.setSize(a.rows(), a.cols());
	int nsample=c.rows();

	int numCon=4;
	matrixn Augmented(c.rows()+numCon, c.rows()+numCon);
	matrixnView H=Augmented.range(0,c.rows(), 0, c.rows());
	matrixnView A=Augmented.range(c.rows(), c.rows()+numCon, 0, c.rows());
	matrixnView At=Augmented.range(0, c.rows(), c.rows(), c.rows()+numCon);

	vectorn x, d(c.rows()+numCon);
	// minimize sum (가속도)^2
	// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2]-0)^2} 의 hessian(=gradient의 계수).
	HessianQuadratic h(c.rows());
	intvectorn index;
	vectorn coef;
	for(int i=0; i<nsample-2; i++)
	{
		index.setValues(3, i, i+1, i+2);
		coef.setValues(4, 1.0, -2.0, 1.0, 0.0);
		h.addSquared(index, coef);
	}

	H=h.H;

	// set constraint matrix (constraint Ax=b)
	A.setAllValue(0.0);
	A[0][0]=1.0;
	A[1][1]=1.0;
	A[2][c.rows()-2]=1.0;
	A[3][c.rows()-1]=1.0;
	
	At.transpose(A);

	Augmented.range(c.rows(), c.rows()+numCon, c.rows(), c.rows()+numCon).setAllValue(0.0);
#define USE_LUDCMP
#ifdef USE_LUDCMP
	DP p;
	matrixn LU(Augmented);
	Vec_INT indx(Augmented.rows());
	NR::ludcmp(LU,indx,p);
#else
	matrixn invAugmented;
	invAugmented.inverse(Augmented);
#endif

	for(int dim=0; dim<a.cols(); dim++)
	{
#define f(xxx) c(xxx,dim)
#define fa(xxx)	a(xxx,dim)
		d.range(0, c.rows())=h.R;

		// set constraint
		d[c.rows()]=fa(0);
		d[c.rows()+1]=fa(1);
		d[c.rows()+2]=fa(a.rows()-2);
		d[c.rows()+3]=fa(a.rows()-1);

#ifdef USE_LUDCMP
		x=d;
		NR::lubksb(LU,indx,x);
#else
		x.multmat(invAugmented, d);
#endif
		// save results.
		for(int i=0; i<c.rows(); i++)
			f(i)=x[i];
	}
}

