#pragma once

namespace v0
{
	struct alignAngles: public _op
	{
		m_real mRefAngle;
		alignAngles(m_real refAngle):mRefAngle(refAngle){}
		virtual void calc(vectorn& c) const;

		// utility functions
		static void projectAngle(m_real& angle);	// represent angle in [-pi, pi]
		static void alignAngle(m_real prev, m_real& next);
	};
}
namespace v1
{
	struct inversePiecewiseLinearFunction: public _op
	{
		inversePiecewiseLinearFunction(){}
		// In case that in[0]==0, in[in.size()-1]==a,
		// out[0]=0, out[a]=in.size()-1.
		virtual void calc(vectorn& out, const vectorn &in) const;
	};
};

namespace m1
{
	struct inpaint: public _op
	{
		inpaint(){}
		virtual void calc(matrixn& c, const matrixn& a) const ;
	};

	// safer, but can be jerkier
	struct inpaint0: public _op
	{
		inpaint0(){}
		virtual void calc(matrixn& c, const matrixn& a) const ;
	};

	struct upsampling: public _op
	{
		upsampling(int xn, int start=0, int end=INT_MAX):m_nXn(xn), m_nStart(start), m_nEnd(end){}
		virtual void calc(matrixn& c, const matrixn& a) const;/// c.op(a)
		int m_nXn, m_nStart, m_nEnd;
	};

	// desired length: out.rows()-1 == tmwpFunction.size()-1
	// usage: in case of uniform stretching
	//	tmwpFunction.linspace(0, in.rows()-1, desiredLength+1);
	struct timewarpingLinear: public _op
	{
		timewarpingLinear(const vectorn& tmwpFunction):timewarpFunction(tmwpFunction){}
		virtual void calc(matrixn& out, const matrixn& in) const;
		const vectorn& timewarpFunction;
	};

}
