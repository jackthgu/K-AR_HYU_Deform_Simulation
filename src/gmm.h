#pragma once

#include "umfsolver.h"
#include "BaseLib/math/OperatorStitch.h"

// numerical/analytic unconstrained SQP solver utility written by Taesoo.
class SparseQuadraticFunction: public QuadraticFunction
{
	int m_nVar;
public:	
	typedef gmm::wsmatrixn MAT_TYPE;	// traits.

	SparseQuadraticFunction():QuadraticFunction(){}
	SparseQuadraticFunction(int nvar):QuadraticFunction(), m_nVar(nvar){}
	virtual ~SparseQuadraticFunction(){}
	
	void buildSystem(int nvar, gmm::wsmatrixn & A, vectorn &b);
	void buildSystem(MAT_TYPE & A, vectorn &b)
	{
		buildSystem(m_nVar, A, b);
	}
	void solve(MAT_TYPE const& A, vectorn const& b, vectorn & x)
	{
		sm::UMFsolve(A, b, x);
	}

};

// analytic constrained SQP solver.
class SparseQuadraticFunctionHardCon : public SparseQuadraticFunction
{
	int mNumVar;
	int mNumCon;
public:

	typedef gmm::wsmatrixn MAT_TYPE;	// traits.

	struct Con
	{
		intvectorn index;
		vectorn coef;
	};

	std::list<Con*> mListConTerms;

	SparseQuadraticFunctionHardCon (int numVar, int numCon):mNumVar(numVar), mNumCon(numCon){}
	virtual ~SparseQuadraticFunctionHardCon ();

	void addCon(int n, m_real coef1, int index1, ...);
	void addCon(intvectorn const& index, vectorn const& value);

	void buildSystem(gmm::wsmatrixn & A, vectorn &b);
	
	void solve(gmm::wsmatrixn const& A, vectorn const& b, vectorn & x)
	{
		sm::UMFsolve(A, b, x);
	}
	vectorn factorize(gmm::wsmatrixn const& A)
	{
		return sm::UMFfactorize(A);
	}
	void solveFactorized(vectorn const& solver, vectorn const& b, vectorn &x)
	{
		sm::UMFsolveFactorized(solver,  b, x);
	}
	void freeUMFsolver(vectorn const& solver)
	{
		sm::freeUMFsolver(solver);
	}
};


void test_GMM_LU();


