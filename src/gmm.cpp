#include "stdafx.h"


// -*- c++ -*- (enables emacs c++ mode)
//===========================================================================
//
// Copyright (C) 2007-2008 Yves Renard, Julien Pommier.
//
// This file is a part of GETFEM++
//
// Getfem++  is  free software;  you  can  redistribute  it  and/or modify it
// under  the  terms  of the  GNU  Lesser General Public License as published
// by  the  Free Software Foundation;  either version 2.1 of the License,  or
// (at your option) any later version.
// This program  is  distributed  in  the  hope  that it will be useful,  but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or  FITNESS  FOR  A PARTICULAR PURPOSE.  See the GNU Lesser General Public
// License for more details.
// You  should  have received a copy of the GNU Lesser General Public License
// along  with  this program;  if not, write to the Free Software Foundation,
// Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301, USA.
//
//===========================================================================
// SQUARED_MATRIX_PARAM;
// DENSE_VECTOR_PARAM;
// VECTOR_PARAM;
// ENDPARAM;

#include "gmm.h"
#include "dependencies/gmm-3.0/include/gmm/gmm_dense_lu.h"
#include "dependencies/gmm-3.0/include/gmm/gmm_condition_number.h"


void gmm::conversion(rsmatrixn & out, wsmatrixn & in)
{
	gmm::clean(in, 1E-12);
	gmm::copy(out, in);
}

void gmm::zero(wsmatrixn& inout, int n, int m)
{
    gmm::wsmatrixn temp(n, m);
	std::swap(inout, temp);
}

void gmm::assign(wsmatrixn& out, matrixn const& in)
{
	gmm::resize(out, in.rows(), in.cols());

	for(int i=0; i<in.rows(); i++)
		for(int j=0; j<in.cols(); j++)
		{
			m_real v=in[i][j];
			if(v!=0.0)
				out(i,j)=v;
		}
}

void test_GMM_LU()
{
	gmm::wsvectorn b(10);

	for(int i=0; i<10; i+=2)
	{
		cout <<"b[i]="<< b[i]<< endl;
		b[i]=SQR(i);
		cout <<"b[i]="<< b[i]<< endl;
	}

	for(gmm::wsv_iterator i=b.begin(); i!=b.end(); i++)
	{
		cout << i.index()<< ": "<< *i<<endl;
	}

	gmm::wsmatrixn a(10,10);

	for(int i=0; i<10; i++)
		for(int j=0; j<10; j++)
		{
			if(j==5)
				cout <<"a["<<i<<"]="<< a[i]<< endl;

			a(i,j)=i*j;
			cout <<"a("<<i<<","<<j<<")="<< a(i,j)<< endl;
		}
}


void SparseQuadraticFunction::buildSystem(int dim, gmm::wsmatrixn & H, vectorn &R)
{
	gmm::zero(H, dim, dim);

	R.setSize(dim);
	//H.setAllValue(0);	// sparse matrix has no element in it by default.
	R.setAllValue(0);

	for(std::list<SquaredTerm*>::iterator i=mListSQTerms.begin(); i!=mListSQTerms.end(); ++i)
	{
		intvectorn& index=(*i)->index;
		vectorn& value=(*i)->coef;

		// updateH
		for(int i=0; i<index.size(); i++)
			H(index[i],index[i])+=2.0*SQR(value[i]);
		for(int i=0; i<index.size(); i++)
			for(int j=i+1; j<index.size(); j++)
			{
				m_real prev=H(index[j],index[i]);
				H(index[i],index[j])=H(index[j],index[i])=prev+2.0*value[i]*value[j];
			}

		// update R
		for(int i=0; i<index.size(); i++)
		{
			R[index[i]]-=2.0*value[index.size()]*value[i];
		}
	}
}

SparseQuadraticFunctionHardCon::~SparseQuadraticFunctionHardCon()
{
	for(std::list<Con*>::iterator i=mListConTerms.begin(); i!=mListConTerms.end(); ++i)
		delete (*i);
}

void SparseQuadraticFunctionHardCon:: addCon(intvectorn const& index, vectorn const& value)
{
	Con* term=new Con;
	mListConTerms.push_back(term);
	term->index.assign(index);
	term->coef.assign(value);
}
void SparseQuadraticFunctionHardCon ::addCon(int n, m_real coef1, int index1, ...)
{
	Con* term=new Con;
	mListConTerms.push_back(term);

	term->index.setSize(n);
	term->coef.setSize(n+1);

	term->coef[0]=coef1;
	term->index[0]=index1;

	va_list marker;
	va_start( marker, index1);

	int i;
	for(i=1; i<n; i++)
	{
		term->coef[i]=va_arg(marker, m_real);
		term->index[i]=va_arg(marker, int);
	}

	term->coef[i]=va_arg(marker, m_real);
	va_end(marker);
}


void SparseQuadraticFunctionHardCon ::buildSystem(gmm::wsmatrixn & Augmented, vectorn &d)
{
	int nvar=mNumVar;
	int numCon=mNumCon;

	// LAGRANGIAN MULTIPLIER version
	//    Augmented* x= d , w=(X, lamda)
	//   (H A^T) (X    )= (d)
	//   (A 0  ) (lamda)

	gmm::zero(Augmented, nvar, nvar);
	d.setSize(nvar+numCon);

	SparseQuadraticFunction::buildSystem(nvar, Augmented, d.range(0, nvar).lval());


	gmm::resize(Augmented, nvar+numCon, nvar+numCon);


	//matrixnView H=Augmented.range(0, nvar, 0, nvar);
	//matrixnView A=Augmented.range(nvar, nvar+numCon, 0, nvar);
	//matrixnView At=Augmented.range(0, nvar, nvar, nvar+numCon);



	// set constraint matrix (constraint Ax=b)
	//A.setAllValue(0.0);	-> already zero

	int icon=0;
	for(std::list<Con*>::iterator i=mListConTerms.begin(); i!=mListConTerms.end(); ++i)
	{
		Con* con=*i;

		for(int i=0; i<con->index.size(); i++)
		{
			Augmented(icon+nvar,con->index[i])=con->coef[i];
			Augmented(con->index[i], icon+nvar)=con->coef[i];
			d[nvar+icon]=con->coef[con->index.size()]*-1.0;
		}
		icon++;
	}
	ASSERT(icon==mNumCon);

	//At.transpose(A);

	//Augmented.range(nvar, nvar+numCon, nvar, nvar+numCon).setAllValue(0.0); -> already zero
}
