///////////////////////////////////////////////////////////////////////////////
//                                                                           //
// HLBFGS                                                                    //
// http://www.loria.fr/~liuyang/software/HLBFGS/							 //
//                                                                           //
// HLBFGS is a hybrid L-BFGS optimization framework which unifies L-BFGS     //
// method, Preconditioned L-BFGS method and                                  //
// Preconditioned Conjugate Gradient method.                                 //
//                                                                           //
// Version 1.2                                                               //
// March 09, 2010                                                            //
//                                                                           //
// Copyright (C) 2009--2010                                                  //
// Yang Liu                                                                  //
//																			 //
// xueyuhanlang@gmail.com                                                    //
//                                                                           //
// HLBFGS is HLBFGS is freely available for non-commercial purposes.		 //
//                                                                           //
///////////////////////////////////////////////////////////////////////////////


#include "HLBFGS.h"


//////////////////////////////////////////////////////////////////////////
void INIT_HLBFGS(double PARAMETERS[], int INFO[])
{
	PARAMETERS[0] = 1.0e-4; //ftol
	PARAMETERS[1] = 1.0e-16; //xtol
	PARAMETERS[2] = 0.9; //gtol
	PARAMETERS[3] = 1.0e-20; //stpmin
	PARAMETERS[4] = 1.0e+20; //stpmax
	PARAMETERS[5] = 1.0e-9; // ||g||/max(1,||x||)
	PARAMETERS[6] = 1.0e-10; // ||g||

	INFO[0] = 20; //max_fev_in_linesearch
	INFO[1] = 0; //total_num_fev
	INFO[2] = 0; //iter
	INFO[3] = 0; //update strategy. 0: standard lbfgs, 1: m1qn3;
	INFO[4] = 100000; // max iterations
	INFO[5] = 1; //1: print message, 0: do nothing
	INFO[6] = 10; // T: update interval of Hessian
	INFO[7] = 0; // 0: without hessian, 1: with accurate hessian
	INFO[8] = 15; // icfs parameter
	INFO[9] = 0; // 0: linesearch 1: modified linesearch (do not set 1 in pratice !)
	INFO[10] = 0; // 0: Disable preconditioned CG 1: preconditioned CG
	INFO[11] = 1; // different methods for choosing beta in CG.
	INFO[12] = 1; //internal usage. 0: only update diag in USER_DEFINED_HLBFGS_UPDATE_H
	INFO[13] = 0; // 0: standard lbfgs update, 1: Biggs's update, 2: Yuan's update; 3: Zhang and Xu's update
}

//////////////////////////////////////////////////////////////////////////
void HLBFGS_MESSAGE(bool print, int id, const double PARAMETERS[])
{
	if (!print)
	{
		return;
	}
	switch (id)
	{
	case 0:
		std::cout << "Please check your input parameters !\n";
		break;
	case 1:
		std::cout << "Linesearch is failed !\n";
		break;
	case 2:
		std::cout << "Convergence : ||g||/max(1,||x||) <= " << PARAMETERS[5]
		<< std::endl;
		break;
	case 3:
		std::cout << "Convergence : ||g|| <=  " << PARAMETERS[6] << std::endl;
		break;
	case 4:
		std::cout << "Convergence: linesearch cannot improve anymore \n";
		break;
	case 5:
		std::cout << "Exceeds max iteration \n";
		break;
	default:
		break;
	}
}

//////////////////////////////////////////////////////////////////////////
void HLBFGS_UPDATE_Hessian(int N, int M, double *q, double *s, double *y,
						   int cur_pos, double *diag, int INFO[])
{
	if (M <= 0 || INFO[2] == 0)
	{
		return;
	}

	int start = cur_pos * N;

	double *y_start = &y[start];
	double *s_start = &s[start];

	double ys = HLBFGS_DDOT(N, y_start, s_start);

	if (INFO[3] == 0)
	{
		double yy = HLBFGS_DDOT(N, y_start, y_start);
		double factor = ys / yy;
		if (INFO[12] == 1)
		{
			HLBFGS_DSCAL(N, factor, q);
		}
		else
		{
			diag[0] = factor;
		}

	}
	else if (INFO[3] == 1)
	{

		//m1qn3 update
		double dyy = 0;
		double dinvss = 0;
		int i = 0;
#ifdef USE_OPENMP
#pragma omp parallel for private(i) reduction(+:dinvss) reduction(+:dyy)
#endif
		for (i = 0; i < N; i++)
		{
			dinvss += s_start[i] * s_start[i] / diag[i];
			dyy += diag[i] * y_start[i] * y_start[i];
		}
#ifdef USE_OPENMP
#pragma omp parallel for private(i)
#endif
		for (i = 0; i < N; i++)
		{
			diag[i] = 1.0 / (dyy / (ys * diag[i]) + y_start[i] * y_start[i]
			/ ys - dyy * s_start[i] * s_start[i] / (ys * dinvss
				* diag[i] * diag[i]));
		}
		if (INFO[12] == 1)
		{
#ifdef USE_OPENMP
#pragma omp parallel for private(i)
#endif
			for (i = 0; i < N; i++)
			{
				q[i] *= diag[i];
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
void HLBFGS_UPDATE_First_Step(int N, int M, double *q, double *s, double *y,
							  double *rho, double *alpha, int bound, int cur_pos, int iter)
{
	if (M <= 0)
	{
		return;
	}

	int start;
	double tmp;

	for (int i = bound; i >= 0; i--)
	{
		start = iter <= M ? cur_pos - bound + i : (cur_pos - (bound - i) + M)
			% M;
		alpha[i] = rho[start] * HLBFGS_DDOT(N, q, &s[start * N]);
		tmp = -alpha[i];
		HLBFGS_DAXPY(N, tmp, &y[start * N], q);
	}
}

//////////////////////////////////////////////////////////////////////////
void HLBFGS_UPDATE_Second_Step(int N, int M, double *q, double *s, double *y,
							   double *rho, double *alpha, int bound, int cur_pos, int iter)
{
	if (M <= 0)
	{
		return;
	}

	int start;
	double tmp;

	for (int i = 0; i <= bound; i++)
	{
		start = iter <= M ? i : (cur_pos + 1 + i) % M;
		tmp = alpha[i] - rho[start] * HLBFGS_DDOT(N, &y[start * N], q);
		HLBFGS_DAXPY(N, tmp, &s[start * N], q);
	}
}

//////////////////////////////////////////////////////////////////////////
void HLBFGS_BUILD_HESSIAN_INFO(HESSIAN_MATRIX& m_hessian, int INFO[])
{
	ICFS_INFO& l_info = m_hessian.get_icfs_info();
	l_info.get_p() = INFO[8];
	l_info.set_lrow_ind_size(m_hessian.get_nonzeros()
		+ m_hessian.get_dimension() * l_info.get_p());
	l_info.set_l_size(m_hessian.get_nonzeros() + m_hessian.get_dimension()
		* l_info.get_p());
	l_info.get_icfs_alpha() = 0;
	int n = m_hessian.get_dimension();
	int nnz = m_hessian.get_nonzeros();
	dicfs_(&n, &nnz, m_hessian.get_values(), m_hessian.get_diag(),
		m_hessian.get_colptr(), m_hessian.get_rowind(), l_info.get_l(),
		l_info.get_ldiag(), l_info.get_lcol_ptr(), l_info.get_lrow_ind(),
		&l_info.get_p(), &l_info.get_icfs_alpha(), l_info.get_iwa(),
		l_info.get_wa1(), l_info.get_wa2());
}

//////////////////////////////////////////////////////////////////////////
void CONJUGATE_GRADIENT_UPDATE(int N, double *q, double *prev_q_update,
							   double *prev_q_first_stage, int INFO[])
{
	//determine beta
	double cg_beta = 1.0;
	if (INFO[11] == 1)
	{
		if (INFO[2] == 0)
		{
			std::copy(q, &q[N], prev_q_first_stage);
			std::copy(q, &q[N], prev_q_update);
			return;
		}
		else
		{
			cg_beta = HLBFGS_DDOT(N, q, q);
			cg_beta /= std::fabs(cg_beta
				- HLBFGS_DDOT(N, q, prev_q_first_stage));
			std::copy(q, &q[N], prev_q_first_stage);
		}
	}
	else
	{
		if (INFO[2] == 0)
		{
			std::copy(q, &q[N], prev_q_update);
			return;
		}
	}
	//determine new q

	const double minus_one = -1.0;
	if (cg_beta != 1.0)
		HLBFGS_DSCAL(N, cg_beta, prev_q_update);

	int i = 0;
//#ifdef USE_OPENMP
//#pragma omp parallel for private(i)
//#endif
	for (i = 0; i < N; i++)
	{
		q[i] -= prev_q_update[i];
	}

	double quad_a = HLBFGS_DDOT(N, q, q);
	double quad_b = HLBFGS_DDOT(N, q, prev_q_update);
	double cg_lambda = -quad_b / quad_a;
	if (cg_lambda > 1)
		cg_lambda = 1;
	else if (cg_lambda < 0)
		cg_lambda = 0;

#ifdef USE_OPENMP
#pragma omp parallel for private(i)
#endif
	for (i = 0; i < N; i++)
	{
		q[i] = cg_lambda * q[i] + prev_q_update[i];
	}
	std::copy(q, &q[N], prev_q_update);
}

//////////////////////////////////////////////////////////////////////////
