/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "../include/sophus/so3.hpp"
#include "../include/sophus/se3.hpp"

///
template <typename T> inline
void QuaternionInverse(const T q[4], T q_inverse[4])
{
	q_inverse[0] = q[0];
	q_inverse[1] = -q[1];
	q_inverse[2] = -q[2];
	q_inverse[3] = -q[3];
};


struct TError
{
	TError(double t_x, double t_y, double t_z, double var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), var(var){}

	template <typename T>
	bool operator()(const T* tj, T* residuals) const
	{
		residuals[0] = (tj[0] - T(t_x)) / T(var);
		residuals[1] = (tj[1] - T(t_y)) / T(var);
		residuals[2] = (tj[2] - T(t_z)) / T(var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z, const double var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          TError, 3, 3>(
	          	new TError(t_x, t_y, t_z, var)));
	}

	double t_x, t_y, t_z, var;

};

struct TError2
{
	TError2(double t_x, double t_y, double t_z, double var_lat ,double var_lon,double var_alt)
				  :t_x(t_x), t_y(t_y), t_z(t_z), var_lat(var_lat) ,var_lon(var_lon) ,var_alt(var_alt){}

	template <typename T>
	bool operator()(const T* tj, T* residuals) const
	{
		residuals[0] = (tj[0] - T(t_x)) / T(var_lat);
		residuals[1] = (tj[1] - T(t_y)) / T(var_lon);
		residuals[2] = (tj[2] - T(t_z)) / T(var_alt);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z, 
									   const double var_lat, const double var_lon ,const double var_alt)
	{
	  return (new ceres::AutoDiffCostFunction<
	          TError2, 3, 3>(
	          	new TError2(t_x, t_y, t_z, var_lat ,var_lon ,var_alt)));
	}

	double t_x, t_y, t_z, var_lat,var_lon,var_alt;

};

struct RelativeRTError
{
	RelativeRTError(double t_x, double t_y, double t_z, 
					double q_w, double q_x, double q_y, double q_z,
					double t_var, double q_var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), 
				   q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
				   t_var(t_var), q_var(q_var){}

	template <typename T>
	bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		T i_q_w[4];
		QuaternionInverse(w_q_i, i_q_w);

		T t_i_ij[3];
		ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

		residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
		residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
		residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

		T relative_q[4];
		relative_q[0] = T(q_w);
		relative_q[1] = T(q_x);
		relative_q[2] = T(q_y);
		relative_q[3] = T(q_z);

		T q_i_j[4];
		ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);

		T relative_q_inv[4];
		QuaternionInverse(relative_q, relative_q_inv);

		T error_q[4];
		ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q); 

		residuals[3] = T(2) * error_q[1] / T(q_var);
		residuals[4] = T(2) * error_q[2] / T(q_var);
		residuals[5] = T(2) * error_q[3] / T(q_var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double q_w, const double q_x, const double q_y, const double q_z,
									   const double t_var, const double q_var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          RelativeRTError, 6, 4, 3, 4, 3>(
	          	new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
	}

	double t_x, t_y, t_z, t_norm;
	double q_w, q_x, q_y, q_z;
	double t_var, q_var;

};



struct RelativeRTError2
{
	RelativeRTError2(double t_x, double t_y, double t_z, 
					double q_w, double q_x, double q_y, double q_z,
					double t_var, double q_var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), 
				   q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
				   t_var(t_var), q_var(q_var){}

	template <typename T>
	bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, T* residuals) const
	{


		{ ///旋转矩阵法

			T t_w_ij[3];
			t_w_ij[0] = tj[0] - ti[0];
			t_w_ij[1] = tj[1] - ti[1];
			t_w_ij[2] = tj[2] - ti[2];

			Eigen::Matrix3d Rj = w_q_j.toRotationMatrix();
			Eigen::Matrix3d Ri = w_q_i.toRotationMatrix();
			Eigen::Matrix3d Ri_j = Rj * Ri.inverse();

			T t_i_ij[3];
			Eigen::Vector3d t(t_w_ij[0],t_w_ij[1],t_w_ij[2]);
			Eigen::Vector3d t2 = Ri_j * t;
			t_i_ij[0] = t2[0];
			t_i_ij[1] = t2[1];
			t_i_ij[2] = t2[2];

			residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
			residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
			residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);


			T relative_q[4];

			Eigen::Quaterniond QW(T(q_x),T(q_y),T(q_z),T(q_w));

			Eigen::Matrix3d R = QW.toRotationMatrix();
			Sophus::SO3<double> SO3_R(R);
		    Eigen::Vector3d so3 = SO3_R.log();    ///

			Sophus::SO3<double> SO3_R_i_j(Ri_j);
			Eigen::Vector3d so3_i_j = SO3_R_i_j.log();


			T error_q[4];
			// error_q[0] = so3_i_j[0] - so3[0];
			// error_q[1] = so3_i_j[1] - so3[1];
			// error_q[2] = so3_i_j[2] - so3[2];

			error_q[0] = so3[0] - so3_i_j[0]  ;
			error_q[1] = so3[1] - so3_i_j[1]  ;
			error_q[2] = so3[2] - so3_i_j[2]  ;

			residuals[3] = T(1) * error_q[1] / T(q_var);
			residuals[4] = T(1) * error_q[2] / T(q_var);
			residuals[5] = T(1) * error_q[3] / T(q_var);
			
		}
		// T t_w_ij[3];
		// t_w_ij[0] = tj[0] - ti[0];
		// t_w_ij[1] = tj[1] - ti[1];
		// t_w_ij[2] = tj[2] - ti[2];

		// T i_q_w[4];
		// QuaternionInverse(w_q_i, i_q_w);

		// T t_i_ij[3];
		// ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

		// residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
		// residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
		// residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

		// T relative_q[4];
		// relative_q[0] = T(q_w);
		// relative_q[1] = T(q_x);
		// relative_q[2] = T(q_y);
		// relative_q[3] = T(q_z);

		// T q_i_j[4];
		// ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);

		// T relative_q_inv[4];
		// QuaternionInverse(relative_q, relative_q_inv);

		// T error_q[4];
		// ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q); 

		// residuals[3] = T(2) * error_q[1] / T(q_var);
		// residuals[4] = T(2) * error_q[2] / T(q_var);
		// residuals[5] = T(2) * error_q[3] / T(q_var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double q_w, const double q_x, const double q_y, const double q_z,
									   const double t_var, const double q_var) 
	{
			
	  return (new ceres::AutoDiffCostFunction<
	          RelativeRTError, 6, 4, 3, 4, 3>(
	          	new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
	}


	
	double t_x, t_y, t_z, t_norm;
	double q_w, q_x, q_y, q_z;
	double t_var, q_var;

};

struct Prior_RError
{
	Prior_RError(double q_w, double q_x, double q_y, double q_z, double q_var)
			:q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),q_var(q_var)
			{}

	template <typename T>
	bool operator()(const T* const Ob_w_q, T* residuals) const
	{
		////////////////////vmlinuz
		 T Ob_w_q_inverse[4];
		 QuaternionInverse(Ob_w_q, Ob_w_q_inverse);
		/////////////////////////////

		T now_q[4];
		now_q[0] = T(q_w);
		now_q[1] = T(q_x);
		now_q[2] = T(q_y);
		now_q[3] = T(q_z);

		T error_q[4];
		ceres::QuaternionProduct(now_q, Ob_w_q_inverse, error_q); 

		residuals[0] = T(2) * error_q[1] / T(q_var);
		residuals[1] = T(2) * error_q[2] / T(q_var);
		residuals[2] = T(2) * error_q[3] / T(q_var);

		return true;
	}


	static ceres::CostFunction* Create( const double q_w, const double q_x, const double q_y, const double q_z,
										const double q_var) 
	{
		
		return (new ceres::AutoDiffCostFunction<
	          Prior_RError, 3, 4>(
	          	new Prior_RError( q_w, q_x, q_y, q_z, q_var)));
	}



	double q_w, q_x, q_y, q_z;
	double q_var;

	Eigen::Matrix4d TT;
	
};