// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LIDAR_OPTIMIZATION_ANALYTIC_H_
#define _LIDAR_OPTIMIZATION_ANALYTIC_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t);

Eigen::Matrix3d skew(Eigen::Vector3d& mat_in);

class PenaltyCostFunction
    : public ceres::SizedCostFunction<1 /* number of residuals */,
                               1 /* size of first parameter */> {
 public:
 
      PenaltyCostFunction() = delete;
  
    PenaltyCostFunction(int _weight_index): weight_idx(_weight_index)
	{
		
	}

    bool Evaluate(double const* const* parameters,
                double* residuals,
                double** jacobians) const override {
		double x = parameters[0][0];
		// here is 0.1, 0.1 square is 0.01
		residuals[0] = (1-x)*0.1;
		if (jacobians != nullptr && jacobians[0] != nullptr) {
		   jacobians[0][0] = -0.1;
		}
		return true;
	}
	int weight_idx;

};

class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
	public:

		EdgeAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_);
		virtual ~EdgeAnalyticCostFunction() {}
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

		Eigen::Vector3d curr_point;
		Eigen::Vector3d last_point_a;
		Eigen::Vector3d last_point_b;
};

class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
	public:
		SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_);
		virtual ~SurfNormAnalyticCostFunction() {}
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

		Eigen::Vector3d curr_point;
		Eigen::Vector3d plane_unit_norm;
		double negative_OA_dot_norm;
};

class PoseSE3Parameterization : public ceres::LocalParameterization {
public:
	
    PoseSE3Parameterization() {}
    virtual ~PoseSE3Parameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize() const { return 6; }
};

class DynamicSurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7, 1> {
	public:
		DynamicSurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_);
		virtual ~DynamicSurfNormAnalyticCostFunction() {}
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

		Eigen::Vector3d curr_point;
		Eigen::Vector3d plane_unit_norm;
		double negative_OA_dot_norm;
};


#endif // _LIDAR_OPTIMIZATION_ANALYTIC_H_

