#pragma once

#include <UGM/UGM.h>
#include"imgui/imgui.h"
#include"../Eigen/Core"
#include"../Eigen/Dense"
struct CanvasData {
	std::vector<Ubpa::pointf2> points;
	Ubpa::valf2 scrolling{ 0.f,0.f };
	bool opt_enable_grid{ true };//网格开关
	bool opt_enable_context_menu{ true };//右键菜单栏开关
	bool adding_line{ false };

	int SampleNum = 1000;//采样数

	//曲线参数化
	bool opt_Uniform_Parameterization{ false };//均匀参数化
	bool opt_Chordal_Parameterization{ false };//Chordal参数化
	bool opt_Centripetal_Parameterization{ false };//平方中心参数化
	bool opt_Foley_Parameterization{ false };//Foley参数化
	 
	////插值/拟合
	//bool opt_lagrange{ false };//拉格朗日插值 开关
	//bool opt_Gauss{ false };//高斯基函数插值 开关
	//bool opt_LS{ false };//最小二乘法回归 开关
	//bool opt_Ridge_Regression{ false };// 岭回归拟合 开关
	////参数
	//int LeastSquaresM = 4;
	//float RidgeRegressionLambda = 0.1;
	//float GaussTheta = 1;
	
	Eigen::VectorXf Uniform_Result;
	Eigen::VectorXf Chordal_Result;
	Eigen::VectorXf Centripetal_Result;
	Eigen::VectorXf Foley_Result;

};


#include "details/CanvasData_AutoRefl.inl"
