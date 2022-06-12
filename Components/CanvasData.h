#pragma once

#include <UGM/UGM.h>
#include"imgui/imgui.h"
#include"../Eigen/Core"
#include"../Eigen/Dense"
struct CanvasData {
	std::vector<Ubpa::pointf2> points;
	Ubpa::valf2 scrolling{ 0.f,0.f };
	bool opt_enable_grid{ true };//���񿪹�
	bool opt_enable_context_menu{ true };//�Ҽ��˵�������
	bool adding_line{ false };

	int SampleNum = 1000;//������

	//���߲�����
	bool opt_Uniform_Parameterization{ false };//���Ȳ�����
	bool opt_Chordal_Parameterization{ false };//Chordal������
	bool opt_Centripetal_Parameterization{ false };//ƽ�����Ĳ�����
	bool opt_Foley_Parameterization{ false };//Foley������
	 
	////��ֵ/���
	//bool opt_lagrange{ false };//�������ղ�ֵ ����
	//bool opt_Gauss{ false };//��˹��������ֵ ����
	//bool opt_LS{ false };//��С���˷��ع� ����
	//bool opt_Ridge_Regression{ false };// ��ع���� ����
	////����
	//int LeastSquaresM = 4;
	//float RidgeRegressionLambda = 0.1;
	//float GaussTheta = 1;
	
	Eigen::VectorXf Uniform_Result;
	Eigen::VectorXf Chordal_Result;
	Eigen::VectorXf Centripetal_Result;
	Eigen::VectorXf Foley_Result;

};


#include "details/CanvasData_AutoRefl.inl"
