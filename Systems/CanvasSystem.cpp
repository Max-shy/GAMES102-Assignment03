#include "CanvasSystem.h"

#include "../Components/CanvasData.h"
#include <_deps/imgui/imgui.h>
#include<cmath>

# define M_PI 3.14159265358979323846
using namespace Ubpa;

//参数化
void Parameterization(CanvasData *data);
void Update_Uniform_Parameterization(CanvasData* data);
void Update_Chordal_Parameterization(CanvasData* data);
void Update_Centripetal_Parameterization(CanvasData* data);
void Update_Foley_Parameterization(CanvasData* data);
void Draw_Parameterization(CanvasData* data, ImDrawList* draw_list, int num_samples, const ImVec2 origin);

//插值函数
ImVec2 Lagrange_Interpolation(float t, const Eigen::VectorXf& parameterization, const std::vector<Ubpa::pointf2>& points);
ImVec2 Gauss_Interpolation(float t, const Eigen::VectorXf& parameterization, const std::vector<Ubpa::pointf2>& points, float theta);


float _getAlpha(int i, const std::vector<Ubpa::pointf2>& points);


void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<CanvasData>();
		if (!data)
			return;

		if (ImGui::Begin("Canvas")) {
			ImGui::Checkbox("Enable grid", &data->opt_enable_grid);//是否加入网格
			ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);//是否启动右键文本栏
			ImGui::Text("Mouse Left: drag to add point,\nMouse Right: drag to scroll, click for context menu.");//文本

			//参数化开关
			ImGui::Checkbox("Uniform Parameterization", &data->opt_Uniform_Parameterization);
			ImGui::Checkbox("Chordal Parameterization", &data->opt_Chordal_Parameterization);
			ImGui::Checkbox("Centripetal Parameterization", &data->opt_Centripetal_Parameterization);
			ImGui::Checkbox("Foley Parameterization", &data->opt_Foley_Parameterization);
			ImGui::SameLine(200);
			ImGui::InputInt("Sample Num", &data->SampleNum);//输入最小二乘法中幂基的最高次数

			////插值开关
			//ImGui::Checkbox("Lagrange", &data->opt_lagrange);//是否进行拉格朗日插值
			//ImGui::Checkbox("Gauss", &data->opt_Gauss);//是否进行高斯基函数插值
			//ImGui::SameLine(200);
			//ImGui::InputFloat("theta", &data->GaussTheta);//高斯计函数theta

			////回归开关
			//ImGui::Checkbox("LS", &data->opt_LS);//是否进行最小二乘法回归
			//ImGui::SameLine(200);
			//ImGui::InputInt("m", &data->LeastSquaresM);//输入最小二乘法中幂基的最高次数

			//ImGui::Checkbox("Ridge_Regression", &data->opt_Ridge_Regression);//是否进行岭回归
			//ImGui::SameLine(200);
			//ImGui::InputFloat("lambda", &data->RidgeRegressionLambda);//岭回归lambda

			// Using InvisibleButton() as a convenience 1) it will advance the layout cursor and 2) allows us to use IsItemHovered()/IsItemActive()
			ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();      //ImDrawList API使用屏幕坐标!  
			ImVec2 canvas_sz = ImGui::GetContentRegionAvail();   //调整画布的大小为可用的  
			if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
			if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
			ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

			// 绘制边框和背景颜色
			ImGuiIO& io = ImGui::GetIO();
			ImDrawList* draw_list = ImGui::GetWindowDrawList();
			draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
			draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

			// 这将捕捉到我们的互动
			ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
			const bool is_hovered = ImGui::IsItemHovered(); // Hovered
			const bool is_active = ImGui::IsItemActive();   // Held
			const ImVec2 origin(canvas_p0.x + data->scrolling[0], canvas_p0.y + data->scrolling[1]); //原点
			const pointf2 mouse_pos_in_canvas(io.MousePos.x - origin.x, io.MousePos.y - origin.y);//点击的点坐标

			//添加坐标点
			if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
				data->points.push_back(mouse_pos_in_canvas);//在数据中加入采样点
			}

			//滑动窗口？
			const float mouse_threshold_for_pan = data->opt_enable_context_menu ? -1.0f : 0.0f;
			if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Right, mouse_threshold_for_pan)){
				data->scrolling[0] += io.MouseDelta.x;
				data->scrolling[1] += io.MouseDelta.y;
			}

			// Context menu (under default mouse threshold)
			ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
			if (data->opt_enable_context_menu && ImGui::IsMouseReleased(ImGuiMouseButton_Right) && drag_delta.x == 0.0f && drag_delta.y == 0.0f)
				ImGui::OpenPopupContextItem("context");
			if (ImGui::BeginPopup("context"))
			{
				if (data->adding_line)
					data->points.resize(data->points.size() - 2);
				data->adding_line = false;
				if (ImGui::MenuItem("Remove one", NULL, false, data->points.size() > 0)) { data->points.resize(data->points.size() - 1); }
				if (ImGui::MenuItem("Remove all", NULL, false, data->points.size() > 0)) { data->points.clear(); }
				ImGui::EndPopup();
			}

			// Draw grid + all lines in the canvas
			draw_list->PushClipRect(canvas_p0, canvas_p1, true);
			if (data->opt_enable_grid)
			{
				const float GRID_STEP = 64.0f;
				for (float x = fmodf(data->scrolling[0], GRID_STEP); x < canvas_sz.x; x += GRID_STEP)
					draw_list->AddLine(ImVec2(canvas_p0.x + x, canvas_p0.y), ImVec2(canvas_p0.x + x, canvas_p1.y), IM_COL32(200, 200, 200, 40));
				for (float y = fmodf(data->scrolling[1], GRID_STEP); y < canvas_sz.y; y += GRID_STEP)
					draw_list->AddLine(ImVec2(canvas_p0.x, canvas_p0.y + y), ImVec2(canvas_p1.x, canvas_p0.y + y), IM_COL32(200, 200, 200, 40));
			}

			//画点
			for (int i = 0; i < data->points.size(); i++) {
				draw_list->AddCircleFilled(ImVec2(origin.x + data->points[i][0], origin.y + data->points[i][1]), 4.0f, IM_COL32(255, 255, 0, 255));
			}
			

			//参数化
			Parameterization(data);
			Draw_Parameterization(data, draw_list, data->SampleNum, origin);

			draw_list->PopClipRect();
		}

		ImGui::End();
		});
}

void Parameterization(CanvasData* data) {
	if (data->points.empty())
		return;
	Update_Uniform_Parameterization(data);
	Update_Chordal_Parameterization(data);
	Update_Centripetal_Parameterization(data);
	Update_Foley_Parameterization(data);

}

void Update_Uniform_Parameterization(CanvasData* data) {
	int n = data->points.size();
	data->Uniform_Result = Eigen::VectorXf::LinSpaced(n, 0, 1);//[0,1]等距分成n份，等距离参数化
}

void Update_Chordal_Parameterization(CanvasData* data) {
	int n = data->points.size();
	data->Chordal_Result = Eigen::VectorXf::Zero(n);
	if (n == 1 || n == 2){
		data->Chordal_Result(n - 1) = 1;
		return;
	}

	for (size_t i = 1; i < n ; i++){
		data->Chordal_Result[i] = data->Chordal_Result[i - 1] + (data->points[i] - data->points[i - 1]).norm();
	}
	data->Chordal_Result /= data->Chordal_Result[n - 1];
}

void Update_Centripetal_Parameterization(CanvasData* data) {
	int n = data->points.size();
	data->Centripetal_Result = Eigen::VectorXf::Zero(n);
	if (n == 1 || n == 2)
	{
		data->Centripetal_Result(n - 1) = 1;
		return;
	}

	for (size_t i = 1; i < n ; i++){
		data->Centripetal_Result[i] = data->Centripetal_Result[i - 1] + std::sqrt((data->points[i] - data->points[i - 1]).norm());
	}
	data->Centripetal_Result /= data->Centripetal_Result[n - 1];
}

void Update_Foley_Parameterization(CanvasData* data) {
	int n = data->points.size();
	data->Foley_Result = Eigen::VectorXf::Zero(n);
	if (n == 1 || n == 2)
	{
		data->Foley_Result(n - 1) = 1;
		return;
	}

	float d_prev = 0, d_next = 0;
	for (size_t i = 0; i < n - 1; i++)
	{
		float dx = data->points[i + 1][0] - data->points[i][0];
		float dy = data->points[i + 1][1] - data->points[i][1];
		float chord = sqrt(dx * dx + dy * dy);

		if (i == n - 2) d_next = 0;
		else {
			float dx = data->points[i + 2][0] - data->points[i + 1][0];
			float dy = data->points[i + 2][1] - data->points[i + 1][1];

			d_next = sqrt(dx * dx + dy * dy);
		}

		float factor = 1.0;
		if (i == 0)
		{
			float theta_next = fminf(M_PI / 2, _getAlpha(i + 1, data->points));
			factor = 1 + 1.5 * (theta_next * d_next) / (chord + d_next);
		}
		else if (i == n - 2) {
			float theta = fminf(M_PI / 2, _getAlpha(i, data->points));
			factor = 1 + 1.5 * (theta * d_prev) / (d_prev + chord);
		}
		else {
			float theta = fminf(M_PI / 2, _getAlpha(i, data->points));
			float theta_next = fminf(M_PI / 2, _getAlpha(i + 1, data->points));

			factor = 1 + 1.5 * (theta * d_prev) / (d_prev + chord) + 1.5 * (theta_next * d_next) / (chord + d_next);
		}

		data->Foley_Result[i + 1] = data->Foley_Result[i] + chord * factor;

		d_prev = chord;
	}

	data->Foley_Result /= data->Foley_Result[n - 1];
}

float _getAlpha(int i, const std::vector<Ubpa::pointf2>& points) {
	float dx, dy;
	// d_prev
	dx = points[i][0] - points[i - 1][0];
	dy = points[i][1] - points[i - 1][1];
	float d_prev = sqrt(dx * dx + dy * dy);

	// d_next
	dx = points[i + 1][0] - points[i][0];
	dy = points[i + 1][1] - points[i][1];
	float d_next = sqrt(dx * dx + dy * dy);

	// l2
	dx = points[i + 1][0] - points[i - 1][0];
	dy = points[i + 1][1] - points[i - 1][1];
	float l2 = dx * dx + dy * dy;

	float alpha = M_PI - acos((d_prev * d_prev + d_next * d_next - l2 / (2 * d_next * d_prev)));

	return alpha;
}

ImVec2 Lagrange_Interpolation(float t, const Eigen::VectorXf& parameterization, const std::vector<Ubpa::pointf2>& points) {
	int n = points.size();
	for (size_t i = 0; i < n; i++){
		if (t == parameterization[i]) return ImVec2(points[i][0], points[i][1]);
	}

	float x = 0, y = 0;
	for (size_t j = 0; j < n; j++)
	{
		float l = 1.0;
		for (size_t i = 0; i < n; i++) {
			if (j == i) continue;
			l = l * (t - parameterization[i]) / (parameterization[j] - parameterization[i]);
		}

		x = x + points[j][0] * l;
		y = y + points[j][1] * l;
	}

	return ImVec2(x, y);
}

ImVec2 Gauss_Interpolation(float t, const Eigen::VectorXf& parameterization, const std::vector<Ubpa::pointf2>& points, float theta) {
	int n = points.size();
	for (size_t i = 0; i < n; i++) {
		if (t == parameterization[i]) return ImVec2(points[i][0], points[i][1]);
	}
	int m = 2;
	//float theta = 100;

	Eigen::MatrixXf A(n, m+n);//g(x)矩阵A计算
	Eigen::VectorXf v(n);

	for (int i = 0; i < n; i++) {
		v(i) = points[i][1];
		for (int j = m; j <m + n; j++) {
			A(i, j) = (std::exp(-std::pow(points[j-m][0] - points[i][0],2) / (2 * theta * theta)));
		}
		for (int j = 0; j < m; j++)
			A(i, j) = std::pow(points[i][0], j);
	}
	
	//计算Ax=y中的x(向量)
	Eigen::VectorXf a = A.colPivHouseholderQr().solve(v);

	auto interp = [&](float x) {
		float y = a(0);
		for (int j = 1; j < m; ++j)
			y += a(j) * std::pow(x, j);
		for (int j = m; j < m + n; ++j)
			y += a(j) * std::exp(-0.5 * std::pow(x - points[j - m][0], 2) / (2 * theta * theta));
		return ImVec2(x, y);
	};
}

void Draw_Parameterization(CanvasData* data, ImDrawList* draw_list, int num_samples, const ImVec2 origin) {
	if (data->points.empty()) return;

	Eigen::VectorXf T = Eigen::VectorXf::LinSpaced(num_samples, 0, 1);

	// Uniform Parameterization
	if (data->opt_Uniform_Parameterization) {
		std::vector<ImVec2> uniformResult;
		for (size_t i = 0; i < num_samples; i++)
		{ 
			ImVec2 point = Lagrange_Interpolation(T[i], data->Uniform_Result, data->points);
			uniformResult.push_back(ImVec2(origin.x + point.x, origin.y + point.y));
		}
		draw_list->AddPolyline(uniformResult.data(), uniformResult.size(), IM_COL32(64, 128, 255, 255), false, 1.0f);
	}

	// Chordal Parameterization
	if (data->opt_Chordal_Parameterization)
	{
		std::vector<ImVec2> chordalResult;
		T = Eigen::VectorXf::LinSpaced(num_samples, 0, data->Chordal_Result.tail(1)(0));
		for (size_t i = 0; i < num_samples; i++)
		{
			ImVec2 point = Lagrange_Interpolation(T[i], data->Chordal_Result, data->points);
			chordalResult.push_back(ImVec2(origin.x + point.x, origin.y + point.y));
		}
		draw_list->AddPolyline(chordalResult.data(), chordalResult.size(), IM_COL32(128, 255, 255, 255), false, 1.0f);
	}

	// Centripetal Parameterization
	if (data->opt_Centripetal_Parameterization)
	{
		std::vector<ImVec2> centripetalResult;
		T = Eigen::VectorXf::LinSpaced(num_samples, 0, data->Centripetal_Result.tail(1)(0));
		for (size_t i = 0; i < num_samples; i++)
		{
			ImVec2 point = Lagrange_Interpolation(T[i], data->Centripetal_Result, data->points);
			centripetalResult.push_back(ImVec2(origin.x + point.x, origin.y + point.y));
		}
		draw_list->AddPolyline(centripetalResult.data(), centripetalResult.size(), IM_COL32(255, 128, 128, 255), false, 1.0f);
	}

	// Foley-Nielson Parameterization
	if (data->opt_Foley_Parameterization)
	{
		std::vector<ImVec2> foleyResult;
		T = Eigen::VectorXf::LinSpaced(num_samples, 0, data->Foley_Result.tail(1)(0));
		for (size_t i = 0; i < num_samples; i++)
		{
			ImVec2 point = Lagrange_Interpolation(T[i], data->Foley_Result, data->points);
			foleyResult.push_back(ImVec2(origin.x + point.x, origin.y + point.y));
		}
		draw_list->AddPolyline(foleyResult.data(), foleyResult.size(), IM_COL32(255, 64, 64, 255), false, 1.0f);
	}
}