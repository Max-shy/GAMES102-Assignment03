# GAMES102-Assignment03
GAMES作业3，实现曲线参数化

## Assignment 03

Assignment 3 requires a single parameter curve to fit an arbitrary sequence of ordered points on a plane. 

I need to use four parameterization methods for the ordered points.

- Equidistant/Uniform parameterization
- Chordal parameterization
- Centripetal parameterization
- Foley parameterization



### Equidistant/Uniform parameterization

Uniform parameterization is one of the easiest parameterization methods. According to the number of sampling points, parameter t is evenly distributed in the interval [0,1].


```CPP
void Update_Uniform_Parameterization(CanvasData* data) {
	int n = data->points.size();
	data->Uniform_Result = Eigen::VectorXf::LinSpaced(n, 0, 1);//[0,1]等距分成n份，等距离参数化
}
```

Because of the uniform sampling, there will be bumps between the closest points.


### Chordal parameterization

Chordal parameterization determines the distribution of parameter **t** according to the spacing between adjacent sampling points.



```CPP
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
```




### Centripetal parameterization

Centripetal parameterization also determines the distribution of parameter **t** according to the spacing between adjacent sampling points and performs the square root operation based on Chordal parameterization

```CPP
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
```




### Foley parameterization

Foley Parameterization uses the function, Nielson, to measure the distance between adjacent sample points.

```CPP
void Update_Foley_Parameterization(CanvasData* data) {
	int n = data->points.size();
	data->Foley_Result = Eigen::VectorXf::Zero(n);
	if (n == 1 || n == 2){
		data->Foley_Result(n - 1) = 1;
		return;
	}

	float d_prev = 0, d_next = 0;
	for (size_t i = 0; i < n - 1; i++){
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
		if (i == 0){
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
```


The curve was fitted using Lagrange polynomials in Assignment 3.

code: [Max-shy/GAMES102-Assignment03](https://github.com/Max-shy/GAMES102-Assignment03)
