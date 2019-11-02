#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function
struct RegistrationCostFunction
{
	RegistrationCostFunction(const Point2D& pointP_, const Point2D& pointQ_, const Weight& weight_)
		: pointP(pointP_), pointQ(pointQ_), weight(weight_)
	{
	}

	template<typename T>
	bool operator()(const T* const deg, const T* const tx, const T* const ty, T* residual) const {
		T rad = *deg * T(M_PI / 180.0);
		T x = T(cos(rad) * pointP.x - sin(rad) * pointP.y) + *tx - T(pointQ.x);
		T y = T(sin(rad) * pointP.x + cos(rad) * pointP.y) + *ty - T(pointQ.y);

		residual[0] = T(weight.w * (x * x + y * y));

		return true;
	}

private:
	const Point2D pointP;
	const Point2D pointQ;
	const Weight weight;

};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// TODO: Read data points and the weights. Define the parameters of the problem
	const std::string file_path_1 = "../data/points_dragon_1.txt";
	const std::string file_path_2 = "../data/points_dragon_2.txt";
	const std::string file_path_weights = "../data/weights_dragon.txt";

	const auto pointsP = read_points_from_file<Point2D>(file_path_1);
	const auto pointsQ = read_points_from_file<Point2D>(file_path_2);
	const auto weights = read_points_from_file<Weight>(file_path_weights);

	// Good initial values make the optimization easier
	const double deg_initial = 1.0;
	const double tx_initial = 1.0;
	const double ty_initial = 1.0;

	double deg = deg_initial;
	double tx = tx_initial;
	double ty = ty_initial;


	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block
	int i = 0;
	for (auto& weight : weights)
	{
		auto& pointP = pointsP[i];
		auto& pointQ = pointsQ[i];

		// Set up the only cost function (also known as residual). This uses
		// auto-differentiation to obtain the derivative (jacobian).
		ceres::CostFunction* cost_function =
			new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1>(new RegistrationCostFunction(pointP, pointQ, weight));
		problem.AddResidualBlock(cost_function, NULL, &deg, &tx, &ty);
		i++;
	}

	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// TODO: Output the final values of the translation and rotation (in degree)
	std::cout << "Initial deg: " << deg_initial << "\ttx: " << tx_initial << "\tty: " << ty_initial << std::endl;
	std::cout << "Final deg: " << deg << "\ttx: " << tx << "\tty: " << ty << std::endl;

	return 0;
}