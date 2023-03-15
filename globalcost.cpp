#include "settings.h"
// #include "mag_models.h"
#include "utils.h"
#include "file_utils.h"
#include <gflags/gflags.h>

#define RANDOM_SAMPLING 1
#define N 30000

// #define N 30000


DEFINE_string(trial, "", "trial to process");
DEFINE_string(calibration, "", "base calibration file");
DEFINE_string(suffix, "", "suffix");
DEFINE_bool(learn_sensor_offset, true, "learn sensor offset");
DEFINE_bool(learn_ring_offset, true, "learn ring offset");
DEFINE_bool(learn_channel_gain, true, "learn channel gain");
DEFINE_bool(learn_channel_noise, true, "learn noise");
DEFINE_bool(learn_channel_bias, true, "learn bias");
DEFINE_bool(learn_gain, true, "learn gain");
DEFINE_bool(learn_crosstalk, true, "learn crosstalk");



class GlobalCostFunctor
{
public:
	void* operator new(size_t i)
	{
		return _mm_malloc(i, 16);
	}

	void operator delete(void* p)
	{
		_mm_free(p);
	}
	GlobalCostFunctor(const array<DataFrame, N>& data, const CoilCalibrationModel<double>& initial_model, int sensor_offset) : data_(data), initial_model_(initial_model), sensor_offset_(sensor_offset){}

	template <typename T> bool operator()(const T* const p_global_gain, const T* const p_sensor_offset, const T* const p_sensor_rot_offset, 
		const T* const p_ring_offset, const T* const p_ring_rot_offset, const T* const p_per_channel_gain,
		const T* const p_noise, const T* const p_bias, const T* const p_gain,
		const T* const p_crosstalk, T* residual) const
	{
		Eigen::Map<const Eigen::Vector3<T>> sensor_offset(p_sensor_offset);
		Eigen::Map<const Eigen::Quaternion<T>> sensor_rot_offset(p_sensor_rot_offset);

		Eigen::Map<const Eigen::Vector3<T>> ring_offset(p_ring_offset);
		Eigen::Map<const Eigen::Quaternion<T>> ring_rot_offset(p_ring_rot_offset);
		Eigen::Map<const Eigen::Vector3<T>> per_channel_gain(p_per_channel_gain);
		Eigen::Map<const Eigen::Vector3<T>> noise(p_noise);
		Eigen::Map<const Eigen::Vector3<T>> bias(p_bias);
		Eigen::Map<const Eigen::Vector3<T>> gain(p_gain);
		

		// CoilCalibrationModel<T> model = initial_model_;
        CoilCalibrationModel<T> model;

		model.sensor_offset = sensor_offset;
		model.sensor_rot_offset = sensor_rot_offset;
		model.ring_offset = ring_offset;
		model.ring_rot_offset = ring_rot_offset;
		model.global_gain = *p_global_gain;
		model.per_channel_gain = per_channel_gain;
		model.crosstalk = { p_crosstalk[0], p_crosstalk[1], p_crosstalk[2] };

		model.noise = noise;
		model.bias = bias;
		model.gain = gain;

		print_calibration(model);

		Eigen::Array<T, Eigen::Dynamic, 3> mat_sensor_pred = Eigen::Array<T, Eigen::Dynamic, 3>::Zero(N, 3);
		Eigen::Array<T, Eigen::Dynamic, 3> mat_sensor = Eigen::Array<T, Eigen::Dynamic, 3>::Zero(N, 3);

		for (unsigned int frame_idx = 0; frame_idx < N; frame_idx++) {
			Eigen::Vector3<T> pos = data_[frame_idx].ring_pos.cast<T>();
			Eigen::Quaternion<T> rot = data_[frame_idx].ring_q.cast<T>();
			Vector3<T> field;
			Eigen::Vector3<T> sensor_pred = forward_model<T>(pos, rot, model, false, field); // Do not preserve sign

			for (unsigned int i = 0; i < 3; i++) {
				mat_sensor(frame_idx, i) = T(data_[frame_idx].sensors[i+ sensor_offset_]);
				mat_sensor_pred(frame_idx, i) = sensor_pred[i];
				if (ceres::abs(data_[frame_idx].sensors[i + sensor_offset_]) < 0.8)
				{
					T diff = T(data_[frame_idx].sensors[i + sensor_offset_]) - sensor_pred[i];
					residual[frame_idx * 3 + i] = diff;// / T(data_[frame_idx].sensors[i + sensor_offset_] + .001);
				}
				else
				{
					residual[frame_idx * 3 + i] = T(0);
				}
				//residual[i] += diff * diff;
			}
		}
		Eigen::Array<T, 1, 3> sensor_mean = mat_sensor.colwise().mean();
		Eigen::Array<T, 1, 3> sensor_pred_mean = mat_sensor_pred.colwise().mean();

		Eigen::Array<T, 1, 3> mat_sensor_std_dev = ((mat_sensor.rowwise() - sensor_mean).square().colwise().sum() / T(N - 1)).sqrt();
		Eigen::Array<T, 1, 3> mat_sensor_pred_std_dev = ((mat_sensor_pred.rowwise() - sensor_pred_mean).square().colwise().sum() / T(N - 1)).sqrt();

		// https://en.wikipedia.org/wiki/Pearson_correlation_coefficient
		Eigen::Array<T, 1, 3> r = ((mat_sensor.rowwise() - sensor_mean) * (mat_sensor_pred.rowwise() - sensor_pred_mean)).colwise().sum() / (
			(mat_sensor.rowwise() - sensor_mean).square().colwise().sum().sqrt() * (mat_sensor_pred.rowwise() - sensor_pred_mean).square().colwise().sum().sqrt());


		T cost = r.mean();


		// printf("Rs: ");
		// for (unsigned int kk = 0; kk < 3; kk++) {
		// 	printf("%f ", D(r(kk)));
		// }

		printf("\nCost: ");
		// print_float(cost);

		return true;
	}

private:
	const array<DataFrame, N>& data_;
	int sensor_offset_;
	CoilCalibrationModel<double> initial_model_;
};


int main(int argc, char* argv[])
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	google::InitGoogleLogging(argv[0]);

	cout << "STARTED PARSING DATA" << endl;
	vector<DataFrame> data1 = parseCSVFile(PROCESSED_DRECTORY + "resampled", FLAGS_trial, ' ');

	cout << "FINISHED PARSING DATA" << endl;
	for (unsigned int i = 0; i < data1.size(); i++) {
		array<double, NUM_RX * 3> sensor_v;
		sensor_v[0] = data1[i].sensors[0];
		sensor_v[1] = data1[i].sensors[4];
		sensor_v[2] = data1[i].sensors[2];
		sensor_v[3] = data1[i].sensors[1];
		sensor_v[4] = data1[i].sensors[5];
		sensor_v[5] = data1[i].sensors[3];
		sensor_v[6] = data1[i].sensors[6];
		sensor_v[7] = data1[i].sensors[8];
		sensor_v[8] = data1[i].sensors[7];
		data1[i].sensors = sensor_v;
		data1[i].ring_pos *= 1000;
	}

	CalibrationModel<double> global_param = parseCalibratedFile(CERES_DIRECTORY + "calibrate", FLAGS_calibration, ' ');

    std::array<DataFrame, N> sampled_data;
    srand((unsigned int)time(NULL));
	int indices;
	for (int i = 0; i < N; i++) {
        #if RANDOM_SAMPLING
                indices = rand() % data1.size();
        #else
                indices = i;
        #endif
                sampled_data[i] = data1[indices];
            }
    ceres::Problem global_problem;

    GlobalCostFunctor* cost = new GlobalCostFunctor(sampled_data, global_param.coil1, 0);

    ceres::CostFunction* cost_function_global = new ceres::AutoDiffCostFunction<GlobalCostFunctor, 3*N, 1, 3, 4, 3, 4, 3, 3, 3, 3, 3>(cost);
	global_problem.AddResidualBlock(cost_function_global, NULL, &global_param.coil1.global_gain, global_param.coil1.sensor_offset.data(), global_param.coil1.sensor_rot_offset.coeffs().data(),
		global_param.coil1.ring_offset.data(), global_param.coil1.ring_rot_offset.coeffs().data(), global_param.coil1.per_channel_gain.data(), 
		global_param.coil1.noise.data(), global_param.coil1.bias.data(), global_param.coil1.gain.data(),
		global_param.coil1.crosstalk.data());
    return 0;
}

