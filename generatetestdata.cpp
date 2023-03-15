#include "settings.h"
// #include "mag_models.h"
#include "utils.h"
#include "file_utils.h"
#include <gflags/gflags.h>
#include <fstream>

#define RANDOM_SAMPLING 1

#define N 30000


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


void solve_global_model(const vector<DataFrame>& rawData, CoilCalibrationModel<double>& coil_param, CoilCalibrationModel<double>& coil_param1,CoilCalibrationModel<double>& coil_param2)
{
	Eigen::Array<double, Eigen::Dynamic, 3> mat_sensor_pred = Eigen::Array<double, Eigen::Dynamic, 3>::Zero(N, 3);
	Eigen::Array<double, Eigen::Dynamic, 3> mat_sensor = Eigen::Array<double, Eigen::Dynamic, 3>::Zero(N, 3);

	fstream fout;
	fout.open("/home/gaofei/CaculateCoil/csvfile/hello.csv", ios::out);
	for (unsigned int i = 0; i < rawData.size(); i++) {
		Eigen::Vector3<double> pos = rawData[i].ring_pos.cast<double>();
		Eigen::Quaternion<double> rot = rawData[i].ring_q.cast<double>();
		Vector3<double> field;
		Eigen::Vector3<double> sensor_pred = forward_model<double>(pos, rot, coil_param, false, field); 
		Eigen::Vector3<double> sensor_pred1 = forward_model<double>(pos, rot, coil_param1, false, field); 
		Eigen::Vector3<double> sensor_pred2 = forward_model<double>(pos, rot, coil_param2, false, field); 

		fout << sensor_pred[0]<< " " << sensor_pred[1]<< " " << sensor_pred[2]<< " " << sensor_pred1[0]<< " " << sensor_pred1[1]
		<< " " << sensor_pred1[2]<< " " << sensor_pred2[0]<< " " << sensor_pred2[1]<< " " << sensor_pred2[2];
		fout << " " << rawData[i].ring_pos[0] << " " << rawData[i].ring_pos[1] << " " << rawData[i].ring_pos[2];
		fout << " " << rawData[i].ring_q.w()<< " " << rawData[i].ring_q.x() << " " << rawData[i].ring_q.y()<< " " << rawData[i].ring_q.z();
		fout << "\n";
	}
	fout.close();
}

int main(int argc, char* argv[])
{
	cout << "STARTED PARSING DATA" << endl;
	vector<DataFrame> data1 = parseCSVFile(PROCESSED_DRECTORY + "resampled", FLAGS_trial, ',');

	cout << "FINISHED PARSING DATA" << endl;

	CalibrationModel<double> global_param = parseCalibratedFile(CERES_DIRECTORY + "calibrate", FLAGS_calibration, ' ');
    print_calibration(global_param.coil1);
	solve_global_model(data1, global_param.coil1,global_param.coil2,global_param.coil3);
	// cout << "Coil 1 Global opt finished";
	// solve_global_model(data1, global_param.coil2, 3);
	// cout << "Coil 2 Global opt finished";
	// solve_global_model(data1, global_param.coil3, 6);
	// cout << "Coil 3 Global opt finished";

	return 0;
}
