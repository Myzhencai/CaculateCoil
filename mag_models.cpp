#include "mag_models.h"

// int main()
// {
//     CoilCalibrationModel<double> frame;
//     frame.global_gain = 0;
//     frame.gain = Eigen::Vector3d(2.0, 3.0, 4.0);
//     frame.per_channel_gain = Eigen::Vector3d(2.0, 3.0, 4.0);
//     frame.bias = Eigen::Vector3d(2.0, 3.0, 4.0);
//     frame.noise = Eigen::Vector3d(2.0, 3.0, 4.0);
//     frame.sensor_offset = Eigen::Vector3d(2.0, 3.0, 4.0);
//     frame.sensor_rot_offset = Eigen::Quaterniond(2.0, 3.0, 4.0,5.0);
//     frame.ring_offset = Eigen::Vector3d(2.0, 3.0, 4.0);
//     frame.ring_rot_offset = Eigen::Quaterniond(2.0, 3.0, 4.0,5.0);
//     frame.crosstalk = {2.0, 3.0, 4.0};
//     frame.base_sensor_offset = Eigen::Vector3d(2.0, 3.0, 4.0);
//     frame.base_sensor_rot_offset = Eigen::Quaterniond(2.0, 3.0, 4.0,5.0);
//     print_calibration(frame);
//     Eigen::Vector3d ring_pos = Eigen::Vector3d(2.0, 3.0, 4.0);
//     Eigen::Quaterniond ring_rot = Eigen::Quaterniond(1.0,2.0,3.0,4.0);
//     Vector3<double>out_field;
//     bool preserve_sign =false;
//     forward_model(ring_pos,ring_rot,frame,preserve_sign,out_field);
//     return 0;
// }

