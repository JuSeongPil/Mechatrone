#include <Glob_def.h>

float Norm(std::vector<float>);
float dot(std::vector<float>, std::vector<float>);
std::vector<float> emult(std::vector<float>, std::vector<float>);
std::vector<float> mult_var(std::vector<float>, float);
std::vector<float> eplus(std::vector<float>, std::vector<float>);
std::vector<float> eminus(std::vector<float>, std::vector<float>);
std::vector<std::vector<float>> Multiply(std::vector<std::vector<float>>,
                                         std::vector<std::vector<float>>);

std::vector<double> llh2ecef(std::vector<double>);
std::vector<double> ecef2llh(std::vector<double>);
std::vector<float> ecef2enu(std::vector<double>, std::vector<double>);
std::vector<float> ecef2en(std::vector<double>, std::vector<double>);

float PointDist(std::vector<float>, std::vector<float>, std::vector<float>);
std::vector<std::vector<float>> Outer(std::vector<float>,std::vector<float>);
std::vector<std::vector<float>> F_Circle(std::vector<float>, std::vector<float>, std::vector<float>);
bool ChkRange(std::vector<float>, std::vector<float>, std::vector<float>);
std::vector<std::vector<float>> MakePath(std::vector<float>, std::vector<float>, std::vector<float>, float);

float dist_pose(std::vector<float>d, geometry_msgs::PoseStamped);
