
// Point2Voxel model

#ifndef P2V_MODEL_H__
#define P2V_MODEL_H__

#include <torch/script.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <Eigen/Eigen>

using std::vector;
using std::cout, std::endl;



class P2VModel {

public:
    P2VModel(void){;}
    void loadModel(std::string model_path);

    void predict(const vector<Eigen::Vector3d>& points, const Eigen::Vector3d& query, Eigen::Vector3d& p2v_pred, double& weight);



public:
    torch::jit::script::Module model_;

};


#endif 
