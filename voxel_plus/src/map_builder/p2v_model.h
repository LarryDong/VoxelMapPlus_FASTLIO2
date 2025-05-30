
// Point2Voxel model

#ifndef P2V_MODEL_H__
#define P2V_MODEL_H__

#include <torch/script.h>
#include <torch/torch.h>
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
    // P2VModel() : device_(torch::kCPU)      // device_ should be inited not assigned.
    P2VModel() : device_(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU)      // device_ should be inited not assigned.
    {
        if (device_.is_cuda()){
            std::cout << "CUDA is available! Using GPU." << std::endl;
        }
        else{
            std::cout << "CUDA not available. Using CPU." << std::endl;
        }
    }
    void loadModel(std::string model_path);
    void predictP2V(const vector<Eigen::Vector3d>& points, const Eigen::Vector3d& query, Eigen::Vector3d& p2v_pred, double& weight, std::vector<Eigen::Vector3d>& debug_selected_voxel_points);
    void batchPredictP2V(const vector<vector<Eigen::Vector3d>>& batch_points, const vector<Eigen::Vector3d>& batch_query, 
                            vector<Eigen::Vector3d>& batch_p2v_pred, vector<double>& batch_weight, const int BATCH=8);




public:
    torch::jit::script::Module model_;
    torch::Device device_;

};


#endif 
