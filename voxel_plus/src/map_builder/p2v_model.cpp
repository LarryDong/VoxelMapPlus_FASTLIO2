
#include "p2v_model.h"
#include "my_ros_debugger.hpp"


#include <iostream>
#include <fstream>


void P2VModel::loadModel(std::string model_path){
    try{
        torch::Device device(torch::kCPU);
        model_ = torch::jit::load(model_path, device);
        model_.eval();
        torch::NoGradGuard no_grad;  // Disable PyTorch's gradient calculation
        std::cout <<"P2VModel loaded: " << model_path << std::endl;
    }
    catch (const c10::Error& e) {
        std::cerr << "error loading the model" << std::endl;
    }
}


void P2VModel::predictP2V(const vector<Eigen::Vector3d>& points, const Eigen::Vector3d& query, Eigen::Vector3d& p2v_pred, double& weight){
    
    my_ros_utility::MyTimer timer;
    timer.tic();

    // [batch=1, N=50, Dim=3]
    const int BATCH = 1;
    const int N = 50;
    const int DIM = 3;

    if (points.size() < N){
        cout <<"Not enough points. Skip." << endl;
        return ;
    }

    // 1. points convert to tensor.
    std::vector<float> flattened_points;
    flattened_points.reserve(BATCH * N * DIM);
    // only reserve 50 points.          // TODO: use random selection.
    for(int i=0; i<N; ++i){
        const auto& p = points[i];
        flattened_points.push_back(p[0]);
        flattened_points.push_back(p[1]);
        flattened_points.push_back(p[2]);
    }
    // create the tensor, as [batch=1, N=50, dim=3];
    torch::Tensor points_tensor = torch::tensor(flattened_points, torch::dtype(torch::kFloat32)).reshape({BATCH, N, DIM});


    // 2. query-point to tensor.
    std::vector<float> p = {float(query[0]), float(query[1]), float(query[2])};
    torch::Tensor p_tensor = torch::tensor(p, torch::dtype(torch::kFloat32)).reshape({BATCH,  DIM});


    // 3. Predict
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(points_tensor);
    inputs.push_back(p_tensor);

    double t1 = timer.toc();        // pre-process time


    timer.tic();
    auto output = model_.forward(inputs);           // Prediction.
    double t2 = timer.toc();        // prediction time

    timer.tic();
    // process output
    assert(output.isTuple());
    const auto& elements = output.toTuple()->elements();

    // Output 1: P2V
    torch::Tensor tensor1 = elements[0].toTensor().contiguous().to(torch::kCPU);
    auto tensor_data1 = tensor1.data_ptr<float>();
    p2v_pred << double(tensor_data1[0]), double(tensor_data1[1]), double(tensor_data1[2]);

    // Output 2: weight.
    torch::Tensor tensor2 = elements[1].toTensor().to(torch::kCPU);
    weight = tensor2.item<float>();

    double t3 = timer.toc();        // post processing time


#ifdef DEBUG_INFO
    cout <<"query point: " << endl;
    cout << p_tensor << endl;

    cout <<"points_tensor: " << endl;
    std::cout << points_tensor << std::endl;

    cout <<"Predicted p2v: " << p2v_pred.transpose() << endl;
    cout <<"Predicted weight: " << weight << endl;

    cout <<"Timer for prediction: 1) data-prepare: " << t1 << ", 2) model-predict: " << t2 << ", 3) post-process: " << t3 << endl;
#endif


    // // Output 3: global feature:
    // torch::Tensor tensor3 = elements[2].toTensor().contiguous().to(torch::kCPU);
    // std::vector<float> vec3(tensor3.data_ptr<float>(),tensor3.data_ptr<float>() + tensor3.numel());


    // // save points for further debug. 
    // // Format: line1: query,  line2-51, points in voxel; 52: p2v_predicted; 53: weight*3
    // using namespace std;
    // ofstream file("/home/larry/featVoxelMap_ws/data/data.csv");
    // if (!file.is_open()) {
    //     cerr << "Error opening file!" << endl;
    //     return ;
    // }
    // file << query[0] << "," << query[1] << "," << query[2] << endl;
    // for(int i=0; i<50; ++i){
    //     auto pt = points[i];
    //     file << pt[0] << "," << pt[1] << "," << pt[2] << endl;
    // }
    // file << p2v_pred[0] << "," << p2v_pred[1] << "," << p2v_pred[2] << endl;
    // file << weight << "," << weight << "," << weight << endl;
    // file.close();

    // std::abort();
}


