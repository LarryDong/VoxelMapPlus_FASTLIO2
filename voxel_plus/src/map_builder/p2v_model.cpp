
#include "p2v_model.h"
#include "my_ros_debugger.hpp"


void P2VModel::loadModel(std::string model_path){
    try{
        model_ = torch::jit::load(model_path);
        model_.eval();
        std::cout <<"P2VModel loaded: " << model_path << std::endl;
    }
    catch (const c10::Error& e) {
        std::cerr << "error loading the model" << std::endl;
    }
}


void P2VModel::predictP2V(const vector<Eigen::Vector3d>& points, const Eigen::Vector3d& query, Eigen::Vector3d& p2v_pred, double& weight){
    
    my_ros_utility::MyTimer timer;
    timer.tic();

    if(points.size()<50){
        cout <<"Not enough points. Skip." << endl;
        return ;
    }


    // 1. points convert to tensor.
    std::vector<std::vector<float>> vv_points;
    vv_points.reserve(points.size());
    // only reserve 50 points.
    for(int i=0; i<50; ++i){
        const auto& p = points[i];
        std::vector<float> v_points;
        v_points.push_back(p[0]);
        v_points.push_back(p[1]);
        v_points.push_back(p[2]);
        vv_points.push_back(v_points);
    }

    // 更安全的points处理方式
    std::vector<float> flattened_points;
    for (const auto& row : vv_points) {
        flattened_points.insert(flattened_points.end(), row.begin(), row.end());
    }
    torch::Tensor points_tensor = torch::tensor(flattened_points, torch::dtype(torch::kFloat32))
        .reshape({1, static_cast<long>(vv_points.size()), static_cast<long>(vv_points[0].size())});

    // 2. query-point to tensor.
    std::vector<float> p = {float(query[0]), float(query[1]), float(query[2])};
    torch::Tensor p_tensor = torch::tensor(p, torch::dtype(torch::kFloat32)).unsqueeze(0);

    // Predict
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(points_tensor);
    inputs.push_back(p_tensor);


    double t1 = timer.toc();
    timer.tic();

    auto output = model_.forward(inputs);

    double t2 = timer.toc();
    timer.tic();

    assert(output.isTuple());

    // process output
    auto tuple_ptr = output.toTuple();
    const auto& elements = tuple_ptr->elements();

    // Output 1: P2V
    torch::Tensor tensor1 = elements[0].toTensor().contiguous().to(torch::kCPU);
    std::vector<float> vec1(tensor1.data_ptr<float>(), tensor1.data_ptr<float>() + tensor1.numel());

    // cout <<"--> P2V: " << endl;
    // for(auto x:vec1)
    //     cout << x << ", ";
    // cout << endl;

    // Output 2: weight.
    torch::Tensor tensor2 = elements[1].toTensor();
    float scalar_value = tensor2.item<float>();
    // cout <<"--> Weight: " << scalar_value << endl;

    // // Output 3: global feature:
    // torch::Tensor tensor3 = elements[2].toTensor().contiguous().to(torch::kCPU);
    // std::vector<float> vec3(tensor3.data_ptr<float>(),tensor3.data_ptr<float>() + tensor3.numel());

    double t3 = timer.toc();

    // cout <<"Timer: data prepare: " << t1 << ", prediction: " << t2 << ", post-process: " << t3 << endl;
}

