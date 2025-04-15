
#include "p2v_model.h"
#include "my_ros_debugger.hpp"


#include <iostream>
#include <fstream>


void P2VModel::loadModel(std::string model_path){
    try{
        torch::Device device(torch::kCPU);
        model_ = torch::jit::load(model_path, device);
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


    //////////// TODO: save points, and query-point into file, and call python's code to compare the output.
    //////////// TODO: save points, and query-point into file, and call python's code to compare the output.
    //////////// TODO: save points, and query-point into file, and call python's code to compare the output.
    //////////// TODO: save points, and query-point into file, and call python's code to compare the output.
    //////////// TODO: save points, and query-point into file, and call python's code to compare the output.
    //////////// TODO: save points, and query-point into file, and call python's code to compare the output.
    //////////// TODO: save points, and query-point into file, and call python's code to compare the output.
    //////////// TODO: save points, and query-point into file, and call python's code to compare the output.
    //////////// TODO: save points, and query-point into file, and call python's code to compare the output.
    //////////// TODO: save points, and query-point into file, and call python's code to compare the output.


    // 更安全的points处理方式
    std::vector<float> flattened_points;
    for (const auto& row : vv_points) {
        flattened_points.insert(flattened_points.end(), row.begin(), row.end());
    }
    torch::Tensor points_tensor = torch::tensor(flattened_points, torch::dtype(torch::kFloat32))
        .reshape({1, static_cast<long>(vv_points.size()), static_cast<long>(vv_points[0].size())});

    // cout <<"-----------" << endl;
    // for(auto t: flattened_points)
    //     cout << t << ",";
    // cout << endl;
    // cout << "-----------" << endl;



    // 2. query-point to tensor.
    std::vector<float> p = {static_cast<float>(query[0]), static_cast<float>(query[1]), static_cast<float>(query[2])};
    torch::Tensor p_tensor = torch::tensor(p, torch::dtype(torch::kFloat32)).unsqueeze(0);


    // cout <<"query point: " << endl;
    // cout << p_tensor << endl;

    // cout <<"points_tensor: " << endl;
    // std::cout << points_tensor << std::endl;


    // Predict
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(points_tensor);
    inputs.push_back(p_tensor);


    double t1 = timer.toc();
    timer.tic();

    auto output = model_.forward(inputs);

    torch::NoGradGuard no_grad;  // 临时禁用梯度


    double t2 = timer.toc();
    timer.tic();

    assert(output.isTuple());

    // process output
    auto tuple_ptr = output.toTuple();
    const auto& elements = tuple_ptr->elements();

    // Output 1: P2V
    torch::Tensor tensor1 = elements[0].toTensor().contiguous().to(torch::kCPU);
    std::vector<float> vec1(tensor1.data_ptr<float>(), tensor1.data_ptr<float>() + tensor1.numel());

    p2v_pred[0] = double(vec1[0]);
    p2v_pred[1] = double(vec1[1]);
    p2v_pred[2] = double(vec1[2]);




    // cout <<"--> P2V: " << endl;
    // for(auto x:vec1)
    //     cout << x << ", ";
    // cout << endl;

    // Output 2: weight.
    torch::Tensor tensor2 = elements[1].toTensor();
    float scalar_value = tensor2.item<float>();
    weight = scalar_value;

    // cout <<"--> Weight: " << scalar_value << endl;

    // // Output 3: global feature:
    // torch::Tensor tensor3 = elements[2].toTensor().contiguous().to(torch::kCPU);
    // std::vector<float> vec3(tensor3.data_ptr<float>(),tensor3.data_ptr<float>() + tensor3.numel());

    double t3 = timer.toc();

    // cout <<"Timer: data prepare: " << t1 << ", prediction: " << t2 << ", post-process: " << t3 << endl;

    // cout <<"Predict p2v: " << p2v_pred.transpose() << endl;
    // cout << "Predict weight: " << weight << endl;
    

    // // save points for further debug.
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



// void P2VModel::predictP2V(const vector<Eigen::Vector3d>& points, const Eigen::Vector3d& query, Eigen::Vector3d& p2v_pred, double& weight){

//     std::vector<float> p;
//     p.push_back(float(query[0]));
//     p.push_back(float(query[1]));
//     p.push_back(float(query[2]));
//     torch::Tensor p_tensor = torch::tensor(p, torch::dtype(torch::kFloat32)).unsqueeze(0);

//     std::vector<std::vector<float>> vv_points;
//     for(int i=0; i<50; ++i){
//         const auto& p = points[i];
//         std::vector<float> v_points;
//         v_points.push_back(float(p[0]));
//         v_points.push_back(float(p[1]));
//         v_points.push_back(float(p[2]));
//         vv_points.push_back(v_points);
//     }
//     std::vector<std::vector<float>> vv_points_2(vv_points.begin(), vv_points.end());
//     std::vector<float> flattened_points;
//     for (const auto& row : vv_points_2) {
//         flattened_points.insert(flattened_points.end(), row.begin(), row.end());
//     }
//     torch::Tensor points_tensor = torch::tensor(flattened_points, torch::dtype(torch::kFloat32)).reshape({1, static_cast<long>(vv_points_2.size()), static_cast<long>(vv_points_2[0].size())});



//     // Test output:
//     std::vector<torch::jit::IValue> inputs;
//     inputs.push_back(points_tensor);
//     inputs.push_back(p_tensor);

//     cout <<"p_tensor: " << p_tensor << endl;
//     cout << "points_tensor: " << points_tensor << endl;

//     // 修改后的前向传播部分
//     try {
//         // 先不假设输出是元组，直接获取输出
//         auto output = model_.forward(inputs);
//         assert(output.isTuple());
            
//         auto tuple_ptr = output.toTuple();
//         const auto& elements = tuple_ptr->elements();

//         // Output 1: P2V
//         torch::Tensor tensor1 = elements[0].toTensor().contiguous().to(torch::kCPU);
//         std::vector<float> vec1(tensor1.data_ptr<float>(), tensor1.data_ptr<float>() + tensor1.numel());

//         cout <<"--> P2V: " << endl;
//         for(auto x:vec1)
//             cout << x << ", ";
//         cout << endl;

//         // Output 2: weight.
//         torch::Tensor tensor2 = elements[1].toTensor();
//         float scalar_value = tensor2.item<float>();
//         cout <<"--> Weight: " << scalar_value << endl;

//         // Output 3: global feature:
//         torch::Tensor tensor3 = elements[2].toTensor().contiguous().to(torch::kCPU);
//         std::vector<float> vec3(tensor3.data_ptr<float>(),tensor3.data_ptr<float>() + tensor3.numel());

//     } catch (const c10::Error& e) {
//         std::cerr << "Error during inference: " << e.what() << "\n";
//         return ;
//     }


//     // save points for further debug.
//     using namespace std;
//     ofstream file("/home/larry/featVoxelMap_ws/data/data.csv");
//     if (!file.is_open()) {
//         cerr << "Error opening file!" << endl;
//         return ;
//     }
//     file << query[0] << "," << query[1] << "," << query[2] << endl;
//     for(int i=0; i<50; ++i){
//         auto pt = points[i];
//         file << pt[0] << "," << pt[1] << "," << pt[2] << endl;
//     }
//     file << p2v_pred[0] << "," << p2v_pred[1] << "," << p2v_pred[2] << endl;
//     file << weight << "," << weight << "," << weight << endl;
//     file.close();

//     std::abort();


//     std::abort();
// }


