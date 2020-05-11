#include "kalman_filter.hpp"
#include <vector>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;

vector<string> split_str(string& str, string pattern) {
    vector<string> res;
    if (pattern.empty()) return res;
    size_t start = 0, index = str.find_first_of(pattern, 0);
    while (index != str.npos) {
        if (start != index) res.push_back(str.substr(start, index - start));
        start = index + 1;
        index = str.find_first_of(pattern, start);
    }
    if (!str.substr(start).empty()) res.push_back(str.substr(start));
    return res;
}

int main(int argc, char* argv[]) {
    const int dim_x = 1, dim_u = 1, dim_z = 1;
    float *init_x;
    float *A, *B, *H;
    float *Q, *R, *P;

    init_x = new float[dim_x * dim_x];
    for(int i = 0; i < dim_x*dim_x; ++i) init_x[i] = 0;
    A = new float[dim_x * dim_x];
    B = new float[dim_x * dim_u];
    H = new float[dim_z * dim_x];
    Q = new float[dim_x * dim_x];
    R = new float[dim_z * dim_z];
    P = new float[dim_x * dim_x];


    H[0] = 1;
    A[0] = 1;
    float dt = 0.02;
    B[0] = dt;

    Q[0] = 0.00006; 
    R[0] = 0.05; 
    P[0] = 0.; 
    int N;

    vector<float> step;
    vector<float> m_distance, m_speed, r_distance;

    ifstream file("data.txt");
    string line;
    for(int i = 0; getline(file, line); ++i) {
        vector<string> tmp = split_str(line, ",");
        if(i == 0) {
            for(auto&e :tmp) step.push_back(atof(e.c_str()));
        } else if(i == 1) {
            for(auto&e :tmp) m_speed.push_back(atof(e.c_str()));
        } else if(i == 2) {
            for(auto&e :tmp) r_distance.push_back(atof(e.c_str()));
        } else if(i == 3) {
            for(auto&e :tmp) m_distance.push_back(atof(e.c_str()));
        }
    }
    N = m_speed.size();

    vector<float> opt_distance(N);

    KalmanFilter kf (dim_x, dim_u, dim_z, init_x, A, B, H, Q, R, P);

    for(int i = 0; i < N; ++i) {
        kf.predict(&m_speed[i]);
        kf.update(&m_distance[i]);
        opt_distance[i] = kf.get_state()[0];
    }

    ofstream out_file;
    out_file.open("predict.txt");

    for(int i = 0; i < N; ++i) {
        out_file << opt_distance[i] << ",";
    }
    out_file.close();






    delete [] init_x;
    delete [] A;
    delete [] B;
    delete [] H;
    delete [] Q;
    delete [] R;
    delete [] P;

    return 0;
}
