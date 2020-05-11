#pragma once

#include <cassert>
#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilter {
    int _dim_x;
    int _dim_u;
    int _dim_z;
    bool _optional_u;

    VectorXf _x_vec;
    VectorXf _u_vec;
    VectorXf _z_vec;
    MatrixXf _A_mat;
    MatrixXf _B_mat;
    MatrixXf _H_mat;
    MatrixXf _Q_mat;
    MatrixXf _R_mat;
    MatrixXf _P_mat;
    MatrixXf _K_mat;
    MatrixXf _I_mat;
public:
    KalmanFilter(const int dim_x, const int dim_u, const int dim_z, 
            float* x, float* A, float* B, 
            float* H, float* Q, float* R,
            float* P): 
        _dim_x(dim_x), _dim_u(dim_u), _dim_z(dim_z) {
        assert(_dim_x > 0 && _dim_z > 0);
    
        _x_vec = Map<VectorXf>(x, _dim_x);
        _optional_u = _dim_u > 0;
        if (_optional_u) {
            _u_vec = VectorXf(_dim_u);
            for(int i = 0; i < _dim_x; ++i) _u_vec(i) = 0;
            _B_mat = Map<MatrixXf>(B, _dim_x, _dim_u);

        }
        _z_vec = VectorXf(_dim_z);
        for(int i = 0; i < _dim_z; ++i) _z_vec(i) = 0;

        _A_mat = Map<MatrixXf>(A, _dim_x, _dim_x);
        _H_mat = Map<MatrixXf>(H, _dim_z, _dim_x);
        _Q_mat = Map<MatrixXf>(Q, _dim_x, _dim_x);
        _R_mat = Map<MatrixXf>(R, _dim_z, _dim_z);
        _P_mat = Map<MatrixXf>(P, _dim_x, _dim_x);
        _K_mat = MatrixXf(_dim_x, _dim_z);
        _I_mat = MatrixXf::Identity(_dim_x, _dim_x);
    }

    ~KalmanFilter() {}
    void init_state(const float* x) {
        for(int i = 0; i < _dim_x; ++i) {
            _x_vec(i) = x[i];
        }
    }

    const float* get_state() const {
        return _x_vec.data();
    }

    const int get_state_dim() const {
        return _dim_x;
    }
    const int get_measure_dim() const {
        return _dim_z;
    }

    void predict(const float* u) {
        /// update the state
        if (_optional_u) {
            assert(u);
            for(int i = 0; i < _dim_x; ++i) _u_vec(i) = u[i];
            _x_vec = _A_mat * _x_vec + _B_mat * _u_vec;
        } else {
            _x_vec = _A_mat * _x_vec; 
        }
        /// update the covariance
        _P_mat = _A_mat * _P_mat * _A_mat.transpose() + _Q_mat;  
    }
    void update(const float* z) {
        assert(z);
        for(int i = 0; i < _dim_z; ++i) _z_vec(i) = z[i];

        /// update the gain
        _K_mat = _P_mat * _H_mat.transpose() * (
                _H_mat * _P_mat * _H_mat.transpose() + _R_mat).inverse(); 

        /// update the optimal state
        _x_vec = _x_vec + _K_mat * (_z_vec - _H_mat * _x_vec);

        _P_mat = (_I_mat - _K_mat * _H_mat) * _P_mat;

    }


};
/// end of this file
