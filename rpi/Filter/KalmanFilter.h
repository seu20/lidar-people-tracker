#pragma once
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector4d;

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::Matrix;

 
class KalmanFilter {
private:
    VectorXd m_state;
    MatrixXd m_P;
    bool m_initialized;
    uint64_t last_time_ms; //최근 업데이트 한 시간
public:
    KalmanFilter(): 
            m_initialized(false),
            last_time_ms(0) {};

    void init(float x, float y, uint64_t time_ms);
    void setState(const VectorXd& state){ m_state = state; m_initialized = true; };
    void setCovariance(const MatrixXd& covariance){ m_P = covariance; };
    VectorXd getState() const { return m_state; };
    MatrixXd getCovariance() const { return m_P; };

    // 다음 프레임 시점으로 state, P를 예측 (등속 모델)
    void predict(uint64_t time);
    // 매칭된 measurement(x, y)로 state, P를 보정
    void update(float measX, float measY);
};
 