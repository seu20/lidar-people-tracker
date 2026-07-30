#include "KalmanFilter.h"
#include <Eigen/Dense>

// -------------------------------------------------- //
// 공분산 값
constexpr bool INIT_ON_FIRST_PREDICTION = true;
constexpr double INIT_POS_STD = 0.3;           
constexpr double INIT_VEL_STD = 2.0;
constexpr double ACCEL_STD = 2.0;
constexpr double LIDAR_POS_STD = 0.05;
// -------------------------------------------------- //

void KalmanFilter::init(float x, float y, uint64_t time_ms)
{
    VectorXd state = Vector4d::Zero();
    state(0) = x;   // 실제 감지된 위치
    state(1) = y;
    // state(2), state(3)은 속도, 아직 모르니까 0

    MatrixXd P = Matrix4d::Zero();
    P(0,0) = INIT_POS_STD * INIT_POS_STD;
    P(1,1) = INIT_POS_STD * INIT_POS_STD;
    P(2,2) = INIT_VEL_STD * INIT_VEL_STD;
    P(3,3) = INIT_VEL_STD * INIT_VEL_STD;

    setState(state);
    setCovariance(P);
    m_initialized = true;   // 반드시 필요
    last_time_ms = time_ms;
}

void KalmanFilter::predict(uint64_t time_ms)
{
     if (!m_initialized) return;

     float dt = (time_ms - last_time_ms) / 1000.0f;
     last_time_ms = time_ms;
     VectorXd state = getState();
     MatrixXd P = getCovariance();

     Eigen::Matrix4d F;
     F << 1, 0, dt, 0,
          0, 1, 0, dt,
          0, 0, 1, 0,
          0, 0, 0, 1;

     // process noise: 모델(등속 가정)이 얼마나 부정확할 수 있는지
     Eigen::Matrix2d Q = Matrix2d::Zero();
     Q(0,0) = (ACCEL_STD*ACCEL_STD);
     Q(1,1) = (ACCEL_STD*ACCEL_STD);

     
     MatrixXd L = MatrixXd(4,2);
     L << (0.5*dt*dt), 0,
          0,(0.5*dt*dt),
          dt,0,
          0,dt;

     state = F * state;
     P = F * P * F.transpose() + L * Q * L.transpose();

     setState(state);
     setCovariance(P);
}

void KalmanFilter::update(float measX, float measY)
{
     VectorXd state = getState();
     MatrixXd P = getCovariance();

     // measurement matrix: state(x,y,vx,vy) 중 위치(x,y)만 측정 가능
     Matrix<double, 2, 4> H;
     H << 1, 0, 0, 0,
     0, 1, 0, 0;

     // measurement noise: 라이다 측정 자체의 오차 (m^2 단위, 튜닝 필요)
     Matrix2d R = Matrix2d::Identity() * (LIDAR_POS_STD * LIDAR_POS_STD);

     Vector2d z(measX, measY);

     // innovation: 측정값과 예측값의 차이
     Vector2d y = z - H * state;

     // innovation covariance
     Matrix2d S = H * P * H.transpose() + R;

     // Kalman gain
     Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();

     // state, covariance 갱신
     state = state + K * y;
     P = (Matrix4d::Identity() - K * H) * P;

     // 최종 state 와 P 저장
     setState(state);
     setCovariance(P);
}