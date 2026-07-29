#include "SensorWorker.h"

//라이다 센서
Lidar::Lidar()
{
    pthread_mutex_init(&mtx_, nullptr);
    pthread_cond_init(&cv_, nullptr);
}
bool Lidar::init(const std::string &port, int baudrate)
{
    ////////////////// 문자열 프로퍼티 설정 //////////////////
    lidar.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());

    std::string ignore_array;
    ignore_array.clear();
    lidar.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), 
                      ignore_array.size());

                

    ////////////////// 정수 프로퍼티 설정 //////////////////
    int optval = baudrate;
    lidar.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));

    // X4 Pro는 삼각측량(triangulation) 방식 라이다
    optval = TYPE_TRIANGLE;
    lidar.setlidaropt(LidarPropLidarType, &optval, sizeof(int));


    optval = YDLIDAR_TYPE_SERIAL;                              // 추가: 시리얼(UART) 방식 명시
    lidar.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));

    optval = 5;                                                 // 추가: X4 Pro 샘플레이트 5K
    lidar.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));

    /// abnormal count
    optval = 4;
    lidar.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

    // ★ intensity 비트 수를 명시하면 SDK가 16→8→0 자동 탐색을 하지 않는다.
    //   (그 탐색 과정에서 나오던 checksum error 가 사라진다)
    optval = 10;
    lidar.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

    
    ////////////////// 실수(float) 프로퍼티 //////////////////
    float scan_freq = 10.0f;                                    // 추가: 스캔 주파수 (6~12Hz 범위 내)
    lidar.setlidaropt(LidarPropScanFrequency, &scan_freq, sizeof(float));

    // bool 프로퍼티
    bool single_channel = true;                                // 추가: X4 Pro는 싱글채널 아님
    lidar.setlidaropt(LidarPropSingleChannel, &single_channel, sizeof(bool));

    // SDK 초기화
    bool ret = lidar.initialize();
    if (!ret) {
        std::cerr << "[Lidar] initialize failed: " << lidar.DescribeError() << std::endl;
        return false;
    }

    // 모터 켜고 스캔 시작
    ret = lidar.turnOn();
    if (!ret) {
        std::cerr << "[Lidar] turnOn failed: " << lidar.DescribeError() << std::endl;
        return false;
    }

    return true;
};

void Lidar::setScan(std::vector<SensorPoint> points)
{
    pthread_mutex_lock(&mtx_);
    points_ = std::move(points);
    pthread_mutex_unlock(&mtx_);
    notifyDataReady();  // SDK가 setter 함수 호출 후 데이터가 준비되었음을
}

void Lidar::getScan()
{
    LaserScan scan;
    bool ret = lidar.doProcessSimple(scan);

    if (!ret) {
        throw std::runtime_error("SDK Not read!");   // SDK 스캔 실패 - 에러는 상위(LidarThread)에서 로그 남기게
    }

    std::vector<SensorPoint> points;
    points.reserve(scan.points.size());

    for (auto& p : scan.points) {
        points.push_back({p.angle, p.range});   // (각도, 거리) 쌍으로 저장
    }

    setScan(points);   // 이미 짜신 setScan() 재사용 - mutex 잠그고 저장 + notify
}

std::vector<SensorPoint> Lidar::getData()    // GridThread 에서 받을때
{
    pthread_mutex_lock(&mtx_);
    std::vector<SensorPoint> copy = points_;
    pthread_mutex_unlock(&mtx_);
    return copy;
}


void Lidar::waitForData()
{
    pthread_mutex_lock(&mtx_);
    while (!data_ready_) {                  // spurious wakeup 대비 while로 재확인
        pthread_cond_wait(&cv_, &mtx_);
    }
    data_ready_ = false;
    pthread_mutex_unlock(&mtx_);
}

void Lidar::notifyDataReady()
{
    pthread_mutex_lock(&mtx_);
    data_ready_ = true;
    pthread_cond_signal(&cv_);
    pthread_mutex_unlock(&mtx_);
}
