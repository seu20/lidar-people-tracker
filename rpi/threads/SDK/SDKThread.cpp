#include "SDKThread.h"
#include <stdexcept>

void *SDKThread::threadfunc(void *arg)
{
    static_cast<SDKThread*>(arg)->run();
    return nullptr;
}

void SDKThread::run()
{
    while(running_)
    {
        try{
            sensor_->getScan();
        }catch(const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            usleep(200 * 1000);   // 200ms 쉬고 재시도
        }

    }
}

SDKThread::SDKThread(const std::string &port, int baudrate, Lidar *sensor)
    : sensor_(sensor),
      port_(port),
      baudrate_(baudrate),
      running_(false)
{

}

SDKThread::~SDKThread()
{
    if (running_)
    {
        stop();
    }
}

bool SDKThread::start()
{
    if (!sensor_->init(port_, baudrate_))
    {
        std::cerr << "sdk initialization failed" << std::endl;
        return false;
    }
    running_ = true;
    if(pthread_create(&thread_id_, nullptr, &SDKThread::threadfunc, this) != 0)
    {
        std::cerr << "sdk thread not create!" << std::endl;
        running_ = false;
        return false;
    }
    return true;
}

void SDKThread::stop()
{
    if (!running_)   return;
    running_ = false;
    pthread_join(thread_id_, nullptr);
}
