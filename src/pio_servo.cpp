#include "pio_servo.hpp"

void DMAUpdateTaskCheckChannels(void *_this)
{
        static_cast<PioServo *>(_this)->CheckDMAChannels();
}



void DMAUpdateTaskCh0(void *_this)
{
    while (true)
    {
        xTaskNotifyWait(0x00, ULONG_MAX, nullptr, portMAX_DELAY);
        static_cast<PioServo *>(_this)->DMAUpdate(0);
    }
}

void DMAUpdateTaskCh1(void *_this)
{
    while (true)
    {
        xTaskNotifyWait(0x00, ULONG_MAX, nullptr, portMAX_DELAY);
        static_cast<PioServo *>(_this)->DMAUpdate(1);
    }
}

void DMAUpdateTaskCh2(void *_this)
{
    while (true)
    {
        xTaskNotifyWait(0x00, ULONG_MAX, nullptr, portMAX_DELAY);
        static_cast<PioServo *>(_this)->DMAUpdate(2);
    }
}

void DMAUpdateTaskCh3(void *_this)
{
    while (true)
    {
        xTaskNotifyWait(0x00, ULONG_MAX, nullptr, portMAX_DELAY);
        static_cast<PioServo *>(_this)->DMAUpdate(3);
    }
}

