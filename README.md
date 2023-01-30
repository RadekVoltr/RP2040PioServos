# RP2040PioServos

### Servo library for up to 25 (32 is theoretical maximum) servos on Raspberry Pico.

<br>

## Principles

Library utilizes PIO, SMs and DMA transfers for driving up to 8 servos per each SM. Code using similar approach as old FM RC transmitters/receivers and so servo PWMs follows.

One SM need 9 32bit values for 8 servos (at maximum) and in this 9 values you can find 8 times combination of pin mask and delay (in microseconds). 9th parameter have pin mas to 0 and contains only delay needed to spend time in cycle to be 20ms total lenght.

All data are prepared in one of two buffers. DMA reading data from buffer. In DMA IRQ will code switch between buffers (similar way as framebuffer on video systems), task DMA to send next frame and wake up task which will prepare data for next round.

Due 20ms timing and buffer switching can be whole code CPU/thread safe even without slow locking. Library uses spinlock for keeping buffer modifications safe (spinlock is 100x faster than mutex/binary semaphore).




<br>
<br>

## Changelog

### 0.9.1
- library now eat one free spinlock
- setServoMicros and reading data is now secured by spinlock
- default servo value set in constructor is now 0. Most servos stay disabled with zero but NOT ALL. Test your servos first!!!
- value can be changed before calling addServoBlock. addServoBlock will send set value to the servos immediatelly.



<br>
<br>

## Usage

Needed libraries include :

```

#include <pio_servo.hpp>
#include <Arduino.h>
#include "hardware/dma.h"
#include <FreeRTOS.h>
#include <task.h>
```

Select proper PIO. PIO0/SM0 is used by system and so you can use only 3 SMs (and so only 24 servos in 3 blocks).PIO1 have all 4 SMs available.

Create PioServo management on PIO1 :
```
PioServo pioServo(pio1);
```

Support task for servo values update. Task will sleep for about 20ms (one servo PPM frame). Buffer switching is processed behind so you just need to call one of 2 versions of setServoMicros with proper value inside of task

```
int servo_micros = 500;
int blck;

//update servo data task
void UpdateServoBlock0(void *_this)
{
  while (true)
  {
    //task will sleep for about 20ms (one servo PPM frame)
    xTaskNotifyWait(0x00, ULONG_MAX, nullptr, portMAX_DELAY);
    //task is wake up after DMA processing start sending data
    //so system sending servo data in same time as you are updating data for next frame
    for (int i = 0;i<8;i++){
      //setServoMicros - there is two version of this fce. 
      //One with block specification and then are pins counted from 0 to count
      //Without block specification and then you need use pin number on raspberry pico
      pioServo.setServoMicros(blck,i,servo_micros); 
      }
    servo_micros = servo_micros+10;
    if (servo_micros > 2000)
      servo_micros = 500;

  }
}
```

Setup and initialization of servo block is done in setup() function.


```
void setup()
{
...
  //Create servo update task
  //this task will be called on free CPU and should update data in servo buffer
  //task needs to update data in less than 20ms (which is plenty of time)
  TaskHandle_t Handle1;
  xTaskCreate(UpdateServoBlock0, "ServoUpdate1", 1024, NULL, 1, &Handle1);

  //Allocate servo block
  //Servo block starts on pin 1 and contains 8 servos
  //8 is maximum count
  //each block allocates one SM so 4 is maximum (in case that nothing else using SMs on selected PIO)
  //all used pins must be in sequence
  //WARNING : Pin 0 is captured by pico hw framework and can't be used for servo for now
  blck = pioServo.addServoBlock(1, 8, Handle1);
  if (blck < 0) {
    Serial.printf("addServoblock error :%d", blck);
  } else {
    pioServo.runServoBlock(blck);
  }
...
}
```

<br>

Loop is not necessary at all. You can put there your code or just delay.

