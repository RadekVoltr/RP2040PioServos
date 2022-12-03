#include <pio_servo.hpp>


#include <Arduino.h>
#include "hardware/dma.h"
#include <FreeRTOS.h>
#include <task.h>

//create PioServo management on PIO1
//PIO0/SM0 is used by system and so you can use only 3 SMs (and so only 24 servos in 3 blocks)
//PIO1 have all 4 SMs available
PioServo pioServo(pio1);

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

void setup()
{
  Serial.begin(115200);

  //wait for serial connection
  //REMOVE THIS FOR REAL USE!
  delay(5000);

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
}

void loop()
{
  //PIO servos are run with DMA/Tasks so loop is not needed at all
  delay(10);
}
