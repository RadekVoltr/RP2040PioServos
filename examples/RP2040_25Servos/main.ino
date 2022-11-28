#include <Arduino.h>
#include "hardware/dma.h"
#include <FreeRTOS.h>
#include <task.h>
#include <Easing.h>
#include "pio_servo.hpp"


PioServo pioServo(pio1);


void UpdateServoBlock0(void *_this)
{
  EasingFunc<Ease::QuadInOut> *qd;
  EasingFunc<Ease::ExpoInOut> *ed;
  EasingFunc<Ease::CircInOut> *cd;

  float duration = 8000;
  float scale = 1000;
  int start1 = 0;
  int start2 = 0;
  int start3 = 0;
  bool dir1 = true;
  bool dir2 = true;
  bool dir3 = true;

  start1 = millis();
  start2 = millis();
  start3 = millis();

  qd = new EasingFunc<Ease::QuadInOut>();
  qd->duration(duration);
  qd->scale(scale);

  ed = new EasingFunc<Ease::ExpoInOut>();

  ed->duration(duration / 2);
  ed->scale(scale);

  cd = new EasingFunc<Ease::CircInOut>();
  cd->duration(duration / 4);
  cd->scale(scale);

  while (true)
  {
    xTaskNotifyWait(0x00, ULONG_MAX, nullptr, portMAX_DELAY);

    if (start1 + qd->duration() <= millis())
      {
        start1 = millis();
        dir1 = !dir1;
      }
      else
      {
        int res1 = qd->get(millis() - start1);
        if (!dir1)
          res1 = scale - res1;

        pioServo.setServoMicros(0,0,1000 + res1);
        pioServo.setServoMicros(0,1,1000 + res1);
        pioServo.setServoMicros(0,2,1000 + res1);
      }

    if (start2 + ed->duration() <= millis())
      {
        start2 = millis();
        dir2 = !dir2;
      }
      else
      {
        int res1 = ed->get(millis() - start2);
        if (!dir2)
          res1 = scale - res1;

        pioServo.setServoMicros(0,3,1000 + res1);
        pioServo.setServoMicros(0,4,1000 + res1);
        pioServo.setServoMicros(0,5,1000 + res1);
      }

    if (start3 + cd->duration() <= millis())
      {
        start3 = millis();
        dir3 = !dir3;
      }
      else
      {
        int res1 = cd->get(millis() - start3);
        if (!dir3)
          res1 = scale - res1;

        pioServo.setServoMicros(0,6,1000 + res1);
        pioServo.setServoMicros(0,7,1000 + res1);
      }
  }

}

void UpdateServoBlock1(void *_this)
{
  EasingFunc<Ease::QuadInOut> *qd;
  EasingFunc<Ease::ExpoInOut> *ed;
  EasingFunc<Ease::CircInOut> *cd;

  float duration = 10000;
  float scale = 1000;
  int start1 = 0;
  int start2 = 0;
  int start3 = 0;
  bool dir1 = true;
  bool dir2 = true;
  bool dir3 = true;

  start1 = millis();
  start2 = millis();
  start3 = millis();

  qd = new EasingFunc<Ease::QuadInOut>();
  qd->duration(duration);
  qd->scale(scale);

  ed = new EasingFunc<Ease::ExpoInOut>();

  ed->duration(duration / 2);
  ed->scale(scale);

  cd = new EasingFunc<Ease::CircInOut>();
  cd->duration(duration / 4);
  cd->scale(scale);

  while (true)
  {
    xTaskNotifyWait(0x00, ULONG_MAX, nullptr, portMAX_DELAY);

    if (start1 + qd->duration() <= millis())
      {
        start1 = millis();
        dir1 = !dir1;
      }
      else
      {
        int res1 = qd->get(millis() - start1);
        if (!dir1)
          res1 = scale - res1;

        pioServo.setServoMicros(1,0,1000 + res1);
        pioServo.setServoMicros(1,1,1000 + res1);
        pioServo.setServoMicros(1,2,1000 + res1);
      }

    if (start2 + ed->duration() <= millis())
      {
        start2 = millis();
        dir2 = !dir2;
      }
      else
      {
        int res1 = ed->get(millis() - start2);
        if (!dir2)
          res1 = scale - res1;

        pioServo.setServoMicros(1,3,1000 + res1);
        pioServo.setServoMicros(1,4,1000 + res1);
        pioServo.setServoMicros(1,5,1000 + res1);
      }

    if (start3 + cd->duration() <= millis())
      {
        start3 = millis();
        dir3 = !dir3;
      }
      else
      {
        int res1 = cd->get(millis() - start3);
        if (!dir3)
          res1 = scale - res1;

        pioServo.setServoMicros(1,6,1000 + res1);
        pioServo.setServoMicros(1,7,1000 + res1);
      }
  }

}

void UpdateServoBlock2(void *_this)
{
  EasingFunc<Ease::ElasticInOut> *qd;
  EasingFunc<Ease::BounceInOut> *ed;
  EasingFunc<Ease::BackInOut> *cd;

  float duration = 5000;
  float scale = 600;
  int start1 = 0;
  int start2 = 0;
  int start3 = 0;
  bool dir1 = true;
  bool dir2 = true;
  bool dir3 = true;

  start1 = millis();
  start2 = millis();
  start3 = millis();

  qd = new EasingFunc<Ease::ElasticInOut>();
  qd->duration(duration);
  qd->scale(scale);

  ed = new EasingFunc<Ease::BounceInOut>();

  ed->duration(duration / 2);
  ed->scale(scale);

  cd = new EasingFunc<Ease::BackInOut>();
  cd->duration(duration / 4);
  cd->scale(scale);

  while (true)
  {
    xTaskNotifyWait(0x00, ULONG_MAX, nullptr, portMAX_DELAY);

    if (start1 + qd->duration() <= millis())
      {
        start1 = millis();
        dir1 = !dir1;
      }
      else
      {
        int res1 = qd->get(millis() - start1);
        if (!dir1)
          res1 = scale - res1;

        pioServo.setServoMicros(2,0,1000 + res1);
        pioServo.setServoMicros(2,1,1000 + res1);
      }

    if (start2 + ed->duration() <= millis())
      {
        start2 = millis();
        dir2 = !dir2;
      }
      else
      {
        int res1 = ed->get(millis() - start2);
        if (!dir2)
          res1 = scale - res1;

        pioServo.setServoMicros(2,2,1000 + res1);
        pioServo.setServoMicros(2,3,1000 + res1);
      }

    if (start3 + cd->duration() <= millis())
      {
        start3 = millis();
        dir3 = !dir3;
      }
      else
      {
        int res1 = cd->get(millis() - start3);
        if (!dir3)
          res1 = scale - res1;

        pioServo.setServoMicros(2,4,1000 + res1);
        pioServo.setServoMicros(2,5,1000 + res1);
      }
  }

}

void UpdateServoBlock3(void *_this)
{
  EasingFunc<Ease::QuadInOut> *qd;
  EasingFunc<Ease::ExpoInOut> *ed;
  EasingFunc<Ease::CircInOut> *cd;

  float duration = 5000;
  float scale = 1000;
  int start1 = 0;
  int start2 = 0;
  int start3 = 0;
  bool dir1 = true;
  bool dir2 = true;
  bool dir3 = true;

  start1 = millis();
  start2 = millis();
  start3 = millis();

  qd = new EasingFunc<Ease::QuadInOut>();
  qd->duration(duration);
  qd->scale(scale);

  ed = new EasingFunc<Ease::ExpoInOut>();

  ed->duration(duration / 2);
  ed->scale(scale);

  cd = new EasingFunc<Ease::CircInOut>();
  cd->duration(duration / 4);
  cd->scale(scale);

  while (true)
  {
    xTaskNotifyWait(0x00, ULONG_MAX, nullptr, portMAX_DELAY);

    if (start1 + qd->duration() <= millis())
      {
        start1 = millis();
        dir1 = !dir1;
      }
      else
      {
        int res1 = qd->get(millis() - start1);
        if (!dir1)
          res1 = scale - res1;

        pioServo.setServoMicros(3,0,1000 + res1);
      }

    if (start2 + ed->duration() <= millis())
      {
        start2 = millis();
        dir2 = !dir2;
      }
      else
      {
        int res1 = ed->get(millis() - start2);
        if (!dir2)
          res1 = scale - res1;

        pioServo.setServoMicros(3,1,1000 + res1);
      }

    if (start3 + cd->duration() <= millis())
      {
        start3 = millis();
        dir3 = !dir3;
      }
      else
      {
        int res1 = cd->get(millis() - start3);
        if (!dir3)
          res1 = scale - res1;

        pioServo.setServoMicros(3,2,1000 + res1);
      }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(10000);
  Serial.println("Init done");

  TaskHandle_t Handle1;
  xTaskCreate(UpdateServoBlock0, "ServoUpdate1", 1024, NULL, 1, &Handle1);

  auto blck = pioServo.addServoBlock(1, 8, Handle1);
  if (blck < 0)
    Serial.printf("addServoblock error :%d", blck);
  else
    pioServo.runServoBlock(blck);

  TaskHandle_t Handle2;
  xTaskCreate(UpdateServoBlock1, "ServoUpdate2", 1024, NULL, 1, &Handle2);

  blck = pioServo.addServoBlock(9, 8, Handle2);
  if (blck < 0)
    Serial.printf("addServoblock error :%d", blck);
  else
    pioServo.runServoBlock(blck);

  TaskHandle_t Handle3;
  xTaskCreate(UpdateServoBlock2, "ServoUpdate3", 1024, NULL, 1, &Handle3);

  blck = pioServo.addServoBlock(17, 6, Handle3);
  if (blck < 0)
    Serial.printf("addServoblock error :%d", blck);
  else
    pioServo.runServoBlock(blck);

  TaskHandle_t Handle4;
  xTaskCreate(UpdateServoBlock3, "ServoUpdate4", 1024, NULL, 1, &Handle4);

  blck = pioServo.addServoBlock(26, 3, Handle4);
  if (blck < 0)
    Serial.printf("addServoblock error :%d", blck);
  else
    pioServo.runServoBlock(blck);

}

int i = 0;
void loop()
{
  delay(20);
}