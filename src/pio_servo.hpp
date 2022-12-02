#ifndef SRC_PIOSERVO_H_
#define SRC_PIOSERVO_H_
#include <Arduino.h>

#include "hardware/dma.h"
#include "hardware/clocks.h"
#include <FreeRTOS.h>
#include <task.h>
#include <string.h>
#include <climits>
#include "multi_servo.pio.h"

#ifndef MAX_SERVO_SETS
#define MAX_SERVO_SETS 4
#endif

#define MAX_DMA_CHANNELS 12
#define MAX_SERVOS 8

void DMAUpdateTaskCheckChannels(void *_this);

void DMAUpdateTaskCh0(void *_this);
void DMAUpdateTaskCh1(void *_this);
void DMAUpdateTaskCh2(void *_this);
void DMAUpdateTaskCh3(void *_this);
void __not_in_flash_func(DMAIrq0Handler)();

struct servoSet
{
    TaskHandle_t xDMAUpdate;
    TaskHandle_t xExternalUpdate;

    uint16_t servo_micros[MAX_SERVOS];
    unsigned long servo_buffer_a[MAX_SERVOS + 1];
    unsigned long servo_buffer_b[MAX_SERVOS + 1];
    bool servo_buffer;
    int8_t dmaChannel;
    uint8_t pin_start;
    uint8_t pin_count;
    int SM;
};

class PioServo
{
private:
    int8_t dmaChannelBlock[MAX_DMA_CHANNELS];
    int8_t last_block;
    bool interrupt_active;
    bool dma_validation_active;

    PIO pio;
    int offset;
    servoSet ServoSet[MAX_SERVO_SETS];

public:
    static inline PioServo *self = nullptr;

    PioServo(PIO Pio)
    {
        pio = Pio;
        offset = -1;
        last_block = -1;
        interrupt_active = false;
        dma_validation_active = false;

        self = this;
        for (size_t i = 0; i < MAX_DMA_CHANNELS; i++)
            dmaChannelBlock[i] = -1;

        for (size_t i = 0; i < MAX_SERVO_SETS; i++)
        {
            ServoSet[i].servo_buffer = true;
            for (size_t y = 0; y < 8; y++)
                ServoSet[i].servo_micros[y] = 1500;
        }
    };

    ~PioServo(){/* noop */};

    inline int8_t getDMABlock(int8_t Channel)
    {
        if (Channel >= MAX_DMA_CHANNELS)
            return -1;
        else
            return dmaChannelBlock[Channel];
    }

    long DMAIrq[MAX_SERVO_SETS];

    inline void setServoMicros(int8_t pin, int16_t micros)
    {
        for (size_t i = 0; i < MAX_SERVO_SETS; i++)
            if (pin >= ServoSet[i].pin_start && pin < ServoSet[i].pin_start + ServoSet[i].pin_count)
                {
                    setServoMicros(i, pin - ServoSet[i].pin_start, micros);
                    return;
                }

    }

    inline void setServoMicros(int8_t Block, int8_t pinNum, int16_t micros)
    {
        ServoSet[Block].servo_micros[pinNum] = micros;
    }

    int8_t addServoBlock(int8_t startPin, int8_t pinCount, TaskHandle_t UpdateTask)
    {

        if (last_block == MAX_SERVO_SETS)
            return -1;

        int sm = pio_claim_unused_sm(pio, false);

        if (sm < 0)
            return -2;

        if (offset < 0)
        // load PIO code into memory
        {
            if (!pio_can_add_program(pio, &pio_servo_program))
                return -3;

            offset = pio_add_program(pio, &pio_servo_program);
        }

        int dmaCh = dma_claim_unused_channel(false);
        if (dmaCh < 0)
            return -4;

        // we have claimed all needed resources, let's continue
        last_block++;

        ServoSet[last_block].SM = sm;
        ServoSet[last_block].dmaChannel = dmaCh;
        dmaChannelBlock[dmaCh] = last_block;
        ServoSet[last_block].xExternalUpdate = UpdateTask;
        ServoSet[last_block].pin_start = startPin;
        ServoSet[last_block].pin_count = pinCount;

        //Serial.printf(" %d %d %d %d %d \r\n", last_block, sm, dmaCh, startPin, pinCount);

        //for (size_t i = 0; i < MAX_DMA_CHANNELS; i++)
            //Serial.printf("dma settings %d %d \r\n", i, dmaChannelBlock[i]);

        // set up the wave_dma_chan_a DMA channel
        dma_channel_config c = dma_channel_get_default_config(dmaCh);


        channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);

        // Setup the first wave DMA channel for PIO output
        dma_channel_configure(
            dmaCh,
            &c,
            &pio->txf[sm],
            NULL,
            pinCount + 1,
            false);

        if (!interrupt_active)
        {
            irq_set_exclusive_handler(DMA_IRQ_0, PioServo::DMAIrq0Handler);
            irq_set_enabled(DMA_IRQ_0, true);

        }

        return last_block;
    }

    void runServoBlock(int8_t Block)
    {
        uint dma_mask = 0;
        for (size_t i = 0; i <= last_block; i++)
        {
            dma_mask |= 1 << ServoSet[i].dmaChannel;
        }
        //Serial.println(dma_mask, BIN);

        dma_set_irq0_channel_mask_enabled(dma_mask, true);

        pio_servo_program_init(pio, ServoSet[Block].SM, offset, ServoSet[Block].pin_start, ServoSet[Block].pin_count);
        //Serial.println("PIO init done");

        TaskHandle_t Handle;

        switch (Block)
        {
        case 0:
            //Serial.println("Create 0");
            xTaskCreate(DMAUpdateTaskCh0, "Servos1", 1024, this, 3, &Handle);
            break;
        case 1:
            //Serial.println("Create 1");
            xTaskCreate(DMAUpdateTaskCh1, "Servos2", 1024, this, 3, &Handle);
            break;
        case 2:
            //Serial.println("Create 2");
            xTaskCreate(DMAUpdateTaskCh2, "Servos3", 1024, this, 3, &Handle);
            break;
        case 3:
            //Serial.println("Create 3");
            xTaskCreate(DMAUpdateTaskCh3, "Servos4", 1024, this, 3, &Handle);
            break;
        default:
            //Serial.println("default return");
            return;
        }

        //Serial.println("task created");
        //Serial.flush();

        ServoSet[Block].xDMAUpdate = Handle;

        //Serial.println("task done");
        //Serial.flush();
        ServoBufferUpdate(Block);
        //Serial.println("Servo buff done");
        //Serial.flush();
        memcpy(&ServoSet[Block].servo_buffer_b, &ServoSet[Block].servo_buffer_a, sizeof(ServoSet[Block].servo_buffer_a));
        //Serial.println("mem cpy done");
        //Serial.flush();
        DMAUpdate(Block);
        //Serial.println("DMA update done");
        //Serial.flush();

        if (!dma_validation_active)
            xTaskCreate(DMAUpdateTaskCheckChannels, "DMACheckChannels", 1024, this, 1, NULL);

    }

    void CheckDMAChannels()
    {
        while (true)
        {
            for (size_t i = 0; i <= last_block; i++) // 12 is max channel
                if (ServoSet[i].dmaChannel >= 0)
                    if (!dma_channel_is_busy(ServoSet[i].dmaChannel))
                    {
                        //Serial.println("Restart channel");
                        DMAUpdate(i);
                    }
            delay(22);
        }
    }

    void DMAUpdate(uint8_t block)
    {
        if (block > last_block)
            return;

        unsigned long *servo_pointer;

        if (ServoSet[block].servo_buffer)
        {
            servo_pointer = &(ServoSet[block].servo_buffer_a)[0];
        }
        else
            servo_pointer = &(ServoSet[block].servo_buffer_b)[0];
        ServoSet[block].servo_buffer = !ServoSet[block].servo_buffer;

        if (!dma_channel_is_busy(ServoSet[block].dmaChannel))
            dma_channel_transfer_from_buffer_now(ServoSet[block].dmaChannel, servo_pointer, ServoSet[block].pin_count + 1);

        ServoBufferUpdate(block);
    }

    void ServoBufferUpdate(uint8_t block)
    {
        //Serial.printf("DMAUpdateTask block %d ",block);

        unsigned long *servo_pointer;

        if (ServoSet[block].servo_buffer)
            servo_pointer = &ServoSet[block].servo_buffer_a[0];
        else
            servo_pointer = &ServoSet[block].servo_buffer_b[0];

        int frame = 20000;

        for (size_t i = 0; i < ServoSet[block].pin_count; i++)
        {
            //Serial.printf("/ %d %d ",i, ServoSet[block].servo_micros[i]);
            servo_pointer[i] = ServoSet[block].servo_micros[i] << ServoSet[block].pin_count | 1 << i;
            frame = frame - ServoSet[block].servo_micros[i];
        }

        //Serial.printf("DMAUpdateTask servos %d \r\n",frame);

        if (frame < 0)
            frame = 0;

        servo_pointer[ServoSet[block].pin_count] = frame << ServoSet[block].pin_count;

        if (ServoSet[block].xExternalUpdate != NULL)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR(ServoSet[block].xExternalUpdate, 0, eNoAction, &xHigherPriorityTaskWoken);
        }
    }

    long mask_out = 0;
    long ints0_out = 0;
    long stage = 0;

    void DMAHandler()
    {
        // iterate channels
        for (size_t i = 0; i <= last_block; i++) // 12 is max channel
        {
            if (dma_channel_get_irq0_status(ServoSet[i].dmaChannel))
            {
                // chanel called interrupt
                DMAIrq[i]++;
                // wake up task for servo updates in next 20ms
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xTaskNotifyFromISR(ServoSet[i].xDMAUpdate, 0, eNoAction, &xHigherPriorityTaskWoken);

                dma_channel_acknowledge_irq0(ServoSet[i].dmaChannel);
            }
        }
    }

    static void DMAIrq0Handler()
    {
        self->DMAHandler();
    }
};

#endif
