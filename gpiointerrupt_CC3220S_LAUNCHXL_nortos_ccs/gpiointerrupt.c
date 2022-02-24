/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/*UART driver*/
#include <ti/drivers/UART.h>

/* I2C driver */
#include <ti/drivers/I2C.h>

/* Timer Driver */
#include <ti/drivers/Timer.h>

//UART
#define DISPLAY(x) UART_write(uart, &output, x);

//UART Global Variables
char output[64];
int  bytesToSend;

//Driver Handles - Global Variables
UART_Handle uart;

void initUART(void){
    UART_Params uartParams;

    //Init the driver
    UART_init();

    //Configure driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    //Open driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL){
        /* UART_open() failed */
        while (1);
    }
}

//I2C Global Variables
static const struct{
    uint8_t address;
    uint8_t resultReg;
char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

//Driver Handles - Global Variables
I2C_Handle i2c;

//Call initUART() before this function
void initI2C(void){
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    //Init driver
    I2C_init();

    //Configure driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    //Open driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL){
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while(1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    //Boards were shipped with different sensors.
    //Welcome to the world of embedded systems.
    //Try to determine which sensor we have.
    //Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i = 0; i < 3; ++i){
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

    DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));
    if (I2C_transfer(i2c, &i2cTransaction)){
        DISPLAY(snprintf(output, 64, "Found\n\r"));
        found = true;
             break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    }

    if(found){
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress));
    } else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"));
    }
}

int16_t readTemp(void){
    int j;
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)){
        /*
         * Extract degrees C from the received data
         * See TMP sensor data sheet
         */
         temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
         temperature *= 0.0078125;

         /*
          * If the MSB is set '1', then we have a 2's complement
          * negative value which needs to be sign extended
          */
          if (rxBuffer[0] & 0x80){
              temperature |= 0xF000;
          }
    } else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }

    return temperature;
}

// Timer
//Driver Handles - Global Variables
Timer_Handle timer0;

volatile unsigned char TimerFlag = 0;
void timerCallBack(Timer_Handle myHandle, int_fast16_t status){
    TimerFlag = 1;
}

void initTimer(void){
    Timer_Params params;

    //Init driver
    Timer_init();

    //Configure driver
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    //Open driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL){
        /* Failed to initialize timer */
        while(1){}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR){
        /* Failed to start timer */
        while (1) {}
    }
}

//Global flags for buttons
int16_t L_Button_Flag = 0;
int16_t R_Button_Flag = 0;

//Left button
void gpioButtonFxn0(uint_least8_t index){
    L_Button_Flag = 1;
}

//Right button
void gpioButtonFxn1(uint_least8_t index){
    R_Button_Flag = 1;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{

    const unsigned long timerPeriod = 100;
    unsigned long uartElapsedTime = 1000;
    unsigned long buttonElapsedTime = 1500;
    unsigned long tempElapsedTime = 500;

    int16_t temperature;
    int16_t heat = 0;
    int16_t msAfterReset = 0;
    int16_t tempSetPoint = 30;

    /* Call driver init functions */
    GPIO_init();
    initUART();
    initI2C();
    initTimer();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    while(1) {
        //Check button flags every 200ms
        if (buttonElapsedTime >= 200) {
            if (L_Button_Flag == 1){
                heat = 1;
                tempSetPoint++;
                L_Button_Flag = 0;
            }
            if (R_Button_Flag == 1) {
                heat = 0;
                tempSetPoint--;
                R_Button_Flag = 0;
            }
            buttonElapsedTime = 0;
        }

        //Read temp and update LED every 500ms
        if (tempElapsedTime >= 500) {
            temperature = readTemp();
            if (temperature > tempSetPoint) {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            } else {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            }
            tempElapsedTime = 0;
        }

        //Report to server every second, via UART
        if (uartElapsedTime >= 1000) {
            DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, tempSetPoint, heat, msAfterReset / 1000));
            uartElapsedTime = 0;
        }

        while(!TimerFlag) {

        }
        TimerFlag = 0;
        buttonElapsedTime += timerPeriod;
        tempElapsedTime += timerPeriod;
        uartElapsedTime += timerPeriod;
        msAfterReset += timerPeriod;
    }

    return (NULL);
}
