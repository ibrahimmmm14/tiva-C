#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "driverlib/uart.h"
#include "inc/hw_uart.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h" 
#include "driverlib/i2c.h"    
#include "driverlib/pin_map.h"  

#define LED_RED GPIO_PIN_3   
#define LED_RED_PORT GPIO_PORTF_BASE
#define LED_BLUE GPIO_PIN_4     
#define LED_BLUE_PORT GPIO_PORTC_BASE 
#define LED_GREEN GPIO_PIN_3    
#define LED_GREEN_PORT GPIO_PORTB_BASE 
#define BUT1 GPIO_PIN_6
#define BUT2 GPIO_PIN_7
#define BUT_PORT GPIO_PORTD_BASE
#define INT_TIMER0A 19 // Timer0A interrupt number
#define TEMP_SENSOR_ADDR 0x48 // Example address for a temperature sensor

volatile bool flash_flag = false;
volatile bool stop_flag = false;
volatile uint8_t led_state = 0;

void rgpledset();
void rgpflash();
void rgpstop();
void butset();
void UARTPutString();
uint16_t readTemperatureRaw();
float convertTemp(uint16_t raw);
void UARTPrintTemperature(float tempC);


void GPIOPortDIntHandler(void) {
    uint32_t status = GPIOIntStatus(BUT_PORT, true);
    GPIOIntClear(BUT_PORT, status);
 // طباعة قيمة status على UART
   // char buffer[32];
   // snprintf(buffer, sizeof(buffer), "INT STATUS: 0x%02X\n", (unsigned int)status);
   // UARTPutString(UART0_BASE, buffer);

    if (status & BUT1) {
        flash_flag = true;
        stop_flag = false;
    }
    if (status & BUT2) {
        stop_flag = true;
        flash_flag = false;
    }
}

void uart_init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Configure the GPIO pins for UART0
    // PA0 is UART0 RX, PA1 is UART0 TX
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                       (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));
}
void i2c_init(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);       // I2C1 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);      // Port A for PA6, PA7
    SysCtlDelay(3); // Wait for the peripheral to be ready
    // Configure GPIO pins for I2C1
    // PA6 is SCL, PA7 is SDA
   
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true); // Initialize I2C1 
}

void setup() {
    SysCtlClockSet(
        SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);


    rgpledset();
    butset();
    uart_init();
    i2c_init();
}

void loop() {
    static bool flashing = false;

    
    if (flash_flag && !flashing) {
        rgpflash();      // Start timer only once
        flashing = true;
    }
    if (stop_flag && flashing) {
        rgpstop();       // Stop timer only once
        flashing = false;
        flash_flag = false;
        stop_flag = false;
    }

}


void butset() {
    // Enable the GPIO peripheral for Port D
    GPIOPinTypeGPIOInput(BUT_PORT, BUT1 | BUT2);
    GPIOPadConfigSet(BUT_PORT, BUT1 | BUT2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    // Configure interrupts for Port D
    GPIOIntDisable(BUT_PORT, BUT1 | BUT2);
    GPIOIntClear(BUT_PORT, BUT1 | BUT2);
    GPIOIntRegister(BUT_PORT, GPIOPortDIntHandler);
    GPIOIntTypeSet(BUT_PORT, BUT1 | BUT2, GPIO_FALLING_EDGE);
    GPIOIntEnable(BUT_PORT, BUT1 | BUT2);
    IntMasterEnable();
}

void rgpledset() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF|
                           SYSCTL_PERIPH_GPIOC|
                           SYSCTL_PERIPH_GPIOB|
                           SYSCTL_PERIPH_GPIOD);

    GPIOPinTypeGPIOOutput(LED_RED_PORT, LED_RED);
    GPIOPinTypeGPIOOutput(LED_BLUE_PORT, LED_BLUE);
    GPIOPinTypeGPIOOutput(LED_GREEN_PORT, LED_GREEN);

 
}

void UARTPutString(uint32_t uart_base, const char *str) {
    while (*str) {
        UARTCharPut(uart_base, *str++);
    }
}

void Timer0AIntHandler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //UARTPutString(UART0_BASE, "FLAESH TIMER handler\n");

    // Read temperature and toggle LEDs
    // Assuming a temperature sensor is connected and configured
    uint16_t temp_raw = readTemperatureRaw(); // Read from sensor
    float temp_c = convertTemp(temp_raw);     // Convert to Celsius
    UARTPrintTemperature(temp_c);       

    switch (led_state) {
        case 0:
            GPIOPinWrite(LED_RED_PORT, LED_RED, LED_RED);
            GPIOPinWrite(LED_BLUE_PORT, LED_BLUE, 0);
            GPIOPinWrite(LED_GREEN_PORT, LED_GREEN, 0);
            //UARTPutString(UART0_BASE, "LED: RED ON\n");
            led_state = 1;
            break;
        case 1:
            GPIOPinWrite(LED_RED_PORT, LED_RED, 0);
            GPIOPinWrite(LED_BLUE_PORT, LED_BLUE, LED_BLUE);
            GPIOPinWrite(LED_GREEN_PORT, LED_GREEN, 0);
            //UARTPutString(UART0_BASE, "LED: BLUE ON\n");
            led_state = 2;
            break;
        case 2:
            GPIOPinWrite(LED_RED_PORT, LED_RED, 0);
            GPIOPinWrite(LED_BLUE_PORT, LED_BLUE, 0);
            GPIOPinWrite(LED_GREEN_PORT, LED_GREEN, LED_GREEN);
            //UARTPutString(UART0_BASE, "LED: GREEN ON\n");
            led_state = 0;
            break;
    }
}

void timer_init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    uint32_t period = (SysCtlClockGet() / 2); // 0.5s interval (adjust as needed)
    TimerLoadSet(TIMER0_BASE, TIMER_A, period - 1);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler);
    IntEnable(INT_TIMER0A); // Enable Timer0A interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerControlStall(TIMER0_BASE, TIMER_A, false); // Allow timer to run in deep sleep mode
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true); // Enable trigger for the timer
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE); // Set event to positive edge
    TimerControlLevel(TIMER0_BASE, TIMER_A, false); // Set level to low 
    TimerEnable(TIMER0_BASE, TIMER_A);
    //UARTPutString(UART0_BASE, "FLAESH TIMER init\n");
}

void rgpflash() {
    timer_init(); // Start timer-based flashing
    
}

void rgpstop() {
    TimerDisable(TIMER0_BASE, TIMER_A);
    GPIOPinWrite(LED_RED_PORT, LED_RED, 0);
    GPIOPinWrite(LED_BLUE_PORT, LED_BLUE, 0);
    GPIOPinWrite(LED_GREEN_PORT, LED_GREEN, 0);
    //UARTPutString(UART0_BASE, "LED OFF\n");
}

uint16_t readTemperatureRaw() {
    I2CMasterSlaveAddrSet(I2C1_BASE, TEMP_SENSOR_ADDR, false); // Write
    I2CMasterDataPut(I2C1_BASE, 0x00); // Temperature register
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C1_BASE));

    I2CMasterSlaveAddrSet(I2C1_BASE, TEMP_SENSOR_ADDR, true); // Read
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C1_BASE));
    uint8_t msb = I2CMasterDataGet(I2C1_BASE);

    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C1_BASE));
    uint8_t lsb = I2CMasterDataGet(I2C1_BASE);

    return ((msb << 8) | lsb);
}

float convertTemp(uint16_t raw) {
    raw >>= 4; // TMP102 sends left-justified 12-bit data
    return raw * 0.0625;
}
void UARTPrintTemperature(float tempC) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Temperature: %.2f°C\n", tempC);
    UARTPutString(UART0_BASE, buffer);
}