 /*
 * main.c
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/ssi.h"
#include "utils/uartstdio.h"



void ConfigureUART(void);

uint32_t ADC_data;
uint8_t PumpON_Count;
uint8_t PumpOFF_Count;


char strOut[80];
uint8_t strIndex;
uint32_t  colck_freq;

//for firmware Version Number
char FverData[];

uint32_t g_ui32Flags;

//for adc
//
// This array is used for storing the data read from the ADC FIFO. It
// must be as large as the FIFO for the sequencer in use.  This example
// uses sequence 3 which has a FIFO depth of 1.  If another sequence
// was used with a deeper FIFO, then the array size must be changed.
//
volatile uint32_t pui32ADC0Value[513];
uint32_t ADC_count;
bool ADC_data_ready;
void delayMs(uint32_t ui32Ms);
void
GetVersion()
{
	FverData[0]='1';
	FverData[1]='0';
	FverData[2]='0';
	FverData[3]='0';
	//history
	//1.0.0.0 initial

}

uint32_t ADC0task(uint8_t adc_CHx, uint8_t adcEN_DIS)
{
    uint32_t adcResult;
    // Enable or Disable the ADC0 module.
    //
    if (adcEN_DIS == 1)
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
        //
        // Wait for the ADC0 module to be ready.
        //
        while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
            ;
        //
        // Enable the first sample sequencer to capture the value of channel 0 when
        // the processor trigger occurs.
        //
        ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

        ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | adc_CHx);

        ADCSequenceEnable(ADC0_BASE, 0);
        //
        // Trigger the sample sequence.
        //
        ADCProcessorTrigger(ADC0_BASE, 0);
        //
        // Wait until the sample sequence has completed.
        //
        while (!ADCIntStatus(ADC0_BASE, 0, false))
            ;

        //
        // Read the value from the ADC.
        //
        //adcCount=ADCSequenceDataGet(ADC0_BASE, 0, &uiADC32Value);

        // uiADC32Value=*((uint32_t*)(ADC0_BASE + ADC_O_SSFIFO0));

        adcResult = HWREG(ADC0_BASE + ADC_O_SSFIFO0); //read register FIFO0
    }
    else
    {
        SysCtlPeripheralDisable(SYSCTL_PERIPH_ADC0);
        adcResult = 0;
    }

//		    UARTprintf("\rADC: %d\n ", adcResult);

    return adcResult;

}



int main(void) {

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    FPULazyStackingEnable();

//	400MHZ(PLL) /2 / 4 = 50 Mhz
//
//	The 4 is SYSCTL_SYSDIV_4

    SysCtlClockSet(
    SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //PB6 is for pump on/off, pb7 for led indicator
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    ConfigureUART();

    UARTprintf("starting\n");

	while (1) {
	    //when the capacitive soil sensor into the water the value changes from 3539(dry) to 1921(wet) base on the depth
	    //turn on the pump when reading below 1950 three times
	    //turn off the pump when reading above 3300 three times

	    ADC_data =  ADC0task(0, 1);//
	    UARTprintf("\rADC Count: %d\n ",  ADC_data);
	    if(ADC_data< 2900)//1950 is for tap water when almost full, but for the condensate water is 2400 when almost full so pick 2900
	    {
	        PumpON_Count++;
	    }
	    if(PumpON_Count==3)
	    {
	        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //pump on
	        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7); //pump on LED indicator ON
	     //   delayMs(1000);
	        while(ADC_data < 3300) //water not empty yet
	        {
	            ADC_data =  ADC0task(0, 1);//
	            UARTprintf("\rADC Count: %d\n ",  ADC_data);
	            UARTprintf("Pump is working hard...\n");
	            delayMs(1000);
	        }
	        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0); //pump off
	        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0); //pump on LED indicator Off
	        PumpON_Count=0;
	    }

        delayMs(500);

	}	//while
	//return 0;
}//main

////for delay
void delayMs(uint32_t ui32Ms) {

  // 1 clock cycle = 1 / SysCtlClockGet() second
  // 1 SysCtlDelay = 3 clock cycle = 3 / SysCtlClockGet() second
  // 1 second = SysCtlClockGet() / 3
  // 0.001 second = 1 ms = SysCtlClockGet() / 3 / 1000
 // SysCtlDelay(ui32Ms * (120000000 / 3 / 1000));//for TM4C1294@120Mhz
  SysCtlDelay(ui32Ms * (SysCtlClockGet() / 3 / 1000)); //for TM4C123 only
}

void delayUs(uint32_t ui32Us) {
  SysCtlDelay(ui32Us * (120000000 / 3 / 1000000));
  //SysCtlDelay(ui32Us * (SysCtlClockGet() / 3 / 1000000));
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);

    //second uart UART2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE+GPIO_O_CR) |= GPIO_PIN_7; //unlock PD7


    GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinConfigure(GPIO_PD7_U2TX);

    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}
