/************************************************************************************************************************** 
 * Author:  Mike Collins
 * Purpose: This is working example code which shows how to use the MC3672 3-Axis Acceleorometer from mCube Inc
 *  The project has been configured to work with the nRF52840-DK.
 *    
 * History:
 *      - 2019/12/12 : Initial code extracted and merged from NextFlexad7746 and mCube demo
 *
 * References:
 *    https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v14.0.0%2Fgroup__nrf__drv__twi.html
 **************************************************************************************************************************/
#define DISABLE_UART_DRV

#include <stdio.h>

#include "boards.h"
#include "app_uart.h"  // Need for uart coms
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "MC3672.h"


#define LED0                   BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define LED1                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LED2                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */

// Needed for uart coms
#include "bsp.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define DONGLE_RX_PIN_NUMBER             NRF_GPIO_PIN_MAP(0,3)
#define DONGLE_TX_PIN_NUMBER             NRF_GPIO_PIN_MAP(0,4)

// Device 1
#define PWR_PIN_1            NRF_GPIO_PIN_MAP(0,31)
#define SCL_PIN_1            NRF_GPIO_PIN_MAP(0,2)
#define SDA_PIN_1            NRF_GPIO_PIN_MAP(0,29)

// Device 2
#define PWR_PIN_2            NRF_GPIO_PIN_MAP(1,10)
#define SCL_PIN_2            NRF_GPIO_PIN_MAP(1,13)
#define SDA_PIN_2            NRF_GPIO_PIN_MAP(1,15)



/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED


/**
 * @brief Function for main application entry.
 */
int main(void)
{  
    leds_init();  // Used to initialize bsp_board_init, which sets GPIO to 3.0V.
    // Used to initalize and setup RTT Debug Logging
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("MC3672 Demo Starting!.");
    NRF_LOG_FLUSH();


   uint32_t err_code;  

    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////


    int ledToggle = 0;

    nrf_gpio_pin_set(PWR_PIN_2);
    nrf_gpio_cfg(PWR_PIN_2, NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_H0S1,
        NRF_GPIO_PIN_NOSENSE);
 
    // time for power on 
    nrf_delay_ms(50);


    // initialize I2C
    MC36XX_start(SCL_PIN_2, SDA_PIN_2);

    while(true)
    {
        int8_t accOut[80];

        MC36XX_acc_t acc = {0};
        MC36XX_readRawAccel(&acc);

        if (ledToggle ^= 1)
            bsp_board_led_off(LED0);
        else
            bsp_board_led_off(LED1);

        // display results
        sprintf(accOut, "X:%g, Y:%g, Z:%g\r\n", acc.XAxis_g, acc.YAxis_g, acc.ZAxis_g);
        printf(accOut);

        nrf_delay_ms(50);
    }
}


