// When the button on the base board is pressed, send a character over the I2C bus to
// the shield board. This is used by the processor on the shield to cause the red/blue
// LEDs to alternate on/off states.
#include "wiced.h"

#define I2C_ADDRESS  (0x42)
#define RETRIES      (1)
#define DISABLE_DMA  (WICED_TRUE)
#define NUM_MESSAGES (1)

/* I2C register locations */
#define CONTROL_REG (0x05)
#define LED_REG     (0x04)

volatile wiced_bool_t buttonPress = WICED_FALSE;

/* Interrupt service routine for the button */
void button_isr(void* arg)
{
	buttonPress = WICED_TRUE;
}

/* Main application */
void application_start( )
{
	wiced_init();	/* Initialize the WICED device */

    wiced_gpio_input_irq_enable(WICED_SH_MB1, IRQ_TRIGGER_FALLING_EDGE, button_isr, NULL); /* Setup interrupt */

    /* Setup I2C master */
    const wiced_i2c_device_t i2cDevice = {
    	.port = WICED_I2C_1,
		.address = I2C_ADDRESS,
		.address_width = I2C_ADDRESS_WIDTH_7BIT,
		.speed_mode = I2C_STANDARD_SPEED_MODE
    };

    wiced_i2c_init(&i2cDevice);

    /* Setup transmit buffer and message */
    /* We will always write an offset and then a single value, so we need 2 bytes in the buffer */
    uint8_t tx_buffer[] = {0, 0};
    wiced_i2c_message_t msg;
    wiced_i2c_init_tx_message(&msg, tx_buffer, sizeof(tx_buffer), RETRIES, DISABLE_DMA);

    /* Write a value of 0x01 to the control register to enable control of the CapSense LEDs over I2C */
    tx_buffer[0] = CONTROL_REG;
    tx_buffer[1] = 0x01;
	wiced_i2c_transfer(&i2cDevice, &msg, NUM_MESSAGES);

	tx_buffer[0] = LED_REG; /* Set offset for the LED register */

    while ( 1 )
    {
    		wiced_i2c_transfer(&i2cDevice, &msg, NUM_MESSAGES);
    		tx_buffer[1] = tx_buffer[1] << 1; /* Shift to the next LED */
    		if (tx_buffer[1] > 0x08) /* Reset after turning on LED3 */
    		{
    			tx_buffer[1] = 0x01;
    		}
    		wiced_rtos_delay_milliseconds(1000);
    }

}
