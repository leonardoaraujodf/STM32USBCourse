#include "usbd_framework.h"
#include "usbd_driver.h"

void usbd_initialize(void)
{
	initialize_gpio_pins();
	initialize_core();
	connect();
}
