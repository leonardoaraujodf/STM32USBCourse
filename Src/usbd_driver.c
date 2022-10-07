#include "usbd_driver.h"
#include "usb_standards.h"
#include "Helpers/logger.h"

void initialize_gpio_pins(void)
{
	// The usb micro connector from the STM32F407VG-DISC1 board is connected directly to the OTG FS IP
	// of the board, not to the HS IP as in the STM32F429ZI board. So  we need to adapt the code from
	// the course to use USB FS IP instead of the HS one.
#if defined(STM32F407xx)
	// Enables the clock for GPIOA
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);

	// Sets alternate function 10 for PA11 (D-), and PA12 (D+).
	MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL11 | GPIO_AFRH_AFSEL12,
			   _VAL2FLD(GPIO_AFRH_AFSEL11, 0xA) | _VAL2FLD(GPIO_AFRH_AFSEL12, 0xA));

	// Configures USB pins (in GPIOA) to work in alternate function mode
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER11 | GPIO_MODER_MODER12,
			   _VAL2FLD(GPIO_MODER_MODER11, 2) | _VAL2FLD(GPIO_MODER_MODER12, 2));
#elif defined (STM32F429xx)
	// Enables the clock for GPIOB
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);

	// Sets alternate function 12 for: PB14 (D-), and PB15 (D+).
	MODIFY_REG(GPIOB->AFR[1], GPIO_AFRH_AFSEL14 | GPIO_AFRH_AFSEL15,
			   _VAL2FLD(GPIO_AFRH_AFSEL14, 0xC) | _VAL2FLD(GPIO_AFRH_AFSEL15, 0xC));

	// Configures USB pins (in GPIOB) to work in alternate function mode.
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER14 | GPIO_MODER_MODER15,
			   _VAL2FLD(GPIO_MODER_MODER14, 2) | _VAL2FLD(GPIO_MODER_MODER15, 2));
#else
	#error "Please select first the target STM32F4xx device used in your application (in stm32f4xx.h file)"
#endif
}

void initialize_core()
{
#if defined(STM32F407xx)
	// Enables the clock for USB core.
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_OTGFSEN);

	// Configures the USB core to run in device mode, and to use the embedded full-speed PHY.
	MODIFY_REG(USB_OTG_FS->GUSBCFG,
               USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_TRDT,
			   USB_OTG_GUSBCFG_FDMOD | _VAL2FLD(USB_OTG_GUSBCFG_TRDT, 0x09));

	// Configures the device to run in full speed mode.
	MODIFY_REG(USB_OTG_FS_DEVICE->DCFG,
		       USB_OTG_DCFG_DSPD,
		       _VAL2FLD(USB_OTG_DCFG_DSPD, 0x03));

	// Enables VBUS sensing device.
	SET_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_VBUSBSEN);

	// Unmasks the main USB core interrupts.
	SET_BIT(USB_OTG_FS->GINTMSK,
		    USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_SOFM |
		    USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_WUIM | USB_OTG_GINTMSK_IEPINT |
		    USB_OTG_GINTSTS_OEPINT | USB_OTG_GINTMSK_RXFLVLM);

	// Clears all pending core interrupts.
	WRITE_REG(USB_OTG_FS->GINTSTS, 0xFFFFFFFF);

	// Unmasks USB global interrupt.
	SET_BIT(USB_OTG_FS->GAHBCFG, USB_OTG_GAHBCFG_GINT);

	// Unmasks transfer completed interrupts for all endpoints.
	SET_BIT(USB_OTG_FS_DEVICE->DOEPMSK, USB_OTG_DOEPMSK_XFRCM);
	SET_BIT(USB_OTG_FS_DEVICE->DIEPMSK, USB_OTG_DIEPMSK_XFRCM);


#elif defined (STM32F429xx)
	// Enables the clock for USB core.
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_OTGHSEN);

	// Configures the USB core to run in device mode, and to use the embedded full-speed PHY.
	MODIFY_REG(USB_OTG_HS->GUSBCFG,
		USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL | USB_OTG_GUSBCFG_TRDT,
		USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL | _VAL2FLD(USB_OTG_GUSBCFG_TRDT, 0x09)
	);

	// Configures the device to run in full speed mode.
	MODIFY_REG(USB_OTG_HS_DEVICE->DCFG,
		USB_OTG_DCFG_DSPD,
		_VAL2FLD(USB_OTG_DCFG_DSPD, 0x03)
	);

	// Enables VBUS sensing device.
	SET_BIT(USB_OTG_HS->GCCFG, USB_OTG_GCCFG_VBUSBSEN);

	// Unmasks the main USB core interrupts.
	SET_BIT(USB_OTG_HS->GINTMSK,
		USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_SOFM |
		USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_WUIM | USB_OTG_GINTMSK_IEPINT |
		USB_OTG_GINTSTS_OEPINT | USB_OTG_GINTMSK_RXFLVLM
	);

	// Clears all pending core interrupts.
	WRITE_REG(USB_OTG_HS->GINTSTS, 0xFFFFFFFF);

	// Unmasks USB global interrupt.
	SET_BIT(USB_OTG_HS->GAHBCFG, USB_OTG_GAHBCFG_GINT);

	// Unmasks transfer completed interrupts for all endpoints.
	SET_BIT(USB_OTG_HS_DEVICE->DOEPMSK, USB_OTG_DOEPMSK_XFRCM);
	SET_BIT(USB_OTG_HS_DEVICE->DIEPMSK, USB_OTG_DIEPMSK_XFRCM);

#endif
}

/** \brief Connects the USB device to the bus.
 */
void connect()
{
#if defined(STM32F407xx)
	// Powers the transceivers on.
    SET_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_PWRDWN);

	// Connects the device to the bus.
    CLEAR_BIT(USB_OTG_FS_DEVICE->DCTL, USB_OTG_DCTL_SDIS);
#elif defined (STM32F429xx)
	// Powers the transceivers on.
    SET_BIT(USB_OTG_HS->GCCFG, USB_OTG_GCCFG_PWRDWN);

	// Connects the device to the bus.
    CLEAR_BIT(USB_OTG_HS_DEVICE->DCTL, USB_OTG_DCTL_SDIS);
#endif
}

/** \brief Disconnects the USB device from the bus.
 */
void disconnect()
{
#if defined(STM32F407xx)
	// Disconnects the device from the bus.
	SET_BIT(USB_OTG_FS_DEVICE->DCTL, USB_OTG_DCTL_SDIS);

	// Powers the transceivers off.
	CLEAR_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_PWRDWN);
#elif defined (STM32F429xx)
	// Disconnects the device from the bus.
	SET_BIT(USB_OTG_HS_DEVICE->DCTL, USB_OTG_DCTL_SDIS);

	// Powers the transceivers off.
	CLEAR_BIT(USB_OTG_HS->GCCFG, USB_OTG_GCCFG_PWRDWN);
#endif
}

static void configure_endpoint0(uint8_t endpoint_size)
{
	// Unmasks all interrupts of IN and OUT endpoint0.
	SET_BIT(USB_OTG_FS_DEVICE->DAINTMSK, 1 << 0 | 1 << 16);

	// Configures the maximum packet size, activates the endpoint, and NAK the endpoint (cannot send data yet).
	MODIFY_REG(IN_ENDPOINT(0)->DIEPCTL,
			   USB_OTG_DIEPCTL_MPSIZ,
			   USB_OTG_DIEPCTL_USBAEP |
			   _VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, endpoint_size) |
			   USB_OTG_DIEPCTL_SNAK);

	// Clears NAK, and enables endpoint data transmission.
	SET_BIT(OUT_ENDPOINT(0)->DOEPCTL, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK);
}

static void configure_in_endpoint(uint8_t endpoint_number, UsbEndpointType endpoint_type, uint16_t endpoint_size)
{
	// Unmasks all interrupt of the targeted IN endpoint.
	SET_BIT(USB_OTG_FS_DEVICE->DAINTMSK, 1 << endpoint_number);

	// Activates the endpoint, sets endpoint handshake to NAK (not ready to send data),
	// sets DATA0 packet identifier, configures its type and its maximum packet size
	MODIFY_REG(IN_ENDPOINT(endpoint_number)->DIEPCTL,
               USB_OTG_DIEPCTL_MPSIZ |
               USB_OTG_DIEPCTL_EPTYP |
               USB_OTG_DIEPCTL_TXFNUM,
               USB_OTG_DIEPCTL_USBAEP |
               _VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, endpoint_size) |
               USB_OTG_DIEPCTL_SNAK |
               _VAL2FLD(USB_OTG_DIEPCTL_EPTYP, endpoint_type) |
               _VAL2FLD(USB_OTG_DIEPCTL_TXFNUM, endpoint_number) |
               USB_OTG_DIEPCTL_SD0PID_SEVNFRM);
}

/**
 * \brief Deconfigures IN and OUT endpoints of a specific endpoint number.
 * \param endpoint_number The number of IN and OUT endpoints to deconfigure.
 */
static void deconfigure_endpoint(uint8_t endpoint_number)
{
	USB_OTG_INEndpointTypeDef * in_endpoint = IN_ENDPOINT(endpoint_number);
	USB_OTG_OUTEndpointTypeDef * out_endpoint = OUT_ENDPOINT(endpoint_number);

	// Masks all interrupts of the targeted IN and OUT endpoints.
	CLEAR_BIT(USB_OTG_FS_DEVICE->DAINTMSK, (1 << endpoint_number) | (1 << 16 << endpoint_number));

	// Clear all interrupts of the endpoint.
	SET_BIT(in_endpoint->DIEPINT, 0x28FB);
	SET_BIT(out_endpoint->DOEPINT, 0x31BB);

	// Disables the endpoints if possible.
	if (in_endpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
	{
		// Disables endpoint transmission
		SET_BIT(in_endpoint->DIEPCTL, USB_OTG_DIEPCTL_EPDIS);
	}

	// Deactivates the endpoint
	CLEAR_BIT(in_endpoint->DIEPCTL, USB_OTG_DIEPCTL_USBAEP);

	if (endpoint_number != 0)
	{
		if (out_endpoint->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
		{
			// Disables endpoint transmission
			SET_BIT(out_endpoint->DOEPCTL, USB_OTG_DOEPCTL_EPDIS);
		}

		// Deactivates the endpoint.
		CLEAR_BIT(out_endpoint->DOEPCTL, USB_OTG_DOEPCTL_USBAEP);
	}
}

/**
 * \brief Updates the start addresses of all FIFOs according to the size of each FIFO.
 */
static void refresh_fifo_start_address()
{
	// The first changeable start address begins after the region of RxFIFO.
	uint16_t start_address = _FLD2VAL(USB_OTG_GRXFSIZ_RXFD, USB_OTG_FS->GRXFSIZ) * 4;

	// Updates the start address of the TxFIFO0.
	MODIFY_REG(USB_OTG_FS->DIEPTXF0_HNPTXFSIZ,
			   USB_OTG_TX0FSA,
			   _VAL2FLD(USB_OTG_TX0FSA, start_address));

	// The next start address is after where the last TxFIFO ends.
	start_address += _FLD2VAL(USB_OTG_TX0FD, USB_OTG_FS->DIEPTXF0_HNPTXFSIZ) * 4;

	for (uint8_t txfifo_number = 0; txfifo_number < ENDPOINT_COUNT - 1; txfifo_number++)
	{
		MODIFY_REG(USB_OTG_FS->DIEPTXF[txfifo_number],
				   USB_OTG_NPTXFSA,
				   _VAL2FLD(USB_OTG_NPTXFSA, start_address));
		start_address += _FLD2VAL(USB_OTG_NPTXFD, USB_OTG_FS->DIEPTXF[txfifo_number]) * 4;
	}
}


/**
 * \brief Configures the RxFIFO of all OUT endpoints.
 * \param size The size of the largest OUT endpoint in bytes.
 * \note The RxFIFO is shared between all OUT endpoints.
 */
static void configure_rxfifo_size(uint16_t size)
{
	// Consider the space required to save status packets in RxFIFO and gets the size in term of
	// 32-bit words.
	size = 10 + (2 * ((size / 4) + 1));

	// Configures the depth of the FIFO.
	MODIFY_REG(USB_OTG_FS->GRXFSIZ,
			   USB_OTG_GRXFSIZ_RXFD,
			   _VAL2FLD(USB_OTG_GRXFSIZ_RXFD, size));

	refresh_fifo_start_addresses();
}

/**
 * \brief Configures the TxFIFO of an IN endpoint.
 * \param endpoint_number The number of the IN endpoint we want to configure its TxFIFO
 * \param size The size of the IN endpoint in bytes.
 * \note Any change on any FIFO will update the registers of all TxFIFOs to adapt the start offsets in the FIFO
 * dedicated memory.
 */
static void configure_txfifo_size(uint8_t endpoint_number, uint16_t size)
{
	// Gets the FIFO size in term of 32-bit words.
	size = (size + 3) / 4;

	// Configures the depth of the TxFIFO.
	if (endpoint_number == 0)
	{
		MODIFY_REG(USB_OTG_FS->DIEPTXF0_HNPTXFSIZ,
				   USB_OTG_TX0FD,
				   _VAL2FLD(USB_OTG_TX0FG, size));
	}
	else
	{
		MODIFY_REG(USB_OTG_FS->DIEPTXF[endpoint_number - 1],
				   USB_OTG_NPTXFD,
				   _VAL2FLD(USB_OTG_NPTXFD, size));
	}

	refresh_fifo_start_addresses();
}

static void usbrst_handler()
{
	log_info("USB reset signal was detected.");
	for (uint8_t i = 0; i <= ENDPOINT_COUNT; i++)
	{
		deconfigure_endpoint(i);
	}
}

/**
 * \brief Handles the USB core interrupts.
 */
static void gintsts_handler()
{
	volatile uint32_t gintsts = USB_OTG_FS_GLOBAL->GINTSTS;

	if (gintsts & USB_OTG_GINTSTS_USBRST)
	{
		usbrst_handler();
		// Clears the interrupt.
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_USBRST);
	}
	else if (gintsts & USB_OTG_GINTSTS_ENUMDNE)
	{
		// enumdne_handler();
		// Clears the interrupt.
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_ENUMDNE);
	}
	else if (gintsts & USB_OTG_GINTSTS_RXFLVL)
	{
		// rxflvl_handler();
		// Clears the interrupt.
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_RXFLVL);
	}
	else if (gintsts & USB_OTG_GINTSTS_IEPINT)
	{
		// iepint_handler();
		// Clears the interrupt.
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_IEPINT);
	}
	else if (gintsts & USB_OTG_GINTSTS_OEPINT)
	{
		// oepint_handler();
		// Clears the interrupt.
		SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_OEPINT);
	}
}
