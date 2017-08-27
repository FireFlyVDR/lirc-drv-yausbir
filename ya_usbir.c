/*****************************************************************************
 ** ya_usbir.c ***************************************************************
 *****************************************************************************
 *
 * Mode2 receiver + transmitter using the yaUsbIr with power switch
 * Copyright (C) 2012 Uwe Guenther
 * Copyright (C) 2017 Christoph Haubrich
 *
 * change: 15.04.2012 speed up send_func 
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published
 *  by the Free Software Foundation; either version 2 of the License,
 *  or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details. You should
 *  have received a copy of the GNU General Public License along with
 *  this program; if not, write to:
 *		Free Software Foundation, Inc.,
 *		59 Temple Place - Suite 330,
 *		Boston, MA 02111-1307, USA
 *
 * driver code version inspired by the hw_ftdi driver.
 *
 ****************************************************************************/

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <usb.h>
#include <stdint.h>
#include <linux/hiddev.h>
#include <linux/input.h>
#include <sys/wait.h>
#include <signal.h>

#include "lirc_driver.h"

#define CMD_NONE	       0x00
#define CMD_IRDATA	       0x01
#define CMD_COMDATA	       0x02
#define CMD_SETCOMBAUD	       0x03
#define CMD_GETCOMBAUD	       0x04
#define CMD_GETIOS	       0x05
#define CMD_GETIO	       0x06
#define CMD_SETIOS	       0x07
#define CMD_SETIO	       0x08
#define IRRX_NODATA	       0x0000
#define IRRX_F_POLL	       6000000 // 6MHz
#define IRRX_CMD	       0x7500

//*** raw hid interface ******************************************************

typedef struct {
	usb_dev_handle *usb;
	int iface;
	int ep_in;
	int ep_out;
} raw_hid;

//****************************************************************************

static raw_hid *ya_usbir_dev = NULL;
static int usb_vendor = 0x10c4;
static int usb_product = 0x876c;
static int pipe_main2tx[2] = { -1, -1 };
static int pipe_tx2main[2] = { -1, -1 };
static pid_t child_pid = -1;// PID of the child process
static int first = 1;

/*****************************************************************************
 *  rawhidrecv - receive a packet
 *    Inputs:
 *  hid = device to receive from
 *  buf = buffer to receive packet
 *  len = buffer's size
 *  timeout = time to wait, in milliseconds
 *    Output:
 *  number of bytes received, or -1 on error
 *
 ****************************************************************************/
int rawhidrecv(raw_hid *hid, void *buf, int len, int timeout)
{
	int r;
	if (hid==NULL)
		return -1;
	r = usb_interrupt_read(hid->usb, hid->ep_in, (char*)buf, len, timeout);
	if (r >= 0)
		return r;
	if ((errno == EAGAIN) || (errno == ETIMEDOUT)) // timeout
		 return 0;
	logprintf(LOG_NOTICE,"yaUsbIr: Interrupt read (%d: %m) (%s).\n", r, usb_strerror());
	return -1;
}

/*****************************************************************************
 *  rawhidsend - send a packet
 *    Inputs:
 *  hid = device to transmit to
 *  buf = buffer containing packet to send
 *  len = number of bytes to transmit
 *  timeout = time to wait, in milliseconds
 *    Output:
 *  number of bytes sent, or -1 on error
 *
 ****************************************************************************/
int rawhidsend(raw_hid *hid, void *buf, int len, int timeout)
{
	if (hid==NULL) return -1;
	if (hid->ep_out) {
		return usb_interrupt_write(hid->usb, hid->ep_out, (char*)buf, len, timeout);
	} else {
		return usb_control_msg(hid->usb, 0x21, 9, 0, hid->iface, (char*)buf, len, timeout);
	}
}

/*****************************************************************************
 *  rawhidOpen - open first device
 *
 *    Inputs:
 *  vid = Vendor ID
 *  pid = Product ID
 *    Output:
 *  first openend device
 *
 *****************************************************************************/
raw_hid *rawhidopen(int vid, int pid, int info)
{
	struct usb_bus *bus;
	struct usb_device *dev;
	struct usb_interface *iface;
	struct usb_interface_descriptor *desc;
	struct usb_endpoint_descriptor *ep;
	usb_dev_handle *usb;
	uint8_t buf[1024];
	int ifacenum, n, ep_in, ep_out;
//	int len;
	raw_hid *hid;
	char text[512];
	//usb_set_debug(3);
	
	usb_init();
	usb_find_busses();
	usb_find_devices();	
	for (bus = usb_get_busses(); bus; bus = bus->next) {
		for (dev = bus->devices; dev; dev = dev->next) {
			if (dev->descriptor.idVendor != vid) continue;
			if (dev->descriptor.idProduct != pid) continue;
			if (!dev->config) continue;
			if (dev->config->bNumInterfaces < 1) continue;
			if (info)
				logprintf(LOG_NOTICE,"yaUsbIr: device: vid=%04X, pic=%04X, with %d interface",
					dev->descriptor.idVendor,dev->descriptor.idProduct,dev->config->bNumInterfaces);

			iface = dev->config->interface;
			usb = NULL;
			for (ifacenum=0; ifacenum<dev->config->bNumInterfaces && iface; ifacenum++, iface++) {
				desc = iface->altsetting;
				if (!desc) continue;
				//logprintf(LOG_NOTICE,"yaUsbIr:   type %d, %d, %d\n", desc->bInterfaceClass, desc->bInterfaceSubClass, desc->bInterfaceProtocol);
				if (desc->bInterfaceClass != 3) continue;
				if (desc->bInterfaceSubClass != 0) continue;
				if (desc->bInterfaceProtocol != 0) continue;
				ep = desc->endpoint;
				ep_in = ep_out = 0;
				for (n = 0; n < desc->bNumEndpoints; n++, ep++) {
					if (ep->bEndpointAddress & 0x80) {
						if (!ep_in) ep_in = ep->bEndpointAddress & 0x7F;
						ep_in |= USB_ENDPOINT_IN;
						//logprintf(LOG_NOTICE,"yaUsbIr:     IN endpoint 0x%02x\n", ep_in);
					} else {
						if (!ep_out) ep_out = ep->bEndpointAddress;
						//logprintf(LOG_NOTICE,"yaUsbIr:     OUT endpoint 0x%02x\n", ep_out);
					}
				}
				if (!ep_in) continue;
				if (!usb) {
					usb = usb_open(dev);
					if (!usb) {
						logprintf(LIRC_ERROR,"yaUsbIr: unable to open device");
						break;
					}
				}

				usb_get_string_simple(usb, 1,(char *)buf, sizeof(buf));
				usb_get_string_simple(usb, 2,text, sizeof(text));
				if (info)
					logprintf(LIRC_NOTICE,"         Manufacturer: %s\n                Product: %s\n                hid interface (generic)", buf,text);

				if (usb_get_driver_np(usb, ifacenum, (char *)buf, sizeof(buf)) >= 0) {
					if (info)
						logprintf(LIRC_NOTICE,"yaUsbIr: in use by driver \"%s\"", buf);

					if (usb_detach_kernel_driver_np(usb, ifacenum) < 0) {
						logprintf(LIRC_ERROR,"yaUsbIr: unable to detach from kernel");
						continue;
					}
				}
				
				if (usb_claim_interface(usb, ifacenum) < 0) {
					logprintf(LIRC_ERROR,"yaUsbIr: unable to claim interface %d", ifacenum);
					continue;
				}
/*				
				len = usb_control_msg(usb, 0x81, 6, 0x2200, ifacenum, (char *)buf, sizeof(buf), 250);
				//logprintf(LIRC_NOTICE,"descriptor, len=%d\n", len);
				if (len < 2) {
					usb_release_interface(usb, ifacenum);
					continue;
				}
*/				
				hid = (raw_hid *)malloc(sizeof(raw_hid));
				if (!hid) {
					usb_release_interface(usb, ifacenum);
					continue;
				}
				hid->usb = usb;
				hid->iface = ifacenum;
				hid->ep_in = ep_in;
				hid->ep_out = ep_out;
				return hid;
			}
			if (usb) usb_close(usb);
		}
    }
    return NULL;
}

/*****************************************************************************
 *   rawhidClose - close a device
 *
 *    Inputs:
 *  hid = device to close
 *    Output
 *  (nothing)
 *
 ****************************************************************************/
void rawhidclose(raw_hid **hid)
{
	if ((hid==NULL)||(*hid==NULL)) return;
	usb_release_interface((*hid)->usb, (*hid)->iface);
	usb_close((*hid)->usb);
	free(*hid);
	*hid = NULL;
}

/*****************************************************************************
 *
 *
 ****************************************************************************/
static int parsesamples(int pipe_rxir_w)
{
	lirc_t rcvdata = 0;
	int num, n;
	uint8_t ya_usbir_rxbuf[64];

	if (ya_usbir_dev == NULL) return 0;
	num = rawhidrecv(ya_usbir_dev, ya_usbir_rxbuf, sizeof(ya_usbir_rxbuf), 50000);
	if (num < 0) {
		logprintf(LIRC_ERROR,"yaUsbIr: error reading, device went offline");
		return 0;
	}

	if ((num > 2) && (ya_usbir_rxbuf[0] == CMD_IRDATA)) {
		for (n=2; n < sizeof(ya_usbir_rxbuf) ;n += 2) {
			rcvdata = ((int)ya_usbir_rxbuf[n]&0x7F)<<8;// MSB
			rcvdata |= ya_usbir_rxbuf[n+1];// LSB
			rcvdata *= ya_usbir_rxbuf[1];// us step
			if (rcvdata==IRRX_NODATA) break;
			if ((ya_usbir_rxbuf[n] & 0x80)==0x80)
				rcvdata |= PULSE_BIT;
			chk_write(pipe_rxir_w, &rcvdata, sizeof(rcvdata));// Send the sample
			//logprintf(LIRC_NOTICE, "yaUsbIr: i=%2d, data=0x%08lx (0x%02x%02x)", n, rcvdata, ya_usbir_rxbuf[n], ya_usbir_rxbuf[n+1]);
		}
	}

	return 1;
}

/*****************************************************************************
 *
 *
 ****************************************************************************/
static void child_process(int fd_rx2main, int fd_main2tx, int fd_tx2main)
{
	int ret;
	int ya_usbir_ok = 0;
	unsigned char buf[64];
	int transmit_ir;

	alarm(0);
	signal(SIGTERM, SIG_DFL);
	signal(SIGPIPE, SIG_DFL);
	signal(SIGINT, SIG_DFL);
	signal(SIGHUP, SIG_IGN);
	signal(SIGALRM, SIG_IGN);

	while (1) {
   	     	if (ya_usbir_dev!=NULL) {
			rawhidclose(&ya_usbir_dev);
			ya_usbir_dev = NULL;
                }
		ya_usbir_dev = rawhidopen(usb_vendor, usb_product, first);// Open the USB device

		if ((ya_usbir_dev == NULL) && (ya_usbir_ok == 0)) {
			if (first > 0)
				logprintf(LIRC_ERROR, "yaUsbIr: unable to open yaUsbIr device '%04x:%04x'", usb_vendor, usb_product);
			ya_usbir_ok = 1;
		}
		if ((ya_usbir_dev != NULL) && (ya_usbir_ok == 1)) {
			logprintf(LIRC_NOTICE, "yaUsbIr: connect to yaUsbIr device '%04x:%04x'", usb_vendor, usb_product);
			ya_usbir_ok = 0;
		}
		if (first==1) {
			ret = write(fd_tx2main, &ret, 1);// indicate we're started
			first = 0;
		}

                if (ya_usbir_dev != NULL) {
			transmit_ir = 0;
			do {
				// transmit IR
				ret = read(fd_main2tx, buf, sizeof(buf));
				if (ret > 0) {
					/*
					char text[1024];
					int t;
					text[0] = 0;
					for (t=0;t<sizeof(buf);t++) {
						sprintf(text+strlen(text),"%02x ",buf[t]);
					}
					text[(3*32)-1] = '\n';
					logprintf(LIRC_NOTICE, "yaUsbIr: transmit IR =\n%s", text);
					*/
					rawhidsend(ya_usbir_dev, buf, sizeof(buf), 2000);// send it
					transmit_ir = 1;
					continue;
				}
				if (transmit_ir == 1) {
					transmit_ir = 0;
					ret = write(fd_tx2main, &ret, 1);// signal transmission ready
					usleep(5000);
					continue;
				}
				
				// receive IR
				ret = parsesamples(fd_rx2main);

			} while (ret > 0);
                }

		usleep(500000);// Wait a while and try again
	}
}

/*****************************************************************************
 * This function is called by the LIRC daemon when the first client
 * registers itself.
 * Returns 1 on success, 0 on error.
 *
 ****************************************************************************/
static int ya_usbir_init()
{
	int flags;
	int pipe_rx2main[2] = { -1, -1 };
	unsigned char buf[1];

	logprintf(LIRC_INFO, "yaUsbIr: Initializing yaUsbIr");

	rec_buffer_init();

	// Allocate a pipe for lircd to read from
	if (pipe(pipe_rx2main) == -1) {
		logprintf(LIRC_ERROR, "yaUsbIr: unable to create pipe_rx2main");
		goto fail_start;
	}
	if (pipe(pipe_main2tx) == -1) {
		logprintf(LIRC_ERROR, "yaUsbIr: unable to create pipe_main2tx");
		goto fail_main2tx;
	}
	if (pipe(pipe_tx2main) == -1) {
		logprintf(LIRC_ERROR, "yaUsbIr: unable to create pipe_tx2main");
		goto fail_tx2main;
	}

	drv.fd = pipe_rx2main[0];

	flags = fcntl(drv.fd, F_GETFL);

	// make the read end of the pipe non-blocking
	if (fcntl(drv.fd, F_SETFL, flags | O_NONBLOCK) == -1) {
		logprintf(LIRC_ERROR, "yaUsbIr: unable to make pipe read end non-blocking");
		goto fail;
	}

	// Make the read end of the send pipe non-blocking
	flags = fcntl(pipe_main2tx[0], F_GETFL);
	if (fcntl(pipe_main2tx[0], F_SETFL, flags | O_NONBLOCK) == -1) {
		logprintf(LIRC_ERROR, "yaUsbIr: unable to make pipe read end non-blocking");
		goto fail;
	}

	// Spawn the child process
	child_pid = fork();
	if (child_pid == -1) {
		logprintf(LIRC_ERROR, "yaUsbIr: unable to fork child process");
		goto fail;
	} else if (child_pid == 0) {
		// we're the child
		close(pipe_rx2main[0]);
		close(pipe_main2tx[1]);
		close(pipe_tx2main[0]);
		child_process(pipe_rx2main[1], pipe_main2tx[0], pipe_tx2main[1]);
	}

	// we're the parent
	close(pipe_rx2main[1]);
	close(pipe_main2tx[0]);
	pipe_main2tx[0] = -1;
	close(pipe_tx2main[1]);
	pipe_tx2main[1] = -1;

	chk_read(pipe_tx2main[0], &buf, 1);// wait for child to be started

	//logprintf(LIRC_INFO, "yaUsbIr: child started");

	return 1;

fail:
	drv.fd = -1;

	close(pipe_tx2main[0]);
	close(pipe_tx2main[1]);
	pipe_tx2main[0] = -1;
	pipe_tx2main[1] = -1;

fail_tx2main:
	close(pipe_main2tx[0]);
	close(pipe_main2tx[1]);
	pipe_main2tx[0] = -1;
	pipe_main2tx[1] = -1;

fail_main2tx:
	close(pipe_rx2main[0]);
	close(pipe_rx2main[1]);

fail_start:
	return 0;
}

/*****************************************************************************
 * Close and release the serial line.
 * Returns 1 on success, 0 on error.
 *
 ****************************************************************************/
static int ya_usbir_deinit(void)
{
	//logprintf(LIRC_NOTICE, "yaUsbIr: Entering deinit()");
	if (child_pid != -1) {
		// Kill the child process, and wait for it to exit
		if (kill(child_pid, SIGTERM) == -1)
			return 0;

		if (waitpid(child_pid, NULL, 0) == 0)
			return 0;

		child_pid = -1;
	}

	close(drv.fd);
	drv.fd = -1;

	close(pipe_main2tx[1]);
	pipe_main2tx[1] = -1;
	close(pipe_tx2main[0]);
	pipe_tx2main[0] = -1;

	rawhidclose(&ya_usbir_dev);
	ya_usbir_dev = NULL;

	return 1;
}

/*****************************************************************************
 * Receive code from the remote.
 * This function is called by the LIRC daemon when I/O is pending
 * from a registered client, e.g. irw.
 *
 * Returns NULL if nothing has been received, lirc code otherwise
 *
 ****************************************************************************/
static char *ya_usbir_rec(struct ir_remote *remotes)
{
	if (!rec_buffer_clear()) return (NULL);
	return (decode_all(remotes));
}

/*****************************************************************************
 * Read data from the remote. timeout is in us.
 *
 * Returns lirc_t type data; 0 on error or timeout
 *
 ****************************************************************************/
static lirc_t ya_usbir_readdata(lirc_t timeout)
{
	int n;
	lirc_t res = 0;

	if (!waitfordata((long)timeout))
		return 0;

	n = read(drv.fd, &res, sizeof res);
	if (n != sizeof res)
		res = 0;

	return res;
}

/*****************************************************************************
 * Send to the remote.
 * This function is called by the LIRC daemon
 *
 * Returns number of bytes sent
 *
 ****************************************************************************/
int ya_usbir_send(struct ir_remote *remote, struct ir_ncode *code)
{
	int length, i;
	lirc_t txbuf;
	uint8_t ya_usbir_txbuf[64];
	int ya_usbir_txbufpos;
	unsigned int freq;
	double pwm;
	const lirc_t* txptr;
	//double step; // unused

	logprintf(LIRC_NOTICE, "yaUsbIr: entering send()");

	// initialize decoded buffer
	if (!send_buffer_put(remote, code)) {
		logprintf(LIRC_ERROR, "yaUsbIr: init_send() failed");
		return 0;
	}
	if ((send_buffer_length() == 0)||(send_buffer_data() == 0)) {
		logprintf(LIRC_ERROR, "yaUsbIr: nothing to send");
		return 0;
	}
	length = send_buffer_length();
	txptr = send_buffer_data();

	ya_usbir_txbufpos = 0;
	for ( i=0; i<length; i++, txptr++) {
		txbuf = *txptr;
		if (ya_usbir_txbufpos==0) {
			memset(ya_usbir_txbuf, 0, sizeof(ya_usbir_txbuf));
			ya_usbir_txbuf[ya_usbir_txbufpos++] = CMD_IRDATA;
			
//			ya_usbir_txbuf[ya_usbir_txbufpos++] = 85;// 35,21 Khz
//			ya_usbir_txbuf[ya_usbir_txbufpos++] = 84;// 35,71 Khz
//			ya_usbir_txbuf[ya_usbir_txbufpos++] = 83;// 36,24 Khz
//			ya_usbir_txbuf[ya_usbir_txbufpos++] = 82;// 36,50 Khz
//			ya_usbir_txbuf[ya_usbir_txbufpos++] = 81;// 37,04 Khz
//			ya_usbir_txbuf[ya_usbir_txbufpos++] = 80;// 37,50 Khz
//			ya_usbir_txbuf[ya_usbir_txbufpos++] = 79;// 37,88 Khz
//			ya_usbir_txbuf[ya_usbir_txbufpos++] = 78;// 38,46 Khz
//			ya_usbir_txbuf[ya_usbir_txbufpos++] = 77;// 39,06 Khz
//			ya_usbir_txbuf[ya_usbir_txbufpos++] = 76;// 39,38 Khz
//			ya_usbir_txbuf[ya_usbir_txbufpos++] = 75;// 40Khz

			freq = remote->freq;
			if (freq == 0) freq = 38000;
			if (freq < 30000) freq = 30000;
			if (freq > 42000) freq = 42000;
			if ((remote->freq != 0) && (txbuf < IRRX_CMD)) // is not a yaUsbIR V3 command
				logprintf(LIRC_NOTICE, "yaUsbIr: set carrier frequency = %d Hz", freq);
			pwm = 1.0 / (((double)freq * 2.0) / (double)IRRX_F_POLL);
			//logprintf(LIRC_NOTICE, "yaUsbIr: pwm = %d",(int)pwm);
			ya_usbir_txbuf[ya_usbir_txbufpos++] = (uint8_t)pwm;
		}

		if (txbuf>=IRRX_CMD) // is a yaUsbIR V3 command
			txbuf /= 0x0D;
		else 
			txbuf = (lirc_t)(((double)txbuf)/(((double)ya_usbir_txbuf[1])/6.0));
		if (txbuf>0x7FFF) txbuf = 0x7FFF;
		txbuf |= (i%2 == 0) ? 0x8000:0x0000;// Pulse or Space

		ya_usbir_txbuf[ya_usbir_txbufpos++] = (txbuf>>8) & 0xFF;// MSB
		ya_usbir_txbuf[ya_usbir_txbufpos++] =  txbuf	& 0xFF;// LSB
		//logprintf(LIRC_NOTICE, "yaUsbIr: i=%2d, lircdata=0x%08lx (yaUsbIRdata=0x%04x)", i, *txptr, txbuf);

		if (ya_usbir_txbufpos >= sizeof(ya_usbir_txbuf)) {// send it
			// let the child process transmit the pattern
			chk_write(pipe_main2tx[1], ya_usbir_txbuf, sizeof(ya_usbir_txbuf));
			ya_usbir_txbufpos = 0;
		}
	}
	if (ya_usbir_txbufpos > 2) // send it
		// let the child process transmit the pattern
		chk_write(pipe_main2tx[1], ya_usbir_txbuf, sizeof(ya_usbir_txbuf));

	chk_read(pipe_tx2main[0], ya_usbir_txbuf, 1);// wait for child process to be ready with it

	return 1;
}

/*****************************************************************************
 * Definition of the standard internal hardware interface
 * used by lirc for the yaUsbIr device
 *
 ****************************************************************************/
const struct driver hw_ya_usbir = {
        .name           =       "ya_usbir",
        .device         =       "",
        .features       =       LIRC_CAN_REC_MODE2 | \
                                LIRC_CAN_SEND_PULSE,
        .send_mode      =       LIRC_MODE_PULSE,
        .rec_mode       =       LIRC_MODE_MODE2,
        .code_length    =       0,
        .init_func      =       ya_usbir_init,
        .deinit_func    =       ya_usbir_deinit,
        .open_func      =       default_open,
        .close_func     =       default_close,
        .send_func      =       ya_usbir_send,
        .rec_func       =       ya_usbir_rec,
        .decode_func    =       receive_decode,
        //.drvctl_func    =       hwftdi_ioctl,
        .readdata       =       ya_usbir_readdata,
        .api_version    =       2,
        .driver_version =       "0.9.4",
        .info           =       "No info available"
};

const struct driver* hardwares[] = { &hw_ya_usbir, (const struct driver*)NULL };
