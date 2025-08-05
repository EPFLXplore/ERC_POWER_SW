/*
 * FT81xRenderer.c
 *
 *  Created on: 29 May 2023
 *      Author: arion
 */

/**
 *  @file    FT81xRenderer.c
 *  @author  Sean Mathews <coder@f34r.com>
 *  @date    08/01/2018
 *  @version 1.1
 *
 *  @brief API to communicate with a FT81xRenderer chip from a ESP32 uP
 *
 *  This code provides an API to communicate with the FT81xRenderer chip
 *  from an ESP32. It simplifies the complexity of SPI on the ESP32
 *  correctly formatting the SPI communications to work with
 *  the FT81xRenderer. It also allow for QUAD SPI communications where
 *  permitted to increase data transfer speeds. The FT81xRenderer and the
 *  ESP32 are both little-endian so byte order does not need to
 *  be adjusted.
 *
 *
 *  @copyright Copyright (C) 2018 Nu Tech Software Solutions, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include "FT81xRenderer.h"
#include "FreeRTOS.h"

#include <cstring>

FT81xRenderer::FT81xRenderer(QSPIWrapper *spi, Console *console) : spi(spi), console(console) {

}

void FT81xRenderer::load_default_command(QSPI_CommandTypeDef *cmd) {
	cmd->Instruction = 0;
	cmd->Address = 0;
	cmd->AlternateBytes = 0;
	cmd->AddressSize = 0;
	cmd->AlternateBytesSize = 0;
	cmd->DummyCycles = 0;
	cmd->InstructionMode = QSPI_INSTRUCTION_NONE;
	cmd->AddressMode = QSPI_ADDRESS_NONE;
	cmd->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	cmd->DataMode = QSPI_DATA_NONE;
	cmd->NbData = 0;
	cmd->DdrMode = QSPI_DDR_MODE_DISABLE;
	cmd->DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	cmd->SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
}

void FT81xRenderer::restart_core() {
	// FIXME: Host Commands must be sent in single byte mode. Just in case the FT81xRenderer is stuck
	// in QUAD mode attempt to change it to single byte mode. Set the Quad Mode flag so that
	// the Write tries to perform it in that mode, then reset the flag for single byte mode.

	/*qio = 1;
	wr16(REG_SPI_WIDTH, SPI_WIDTH_SINGLE);
	qio = 0;*/

	spi->setup(SPI_BAUDRATEPRESCALER_16);

	// Put the FT81xRenderer to sleep.
	hostcmd(CMD_SLEEP);
	// Set default clock speed.
	hostcmd_param(CMD_CLKSEL, 0x00);
	// Performing a read at address zero will return to an Active mode. Documentation suggests
	// doing two reads followed by at least a 20ms delay when returning from a Sleep state to
	// allow things to settle.
	rd(CMD_ACTIVE);
	rd(CMD_ACTIVE);
	vTaskDelay(20 / portTICK_PERIOD_MS);
	// Select internal clock (default), which may cause a system reset.
	hostcmd(CMD_CLKINT);
	// Power up all all ROMs.
	hostcmd_param(CMD_PD_ROMS, 0x00);
	// Power up without resetting the things just done above.
	hostcmd(CMD_RST_PULSE);
}

bool FT81xRenderer::test_comm() {
	console->printf("Performing communication test with FT813 touch-screen...\r\n");

	if(rd16(REG_ID) == 0x7C) {
		console->printf("[FT813] Read succeeded\r\n");
	} else {
		console->printf("[FT813] Read failed\r\n");
		return false;
	}

	uint16_t current_state = rd16(0);
	uint16_t next_state = ~current_state + 0xC0FFEE;

	wr16(0, next_state);

	uint16_t new_state = rd16(0);

	if(new_state == next_state) {
		console->printf("[FT813] Write succeeded\r\n");
	} else {
		console->printf("[FT813] Write failed\r\n");
		return false;
	}

	return true;
}

bool FT81xRenderer::read_chip_id() {


	// Read CHIP ID address until it returns a valid result.
	for (uint16_t count = 0; count < 100; count++) {
		chip_id = rd16(MEM_CHIP_ID);
		// Chip id: 08h, [id], 01h, 00h
		// [id]: FT8xx=10h, 11h, 12h, 13h
		if ((chip_id & 0xff) == 0x08) {
			//ESP_LOGW(TAG, "HWID: 0x%04x", chip_id);
			return true;
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	};
	//ESP_LOGW(TAG, "HWID: 0x%04x", chip_id);
	chip_id = 0;
	return false;
}

void FT81xRenderer::select_spi_byte_width() {
	// Enable QUAD spi mode if configured
#if (FT_QUADSPI)
	// Enable QAUD spi mode no dummy
	wr16(REG_SPI_WIDTH, SPI_WIDTH_QUAD);
	qio = 1;
#else
	// Enable single channel spi mode
	wr16(REG_SPI_WIDTH, SPI_WIDTH_SINGLE);
	qio = 0;
#endif
}


void FT81xRenderer::assert_cs(bool asserted) {
	if(asserted) {
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	}
}


void FT81xRenderer::init_display_settings() {
	// Screen specific settings
	// GEN4 - 4.3'

	wr32(REG_HCYCLE, 531);
	wr32(REG_HOFFSET, 43);
	wr32(REG_HSIZE, DISPLAY_WIDTH);
	wr32(REG_HSYNC0, 0);
	wr32(REG_HSYNC1, 4);
	wr32(REG_VCYCLE, 292);
	wr32(REG_VOFFSET, 12);
	wr32(REG_VSIZE, DISPLAY_HEIGHT);
	wr32(REG_VSYNC0, 0);
	wr32(REG_VSYNC1, 4);
	wr32(REG_DITHER, 1);
	wr32(REG_PCLK_POL, 1);
	wr(REG_ROTATE, 0);
	wr(REG_SWIZZLE, 0);
	// Get screen size W,H to confirm
	display_width = rd16(REG_HSIZE);
	display_height = rd16(REG_VSIZE);

	console->printf("[FT81xRenderer] REG_HSIZE:%i  REG_VSIZE:%i\r\n",
			display_width, display_height);

	wr(REG_PCLK, 6);
}

void FT81xRenderer::init_touch_settings() {
	// Turn on=0/(off=1) multi-touch
	wr(REG_CTOUCH_EXTENDED, 1);
}

void FT81xRenderer::init_audio_settings() {
	// Turn playback volume off
	wr(REG_VOL_PB, 0);
	// Turn synthesizer volume off
	wr(REG_VOL_SOUND, 0);
	// Set synthesizer to 'Mute'
	wr(REG_SOUND, MUTE);
}

void FT81xRenderer::init_gpio() {
	// Setup the FT81xRenderer GPIO PINS. These assume little-endian.
	// DISP = output, GPIO 3 - 0 = input

	wr16(REG_GPIOX_DIR, 0x8000);
	// Turn on GPIO power to 10ma for SPI pins: MOSI, IO2, IO3, INT_N
	// Retain all other settings.
	wr16(REG_GPIOX, (rd16(REG_GPIOX) & !0xc00) | 0x400);
	// Sleep a little
	vTaskDelay(100 / portTICK_PERIOD_MS);
}

#ifdef configFT_BLACK_TEST // Test with black screen before starting display clock
void FT81xRenderer:: test_black_screen() {
	// Build a black display and display it
	stream_start(); // Start streaming
	cmd_dlstart();  // Set REG_CMD_DL when done
	cmd_swap();     // Set AUTO swap at end of display list
	clear_color_rgb32(0x000000);
	clear();
	display();
	getfree(0);     // trigger FT81xRenderer to read the command buffer
	stream_stop();  // Finish streaming to command buffer
	wait_finish(); // Wait till the GPU is finished processing the commands
	// Sleep a little
	vTaskDelay(100 / portTICK_PERIOD_MS);
}
#else
void FT81xRenderer::test_black_screen() {}
#endif

#ifdef configFT_SOUND_TEST // SOUND test
void FT81xRenderer:: test_sound(
)
{
	// Set volume to MAX
	wr(REG_VOL_SOUND,0xff);

	for(int a = 0; a < 4; a++) {

		// Turn on the audio amp connected to GPIO1
		wr16(REG_GPIOX_DIR, rd16(REG_GPIOX_DIR) | (0x1 << 1));
		wr16(REG_GPIOX, rd16(REG_GPIOX) | (0x1 << 1));

		wr16(REG_SOUND,(0x3C<< 8) | 0x52);
		wr(REG_PLAY, 1);
		//// Wait till the sound is finished
		while(rd(REG_PLAY)) {
			// Sleep a little
			vTaskDelay(50 / portTICK_PERIOD_MS);
		}

		// Turn off the audio amp connected to GPIO3
		wr16(REG_GPIOX_DIR, rd16(REG_GPIOX_DIR) & ~(0x1 << 1));
		wr16(REG_GPIOX, rd16(REG_GPIOX) & ~(0x1 << 1));

	}
	// Set volume to MUTE
	wr(REG_VOL_SOUND,0);
}
#else
void FT81xRenderer::test_sound() {}
#endif

#ifdef configFT_LOGO_TEST
// Display the built in FTDI logo animation and then calibrate
void FT81xRenderer:: test_logo(
)
{


	logo();
	calibrate();

}
#else
void FT81xRenderer::test_logo() {}
#endif

#ifdef configFT_IMAGE_TEST
	// Test LOAD_IMAGE ON and OFF with a transparent PNG and update when touched
void FT81xRenderer:: test_load_image(
)
{
	uint32_t imgptr, widthptr, heightptr;
	uint32_t ptron, ptroff, ptrnext, width, height;
	uint32_t lasttag = 0;
	uint8_t  soundplaying = 0;

  // wakeup the display and set brightness
  wake(22);

	// Load the OFF image to the MEDIA FIFO
	//// Start streaming
	stream_start();

	//// Configure MEDIA FIFO
	cmd_mediafifo(0x100000UL-0x40000UL, 0x40000UL);

	//// Trigger FT81xRenderer to read the command buffer
	getfree(0);

	//// Finish streaming to command buffer
	stream_stop();

	//// Wait till the GPU is finished
	wait_finish();

	//// stop media fifo
	wr32(REG_MEDIAFIFO_WRITE, 0);

	//// Load the image at address 0
	ptroff = 0;

	// Load the OFF image
	//// Start streaming
	stream_start();

	//// USE MEDIA_FIFO
	//// Load the image at address transparent_test_file_png_len+1
	cmd_loadimage(ptroff, OPT_RGB565 | OPT_NODL | OPT_MEDIAFIFO);

	//// Get the decompressed image properties
	cmd_getprops(&imgptr, &widthptr, &heightptr);

	//// Trigger FT81xRenderer to read the command buffer
	getfree(0);

	//// Finish streaming to command buffer
	stream_stop();

	//// Send the image to the media fifo
	cSPOOL_MF(transparent_test_file_off_png, transparent_test_file_off_png_len);

	//// Wait till the GPU is finished
	wait_finish();

	//// Dump results
	ptron = rd32(imgptr); // pointer to end of image and start of next free memory
	width = rd32(widthptr);
	height = rd32(heightptr);
	console->printf("[FT81xRenderer] loadimage off: start:0x%04x end: 0x%04x width: 0x%04x height: 0x%04x\r\n", ptroff, ptron-1, width, height);

	// Load the OFF image
	//// Start streaming
	stream_start();

	//// USING CMD BUFFER. Max size is ~4k
	//// Load the image at address transparent_test_file_png_len+1 using CMD buffer
	cmd_loadimage(ptron, OPT_RGB565 | OPT_NODL);

	//// spool the image to the FT81xRenderer
	cSPOOL((uint8_t *)transparent_test_file_on_png, transparent_test_file_on_png_len);

	//// Get the decompressed image properties
	cmd_getprops(&imgptr, &widthptr, &heightptr);

	//// Trigger FT81xRenderer to read the command buffer
	getfree(0);

	//// Finish streaming to command buffer
	stream_stop();

	//// Wait till the GPU is finished
	wait_finish();

	//// Dump results
	ptrnext = rd32(imgptr); // pointer to end of image and start of next free memory
	width = rd32(widthptr);
	height = rd32(heightptr);
	console->printf("[FT81xRenderer] loadimage on: start:0x%04x end: 0x%04x width: 0x%04x height: 0x%04x\r\n", ptron, ptrnext-1, width, height);

	//get_touch_inputs();
	//ESP_LOGW(TAG, "ctouch mode: 0x%04x multi-touch: %s", touch_mode(), multi_touch_enabled() ? "true" : "false");

	// Capture input events and update the image if touched
	uint8_t sound = 0x40;
	for (int x=0; x<1000; x++) {

#if 1 // TEST SOUND TOUCH FEEDBACK
		if(touch_input[0].tag) {
			if(touch_input[0].tag != lasttag) {
				lasttag = touch_input[0].tag;#if 1
				// Max volume
					wr(REG_VOL_SOUND,0xff);
				// Turn ON the AMP using enable pin connected to GPIO3
				wr16(REG_GPIOX_DIR, rd16(REG_GPIOX_DIR) | (0x1 << 3));
				wr16(REG_GPIOX, rd16(REG_GPIOX) | (0x1 << 3));

				wr16(REG_SOUND,(0x4C<< 8) | sound++);
				wr(REG_PLAY, 1);

				//wr(REG_VOL_PB,0xFF);//configure audio playback volume
				//wr32(REG_PLAYBACK_START,0);//configure audio buffer starting address
				//wr32(REG_PLAYBACK_LENGTH,100*1024);//configure audio buffer length
				//wr16(REG_PLAYBACK_FREQ,44100);//configure audio sampling frequency
				//wr(REG_PLAYBACK_FORMAT,ULAW_SAMPLES);//configure audio format
				//wr(REG_PLAYBACK_LOOP,0);//configure once or continuous playback
				//wr(REG_PLAYBACK_PLAY,1);//start the audio playback

				soundplaying = 1;
				if(sound>0x58)
					sound = 0x40;
			}
		} else
			lasttag = 0;

		if(soundplaying && !rd(REG_PLAY)) {
			soundplaying = 0;
			// Mute
			wr(REG_VOL_SOUND,0);
			// Turn OFF the AMP using enable pin connected to GPIO3
			wr16(REG_GPIOX_DIR, rd16(REG_GPIOX_DIR) & ~(0x1 << 3));
			wr16(REG_GPIOX, rd16(REG_GPIOX) & ~(0x1 << 3));
		}
#endif

		// Start streaming
		stream_start();

		// Define the bitmap we want to draw
		cmd_dlstart();  // Set REG_CMD_DL when done
		cmd_swap();     // Set AUTO swap at end of display list

		// Draw ON/OFF based upon touch
		if (touch_input[0].tag) {
			console->printf("[FT81xRenderer] touched\r\n");

			// Clear the display
			clear_color_rgb32(0x28e800);
			clear();
			// Draw the image
			bitmap_source(ptron);

		} else {
			// Clear the display
			clear_color_rgb32(0xfdfdfd);
			clear();
			// Draw the image
			bitmap_source(ptroff);

		}


		// Turn on tagging
		tag_mask(1);

		// Track touches for a specific object
		//cmd_track(1, 1, display_width, display_height, 3); // track touches to the tag

		bitmap_layout(ARGB4, 75*2, 75);
		bitmap_size(NEAREST, BORDER, BORDER, 75, 75);
		begin(BITMAPS);

		tag(3); // tag the image button #3
		vertex2ii(100, 100, 0, 0);

		// stop tagging
		tag_mask(0);

		// end of commands
		end();
		display();

		// Trigger FT81xRenderer to read the command buffer
		getfree(0);

		// Finish streaming to command buffer
		stream_stop();

		//// Wait till the GPU is finished
		wait_finish();

		// download the display touch memory into touch_tracker
		get_touch_inputs();
#ifdef configFT_DEBUG
		console->printf("[FT81xRenderer] tag0: %i xy0: 0x%04x,0x%04x\r\n", touch_input[0].tag, touch_input[0].tag_x, touch_input[0].tag_y);
		// multitouch
		// ESP_LOGW(TAG, "tag1: %i xy0: 0x%04x,0x%04x", touch_input[1].tag, touch_input[1].tag_x, touch_input[1].tag_y);
#endif
		// Sleep
		vTaskDelay(10 / portTICK_PERIOD_MS);

	}
}
#else
void FT81xRenderer::test_load_image() {}
#endif

#ifdef configFT_MEM_TEST
// Test memory operation(s) and CRC32 on 6 bytes of 0x00 will be 0xB1C2A1A3
void FT81xRenderer:: test_memory_ops() {


	// Start streaming
	stream_start();

	// Write a predictable sequence of bytes to a memory location
	cmd_memzero(0, 0x0006);

	// Calculate crc on the bytes we wrote
	uint32_t r = cmd_memcrc(0, 0x0006);

	// Trigger FT81xRenderer to read the command buffer
	getfree(0);

	// Finish streaming to command buffer
	stream_stop();

	// Wait till the GPU is finished
	wait_finish();

	// Dump results
	uint32_t res = rd32(r);
	console->printf("[FT81xRenderer] crc: ptr: 0x%04x val: 0x%4x expected: 0xB1C2A1A3\r\n", r, res);
}
#else
void FT81xRenderer:: test_memory_ops() {}
#endif

#ifdef configFT_DISP_TEST
// Draw a gray screen and write Hello World, 123, button etc.
void FT81xRenderer:: test_display(
)
{
	wr(REG_PWM_DUTY, 128);

	// Wait till the GPU is finished
	for (int x=0; x<300; x++) {
		// Sleep
		vTaskDelay(200 / portTICK_PERIOD_MS);

		// download the display touch memory into touch_tracker
		get_touch_inputs();

		stream_start(); // Start streaming
		cmd_dlstart();  // Set REG_CMD_DL when done
		cmd_swap();     // Set AUTO swap at end of display list
		clear_color_rgb32(0xfdfdfd);
		clear();
		color_rgb32(0x101010);
		bgcolor_rgb32(0xff0000);
		fgcolor_rgb32(0x0000ff);

		// Turn off tagging
		tag_mask(0);

		// Draw some text and a number display value of dial
		cmd_text(240, 300, 30, OPT_CENTERY, "Hello World");

		cmd_text(130, 200, 30, OPT_RIGHTX, "TAG");
		cmd_number(140, 200, 30, 0, touch_tracker[0].tag);

		cmd_text(130, 230, 30, OPT_RIGHTX, "VALUE");
		cmd_number(140, 230, 30, 0, touch_tracker[0].value * 100 / TRACKER_UNITS);

		bgcolor_rgb32(0x007f7f);
		cmd_clock(730,80,50,0,12,1,2,4);

		// Turn on tagging
		tag_mask(1);

		tag(3); // tag the button #3
		cmd_track(10, 10, 140, 100, 3); // track touches to the tag
		cmd_button(10, 10, 140, 100, 31, 0, "OK");

		tag(4); // tag the button #4
		cmd_track(300, 100, 1, 1, 4); // track touches to the tag
		cmd_dial(300, 100, 100, OPT_FLAT, x * 100);

		uint8_t tstate = rand()%((253+1)-0) + 0;
		if(tstate > 128)
			bgcolor_rgb32(0x00ff00);
		else
			bgcolor_rgb32(0xff0000);

		tag(5); // tag the spinner #5
		cmd_toggle(500, 100, 100, 30, 0, tstate > 128 ? 0 : 65535, "YES\xffNO");

		// Turn off tagging
		tag_mask(0);

		// Draw a keyboard
		cmd_keys(10, 400, 300, 50, 26, 0, "12345");

		// FIXME: Spinner if used above does odd stuff? Seems ok at the end of the display.
		cmd_spinner(500, 200, 3, 0);

		display();
		getfree(0);     // trigger FT81xRenderer to read the command buffer
		stream_stop();  // Finish streaming to command buffer

		//// Wait till the GPU is finished
		wait_finish();
	}
}
#else
void FT81xRenderer::test_display() {}
#endif

#ifdef configFT_COLOR_TEST
// Fill the screen with a solid color cycling colors
void FT81xRenderer:: test_cycle_colors(
)
{
	uint32_t rgb = 0xff0000;
	for (int x=0; x<300; x++) {

		stream_start(); // Start streaming
		cmd_dlstart();  // Set REG_CMD_DL when done
		cmd_swap();     // Set AUTO swap at end of display list
		clear_color_rgb32(rgb);
		clear();
		color_rgb32(0xffffff);
		bgcolor_rgb32(0xffffff);
		fgcolor_rgb32(0xffffff);
		display();
		getfree(0);     // trigger FT81xRenderer to read the command buffer
		stream_stop();  // Finish streaming to command buffer

		// rotate colors
		rgb>>=8; if(!rgb) rgb=0xff0000;

		// Sleep
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
#else
void FT81xRenderer::test_cycle_colors() {}
#endif

#ifdef configFT_RENDER_TEST
// Draw some dots of rand size, location and color.
void FT81xRenderer:: test_dots(
)
{
	for (int x=0; x<300; x++) {

		stream_start(); // Start streaming
		//alpha_funct(0b111, 0b00000000);
		//bitmap_handle(0b10101010);
		bitmap_layout(0b11111, 0x00, 0x00);

		cmd_dlstart();  // Set REG_CMD_DL when done
		cmd_swap();     // Set AUTO swap at end of display list
		clear_color_rgb32(0x000000);
		clear();

		fgcolor_rgb32(0xffffff);
		bgcolor_rgb32(0xffffff);

		uint8_t rred, rgreen, rblue;
		rred = rand()%((253+1)-0) + 0;
		rgreen = rand()%((253+1)-0) + 0;
		rblue = rand()%((253+1)-0) + 0;
		color_rgb888(rred, rgreen, rblue);
		begin(POINTS);
		uint16_t size = rand()%((600+1)-0) + 0;
		uint16_t rndx = rand()%((display_width+1)-0) + 0;
		uint16_t rndy = rand()%((display_height+1)-0) + 0;
		point_size(size);
		vertex2f(rndx<<4,rndy<<4); // defaut is 1/16th pixel precision
		console->printf("[FT81xRenderer] c: x:%i y:%i z:%i\r\n", rndx, rndy, size);
		display();
		getfree(0);     // trigger FT81xRenderer to read the command buffer
		stream_stop();  // Finish streaming to command buffer
		vTaskDelay(100 / portTICK_PERIOD_MS);

	}

	// Sleep
	vTaskDelay(10 / portTICK_PERIOD_MS);
}
#else
void FT81xRenderer::test_dots() {}
#endif

/*
 * Initialize the FT81xRenderer GPU and test for a valid chip response
 */
bool FT81xRenderer::init_gpu() {
	spi->setup(SPI_BAUDRATEPRESCALER_16);

	restart_core();

	if (!read_chip_id()) {
		return false;
	}

	select_spi_byte_width();

	// Set the PWM to 0 turn off the backlight
	wr(REG_PWM_DUTY, 0);

	fifo_reset();
	init_gpio();

	init_display_settings();
	init_touch_settings();
	init_audio_settings();

	// Clear screen
	stream_start(); // Start streaming
	cmd_dlstart();  // Set REG_CMD_DL when done
	cmd_swap();     // Set AUTO swap at end of display list
	clear_color_rgb32(0x000000);
	clear();
	display();
	getfree(0);     // trigger FT81xRenderer to read the command buffer
	stream_stop();  // Finish streaming to command buffer
	wait_finish();

	spi->setup(SPI_BAUDRATEPRESCALER_8);

	test_black_screen();
	test_sound();
	test_logo();
	test_load_image();
	test_memory_ops();
	test_display();
	test_cycle_colors();
	test_dots();

	return true;
}

/*
 * Turn off the display entering low power mode
 */
void FT81xRenderer::sleep() {
	// Disable the pixel clock
	wr32(REG_PCLK, 0);

	// Set PWM to 0
	wr(REG_PWM_DUTY, 0);

	// Turn the display off
	wr16(REG_GPIOX, rd16(REG_GPIOX) | 0x7fff);
}

/*
 * Wake the display up and set pwm level
 */
void FT81xRenderer::wake(uint8_t pwm) {
	// Enable the pixel clock
	wr32(REG_PCLK, 6);

	// Set PWM to pwm
	wr(REG_PWM_DUTY, pwm);

	// Turn the display on
	wr16(REG_GPIOX, rd16(REG_GPIOX) | 0x8000);
}

/*
 * Initialize our command buffer pointer state vars
 * for streaming mode.
 */
void FT81xRenderer::fifo_reset() {
	fifo_wp = fifo_rp();
	fifo_freespace = MAX_FIFO_SPACE;
}

/*
 * Get our current fifo write state location
 */
uint32_t FT81xRenderer::getwp() {
	return RAM_CMD + (fifo_wp & 0xffc);
}

/*
 * Write 16 bit command+arg and dummy byte
 * See 4.1.5 Host Command
 * Must never be sent in QUAD spi mode except for reset?
 * A total of 24 bytes will be on the SPI BUS.
 */
void FT81xRenderer::hostcmd_param(uint8_t command, uint8_t args) {

	// start the transaction ISR watches CS bit
	assert_cs(true);

	QSPI_CommandTypeDef cmd;
	load_default_command(&cmd);

	if (qio) {
		cmd.DataMode = QSPI_DATA_4_LINES;
		cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;

	} else {
		cmd.DataMode = QSPI_DATA_1_LINE;
		cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	}

	cmd.NbData = 1;
	cmd.Instruction = command;
	cmd.DummyCycles = 1;

	if (spi->run(&cmd, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to send QSPI command for hostcmd\r\n");
		return;
	}

	if (spi->transmit((uint8_t*) &args, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to transmit QSPI data for hostcmd\r\n");
		return;
	}

	// end the transaction
	assert_cs(false);
}

/*
 * Write 24 bit address + dummy byte
 * and read the 8 bit result.
 * A total of 6 bytes will be on the SPI BUS.
 */
uint8_t FT81xRenderer::rd(uint32_t addr) {
	uint8_t output;

	// start the transaction ISR watches CS bit
	assert_cs(true);

	QSPI_CommandTypeDef cmd;
	load_default_command(&cmd);

	if (qio) {
		cmd.DataMode = QSPI_DATA_4_LINES;
		cmd.AddressMode = QSPI_ADDRESS_4_LINES;

	} else {
		cmd.DataMode = QSPI_DATA_1_LINE;
		cmd.AddressMode = QSPI_ADDRESS_1_LINE;
	}

	cmd.NbData = 1;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;
	cmd.Address = addr;
	cmd.DummyCycles = 1;

	if (spi->run(&cmd, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to send QSPI command for read8\r\n");
		return -1;
	}

	if (spi->receive(&output, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to receive QSPI data for read8\r\n");
		return -1;
	}

	// end the transaction
	assert_cs(false);

	return output;
}

/*
 * Write 24 bit address + dummy byte
 * and read the 16 bit result.
 * A total of 6 bytes will be on the SPI BUS.
 */
uint16_t FT81xRenderer::rd16(uint32_t addr) {
	uint16_t output;

	// start the transaction ISR watches CS bit
	assert_cs(true);

	QSPI_CommandTypeDef cmd;
	load_default_command(&cmd);

	if (qio) {
		cmd.DataMode = QSPI_DATA_4_LINES;
		cmd.AddressMode = QSPI_ADDRESS_4_LINES;

	} else {
		cmd.DataMode = QSPI_DATA_1_LINE;
		cmd.AddressMode = QSPI_ADDRESS_1_LINE;
	}

	cmd.NbData = 2;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;
	cmd.Address = addr;
	cmd.DummyCycles = 1;

	if (spi->run(&cmd, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to send QSPI command for read16\r\n");
		return -1;
	}

	if (spi->receive((uint8_t*) &output, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to receive QSPI data for read16\r\n");
		return -1;
	}

	// end the transaction
	assert_cs(false);

	return output;
}

/*
 * Write 24 bit address + dummy byte
 * and read the 32 bit result.
 * A total of 8 bytes will be on the SPI BUS.
 */
uint32_t FT81xRenderer::rd32(uint32_t addr) {
	uint32_t output;

	// start the transaction ISR watches CS bit
	assert_cs(true);

	QSPI_CommandTypeDef cmd;
	load_default_command(&cmd);

	if (qio) {
		cmd.DataMode = QSPI_DATA_4_LINES;
		cmd.AddressMode = QSPI_ADDRESS_4_LINES;

	} else {
		cmd.DataMode = QSPI_DATA_1_LINE;
		cmd.AddressMode = QSPI_ADDRESS_1_LINE;
	}

	cmd.NbData = 4;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;
	cmd.Address = addr;
	cmd.DummyCycles = 1;

	if (spi->run(&cmd, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to send QSPI command for read32\r\n");
		return -1;
	}

	if (spi->receive((uint8_t*) &output, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to receive QSPI data for read32\r\n");
		return -1;
	}

	// end the transaction
	assert_cs(false);

	return output;
}

/*
 * Spool a large block of memory in chunks from the FT81xRenderer
 *
 * Currently the max chunk size on ESP32 with DMA of 0 is 32 bytes.
 */
void FT81xRenderer::rdN(uint32_t addr, uint8_t *results, int8_t len) {
	while (len) {
		if (len < CHUNKSIZE) {
			rdn(addr, results, len);
			// all done
			break;
		} else {
			rdn(addr, results, CHUNKSIZE);
			len -= CHUNKSIZE;
			results += CHUNKSIZE;
			addr += CHUNKSIZE;
			if (len < 0)
				len = 0;
		}
	}
}

void FT81xRenderer::rdn(uint32_t addr, uint8_t *results, int8_t len) {
	// start the transaction ISR watches CS bit
	assert_cs(true);

	QSPI_CommandTypeDef cmd;
	load_default_command(&cmd);

	if (qio) {
		cmd.DataMode = QSPI_DATA_4_LINES;
		cmd.AddressMode = QSPI_ADDRESS_4_LINES;

	} else {
		cmd.DataMode = QSPI_DATA_1_LINE;
		cmd.AddressMode = QSPI_ADDRESS_1_LINE;
	}

	cmd.NbData = len;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;
	cmd.Address = addr;
	cmd.DummyCycles = 1;

	if (spi->run(&cmd, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to send QSPI command for readn\r\n");
		return;
	}

	if (spi->receive(results, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to receive QSPI data for readn\r\n");
		return;
	}

	// end the transaction
	assert_cs(false);
}

/*
 * Write 24 bit address + 8 bit value
 * A total of 4 bytes will be on the SPI BUS.
 */
void FT81xRenderer::wr(uint32_t addr, uint8_t byte) {
	// start the transaction ISR watches CS bit
	assert_cs(true);

	QSPI_CommandTypeDef cmd;
	load_default_command(&cmd);

	if (qio) {
		cmd.DataMode = QSPI_DATA_4_LINES;
		cmd.AddressMode = QSPI_ADDRESS_4_LINES;

	} else {
		cmd.DataMode = QSPI_DATA_1_LINE;
		cmd.AddressMode = QSPI_ADDRESS_1_LINE;
	}

	cmd.NbData = 1;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;
	cmd.Address = addr | 0x800000;

	if (spi->run(&cmd, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to send QSPI command for write8\r\n");
		return;
	}

	if (spi->transmit((uint8_t*) &byte, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to transmit QSPI data for write8\r\n");
		return;
	}

	// end the transaction
	assert_cs(false);
}

/*
 * Write 24 bit address + 16 bit value
 * A total of 5 bytes will be on the SPI BUS.
 */
void FT81xRenderer::wr16(uint32_t addr, uint16_t word) {
	// start the transaction ISR watches CS bit
	assert_cs(true);

	QSPI_CommandTypeDef cmd;
	load_default_command(&cmd);

	if (qio) {
		cmd.DataMode = QSPI_DATA_4_LINES;
		cmd.AddressMode = QSPI_ADDRESS_4_LINES;

	} else {
		cmd.DataMode = QSPI_DATA_1_LINE;
		cmd.AddressMode = QSPI_ADDRESS_1_LINE;
	}

	cmd.NbData = 2;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;
	cmd.Address = addr | 0x800000;

	if (spi->run(&cmd, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to send QSPI command for write16\r\n");
		return;
	}

	if (spi->transmit((uint8_t*) &word, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to transmit QSPI data for write16\r\n");
		return;
	}

	// end the transaction
	assert_cs(false);
}

/*
 * Write 24 bit address + 32 bit value
 * A total of 7 bytes will be on the SPI BUS.
 */
void FT81xRenderer::wr32(uint32_t addr, uint32_t word) {
	// start the transaction ISR watches CS bit
	assert_cs(true);

	QSPI_CommandTypeDef cmd;
	load_default_command(&cmd);

	if (qio) {
		cmd.DataMode = QSPI_DATA_4_LINES;
		cmd.AddressMode = QSPI_ADDRESS_4_LINES;

	} else {
		cmd.DataMode = QSPI_DATA_1_LINE;
		cmd.AddressMode = QSPI_ADDRESS_1_LINE;
	}

	cmd.NbData = 4;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;
	cmd.Address = addr | 0x800000;

	if (spi->run(&cmd, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to send QSPI command for write32\r\n");
		return;
	}

	if (spi->transmit((uint8_t*) &word, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to transmit QSPI data for write32\r\n");
		return;
	}

	// end the transaction
	assert_cs(false);
}

/*
 * Write 24 bit address leave CS open for data to be written
 * A total of 3 bytes will be on the SPI BUS.
 */
void FT81xRenderer::wrA(uint32_t addr) {

	// set write bit if rw=1
	addr |= 0x800000;
	addr = SPI_REARRANGE_DATA(addr, 24);

	// start the transaction ISR watches CS bit
	assert_cs(true);

	QSPI_CommandTypeDef cmd;
	load_default_command(&cmd);

	if (qio) {
		cmd.DataMode = QSPI_DATA_4_LINES;

	} else {
		cmd.DataMode = QSPI_DATA_1_LINE;
	}

	cmd.NbData = 3;

	if (spi->run(&cmd, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to send QSPI command for writea\r\n");
		return;
	}

	if (spi->transmit((uint8_t*) &addr, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to transmit QSPI data for writea\r\n");
		return;
	}

}

/*
 * Write bytes to the spi port no tracking.
 */
void FT81xRenderer::wrN(uint8_t *buffer, uint8_t size) {
	QSPI_CommandTypeDef cmd;
	load_default_command(&cmd);

	if (qio) {
		cmd.DataMode = QSPI_DATA_4_LINES;
	} else {
		cmd.DataMode = QSPI_DATA_1_LINE;
	}

	cmd.NbData = size;

	if (spi->run(&cmd, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to send QSPI command for writen\r\n");
		return;
	}

	if (spi->transmit(buffer, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to transmit QSPI data for writen\r\n");
		return;
	}
}

void FT81xRenderer::wrE() {
	// end the transaction
	assert_cs(false);
}

/*
 * Spool a large block of memory in chunks into the FT81xRenderer
 * using the MEDIA FIFO registers. Currently the max chunk size
 * on ESP32 with DMA of 0 is 32 bytes.
 *
 * If the size is > the available space this routine will block.
 * To avoid FT81xRenderer:: blocking check free space before calling this routine.
 */
void FT81xRenderer::cSPOOL_MF(uint8_t *buffer, int32_t size) {
	uint32_t fullness;
	int32_t rds, ts;
	uint8_t stopped = 1; // Stopped at startup
	int32_t written = 0;

	// Get the read pointer where the GPU is working currently
	uint32_t mf_rp = rd32(REG_MEDIAFIFO_READ);

	// Calculate how full our fifo is based upon our read/write pointers
	fullness = (mf_wp - mf_rp) & (mf_size - 1);
#ifdef configFT_DEBUG
	console.printf("[FT81xRenderer] rp1 0x%08x wp 0x%08x full 0x%08x\r\n", mf_rp, mf_wp, fullness);
#endif
	// Blocking! Keep going until all the data is sent.
	do {

		// Wait till we have enough room to send some data
		if (!(fullness < (mf_size - CHUNKSIZE))) {

			// Release the SPI bus
			stream_stop();
			stopped = 1;

			// Did we write anything? If so tell the FT813
			if (written) {
				wr32(REG_MEDIAFIFO_WRITE, mf_wp);
				written = 0;
			}

			// sleep a little let other processes go.
			vTaskDelay(1 / portTICK_PERIOD_MS);

			// Get the read pointer where the GPU is working currently
			// consuming bytes.
			mf_rp = rd32(REG_MEDIAFIFO_READ);

			// Calculate how full our fifo is based upon our read/write pointers
			fullness = (mf_wp - mf_rp) & (mf_size - 1);

			continue;
		}

		// resume streaming data if needed
		if (stopped) {
			// Start streaming to our fifo starting with our address
			// same as stream_start() but different address area
			// and no auto wrapping :(
			wrA(mf_base + mf_wp);
			stopped = 0;
		}

		// write up to the very end of the fifo buffer
		rds = (mf_size - mf_wp);
		if (rds > CHUNKSIZE) {
			rds = CHUNKSIZE;
		}

		// default write size to chunk size or enough for the end of the fifo
		ts = rds;

		// if we have less to send than we can then update transmit size
		if (size < ts) {
			ts = size;
		}

		// write the block to the FT81xRenderer
		wrN((uint8_t*) buffer, ts);

		// increment the pointers/counters
		buffer += ts;
		mf_wp += ts;
		fullness += ts;
		size -= ts;
		written += ts;

#ifdef configFT_DEBUG
		console.printf("[FT81xRenderer] rp2 0x%08x wp 0x%08x full 0x%08x\r\n", mf_rp, mf_wp, fullness);
    // sleep a little let other processes go.
    vTaskDelay(10 / portTICK_PERIOD_MS);
#endif

		// loop around if we overflow the fifo.
		mf_wp &= (mf_size - 1);

		// force flush if we reached the end of the fifo buffer
		if (!mf_wp)
			fullness = -mf_size;

	} while (size);

	// Release the SPI bus
	stream_stop();
	stopped = 1;

	// Did we write anything? If so tell the FT813
	if (written) {
		wr32(REG_MEDIAFIFO_WRITE, mf_wp);
		written = 0;
	}
}

/*
 * Read the FT81xRenderer command pointer
 */
uint16_t FT81xRenderer::fifo_rp() {
	uint16_t rp = rd16(REG_CMD_READ);
	if (rp == DL_CMD_FAULT) {
		console->printf("[FT81xRenderer] COPROCESSOR EXCEPTION\r\n");
		//vTaskDelay(50 / portTICK_PERIOD_MS);
		// Resetting co-processor sets REG_CMD_READ to zero.
		wr(REG_CPURESET, 1);
		wr16(REG_CMD_READ, 0);
		wr16(REG_CMD_WRITE, 0);
		wr(REG_CPURESET, 0);
		rp = 0;
	}
	return rp;
}

/*
 * Write out padded bits to be sure we are 32 bit aligned
 * as required by the FT81xRenderer
 */
void FT81xRenderer::align32(uint32_t written) {
	uint8_t dummy[4] = { 0x00, 0x00, 0x00, 0x00 };
	int8_t align = 4 - (written & 0x3);
	if (align & 0x3)
		cN((uint8_t*) dummy, align);
}

/*
 * Start a new transactions to write to the command buffer at the current write pointer.
 * Assert CS pin.
 */
void FT81xRenderer::stream_start() {
	// be sure we ended the last tranaction
	stream_stop();
	// begin a new write transaction.
	wrA(RAM_CMD + (fifo_wp & 0xffc));
}

/*
 * Close any open transactions.
 * De-assert CS pin
 */
void FT81xRenderer::stream_stop() {
	// end the transaction
	assert_cs(false);
}

/*
 * Get free space from FT81xRenderer until it reports
 * we have required space. Also triggers processing
 * of any display commands in the fifo buffer.
 */
void FT81xRenderer::getfree(uint16_t required) {
	fifo_wp &= 0xffc;
	stream_stop();

	// force command to complete write to CMD_WRITE
	wr16(REG_CMD_WRITE, fifo_wp & 0xffc);

	do {
		vTaskDelay(10 / portTICK_PERIOD_MS);
		uint16_t rp = fifo_rp();
		uint16_t howfull = (fifo_wp - rp) & 4095;
		fifo_freespace = MAX_FIFO_SPACE - howfull;
#ifdef configFT_DEBUG
	console.printf("[FT81xRenderer] fifoA: wp:0x%04x rp:0x%04x fs:0x%04x\r\n", fifo_wp, rp, fifo_freespace);
    vTaskDelay(25 / portTICK_PERIOD_MS);
#endif
	} while (fifo_freespace < required);
	stream_start();
}

void FT81xRenderer::checkfree(uint16_t required) {
	// check that we have space in our fifo buffer
	// block until the FT81xRenderer says we do.
	if (fifo_freespace < required) {
		getfree(required);
#ifdef configFT_DEBUG
	console.printf("[FT81xRenderer] free: 0x%04x\r\n", fifo_freespace);
#endif
	}
}

/*
 * wrapper to send out a 32bit command into the fifo buffer
 * while in stream() mode.
 */
void FT81xRenderer::cI(uint32_t word) {
	//word = SPI_REARRANGE_DATA(word, 32);
	cmd32(word);
}

/*
 * wrapper to send a 8bit command into the fifo buffer
 * while in stream() mode.
 */
void FT81xRenderer::cFFFFFF(uint8_t byte) {
	cmd32(byte | 0xffffff00);
}

/*
 * Write 32 bit command into our command fifo buffer
 * while in stream() mode.
 * Use tx_buffer to transmit the 32 bits.
 */
void FT81xRenderer::cmd32(uint32_t word) {

	fifo_wp += sizeof(word);
	fifo_freespace -= sizeof(word);

	QSPI_CommandTypeDef cmd;
	load_default_command(&cmd);

	if (qio) {
		cmd.DataMode = QSPI_DATA_4_LINES;

	} else {
		cmd.DataMode = QSPI_DATA_1_LINE;
	}

	cmd.NbData = 4;

	if (spi->run(&cmd, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to send QSPI command for cmd32\r\n");
		return;
	}

	if (spi->transmit((uint8_t*) &word, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to transmit QSPI data for cmd32\r\n");
		return;
	}

}

/*
 * Write N bytes command into our command fifo buffer
 * while in stream() mode.
 * Use tx_buffer to transmit the bits. Must be less
 * than buffer size.
 */
void FT81xRenderer::cN(uint8_t *buffer, uint16_t size) {

	fifo_wp += size;
	fifo_freespace -= size;

	QSPI_CommandTypeDef cmd;
	load_default_command(&cmd);

	if (qio) {
		cmd.DataMode = QSPI_DATA_4_LINES;

	} else {
		cmd.DataMode = QSPI_DATA_1_LINE;
	}

	cmd.NbData = size;

	if (spi->run(&cmd, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to send QSPI command for cmdn\r\n");
		return;
	}

	if (spi->transmit(buffer, FT_IO_TIMEOUT) != HAL_OK) {
		console->printf("[FT81xRenderer] Failed to transmit QSPI data for cmdn\r\n");
		return;
	}
}

/*
 * Spool a large block of memory in chunks into the FT81xRenderer
 *
 * Currently the max chunk size on ESP32 with DMA of 0 is 32 bytes.
 */
void FT81xRenderer::cSPOOL(uint8_t *buffer, int32_t size) {
	int32_t savesize = size;

	while (size) {
		if (size < CHUNKSIZE) {
			// check that we have enough space then send command
			checkfree(size);
			cN((uint8_t*) buffer, size);
			// all done
			break;
		} else {
			checkfree(CHUNKSIZE);
			cN((uint8_t*) buffer, CHUNKSIZE);
			size -= CHUNKSIZE;
			buffer += CHUNKSIZE;
			if (size < 0)
				size = 0;
		}
	}

	int8_t align = (4 - (savesize & 0x3)) & 0x3;
	checkfree(align);
	align32(savesize);
}

/*
 * Wait until REG_CMD_READ == REG_CMD_WRITE indicating
 * the GPU has processed all of the commands in the
 * circular command buffer
 */
void FT81xRenderer::wait_finish() {
#ifdef configFT_DEBUG
  uint32_t twp = fifo_wp;
#endif
	uint16_t rp;
	fifo_wp &= 0xffc;
#ifdef configFT_DEBUG
	console.printf("[FT81xRenderer] waitfin: twp:0x%04x wp:0x%04x rp:0x%04x\r\n", twp, fifo_wp, fifo_rp());
  #endif
	while (((rp = fifo_rp()) != fifo_wp)) {
	}
	fifo_freespace = MAX_FIFO_SPACE;
}

void FT81xRenderer::multi_touch_enable(bool enable) {
	wr(REG_CTOUCH_EXTENDED, !enable); // Turn on=0/(off=1) assume little-endian
}

bool FT81xRenderer::multi_touch_enabled() {
	return !(rd(REG_CTOUCH_EXTENDED) & 0x01);
}

uint8_t FT81xRenderer::touch_mode() {
	return rd(REG_CTOUCH_MODE) & 0x03;
}

/*
 * Read the FT813 tracker and ctouch registers
 * and update our global touch state structure
 */
struct touch_input_t* FT81xRenderer::fetch_touch_inputs() {
	// read in the tracker memory to our local structure
	rdN(REG_TRACKER, (uint8_t*) &touch_tracker,
			sizeof(touch_tracker));

	// Read in the ctouch registers depending on multitouch mode some or all.
	if (multi_touch_enabled()) {
		// Read in our all ctouch registers. Most are continuous but the last 3 are not :(
		// If we are in single touch mode then we only need a few registers.
		struct {
			uint32_t touch1_xy;
			uint32_t touch4_y;
			uint32_t touch0_xy;
			uint32_t tag0_xy;
			uint32_t tag0;
			uint32_t tag1_xy;
			uint32_t tag1;
			uint32_t tag2_xy;
			uint32_t tag2;
			uint32_t tag3_xy;
			uint32_t tag3;
			uint32_t tag4_xy;
			uint32_t tag4;
		} ctouch;
		rdN(REG_CTOUCH_TOUCH1_XY, (uint8_t*) &ctouch,
				sizeof(ctouch));
		touch_input[0].tag = ctouch.tag0;
		touch_input[0].tag_x = ctouch.tag0_xy >> 16;
		touch_input[0].tag_y = ctouch.tag0_xy & 0xffff;
		touch_input[0].display_x =
				(int16_t) (ctouch.touch0_xy >> 16);
		touch_input[0].display_y = (int16_t) (ctouch.touch0_xy
				& 0xffff);

		touch_input[1].tag = ctouch.tag1;
		touch_input[1].tag_x = ctouch.tag1_xy >> 16;
		touch_input[1].tag_y = ctouch.tag1_xy & 0xffff;
		touch_input[1].display_x =
				(int16_t) (ctouch.touch1_xy >> 16);
		touch_input[1].display_y = (int16_t) (ctouch.touch1_xy
				& 0xffff);

		touch_input[2].tag = ctouch.tag2;
		touch_input[2].tag_x = ctouch.tag2_xy >> 16;
		touch_input[2].tag_y = ctouch.tag2_xy & 0xffff;
		uint32_t touch_xy = rd32(REG_CTOUCH_TOUCH2_XY);
		touch_input[2].display_x = (int16_t) (touch_xy >> 16);
		touch_input[2].display_y = (int16_t) (touch_xy & 0xffff);

		touch_input[3].tag = ctouch.tag3;
		touch_input[3].tag_x = ctouch.tag3_xy >> 16;
		touch_input[3].tag_y = ctouch.tag3_xy & 0xffff;
		touch_xy = rd32(REG_CTOUCH_TOUCH3_XY);
		touch_input[3].display_x = (int16_t) (touch_xy >> 16);
		touch_input[3].display_y = (int16_t) (touch_xy & 0xffff);

		touch_input[4].tag = ctouch.tag4;
		touch_input[4].tag_x = ctouch.tag4_xy >> 16;
		touch_input[4].tag_y = ctouch.tag4_xy & 0xffff;
		touch_input[4].display_x = (int16_t) rd16(
		REG_CTOUCH_TOUCH4_X);
		touch_input[4].display_y = (int16_t) (ctouch.touch4_y
				& 0xffff);
	} else {
		struct {
			uint32_t touch0_xy;
			uint32_t tag0_xy;
			uint32_t tag0;
		} ctouch;
		// Only bring in TOUCH0_XY, TAG0_XY and TAG0 for single touch mode
		rdN(REG_CTOUCH_TOUCH0_XY, (uint8_t*) &ctouch,
				sizeof(ctouch));
		touch_input[0].tag = ctouch.tag0;
		touch_input[0].tag_x = ctouch.tag0_xy >> 16;
		touch_input[0].tag_y = ctouch.tag0_xy & 0xffff;
		touch_input[0].display_x =
				(int16_t) (ctouch.touch0_xy >> 16);
		touch_input[0].display_y = (int16_t) (ctouch.touch0_xy
				& 0xffff);
	}

	#ifdef configFT_DEBUG
	console.printf("[FT81xRenderer] ttag0: %i val: 0x%08x\r\n", touch_tracker[0].tag, touch_tracker[0].value);
	console.printf("[FT81xRenderer] ctag0: %i xy: 0x%04x,0x%04x\r\n", touch_input[0].tag, touch_input[0].tag_x, touch_input[0].tag_y);
	#endif

	return touch_input;
}

/*
 * Swap the display
 */
void FT81xRenderer::swap() {
	display(); // end current display list
	cmd_swap(); // Set AUTO swap at end of display list
	//cmd_loadidentity();
	cmd_dlstart(); // Set REG_CMD_DL when done
	getfree(0); // trigger display processing
}

/*
 * Programming Guide sections
 *
 */

/*
 * 4.4 ALPHA_FUNCT
 * Specify the alpha test function
 * SM20180828:QA:PASS
 */
void FT81xRenderer::alpha_funct(uint8_t func, uint8_t ref) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x09UL << 24) | // CMD 0x09      24 - 31
							   // RESERVED      11 - 23
			((func & 0x7L) << 8) | // func           8 - 10
			((ref & 0xffL) << 0)   // ref            0 -  7
			);
}

/*
 * 4.5 BEGIN
 * Begin drawing a graphics primitive
 * SM20180828:QA:PASS
 */
void FT81xRenderer::begin(uint8_t prim) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x1fUL << 24) | (prim & 0x0f));
}

/*
 * 4.6 BITMAP_HANDLE
 * Specify the bitmap handle
 * SM20180828:QA:PASS
 */
void FT81xRenderer::bitmap_handle(uint8_t handle) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x05UL << 24) | (handle & 0x1f));
}

/*
 * 4.7 BITMAP_LAYOUT
 * Specify the source bitmap memory format and layout for the current handle
 * SM20180828:QA:PASS
 */
void FT81xRenderer::bitmap_layout(uint8_t format, uint16_t linestride,
		uint16_t height) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x07UL << 24) | // CMD 0x07      24 - 31
			((format & 0x1fL) << 19) | // format        19 - 23
			((linestride & 0x3ffL) << 9) | // linestride     9 - 18
			((height & 0x1ffL) << 0)   // height         0 -  8
			);
}

/*
 * 4.8 BITMAP_LAYOUT_H
 * Specify the 2 most significant bits of the source bitmap memory format and layout for the current handle
 * SM20180828:QA:PASS
 */
void FT81xRenderer::bitmap_layout_h(uint8_t linestride, uint8_t height) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x28UL << 24) | // CMD 0x28      24 - 31
							   // RESERVED       4 - 23
			((linestride & 0x3L) << 2) | // linestride     2 -  3
			((height & 0x3L) << 0)   // height         0 -  1
			);
}

/*
 * 4.9 BITMAP_SIZE
 * Specify the screen drawing of bitmaps for the current handle
 * SM20180828:QA:PASS
 */
void FT81xRenderer::bitmap_size(uint8_t filter, uint8_t wrapx, uint8_t wrapy,
		uint16_t width, uint16_t height) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x08UL << 24) | // CMD 0x08      24 - 31
							   // RESERVED      21 - 23
			((filter & 0x1L) << 20) | // filter        20 - 20
			((wrapx & 0x1L) << 19) | // wrapx         19 - 19
			((wrapy & 0x1L) << 18) | // wrapy         18 - 18
			((width & 0x1ffL) << 9) | // width          9 - 17
			((height & 0x1ffL) << 0)   // height         0 -  8
			);
}

/*
 * 4.10 BITMAP_SIZE_H
 * Specify the source address of bitmap data in FT81xRenderer graphics memory RAM_G
 * SM20180828:QA:PASS
 */
void FT81xRenderer::bitmap_size_h(uint8_t width, uint8_t height) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x29UL << 24) | (((width) & 0x3) << 2) | (((height) & 0x3) << 0));
}

/*
 * 4.11 BITMAP_SOURCE
 * Specify the source address of bitmap data in FT81xRenderer graphics memory RAM_G
 * SM20180828:QA:PASS
 */
void FT81xRenderer::bitmap_source(uint32_t addr) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x01UL << 24) | ((addr & 0x3fffffL) << 0));
}

/*
 * 4.12 BITMAP_TRANSFORM_A
 * Specify the A coefficient of the bitmap transform matrix
 * SM20180828:QA:PASS
 */
void FT81xRenderer::bitmap_transform_a(uint32_t a) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x15UL << 24) | ((a & 0xffffL) << 0));
}

/*
 * 4.13 BITMAP_TRANSFORM_B
 * Specify the B coefficient of the bitmap transform matrix
 * SM20180828:QA:PASS
 */
void FT81xRenderer::bitmap_transform_b(uint32_t b) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x16UL << 24) | ((b & 0xffffL) << 0));
}

/*
 * 4.14 BITMAP_TRANSFORM_C
 * Specify the C coefficient of the bitmap transform matrix
 * SM20180828:QA:PASS
 */
void FT81xRenderer::bitmap_transform_c(uint32_t c) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x17UL << 24) | ((c & 0xffffffL) << 0));
}

/*
 * 4.15 BITMAP_TRANSFORM_D
 * Specify the D coefficient of the bitmap transform matrix
 * SM20180828:QA:PASS
 */
void FT81xRenderer::bitmap_transform_d(uint32_t d) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x18UL << 24) | ((d & 0xffffL) << 0));
}

/*
 * 4.16 BITMAP_TRANSFORM_E
 * Specify the E coefficient of the bitmap transform matrix
 * SM20180828:QA:PASS
 */
void FT81xRenderer::bitmap_transform_e(uint32_t e) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x19UL << 24) | ((e & 0xffffL) << 0));
}

/*
 * 4.17 BITMAP_TRANSFORM_F
 * Specify the F coefficient of the bitmap transform matrix
 * SM20180828:QA:PASS
 */
void FT81xRenderer::bitmap_transform_f(uint32_t f) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x1aUL << 24) | ((f & 0xffffffL) << 0));
}

/*
 * 4.18 BLEND_FUNC
 * Specify pixel arithmetic
 * SM20180828:QA:PASS
 */
void FT81xRenderer::blend_func(uint8_t src, uint8_t dst) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x0bUL << 24) | ((src & 0x7L) << 3) | ((dst & 0x7L) << 0));
}

/*
 * 4.19 CALL
 * Execute a sequence of commands at another location in the display list
 * SM20180828:QA:PASS
 */
void FT81xRenderer::call(uint16_t dest) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x1dUL << 24) | ((dest & 0xffffL) << 0));
}

/*
 * 4.20 CELL
 * Specify the bitmap cell number for the VERTEX2F command
 * SM20180828:QA:PASS
 */
void FT81xRenderer::cell(uint8_t cell) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x06UL << 24) | ((cell & 0x7fL) << 0));
}

/*
 * 4.21 CLEAR
 * Clear buffers to preset values
 * SM20180828:QA:PASS
 */
void FT81xRenderer::clear() {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x26UL << 24) | 0x7);
}

void FT81xRenderer::clearCST(uint8_t color, uint8_t stencil, uint8_t tag) {
	uint8_t cst = 0;
	cst = color & 0x01;
	cst <<= 1;
	cst |= (stencil & 0x01);
	cst <<= 1;
	cst |= (tag & 0x01);

	// check that we have enough space then send command
	checkfree(4);
	cI((0x26UL << 24) | cst);
}

/*
 * 4.21 CLEAR_COLOR_A
 * Specify clear value for the alpha channel
 * SM20180828:QA:PASS
 */
void FT81xRenderer::clear_color_a(uint8_t alpha) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x0fUL << 24) | (alpha & 0xffL));
}

/*
 * 4.23 CLEAR_COLOR_RGB
 * Specify clear values for red, green and blue channels
 * SM20180828:QA:PASS
 */
void FT81xRenderer::clear_color_rgb32(uint32_t rgb) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x2UL << 24) | (rgb & 0xffffffL));
}

void FT81xRenderer::clear_color_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
	clear_color_rgb32(
			((red & 0xffL) << 16) | ((green & 0xffL) << 8)
					| ((blue & 0xffL) << 0));
}

/*
 * 4.24 CLEAR_STENCIL
 * Specify clear value for the stencil buffer
 * SM20180828:QA:PASS
 */
void FT81xRenderer::clear_stencil(uint8_t stencil) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x11UL << 24) | ((stencil & 0xffL) << 0));
}

/*
 * 4.25 CLEAR_TAG
 * Specify clear value for the tag buffer
 * SM20180828:QA:PASS
 */
void FT81xRenderer::clear_tag(uint8_t tag) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x12UL << 24) | ((tag & 0xffL) << 0));
}

/*
 * 4.26 COLOR_A
 * Set the current color alpha
 * SM20180828:QA:PASS
 */
void FT81xRenderer::color_a(uint8_t alpha) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x10UL << 24) | ((alpha & 0xffL) << 0));
}

/*
 * 4.27 COLOR_MASK
 * Enable or disable writing of color components
 * SM20180828:QA:PASS
 */
void FT81xRenderer::color_mask(uint8_t red, uint8_t green, uint8_t blue,
		uint8_t alpha) {
	// check that we have enough space then send command
	checkfree(4);
	cI(
			(0x20L << 24)
					| (((red & 0x1) << 3) | ((green & 0x1) << 2)
							| ((blue & 0x1) << 1) | ((alpha & 0x1) << 0)));
}

/*
 * 4.28 COLOR_RGB
 * Set the current color red, green, blue
 * SM20180828:QA:PASS
 */
void FT81xRenderer::color_rgb32(uint32_t rgb) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x4UL << 24) | (rgb & 0xffffffL));
}

void FT81xRenderer::color_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
	color_rgb32(
			((red & 0xffL) << 16) | ((green & 0xffL) << 8)
					| ((blue & 0xffL) << 0));
}

/*
 * 4.29 DISPLAY
 * End the display list. FT81xRenderer will ignore all commands following this command.
 * SM20180828:QA:PASS
 */
void FT81xRenderer::display() {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x0UL << 24));
}

/*
 * 4.30 END
 * End drawing a graphics primitive
 * SM20180828:QA:PASS
 */
void FT81xRenderer::end() {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x21UL << 24));
}

/*
 * 4.31 JUMP
 * Execute commands at another location in the display list
 * SM20180828:QA:PASS
 */
void FT81xRenderer::jump(uint16_t dest) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x1eUL << 24) | (dest & 0xffffL));
}

/*
 * 4.32 LINE_WIDTH
 * Specify the width of lines to be drawn with primitive LINES in 1/16 pixel precision
 * SM20180828:QA:PASS
 */
void FT81xRenderer::line_width(uint16_t width) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x0eUL << 24) | (width & 0xfff));
}

/*
 * 4.33 MACRO
 * Execute a single command from a macro register
 * SM20180828:QA:PASS
 */
void FT81xRenderer::macro(uint8_t macro) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x25UL << 24) | (macro & 0x1L));
}

/*
 * 4.34 NOP
 * No Operation
 * SM20180828:QA:PASS
 */
void FT81xRenderer::nop() {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x2dUL << 24));
}

/*
 * 4.35 PALETTE_SOURCE
 * Specify the base address of the palette
 * SM20180828:QA:PASS
 */
void FT81xRenderer::palette_source(uint32_t addr) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x2aUL << 24) | ((addr) & 0x3fffffUL));
}

/*
 * 4.36 POINT_SIZE
 * Specify the radius of points
 * SM20180828:QA:PASS
 */
void FT81xRenderer::point_size(uint16_t size) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x0dUL << 24) | ((size & 0x1fffL) << 0));
}

/*
 * 4.37 RESTORE_CONTEXT
 * Restore the current graphics context from the context stack
 * SM20180828:QA:PASS
 */
void FT81xRenderer::restore_context() {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x23UL << 24));
}

/*
 * 4.38 RETURN
 * Return from a previous CALL command
 * SM20180828:QA:PASS
 */
void FT81xRenderer::return_call() {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x24UL << 24));
}

/*
 * 4.39 SAVE_CONTEXT
 * Push the current graphics context on the context stack
 * SM20180828:QA:PASS
 */
void FT81xRenderer::save_context() {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x22UL << 24));
}

/*
 * 4.40 SCISSOR_SIZE
 * Specify the size of the scissor clip rectangle
 * SM20180828:QA:PASS
 */
void FT81xRenderer::scissor_size(uint16_t width, uint16_t height) {
	// check that we have enough space then send command
	checkfree(4);
	cI(
			(0x1cUL << 24) | ((width & 0xfffL) << 12)
					| ((height & 0xfffL) << 0));
}

/*
 * 4.41 SCISSOR_XY
 * Specify the top left corner of the scissor clip rectangle
 * SM20180828:QA:PASS
 */
void FT81xRenderer::scissor_xy(uint16_t x, uint16_t y) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x1bUL << 24) | ((x & 0x7ffL) << 11) | ((y & 0x7ffL) << 0));
}

/*
 * 4.42 STENCIL_FUNC
 * Set function and reference value for stencil testing
 * SM20180828:QA:PASS
 */
void FT81xRenderer::stencil_func(uint8_t func, uint8_t ref, uint8_t mask) {
	// check that we have enough space then send command
	checkfree(4);
	cI(
			(0x0aUL << 24) | ((func & 0xfL) << 16) | ((ref & 0xffL) << 8)
					| ((mask & 0xffL) << 0));
}

/*
 * 4.43 STENCIL_MASK
 * Control the writing of individual bits in the stencil planes
 * SM20180828:QA:PASS
 */
void FT81xRenderer::stencil_mask(uint8_t mask) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x013UL << 24) | ((mask & 0xffL) << 0));
}

/*
 * 4.44 STENCIL_OP
 * Set stencil test actions
 * SM20180828:QA:PASS
 */
void FT81xRenderer::stencil_op(uint8_t sfail, uint8_t spass) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x0cUL << 24) | ((sfail & 0x7L) << 3) | ((spass & 0x7L) << 0));
}

/*
 * 4.45 TAG
 * Attach the tag value for the following graphics objects
 * drawn on the screen. The initial tag buffer value is 255.
 * SM20180828:QA:PASS
 */
void FT81xRenderer::tag(uint8_t s) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x3UL << 24) | ((s & 0xffL) << 0));
}

/*
 * 4.46 TAG_MASK
 * Control the writing of the tag buffer
 * SM20180828:QA:PASS
 */
void FT81xRenderer::tag_mask(uint8_t mask) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x14UL << 24) | ((mask & 1L) << 0));
}

/*
 * 4.47 VERTEX2F
 * Start the operation of graphics primitives at the specified
 * screen coordinate, in the pixel precision defined by VERTEX_FORMAT
 * SM20180828:QA:PASS
 */
void FT81xRenderer::vertex2f(int16_t x, int16_t y) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x1UL << 30) | ((x & 0x7fffL) << 15) | ((y & 0x7fffL) << 0));
}

/*
 * 4.48 VERTEX2ii
 * Start the operation of graphics primitives at the specified
 * screen coordinate, in the pixel precision defined by VERTEX_FORMAT
 * SM20180828:QA:PASS
 */
void FT81xRenderer::vertex2ii(int16_t x, int16_t y, uint8_t handle,
		uint8_t cell) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x02UL << 30) | // CMD 0x02      30 - 31
			((x & 0x1ffL) << 21) | // x             21 - 29
			((y & 0x1ffL) << 12) | // y             12 - 20
			((handle & 0x1fL) << 7) | // handle         7 - 11
			((cell & 0x7fL) << 0)   // cell           0 -  6
			);
}

/*
 * 4.49 VERTEX_FORMAT
 * Set the precision of VERTEX2F coordinates
 * SM20180828:QA:PASS
 */
void FT81xRenderer::vertex_format(int8_t frac) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x27UL << 24) | (((frac) & 0x7) << 0));
}

/*
 * 4.50 VERTEX_TRANSLATE_X
 * Specify the vertex transformation’s X translation component
 * SM20180828:QA:PASS
 */
void FT81xRenderer::vertex_translate_x(uint32_t x) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x2bUL << 24) | (((x) & 0x1ffffUL) << 0));
}

/*
 * 4.51 VERTEX_TRANSLATE_Y
 * Specify the vertex transformation’s Y translation component
 * SM20180828:QA:PASS
 */
void FT81xRenderer::vertex_translate_y(uint32_t y) {
	// check that we have enough space then send command
	checkfree(4);
	cI((0x2cUL << 24) | (((y) & 0x1ffffUL) << 0));
}

/*
 * 5.11 CMD_DLSTART
 * Start a new display list
 * SM20180828:QA:PASS
 */
void FT81xRenderer::cmd_dlstart() {
	// check that we have enough space then send command
	checkfree(4);
	cFFFFFF(0x00);
}

/*
 * 5.12 CMD_SWAP
 * Swap the current display list
 * SM20180828:QA:PASS
 */
void FT81xRenderer::cmd_swap() {
	// check that we have enough space then send command
	checkfree(4);
	cFFFFFF(0x01);
}

/*
 * 5.13 CMD_COLDSTART
 * This command sets the co-processor engine to default reset states
 * SM20180828:QA:PASS
 */
void FT81xRenderer::cmd_coldstart() {
	// check that we have enough space then send command
	checkfree(4);
	cFFFFFF(0x32);
}

/*
 * 5.14 CMD_INTERRUPT
 * trigger interrupt INT_CMDFLAG
 * SM20180828:QA:PASS
 */
void FT81xRenderer::cmd_interrupt(uint32_t ms) {
	// check that we have enough space then send command
	checkfree(8);
	cFFFFFF(0x02);
	cI(ms);
}

/*
 * 5.15 CMD_APPEND
 * Append more commands to current display list
 * SM20180828:QA:PASS
 */
void FT81xRenderer::cmd_append(uint32_t ptr, uint32_t num) {
	// check that we have enough space then send command
	checkfree(12);
	cFFFFFF(0x1e);
	cI(ptr);
	cI(num);
}

/*
 * 5.16 CMD_REGREAD
 * Read a register value
 * FIXME
 */
void FT81xRenderer::cmd_regread(uint32_t ptr, uint32_t *result) {
	// check that we have enough space then send command
	checkfree(8);
	cFFFFFF(0x19);
	cI(ptr);

	// The data will be written starting here in the buffer so get the pointer
	uint32_t r = getwp();

	// Fill in memory where results will go with dummy data
	cI(0xffffffff); // Will be result

	// report back memory locations of the results to caller
	*result = r;
}

/*
 * 5.17 CMD_MEMWRITE
 * Write bytes into memory
 */
void FT81xRenderer::cmd_memwrite(uint32_t ptr, uint32_t num) {
	// check that we have enough space then send command
	checkfree(12);
	cFFFFFF(0x1a);
	cI(ptr);
	cI(num);
}

/*
 * 5.18 CMD_INFLATE
 * Decompress data into memory
 */
void FT81xRenderer::cmd_inflate(uint32_t ptr) {
	// check that we have enough space then send command
	checkfree(8);
	cFFFFFF(0x22);
	cI(ptr);
}

/*
 * 5.19 CMD_LOADIMAGE
 * Load a JPEG or PNG image
 * SM20180828:QA:PASS
 */
void FT81xRenderer::cmd_loadimage(uint32_t ptr, uint32_t options) {
	// check that we have enough space then send command
	checkfree(12);
	cFFFFFF(0x24);
	cI(ptr);
	cI(options);
}

/*
 * 5.20 CMD_MEDIAFIFO
 * set up a streaming media FIFO in RAM_G
 * SM20180828:QA:PASS
 */
void FT81xRenderer::cmd_mediafifo(uint32_t base, uint32_t size) {
	mf_wp = 0;
	mf_size = size;
	mf_base = base;

	// check that we have enough space then send command
	checkfree(12);
	cFFFFFF(0x39);
	cI(base);
	cI(size);
}

/*
 * 5.21 CMD_PLAYVIDEO
 * Video playback
 */
void FT81xRenderer::cmd_playvideo(uint32_t options) {
	// check that we have enough space then send command
	checkfree(8);
	cFFFFFF(0x3a);
	cI(options);
}

/*
 * 5.22 CMD_VIDEOSTART
 * Initialize the AVI video decoder
 */
void FT81xRenderer::cmd_videostart() {
	// check that we have enough space then send command
	checkfree(4);
	cFFFFFF(0x40);
}

/*
 * 5.23 CMD_VIDEOFRAME
 * Loads the next frame of video
 */
void FT81xRenderer::cmd_videoframe(uint32_t dst, uint32_t ptr) {
	// check that we have enough space then send command
	checkfree(12);
	cFFFFFF(0x40);
	cI(dst);
	cI(ptr);
}

/*
 * 5.24 CMD_MEMCRC
 * Compute a CRC-32 for memory
 */
uint32_t FT81xRenderer::cmd_memcrc(uint32_t ptr, uint32_t num) {
	// check that we have enough space then send command
	checkfree(16);

	cFFFFFF(0x18);
	cI(ptr);
	cI(num);

	// The data will be written here in the buffer so get the pointer
	uint32_t r = getwp();

	// Dummy data where our results will go
	cI(0xffffffff);
	return r;
}

/*
 * 5.25 CMD_MEMZERO
 * Write zero to a block of memory
 */
void FT81xRenderer::cmd_memzero(uint32_t ptr, uint32_t num) {
	// check that we have enough space then send command
	checkfree(12);
	cFFFFFF(0x1c);
	cI(ptr);
	cI(num);
}

/*
 * 5.26 CMD_MEMSET
 * Fill memory with a byte value
 */
void FT81xRenderer::cmd_memset(uint32_t ptr, uint32_t value, uint32_t num) {
	// check that we have enough space then send command
	checkfree(12);
	cFFFFFF(0x1b);
	cI(value);
	cI(num);
}

/*
 * 5.27 CMD_MEMCPY
 * Copy a block of memory
 */
void FT81xRenderer::cmd_memcpy(uint32_t dest, uint32_t src, uint32_t num) {
	// check that we have enough space then send command
	checkfree(16);
	cFFFFFF(0x1d);
	cI(dest);
	cI(src);
	cI(num);
}

/*
 * 5.28 CMD_BUTTON
 * Draw a button
 */
void FT81xRenderer::cmd_button(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint16_t font, uint16_t options, const char *s) {
	uint16_t b[6];
	b[0] = x;
	b[1] = y;
	b[2] = w;
	b[3] = h;
	b[4] = font;
	b[5] = options;
	uint32_t len = strlen(s) + 1;
	int8_t align = (4 - (len & 0x3)) & 0x3;

	// check that we have enough space then send command
	checkfree(4 + sizeof(b) + len + align);
	cFFFFFF(0x0d);
	cN((uint8_t*) b, sizeof(b));
	cN((uint8_t*) s, len);
	align32(len);
}

/*
 * 5.29 CMD_CLOCK
 * Draw an analog clock
 */
void FT81xRenderer::cmd_clock(uint16_t x, uint16_t y, uint16_t r,
		uint16_t options, uint16_t h, uint16_t m, uint16_t s, uint16_t ms) {
	uint16_t b[8];
	b[0] = x;
	b[1] = y;
	b[2] = r;
	b[3] = options;
	b[4] = h;
	b[5] = m;
	b[6] = s;
	b[7] = ms;

	// check that we have enough space then send command
	checkfree(4 + sizeof(b));
	cFFFFFF(0x14);
	cN((uint8_t*) b, sizeof(b));
}

/*
 * 5.30 CMD_FGCOLOR
 * set the foreground color
 * SM20180828:QA:PASS
 */
void FT81xRenderer::fgcolor_rgb32(uint32_t rgb) {
	// check that we have enough space then send command
	checkfree(8);
	cFFFFFF(0x0a);
	cI(rgb);
}
void FT81xRenderer::fgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
	fgcolor_rgb32(
			((red & 255L) << 16) | ((green & 255L) << 8)
					| ((blue & 255L) << 0));
}

/*
 * 5.31 CMD_BGCOLOR
 * Set the background color
 */
void FT81xRenderer::bgcolor_rgb32(uint32_t rgb) {
	// check that we have enough space then send command
	checkfree(8);
	cFFFFFF(0x09);
	cI(rgb);
}
void FT81xRenderer::bgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
	bgcolor_rgb32(
			((red & 255L) << 16) | ((green & 255L) << 8)
					| ((blue & 255L) << 0));
}

/*
 * 5.32 CMD_GRADCOLOR
 * Set the 3D button highlight color
 */
void FT81xRenderer::cmd_gradcolor_rgb32(uint32_t rgb) {
	// check that we have enough space then send command
	checkfree(8);
	cFFFFFF(0x34);
	cI(rgb);
}
void FT81xRenderer::cmd_gradcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue) {
	cmd_gradcolor_rgb32(
			((red & 255L) << 16) | ((green & 255L) << 8)
					| ((blue & 255L) << 0));
}

/*
 * 5.33 CMD_GAUGE
 * Draw a gauge
 */
void FT81xRenderer::cmd_gauge(int16_t x, int16_t y, int16_t r, uint16_t options,
		uint16_t major, uint16_t minor, uint16_t val, uint16_t range) {
	uint16_t b[8];
	b[0] = x;
	b[1] = y;
	b[2] = r;
	b[3] = options;
	b[4] = major;
	b[5] = minor;
	b[6] = val;
	b[7] = range;
	// check that we have enough space then send command
	checkfree(4 + sizeof(b));
	cFFFFFF(0x13);
	cN((uint8_t*) b, sizeof(b));
}

/*
 * 5.34 CMD_GRADIENT
 * Draw a smooth color gradient
 */
void FT81xRenderer::cmd_gradient_rgb32(int16_t x0, int16_t y0, uint32_t rgb0,
		int16_t x1, int16_t y1, uint32_t rgb1) {
	uint16_t b[8];
	b[0] = x0;
	b[1] = y0;
	b[2] = rgb0 >> 16;
	b[3] = rgb0 & 0xffff;
	b[4] = x1;
	b[5] = y1;
	b[6] = rgb1 >> 16;
	b[7] = rgb1 & 0xffff;

	// check that we have enough space then send command
	checkfree(4 + sizeof(b));
	cFFFFFF(0x0b);
	cN((uint8_t*) b, sizeof(b));
}

/*
 * 5.35 CMD_KEYS
 * Draw a row of keys
 */
void FT81xRenderer::cmd_keys(int16_t x, int16_t y, int16_t w, int16_t h,
		int16_t font, uint16_t options, const char *s) {
	uint16_t b[6];
	b[0] = x;
	b[1] = y;
	b[2] = w;
	b[3] = h;
	b[4] = font;
	b[5] = options;

	uint32_t len = strlen(s) + 1;
	int8_t align = (4 - (len & 0x3)) & 0x3;

	// check that we have enough space then send command
	checkfree(4 + sizeof(b) + len + align);
	cFFFFFF(0x0e);
	cN((uint8_t*) b, sizeof(b));
	cN((uint8_t*) s, len);
	align32(len);
}

/*
 * 5.36 CMD_PROGRESS
 * Draw a progress bar
 */
void FT81xRenderer::cmd_progress(int16_t x, int16_t y, int16_t w, int16_t h,
		uint16_t options, uint16_t val, uint16_t range) {
	uint16_t b[8];
	b[0] = x;
	b[1] = y;
	b[2] = w;
	b[3] = h;
	b[4] = options;
	b[5] = val;
	b[6] = range;
	b[7] = 0; // dummy pad

	// check that we have enough space then send command
	checkfree(4 + sizeof(b));
	cFFFFFF(0x0f);
	cN((uint8_t*) b, sizeof(b));
}

/*
 * 5.37 CMD_SCROLLBAR
 * Draw a scroll bar
 */
void FT81xRenderer::cmd_scrollbar(int16_t x, int16_t y, int16_t w, int16_t h,
		uint16_t options, uint16_t val, uint16_t size, uint16_t range) {
	uint16_t b[8];
	b[0] = x;
	b[1] = y;
	b[2] = w;
	b[3] = h;
	b[4] = options;
	b[5] = val;
	b[6] = size;
	b[7] = range;

	// check that we have enough space then send command
	checkfree(4 + sizeof(b));
	cFFFFFF(0x11);
	cN((uint8_t*) b, sizeof(b));
}

/*
 * 5.38 CMD_SLIDER
 * Draw a slider
 */
void FT81xRenderer::cmd_slider(int16_t x, int16_t y, int16_t w, int16_t h,
		uint16_t options, uint16_t val, uint16_t range) {
	uint16_t b[8];
	b[0] = x;
	b[1] = y;
	b[2] = w;
	b[3] = h;
	b[4] = options;
	b[5] = val;
	b[6] = range;
	b[7] = 0; // dummy pad

	// check that we have enough space then send command
	checkfree(4 + sizeof(b));
	cFFFFFF(0x10);
	cN((uint8_t*) b, sizeof(b));
}

/*
 * 5.39 CMD_DIAL
 * Draw a rotary dial control
 */
void FT81xRenderer::cmd_dial(int16_t x, int16_t y, int16_t r, uint16_t options,
		uint16_t val) {
	uint16_t b[6];
	b[0] = x;
	b[1] = y;
	b[2] = r;
	b[3] = options;
	b[4] = val;
	b[5] = 0; // dummy pad

	// check that we have enough space then send command
	checkfree(4 + sizeof(b));
	cFFFFFF(0x2d);
	cN((uint8_t*) b, sizeof(b));
}

/*
 * 5.40 CMD_TOGGLE
 * Draw a toggle switch
 */
void FT81xRenderer::cmd_toggle(int16_t x, int16_t y, int16_t w, int16_t font,
		uint16_t options, uint16_t state, const char *s) {
	uint16_t b[6];
	b[0] = x;
	b[1] = y;
	b[2] = w;
	b[3] = font;
	b[4] = options;
	b[5] = state;
	uint32_t len = strlen(s) + 1;
	int8_t align = (4 - (len & 0x3)) & 0x3;

	// check that we have enough space then send command
	checkfree(4 + sizeof(b) + len + align);
	cFFFFFF(0x12);
	cN((uint8_t*) b, sizeof(b));
	cN((uint8_t*) s, len);
	align32(len);
}

/*
 * 5.41 CMD_TEXT
 * Draw text
 */
void FT81xRenderer::cmd_text(int16_t x, int16_t y, int16_t font, uint16_t options,
		const char *s) {
	uint16_t b[4];
	b[0] = x;
	b[1] = y;
	b[2] = font;
	b[3] = options;
	uint32_t len = strlen(s) + 1;
	int8_t align = (4 - (len & 0x3)) & 0x3;

	// check that we have enough space then send command
	checkfree(4 + sizeof(b) + len + align);
	cFFFFFF(0x0c);
	cN((uint8_t*) b, sizeof(b));
	cN((uint8_t*) s, len);
	align32(len);
}

/*
 * 5.42 CMD_SETBASE
 * Set the base for number output
 */
void FT81xRenderer::cmd_setbase(uint32_t b) {
	// check that we have enough space then send command
	checkfree(8);
	cFFFFFF(0x38);
	cI(b);
}

/*
 * 5.43 CMD_NUMBER
 * Draw number
 */
void FT81xRenderer::cmd_number(int16_t x, int16_t y, int16_t font,
		uint16_t options, int32_t n) {
	uint16_t b[6];
	b[0] = x;
	b[1] = y;
	b[2] = font;
	b[3] = options;
	b[4] = n;
	b[5] = 0; // dummy pad

	// check that we have enough space then send command
	checkfree(sizeof(b) + 4);
	cFFFFFF(0x2e);
	cN((uint8_t*) &b, sizeof(b));
}

/*
 * 5.44 CMD_LOADIDENTITY
 * Set the current matrix to the identity matrix
 */
void FT81xRenderer::cmd_loadidentity() {
	// check that we have enough space then send command
	checkfree(4);
	cFFFFFF(0x26);
}

/*
 * 5.45 CMD_SETMATRIX
 * Write the current matrix to the display list
 */
void FT81xRenderer::cmd_setmatrix() {
	// check that we have enough space then send command
	checkfree(4);
	cFFFFFF(0x2a);
}

/*
 * 5.46 CMD_GETMATRIX
 * Retrieves the current matrix within the context of the co-processor engine
 */
void FT81xRenderer::cmd_getmatrix(int32_t *a, int32_t *b, int32_t *c, int32_t *d,
		int32_t *e, int32_t *f) {
	// check that we have enough space then send command
	checkfree(4);
	cFFFFFF(0x33);

	// The data will be written starting here in the buffer so get the pointer
	uint32_t r = getwp();

	// Fill in memory where results will go with dummy data
	cI(0xffffffff); // Will be a
	cI(0xffffffff); // Will be b
	cI(0xffffffff); // Will be c
	cI(0xffffffff); // Will be d
	cI(0xffffffff); // Will be e
	cI(0xffffffff); // Will be f

	// report back memory locations of the results to caller
	*a = r;
	r += 4;
	*b = r;
	r += 4;
	*c = r;
	r += 4;
	*d = r;
	r += 4;
	*e = r;
	r += 4;
	*f = r;
}

/*
 * 5.47 CMD_GETPTR
 * Get the end memory address of data inflated by CMD_INFLATE
 */
void FT81xRenderer::cmd_getptr(uint32_t *result) {
	// check that we have enough space then send command
	checkfree(8);
	cFFFFFF(0x23);

	// The data will be written starting here in the buffer so get the pointer
	uint32_t r = getwp();

	// Fill in memory where results will go with dummy data
	cI(0xffffffff); // Will be ptr

	// report back memory locations of the results to caller
	*result = r;
}

/*
 * 5.48 CMD_GETPROPS
 * Get the image properties decompressed by CMD_LOADIMAGE
 * BLOCKING CALL, expects to be in streaming mode
 */
void FT81xRenderer::cmd_getprops(uint32_t *ptr, uint32_t *width,
		uint32_t *height) {

	// check that we have enough space then send command
	checkfree(16);
	cFFFFFF(0x25);

	// The data will be written starting here in the buffer so get the pointer
	uint32_t r = getwp();

	// Fill in memory where results will go with dummy data
	cI(0xffffffff); // Will be ptr
	cI(0xffffffff); // Will be width
	cI(0xffffffff); // Will be height

	// report back memory locations of the results to caller
	*ptr = r;
	r += 4;
	*width = r;
	r += 4;
	*height = r;
}

/*
 * 5.49 CMD_SCALE
 * Apply a scale to the current matrix
 */
void FT81xRenderer::cmd_scale(int32_t sx, int32_t sy) {
	// check that we have enough space then send command
	checkfree(12);
	cFFFFFF(0x28);
	cI(sx);
	cI(sy);
}

/*
 * 5.50 CMD_ROTATE
 * Apply a rotation to the current matrix
 */
void FT81xRenderer::cmd_rotate(int32_t a) {
	// check that we have enough space then send command
	checkfree(8);
	cFFFFFF(0x29);
	cI(a);
}

/*
 * 5.51 CMD_TRANSLATE
 * Apply a translation to the current matrix
 */
void FT81xRenderer::cmd_translate(int32_t tx, int32_t ty) {
	// check that we have enough space then send command
	checkfree(12);
	cFFFFFF(0x27);
	cI(tx);
	cI(tx);
}

/*
 * 5.52 CMD_CALIBRATE
 * Execute the touch screen calibration routine
 */
void FT81xRenderer::cmd_calibrate(uint32_t *result) {
	// check that we have enough space then send command
	checkfree(4);
	cFFFFFF(0x15);

	// The data will be written starting here in the buffer so get the pointer
	uint32_t r = getwp();

	// Fill in memory where results will go with dummy data
	cI(0xffffffff); // Will be result

	// report back memory locations of the results to caller
	*result = r;
}

void FT81xRenderer::calibrate() {
	stream_start();  // Start streaming
	clear_color_rgb32(0xffffff);
	color_rgb32(0xffffff);
	bgcolor_rgb32(0x402000);
	fgcolor_rgb32(0x703800);
	cmd_dlstart();   // Set REG_CMD_DL when done
	clear();         // Clear the display
	getfree(0);      // trigger FT81xRenderer to read the command buffer

	cmd_text(180, 30, 40, OPT_CENTER, "Please tap on the dot..");
	//cmd_calibrate(0);// Calibration command
	//cmd_swap();      // Set AUTO swap at end of display list

	stream_stop();   // Finish streaming to command buffer
	// Wait till the Logo is finished
	wait_finish();
}

/*
 * 5.53 CMD_SETROTATE
 * Rotate the screen
 */
void FT81xRenderer::cmd_setrotate(uint32_t r) {
	// check that we have enough space then send command
	checkfree(8);
	cFFFFFF(0x36);
	cI(r);

	// Get our screen size W,H to confirm
	display_width = rd16(REG_HSIZE);
	display_height = rd16(REG_VSIZE);

	// portrait mode swap w & h
	if (r & 2) {
		int t = display_height;
		display_height = display_width;
		display_width = t;
	}
}

/*
 * 5.54 CMD_SPINNER
 * Start an animated spinner
 */
void FT81xRenderer::cmd_spinner(int16_t x, int16_t y, int16_t style,
		int16_t scale) {
	uint16_t b[4];
	b[0] = x;
	b[1] = y;
	b[2] = style;
	b[3] = scale;

	// check that we have enough space to run the command
	checkfree(sizeof(b) + 4);
	cFFFFFF(0x16);
	cN((uint8_t*) &b, sizeof(b));
}

/*
 * 5.55 CMD_SCREENSAVER
 * Start an animated screensaver
 */
void FT81xRenderer::cmd_screensaver() {
	// check that we have enough space then send command
	checkfree(4);
	cFFFFFF(0x2f);
}

/*
 * 5.56 CMD_SKETCH
 * Start a continuous sketch update
 */
void FT81xRenderer::cmd_sketch(int16_t x, int16_t y, int16_t w, int16_t h,
		int16_t ptr, int16_t format) {
	uint16_t b[6];
	b[0] = x;
	b[1] = y;
	b[2] = w;
	b[3] = h;
	b[4] = ptr;
	b[5] = format; // dummy pad

	// check that we have enough space then send command
	checkfree(sizeof(b) + 4);
	cFFFFFF(0x30);
	cN((uint8_t*) &b, sizeof(b));
}

/*
 * 5.57 CMD_STOP
 * Stop any active spinner, screensaver or sketch
 */
void FT81xRenderer::cmd_stop() {
	// check that we have enough space then send command
	checkfree(4);
	cFFFFFF(0x17);
}

/*
 * 5.58 CMD_SETFONT
 * Set up a custom font
 */
void FT81xRenderer::cmd_setfont(uint32_t font, uint32_t ptr) {
	// check that we have enough space then send command
	checkfree(12);
	cFFFFFF(0x2b);
	cI(font);
	cI(ptr);
}

/*
 * 5.59 CMD_SETFONT2
 * Set up a custom font
 */
void FT81xRenderer::cmd_setfont2(uint32_t handle, uint32_t font, uint32_t ptr,
		uint32_t firstchar) {
	// check that we have enough space then send command
	checkfree(16);
	cFFFFFF(0x3b);
	cI(font);
	cI(ptr);
	cI(firstchar);
}

/*
 * 5.60 CMD_SETSCRATCH
 * Set the scratch bitmap for widget use
 */
void FT81xRenderer::cmd_setscratch(uint32_t handle) {
	// check that we have enough space then send command
	checkfree(8);
	cFFFFFF(0x3c);
	cI(handle);
}

/*
 * 5.61 CMD_ROMFONT
 * Load a ROM font into bitmap handle
 */
void FT81xRenderer::cmd_romfont(uint32_t font, uint32_t slot) {
	// check that we have enough space then send command
	checkfree(12);
	cFFFFFF(0x3f);
	cI(font);
	cI(slot);
}

/*
 * 5.62 CMD_TRACK
 * Track touches for a graphics object
 */
void FT81xRenderer::cmd_track(int16_t x, int16_t y, int16_t width, int16_t height,
		int16_t tag) {
	uint16_t b[6];
	b[0] = x;
	b[1] = y;
	b[2] = width;
	b[3] = height;
	b[4] = tag;
	b[5] = 0; // dummy pad

	// check that we have enough space then send command
	checkfree(4 + sizeof(b));
	cFFFFFF(0x2c);
	cN((uint8_t*) &b, sizeof(b));
}

/*
 * 5.63 CMD_SNAPSHOT
 * Take a snapshot of the current screen
 */
void FT81xRenderer::cmd_snapshot(uint32_t ptr) {
	// check that we have enough space then send command
	checkfree(8);
	cFFFFFF(0x1f);
	cI(ptr);
}

/*
 * 5.64 CMD_SNAPSHOT2
 * Take a snapshot of the current screen
 */
void FT81xRenderer::cmd_snapshot2(uint32_t fmt, uint32_t ptr, uint16_t x,
		uint16_t y, uint16_t width, uint16_t height) {
	uint16_t b[4];
	b[0] = x;
	b[1] = y;
	b[2] = width;
	b[3] = height;

	// check that we have enough space then send command
	checkfree(12 + sizeof(b));
	cFFFFFF(0x37);
	cI(fmt);
	cI(ptr);
	cN((uint8_t*) &b, sizeof(b));
}

/*
 * 5.65 CMD_SETBITMAP
 * Set up display list for bitmap
 */
void FT81xRenderer::cmd_setbitmap(uint32_t addr, uint16_t fmt, uint16_t width,
		uint16_t height) {
	uint16_t b[4];
	b[0] = fmt;
	b[1] = width;
	b[2] = height;
	b[3] = 0;

	// check that we have enough space then send command
	checkfree(8 + sizeof(b));
	cFFFFFF(0x43);
	cI(addr);
	cN((uint8_t*) &b, sizeof(b));
}

/*
 * 5.66 CMD_LOGO
 * Play FTDI logo animation wait till it is done
 * FIXME: freespace()
 */
void FT81xRenderer::logo() {
	stream_start(); // Start streaming
	cmd_dlstart();  // Set REG_CMD_DL when done
	cFFFFFF(0x31);  // Logo command
	cmd_swap();     // Set AUTO swap at end of display list
	getfree(0);     // trigger FT81xRenderer to read the command buffer
	stream_stop();  // Finish streaming to command buffer
	// Wait till the Logo is finished
	wait_finish();
	// AFAIK the only command that will set the RD/WR to 0 when finished
	fifo_reset();
}
