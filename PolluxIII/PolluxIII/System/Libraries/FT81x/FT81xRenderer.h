/*
 * FT81xRenderer.h
 *
 *  Created on: 29 May 2023
 *      Author: arion (Just the C++ wrapper, all credits go to Sean Mathews)
 */

/**
 *  @file    ft81x.h
 *  @author  Sean Mathews <coder@f34r.com>
 *  @date    08/01/2018
 *  @version 1.1
 *
 *  @brief API to communicate with a FT81X chip from a ESP32 uP
 *
 *  This code provides an API to communicate with the FT81X chip
 *  from an ESP32. It simplifies the complexity of SPI on the ESP32
 *  correctly formatting the SPI communications to work with
 *  the FT81X. It also allow for QUAD SPI communications where
 *  permitted to increase data transfer speeds.
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


#ifndef FT81XRENDERER_H_
#define FT81XRENDERER_H_


#include <stdbool.h>
#include <stdint.h>

#include "EmbeddedGL/Renderer.h"
#include "Debug/Console.h"
#include "QSPIWrapper.h"

/*
 * Begin configuration
 */

#define configFT_SOUND_TEST

/*
 * End configuration
 */

#define FT_QUADSPI         0


#define FT_IO_TIMEOUT		  100

#define SPI_WIDTH_SINGLE 0b0
#define SPI_WIDTH_DUAL 0b01
#define SPI_WIDTH_QUAD 0b10

#define DL_CMD_FAULT 0xfff
#define MAX_FIFO_SPACE (4096 - 4)
#define DISPLAY_WIDTH 480
#define DISPLAY_HEIGHT 272

// Currently with DMA setting of 0 the ESP32 can only transfer 32 bytes per transaction.
// Setting DMA mode > 0 has other issues with data not readable. Bugs that may change
// this later. For now this is the MAX
#define CHUNKSIZE 0x20

#define SPI_SHIFT_DATA(data, len) __builtin_bswap32((uint32_t)data<<(32-len))
#define SPI_REARRANGE_DATA(data, len) (__builtin_bswap32(data)>>(32-len))
#define WRITE_OP              1
#define READ_OP               0
#define ORIENTATION           0

// Table 6 FT81X graphics primitive operation definition
#define BITMAPS               1
#define POINTS                2
#define LINES                 3
#define LINE_STRIP            4
#define EDGE_STRIP_R          5
#define EDGE_STRIP_L          6
#define EDGE_STRIP_A          7
#define EDGE_STRIP_B          8
#define RECTS                 9

// Table 7 BITMAP_LAYOUT format list
#define ARGB1555              0
#define L1                    1
#define L4                    2
#define L8                    3
#define RGB332                4
#define ARGB2                 5
#define ARGB4                 6
#define RGB565                7
#define PALETTED              8
#define TEXT8X8               9
#define TEXTVGA              10
#define BARGRAPH             11
#define PALETTED565          14
#define PALETTED4444         15
#define PALETTED8            16
#define L2                   17


#define NEAREST              0
#define BILINEAR             1
#define BORDER               0
#define REPEAT               1

// Table 8 BLEND_FUNC constant value definition
#define ZERO                  0
#define ONE                   1
#define SRC_ALPHA             2
#define DST_ALPHA             3
#define ONE_MINUS_SRC_ALPHA   4
#define ONE_MINUS_DST_ALPHA   5

// Table 13 FT81X OPTIONS
#define OPT_3D                0
#define OPT_RGB565            0
#define OPT_MONO              1
#define OPT_NODL              2
#define OPT_FLAT            256
#define OPT_SIGNED          256
#define OPT_CENTERX         512
#define OPT_CENTERY        1024
#define OPT_CENTER         1536
#define OPT_RIGHTX         2048
#define OPT_NOBACK         4096
#define OPT_NOTICKS        8192
#define OPT_NOHM          16384
#define OPT_NOPOINTER     16384
#define OPT_NOSECS        32768
#define OPT_NOHANDS       49152
#define OPT_NOTEAR            4
#define OPT_FULLSCREEN        8
#define OPT_MEDIAFIFO        16
#define OPT_SOUND            32

// Figure 5 FT81X The constants of ALPHA_FUNC
#define NEVER                 0
#define LESS                  1
#define LEQUAL                2
#define GREATER               3
#define GEQUAL                4
#define EQUAL                 5
#define NOTEQUAL              6
#define ALWAYS                7

// Figure 10 FT81X STENCIL_OP constants definition
#define ZERO                  0
#define KEEP                  1
#define REPLACE               2
#define INCR                  3
#define DECR                  4
#define INVERT                5

// Sound stream types
#define LINEAR_SAMPLES       0
#define ULAW_SAMPLES         1
#define ADPCM_SAMPLES        2

// Table 4-15 Sound Effect
#define SILENCE            0x00
#define SQUAREWAVE         0x01
#define SINEWAVE           0x02
#define SAWTOOTH           0x03
#define TRIANGLE           0x04
#define BEEPING            0x05
#define ALARM              0x06
#define WARBLE             0x07
#define CAROUSEL           0x08
#define PIPS(n)    (0x0f + (n))
#define HARP               0x40
#define XYLOPHONE          0x41
#define TUBA               0x42
#define GLOCKENSPIEL       0x43
#define ORGAN              0x44
#define TRUMPET            0x45
#define PIANO              0x46
#define CHIMES             0x47
#define MUSICBOX           0x48
#define BELL               0x49
#define CLICK              0x50
#define SWITCH             0x51
#define COWBELL            0x52
#define NOTCH              0x53
#define HIHAT              0x54
#define KICKDRUM           0x55
#define POP                0x56
#define CLACK              0x57
#define CHACK              0x58
#define MUTE               0x60
#define UNMUTE             0x61



// MEMORY MAP DEFINES
#define RAM_G          0x000000UL // General purpose graphics RAM
#define ROM_FONT       0x1e0000UL // Font table and bitmap
#define ROM_FONT_ADDR  0x2ffffcUL // Font table pointer address
#define RAM_DL         0x300000UL // Display List RAM
#define RAM_REG        0x302000UL // Registers
#define RAM_CMD        0x308000UL // Command Buffer

// REGISTERS                             BITSRW DESC
#define MEM_CHIP_ID          0x0c0000UL // 32RW Chip id 08h, [id], 01h, 00h: FT8xx=10h, 11h, 12h, 13h
#define REG_ID               0x302000UL //  8RO id register = 0x7ch
#define REG_FRAMES           0x302004UL // 32RO frame counter since reset
#define REG_CLOCK            0x302008UL // 32RO Clock cyc since reset
#define REG_FREQUENCY        0x30200cUL // 28RW Main clock frequency (Hz)
#define REG_RENDERMODE       0x302010UL //  1RW Render mode : 0 = normal, 1 = single line
#define REG_SNAPY            0x302014UL // 11RW Scanline select for RENDERMODE 1
#define REG_SNAPSHOT         0x302018UL //  1RW Trigger for RENDERMODE 1
#define REG_SNAPFORMAT       0x30201cUL //  6RW Pixel format for scanline redout
#define REG_CPURESET         0x302020UL //  3RW Reset Coprocessor/Touch/Audio engine
#define REG_TAP_CRC          0x302024UL // 32RO Live video tap crc. Frame CRC is
#define REG_TAP_MASK         0x302028UL // 32RW Live video tap mask
#define REG_HCYCLE           0x30202CUL // 12RW Horizontal total cycle count
#define REG_HOFFSET          0x302030UL // 12RW Horizontal display start offset
#define REG_HSIZE            0x302034UL // 12RW Horizontal display pixel count
#define REG_HSYNC0           0x302038UL // 12RW Horizontal sync fall offset
#define REG_HSYNC1           0x30203cUL // 12RW Horizontal sync rise offset

#define REG_VCYCLE           0x302040UL // 12RW Vertical total cycle count
#define REG_VOFFSET          0x302044UL // 12RW Vertical display start offset
#define REG_VSIZE            0x302048UL // 12RW Vertical display line count
#define REG_VSYNC0           0x30204cUL // 10RW Vertical sync fall offset

#define REG_VSYNC1           0x302050UL // 10RW Vertical sync rise offset
#define REG_ROTATE           0x302058UL //  3RW Screen rot control. norm, invert, mirror portrait etc.

#define REG_DITHER           0x302060UL //  1RW Output dither enable
#define REG_SWIZZLE          0x302064UL //  4RW Output RGB signal swizzle
#define REG_PCLK_POL         0x30206cUL //  1RW PCLK Polarity out on edge 0=rise 1=fall

#define REG_PCLK             0x302070UL //  8RW PCLK frequency divider, 0=disable
#define REG_TAG              0x30207cUL //  8RO Tag of touched objected

#define REG_VOL_PB           0x302080UL //  8RW Playback volume of file/stream
#define REG_VOL_SOUND        0x302084UL //  8RW Playback volume of synthesizer
#define REG_SOUND            0x302088UL // 16RW Select synthesized sound effect
#define REG_PLAY             0x30208cUL //  1RW Play ON/OFF

#define REG_GPIO_DIR         0x302090UL //  8RW Legacy GPIO pin direction 0=In 1=Out
#define REG_GPIO             0x302094UL //  8RW Legacy GPIO pin direction
#define REG_GPIOX_DIR        0x302098UL // 16RW Extended GPIO pin direction 0=In 1=Out
#define REG_GPIOX            0x30209cUL // 16RW Extended GPIO read/write
#define REG_PLAYBACK_START   0x3020b4UL // 20RW Start of audio data in G_RAM
#define REG_PLAYBACK_LENGTH  0x3020b8UL // 20RW Length of audio data in G_RAM to play
#define REG_PLAYBACK_FREQ    0x3020c0UL // 16RO The sampling fequency of audio playback data. Units is in Hz.
#define REG_PLAYBACK_FORMAT  0x3020c4UL //  2RW The format of the audio data in RAM_G
#define REG_PLAYBACK_LOOP    0x3020c8UL //  1RW Loop back to start if 1
#define REG_PLAYBACK_PLAY    0x3020ccUL //  1RW Start when 1 is written

#define REG_PWM_DUTY         0x3020d4UL //  8RW Back-light PWM duty cycle
#define REG_PWM_HZ           0x3020d0UL // 14RW PWM output frequencey 250Hz-10000Hz
#define REG_CMD_READ         0x3020f8UL // 12RW Command buffer read pointer
#define REG_CMD_WRITE        0x3020fcUL // 12RW Command buffer write pointer
#define REG_CMDB_SPACE       0x302574UL // 12RW Free space in RAM_CMD
#define REG_CMDB_WRITE       0x302578UL // 32WO Data or Command to be written to RAM_CMD

// CTE Registers
#define REG_CTOUCH_MODE      0x302104UL //  2RW 00=Off 11=On
#define REG_CTOUCH_EXTENDED  0x302108UL //  1RW 0=Extended Mode 1=Compatability mode
#define REG_CTOUCH_TOUCH1_XY 0x30211cUL // 32RO X & Y second touch point
#define REG_CTOUCH_TOUCH4_Y  0x302120UL // 15RO Y fifth touch point
#define REG_CTOUCH_TOUCH0_XY 0x302124UL // 32RO X & Y First touch point
#define REG_CTOUCH_TAG0_XY   0x302128UL // 32RO XY used to calculate the tag of first touch point
#define REG_CTOUCH_TAG0      0x30212cUL //  8RO Tag result of first touch point
#define REG_CTOUCH_TAG1_XY   0x302130UL // 32RO XY used to calculate the tag of second touch point
#define REG_CTOUCH_TAG1      0x302134UL //  8RO Tag result of second touch point
#define REG_CTOUCH_TAG2_XY   0x302138UL // 32RO XY used to calculate the tag of third touch point
#define REG_CTOUCH_TAG2      0x30213cUL //  8RO Tag result of third touch point
#define REG_CTOUCH_TAG3_XY   0x302140UL // 32RO XY used to calculate the tag of forth touch point
#define REG_CTOUCH_TAG3      0x302144UL //  8RO Tag result of forth touch point
#define REG_CTOUCH_TAG4_XY   0x302148UL // 32RO XY used to calculate the tag of fifth touch point
#define REG_CTOUCH_TAG4      0x30214cUL //  8RO Tag result of fifth touch point
#define REG_CTOUCH_TOUCH4_X  0x30216cUL // 16RO X fifth touch point
#define REG_CTOUCH_TOUCH2_XY 0x30218cUL // 32RO X & Y third touch point
#define REG_CTOUCH_TOUCH3_XY 0x302190UL // 32RO X & Y forth touch point

// SPI
#define REG_SPI_WIDTH        0x302188UL //  3RW QSPI bus width and dummy cycle setting

// FT81X Special registers
#define REG_TRACKER          0x309000UL // 32RO Track and TAG value 0
#define REG_TRACKER_1        0x309004UL // 32RO Track and TAG value 1
#define REG_TRACKER_2        0x309008UL // 32RO Track and TAG value 2
#define REG_TRACKER_3        0x30900cUL // 32RO Track and TAG value 3
#define REG_TRACKER_4        0x309010UL // 32RO Track and TAG value 4

#define REG_MEDIAFIFO_READ   0x309014UL // 32RW FIFO Read pointer
#define REG_MEDIAFIFO_WRITE  0x309018UL // 32RW FIFO Write pointer

// COMMANDS
#define CMD_ACTIVE         0x00   // Switch from standby/sleep/pwrdown to active mode
#define CMD_STANDBY        0x41   // Put FT81X into standby mode
#define CMD_SLEEP          0x42   // Put FT81X core to sleep mode
#define CMD_PWRDOWN        0x43   // Switch off 1.2v core voltage. SPI still on.
#define CMD_PD_ROMS        0x49   // Select power down individual ROMs.
#define CMD_CLKEXT         0x44   // Select PLL input from external osc.
#define CMD_CLKINT         0x48   // Select PLL input from internal osc.
#define CMD_PWRDOWN2       0x50   // Switch off 1.2v core voltage. SPI still on.
#define CMD_CLKSEL         0x61   // Set clock in sleep mode. TODO: why 2?
#define CMD_CLKSEL2        0x62   // ""
#define CMD_RST_PULSE      0x68   // Send reset pulse to FT81x core.
#define CMD_PINDRIVE       0x70   // Pin driver power levels
#define PIN_PD_STATE       0x71   // Pin state when powered down Float/Pull-Down

#define TOUCH_POINTS 5
#define TRACKER_UNITS 65535

/*
 * Prototypes
 */



class FT81xRenderer : public Renderer {
public:
	FT81xRenderer(QSPIWrapper* spi, Console* console);

	void restart_core();

	bool read_chip_id();

	void select_spi_byte_width();
	void assert_cs(bool value);

	void init_display_settings();
	void init_touch_settings();
	void init_audio_settings();
	void init_gpio();

	bool test_comm();
	void test_black_screen();
	void test_sound();
	void test_logo();
	void test_load_image();
	void test_memory_ops();
	void test_display();
	void test_cycle_colors();
	void test_dots();

	bool multi_touch_enabled();

	void multi_touch_enable(bool enable);

	uint8_t touch_mode();

	// Initialize the ESP32 SPI device driver for the FT81X chip attached to the VSPI pins
	bool init_spi();

	// Initialize FT81X GPU
	bool init_gpu();

	// Put the display to sleep low power mode
	void sleep();

	// Wake the display from sleep low power mode
	void wake(uint8_t pwm);

	// reset the fifo state vars
	void fifo_reset();

	// While in stream() mode send out a bitmap handle command into the command fifo
	void BitmapHandle(uint8_t byte);

	// Series of commands to swap the display
	void swap();

	// 4.4 ALPHA_FUNCT - Specify the alpha test function
	void alpha_funct(uint8_t func, uint8_t ref);

	// 4.5 BEGIN - Begin drawing a graphics primitive
	void begin( uint8_t prim);

	// 4.6 BEGIN_HANDLE - Specify the bitmap handle
	void bitmap_handle(uint8_t handle);

	// 4.7 BITMAP_LAYOUT - Specify the source bitmap memory format and layout for the current handle
	void bitmap_layout(uint8_t format, uint16_t linestride, uint16_t height);

	// 4.8 BITMAP_LAYOUT_H - Specify the 2 most significant bits of the source bitmap memory format and layout for the current handle
	void bitmap_layout_h(uint8_t linestride, uint8_t height);

	// 4.9 BITMAP_SIZE - Specify the screen drawing of bitmaps for the current handle
	void bitmap_size(uint8_t filter, uint8_t wrapx, uint8_t wrapy, uint16_t width, uint16_t height);

	// 4.10 BITMAP_SIZE_H - Specify the source address of bitmap data in FT81X graphics memory RAM_G
	void bitmap_size_h(uint8_t width, uint8_t height);

	// 4.11 BITMAP_SOURCE - Specify the source address of bitmap data in FT81X graphics memory RAM_G
	void bitmap_source(uint32_t addr);

	// 4.12 BITMAP_TRANSFORM_A - Specify the A coefficient of the bitmap transform matrix
	void bitmap_transform_a(uint32_t a);

	// 4.13 BITMAP_TRANSFORM_B - Specify the B coefficient of the bitmap transform matrix
	void bitmap_transform_b(uint32_t b);

	// 4.14 BITMAP_TRANSFORM_C - Specify the C coefficient of the bitmap transform matrix
	void bitmap_transform_c(uint32_t c);

	// 4.15 BITMAP_TRANSFORM_D - Specify the D coefficient of the bitmap transform matrix
	void bitmap_transform_d(uint32_t d);

	// 4.16 BITMAP_TRANSFORM_E - Specify the E coefficient of the bitmap transform matrix
	void bitmap_transform_e(uint32_t e);

	// 4.17 BITMAP_TRANSFORM_F - Specify the F coefficient of the bitmap transform matrix
	void bitmap_transform_f(uint32_t f);

	// 4.18 BLEND_FUNC - Specify pixel arithmetic
	void blend_func(uint8_t src, uint8_t dst);

	// 4.19 CALL - Execute a sequence of commands at another location in the display list
	void call(uint16_t dest);

	// 4.20 CELL - Specify the bitmap cell number for the VERTEX2F command
	void cell(uint8_t cell);

	// 4.21 CLEAR - Clears buffers to preset values
	void clear();
	void clearCST(uint8_t color, uint8_t stencil, uint8_t tag);

	// 4.21 CLEAR_COLOR_A - Specify clear value for the alpha channel
	void clear_color_a(uint8_t alpha);

	// 4.23 CLEAR_COLOR_RGB - Specify clear values for red,green and blue channels
	void clear_color_rgb32(uint32_t rgb);
	void clear_color_rgb888(uint8_t red, uint8_t green, uint8_t blue);

	// 4.24 CLEAR_STENCIL - Specify clear value for the stencil buffer
	void clear_stencil(uint8_t stencil);

	// 4.25 CLEAR_TAG - Specify clear value for the tag buffer
	void clear_tag(uint8_t tag);

	// 4.26 COLOR_A - Set the current color alpha
	void color_a(uint8_t alpha);

	// 4.27 COLOR_MASK - Enable or disable writing of color components
	void color_mask(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha);

	// 4.28 COLOR_RGB - Set the current color red, green, blue
	void color_rgb32(uint32_t rgb);
	void color_rgb888(uint8_t red, uint8_t green, uint8_t blue);

	// 4.29 DISPLAY - End the display list. FT81X will ignore all commands following this command.
	void display();

	// 4.30 END - End drawing a graphics primitive
	void end();

	// 4.31 JUMP - Execute commands at another location in the display list
	void jump(uint16_t dest);

	// 4.32 LINE_WIDTH - Specify the width of lines to be drawn with primitive LINES in 1/16 pixel precision
	void line_width(uint16_t width);

	// 4.33 MACRO - Execute a single command from a macro register
	void macro(uint8_t m);

	// 4.34 NOP - No Operation
	void nop();

	// 4.35 PALETTE_SOURCE - Specify the base address of the palette
	void palette_source(uint32_t addr);

	// 4.36 POINT_SIZE - Specify the radius of points
	void point_size(uint16_t size);

	// 4.37 RESTORE_CONTEXT - Restore the current graphics context from the context stack
	void restore_context();

	// 4.38 RETURN - Return from a previous CALL command
	void return_call();

	// 4.39 SAVE_CONTEXT - Push the current graphics context on the context stack
	void save_context();

	// 4.40 SCISSOR_SIZE - Specify the size of the scissor clip rectangle
	void scissor_size(uint16_t width, uint16_t height);

	// 4.41 SCISSOR_XY - Specify the top left corner of the scissor clip rectangle
	void scissor_xy(uint16_t x, uint16_t y);

	// 4.42 STENCIL_FUNC - Set function and reference value for stencil testing
	void stencil_func(uint8_t func, uint8_t ref, uint8_t mask);

	// 4.43 STENCIL_MASK - Control the writing of individual bits in the stencil planes
	void stencil_mask(uint8_t mask);

	// 4.44 STENCIL_OP - Set stencil test actions
	void stencil_op(uint8_t sfail, uint8_t spass);

	// 4.45 TAG - Attach the tag value for the following graphics objects drawn on the screen. def. 0xff
	void tag(uint8_t s);

	// 4.46 TAG_MASK - Control the writing of the tag buffer
	void tag_mask(uint8_t mask);

	// 4.47 VERTEX2F - Start the operation of graphics primitives at the specified screen coordinate
	void vertex2f(int16_t x, int16_t y);

	// 4.48 VERTEX2II - Start the operation of graphics primitives at the specified screen coordinate
	void vertex2ii(int16_t x, int16_t y, uint8_t handle, uint8_t cell);

	// 4.49 VERTEX_FORMAT - Set the precision of VERTEX2F coordinates
	void vertex_format(int8_t frac);

	// 4.50 VERTEX_TRANSLATE_X - Specify the vertex transformation’s X translation component
	void vertex_translate_x(uint32_t x);

	// 4.51 VERTEX_TRANSLATE_Y - Specify the vertex transformation’s Y translation component
	void vertex_translate_y(uint32_t y);

	// 5.11 CMD_DLSTART - Start a new display list
	void cmd_dlstart();

	// 5.12 CMD_SWAP - Swap the current display list
	void cmd_swap();

	// 5.13 CMD_COLDSTART - This command sets the co-processor engine to default reset states
	void cmd_coldstart();

	// 5.14 CMD_INTERRUPT - trigger interrupt INT_CMDFLAG
	void cmd_interrupt(uint32_t ms);

	// 5.15 CMD_APPEND - Append more commands to current display list
	void cmd_append(uint32_t ptr, uint32_t num);

	// 5.16 CMD_REGREAD - Read a register value
	void cmd_regread(uint32_t ptr, uint32_t *result);

	// 5.17 CMD_MEMWRITE - Write bytes into memory
	void cmd_memwrite(uint32_t ptr, uint32_t num);

	// 5.18 CMD_INFLATE - Decompress data into memory
	void cmd_inflate(uint32_t ptr);

	// 5.19 CMD_LOADIMAGE - Load a JPEG or PNG image
	void cmd_loadimage(uint32_t ptr, uint32_t options);

	// 5.20 CMD_MEDIAFIFO - set up a streaming media FIFO in RAM_G
	void cmd_mediafifo(uint32_t base, uint32_t size);

	// 5.21 CMD_PLAYVIDEO - Video playback
	void cmd_playvideo(uint32_t options);

	// 5.22 CMD_VIDEOSTART - Initialize the AVI video decoder
	void cmd_videostart();

	// 5.23 CMD_VIDEOFRAME - Load the next frame of video
	void cmd_videoframe(uint32_t dst, uint32_t ptr);

	// 5.24 CMD_MEMCRC - Compute a CRC-32 for memory
	uint32_t cmd_memcrc(uint32_t ptr, uint32_t num);

	// 5.25 CMD_MEMZERO - Write zero to a block of memory
	void cmd_memzero(uint32_t ptr, uint32_t num);

	// 5.26 CMD_MEMSET - Fill memory with a byte value
	void cmd_memset(uint32_t ptr, uint32_t value, uint32_t num);

	// 5.27 CMD_MEMCPY - Copy a block of memory
	void cmd_memcpy(uint32_t dest, uint32_t src, uint32_t num);

	// 5.28 CMD_BUTTON - Draw a button
	void cmd_button(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t font, uint16_t options, const char* s);

	// 5.29 CMD_CLOCK - draw an analog clock
	void cmd_clock(uint16_t x, uint16_t y, uint16_t r, uint16_t options, uint16_t h, uint16_t m, uint16_t s, uint16_t ms);

	// 5.30 CMD_FGCOLOR - set the foreground color
	void fgcolor_rgb32(uint32_t rgb);
	void fgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue);

	// 5.31 CMD_BGCOLOR - set the background color
	void bgcolor_rgb32(uint32_t rgb);
	void bgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue);

	// 5.32 CMD_GRADCOLOR - set the 3D button highlight color
	void cmd_gradcolor_rgb32(uint32_t rgb);
	void cmd_gradcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue);

	// 5.33 CMD_GAUGE - draw a gauge
	void cmd_gauge(int16_t x, int16_t y, int16_t r, uint16_t options, uint16_t major, uint16_t minor, uint16_t val, uint16_t range);

	// 5.34 CMD_GRADIENT - draw a smooth color gradient
	void cmd_gradient_rgb32(int16_t x0, int16_t y0, uint32_t rgb0, int16_t x1, int16_t y1, uint32_t rgb1);

	// 5.35 CMD_KEYS - draw a row of keys
	void cmd_keys(int16_t x, int16_t y, int16_t w, int16_t h, int16_t font, uint16_t options, const char *s);

	// 5.36 CMD_PROGRESS - draw a progress bar
	void cmd_progress(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t range);

	// 5.37 CMD_SCROLLBAR - draw a scroll bar
	void cmd_scrollbar(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t size, uint16_t range);

	// 5.38 CMD_SLIDER - draw a slider
	void cmd_slider(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t range);

	// 5.39 CMD_DIAL - Draw a rotary dial control
	void cmd_dial(int16_t x, int16_t y, int16_t r, uint16_t options, uint16_t val);

	// 5.40 CMD_TOGGLE - Draw a toggle switch
	void cmd_toggle(int16_t x, int16_t y, int16_t w, int16_t font, uint16_t options, uint16_t state, const char* s);

	// 5.41 CMD_TEXT - Draw text
	void cmd_text(int16_t x, int16_t y, int16_t font, uint16_t options, const char *s);

	// 5.42 CMD_SETBASE - Set the base for number output
	void cmd_setbase(uint32_t b);

	// 5.43 CMD_NUMBER - Draw number
	void cmd_number(int16_t x, int16_t y, int16_t font, uint16_t options, int32_t n);

	// 5.44 CMD_LOADIDENTITY - Set the current matrix to the identity matrix
	void cmd_loadidentity();

	// 5.45 CMD_SETMATRIX FIXME - Write the current matrix to the display list
	void cmd_setmatrix();

	// 5.46 CMD_GETMATRIX FIXME - Retrieves the current matrix within the context of the co-processor engine
	void cmd_getmatrix(int32_t *a, int32_t *b, int32_t *c, int32_t *d, int32_t *e, int32_t *f);

	// 5.47 CMD_GETPTR FIXME - Get the end memory address of data inflated by CMD_INFLATE
	void cmd_getptr(uint32_t *result);

	// 5.48 CMD_GETPROPS FIXME - Get the image properties decompressed by CMD_LOADIMAGE
	void cmd_getprops(uint32_t *ptr, uint32_t *width, uint32_t *height);

	// 5.49 CMD_SCALE - Apply a scale to the current matrix
	void cmd_scale(int32_t sx, int32_t sy);

	// 5.50 CMD_ROTATE - Apply a rotation to the current matrix
	void cmd_rotate(int32_t a);

	// 5.51 CMD_TRANSLATE - Apply a translation to the current matrix
	void cmd_translate(int32_t tx, int32_t ty);

	// 5.52 CMD_CALIBRATE - Execute the touch screen calibration routine
	void cmd_calibrate(uint32_t *result);
	void calibrate();

	// 5.53 CMD_SETROTATE - Rotate the screen
	void cmd_setrotate(uint32_t r);

	// 5.54 CMD_SPINNER - Start an animated spinner
	void cmd_spinner(int16_t x, int16_t y, int16_t style, int16_t scale);

	// 5.55 CMD_SCREENSAVER - Start an animated screensaver
	void cmd_screensaver();

	// 5.56 CMD_SKETCH - Start a continuous sketch update
	void cmd_sketch(int16_t x, int16_t y, int16_t w, int16_t h, int16_t ptr, int16_t format);

	// 5.57 CMD_STOP - Stop any active spinner, screensaver or sketch
	void cmd_stop();

	// 5.58 CMD_SETFONT - Set up a custom font
	void cmd_setfont(uint32_t font, uint32_t ptr);

	// 5.59 CMD_SETFONT2 - Set up a custom font
	void cmd_setfont2(uint32_t handle, uint32_t font, uint32_t ptr, uint32_t firstchar);

	// 5.60 CMD_SETSCRATCH - Set the scratch bitmap for widget use
	void cmd_setscratch(uint32_t handle);

	// 5.61 CMD_ROMFONT - Load a ROM font into bitmap handle
	void cmd_romfont(uint32_t font, uint32_t slot);

	// 5.62 CMD_TRACK - Track touches for a graphics object
	void cmd_track(int16_t x, int16_t y, int16_t width, int16_t height, int16_t tag);

	// 5.63 CMD_SNAPSHOT - Take a snapshot of the current screen
	void cmd_snapshot(uint32_t ptr);

	// 5.64 CMD_SNAPSHOT2 - Take a snapshot of the current screen
	void cmd_snapshot2(uint32_t fmt, uint32_t ptr, uint16_t x, uint16_t y, uint16_t width, uint16_t height);

	// 5.65 CMD_SETBITMAP - Set up display list for bitmap
	void cmd_setbitmap(uint32_t addr, uint16_t fmt, uint16_t width, uint16_t height);

	// 5.66 CMD_LOGO - play FTDI logo animation
	void logo();

	// Wait for READ and WRITE circular buffer command pointers to be equal
	void wait_finish();

	// Read in touch tag and tracker memory from FT813 to our local global structure
	struct touch_input_t* fetch_touch_inputs();

	// Set the current address and write mode to the fifo comand buffer
	// leaving the CS line enabled
	void stream_start();

	// Disable the CS line finish the transaction
	void stream_stop();

	// Get command buffer free block till we have 'required' space
	void getfree(uint16_t required);

	void checkfree(uint16_t required);


private:
	QSPIWrapper* spi;
	Console* console;

	// FT81X section 5.2 Chip ID
	uint16_t chip_id = 0;
	// FT81X command buffer free space
	uint16_t fifo_freespace = 0;
	// FT81X command buffer write pointer
	uint16_t fifo_wp = 0;
	// FT81X enable QIO modes
	uint8_t qio = 0;
	// FT81x screen width
	uint16_t display_width = 0;
	// FT81x screen height
	uint16_t display_height = 0;

	// MEDIA FIFO state vars
	uint32_t mf_wp = 0;
	uint32_t mf_size = 0;
	uint32_t mf_base = 0;

	// FT813 touch screen state loaded by calls to get_touch_inputs
	//// Capacitive touch state

	//struct ctouch_t ctouch;

	//// touch tracker state
	struct touch_tracker_t touch_tracker[TOUCH_POINTS];
	struct touch_input_t touch_input[TOUCH_POINTS];


	void load_default_command(QSPI_CommandTypeDef* command);

	// Get our current fifo write state location
	uint32_t getwp();

	// Send a Host Command to the FT81X chip see 4.1.5 Host Command
	#define hostcmd(command) hostcmd_param((command), 0)
	void hostcmd_param(uint8_t command, uint8_t args);

	// Send a 16 bit address + dummy and read the 8 bit response
	uint8_t rd(uint32_t addr);

	// Send a 16 bit address + dummy and read the 16 bit response
	uint16_t rd16(uint32_t addr);

	// Send a 16 bit address + dummy and read the 32 bit response
	uint32_t rd32(uint32_t addr);

	// Send a 16 bit address + dummy and read the N byte response in chunks
	void rdN(uint32_t addr, uint8_t *results, int8_t len);
	void rdn(uint32_t addr, uint8_t *results, int8_t len);

	// Address write op start. Send a 16 bit address only leave CS open for more data
	void wrA(uint32_t addr);

	// Spool a large block of memory in chunks into the FT81X tracking using MEDIA FIFO state
	void cSPOOL_MF(uint8_t *buffer, int32_t size);

	// Send a block of data to the spi device
	void wrN(uint8_t *buffer, uint8_t size);

	// End address write operation
	void wrE();

	// Send a 16 bit address and write 8 bits of data
	void wr(uint32_t addr, uint8_t byte);

	// Send a 16 bit address and write 16 bits of data
	void wr16(uint32_t addr, uint16_t word);

	// Send a 16 bit address and write 32 bits of data
	void wr32(uint32_t addr, uint32_t word);

	// Read the FT81x command pointer
	uint16_t fifo_rp();

	// Write out padded bits to be sure we are 32 bit aligned as required by the FT81X
	void align32(uint32_t written);

	// Write a 32bit command to the command buffer blocking if
	// free space is needed until enough is free to send the 4 bytes
	void cI(uint32_t word);

	// Write a 8bit+24bits of 0xff to the command buffer blocking if
	// free space is needed until enough is free to send the 4 bytes
	void cFFFFFF(uint8_t byte);

	// While in stream() mode send a 32 word into the command buffer.
	void cmd32(uint32_t word);

	// While in stream() mode send a char buffer into the command buffer.
	void cN(uint8_t *buffer, uint16_t size);

	// Spool a large block of memory in chunks into the FT81X in 1k chunks
	void cSPOOL(uint8_t *buffer, int32_t size);

};




#endif /* FT81XRENDERER_H_ */
