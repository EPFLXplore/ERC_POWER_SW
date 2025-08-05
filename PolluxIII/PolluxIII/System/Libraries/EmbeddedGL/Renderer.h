/*
 * Renderer.h
 *
 *  Created on: Jun 10, 2023
 *      Author: arion
 */

#ifndef LIBRARIES_EMBEDDEDGL_RENDERER_H_
#define LIBRARIES_EMBEDDEDGL_RENDERER_H_

#include <stdint.h>

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

/*
 * Types
 */

// FT813 touch screen state loaded by calls to get_touch_inputs
//// Capacitive touch state
struct touch_input_t {
	uint16_t tag;
	uint16_t tag_x;
	uint16_t tag_y;
	int16_t display_x; //no-touch: -32768
	int16_t display_y; //no-touch: -32768
};

//// touch tracker state
struct touch_tracker_t {
	uint16_t tag;
	uint16_t value;
};



class Renderer {
public:
	virtual ~Renderer() {}

	virtual bool multi_touch_enabled() { return false; }

	virtual void multi_touch_enable(bool enable) {}

	virtual uint8_t touch_mode() { return 0; }

	// 4.4 ALPHA_FUNCT - Specify the alpha test function
	virtual void alpha_funct(uint8_t func, uint8_t ref) {}

	// 4.5 BEGIN - Begin drawing a graphics primitive
	virtual void begin( uint8_t prim) {}

	// 4.6 BEGIN_HANDLE - Specify the bitmap handle
	virtual void bitmap_handle(uint8_t handle) {}

	// 4.7 BITMAP_LAYOUT - Specify the source bitmap memory format and layout for the current handle
	virtual void bitmap_layout(uint8_t format, uint16_t linestride, uint16_t height) {}

	// 4.8 BITMAP_LAYOUT_H - Specify the 2 most significant bits of the source bitmap memory format and layout for the current handle
	virtual void bitmap_layout_h(uint8_t linestride, uint8_t height) {}

	// 4.9 BITMAP_SIZE - Specify the screen drawing of bitmaps for the current handle
	virtual void bitmap_size(uint8_t filter, uint8_t wrapx, uint8_t wrapy, uint16_t width, uint16_t height) {}

	// 4.10 BITMAP_SIZE_H - Specify the source address of bitmap data in FT81X graphics memory RAM_G
	virtual void bitmap_size_h(uint8_t width, uint8_t height) {}

	// 4.11 BITMAP_SOURCE - Specify the source address of bitmap data in FT81X graphics memory RAM_G
	virtual void bitmap_source(uint32_t addr) {}

	// 4.12 BITMAP_TRANSFORM_A - Specify the A coefficient of the bitmap transform matrix
	virtual void bitmap_transform_a(uint32_t a) {}

	// 4.13 BITMAP_TRANSFORM_B - Specify the B coefficient of the bitmap transform matrix
	virtual void bitmap_transform_b(uint32_t b) {}

	// 4.14 BITMAP_TRANSFORM_C - Specify the C coefficient of the bitmap transform matrix
	virtual void bitmap_transform_c(uint32_t c) {}

	// 4.15 BITMAP_TRANSFORM_D - Specify the D coefficient of the bitmap transform matrix
	virtual void bitmap_transform_d(uint32_t d) {}

	// 4.16 BITMAP_TRANSFORM_E - Specify the E coefficient of the bitmap transform matrix
	virtual void bitmap_transform_e(uint32_t e) {}

	// 4.17 BITMAP_TRANSFORM_F - Specify the F coefficient of the bitmap transform matrix
	virtual void bitmap_transform_f(uint32_t f) {}

	// 4.18 BLEND_FUNC - Specify pixel arithmetic
	virtual void blend_func(uint8_t src, uint8_t dst) {}

	// 4.19 CALL - Execute a sequence of commands at another location in the display list
	virtual void call(uint16_t dest) {}

	// 4.20 CELL - Specify the bitmap cell number for the VERTEX2F command
	virtual void cell(uint8_t cell) {}

	// 4.21 CLEAR - Clears buffers to preset values
	virtual void clear() {}
	virtual void clearCST(uint8_t color, uint8_t stencil, uint8_t tag) {}

	// 4.21 CLEAR_COLOR_A - Specify clear value for the alpha channel
	virtual void clear_color_a(uint8_t alpha) {}

	// 4.23 CLEAR_COLOR_RGB - Specify clear values for red,green and blue channels
	virtual void clear_color_rgb32(uint32_t rgb) {}
	virtual void clear_color_rgb888(uint8_t red, uint8_t green, uint8_t blue) {}

	// 4.24 CLEAR_STENCIL - Specify clear value for the stencil buffer
	virtual void clear_stencil(uint8_t stencil) {}

	// 4.25 CLEAR_TAG - Specify clear value for the tag buffer
	virtual void clear_tag(uint8_t tag) {}

	// 4.26 COLOR_A - Set the current color alpha
	virtual void color_a(uint8_t alpha) {}

	// 4.27 COLOR_MASK - Enable or disable writing of color components
	virtual void color_mask(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha) {}

	// 4.28 COLOR_RGB - Set the current color red, green, blue
	virtual void color_rgb32(uint32_t rgb) {}
	virtual void color_rgb888(uint8_t red, uint8_t green, uint8_t blue) {}

	// 4.29 DISPLAY - End the display list. FT81X will ignore all commands following this command.
	virtual void display() {}

	// 4.30 END - End drawing a graphics primitive
	virtual void end() {}

	// 4.31 JUMP - Execute commands at another location in the display list
	virtual void jump(uint16_t dest) {}

	// 4.32 LINE_WIDTH - Specify the width of lines to be drawn with primitive LINES in 1/16 pixel precision
	virtual void line_width(uint16_t width) {}

	// 4.33 MACRO - Execute a single command from a macro register
	virtual void macro(uint8_t m) {}

	// 4.34 NOP - No Operation
	virtual void nop() {}

	// 4.35 PALETTE_SOURCE - Specify the base address of the palette
	virtual void palette_source(uint32_t addr) {}

	// 4.36 POINT_SIZE - Specify the radius of points
	virtual void point_size(uint16_t size) {}

	// 4.37 RESTORE_CONTEXT - Restore the current graphics context from the context stack
	virtual void restore_context() {}

	// 4.38 RETURN - Return from a previous CALL command
	virtual void return_call() {}

	// 4.39 SAVE_CONTEXT - Push the current graphics context on the context stack
	virtual void save_context() {}

	// 4.40 SCISSOR_SIZE - Specify the size of the scissor clip rectangle
	virtual void scissor_size(uint16_t width, uint16_t height) {}

	// 4.41 SCISSOR_XY - Specify the top left corner of the scissor clip rectangle
	virtual void scissor_xy(uint16_t x, uint16_t y) {}

	// 4.42 STENCIL_FUNC - Set function and reference value for stencil testing
	virtual void stencil_func(uint8_t func, uint8_t ref, uint8_t mask) {}

	// 4.43 STENCIL_MASK - Control the writing of individual bits in the stencil planes
	virtual void stencil_mask(uint8_t mask) {}

	// 4.44 STENCIL_OP - Set stencil test actions
	virtual void stencil_op(uint8_t sfail, uint8_t spass) {}

	// 4.45 TAG - Attach the tag value for the following graphics objects drawn on the screen. def. 0xff
	virtual void tag(uint8_t s) {}

	// 4.46 TAG_MASK - Control the writing of the tag buffer
	virtual void tag_mask(uint8_t mask) {}

	// 4.47 VERTEX2F - Start the operation of graphics primitives at the specified screen coordinate
	virtual void vertex2f(int16_t x, int16_t y) {}

	// 4.48 VERTEX2II - Start the operation of graphics primitives at the specified screen coordinate
	virtual void vertex2ii(int16_t x, int16_t y, uint8_t handle, uint8_t cell) {}

	// 4.49 VERTEX_FORMAT - Set the precision of VERTEX2F coordinates
	virtual void vertex_format(int8_t frac) {}

	// 4.50 VERTEX_TRANSLATE_X - Specify the vertex transformation’s X translation component
	virtual void vertex_translate_x(uint32_t x) {}

	// 4.51 VERTEX_TRANSLATE_Y - Specify the vertex transformation’s Y translation component
	virtual void vertex_translate_y(uint32_t y) {}

	// 5.11 CMD_DLSTART - Start a new display list
	virtual void cmd_dlstart() {}

	// 5.12 CMD_SWAP - Swap the current display list
	virtual void cmd_swap() {}

	// 5.13 CMD_COLDSTART - This command sets the co-processor engine to default reset states
	virtual void cmd_coldstart() {}

	// 5.14 CMD_INTERRUPT - trigger interrupt INT_CMDFLAG
	virtual void cmd_interrupt(uint32_t ms) {}

	// 5.15 CMD_APPEND - Append more commands to current display list
	virtual void cmd_append(uint32_t ptr, uint32_t num) {}

	// 5.16 CMD_REGREAD - Read a register value
	virtual void cmd_regread(uint32_t ptr, uint32_t *result) {}

	// 5.17 CMD_MEMWRITE - Write bytes into memory
	virtual void cmd_memwrite(uint32_t ptr, uint32_t num) {}

	// 5.18 CMD_INFLATE - Decompress data into memory
	virtual void cmd_inflate(uint32_t ptr) {}

	// 5.19 CMD_LOADIMAGE - Load a JPEG or PNG image
	virtual void cmd_loadimage(uint32_t ptr, uint32_t options) {}

	// 5.20 CMD_MEDIAFIFO - set up a streaming media FIFO in RAM_G
	virtual void cmd_mediafifo(uint32_t base, uint32_t size) {}

	// 5.21 CMD_PLAYVIDEO - Video playback
	virtual void cmd_playvideo(uint32_t options) {}

	// 5.22 CMD_VIDEOSTART - Initialize the AVI video decoder
	virtual void cmd_videostart() {}

	// 5.23 CMD_VIDEOFRAME - Load the next frame of video
	virtual void cmd_videoframe(uint32_t dst, uint32_t ptr) {}

	// 5.24 CMD_MEMCRC - Compute a CRC-32 for memory
	virtual uint32_t cmd_memcrc(uint32_t ptr, uint32_t num) { return 0; }

	// 5.25 CMD_MEMZERO - Write zero to a block of memory
	virtual void cmd_memzero(uint32_t ptr, uint32_t num) {}

	// 5.26 CMD_MEMSET - Fill memory with a byte value
	virtual void cmd_memset(uint32_t ptr, uint32_t value, uint32_t num) {}

	// 5.27 CMD_MEMCPY - Copy a block of memory
	virtual void cmd_memcpy(uint32_t dest, uint32_t src, uint32_t num) {}

	// 5.28 CMD_BUTTON - Draw a button
	virtual void cmd_button(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t font, uint16_t options, const char* s) {}

	// 5.29 CMD_CLOCK - draw an analog clock
	virtual void cmd_clock(uint16_t x, uint16_t y, uint16_t r, uint16_t options, uint16_t h, uint16_t m, uint16_t s, uint16_t ms) {}

	// 5.30 CMD_FGCOLOR - set the foreground color
	virtual void fgcolor_rgb32(uint32_t rgb) {}
	virtual void fgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue) {}

	// 5.31 CMD_BGCOLOR - set the background color
	virtual void bgcolor_rgb32(uint32_t rgb) {}
	virtual void bgcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue) {}

	// 5.32 CMD_GRADCOLOR - set the 3D button highlight color
	virtual void cmd_gradcolor_rgb32(uint32_t rgb) {}
	virtual void cmd_gradcolor_rgb888(uint8_t red, uint8_t green, uint8_t blue) {}

	// 5.33 CMD_GAUGE - draw a gauge
	virtual void cmd_gauge(int16_t x, int16_t y, int16_t r, uint16_t options, uint16_t major, uint16_t minor, uint16_t val, uint16_t range) {}

	// 5.34 CMD_GRADIENT - draw a smooth color gradient
	virtual void cmd_gradient_rgb32(int16_t x0, int16_t y0, uint32_t rgb0, int16_t x1, int16_t y1, uint32_t rgb1) {}

	// 5.35 CMD_KEYS - draw a row of keys
	virtual void cmd_keys(int16_t x, int16_t y, int16_t w, int16_t h, int16_t font, uint16_t options, const char *s) {}

	// 5.36 CMD_PROGRESS - draw a progress bar
	virtual void cmd_progress(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t range) {}

	// 5.37 CMD_SCROLLBAR - draw a scroll bar
	virtual void cmd_scrollbar(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t size, uint16_t range) {}

	// 5.38 CMD_SLIDER - draw a slider
	virtual void cmd_slider(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t range) {}

	// 5.39 CMD_DIAL - Draw a rotary dial control
	virtual void cmd_dial(int16_t x, int16_t y, int16_t r, uint16_t options, uint16_t val) {}

	// 5.40 CMD_TOGGLE - Draw a toggle switch
	virtual void cmd_toggle(int16_t x, int16_t y, int16_t w, int16_t font, uint16_t options, uint16_t state, const char* s) {}

	// 5.41 CMD_TEXT - Draw text
	virtual void cmd_text(int16_t x, int16_t y, int16_t font, uint16_t options, const char *s) {}

	// 5.42 CMD_SETBASE - Set the base for number output
	virtual void cmd_setbase(uint32_t b) {}

	// 5.43 CMD_NUMBER - Draw number
	virtual void cmd_number(int16_t x, int16_t y, int16_t font, uint16_t options, int32_t n) {}

	// 5.44 CMD_LOADIDENTITY - Set the current matrix to the identity matrix
	virtual void cmd_loadidentity() {}

	// 5.45 CMD_SETMATRIX FIXME - Write the current matrix to the display list
	virtual void cmd_setmatrix() {}

	// 5.46 CMD_GETMATRIX FIXME - Retrieves the current matrix within the context of the co-processor engine
	virtual void cmd_getmatrix(int32_t *a, int32_t *b, int32_t *c, int32_t *d, int32_t *e, int32_t *f) {}

	// 5.47 CMD_GETPTR FIXME - Get the end memory address of data inflated by CMD_INFLATE
	virtual void cmd_getptr(uint32_t *result) {}

	// 5.48 CMD_GETPROPS FIXME - Get the image properties decompressed by CMD_LOADIMAGE
	virtual void cmd_getprops(uint32_t *ptr, uint32_t *width, uint32_t *height) {}

	// 5.49 CMD_SCALE - Apply a scale to the current matrix
	virtual void cmd_scale(int32_t sx, int32_t sy) {}

	// 5.50 CMD_ROTATE - Apply a rotation to the current matrix
	virtual void cmd_rotate(int32_t a) {}

	// 5.51 CMD_TRANSLATE - Apply a translation to the current matrix
	virtual void cmd_translate(int32_t tx, int32_t ty) {}

	// 5.52 CMD_CALIBRATE - Execute the touch screen calibration routine
	virtual void cmd_calibrate(uint32_t *result) {}
	virtual void calibrate() {}

	// 5.53 CMD_SETROTATE - Rotate the screen
	virtual void cmd_setrotate(uint32_t r) {}

	// 5.54 CMD_SPINNER - Start an animated spinner
	virtual void cmd_spinner(int16_t x, int16_t y, int16_t style, int16_t scale) {}

	// 5.55 CMD_SCREENSAVER - Start an animated screensaver
	virtual void cmd_screensaver() {}

	// 5.56 CMD_SKETCH - Start a continuous sketch update
	virtual void cmd_sketch(int16_t x, int16_t y, int16_t w, int16_t h, int16_t ptr, int16_t format) {}

	// 5.57 CMD_STOP - Stop any active spinner, screensaver or sketch
	virtual void cmd_stop() {}

	// 5.58 CMD_SETFONT - Set up a custom font
	virtual void cmd_setfont(uint32_t font, uint32_t ptr) {}

	// 5.59 CMD_SETFONT2 - Set up a custom font
	virtual void cmd_setfont2(uint32_t handle, uint32_t font, uint32_t ptr, uint32_t firstchar) {}

	// 5.60 CMD_SETSCRATCH - Set the scratch bitmap for widget use
	virtual void cmd_setscratch(uint32_t handle) {}

	// 5.61 CMD_ROMFONT - Load a ROM font into bitmap handle
	virtual void cmd_romfont(uint32_t font, uint32_t slot) {}

	// 5.62 CMD_TRACK - Track touches for a graphics object
	virtual void cmd_track(int16_t x, int16_t y, int16_t width, int16_t height, int16_t tag) {}

	// 5.63 CMD_SNAPSHOT - Take a snapshot of the current screen
	virtual void cmd_snapshot(uint32_t ptr) {}

	// 5.64 CMD_SNAPSHOT2 - Take a snapshot of the current screen
	virtual void cmd_snapshot2(uint32_t fmt, uint32_t ptr, uint16_t x, uint16_t y, uint16_t width, uint16_t height) {}

	// 5.65 CMD_SETBITMAP - Set up display list for bitmap
	virtual void cmd_setbitmap(uint32_t addr, uint16_t fmt, uint16_t width, uint16_t height) {}

	// Wait for READ and WRITE circular buffer command pointers to be equal
	virtual void wait_finish() {}

	// Read in touch tag and tracker memory from FT813 to our local global structure
	virtual struct touch_input_t* fetch_touch_inputs() { return nullptr; }

	// Set the current address and write mode to the fifo comand buffer
	// leaving the CS line enabled
	virtual void stream_start() {}

	// Disable the CS line finish the transaction
	virtual void stream_stop() {}

	// Get command buffer free block till we have 'required' space
	virtual void getfree(uint16_t required) {}

	virtual void checkfree(uint16_t required) {}

};


#endif /* LIBRARIES_EMBEDDEDGL_RENDERER_H_ */
