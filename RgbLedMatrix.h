/*
  RgbLedMatrix - SPI Led Matrix Controller
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef _RGBLEDMATRIX_H_
#define _RGBLEDMATRIX_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#include <avr/interrupt.h>
#include <avr/io.h>

// define IO
#define __spi_clock 13   // SCK - hardware SPI
#define __spi_latch 10
#define __spi_data 11    // MOSI - hardware SPI
#define __spi_data_in 12 // MISO - hardware SPI (unused)

// define setup
#define __display_enable 9
#define __rows 8
#define __max_row __rows-1
#define __leds_per_row 8
#define __max_led __leds_per_row-1
#define __brightness_levels 16 // 0...15 above 28 is bad for ISR ( move to timer1, lower irq freq ! )
#define __max_brightness 15
#define __fade_delay 8

#define INIT_TIMER_COUNT 0
#define RESET_TIMER2 TCNT2 = INIT_TIMER_COUNT

class RgbLedMatrixObject {
public:
  byte brightness_red[__leds_per_row][__rows];
  byte brightness_red_back[__leds_per_row][__rows];
  byte brightness_red_tmp[__leds_per_row][__rows];

  byte brightness_green[__leds_per_row][__rows];
  byte brightness_green_back[__leds_per_row][__rows];
  byte brightness_green_tmp[__leds_per_row][__rows];

  byte brightness_blue[__leds_per_row][__rows];
  byte brightness_blue_back[__leds_per_row][__rows];
  byte brightness_blue_tmp[__leds_per_row][__rows];


  void setup(void);
  void clear_matrix(void);
  void flip_back(void);
  void set_led_red(byte row, byte led, byte red);
  void set_led_green(byte row, byte led, byte green);
  void set_led_blue(byte row, byte led, byte blue);
  void set_led_rgb(byte row, byte led, byte red, byte green, byte blue);
  void set_led_hue(byte row, byte led, int hue);
  void set_matrix_hue(int hue);
  void set_row_rgb(byte row, byte red, byte green, byte blue);
  void set_column_rgb(byte column, byte red, byte green, byte blue);
  void set_row_hue(byte row, int hue);
  void set_column_hue(byte column, int hue);
  void set_row_byte_hue(byte row, byte data_byte, int hue);
  void set_matrix_rgb(byte red, byte green, byte blue);
  void fader(void);
  void fader_hue(void);
};

extern RgbLedMatrixObject RgbLedMatrix;

#endif // _RGBLEDMATRIX_H_
