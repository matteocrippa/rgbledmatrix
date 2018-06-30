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
#include "RgbLedMatrix.h"


void RgbLedMatrixObject::setup(void) {
  randomSeed(555);
  byte ctr1;
  byte ctr2;

  pinMode(__spi_clock,OUTPUT);
  pinMode(__spi_latch,OUTPUT);
  pinMode(__spi_data,OUTPUT);
  pinMode(__spi_data_in,INPUT);
  pinMode(__display_enable,OUTPUT);
  digitalWrite(__spi_latch,LOW);
  digitalWrite(__spi_data,LOW);
  digitalWrite(__spi_clock,LOW);

  byte clr;
  SPCR |= ( (1<<SPE) | (1<<MSTR) ); // enable SPI as master
  //SPCR |= ( (1<<SPR1) | (1<<SPR0) ); // set prescaler bits
  SPCR &= ~( (1<<SPR1) | (1<<SPR0) ); // clear prescaler bits
  clr=SPSR; // clear SPI status reg
  clr=SPDR; // clear SPI data reg
  SPSR |= (1<<SPI2X); // set prescaler bits
  //SPSR &= ~(1<<SPI2X); // clear prescaler bits

  delay(10);

  clear_matrix();
  digitalWrite(__display_enable,LOW);

  // set irq to 61 Hz: CS22-bit = 1, CS21-bit = 1, CS20-bit = 1
  TCCR2B |= ( (1<<CS22) | (1<<CS21) | (1<<CS20));
  // Use normal mode
  TCCR2A &= ~( (1<<WGM21) | (1<<WGM20) );
  TCCR2B &= ~( (1<<WGM22) );
  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= (1<<TOIE2);
  TIMSK2 &= ~( (1<<OCIE2A) | (1<<OCIE2B) );
  RESET_TIMER2;
  // enable all interrupts
  sei();
}

void RgbLedMatrixObject::clear_matrix(void) {
  RgbLedMatrix.set_matrix_rgb(0,0,0);
}

void RgbLedMatrixObject::set_led_red(byte row, byte led, byte red) {
  RgbLedMatrix.brightness_red[row][led] = red;
}

void RgbLedMatrixObject::set_led_green(byte row, byte led, byte green) {
  RgbLedMatrix.brightness_green[row][led] = green;
}

void RgbLedMatrixObject::set_led_blue(byte row, byte led, byte blue) {
  RgbLedMatrix.brightness_blue[row][led] = blue;
}

void RgbLedMatrixObject::set_led_rgb(byte row, byte led, byte red, byte green, byte blue) {
  set_led_red(row,led,red);
  set_led_green(row,led,green);
  set_led_blue(row,led,blue);
}

void RgbLedMatrixObject::set_led_hue(byte row, byte led, int hue) {

  // see wikipeda: HSV
  float S=100.0,V=100.0,s=S/100.0,v=V/100.0,h_i,f,p,q,t,R,G,B;

    hue = hue%360;
    h_i = hue/60;
    f = (float)(hue)/60.0 - h_i;
    p = v*(1-s);
    q = v*(1-s*f);
    t = v*(1-s*(1-f));

    if      ( h_i == 0 ) {
      R = v;
      G = t;
      B = p;
    }
    else if ( h_i == 1 ) {
      R = q;
      G = v;
      B = p;
    }
    else if ( h_i == 2 ) {
      R = p;
      G = v;
      B = t;
    }
    else if ( h_i == 3 ) {
      R = p;
      G = q;
      B = v;
    }
    else if ( h_i == 4 ) {
      R = t;
      G = p;
      B = v;
    }
    else                   {
      R = v;
      G = p;
      B = q;
    }

    set_led_rgb(row,led,byte(R*(float)(__max_brightness)),byte(G*(float)(__max_brightness)),byte(B*(float)(__max_brightness)));

}

void RgbLedMatrixObject::set_matrix_rgb(byte red, byte green, byte blue) {
  byte ctr1;
  byte ctr2;
  for(ctr2 = 0; ctr2 <= __max_row; ctr2++) {
    for(ctr1 = 0; ctr1 <= __max_led; ctr1++) {
      set_led_rgb(ctr2,ctr1,red,green,blue);
    }
  }
}

void RgbLedMatrixObject::set_matrix_hue(int hue) {
  byte ctr1;
  byte ctr2;
  for(ctr2 = 0; ctr2 <= __max_row; ctr2++) {
    for(ctr1 = 0; ctr1 <= __max_led; ctr1++) {
      set_led_hue(ctr2,ctr1,hue);
    }
  }
}

void RgbLedMatrixObject::set_row_rgb(byte row, byte red, byte green, byte blue) {
  byte ctr1;
  for(ctr1 = 0; ctr1 <= __max_led; ctr1++) {
      set_led_rgb(row,ctr1,red,green,blue);
  }
}

void RgbLedMatrixObject::set_column_rgb(byte column, byte red, byte green, byte blue) {
  byte ctr1;
  for(ctr1 = 0; ctr1 <= __max_row; ctr1++) {
      set_led_rgb(ctr1,column,red,green,blue);
  }
}

void RgbLedMatrixObject::set_row_hue(byte row, int hue) {
  byte ctr1;
  for(ctr1 = 0; ctr1 <= __max_led; ctr1++) {
      set_led_hue(row,ctr1,hue);
  }
}

void RgbLedMatrixObject::set_column_hue(byte column, int hue) {
  byte ctr1;
  for(ctr1 = 0; ctr1 <= __max_row; ctr1++) {
      set_led_hue(ctr1,column,hue);
  }
}

void RgbLedMatrixObject::set_row_byte_hue(byte row, byte data_byte, int hue) {
  byte led;
  for(led = 0; led <= __max_led; led++) {
    if( (data_byte>>led)&(B00000001) ) {
      set_led_hue(row,led,hue);
    }
    else {
      set_led_rgb(row,led,0,0,0);
    }
  }
}

void RgbLedMatrixObject::fader(void) {
  byte ctr1;
  byte row;
  byte led;

  for(ctr1 = 0; ctr1 <= __max_brightness; ctr1++) {
    for(row = 0; row <= __max_row; row++) {
      for(led = 0; led <= __max_led; led++) {
        set_led_rgb(row,led,ctr1,ctr1,ctr1);
      }
    }
    delay(__fade_delay);
  }
  for(ctr1 = __max_brightness; (ctr1 >= 0) & (ctr1 != 255); ctr1--) {
    for(row = 0; row <= __max_row; row++) {
      for(led = 0; led <= __max_led; led++) {
        set_led_rgb(row,led,ctr1,ctr1,ctr1);
      }
    }
    delay(__fade_delay);
  }
}

void RgbLedMatrixObject::fader_hue(void) {
  int ctr1;
  byte row;
  byte led;

  for(ctr1 = 0; ctr1 < 360; ctr1=ctr1+3) {
    set_matrix_hue((float)(ctr1));
    delay(__fade_delay);
  }
}

void RgbLedMatrixObject::flip_back(void) {
  // swap frame buffers
  memcpy(RgbLedMatrix.brightness_red_tmp, RgbLedMatrix.brightness_red, sizeof(RgbLedMatrix.brightness_red));
  memcpy(RgbLedMatrix.brightness_red, RgbLedMatrix.brightness_red_back, sizeof(RgbLedMatrix.brightness_red));
  memcpy(RgbLedMatrix.brightness_red_back, RgbLedMatrix.brightness_red_tmp, sizeof(RgbLedMatrix.brightness_red));

  memcpy(RgbLedMatrix.brightness_green_tmp, RgbLedMatrix.brightness_green, sizeof(RgbLedMatrix.brightness_green));
  memcpy(RgbLedMatrix.brightness_green, RgbLedMatrix.brightness_green_back, sizeof(RgbLedMatrix.brightness_green));
  memcpy(RgbLedMatrix.brightness_green_back, RgbLedMatrix.brightness_green_tmp, sizeof(RgbLedMatrix.brightness_green));

  memcpy(RgbLedMatrix.brightness_blue_tmp, RgbLedMatrix.brightness_blue, sizeof(RgbLedMatrix.brightness_blue));
  memcpy(RgbLedMatrix.brightness_blue, RgbLedMatrix.brightness_blue_back, sizeof(RgbLedMatrix.brightness_blue));
  memcpy(RgbLedMatrix.brightness_blue_back, RgbLedMatrix.brightness_blue_tmp, sizeof(RgbLedMatrix.brightness_blue));
}

// global instance
RgbLedMatrixObject RgbLedMatrix;

byte spi_transfer(byte data) {
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
  {
  };
  return SPDR;                    // return the received byte, we don't need that
}

ISR(TIMER2_OVF_vect) {
  RESET_TIMER2; // precharge TIMER2 to maximize ISR time --> max led brightness
  byte cycle;
  for(cycle = 0; cycle < __max_brightness; cycle++) {
    byte led;
    byte row = B00000000;    // row: current source. on when (1)
    byte red;    // current sinker when on (0)
    byte green;  // current sinker when on (0)
    byte blue;   // current sinker when on (0)
    for(row = 0; row <= __max_row; row++) {
      red = B11111111;    // off
      green = B11111111;  // off
      blue = B11111111;   // off
      for(led = 0; led <= __max_led; led++) {
        if(cycle < RgbLedMatrix.brightness_red[row][led]) {
          red &= ~(1<<led);
        }
        if(cycle < RgbLedMatrix.brightness_green[row][led]) {
          green &= ~(1<<led);
        }
        if(cycle < RgbLedMatrix.brightness_blue[row][led]) {
          blue &= ~(1<<led);
        }
      }
      digitalWrite(__spi_latch,LOW);
      spi_transfer(blue);
      spi_transfer(green);
      spi_transfer(red);
      spi_transfer(B00000001<<row);
      digitalWrite(__spi_latch,HIGH);
      digitalWrite(__spi_latch,LOW);
    }
  }
  // turn off all leds when ISR is not running
  // otherwise leds will flash to full brightness when 1111 is set, which
  // stays on outside the ISR !
  digitalWrite(__spi_latch,LOW);
  spi_transfer(B11111111); // blue off
  spi_transfer(B11111111); // green off
  spi_transfer(B11111111); // red off
  spi_transfer(B00000000); // rows off
  digitalWrite(__spi_latch,HIGH);
  digitalWrite(__spi_latch,LOW);
}
