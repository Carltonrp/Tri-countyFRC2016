#include "FastLED.h"
#define NUM_LEDS 147
#define MID_LED NUM_LEDS / 2
CRGB leds[NUM_LEDS];

#define DATA_PIN 5
#define CLOCK_PIN 6
#define MAX_BRIGHTNESS 100
int controlPortA = 7;
int controlPortB = 8;
int state = 0;

unsigned long tick = 0;
int animation = 0;

uint32_t colour;


void setup()
{
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.setBrightness(MAX_BRIGHTNESS);
  pinMode(controlPortA, INPUT);
  pinMode(controlPortB, INPUT);
  Serial.begin(9600);
}
void loop()
{
  int controlA = digitalRead(controlPortA);
  int controlB = digitalRead(controlPortB);
  if (controlA == 1)
  {
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.show();
  }
  else if (controlB == 1)
  {
    fill_solid(leds, NUM_LEDS, CRGB::Blue);
    FastLED.show();
  }
  else
  {
    if ( random( 0 , 127 ) == 1 )
    {
      shuffle();
      Serial.print( "\nanimation:\t" ); Serial.print(animation);
    }
    play( animation );
  }
  tick++;
}

void shuffle()
{
  animation = random( 0 , 21 );
}

void disco()
{
  for (int led_number = 0; led_number < NUM_LEDS; led_number++)
  {
    leds[led_number].r = random(0, 255);
    leds[led_number].g = random(0, 255);
    leds[led_number].b = random(0, 255);
  }
}

void shift_up()
{
  for (int n = NUM_LEDS ; n > 0; n--)
  {
    leds[ n ].r = leds[ n - 1 ].r;
    leds[ n ].g = leds[ n - 1 ].g;
    leds[ n ].b = leds[ n - 1 ].b;
  }
}

void shift_down()
{
  for (int n = 0 ; n < NUM_LEDS; n++)
  {
    leds[ n ].r = leds[ n + 1 ].r;
    leds[ n ].g = leds[ n + 1 ].g;
    leds[ n ].b = leds[ n + 1 ].b;
  }
}

void shift_in()
{
  for ( int n = 0 ; n < NUM_LEDS/2 ; n++ )
  {
    int a = NUM_LEDS/2 + n;
    int b = NUM_LEDS/2 - n - 1;
    
    leds[a].r = leds[ a + 1 ].r;
    leds[a].g = leds[ a + 1 ].g;
    leds[a].b = leds[ a + 1 ].b;
    
    leds[b].r = leds[ b - 1 ].r;
    leds[b].g = leds[ b - 1 ].g;
    leds[b].b = leds[ b - 1 ].b;
  }
}

void shift_out()
{
  for ( int n = MID_LED ; n > 0 ; n-- )
  {
    int a = MID_LED + n;
    int b = MID_LED - n;
    
    leds[a].r = leds[ a - 1 ].r;
    leds[a].g = leds[ a - 1 ].g;
    leds[a].b = leds[ a - 1 ].b;
    
    leds[b].r = leds[ b + 1 ].r;
    leds[b].g = leds[ b + 1 ].g;
    leds[b].b = leds[ b + 1 ].b;
  }
}

void BlueLED()
{
  for (int led_number = 0; led_number < NUM_LEDS; led_number++)
  {
    leds[led_number].r = 0;
    leds[led_number].g = 0;
    leds[led_number].b = 255;

  }
  FastLED.show();
  delay(1000);
}

void BlueandGoldAlt()
{
  if (state == 0)
  {
    for (int led_number = led_number / 2; led_number > 0; led_number--)
    {
      leds[led_number].r = 255;
      leds[led_number].g = 255;
      leds[led_number].b = 0;
      FastLED.show();
    }
    for (int led_number = led_number / 2; led_number < NUM_LEDS; led_number++)
    {
      leds[led_number].r = 0;
      leds[led_number].g = 0;
      leds[led_number].b = 255;
      FastLED.show();
    }
    state = 1;
  }
  FastLED.show();
  delay(100);
  if (state == 1)
  {
    for (int led_number = led_number / 2; led_number < NUM_LEDS; led_number++)
    {
      leds[led_number].r = 255;
      leds[led_number].g = 255;
      leds[led_number].b = 0;
      FastLED.show();
    }
    for (int led_number = led_number / 2; led_number > 0; led_number--)
    {
      leds[led_number].r = 0;
      leds[led_number].g = 0;
      leds[led_number].b = 255;
      FastLED.show();
    }
    state = 0;
  }
  FastLED.show();
  delay(100);
}
void BlueandGold()
{
  for (int n = 0; n < (NUM_LEDS / 2); n++)
  {
    int a = (NUM_LEDS / 2) + n;
    int b = (NUM_LEDS / 2)  - n;

    leds[a].r = 255;
    leds[a].g = 255;
    leds[a].b = 0;

    leds[b].r = 0;
    leds[b].g = 0;
    leds[b].b = 255;
    FastLED.show();
    delay(10);
  }
  for (int n = 0; n < (NUM_LEDS / 2); n++)
  {
    int a = (NUM_LEDS / 2) - n;
    int b = (NUM_LEDS / 2) + n;
    
    Serial.println(a);
    
    leds[a].r = 255;
    leds[a].g = 255;
    leds[a].b = 0;
    
    leds[b].r = 0;
    leds[b].g = 0;
    leds[b].b = 255;
    FastLED.show();
    delay(10);
  }
}

void play( int select )
{
  switch ( select )
  {
    // instant disco
    case 0:
    {
      for ( int n = 0 ; n < NUM_LEDS ; n++ )
      {
        leds[n].r = random( 0 , 255 ) ;
        leds[n].g = random( 0 , 255 ) ;
        leds[n].b = random( 0 , 255 ) ;
      }
      FastLED.show();
    }
    break;
    // instant yellow
    case 1:
    {
      for ( int n = 0 ; n < NUM_LEDS ; n++ )
      {
        leds[n].r = 255 ;
        leds[n].g = 255 ;
        leds[n].b = 0 ;
      }
      FastLED.show();
    }
    break;
    // instant blue
    case 2:
    {
      for ( int n = 0 ; n < NUM_LEDS ; n++ )
      {
        leds[n].r = 0;
        leds[n].g = 0;
        leds[n].b = 255;
      }
      FastLED.show();
    }
    break;
    // blue/yellow gradient wave up
    case 3:
    {
      int period = 16;
      leds[0].r = 127 * ( 1 + cos( (double) M_PI * tick / period ) ) ;
      leds[0].g = 127 * ( 1 + cos( (double) M_PI * tick / period ) ) ;
      leds[0].b = 127 * ( 1 + sin( (double) M_PI * tick / period ) ) ;
      shift_up();
      FastLED.show();
      delay(10);
    }
    break;
    // blue/yellow gradient wave down
    case 4:
    {
      int period = 16;
      leds[NUM_LEDS-1].r = 127 * ( 1 + cos( (double) M_PI * tick / period ) ) ;
      leds[NUM_LEDS-1].g = 127 * ( 1 + cos( (double) M_PI * tick / period ) ) ;
      leds[NUM_LEDS-1].b = 127 * ( 1 + sin( (double) M_PI * tick / period ) ) ;
      shift_down();
      FastLED.show();
      delay(10);
    }
    break;
    // blue/yellow gradient wave out
    case 5:
    {
      int period = 16;
      leds[NUM_LEDS/2].r = 127 * ( 1 + cos( (double) M_PI * tick / period ) ) ;
      leds[NUM_LEDS/2].g = 127 * ( 1 + cos( (double) M_PI * tick / period ) ) ;
      leds[NUM_LEDS/2].b = 127 * ( 1 + sin( (double) M_PI * tick / period ) ) ;
      shift_out();
      FastLED.show();
      delay(10);
    }
    break;
    // blue/yellow gradient wave in
    case 6:
    {
      int period = 16;
      leds[0].r = 127 * ( 1 + cos( (double) M_PI * tick / period ) ) ;
      leds[0].g = 127 * ( 1 + cos( (double) M_PI * tick / period ) ) ;
      leds[0].b = 127 * ( 1 + sin( (double) M_PI * tick / period ) ) ;
      leds[NUM_LEDS-1].r = 127 * ( 1 + cos( (double) M_PI * tick / period ) ) ;
      leds[NUM_LEDS-1].g = 127 * ( 1 + cos( (double) M_PI * tick / period ) ) ;
      leds[NUM_LEDS-1].b = 127 * ( 1 + sin( (double) M_PI * tick / period ) ) ;
      shift_in();
      FastLED.show();
      delay(10);
    }
    break;
    // disco up
    case 7:
    {
      leds[0].r = random( 0 , 255 ) ;
      leds[0].g = random( 0 , 255 ) ;
      leds[0].b = random( 0 , 255 ) ;
      shift_up();
      FastLED.show();
      delay(10);
    }
    break;
    // disco down
    case 8:
    {
      leds[NUM_LEDS].r = random( 0 , 255 ) ;
      leds[NUM_LEDS].g = random( 0 , 255 ) ;
      leds[NUM_LEDS].b = random( 0 , 255 ) ;
      shift_down();
      FastLED.show();
      delay(10);
    }
    break;
    // disco out
    case 9:
    {
      leds[NUM_LEDS/2].r = random( 0 , 255 ) ;
      leds[NUM_LEDS/2].g = random( 0 , 255 ) ;
      leds[NUM_LEDS/2].b = random( 0 , 255 ) ;
      shift_out();
      FastLED.show();
      delay(10);
    }
    break;
    // disco in
    case 10:
    {
      leds[0].r = random( 0 , 255 ) ;
      leds[0].g = random( 0 , 255 ) ;
      leds[0].b = random( 0 , 255 ) ;
      leds[NUM_LEDS-1].r = random( 0 , 255 ) ;
      leds[NUM_LEDS-1].g = random( 0 , 255 ) ;
      leds[NUM_LEDS-1].b = random( 0 , 255 ) ;
      shift_in();
      FastLED.show();
      delay(10);
    }
    break;
    // blue up
    case 11:
    {
      leds[0].r = 0 ;
      leds[0].g = 0 ;
      leds[0].b = 255 ;
      shift_up();
      FastLED.show();
      delay(10);
    }
    // yellow up
    case 12:
    {
      leds[0].r = 255 ;
      leds[0].g = 255 ;
      leds[0].b = 0 ;
      shift_up();
      FastLED.show();
      delay(10);
    }
    break;
    // blue down
    case 13:
    {
      leds[NUM_LEDS-1].r = 0 ;
      leds[NUM_LEDS-1].g = 0 ;
      leds[NUM_LEDS-1].b = 255 ;
      shift_down();
      FastLED.show();
      delay(10);
    }
    break;
    // yellow down
    case 14:
    {
      leds[NUM_LEDS-1].r = 255 ;
      leds[NUM_LEDS-1].g = 255 ;
      leds[NUM_LEDS-1].b = 0 ;
      shift_down();
      FastLED.show();
      delay(10);
    }
    break;
    // blue out
    case 15:
    {
      leds[NUM_LEDS/2].r = 0 ;
      leds[NUM_LEDS/2].g = 0 ;
      leds[NUM_LEDS/2].b = 255 ;
      shift_out();
      FastLED.show();
      delay(10);
    }
    break;
    // yellow out
    case 16:
    {
      leds[NUM_LEDS/2].r = 255 ;
      leds[NUM_LEDS/2].g = 255 ;
      leds[NUM_LEDS/2].b = 0 ;
      shift_out();
      FastLED.show();
      delay(10);
    }
    break;
    // blue in
    case 17:
    {
      leds[0].r = 0 ;
      leds[0].g = 0 ;
      leds[0].b = 255 ;
      leds[NUM_LEDS-1].r = 0 ;
      leds[NUM_LEDS-1].g = 0 ;
      leds[NUM_LEDS-1].b = 255 ;
      shift_in();
      FastLED.show();
      delay(10);
    }
    break;
    // yellow in
    case 18:
    {
      leds[0].r = 255 ;
      leds[0].g = 255 ;
      leds[0].b = 0 ;
      leds[NUM_LEDS-1].r = 255 ;
      leds[NUM_LEDS-1].g = 255 ;
      leds[NUM_LEDS-1].b = 0 ;
      shift_in();
      FastLED.show();
      delay(10);
    }
    break;
    // blue up / yellow down
    case 19:
    {
      leds[0].r = 0 ;
      leds[0].g = 0 ;
      leds[0].b = 255 ;
      leds[NUM_LEDS-1].r = 255 ;
      leds[NUM_LEDS-1].g = 255 ;
      leds[NUM_LEDS-1].b = 0 ;
      shift_in();
      FastLED.show();
      delay(10);
    }
    break;
    // yellow up / blue down
    case 20:
    {
      leds[0].r = 255 ;
      leds[0].g = 255 ;
      leds[0].b = 0 ;
      leds[NUM_LEDS-1].r = 0 ;
      leds[NUM_LEDS-1].g = 0 ;
      leds[NUM_LEDS-1].b = 255 ;
      shift_in();
      FastLED.show();
      delay(10);
    }
    break;
  }
}

void DrivePower()
{
  
}
