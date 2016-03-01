#include "FastLED.h"
#define NUM_LEDS 147
CRGB leds[NUM_LEDS];

#define DATA_PIN 5
#define CLOCK_PIN 6
#define MAX_BRIGHTNESS 200
int controlPortA = 7;
int controlPortB = 8;
int state = 0;

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
    //            RandomLED();
    //    fill_solid(leds, NUM_LEDS, CRGB::Blue);
    //    BlueLED();
    //    FastLED.show();
    BlueAndGold();
  }
}
void RandomLED()
{
  for (int led_number = 0; led_number < NUM_LEDS; led_number++)
  {
    leds[led_number].r = random(0, 255);
    leds[led_number].g = random(0, 255);
    leds[led_number].b = random(0, 255);

  }
  FastLED.show();
  delay(1000);
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
    int ledA = (NUM_LEDS / 2) + n;
    int ledB = (NUM_LEDS / 2)  - n;

    leds[ledA].r = 255;
    leds[ledA].g = 255;
    leds[ledA].b = 0;

    leds[ledB].r = 0;
    leds[ledB].g = 0;
    leds[ledB].b = 255;
    FastLED.show();
    delay(10);
  }
  for (int n = 0; n < (NUM_LEDS / 2); n++)
  {
    int ledA = (NUM_LEDS / 2) - n;
    int ledB = (NUM_LEDS / 2) + n;

    Serial.println(ledA);

    leds[ledA].r = 255;
    leds[ledA].g = 255;
    leds[ledA].b = 0;

    leds[ledB].r = 0;
    leds[ledB].g = 0;
    leds[ledB].b = 255;
    FastLED.show();
    delay(10);
  }

}
void ShiftOut()
{
  for (int n = 0 ; n < (NUM_LEDS / 2); n++)
  {
    int ledA = (NUM_LEDS / 2) + n;
    int ledB = (NUM_LEDS / 2) - n;

    leds[ ledA ].r = leds[ ledA - 1 ].r;
    leds[ ledA ].g = leds[ ledA - 1 ].g;
    leds[ ledA ].b = leds[ ledA - 1 ].b;

    leds[ ledB ].r = leds[ ledB + 1 ].r;
    leds[ ledB ].g = leds[ ledB + 1 ].g;
    leds[ ledB ].b = leds[ ledB + 1 ].b;
  }
}
void BlueAndGoldMOD()
{
  int period = 4;
  for ( int i = 0 ; i < period ; i++ )
  {
    leds[NUM_LEDS / 2].r = 0;
    leds[NUM_LEDS / 2].g = 0;
    leds[NUM_LEDS / 2].b = 255;
    ShiftOut();
  }
  for ( int i = 0 ; i < period ; i++ )
  {
    leds[NUM_LEDS / 2].r = 255;
    leds[NUM_LEDS / 2].g = 255;
    leds[NUM_LEDS / 2].b = 0;
    ShiftOut();
  }
}
void DrivePower()
{
  
}
