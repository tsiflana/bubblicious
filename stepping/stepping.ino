#include <TimerOne.h>
#include <assert.h>
using namespace std;

#define signed_int_32 long int
#define signed_int_16 short int


unsigned char orientation_based_current = 6;

// ########################Hardware defs################################


#define STEPPER_1_PORT PORTD

#define STEPPER_1_DDR  DDRD


#define STEPPER_1_PWM_MASK 0B00000100
#define STEPPER_1_DIR_MASK 0B01000000

#define MS1_MASK 0B00010000
#define MS2_MASK 0B00100000
#define MS3_MASK 0B00100000

#define LIMIT_0 12
#define LIMIT_1 13
#define IN_PIN  A0

#define SET(REGISTER, VALUE) REGISTER |=  (VALUE)
#define CLR(REGISTER, VALUE) REGISTER &= ~(VALUE)

#define OUTPUT_PIN 11

#define PWM_FROM_VOLTAGE(V)     int (double(255/5)*V)        // Turns voltage into approximate PWM duty (0..255)

//#########################################Stepping globals##################################
typedef unsigned long int u32int;
typedef long int s32int;

const u32int dither_frequency  = 30000;  //hz

const unsigned char microstepping_settings[4] = {
  0, MS1_MASK, MS2_MASK, MS1_MASK | MS2_MASK}; // TODO - add 16th step?

const unsigned char stepper_1_ms_level = 3;
const double mm_per_step = 0.005; // TODO Calibrate


int step_ticker_1 = 0;

long int step_count_1 = 0;

const long int max_acceleration = 2800; //  (steps per seccond per second)
const int max_step_rate = 1200;

const int stepper_timer_period = 1e6/dither_frequency;
const u32int counter_overflow = 0xffffffff;
const s32int multiplier = (counter_overflow/dither_frequency)/10;

s32int stepper_increment_1 = 0, stepper_increment_1_target = 0;

const s32int a = (counter_overflow / dither_frequency);
const s32int b = (s32int)(a) * max_acceleration;


const u32int max_increment_jump_per_dither_step = b / dither_frequency;

double rate_1_target = 0;

const int threshold = 128;

const int max_speed = max_step_rate;
const int hunt_speed = max_step_rate;
const int track_speed = max_step_rate;


int bubble_position = 0;
int new_track = 0;
int position_updated = 0;

// #################################### Steppiing subs #########################################
void set_rate(double step_rate, long int & increment)
{
  if      (step_rate >  max_step_rate*10) step_rate =  max_step_rate*10;
  else if (step_rate < -max_step_rate*10) step_rate = -max_step_rate*10;

  // s32int step_rate_times_10 = (s32int) (step_rate * 10.0);
  increment = multiplier * (s32int) step_rate;
}

void set_rate_1(double step_rate)
{  
  set_rate(step_rate, stepper_increment_1_target);
}


#define DO_ACCELERATION(increment, target) \
{  \
  if (target > increment) \
  {              \
    increment += max_increment_jump_per_dither_step;                \
    if (increment > target) increment = target;                \
              }          \
  else if (target < increment) \
  {                    \
    increment -= max_increment_jump_per_dither_step;                    \
    if (increment < target) increment = target;                    \
              }          \
}

long int counter_1 = 0;

void timer_one_isr()
{
  DO_ACCELERATION(stepper_increment_1, stepper_increment_1_target);

  bool test_1 = (counter_1 > 0);
  counter_1 += stepper_increment_1 << stepper_1_ms_level;
  bool test_2 = (counter_1 > 0);

  if (test_2 & !test_1)
  { 

    if ( stepper_increment_1 < 0)
    {
      STEPPER_1_PORT &= ~STEPPER_1_DIR_MASK;
      step_ticker_1--;
    }
    else
    {

      STEPPER_1_PORT |= STEPPER_1_DIR_MASK;
      step_ticker_1++;
    }


    STEPPER_1_PORT |=  STEPPER_1_PWM_MASK;

    if (1)// (step_ticker_1 % (1 << stepper_1_ms_level)) == 0 )
    {
      if (step_ticker_1 > 0) step_count_1++;
      else                 step_count_1--;
      step_ticker_1 = 0;
    }
    else delayMicroseconds(1);

    STEPPER_1_PORT &= ~STEPPER_1_PWM_MASK;
  }
}

void toggle_light()
{
  static int osc = 1;
  osc = 1-osc;
  digitalWrite(13,osc);
}

enum t_mode
{
  HUNTING,
  TRACKING
};

t_mode mode;

void setup() {

  // Configure outputs

  CLR(STEPPER_1_PORT, STEPPER_1_PWM_MASK | STEPPER_1_DIR_MASK);
  CLR(STEPPER_1_PORT, MS1_MASK | MS2_MASK);


  SET(STEPPER_1_DDR,  STEPPER_1_PWM_MASK | STEPPER_1_DIR_MASK | MS1_MASK | MS2_MASK);

  //to do SETUP MICROSTEPPING


  SET(STEPPER_1_PORT, microstepping_settings[stepper_1_ms_level]);


  pinMode(OUTPUT_PIN, OUTPUT);
  
  TCCR2B = (TCCR2B & 0b11111000) | 0x1; // Set PWM clock divider for current limit pwm to 1.

  Timer1.initialize(stepper_timer_period);
  Timer1.attachInterrupt( timer_one_isr );
  analogWrite(OUTPUT_PIN, 0);


// TODO Flash state light?

  while (!digitalRead(LIMIT_0))
    {}
  while (!digitalRead(LIMIT_1))
    {}

  delay(1000);

  set_rate_1 (-max_speed);
  while (!digitalRead(LIMIT_0))
    {}
  set_rate_1(0);
  step_count_1 = -1000; // Reset zero
  
  mode = HUNTING;



  Serial.begin(57600);
}

void hunt ()
{
  static int dir = 1;
  if (digitalRead(LIMIT_0)) dir = 1;
  if (digitalRead(LIMIT_1)) dir = -1;

  if (dir == 1)
      set_rate_1( hunt_speed);
  else if (dir == -1)
      set_rate_1( -max_speed);
  else 
       set_rate_1(0);
       
  int adc = analogRead(IN_PIN);
  if (dir == 1 && (adc > threshold) )
  {
    bubble_position = step_count_1;

    set_rate_1(0);
    mode = TRACKING;
    new_track = 1;
  }
}

void track()
{
  static int dir = -10;
  if (new_track) 
   {
     dir = -1;
   }
  
  if (dir == 1) 
  {
      set_rate_1(track_speed);
      int adc = analogRead(IN_PIN);
      if (adc > threshold)
        {    
          bubble_position = step_count_1;
          position_updated = 1;
          dir = -1;
        }
  }
  else if (dir == 1)
  {
    set_rate_1(-max_speed);    
    if (step_count_1 < bubble_position - 1000)
      dir = 1;
  }
 
  new_track = 0;
}

void loop() {
  switch (mode)
  {
     case (HUNTING):
       hunt();
       break;
     case (TRACKING):
       track();
       // TODO if (position_updated) do something
       break;
  }
  
  /// TOOD check for timeout
}










