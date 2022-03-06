
/*
 * Read a PWM input signal coming from a digital arduino PIN
 * This is a demonstration that can be later used for connecting
 * to ROS using ROSSerial library
 * 
 * @author: Tiziano Fiorenzani
 */

/// Define input channel pins
static const int CH_THROTTLE_IN = 9;
static const int CH_STEERING_IN = 10;
static const int NUM_CHANNELS = 2;
static const int PWM_DISCONNECT = 700;

/// Create the arrays
float channel[NUM_CHANNELS];

int chId[NUM_CHANNELS] = 
  { CH_THROTTLE_IN
  , CH_STEERING_IN
  };
 
void setup() {
  /// Define pins as INPUT
  for(int i = 0; i < NUM_CHANNELS; i++)
  {
    pinMode(chId[i], INPUT);
  }
  Serial.begin(9600);
}
 
void loop() {
  /// Use the function pulseIn and returns the timinng in us between two highs
  /// see: https://www.arduino.cc/reference/en/language/functions/advanced-io/pulsein/
  int tmpIsDisconnected = 0;
  for(int i = 0; i < NUM_CHANNELS; i++)
  {
    channel[i] = pulseIn(chId[i], HIGH);
    if(channel[i] < PWM_DISCONNECT)
    {
      tmpIsDisconnected = 1;
    }
    Serial.print(chId[i]); 
    Serial.print(": ");  
    Serial.print(channel[i]);
    Serial.print(" - ");  
  }
  Serial.print(" disconnection: ");
  Serial.println(tmpIsDisconnected);
}
