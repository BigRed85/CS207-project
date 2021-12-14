// CS207 project : software for a flexible continuum are with soft inflatable grasper
// By Dustin Guest
// last updated: 2021-12-13 (added comments and removed redundent code)

// this is the 3rd major iteration of the software for my CS207 project

//the hardware consists of:
//    2 servo motors that control a continuum are by pulling on strings
//    1 modified continuous rotation servo that will compress and decompress a syringe to inflate and deflate the silicone claw
//    All 3 motors are controled through a PWM controller over a I2C bus
//    1 2x16 LCD display controled over the I2C bus


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h> //this library is hard to find on

#define MIN_PULSE_WIDTH       600
#define MAX_PULSE_WIDTH       2400
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50

#define radDelta  0.05;
#define magDelta  0.05;

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();

LiquidCrystal_I2C lcd(0x27,16,2);

int button = 9;

float rad = 0;
float mag = 0;

bool buttonToggle = false;
int buttonState = 1;
int lastButtonState = 1;

void setup() 
{
  pinMode(button, INPUT);

  Serial.begin(9600);

  servos.begin();
  servos.setPWMFreq(FREQUENCY);

  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  
}

void loop() 
{
  int xDirection = joystick(A1);
  int yDirection = joystick(A2);

  String output = ""; //used to display the status on the serial line

  clawControl(); //if button is pressed the claw will actuate
  
  if (yDirection == 1) //use input to change the circular coordinates.
  {
    mag += magDelta;
  }
  else if (yDirection == -1)
  {
    mag -= magDelta;
  }

  if (mag > 1.0)
    mag = 1.0;
  if (mag < -1.0)
    mag = -1.0;

  if (xDirection == 1)
  {
    rad -= radDelta;
  }
  else if (xDirection == -1)
  {
    rad += radDelta;
  }
  

  if (rad > 2*PI)
    rad = rad - 2*PI;
  if (rad < 0)
    rad = rad + 2*PI;

  float posX, posY;

  posX = mag * cos(rad); //change the circular coordinance into X-Y coordinance
  posY = mag * sin(rad);

  posX = posX * 90;
  posY = posY * 90;
    
  int angelX = map(posX, -90, 90, 0, 180);
  int angelY = map(posY, -90, 90, 0, 180);

  servos.setPWM(0, 0, pulseWidth(angelX));
  servos.setPWM(1, 0, pulseWidth(angelY));
  
  
  output = "X: " + String(angelX) + " Y: " + String(angelY) + "  ";
  Serial.println(output); //for debugging
  lcd.setCursor(0,0);
  lcd.print(output);
  
  
}

// this function controls the motor that will inflate and deflate the claw
// and will display a mesage on the LCD display relateing to the state of the motor
void clawControl()
{
   buttonState = digitalRead(button);

  if(lastButtonState == HIGH && buttonState == LOW)
  {
    buttonToggle = !buttonToggle;
    
  }

  lastButtonState = buttonState;
  
  if (buttonState)
  {
    Serial.println("Button Not Pushed!");
    //output += "not Pushed.";
    
    servos.setPWM(2, 0, pulseWidth(90));
    
    lcd.setCursor(0,1);
    lcd.print("Button mode: off");
    
  }
  else
  {
    //output += "Pushed.";
    if (buttonToggle)
    {
      servos.setPWM(2, 0, pulseWidth(0));

      Serial.println("in");

      lcd.setCursor(0,1);
      lcd.print("Button mode: in ");
    }
    else
    {
      servos.setPWM(2, 0, pulseWidth(180));

      Serial.println("out");

      lcd.setCursor(0,1);
      lcd.print("Button mode: out");
    }
    
  }
}

int joystick(int analogPin) //retrun -1, 0 , or 1
{
  int input = analogRead(analogPin);

  if (input < 300)
    return -1;
  if (input > 900)
    return 1;
  return 0;
}

//from the sunfounder wiki
//http://wiki.sunfounder.cc/index.php?title=PCA9685_16_Channel_12_Bit_PWM_Servo_Driver
int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.println(analog_value);
  return analog_value;
}
