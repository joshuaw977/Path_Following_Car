#include <ECE3.h>

uint16_t sensorValues[8];
double mins[8] = {943,875,965,763,920,786,898,1007.4};
double maxs[8] = {1407.6,1625,1557.2,1692.2,1557,1712,1602,1492.6};

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int right_nslp_pin = 11;

const int left_dir_pin=29;  // left wheel direction pin
const int left_pwm_pin=40;  //left wheel speed pin/right pwm pin

const int right_dir_pin = 30;  // right wheel direction 
const int right_pwm_pin = 39;  // right wheel speed pin/right pwm pin

const int LED_rb = 58;

uint16_t bump5 = 28;
uint16_t bump2 = 6;
uint16_t bump1 = 25;
uint16_t bump0 = 24;

int turn_delay = 0;
int post_donut_delay = 150;
int turn_cond;
int stop_cond;
double ratio;
int path = 1;
bool bar_straight = false;
int bar_curve = 0;
bool running = true;
int base_speed = 0;
double fus;
double prev_fus = 0;
double fus_dt;

void setup() 
{
  ECE3_Init();

  pinMode(bump5, INPUT_PULLUP);
  attachInterrupt(bump5, straight_path, FALLING);
  
  pinMode(bump2, INPUT_PULLUP);
  attachInterrupt(bump2, base_speed1, FALLING);
  pinMode(bump1, INPUT_PULLUP);
  attachInterrupt(bump1, base_speed2, FALLING);
  pinMode(bump0, INPUT_PULLUP);
  attachInterrupt(bump0, base_speed3, FALLING);

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  
  // turn on wheel pins
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH); 

  // set wheel direction to forward
  digitalWrite(left_dir_pin,LOW);  // set left wheel direction forward
  digitalWrite(right_dir_pin,LOW);  // set right wheel direction forward 
}

/////////////////////////////////////////////////////////////////////////////

void loop() 
{
  ECE3_read_IR(sensorValues);
  
  if (path == 1)
  {
    donut();
  }
  else if (path == 2)
  {
    donut2();
  }
    
  controller();
}

/////////////////////////////////////////////////////////////////////////////

double Fusion(uint16_t sensorValues[], double mins[], double maxs[])
{
  for (int i=0; i<8; i++)
  {
    sensorValues[i] = sensorValues[i]-mins[i];
    if (mins[i] == 0)
      sensorValues[i]=0;
    else 
      sensorValues[i]=(sensorValues[i]*1000)/maxs[i];
  }

  double x=8*sensorValues[0]+4*sensorValues[1]+2*sensorValues[2]+1*sensorValues[3];
  double y=-8*sensorValues[7]-4*sensorValues[6]-2*sensorValues[5]-1*sensorValues[4];
  double weight = (x+y)/4;

  return weight;
}

/////////////////////////////////////////////////////////////////////////////

void controller()
{
  fus = Fusion(sensorValues, mins, maxs);
  fus_dt = fus - prev_fus;

  // Determine wheel speed change value
  double Kp = (base_speed*0.5)/1919.6;
  double Kd = ratio*Kp;
  double change = Kp*fus + Kd*fus_dt;
  double rws = base_speed - change;
  double lws = base_speed + change;
  
  analogWrite(right_pwm_pin, rws);
  analogWrite(left_pwm_pin, lws);

  prev_fus = fus;
}

/////////////////////////////////////////////////////////////////////////////

void donut()
{
  int count = 0;
  for (int i=0;i<8;i++)
  {
    if (sensorValues[i] >= 2000)
      count++;
  }

  if (count==8)
  {
    bar_curve++;
    if (bar_curve==stop_cond)  // car reaches end of track, stops
    {
      base_speed = 0;
    }
    
    if (bar_curve==turn_cond)  // car reaches halfway through track, turns around
    {
      analogWrite(left_pwm_pin, 0);
      analogWrite(right_pwm_pin, 0);
      delay(100);
      
      analogWrite(left_pwm_pin, base_speed);
      analogWrite(right_pwm_pin, base_speed);
      digitalWrite(left_dir_pin, LOW);
      digitalWrite(right_dir_pin, HIGH);

      delay(turn_delay);
      
      digitalWrite(left_dir_pin, LOW);
      digitalWrite(right_dir_pin, LOW);

      delay(post_donut_delay);
    }
   }
}

//////////////////////////////////////////////////////////////////////////////////

void donut2()
{
  int count = 0;
  for (int i=0;i<8;i++)
  {
    if (sensorValues[i] >= 2000)
      count++;
  }

  if (count == 8)
  {
    if (bar_straight == true)  // car reaches end of track, stops
    {
      base_speed = 0;
    }
    else if (bar_straight == false)  // car reaches halfway through track, turns around
    {
      analogWrite(left_pwm_pin, base_speed);
      analogWrite(right_pwm_pin, base_speed);
      digitalWrite(left_dir_pin, LOW);
      digitalWrite(right_dir_pin, HIGH);

      delay(turn_delay);
      
      digitalWrite(left_dir_pin, LOW);
      digitalWrite(right_dir_pin, LOW);

      delay(post_donut_delay);
      bar_straight = true;
    }
   }
}

//////////////////////////////////////////////////////////////////////////////////

void base_speed1()
{
  base_speed = 90;
  turn_delay = 500;
  ratio = 18;
  stop_cond = 8;
  turn_cond = 4;
}

void base_speed2()
{
  base_speed = 100;
  turn_delay = 500;
  ratio = 18;
  stop_cond = 8;
  turn_cond  = 4;
}

void base_speed3()
{
  base_speed = 140;
  turn_delay =315;
  ratio = 19;
  turn_cond = 4;
  stop_cond = 8;
  post_donut_delay=300;
}

void straight_path()
{
  path = 2;
  digitalWrite(LED_rb, HIGH);
}
