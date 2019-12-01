#include <stdint.h>
#include <uart/uart.h>
#include <stdbool.h>
#include <math.h> 

// a pointer to this is a null pointer, but the compiler does not
// know that because "sram" is a linker symbol from sections.lds.
extern uint32_t sram;

#define reg_spictrl (*(volatile uint32_t *)0x02000000)
#define reg_uart_clkdiv (*(volatile uint32_t *)0x02000004)
#define reg_uart_data (*(volatile uint32_t *)0x02000008)
#define clock (*(volatile uint32_t *)0x03000200)
#define pin_test (*(volatile uint32_t *)0x03000900)

uint32_t set_irq_mask(uint32_t mask);
asm(
    ".global set_irq_mask\n"
    "set_irq_mask:\n"
    ".word 0x0605650b\n"
    "ret\n");
/////////////////

// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.

// --------------------------------------------------------
/* -------------------------------
  Other parameters
-----------------------------------*/
#define pointReachedThreshold 0.05
int timeToNewPoint=0;

bool finish = false;

bool countingTimeToEnd = false;
int timeToEnd = 0;


int timeControl=0;

/* -------------------------------
  LEDS
-----------------------------------*/

/* -------------------------------
  PWM
-----------------------------------*/
#define pinPwmIzqF (*(volatile uint32_t *)0x03000500)
#define pinPwmIzqB (*(volatile uint32_t *)0x03000600)
#define pinPwmDerF (*(volatile uint32_t *)0x03000700)
#define pinPwmDerB (*(volatile uint32_t *)0x03000800)
#define maxPWM 150

int pwmValIzq = 0;
int pwmValDer = 0;

/* -------------------------------
  Encoders
-----------------------------------*/

//Counting variables
#define encoderCountIzq (*(volatile uint32_t *)0x03000300)
#define encoderCountDer (*(volatile uint32_t *)0x03000400)
int previousCountIzq = 0;
int previousCountDer = 0;

int timeCountSpeedIzq=0;
int timeCountSpeedDer=0;

void readEncoders()
{
  if (encoderCountDer != 0)
  {
    previousCountDer += encoderCountDer;
    encoderCountDer = 0;
  }
  if (encoderCountIzq != 0)
  {
    previousCountIzq += encoderCountIzq;
    encoderCountIzq = 0;
  }
}

/* -------------------------------
  Control
-----------------------------------*/
#define kr 2
#define ka  1
#define kb 0.04

float vRefDer = 0;
float vRefIzq = 0;
#define errorsLength 30

float errorIzq = 0;
float prevErrIzq = 0;
float errorSignalIzq = 0;

#define kpIzq  10
#define kiIzq 40
#define kdIzq  1

float errorDer = 0;
float prevErrDer = 0;
float errorSignalDer = 0;

#define kpDer  10
#define kiDer 40
#define kdDer 1

float integralErrorIzq;
float integralErrorDer;

long errorTime = 0;

/* -------------------------------
  Position
-----------------------------------*/

float curX = 0;
float curY = 0;
float curTheta = 0;

float v=0;
float w=0;

long timePassedDer = 0;
long timePassedIzq = 0;
float speedIzq = 0;
float speedDer = 0;
float dsIzq = 0;
float dsDer = 0;
float ds = 0;
float dTheta = 0;

float rho = 0;
float alpha = 0;
float beta = 0;
/* -------------------------------
  Car parameters
-----------------------------------*/
#define b  0.2
#define l  0.1
#define wheelRadius  0.034

int micros(){
	return clock;
}

/* -------------------------------
  Gradient function
-----------------------------------*/

float grad[2] = {0,0};
//Function = (x-2)^2+y^2 Le faltan -7
//3d plot (x-2)^2+y^2+10*e^(-((x-0.3)^2+(y-0.5)^2)*2)+12*e^(-((x-1.2)^2+(y+0.7)^2)*2) from -1 to 3
float gradX()
{
  //return 2*(curX+1);
  //return 4*(curX-2) -100*(curX)*exp(-5*(pow(curX,2) + pow(curY - 0.7,2))) -120*(curX)*exp(-5*(pow(curX,2) + pow(curY + 1,2))) -120*(curX - 1.2)*exp(-5*(pow(curX - 1.2,2) + pow(curY + 1,2)));
}

float gradY()
{
  //return 2*(curY);
  return 4*(curY )  -100*(curY - 0.7)*exp(-5*(pow(curX,2) + pow(curY - 0.7,2))) -120*(curY +1)*exp(-5*(pow(curX,2) + pow(curY + 1,2))) -120*(curY + 1)*exp(-5*(pow(curX - 1.2,2) + pow(curY + 1,2)));
}

#define finalX 2
#define finalY 0

bool reachedNewPoint = true;
float newX = 0;
float newY = 0;

/* -------------------------------
  Odometry
-----------------------------------*/
void updateWheelsMovement()
{
  if (previousCountIzq > 10 || previousCountIzq < -10)
  {
    speedIzq = 2*(float)previousCountIzq * M_PI * wheelRadius*1000000 / (442*(micros()-timeCountSpeedIzq));
    timeCountSpeedIzq=micros();
    previousCountIzq = 0;
  }
  else if(micros()-timeCountSpeedIzq>100000){
    speedIzq=0;
    previousCountIzq = 0;
    timeCountSpeedIzq=micros();
  }
  if (previousCountDer > 10 || previousCountDer < -10)
  {
    speedDer = 2*(float)previousCountDer* M_PI * wheelRadius*1000000 / (442*(micros()-timeCountSpeedDer));
    timeCountSpeedDer=micros();
    previousCountDer = 0;
  }
  else if(micros()-timeCountSpeedDer>100000){
    speedDer=0;
    previousCountDer = 0;
    timeCountSpeedDer=micros();
  }
}

void calcNewPosition()
{
  long timeBetweenIzq = micros() - timePassedIzq;
  timePassedIzq = micros();
  dsIzq = timeBetweenIzq * speedIzq / 1000000;

  long timeBetweenDer = micros() - timePassedDer;
  timePassedDer = micros();
  dsDer = timeBetweenDer * speedDer / 1000000;

  ds = (dsIzq + dsDer) / 2;
  dTheta = (dsDer - dsIzq) / b;

  curX += ds * cos(curTheta + dTheta / 2);
  curY += ds * sin(curTheta + dTheta / 2);
  curTheta += dTheta;

  curTheta=curTheta>2*M_PI?curTheta-2*M_PI:curTheta;
  curTheta=curTheta<-2*M_PI?curTheta+2*M_PI:curTheta;
}

void odometry()
{
  updateWheelsMovement();
  calcNewPosition();
}

void calcNewPoint()
{
  newX = grad[0];
  newY = grad[1];
  float norm = sqrt((grad[0] * grad[0]) + (grad[1] * grad[1]));
  if (norm > 0.0001)
  {
    newX = newX / norm;
    newY = newY / norm;
  }

  newX *= 0.1;
  newY *= 0.1;



  newX+=curX;
  newY+=curY;
}

void calcControlVariables()
{
  float dX = newX - curX;
  float dY = newY - curY;

  rho = sqrt((dX * dX) + (dY * dY));

  if (rho < pointReachedThreshold || micros()-timeToNewPoint>1000000)
  {
    timeToNewPoint=micros();
    reachedNewPoint = true;
    return;
  }
  alpha = atan2(dY, dX) - curTheta;
  while (alpha > M_PI)
  {
    alpha -= 2 * M_PI;
  }
  while (alpha <= -M_PI)
  {
    alpha += 2 * M_PI;
  }
  beta = -alpha - curTheta;
  while (beta > M_PI)
  {
    beta -= 2 * M_PI;
  }
  while (beta <= -M_PI)
  {
    beta += 2 * M_PI;
  }
}

void calcRefVelocities()
{
  v = kr * rho;
  w = ka * alpha + kb * beta;

  vRefDer = (v +l * w);
  vRefIzq = v - l * w;

  if(vRefIzq>0.3) vRefIzq=0.3;
  if(vRefIzq<-0.3) vRefIzq=-0.3;

  if(vRefDer>0.3) vRefDer=0.3;
  if(vRefDer<-0.3) vRefDer=-0.3;

}

void controlOdometry()
{
  calcControlVariables();
  if (reachedNewPoint)
    return;
  calcRefVelocities();
}

void control()
{
  
  //printf(curY);
  //print("a");
  errorIzq = vRefIzq*1000 - speedIzq*1000;
  errorDer = vRefDer*1000 - speedDer*1000;

  integralErrorIzq+= errorIzq/10000;

  integralErrorDer+= errorDer/10000;
  float errorIzqDerivative = (errorIzq - prevErrIzq);
  float errorDerDerivative = (errorDer - prevErrDer);
  prevErrIzq = errorIzq;
  prevErrDer = errorDer;
  errorSignalIzq = kpIzq * errorIzq + kiIzq * integralErrorIzq + kdIzq * errorIzqDerivative;
  errorSignalDer = kpDer * errorDer + kiDer * integralErrorDer + kdDer * errorDerDerivative;

  if (errorSignalIzq < 0.1 && errorSignalIzq > -0.1)
  {
    errorSignalIzq = 0;
  }
  if (errorSignalDer < 0.1 && errorSignalDer > -0.1)
  {
    errorSignalDer = 0;
  }

  pwmValIzq = errorSignalIzq;
  pwmValDer = errorSignalDer;

}

void analogWrite(volatile uint32_t * direction, int number){
	(*direction)=number;
}

void moveCar()
{
  volatile uint32_t * curPinPwmIzq = pwmValIzq >= 0 ? ((volatile uint32_t *)0x03000500) : ((volatile uint32_t *)0x03000600);
  volatile uint32_t * curPinPWMOffIzq = pwmValIzq >= 0 ? ((volatile uint32_t *)0x03000600) : ((volatile uint32_t *)0x03000500);
  int curPWMValIzq = pwmValIzq >= 0 ? pwmValIzq : -pwmValIzq;

  if (vRefIzq == 0)
  {
    curPWMValIzq = 0;
  }

  curPWMValIzq = curPWMValIzq > maxPWM ? maxPWM : curPWMValIzq;

  volatile uint32_t * curPinPwmDer = pwmValDer >= 0 ? ((volatile uint32_t *)0x03000700) : ((volatile uint32_t *)0x03000800);
  volatile uint32_t * curPinPWMOffDer = pwmValDer >= 0 ? ((volatile uint32_t *)0x03000800) : ((volatile uint32_t *)0x03000700);
  int curPWMValDer = pwmValDer >= 0 ? pwmValDer : -pwmValDer;

  if (vRefDer == 0)
  {
    curPWMValDer = 0;
  }

  curPWMValDer = curPWMValDer > maxPWM ? maxPWM : curPWMValDer;

  //printf(curPWMValIzq);
  analogWrite(curPinPWMOffIzq, 0);
  analogWrite(curPinPwmIzq, (int) curPWMValIzq);

  analogWrite(curPinPWMOffDer, 0);
  analogWrite(curPinPwmDer, (int) curPWMValDer);
}

float calcDistanceToEnd()
{
  float dX = finalX - curX;
  float dY = finalY - curY;
  float norm = sqrt((dX * dX) + (dY * dY));
  return norm;
}


void stopCar()
{
  volatile uint32_t * t =((volatile uint32_t *)0x03000800);
  analogWrite(t, 0);
  analogWrite(((volatile uint32_t *)0x03000700), 0);

  analogWrite(((volatile uint32_t *)0x03000600), 0);
  analogWrite(((volatile uint32_t *)0x03000500), 0);
}

void setup(){
    /*float clockA = clock;
    reg_leds = 0;*/
    
  pin_test=1;
	integralErrorIzq = 200/kiIzq;
  integralErrorDer =200/kiDer;
	timeToNewPoint=micros();
	timeCountSpeedIzq=micros();
	timeCountSpeedDer=micros();
}
int cycles=0;
void loop(){
  print("a");
  if (finish)
    return;
  readEncoders();

  odometry();

  float trueGradX = -gradX();
  float trueGradY = -gradY();

  grad[0] = trueGradX;
  grad[1] = trueGradY;

  if (reachedNewPoint)
  {
    reachedNewPoint = false;
    calcNewPoint();
  }

  controlOdometry();

  control();
  moveCar();
  if (calcDistanceToEnd() < 0.15)
  {
    //digitalWrite(pinLedFinishing, HIGH);
    if (countingTimeToEnd == false)
    {
      countingTimeToEnd = true;
      //Serial.println("counting end");
    }
    else
    {
      finish = true;
      stopCar();
    }
  }
  else
  {
    countingTimeToEnd = false;
  }
}

void main()
{
	reg_uart_clkdiv = 139;

	print("\n");
	print("  ____  _          ____         ____\n");
	print(" |  _ \\(_) ___ ___/ ___|  ___  / ___|\n");
	print(" | |_) | |/ __/ _ \\___ \\ / _ \\| |\n");
	print(" |  __/| | (_| (_) |__) | (_) | |___\n");
	print(" |_|   |_|\\___\\___/____/ \\___/ \\____|\n");

  setup();
}