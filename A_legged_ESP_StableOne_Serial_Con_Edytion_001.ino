#define errorAnalysis
//#define ShowCals
  int testDistance = 115;
  //int testDistanceZ = 219;
  int testHeight = 100;
  int testXmove = 0;
  int testYmove = 0;
  int testZmove = 0;
  int staZoff = 0;
int configDelayTime = 10;

double absoluteRadius = 84.853;

 int reverseValue = 1;

 #define Order1 1
 #define Order2 2
 #define Order3 3
 #define Order4 0 

 #define Order1s 0
 #define Order2s 3
 #define Order3s 2
 #define Order4s 1 

  #define P11 0
  #define P12 3
  #define P13 9
  #define P14 10

  #define P21 2
  #define P22 1
  #define P23 5
  #define P24 4

  #define L11 15
  #define L12 14
  #define L13 8
  #define L14 7
  
  #define L21 13
  #define L22 12
  #define L23 6
  #define L24 11

  #define directionL1X  1
  #define directionL1Z -1
  
  #define directionL2X -1
  #define directionL2Z -1 
  
  #define directionP1X  1
  #define directionP1Z  1
  
  #define directionP2X -1
  #define directionP2Z  1

  #define analogPin1 12
  #define analogPin2 12
  #define analogPin3 12
  #define analogPin4 12

  #define digitalPin1 26
  #define digitalPin2 27

  double linearDistance = 60.357;
  
long posP11;/////P////
long posP12;
long posP13;
long posP21;
long posP22;
long posP23;


long posL11;/////L/////
long posL12;
long posL13;
long posL21;
long posL22;
long posL23;

long posP31;
long posP32;
long posP33;/////
long posL31;
long posL32;
long posL33;/////


double Ytest;
double YposT;
double ZposT;

double Xpos = 0;
double Ypos = 0;
double Zpos = 0;
  
  double Leg0 = 56.3, Leg1 = 167, Leg2 = 136;
  double radian = 57.2958;

#include <math.h>

//SoftwareSerial gyroSerial(10, 11);

#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int con = true;
#include <SoftwareSerial.h>// import the serial library
int BluetoothData; // the data given from Computer
SoftwareSerial MeinBluth(10, 11); // RX, TX

double staP11 = 373-150;/////P////
double staP12 = 579;
double staP13 = 691;
double staP21 = 318+150;
double staP22 = 596;
double staP23 = 318;


double staL11 = 455;/////L/////
double staL12 = 566;
double staL13 = 565;
double staL21 = 278;
double staL22 = 325;
double staL23 = 686+260;

double staP31;
double staP32;
double staP33;

double staL31;
double staL32;
double staL33;


int Xp1 = 0; int Yp1 = 500; int Zp1 = 500;
int Xp2 = 0; int Yp2 = 500; int Zp2 = 500;

int Xl1 = 0; int Yl1 = 500; int Zl1 = 500;
int Xl2 = 0; int Yl2 = 500; int Zl2 = 500;

long valP11 = staP11;/////BIS
long valP12 = staP12;    //OFF 30
long valP13 = staP13;//
long valP21 = staP21;   //Change from 45
long valP22 = staP22;//////BIS
long valP23 = staP23;//
long valP31 = staP31;
long valP32 = staP32;    //OFF 30
long valP33 = staP33;//

long valL11 = staL11;
long valL12 = staL12;//////BIS
long valL13 = staL13;//
long valL21 = staL21;
long valL22 = staL22;    //OFF 60
long valL23 = staL23;//BIS
long valL31 = staL31;
long valL32 = staL32;//BIS
long valL33 = staL33;//

int PoseTable[16][2];

// 1-16 : Lista pozycji  [15]
// 1-4  : Lista nóg      [3]
// 1-3  : X , Y , Z      [2]

int testPoseTable[][4][3] = 
{

  {
  {
    140,100,140
  },
  {
    140,100,140
  },
  {
    140,100,140
  },
  {
    140,100,140
  }
},
//                         1
{
  {
    115,120,115
  },
  {
    157,120,42
  },
  {
    60,120,157
  },
  {
    115,120,115
  }
},

//                         2
{
  {
    115,120,115
  },
  {
    157,120,42
  },
  {
    60,120,157
  },
  {
    55,50,55
  }
},

//                         3
{
  {
    115,120,115+40
  },
  {
    157,120,42+40
  },
  {
    60,120,157-40
  },
  {
    55,50,55
  }
},

//                         4
{
  {
    115,120,115
  },
  {
    157,120,42
  },
  {
    60,120,157
  },
  {
    55,50,55
  }
},
//                         5
{
  {
    115+40,120,115
  },
  {
    157-40,120,42
  },
  {
    60+40,120,157
  },
  {
    55,50,55
  }
},
//                         6
{
  {
    115,120,115
  },
  {
    157,120,42
  },
  {
    60,120,157
  },
  {
    55,50,55
  }
},
//                         7
{
  {
    115,120,115
  },
  {
    157,120,42
  },
  {
    60,120,157
  },
  {
    55,50,55
  }
},
//                         8
{
  {
    115,120,115
  },
  {
    157,120,42
  },
  {
    60,120,157
  },
  {
    55,50,55
  }
},
//                         9
{
  {
    115,120,115
  },
  {
    157,120,42
  },
  {
    60,120,157
  },
  {
    55,50,55
  }
},
//                         10
{
  {
    115,120,115
  },
  {
    157,120,42
  },
  {
    60,120,157
  },
  {
    55,50,55
  }
},
//                         11
{
  {
    115,120,115
  },
  {
    157,120,42
  },
  {
    60,120,157
  },
  {
    55,50,55
  }
},
//                         12
{
  {
    115,120,115
  },
  {
    157,120,42
  },
  {
    60,120,157
  },
  {
    55,50,55
  }
},
//                         13
{
  {
    140,170,140
  },
  {
    140,170,140
  },
  {
    140,170,140
  },
  {
    55,50,55
  }
},
//                         14
{
  {
    140,100,140
  },
  {
    140,100,140
  },
  {
    140,100,140
  },
  {
    140,100,140
  }
},
//                         15
{
  {
    140,100,140
  },
  {
    140,100,140
  },
  {
    140,100,140
  },
  {
    140,100,140
  }
},

//                         16
{
  {
    140,100,140
  },
  {
    140,100,140
  },
  {
    140,100,140
  },
  {
    140,100,140
  }
}
};

/* void Prin(){
Serial.print("valL11 ="); Serial.print(valL11); Serial.println(";");
Serial.print("valL12 ="); Serial.print(valL12);Serial.println(";");
Serial.print("valL13 ="); Serial.print(valL13);Serial.println(";");

Serial.print("valL21 ="); Serial.print(valL21);Serial.println(";");
Serial.print("valL22 ="); Serial.print(valL22);Serial.println(";");
Serial.print("valL23 ="); Serial.print(valL23);Serial.println(";");

Serial.print("vaPL11 ="); Serial.print(valP11);Serial.println(";");
Serial.print("vaPL12 ="); Serial.print(valP12);Serial.println(";");
Serial.print("vaPL13 ="); Serial.print(valP13);Serial.println(";");

Serial.print("vaPL21 ="); Serial.print(valP21);Serial.println(";");
Serial.print("vaPL22 ="); Serial.print(valP22);Serial.println(";");
Serial.print("vaPL23 ="); Serial.print(valP23);Serial.println(";");


Serial.println("-----------------------------------------");
Serial.println();
}
*/
void Run2(){
   pwm.setPWM(P11, 0,staP11 );
   pwm.setPWM(P12, 0,staP12 );
   pwm.setPWM(P13, 0,staP13 );

    pwm.setPWM(P21, 0,staP21  );
    pwm.setPWM(P22, 0,staP22  );
    pwm.setPWM(P23, 0,staP23  );

       
    pwm.setPWM(L11, 0,staL11  );
    pwm.setPWM(L12, 0,staL12  );
    pwm.setPWM(L13, 0,staL13  );

    pwm.setPWM(L21 ,0,staL21  );
    pwm.setPWM(L22, 0,staL22  );
    pwm.setPWM(L23, 0,staL23  );

   
}
void Run(){
   pwm.setPWM(P11, 0,posP11 );
   pwm.setPWM(P12, 0,posP12 );
   pwm.setPWM(P13, 0,posP13 );

    pwm.setPWM(P21, 0,posP21  );
    pwm.setPWM(P22, 0,posP22  );
    pwm.setPWM(P23, 0,posP23  );

       
    pwm.setPWM(L11, 0,posL11  );
    pwm.setPWM(L12, 0,posL12  );
    pwm.setPWM(L13, 0,posL13  );

    pwm.setPWM(L21 ,0,posL21  );
    pwm.setPWM(L22, 0,posL22  );
    pwm.setPWM(L23, 0,posL23  );

   
}


class Runner{
    bool Side;
    double pin1, pin2, pin3;
    double St1, St2, St3;
    double LT, L0, L1, L2, L3, L4, L5, L6, L7, X, Y, Z, T1, T2, T3, K1, K2, K3, K4, K5, K6;
    double radian = 57.2958;
    int Lp;
public: 
        Runner(double Pin1, double Pin2, double Pin3, double st1, double st2, double st3, bool side , int lp){
          pin1 =Pin1;
          pin2 =Pin2;
          pin3 = Pin3;
           
          L0 = Leg0;
          L1 = Leg1;
          L2 = Leg2;
           
          St1 = st1;
          St2 = st2;
          St3 = st3;
        
          Side = side;

          Lp = lp;
        }
        void Step (double x, double y, double z)
        {
          if((y > 20) && (z > 17) && (x > 17) && (y < 250) && (z < 350) && (x < 350)){
          bool error = false;
          X = x;
          Y = y;
          Z = z;

          double XZsafeFactor = sqrt(91809-(Y*Y));
          //Serial.print("Safe Factor XZ - ");
          //Serial.println(XZsafeFactor); 

                double sqrtZX = (Z*Z) + (X*X);
                if(sqrt(sqrtZX) > XZsafeFactor){
                  Serial.println("XZ Safe factor ERROR");
                  //Serial.println(sqrtZX);
                  //Serial.println("WTF");
                }
                if(Y > 300){
                  Serial.println("Y Safe factor ERROR");
                  Serial.println("WTF?");
                }
                T1 = atan (Z/X);
                L5 = sqrt(sqrtZX);
                //L5 = LT- L0;
                L4 = sqrt((y*y) + (L5*L5));
                K1 = acos(L5/L4);
                double PowL2L4 , PowL1 , subL2L4L1 ;
                PowL2L4 = ((L2*L2)+(L4*L4));
                PowL1 = L1*L1;
                subL2L4L1 = PowL2L4 - PowL1;
                K2 = acos(subL2L4L1 /(2*L4*L2));
                K3 = K1+K2;
                K5 = acos(((L1*L1)+ (L2*L2)- (L4*L4))/(2*L1*L2));
                K6 = K3;
                T2 = 3.14159-(K5+ K6);
                T3 = 3.14159- K5;
                
                double TD1 = T1*radian*3.333;
                double TD2 = T2*radian*3.333;
                double TD3 = T3*radian*3.333;

                double pos1;
                double pos2;
                double pos3;

                
/*
                if(isnan(pos1) == false || isnan(pos2) == true || isnan(pos3) == true){
                  Serial.print("ERROR LEG"); Serial.print(Lp); Serial.println("IS NAN"); 
                }
*/
                
                

                

                //pos1 = St1 - TD1;    // To jest zamienione na IF
                pos2 = St2 - TD2;
                if( Lp == 2){
                   pos3 = St3 + TD3;
                }else{
                   pos3 = St3 - TD3;
                } 

                if( Lp == 4 || Lp == 1){
                   pos1 = St1 + TD1;
                }else{
                   pos1 = St1 - TD1;
                }
                //pos3 = St3 - TD3;

                
                
                #ifdef errorAnalysis
                
                if(Lp == 1){
                float exportValue1 = float(pos1);
                float exportValue2 = float(pos2);
                float exportValue3 = float(pos3);
                //Serial.println(exportValue1);
                //Serial.println(exportValue2);
                //Serial.println(exportValue3);
                }
                //Serial.print(Lp);
                
                //Serial.println(pos2);
                //Serial.println(pos3);
                if(isnan(pos1) == true){
                  Serial.print("ERROR_LEG 1_"); Serial.print(Lp); Serial.println("_IS NAN");
                  error = true; 
                }
                if(isnan(pos2) == true){
                  Serial.print("ERROR_LEG 2_"); Serial.print(Lp); Serial.println("_IS NAN");
                  error = true;  
                }
                if(isnan(pos3) == true){
                  Serial.print("ERROR_LEG 3_"); Serial.print(Lp); Serial.println("_IS NAN"); 
                  error = true; 
                }
                #endif
                
                #ifdef ShowCals
                Serial.println("");
                Serial.println("-----------NEW CLASS-----------");
                Serial.print("Leg Nr : "); Serial.println(Lp);
                Serial.println("**********************");
                Serial.print("pos1 = "); Serial.println(pos1);
                Serial.print("pos2 = "); Serial.println(pos2);
                Serial.print("pos3 = "); Serial.println(pos3);
                Serial.println("**********************");
                Serial.print("T1 = "); Serial.println(T1*radian);
                Serial.print("T2 = "); Serial.println(T2*radian);
                Serial.print("T3 = "); Serial.println(T3*radian);
                Serial.println("-----------Support-----------");
                Serial.print("sqrt = "); Serial.println(sqrtZX);
                //Serial.print("LT = "); Serial.println(LT);
                Serial.print("TD1 = "); Serial.println(TD1);
                Serial.print("TD2 = "); Serial.println(TD2);
                Serial.print("TD3 = "); Serial.println(TD3);
                Serial.print("PowL2L4 = "); Serial.println(PowL2L4);
                Serial.print("PowL1 = "); Serial.println(PowL1);
                Serial.print("subL2L4L1 = "); Serial.println(subL2L4L1);
                Serial.print("K2 = "); Serial.println(K2*radian);
                Serial.print("K3 = "); Serial.println(K3*radian);
                Serial.print("K5 = "); Serial.println(K5*radian);
                Serial.print("K6 = "); Serial.println(K6*radian);
                
                Serial.println("----------------------");
                Serial.println("");
                #endif

                
                if(error == false){
                pwm.setPWM(pin1, 0,pos1 );
                pwm.setPWM(pin2, 0,pos2 );
                pwm.setPWM(pin3, 0,pos3 );
                }else{
                  Serial.print("ErrorCode = "); Serial.println(error);
                }
          }else{
            Serial.print("DATA_ERROR LEG_NR :   "); Serial.println(Lp);
            Serial.print("X :   "); Serial.println(x);
            Serial.print("Y :   "); Serial.println(y);
            Serial.print("Z :   "); Serial.println(z);
          }
                }
                
        
        };



Runner RunnerP1(P11,P12,P13,staP11,staP12,staP13,1,1);
Runner RunnerP2(P21,P22,P23,staP21,staP22,staP23,1,2);

Runner RunnerL1(L11,L12,L13,staL11,staL12,staL13,0,3);
Runner RunnerL2(L21,L22,L23,staL21,staL22,staL23,0,4);

void moveAll(double XposT ,double YposT ,double ZposT , double XstaT ,double YstaT ,double ZstaT){

RunnerP1.Step(XstaT + XposT  , YstaT + YposT , ZstaT + ZposT);
RunnerP2.Step(XstaT - XposT  , YstaT + YposT , ZstaT + ZposT);

RunnerL1.Step(XstaT + XposT  , YstaT + YposT , ZstaT - ZposT);
RunnerL2.Step(XstaT - XposT  , YstaT + YposT , ZstaT - ZposT);

}
void moveAllOnlyOne1(double XposT ,double YposT ,double ZposT , double XstaT ,double YstaT ,double ZstaT){

RunnerP1.Step(XstaT + XposT  , YstaT + YposT , ZstaT + ZposT);
RunnerP2.Step(XstaT  , YstaT + YposT , ZstaT + ZposT);

RunnerL1.Step(XstaT + XposT  , YstaT + YposT , ZstaT - ZposT);
RunnerL2.Step(XstaT   , YstaT + YposT , ZstaT - ZposT);

}
void moveAllOnlyOne2(double XposT ,double YposT ,double ZposT , double XstaT ,double YstaT ,double ZstaT){

RunnerP1.Step(XstaT  , YstaT + YposT , ZstaT + ZposT);
RunnerP2.Step(XstaT - XposT  , YstaT + YposT , ZstaT + ZposT);

RunnerL1.Step(XstaT   , YstaT + YposT , ZstaT - ZposT);
RunnerL2.Step(XstaT - XposT  , YstaT + YposT , ZstaT - ZposT);

}
  void doCircle(double radius , double velocity , double quanity
  , double XposS , double YposS , double ZposS){
    double Zc;
    double Xc;
    int stepC = 0;

    moveAll(0 , 0 , 0 , XposS , YposS , ZposS);
    delay(400);
    
    while( stepC <= 360 * quanity){
    Zc = sin(stepC / radian) * radius;
    //Serial.print("Zc = ");
    //Serial.println(Zc);
    //Serial.print("Xc = ");
    //Serial.println(Xc);
    Xc = cos(stepC / radian) * radius;
    moveAll(Xc , 0 , Zc , XposS , YposS , ZposS);
    stepC = stepC + 3;
    delay(velocity);
    }
    delay(400);
    moveAll(0 , 0 , 0 , XposS , YposS , ZposS);
  }


  void doCircleForInfinity(double radius , double quality , double quanity
  , double XposS , double YposS , double ZposS){
    double Zc;
    double Xc;

    int nonStable = 1;

    double xMomentary;
    double yMomentary;
    
    int stepC = 0;

    moveAll(0 , 0 , 0 , XposS , YposS , ZposS);
    delay(400);

    //Serial.println("--1--");
   // nonStable = nonStable * -1;
    for(double i = -quality; i <= 0; i++){
       //Serial.println("--");
      xMomentary = (i/quality)*1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      Zc = xMomentary * radius;
      Xc = yMomentary * radius;
      Xc = Xc * -1;

      Serial.print(Zc); Serial.print(" ");
      Serial.println(Xc); Serial.print(" ");

      moveAll(Xc , 0 , Zc , XposS , YposS , ZposS);
      delay(10);
    }

nonStable = nonStable * -1;
    //Serial.println("--2--");
    for(double i = 0; i <= quality; i++){
       //Serial.println("--");
      xMomentary = (i/quality)*1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      Zc = xMomentary * radius;
      Xc = yMomentary * radius;
      Xc = Xc;
      
      Serial.print(Zc); Serial.print(" ");
      Serial.println(Xc); Serial.print(" ");
      moveAll(Xc , 0 , Zc , XposS , YposS , ZposS);
      delay(10);
    }

    nonStable = nonStable * -1;
    //Serial.println("--3--"); 
    for(double i = quality; i > 0; i--){
      //Serial.println("--");
      xMomentary = (i/quality)*1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      Zc = xMomentary * radius;
      Xc = yMomentary * radius;
      Xc = Xc * -1;
      
      Serial.print(Zc); Serial.print(" ");
      Serial.println(Xc); Serial.print(" ");

      moveAll(Xc , 0 , Zc , XposS , YposS , ZposS);
      delay(10);
    }

nonStable = nonStable * -1;
    //Serial.println("--4--");
    for(double i = 0; i > -quality; i--){
       //Serial.println("--");
      xMomentary = (i/quality)*1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      Zc = xMomentary * radius;
      Xc = yMomentary * radius;
      Xc = Xc;
      
      Serial.print(Zc); Serial.print(" ");
      Serial.println(Xc); Serial.print(" ");
      moveAll(Xc , 0 , Zc , XposS , YposS , ZposS);
      delay(10);
    }
    


    
    
    
    delay(400);
    moveAll(0 , 0 , 0 , XposS , YposS , ZposS);
  }

/*
waitTillTimeCome(double xStart,double yStart,double xIncrese,double yIncrese){
  
}
*/



 void doCircleForInfinityDelta(double radius , double quality , double quanity
  , double XposS , double YposS , double ZposS, double deltaMain, double increase){
    double Zc;
    double Xc;

    int nonStable = 1;

    double xMomentary;
    double yMomentary;

    double xMomentaryPrime;
    double yMomentaryPrime;

    bool doOnce;
    
    int stepC = 0;

    double mainDelta = 0;

   // moveAll(0 , 0 , 0 , XposS , YposS , ZposS);
   // delay(400);

    Serial.println("--1--");
   // nonStable = nonStable * -1;





   doOnce = true;
    for(double i = -quality; i <= 0; i=i+increase){
      
      int counter = 0;

      //Serial.print(xMomentary); Serial.print(" ");
      //Serial.println(yMomentary); Serial.print(" ");
      

      if(doOnce == true){
        xMomentaryPrime = ( (i-increase) /quality)*1.45;
        yMomentaryPrime = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      Serial.print(Zc); Serial.print(" ");
      Serial.println(Xc); Serial.print(" ");

      moveAll(Xc , 0 , Zc , XposS , YposS , ZposS);
         
        doOnce = false;
      }
      
      xMomentary = (i/quality)*1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      mainDelta = sqrt(((xMomentary - xMomentaryPrime) * (xMomentary - xMomentaryPrime))+((yMomentary - yMomentaryPrime) * (yMomentary - yMomentaryPrime)));


if(mainDelta > deltaMain){
      Zc = xMomentary * radius;
      Xc = yMomentary * radius;
      Xc = Xc * -1;

      xMomentaryPrime = ( (i) /quality)*1.45;
      yMomentaryPrime = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      Serial.print(Zc); Serial.print(" ");
      Serial.println(Xc); Serial.print(" ");

      moveAll(Xc , 0 , Zc , XposS , YposS , ZposS);
      
}
delay(10);
    }

nonStable = nonStable * -1;
    //Serial.println("--2--");




    
    doOnce = true;
    for(double i = 0; i <= quality; i=i+increase){
      
      int counter = 0;

      if(doOnce == true){
        xMomentaryPrime = ( (i-increase) /quality)*1.45;
        yMomentaryPrime = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));
         
        doOnce = false;
      }
      
       //Serial.println("--");
      xMomentary = (i/quality)*1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      mainDelta = sqrt(((xMomentary - xMomentaryPrime) * (xMomentary - xMomentaryPrime))+((yMomentary - yMomentaryPrime) * (yMomentary - yMomentaryPrime)));


if(mainDelta > deltaMain){
      Zc = xMomentary * radius;
      Xc = yMomentary * radius;
      Xc = Xc;

      xMomentaryPrime = ( (i) /quality)*1.45;
      yMomentaryPrime = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));
      
      Serial.print(Zc); Serial.print(" ");
      Serial.println(Xc); Serial.print(" ");
      moveAll(Xc , 0 , Zc , XposS , YposS , ZposS);
      
    }
    delay(10);
    }
    nonStable = nonStable * -1;
    //Serial.println("--3--"); 






    doOnce = true;
    for(double i = quality; i > 0; i=i-increase){
      
      int counter = 0;

      if(doOnce == true){
        xMomentaryPrime = ( (i+increase) /quality)*1.45;
        yMomentaryPrime = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));
         
        doOnce = false;
      }
     
      //Serial.println("--");
      xMomentary = (i/quality)*1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      mainDelta = sqrt(((xMomentary - xMomentaryPrime) * (xMomentary - xMomentaryPrime))+((yMomentary - yMomentaryPrime) * (yMomentary - yMomentaryPrime)));



if(mainDelta > deltaMain){
      Zc = xMomentary * radius;
      Xc = yMomentary * radius;
      Xc = Xc * -1;

      xMomentaryPrime = ( (i) /quality)*1.45;
      yMomentaryPrime = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));
      
      Serial.print(Zc); Serial.print(" ");
      Serial.println(Xc); Serial.print(" ");

      moveAll(Xc , 0 , Zc , XposS , YposS , ZposS);
      
    }
    delay(10);
    }
nonStable = nonStable * -1;
    //Serial.println("--4--");








    doOnce = true;
    for(double i = 0; i > -quality; i=i-increase){
      
      int counter = 0;

      if(doOnce == true){
        xMomentaryPrime = ( (i+increase) /quality)*1.45;
        yMomentaryPrime = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

        Serial.print(xMomentaryPrime); Serial.print(" ");
        Serial.println(yMomentaryPrime); Serial.print(" ");
         
        doOnce = false;
      }
      
       //Serial.println("--");
      xMomentary = (i/quality)*1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      mainDelta = sqrt(((xMomentary - xMomentaryPrime) * (xMomentary - xMomentaryPrime))+((yMomentary - yMomentaryPrime) * (yMomentary - yMomentaryPrime)));


if(mainDelta > deltaMain){
      Zc = xMomentary * radius;
      Xc = yMomentary * radius;
      Xc = Xc;

      xMomentaryPrime = ( (i) /quality)*1.45;
      yMomentaryPrime = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));
      
      Serial.print(Zc); Serial.print(" ");
      Serial.println(Xc); Serial.print(" ");
      moveAll(Xc , 0 , Zc , XposS , YposS , ZposS);
      
    }
    delay(10);
    }


    
    
    
    delay(400);
    moveAll(0 , 0 , 0 , XposS , YposS , ZposS);
  }
  
  void demo(){
moveAll(Xpos , Ypos , Zpos , 120 , 100 , 120 );
doCircle(75 , 0.5 , 1 , 120 , 100 , 120);
delay(400);
moveAll(Xpos , Ypos , Zpos , 80 , 100 , 80 );
delay(400);
moveAll(Xpos , Ypos , Zpos , 120 , 100 , 120 );
  }


    void demo2(){
moveAll(Xpos , Ypos , Zpos , 120 , 100 , 120 );
doCircleForInfinity(60 , 30 , 1 , 120 , 100 , 120);
delay(400);
moveAll(Xpos , Ypos , Zpos , 80 , 100 , 80 );
delay(400);
moveAll(Xpos , Ypos , Zpos , 120 , 100 , 120 );
  }

  void demo3(){
moveAll(Xpos , Ypos , Zpos , 120 , 100 , 120 );
doCircleForInfinityDelta(60 , 10 , 1 , 120 , 100 , 120,0.01, 0.01);
delay(400);
moveAll(Xpos , Ypos , Zpos , 80 , 100 , 80 );
delay(400);
moveAll(Xpos , Ypos , Zpos , 120 , 100 , 120 );
  }

void cinematicTest(){
  bool loopV = true;
while(loopV == true){
  
  if (Serial.available()){
BluetoothData=Serial.read();
  }
  if(BluetoothData == 'u'){
    testDistance = testDistance + 1;
    BluetoothData= false;
    Serial.print("testDistance = ");
    Serial.println(testDistance);
    moveAll(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  }
    if(BluetoothData == 'j'){
    testDistance = testDistance - 1;
    BluetoothData= false;
    Serial.print("testDistance = ");
    Serial.println(testDistance);
    moveAll(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  }
  if(BluetoothData == 'i'){
    testHeight = testHeight + 1;
    BluetoothData= false;
    Serial.print("testHeight = ");
    Serial.println(testHeight);
    moveAll(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
    
  }
    if(BluetoothData == 'k'){
      testHeight = testHeight - 1;
      BluetoothData= false;
      Serial.print("testHeight = ");
      Serial.println(testHeight);
      moveAll(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff ); 
  }  //////////////////////////////////////////////

  
  if(BluetoothData == 'q'){
    testXmove = testXmove + 1;
    BluetoothData= false;
    Serial.print("testXmove = ");
    Serial.println(testXmove);
    moveAll(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  }
/*
   if(true){
    testXmove = testXmove + 1;
    BluetoothData= false;
    //Serial.print("testXmove = ");
    //Serial.println(testXmove);
    moveAll(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
    delay(40);
  }
*/
  
  if(BluetoothData == 'a'){
    testXmove = testXmove - 1;
    BluetoothData= false;
    Serial.print("testXmove = ");
    Serial.println(testXmove);
    moveAll(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  }
  if(BluetoothData == 'w'){
    testYmove = testYmove + 1;
    BluetoothData= false;
    Serial.print("testYmove = ");
    Serial.println(testYmove);
    moveAll(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  }
  if(BluetoothData == 's'){
    testYmove = testYmove - 1;
    BluetoothData= false;
    Serial.print("testYmove = ");
    Serial.println(testYmove);
    moveAll(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  }
  if(BluetoothData == 'e'){
    testZmove = testZmove + 1;
    BluetoothData= false;
    Serial.print("testZmove = ");
    Serial.println(testZmove);
    moveAll(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  }
  if(BluetoothData == 'd'){
    testZmove = testZmove - 1;
    BluetoothData= false;
    Serial.print("testZmove = ");
    Serial.println(testZmove);
    moveAll(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  }

  
 if(BluetoothData == 'z'){
    testXmove = testXmove - 1;
    BluetoothData= false;
    Serial.print("testXmoveEXP = ");
    Serial.println(testXmove);
    moveAllOnlyOne1(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  }
 if(BluetoothData == 'c'){
    testXmove = testXmove - 1;
    BluetoothData= false;
    Serial.print("testXmoveEXP = ");
    Serial.println(testXmove);
    moveAllOnlyOne2(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  }
 if(BluetoothData == 'x'){
    testXmove = testXmove + 1;
    BluetoothData= false;
    Serial.print("testXmoveEXP = ");
    Serial.println(testXmove);
    moveAllOnlyOne1(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  }
 if(BluetoothData == 'v'){
    testXmove = testXmove + 1;
    BluetoothData= false;
    Serial.print("testXmoveEXP = ");
    Serial.println(testXmove);
    moveAllOnlyOne2(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  }  
 if(BluetoothData == 'b'){
    staZoff = staZoff + 1;
    BluetoothData= false;
    Serial.print("staZoff = ");
    Serial.println(staZoff);
    moveAll(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  } 
 if(BluetoothData == 'n'){
    staZoff = staZoff - 1;
    BluetoothData= false;
    Serial.print("staZoff = ");
    Serial.println(staZoff);
    moveAll(testXmove , testYmove , testZmove ,
    testDistance , testHeight , testDistance - staZoff );
  }   
   if(BluetoothData == 'm'){
    demo();
    BluetoothData= false;
  }  
  if(BluetoothData == 'p'){
    demo2();
    BluetoothData= false;
  }  
   if(BluetoothData == '0'){
    loopV = false;
    BluetoothData= false;
  }   
}
}
/*
void makeMove(  double delayTime,
                double XaP1 ,
                double XaP2 ,
                double XaL1 ,
                double XaL2 ,
  
                double YaP1 ,
                double YaP2 ,
                double YaL1 ,
                double YaL2 ){                
                  RunnerP1.Step(XaP1 , YaP1 , 115);
                  RunnerP2.Step(XaP2 , YaP2 , 115);
                  RunnerL1.Step(XaL1 , YaL1 , 115);
                  RunnerL2.Step(XaL2 , YaL1 , 115);
                  delay(delayTime);

                }
  

void fullSteep(double range , double delayTime){

  double Ylifting = 70;
  double onePiece = 50;
  double XP1 = 115;
  double XP2 = 115;
  double XL1 = 115;
  double XL2 = 115;
  
  double YP1 = 100;
  double YP2 = 100;
  double YL1 = 100;
  double YL2 = 100;
  ///////////////////////Setup
 makeMove(1000,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

 YP1 = YP1 - 40;

 makeMove(1000,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

 XP1 = XP1 + (1.5 * onePiece);

 makeMove(1000,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

 YP1 = YP1 + 40;
 YL1 = YL1 - 40;

 makeMove(1000,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );
 
XL1 = XL1 + (0.5 * onePiece);

  makeMove(1000,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

 YL1 = YL1 + 40;
 YP2 = YP2 - 40;

  makeMove(1000,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

XP2 = XP2 + (0.5 * onePiece);

  makeMove(1000,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

YP2 = YP2 + 40;
YL2 = YL2 - 40;

  makeMove(1000,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

XL2 = XL2 + (1.5 * onePiece);

  makeMove(1000,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

YL2 = YL2 + 40;

  makeMove(1000,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );
///////////////////////////////////
int counter = 0;
while(true){


  
 
  /////////////////////////////////Regular+++Move
  double XP1 = XP1 - onePiece;
  double XP2 = XP2 + onePiece;
  double XL1 = XL1 - onePiece;
  double XL2 = XL2 + onePiece;
  makeMove(500,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );
 ////////////////////////////////////////////////
 
 if(counter == 0){

YP1 = YP1 + 40;
  
  makeMove(300,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

 XP1 = XP1 + (4 * onePiece);

 makeMove(200,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

 YP1 = YP1 + 40;
  
  makeMove(300,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );
 Serial.println("1");
 }

 if(counter == 1){

YL2 = YL2 + 40;
  
  makeMove(300,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

 XL2 = XL2 - (4 * onePiece);

 makeMove(200,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

 YL2 = YL2 + 40;
  
  makeMove(300,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );
  Serial.println("2");
 }

 if(counter == 2){

YP2 = YP2 + 40;
  
  makeMove(300,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

 XP2 = XP2 - (4 * onePiece);

 makeMove(200,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

 YP2 = YP2 + 40;
  
  makeMove(300,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );
  Serial.println("3");
 }

 if(counter == 3){

YL1 = YL1 + 40;
  
  makeMove(300,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

 XL1 = XL1 + (4 * onePiece);

 makeMove(200,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );

 YL1 = YL1 + 40;
  
  makeMove(300,
 XP1 , XP2 , XL1 , XL2 ,
 YP1 , YP2 , YL1 , YL2 );
  Serial.println("4");
 }
counter = counter + 1;

  if(counter == 4){
  counter = 0;
   Serial.println("reset");
 }
}


 delay(100000000);
  
}
*/

/*
void movePreBuild( int FknVelocity){
  int pos[][4][4] = {
         { //Xs  Axes  
          {40 , 190 , 140 , 90},   //L1  normalny posów
          {90 , 40 , 190 , 140},   //P1
          {40 , 90 , 140 , 190},   //L2  odwrócony względem 115 posów
          {90 , 140 , 190 , 40}     //P2  ODWRÓCONE WZGLEDEM 115 AHTUNG !!!!
         },
         {///////////Zs  Axes
          {115 , 115 , 115 , 115},
          {115 , 115 , 115 , 115},
          {115 , 115 , 115 , 115},
          {115 , 115 , 115 , 115}
         }
  };

  
}

void moveFknAss( double deltaX , double deltaY){
  
}



*/
class Machine{   //AKA smoth motion and some other shit

  double maxStepDistance;
  int oneFullStep[15][3][2];

double totalDistance(int bpX1,int bpY1,int bpZ1,
int bpX2,int bpY2,int bpZ2){

           double deltaX = bpX1 - bpX2;
           double deltaY = bpY1 - bpY2;
           double deltaZ = bpZ1 - bpZ2;

           double Td = sqrt( deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);
           return Td;

        //   tdTable[Lp1][Lp2] = Td; 
                }
  /*
void moveExecuter(int toDoPosList[15][3][2]){
  //int timeInterwal = 0;
  int currentPosition[3][2];
  for(int set = 0; set < 16; set++){
    for(int timeInterval = 0; timeInterval < 1001; timeInterval++){
      for(int i = 0; i < 4; i++){
        for(int n = 0; n <3; n++){
          //toDoPosList[set][i][n] = currentPosition[i][n];
          currentPosition[i][n] = map(timeInterval , 0 , 1000 , toDoPosList[set][i][n] , toDoPosList[set+1][i][n]);
                                }
                                }
  
    RunnerP1.Step( currentPosition[0][0] , currentPosition[0][1] , currentPosition[0][2] );
    RunnerP2.Step( currentPosition[1][0] , currentPosition[1][1] , currentPosition[1][2] );

    RunnerL1.Step( currentPosition[2][0] , currentPosition[2][1] , currentPosition[2][2] );
    RunnerL2.Step( currentPosition[3][0] , currentPosition[3][1] , currentPosition[3][2] );
  }
  }

}
*/
void linearMovePlanner(int Xdistance , int Ydistance , int Zdistance){
  
}
/*
bool isLineCrossed( double valX1 ,double valY1 ,double valX2 ,double valY2){
  int safeFactorStability = 0;
 if((valX1 + safeFactorStability < 0) && (valX2 + safeFactorStability < 0)){
    //Do Nuffin
  }else if((valY1 + safeFactorStability < 0) && (valY2 + safeFactorStability < 0)){
    //Do Nuffin
  }else if((valY1 > 0 + safeFactorStability) && (valY2 > 0 + safeFactorStability)){
    //Do Nuffin
  }else if((valX1 > 0) && (valX2 > 0)){
    return 1;
  }else{
     double deltaX;
     double deltaY;
     double m;
     double cutX;
    //Complicated calculations
    deltaX = valX1 - valX2;
    deltaY = valY1 - valY2;

    m = deltaY/deltaX;

    cutX = ((m * valX1) - valY1) / m;
    if(cutX > 0){
    if(valX1 > valX2){
      if((cutX < valX1) && (cutX > valX2)){
        return 1;
      }else{
        return 0;
      }
    }
      if((cutX > valX1) && (cutX < valX2)){
        return 1;
      }else{
        return 0;
      }
    }  
  }
}
  

 
bool isStable(double bX1 ,double bY1 ,double bX2 ,double bY2 ,double bX3 ,double bY3, int sign1, int sign2, int sign3, int sign4, int sign5, int sign6){
  int cutCount = 0;
  int safeFactorStability = 0;

  bX1 = (bX1 + linearDistance)*sign1;
  bY1 = (bY1 + linearDistance)*sign2;

  bX2 = (bX2 + linearDistance)*sign3;
  bY2 = (bY2 + linearDistance)*sign4;

  bX3 = (bX3 + linearDistance)*sign5;
  bY3 = (bY3 + linearDistance)*sign6;


cutCount = 
  isLineCrossed(bX1 , bY1 , bX2 , bY2) +
  isLineCrossed(bX3 , bY3 , bX2 , bY2) +
  isLineCrossed(bX1 , bY1 , bX3 , bY3);

   if((cutCount == 2) && (cutCount != 0)){
    return 0;
   }else{
    return 1;
   }

}
*/
  public:
  

/*
void rotateVerticaly( int inclinationAngle , int azymuthAngle
,double velocity , double XposS , double YposS , double ZposS){
  int stepC = 0;


RunnerP1.Step(XstaT + XposT  , YstaT + YposT , ZstaT + ZposT);
RunnerP2.Step(XstaT - XposT  , YstaT + YposT , ZstaT + ZposT);

RunnerL1.Step(XstaT + XposT  , YstaT + YposT , ZstaT - ZposT);
RunnerL2.Step(XstaT - XposT  , YstaT + YposT , ZstaT - ZposT);
  
}

void Move(double mX,double mY,double mZ){
  
}
*/
};


      void moveExecuter(int toDoPosList[16][4][3] , int speeds , int radiusOfElipse){
  //int timeInterwal = 0;
  double qualityOfSteep = speeds;
  int stableRadius = 5;
  int currentPosition[4][3];
  double deltaPosX;
  double deltaPosZ;
  double sinValue;
  double valMultiplayerLayer;
  for(int set = 1; set < 16 ; set++){
    Serial.print("Set - "); Serial.println(set);
    for(double timeInterval = 1; timeInterval < qualityOfSteep + 1 ; timeInterval++){
      for(int i = 0; i < 4; i++){
        for(int n = 0; n <3; n++){
          //toDoPosList[set][i][n] = currentPosition[i][n];
         // Serial.print("Time - "); Serial.println(timeInterval);
         // Serial.print("i - "); Serial.println(i);
         // Serial.print("n - "); Serial.println(n);
          currentPosition[i][n] = map(timeInterval , 0 , qualityOfSteep , toDoPosList[set][i][n] , toDoPosList[set+1][i][n]);
                                }
                                }
    //Serial.println("LOOP COMPLETED");
int valTesta = -1;

    
        //  valMultiplayerLayer = timeInterval*(1/qualityOfSteep);
        //  sinValue = ((PI/8) * set) + ( PI/8 *valMultiplayerLayer ) + ((3*PI)/2);
        //  deltaPosX = sin(sinValue) * stableRadius;
        //  deltaPosZ = cos(sinValue) * stableRadius;
          //Serial.print("SIN X :  "); Serial.println(sin(sinValue) * stableRadius);
          //Serial.print("COS Z :  "); Serial.println(cos(sinValue) * stableRadius);
          //Serial.print("MUL :  "); Serial.println(valMultiplayerLayer);
          
    RunnerP1.Step( currentPosition[1 + valTesta][1 + valTesta] + deltaPosX , currentPosition[1 + valTesta][2 + valTesta] , currentPosition[1 + valTesta][3 + valTesta] + deltaPosZ );
    RunnerP2.Step( currentPosition[2 + valTesta][1 + valTesta] - deltaPosX , currentPosition[2 + valTesta][2 + valTesta] , currentPosition[2 + valTesta][3 + valTesta] + deltaPosZ );

    RunnerL1.Step( currentPosition[3 + valTesta][1 + valTesta] + deltaPosX , currentPosition[3 + valTesta][2 + valTesta] , currentPosition[3 + valTesta][3 + valTesta] - deltaPosZ );
    RunnerL2.Step( currentPosition[4 + valTesta][1 + valTesta] - deltaPosX , currentPosition[4 + valTesta][2 + valTesta] , currentPosition[4 + valTesta][3 + valTesta] - deltaPosZ );
    yield();

    
  /*
    Serial.print("1.1 - "); Serial.println(currentPosition[1 + valTesta][1 + valTesta]);
    Serial.print("1.2 - "); Serial.println(currentPosition[1 + valTesta][2 + valTesta]);
    Serial.print("1.3 - "); Serial.println(currentPosition[1 + valTesta][3 + valTesta]);

    Serial.print("2.1 - "); Serial.println(currentPosition[2 + valTesta][1 + valTesta]);
    Serial.print("2.2 - "); Serial.println(currentPosition[2 + valTesta][2 + valTesta]);
    Serial.print("2.3 - "); Serial.println(currentPosition[2 + valTesta][3 + valTesta]);

    Serial.print("3.1 - "); Serial.println(currentPosition[3 + valTesta][1 + valTesta]);
    Serial.print("3.2 - "); Serial.println(currentPosition[3 + valTesta][2 + valTesta]);
    Serial.print("3.3 - "); Serial.println(currentPosition[3 + valTesta][3 + valTesta]);

    Serial.print("4.1 - "); Serial.println(currentPosition[4 + valTesta][1 + valTesta]);
    Serial.print("4.2 - "); Serial.println(currentPosition[4 + valTesta][2 + valTesta]);
    Serial.print("4.3 - "); Serial.println(currentPosition[4 + valTesta][3 + valTesta]);

    //

    Serial.print("To Do  1.1 - "); Serial.println(toDoPosList[set][1 + valTesta][1 + valTesta]);
    Serial.print("To Do  1.2 - "); Serial.println(toDoPosList[set][1 + valTesta][2 + valTesta]);
    Serial.print("To Do  1.3 - "); Serial.println(toDoPosList[set][1 + valTesta][3 + valTesta]);

    Serial.print("To Do  2.1 - "); Serial.println(toDoPosList[set][2 + valTesta][1 + valTesta]);
    Serial.print("To Do  2.2 - "); Serial.println(toDoPosList[set][2 + valTesta][2 + valTesta]);
    Serial.print("To Do  2.3 - "); Serial.println(toDoPosList[set][2 + valTesta][3 + valTesta]);

    Serial.print("To Do  3.1 - "); Serial.println(toDoPosList[set][3 + valTesta][1 + valTesta]);
    Serial.print("To Do  3.2 - "); Serial.println(toDoPosList[set][3 + valTesta][2 + valTesta]);
    Serial.print("To Do  3.3 - "); Serial.println(toDoPosList[set][3 + valTesta][3 + valTesta]);

    Serial.print("To Do  4.1 - "); Serial.println(toDoPosList[set][4 + valTesta][1 + valTesta]);
    Serial.print("To Do  4.2 - "); Serial.println(toDoPosList[set][4 + valTesta][2 + valTesta]);
    Serial.print("To Do  4.3 - "); Serial.println(toDoPosList[set][4 + valTesta][3 + valTesta]);
  */
  //delay(10);
    
  }
  //Serial.print("FIN");
  }
  int set = 16;
  Serial.print("Set - "); Serial.println(set);
    for(int timeInterval = 1; timeInterval < qualityOfSteep + 1 ; timeInterval++){
      for(int i = 0; i < 4; i++){
        for(int n = 0; n <3; n++){
          //toDoPosList[set][i][n] = currentPosition[i][n];
         // Serial.print("Time - "); Serial.println(timeInterval);
         // Serial.print("i - "); Serial.println(i);
         // Serial.print("n - "); Serial.println(n);
          currentPosition[i][n] = map(timeInterval , 0 , qualityOfSteep , toDoPosList[16][i][n] , toDoPosList[1][i][n]);
          
                                }
                                }
    //Serial.println("LOOP COMPLETED");
    int valTesta = -1;
       //   valMultiplayerLayer = timeInterval*(1/qualityOfSteep);
       //  sinValue = ((PI/8) * 16) + ( PI/8 *valMultiplayerLayer ) + ((3*PI)/2);
       //   deltaPosX = sin(sinValue) * stableRadius;
       //   deltaPosZ = cos(sinValue) * stableRadius;
          //Serial.print("SIN X :  "); Serial.println(sin(sinValue) * stableRadius);
          //Serial.print("COS Z :  "); Serial.println(cos(sinValue) * stableRadius);
          //Serial.print("MUL :  "); Serial.println(valMultiplayerLayer);
          
    RunnerP1.Step( currentPosition[1 + valTesta][1 + valTesta] + deltaPosX , currentPosition[1 + valTesta][2 + valTesta] , currentPosition[1 + valTesta][3 + valTesta] + deltaPosZ );
    RunnerP2.Step( currentPosition[2 + valTesta][1 + valTesta] - deltaPosX , currentPosition[2 + valTesta][2 + valTesta] , currentPosition[2 + valTesta][3 + valTesta] + deltaPosZ );

    RunnerL1.Step( currentPosition[3 + valTesta][1 + valTesta] + deltaPosX , currentPosition[3 + valTesta][2 + valTesta] , currentPosition[3 + valTesta][3 + valTesta] - deltaPosZ );
    RunnerL2.Step( currentPosition[4 + valTesta][1 + valTesta] - deltaPosX , currentPosition[4 + valTesta][2 + valTesta] , currentPosition[4 + valTesta][3 + valTesta] - deltaPosZ );
    yield();

  //Serial.print("FIN2");
}
      }

//------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------Prime----------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------//



/*
void executePrime(){
 
}
*/




      void moveExecuterPrime(int toDoPosList[16][4][3] , int speeds , int radiusOfElipse){
        
double radius = radiusOfElipse; // 15 - GOOD
double quality;
double quanity;
  double Zc;
  double Xc;

  int nonStable = 1;

  double xMomentary;
  double yMomentary;
  double qualityOfSteep = speeds;
  int stableRadius = 5;
  
  int currentPosition[4][3];
  double deltaPosX;
  double deltaPosZ;
  double sinValue;
  double valMultiplayerLayer;
  double setPrime =0;
  int offSet;
  for(int set = 1; set < 16 ; set++){
    
    if(set < 5){
       offSet = 1;
    }else if(set < 9){
       offSet = 2;
    }else if(set < 13){
       offSet = 3;
    }else{
       offSet = 4;
    }
    //Serial.print("Set - "); Serial.println(set);
     for(double timeInterval = 1; timeInterval < qualityOfSteep + 1 ; timeInterval++){
      for(int i = 0; i < 4; i++){
        for(int n = 0; n <3; n++){
          
          currentPosition[i][n] = map(timeInterval , 0 , qualityOfSteep , toDoPosList[set][i][n] , toDoPosList[set+1][i][n]);
                                }
                                }
                                valMultiplayerLayer = ((setPrime * qualityOfSteep) + timeInterval)/(4*qualityOfSteep);
                                if(offSet == 1){
                                  //Serial.print("SCOPE -   1");
                                   xMomentary = (valMultiplayerLayer * 1.45) - 1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX * -1;

      //Serial.print("MULTI -   "); Serial.println(valMultiplayerLayer);
      //Serial.print(deltaPosX); Serial.print(" ");

    //  Serial.print(deltaPosZ); Serial.print(" ");
    //  Serial.println(deltaPosX); Serial.print(" ");
      

      
      
                                }
                                if(offSet == 2){
                                  //Serial.print("SCOPE -   2");
                                   xMomentary = valMultiplayerLayer * 1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX;
      //Serial.print(setPrime*10); Serial.print(" ");
      
     // Serial.print(deltaPosZ); Serial.print(" ");
     // Serial.println(deltaPosX); Serial.print(" ");
      
                                }
                                if(offSet == 3){
                                  //Serial.print("SCOPE -   3");
                                  xMomentary = 1.45 - (valMultiplayerLayer*1.45);
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX * -1;
      //Serial.print(setPrime*10); Serial.print(" ");
      
   //   Serial.print(deltaPosZ); Serial.print(" ");
    //  Serial.println(deltaPosX); Serial.print(" ");

     
                                }
                                if(offSet == 4){
                                  //Serial.print("SCOPE -   4");
                                  xMomentary = valMultiplayerLayer*(-1.45);
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX;
     // Serial.print(setPrime*10); Serial.print(" ");
      
    //  Serial.print(deltaPosZ); Serial.print(" ");
     // Serial.println(deltaPosX); Serial.print(" ");
      
                                }
                                
int valTesta = -1;

                   
    RunnerP1.Step( currentPosition[1 + valTesta][1 + valTesta] + deltaPosX , currentPosition[1 + valTesta][2 + valTesta] , currentPosition[1 + valTesta][3 + valTesta] + deltaPosZ );
    RunnerP2.Step( currentPosition[2 + valTesta][1 + valTesta] - deltaPosX , currentPosition[2 + valTesta][2 + valTesta] , currentPosition[2 + valTesta][3 + valTesta] + deltaPosZ );

    RunnerL1.Step( currentPosition[3 + valTesta][1 + valTesta] + deltaPosX , currentPosition[3 + valTesta][2 + valTesta] , currentPosition[3 + valTesta][3 + valTesta] - deltaPosZ );
    RunnerL2.Step( currentPosition[4 + valTesta][1 + valTesta] - deltaPosX , currentPosition[4 + valTesta][2 + valTesta] , currentPosition[4 + valTesta][3 + valTesta] - deltaPosZ );
    yield();
  }
  setPrime = setPrime + 1;
    if(setPrime == 4){
      setPrime = 0;
    }
  }
  int set = 16;
  setPrime = 3;
  //Serial.print("Set - "); Serial.println(set);
    for(int timeInterval = 1; timeInterval < qualityOfSteep + 1 ; timeInterval++){
      for(int i = 0; i < 4; i++){
        for(int n = 0; n <3; n++){
          currentPosition[i][n] = map(timeInterval , 0 , qualityOfSteep , toDoPosList[16][i][n] , toDoPosList[1][i][n]);     
                                }
                                }
valMultiplayerLayer = ((3 * qualityOfSteep) + timeInterval)/(4*qualityOfSteep);
                                

    int valTesta = -1;
          //valMultiplayerLayer = 4 * timeInterval * (1/qualityOfSteep);

      xMomentary = valMultiplayerLayer*(-1.45);
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX;
     // Serial.print(setPrime*10); Serial.print(" ");
      
 //     Serial.print(deltaPosZ); Serial.print(" ");
   //   Serial.println(deltaPosX); Serial.print(" ");

          
    RunnerP1.Step( currentPosition[1 + valTesta][1 + valTesta] + deltaPosX , currentPosition[1 + valTesta][2 + valTesta] , currentPosition[1 + valTesta][3 + valTesta] + deltaPosZ );
    RunnerP2.Step( currentPosition[2 + valTesta][1 + valTesta] - deltaPosX , currentPosition[2 + valTesta][2 + valTesta] , currentPosition[2 + valTesta][3 + valTesta] + deltaPosZ );

    RunnerL1.Step( currentPosition[3 + valTesta][1 + valTesta] + deltaPosX , currentPosition[3 + valTesta][2 + valTesta] , currentPosition[3 + valTesta][3 + valTesta] - deltaPosZ );
    RunnerL2.Step( currentPosition[4 + valTesta][1 + valTesta] - deltaPosX , currentPosition[4 + valTesta][2 + valTesta] , currentPosition[4 + valTesta][3 + valTesta] - deltaPosZ );
    yield();
}
    
    
    
      }




  void moveExecuterPrimeReverse(int toDoPosList[16][4][3] , int speeds , int radiusOfElipse){
        
double radius = radiusOfElipse; // 15 - GOOD
double quality;
double quanity;
  double Zc;
  double Xc;

  int nonStable = 1;

  double xMomentary;
  double yMomentary;
  double qualityOfSteep = speeds;
  int stableRadius = 5;
  
  int currentPosition[4][3];
  double deltaPosX;
  double deltaPosZ;
  double sinValue;
  double valMultiplayerLayer;
  double setPrime =0;
  int offSet;
  for(int set = 1; set < 16 ; set++){
    
    if(set < 5){
       offSet = 1;
    }else if(set < 9){
       offSet = 2;
    }else if(set < 13){
       offSet = 3;
    }else{
       offSet = 4;
    }
    //Serial.print("Set - "); Serial.println(set);
     for(double timeInterval = 1; timeInterval < qualityOfSteep + 1 ; timeInterval++){
      for(int i = 0; i < 4; i++){
        for(int n = 0; n <3; n++){
          
          currentPosition[i][n] = map(timeInterval , 0 , qualityOfSteep , toDoPosList[set][i][n] , toDoPosList[set+1][i][n]);
                                }
                                }
                                valMultiplayerLayer = ((setPrime * qualityOfSteep) + timeInterval)/(4*qualityOfSteep);
                                if(offSet == 1){
                                  //Serial.print("SCOPE -   1");
                                   xMomentary = (valMultiplayerLayer * 1.45) - 1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX;

      //Serial.print("MULTI -   "); Serial.println(valMultiplayerLayer);
      //Serial.print(deltaPosX); Serial.print(" ");

    //  Serial.print(deltaPosZ); Serial.print(" ");
    //  Serial.println(deltaPosX); Serial.print(" ");
      

      
      
                                }
                                if(offSet == 2){
                                  //Serial.print("SCOPE -   2");
                                   xMomentary = valMultiplayerLayer * 1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX * -1;
      //Serial.print(setPrime*10); Serial.print(" ");
      
     // Serial.print(deltaPosZ); Serial.print(" ");
     // Serial.println(deltaPosX); Serial.print(" ");
      
                                }
                                if(offSet == 3){
                                  //Serial.print("SCOPE -   3");
                                  xMomentary = 1.45 - (valMultiplayerLayer*1.45);
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX;
      //Serial.print(setPrime*10); Serial.print(" ");
      
   //   Serial.print(deltaPosZ); Serial.print(" ");
    //  Serial.println(deltaPosX); Serial.print(" ");

     
                                }
                                if(offSet == 4){
                                  //Serial.print("SCOPE -   4");
                                  xMomentary = valMultiplayerLayer*(-1.45);
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX * -1;
     // Serial.print(setPrime*10); Serial.print(" ");
      
    //  Serial.print(deltaPosZ); Serial.print(" ");
     // Serial.println(deltaPosX); Serial.print(" ");
      
                                }
                                
int valTesta = -1;

                   
    RunnerP1.Step( currentPosition[1 + valTesta][1 + valTesta] + deltaPosX , currentPosition[1 + valTesta][2 + valTesta] , currentPosition[1 + valTesta][3 + valTesta] + deltaPosZ );
    RunnerP2.Step( currentPosition[2 + valTesta][1 + valTesta] - deltaPosX , currentPosition[2 + valTesta][2 + valTesta] , currentPosition[2 + valTesta][3 + valTesta] + deltaPosZ );

    RunnerL1.Step( currentPosition[3 + valTesta][1 + valTesta] + deltaPosX , currentPosition[3 + valTesta][2 + valTesta] , currentPosition[3 + valTesta][3 + valTesta] - deltaPosZ );
    RunnerL2.Step( currentPosition[4 + valTesta][1 + valTesta] - deltaPosX , currentPosition[4 + valTesta][2 + valTesta] , currentPosition[4 + valTesta][3 + valTesta] - deltaPosZ );
    yield();
  }
  setPrime = setPrime + 1;
    if(setPrime == 4){
      setPrime = 0;
    }
  }
  int set = 16;
  setPrime = 3;
  //Serial.print("Set - "); Serial.println(set);
    for(int timeInterval = 1; timeInterval < qualityOfSteep + 1 ; timeInterval++){
      for(int i = 0; i < 4; i++){
        for(int n = 0; n <3; n++){
          currentPosition[i][n] = map(timeInterval , 0 , qualityOfSteep , toDoPosList[16][i][n] , toDoPosList[1][i][n]);     
                                }
                                }
valMultiplayerLayer = ((3 * qualityOfSteep) + timeInterval)/(4*qualityOfSteep);
                                

    int valTesta = -1;
          //valMultiplayerLayer = 4 * timeInterval * (1/qualityOfSteep);

      xMomentary = valMultiplayerLayer*(-1.45);
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX * -1;
     // Serial.print(setPrime*10); Serial.print(" ");
      
 //     Serial.print(deltaPosZ); Serial.print(" ");
   //   Serial.println(deltaPosX); Serial.print(" ");

          
    RunnerP1.Step( currentPosition[1 + valTesta][1 + valTesta] + deltaPosX , currentPosition[1 + valTesta][2 + valTesta] , currentPosition[1 + valTesta][3 + valTesta] + deltaPosZ );
    RunnerP2.Step( currentPosition[2 + valTesta][1 + valTesta] - deltaPosX , currentPosition[2 + valTesta][2 + valTesta] , currentPosition[2 + valTesta][3 + valTesta] + deltaPosZ );

    RunnerL1.Step( currentPosition[3 + valTesta][1 + valTesta] + deltaPosX , currentPosition[3 + valTesta][2 + valTesta] , currentPosition[3 + valTesta][3 + valTesta] - deltaPosZ );
    RunnerL2.Step( currentPosition[4 + valTesta][1 + valTesta] - deltaPosX , currentPosition[4 + valTesta][2 + valTesta] , currentPosition[4 + valTesta][3 + valTesta] - deltaPosZ );
    yield();
}
    
    
    
      }




 void moveExecuterPrimeReverseCapitalist(int toDoPosList[16][4][3] , int speeds , int radiusOfElipse){
        
double radius = radiusOfElipse; // 15 - GOOD
double quality;
double quanity;
  double Zc;
  double Xc;

  int nonStable = 1;

  double xMomentary;
  double yMomentary;
  double qualityOfSteep = speeds;
  int stableRadius = 40;
  
  int currentPosition[4][3];
  double deltaPosX;
  double deltaPosZ;
  double sinValue;
  double valMultiplayerLayer;
  double setPrime =0;
  int offSet;
  for(int set = 1; set < 16 ; set++){
    
    if(set < 5){
       offSet = 1;
    }else if(set < 9){
       offSet = 2;
    }else if(set < 13){
       offSet = 3;
    }else{
       offSet = 4;
    }
    //Serial.print("Set - "); Serial.println(set);
     for(double timeInterval = 1; timeInterval < qualityOfSteep + 1 ; timeInterval++){
      for(int i = 0; i < 4; i++){
        for(int n = 0; n <3; n++){
          
          currentPosition[i][n] = map(timeInterval , 0 , qualityOfSteep , toDoPosList[set][i][n] , toDoPosList[set+1][i][n]);
                                }
                                }
                                valMultiplayerLayer = ((setPrime * qualityOfSteep) + timeInterval)/(4*qualityOfSteep);
                                if(offSet == 1){
                                  //Serial.print("SCOPE -   1");
                                   xMomentary = (valMultiplayerLayer * 1.45) - 1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX;

      //Serial.print("MULTI -   "); Serial.println(valMultiplayerLayer);
      //Serial.print(deltaPosX); Serial.print(" ");

    //  Serial.print(deltaPosZ); Serial.print(" ");
    //  Serial.println(deltaPosX); Serial.print(" ");
      

      
      
                                }
                                if(offSet == 2){
                                  //Serial.print("SCOPE -   2");
                                   xMomentary = valMultiplayerLayer * 1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX * -1;
      //Serial.print(setPrime*10); Serial.print(" ");
      
     // Serial.print(deltaPosZ); Serial.print(" ");
     // Serial.println(deltaPosX); Serial.print(" ");
      
                                }
                                if(offSet == 3){
                                  //Serial.print("SCOPE -   3");
                                  xMomentary = 1.45 - (valMultiplayerLayer*1.45);
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX;
      //Serial.print(setPrime*10); Serial.print(" ");
      
   //   Serial.print(deltaPosZ); Serial.print(" ");
    //  Serial.println(deltaPosX); Serial.print(" ");

     
                                }
                                if(offSet == 4){
                                  //Serial.print("SCOPE -   4");
                                  xMomentary = valMultiplayerLayer*(-1.45);
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX * -1;
     // Serial.print(setPrime*10); Serial.print(" ");
      
    //  Serial.print(deltaPosZ); Serial.print(" ");
     // Serial.println(deltaPosX); Serial.print(" ");
      
                                }
                                
int valTesta = -1;

                   
   RunnerP1.Step( currentPosition[3 + valTesta][3 + valTesta] + deltaPosX , currentPosition[3 + valTesta][2 + valTesta] , currentPosition[3 + valTesta][1 + valTesta] + deltaPosZ );
    RunnerP2.Step( currentPosition[1 + valTesta][3 + valTesta] + deltaPosX , currentPosition[1 + valTesta][2 + valTesta] , currentPosition[1 + valTesta][1 + valTesta] - deltaPosZ );

    RunnerL1.Step( currentPosition[4 + valTesta][3 + valTesta] - deltaPosX , currentPosition[4 + valTesta][2 + valTesta] , currentPosition[4 + valTesta][1 + valTesta] + deltaPosZ );
    RunnerL2.Step( currentPosition[2 + valTesta][3 + valTesta] - deltaPosX , currentPosition[2 + valTesta][2 + valTesta] , currentPosition[2 + valTesta][1 + valTesta] - deltaPosZ );
    yield();
  }
  setPrime = setPrime + 1;
    if(setPrime == 4){
      setPrime = 0;
    }
  }
  int set = 16;
  setPrime = 3;
  //Serial.print("Set - "); Serial.println(set);
    for(int timeInterval = 1; timeInterval < qualityOfSteep + 1 ; timeInterval++){
      for(int i = 0; i < 4; i++){
        for(int n = 0; n <3; n++){
          currentPosition[i][n] = map(timeInterval , 0 , qualityOfSteep , toDoPosList[16][i][n] , toDoPosList[1][i][n]);     
                                }
                                }
valMultiplayerLayer = ((3 * qualityOfSteep) + timeInterval)/(4*qualityOfSteep);
                                

    int valTesta = -1;
          //valMultiplayerLayer = 4 * timeInterval * (1/qualityOfSteep);

      xMomentary = valMultiplayerLayer*(-1.45);
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX * -1;
     // Serial.print(setPrime*10); Serial.print(" ");
      
 //     Serial.print(deltaPosZ); Serial.print(" ");
   //   Serial.println(deltaPosX); Serial.print(" ");

          
    RunnerP1.Step( currentPosition[3 + valTesta][3 + valTesta] + deltaPosX , currentPosition[3 + valTesta][2 + valTesta] , currentPosition[3 + valTesta][1 + valTesta] + deltaPosZ );
    RunnerP2.Step( currentPosition[1 + valTesta][3 + valTesta] + deltaPosX , currentPosition[1 + valTesta][2 + valTesta] , currentPosition[1 + valTesta][1 + valTesta] - deltaPosZ );

    RunnerL1.Step( currentPosition[4 + valTesta][3 + valTesta] - deltaPosX , currentPosition[4 + valTesta][2 + valTesta] , currentPosition[4 + valTesta][1 + valTesta] + deltaPosZ );
    RunnerL2.Step( currentPosition[2 + valTesta][3 + valTesta] - deltaPosX , currentPosition[2 + valTesta][2 + valTesta] , currentPosition[2 + valTesta][1 + valTesta] - deltaPosZ );
    yield();
}
    
    
    
      }







      void moveExecuterPrimeButComunist(int toDoPosList[16][4][3] , int speeds , int radiusOfElipse){
        
double radius = radiusOfElipse; // 15 - GOOD
double quality;
double quanity;
  double Zc;
  double Xc;

  int nonStable = 1;

  double xMomentary;
  double yMomentary;
  double qualityOfSteep = speeds;
  int stableRadius = 5;
  
  int currentPosition[4][3];
  double deltaPosX;
  double deltaPosZ;
  double sinValue;
  double valMultiplayerLayer;
  double setPrime =0;
  int offSet;
  for(int set = 1; set < 16 ; set++){
    
    if(set < 5){
       offSet = 1;
    }else if(set < 9){
       offSet = 2;
    }else if(set < 13){
       offSet = 3;
    }else{
       offSet = 4;
    }
    //Serial.print("Set - "); Serial.println(set);
     for(double timeInterval = 1; timeInterval < qualityOfSteep + 1 ; timeInterval++){
      for(int i = 0; i < 4; i++){
        for(int n = 0; n <3; n++){
          
          currentPosition[i][n] = map(timeInterval , 0 , qualityOfSteep , toDoPosList[set][i][n] , toDoPosList[set+1][i][n]);
                                }
                                }
                                valMultiplayerLayer = ((setPrime * qualityOfSteep) + timeInterval)/(4*qualityOfSteep);
                                if(offSet == 1){
                                  //Serial.print("SCOPE -   1");
                                   xMomentary = (valMultiplayerLayer * 1.45) - 1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX * -1;

      //Serial.print("MULTI -   "); Serial.println(valMultiplayerLayer);
      //Serial.print(deltaPosX); Serial.print(" ");

    //  Serial.print(deltaPosZ); Serial.print(" ");
    //  Serial.println(deltaPosX); Serial.print(" ");
      

      
      
                                }
                                if(offSet == 2){
                                  //Serial.print("SCOPE -   2");
                                   xMomentary = valMultiplayerLayer * 1.45;
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX;
      //Serial.print(setPrime*10); Serial.print(" ");
      
     // Serial.print(deltaPosZ); Serial.print(" ");
     // Serial.println(deltaPosX); Serial.print(" ");
      
                                }
                                if(offSet == 3){
                                  //Serial.print("SCOPE -   3");
                                  xMomentary = 1.45 - (valMultiplayerLayer*1.45);
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX * -1;
      //Serial.print(setPrime*10); Serial.print(" ");
      
   //   Serial.print(deltaPosZ); Serial.print(" ");
    //  Serial.println(deltaPosX); Serial.print(" ");

     
                                }
                                if(offSet == 4){
                                  //Serial.print("SCOPE -   4");
                                  xMomentary = valMultiplayerLayer*(-1.45);
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX;
     // Serial.print(setPrime*10); Serial.print(" ");
      
    //  Serial.print(deltaPosZ); Serial.print(" ");
     // Serial.println(deltaPosX); Serial.print(" ");
      
                                }
                                
int valTesta = -1;

                   
    RunnerP1.Step( currentPosition[1 + valTesta][3 + valTesta] + deltaPosX , currentPosition[1 + valTesta][2 + valTesta] , currentPosition[1 + valTesta][1 + valTesta] + deltaPosZ );
    RunnerP2.Step( currentPosition[2 + valTesta][3 + valTesta] + deltaPosX , currentPosition[2 + valTesta][2 + valTesta] , currentPosition[2 + valTesta][1 + valTesta] - deltaPosZ );

    RunnerL1.Step( currentPosition[3 + valTesta][3 + valTesta] - deltaPosX , currentPosition[3 + valTesta][2 + valTesta] , currentPosition[3 + valTesta][1 + valTesta] + deltaPosZ );
    RunnerL2.Step( currentPosition[4 + valTesta][3 + valTesta] - deltaPosX , currentPosition[4 + valTesta][2 + valTesta] , currentPosition[4 + valTesta][1 + valTesta] - deltaPosZ );
    yield();
  }
  setPrime = setPrime + 1;
    if(setPrime == 4){
      setPrime = 0;
    }
  }
  int set = 16;
  setPrime = 3;
  //Serial.print("Set - "); Serial.println(set);
    for(int timeInterval = 1; timeInterval < qualityOfSteep + 1 ; timeInterval++){
      for(int i = 0; i < 4; i++){
        for(int n = 0; n <3; n++){
          currentPosition[i][n] = map(timeInterval , 0 , qualityOfSteep , toDoPosList[16][i][n] , toDoPosList[1][i][n]);     
                                }
                                }
valMultiplayerLayer = ((3 * qualityOfSteep) + timeInterval)/(4*qualityOfSteep);
                                

    int valTesta = -1;
          //valMultiplayerLayer = 4 * timeInterval * (1/qualityOfSteep);

      xMomentary = valMultiplayerLayer*(-1.45);
      yMomentary = sqrt(abs(xMomentary*xMomentary*(2.1025-(xMomentary*xMomentary))*0.9));

      deltaPosZ = xMomentary * radius;
      deltaPosX = yMomentary * radius * reverseValue;
      deltaPosX = deltaPosX;
     // Serial.print(setPrime*10); Serial.print(" ");
      
 //     Serial.print(deltaPosZ); Serial.print(" ");
   //   Serial.println(deltaPosX); Serial.print(" ");

          
    RunnerP1.Step( currentPosition[1 + valTesta][3 + valTesta] + deltaPosX , currentPosition[1 + valTesta][2 + valTesta] , currentPosition[1 + valTesta][1 + valTesta] + deltaPosZ );
    RunnerP2.Step( currentPosition[2 + valTesta][3 + valTesta] + deltaPosX , currentPosition[2 + valTesta][2 + valTesta] , currentPosition[2 + valTesta][1 + valTesta] - deltaPosZ );

    RunnerL1.Step( currentPosition[3 + valTesta][3 + valTesta] - deltaPosX , currentPosition[3 + valTesta][2 + valTesta] , currentPosition[3 + valTesta][1 + valTesta] + deltaPosZ );
    RunnerL2.Step( currentPosition[4 + valTesta][3 + valTesta] - deltaPosX , currentPosition[4 + valTesta][2 + valTesta] , currentPosition[4 + valTesta][1 + valTesta] - deltaPosZ );
    yield();
}
    
    
    
      }









bool isLineCrossed( double valX1 ,double valY1 ,double valX2 ,double valY2){
  int safeFactorStability = 0;
 if((valX1 + safeFactorStability < 0) && (valX2 + safeFactorStability < 0)){
    return 0;
    Serial.println("Optimalization exit");
  }else if((valY1 + safeFactorStability < 0) && (valY2 + safeFactorStability < 0)){
    return 0;
    Serial.println("Optimalization exit");
  }else if((valY1 > 0 + safeFactorStability) && (valY2 > 0 + safeFactorStability)){
    return 0;
    Serial.println("Optimalization exit");
  }else if((valX1 > 0) && (valX2 > 0)){
    return 1;
    Serial.println("Optimalization exit");
  }else{
     double deltaX;
     double deltaY;
     double m;
     double cutX;
    //Complicated calculations
    deltaX = valX1 - valX2;
    deltaY = valY1 - valY2;

    m = deltaY/deltaX;

    cutX = ((m * valX1) - valY1) / m;
    if(cutX > 0){
    if(valX1 > valX2){
      if((cutX < valX1) && (cutX > valX2)){
        return 1;
      }else{
        return 0;
      }
    }
      if((cutX > valX1) && (cutX < valX2)){
        return 1;
      }else{
        return 0;
      }
    }  
  }
}
  

 
bool isStable(double bX1 ,double bY1 ,double bX2 ,double bY2 ,double bX3 ,double bY3, int sign1, int sign2, int sign3, int sign4, int sign5, int sign6){
  int cutCount = 0;
  int safeFactorStability = 0;

  bX1 = (bX1 + linearDistance)*sign1;
  bY1 = (bY1 + linearDistance)*sign2;

  bX2 = (bX2 + linearDistance)*sign3;
  bY2 = (bY2 + linearDistance)*sign4;

  bX3 = (bX3 + linearDistance)*sign5;
  bY3 = (bY3 + linearDistance)*sign6;

 Serial.print("Line 1 - ");  Serial.println(isLineCrossed(bX1 , bY1 , bX2 , bY2));
 Serial.print("Line 2 - ");  Serial.println(isLineCrossed(bX3 , bY3 , bX2 , bY2));
 Serial.print("Line 3 - ");  Serial.println(isLineCrossed(bX1 , bY1 , bX3 , bY3));

 
cutCount = 
  isLineCrossed(bX1 , bY1 , bX2 , bY2) +
  isLineCrossed(bX3 , bY3 , bX2 , bY2) +
  isLineCrossed(bX1 , bY1 , bX3 , bY3);

   if((cutCount == 2) && (cutCount != 0)){
    return 0;
   }else{
    return 1;
   }

}


void makeListOneSteep(int xRange, int yRange){
PoseTable[1][2]  = yRange;
PoseTable[2][2]  = yRange;
PoseTable[3][2]  = yRange;
PoseTable[4][2]  = yRange;
PoseTable[5][2]  = yRange;
PoseTable[6][2]  = yRange;
PoseTable[7][2]  = yRange;
PoseTable[8][2]  = yRange;
PoseTable[9][2]  = yRange;
PoseTable[10][2] = yRange;
PoseTable[11][2] = yRange;
PoseTable[12][2] = yRange;
PoseTable[13][2] = ((yRange/2)*(yRange/2)) / (yRange*yRange / xRange) / 2;
PoseTable[14][2] = 0;
PoseTable[15][2] = PoseTable[13][2];
PoseTable[16][2] = yRange;
//Serial.println("makeListOneSteep");
//Serial.println(PoseTable[12][2]);
//Serial.println(PoseTable[16][2]);
//Serial.println("");
  
PoseTable[1][1]  = 0.83 * xRange;
PoseTable[2][1]  = 0.66 * xRange;
PoseTable[3][1]  = 0.5  * xRange;
PoseTable[4][1]  = 0.33 * xRange;
PoseTable[5][1]  = 0.16 * xRange;
PoseTable[6][1]  = 0;
PoseTable[7][1]  = -0.16 * xRange;
PoseTable[8][1]  = -0.33 * xRange;
PoseTable[9][1]  = -0.5  * xRange;
PoseTable[10][1] = -0.66 * xRange;
PoseTable[11][1] = -0.83 * xRange;
PoseTable[12][1] =  -xRange;
PoseTable[13][1] = -0.5  * xRange;
PoseTable[14][1] = 0;
PoseTable[15][1] = 0.5  * xRange;
PoseTable[16][1] = xRange;

}



void makeListOneSteepPrime(int xRange, int yRange){

PoseTable[1][2]  = yRange;
PoseTable[2][2] = yRange;
PoseTable[3][2] = yRange;
PoseTable[4][2] = yRange;
PoseTable[5][2] = ((yRange/2)*(yRange/2)) / (yRange*yRange / xRange) / 2;
PoseTable[6][2] = 0;
PoseTable[7][2] = PoseTable[13][2];
PoseTable[8][2] = yRange;

PoseTable[9][2]  = yRange;
PoseTable[10][2] = yRange;
PoseTable[11][2] = yRange;
PoseTable[12][2] = yRange;
PoseTable[13][2] = ((yRange/2)*(yRange/2)) / (yRange*yRange / xRange) / 2;
PoseTable[14][2] = 0;
PoseTable[15][2] = PoseTable[13][2];
PoseTable[16][2] = yRange;
//Serial.println("makeListOneSteep");
//Serial.println(PoseTable[12][2]);
//Serial.println(PoseTable[16][2]);
//Serial.println("");
  

PoseTable[1][1] = 0.5  * xRange;
PoseTable[2][1] =  0;
PoseTable[3][1] = -0.5  * xRange;
PoseTable[4][1] =  -xRange;
PoseTable[5][1] = -0.5  * xRange;
PoseTable[6][1] = 0;
PoseTable[7][1] = 0.5  * xRange;
PoseTable[8][1] = xRange;

PoseTable[9][1] = 0.5  * xRange;
PoseTable[10][1] =  0;
PoseTable[11][1] = -0.5  * xRange;
PoseTable[12][1] =  -xRange;
PoseTable[13][1] = -0.5  * xRange;
PoseTable[14][1] = 0;
PoseTable[15][1] = 0.5  * xRange;
PoseTable[16][1] = xRange;

}



/*
void makeListOneSteepPrimePlus(int inclination, int yRange){

PoseTable[1][2]  = yRange;
PoseTable[2][2] = yRange;
PoseTable[3][2] = yRange;
PoseTable[4][2] = yRange;
PoseTable[5][2] = ((yRange/2)*(yRange/2)) / (yRange*yRange / xRange) / 2;
PoseTable[6][2] = 0;
PoseTable[7][2] = PoseTable[13][2];
PoseTable[8][2] = yRange;

PoseTable[9][2]  = yRange;
PoseTable[10][2] = yRange;
PoseTable[11][2] = yRange;
PoseTable[12][2] = yRange;
PoseTable[13][2] = ((yRange/2)*(yRange/2)) / (yRange*yRange / xRange) / 2;
PoseTable[14][2] = 0;
PoseTable[15][2] = PoseTable[13][2];
PoseTable[16][2] = yRange;
//Serial.println("makeListOneSteep");
//Serial.println(PoseTable[12][2]);
//Serial.println(PoseTable[16][2]);
//Serial.println("");
  

PoseTable[1][1] = 0.5  * xRange;
PoseTable[2][1] =  0;
PoseTable[3][1] = -0.5  * xRange;
PoseTable[4][1] =  -xRange;
PoseTable[5][1] = -0.5  * xRange;
PoseTable[6][1] = 0;
PoseTable[7][1] = 0.5  * xRange;
PoseTable[8][1] = xRange;

PoseTable[9][1] = 0.5  * xRange;
PoseTable[10][1] =  0;
PoseTable[11][1] = -0.5  * xRange;
PoseTable[12][1] =  -xRange;
PoseTable[13][1] = -0.5  * xRange;
PoseTable[14][1] = 0;
PoseTable[15][1] = 0.5  * xRange;
PoseTable[16][1] = xRange;



PoseTable[1][3] = 0.5  * xRange;
PoseTable[2][3] =  0;
PoseTable[3][3] = -0.5  * xRange;
PoseTable[4][3] =  -xRange;
PoseTable[5][3] = -0.5  * xRange;
PoseTable[6][3] = 0;
PoseTable[7][3] = 0.5  * xRange;
PoseTable[8][3] = xRange;

PoseTable[9][3] = 0.5  * xRange;
PoseTable[10][3] =  0;
PoseTable[11][3] = -0.5  * xRange;
PoseTable[12][3] =  -xRange;
PoseTable[13][3] = -0.5  * xRange;
PoseTable[14][3] = 0;
PoseTable[15][3] = 0.5  * xRange;
PoseTable[16][3] = xRange;

}

*/




void makeOneStep( int xConstant , int yMultiplicator , int zConstant, int Height, int directionOm , int numberOfSpeeds, 
     int balanceRadius){
  int executableList[16][4][3];
  int exampleMove[][3]=
  {
  { PoseTable[1][1] , Height + PoseTable[1][2]*yMultiplicator , zConstant},
  { PoseTable[2][1] , Height + PoseTable[2][2]*yMultiplicator , zConstant},
  { PoseTable[3][1] , Height + PoseTable[3][2]*yMultiplicator , zConstant},
  { PoseTable[4][1] , Height + PoseTable[4][2]*yMultiplicator , zConstant},
  { PoseTable[5][1] , Height + PoseTable[5][2]*yMultiplicator , zConstant},
  { PoseTable[6][1] , Height + PoseTable[6][2]*yMultiplicator , zConstant},
  { PoseTable[7][1] , Height + PoseTable[7][2]*yMultiplicator , zConstant},
  { PoseTable[8][1] , Height + PoseTable[8][2]*yMultiplicator , zConstant},
  { PoseTable[9][1] , Height + PoseTable[9][2]*yMultiplicator , zConstant},
  { PoseTable[10][1] , Height + PoseTable[10][2]*yMultiplicator , zConstant},
  { PoseTable[11][1] , Height + PoseTable[11][2]*yMultiplicator , zConstant},
  { PoseTable[12][1] , Height + PoseTable[12][2]*yMultiplicator , zConstant},
  { PoseTable[13][1] , Height + PoseTable[13][2]*yMultiplicator , zConstant},
  { PoseTable[14][1] , Height + PoseTable[14][2]*yMultiplicator , zConstant},
  { PoseTable[15][1] , Height + PoseTable[15][2]*yMultiplicator , zConstant},
  { PoseTable[16][1] , Height + PoseTable[16][2]*yMultiplicator , zConstant},//

  
  { PoseTable[1][1] , Height + PoseTable[1][2]*yMultiplicator , zConstant},
  { PoseTable[2][1] , Height + PoseTable[2][2]*yMultiplicator , zConstant},
  { PoseTable[3][1] , Height + PoseTable[3][2]*yMultiplicator , zConstant},
  { PoseTable[4][1] , Height + PoseTable[4][2]*yMultiplicator , zConstant},
  { PoseTable[5][1] , Height + PoseTable[5][2]*yMultiplicator , zConstant},
  { PoseTable[6][1] , Height + PoseTable[6][2]*yMultiplicator , zConstant},
  { PoseTable[7][1] , Height + PoseTable[7][2]*yMultiplicator , zConstant},
  { PoseTable[8][1] , Height + PoseTable[8][2]*yMultiplicator , zConstant},
  { PoseTable[9][1] , Height + PoseTable[9][2]*yMultiplicator , zConstant},
  { PoseTable[10][1] , Height + PoseTable[10][2]*yMultiplicator , zConstant},
  { PoseTable[11][1] , Height + PoseTable[11][2]*yMultiplicator , zConstant},
  { PoseTable[12][1] , Height + PoseTable[12][2]*yMultiplicator , zConstant},
  { PoseTable[13][1] , Height + PoseTable[13][2]*yMultiplicator , zConstant},
  { PoseTable[14][1] , Height + PoseTable[14][2]*yMultiplicator , zConstant},
  { PoseTable[15][1] , Height + PoseTable[15][2]*yMultiplicator , zConstant},
  { PoseTable[16][1] , Height + PoseTable[16][2]*yMultiplicator , zConstant}
  };

  int showTime = false;
  //Serial.print("makeOneStep - ");

  if(directionOm == 1){
  for(int stage = 1; stage < 17; stage++){
  
   
      executableList[stage][Order1][0] = (exampleMove [stage +  0][0]  * -1) + xConstant;
      executableList[stage][Order1][1] = exampleMove [stage +  0][1]  *  1;
      executableList[stage][Order1][2] = exampleMove [stage +  0][2];

      
      executableList[stage][Order2][0] =(exampleMove [stage +  4][0]  *  1) + xConstant;
      executableList[stage][Order2][1] = exampleMove [stage +  4][1]  *  1;
      executableList[stage][Order2][2] = exampleMove [stage +  4][2];

      
      executableList[stage][Order3][0] =(exampleMove [stage +  8][0]  * -1) + xConstant;
      executableList[stage][Order3][1] = exampleMove [stage +  8][1]  *  1;
      executableList[stage][Order3][2] = exampleMove [stage +  8][2]  *  1;

      executableList[stage][Order4][0] =(exampleMove [stage + 12][0]  *  1) + xConstant;
      executableList[stage][Order4][1] = exampleMove [stage + 12][1]  *  1;
      executableList[stage][Order4][2] = exampleMove [stage + 12][2]  *  1;
      

      

      
    }
    moveExecuterPrime(executableList , numberOfSpeeds , balanceRadius);
  }


   if(directionOm == 6){
  for(int stage = 1; stage < 9; stage++){
  
   
      executableList[stage][Order1][0] = (exampleMove [stage +  0][0]  * -1) + xConstant;
      executableList[stage][Order1][1] = exampleMove [stage +  0][1]  *  1;
      executableList[stage][Order1][2] = exampleMove [stage +  0][2];

      
      executableList[stage][Order2][0] =(exampleMove [stage +  0][0]  *  1) + xConstant;
      executableList[stage][Order2][1] = exampleMove [stage +  0][1]  *  1;
      executableList[stage][Order2][2] = exampleMove [stage +  0][2];

      
      executableList[stage][Order3][0] =(exampleMove [stage +  4][0]  * -1) + xConstant;
      executableList[stage][Order3][1] = exampleMove [stage +  4][1]  *  1;
      executableList[stage][Order3][2] = exampleMove [stage +  4][2]  *  1;

      executableList[stage][Order4][0] =(exampleMove [stage + 4][0]  *  1) + xConstant;
      executableList[stage][Order4][1] = exampleMove [stage + 4][1]  *  1;
      executableList[stage][Order4][2] = exampleMove [stage + 4][2]  *  1;





      executableList[stage + 8 ][Order1][0] = (exampleMove [stage +  0][0]  * -1) + xConstant;
      executableList[stage + 8 ][Order1][1] = exampleMove [stage +  0][1]  *  1;
      executableList[stage + 8 ][Order1][2] = exampleMove [stage +  0][2];

      
      executableList[stage + 8 ][Order2][0] =(exampleMove [stage +  0][0]  *  1) + xConstant;
      executableList[stage + 8 ][Order2][1] = exampleMove [stage +  0][1]  *  1;
      executableList[stage + 8 ][Order2][2] = exampleMove [stage +  0][2];

      
      executableList[stage + 8 ][Order3][0] =(exampleMove [stage +  4][0]  * -1) + xConstant;
      executableList[stage + 8 ][Order3][1] = exampleMove [stage +  4][1]  *  1;
      executableList[stage + 8 ][Order3][2] = exampleMove [stage +  4][2]  *  1;

      executableList[stage + 8 ][Order4][0] =(exampleMove [stage + 4][0]  *  1) + xConstant;
      executableList[stage + 8 ][Order4][1] = exampleMove [stage + 4][1]  *  1;
      executableList[stage + 8 ][Order4][2] = exampleMove [stage + 4][2]  *  1;
      

      

      
    }
    moveExecuterPrime(executableList , numberOfSpeeds , 0);
  }


  if(directionOm == 2){
    //Serial.println("IM_IN");
  //  xConstant = xConstant + PoseTable[16][1];
  for(int stage = 1; stage < 17; stage++){



      executableList[stage][Order1s][0] = (exampleMove [stage +  0][0]  * -1) + xConstant;
      executableList[stage][Order1s][1] = exampleMove [stage +  0][1]  *  1;
      executableList[stage][Order1s][2] = exampleMove [stage +  0][2];

      
      executableList[stage][Order2s][0] =(exampleMove [stage +  4][0]  *  1) + xConstant;
      executableList[stage][Order2s][1] = exampleMove [stage +  4][1]  *  1;
      executableList[stage][Order2s][2] = exampleMove [stage +  4][2];

      
      executableList[stage][Order3s][0] =(exampleMove [stage +  8][0]  * -1) + xConstant;
      executableList[stage][Order3s][1] = exampleMove [stage +  8][1]  *  1;
      executableList[stage][Order3s][2] = exampleMove [stage +  8][2]  *  1;

      executableList[stage][Order4s][0] =(exampleMove [stage + 12][0]  *  1) + xConstant;
      executableList[stage][Order4s][1] = exampleMove [stage + 12][1]  *  1;
      executableList[stage][Order4s][2] = exampleMove [stage + 12][2]  *  1;
   
   
   /*   executableList[stage][Order1][0] = xConstant - (exampleMove [stage +  0][0]  * -1);
      executableList[stage][Order1][1] = exampleMove [stage +  0][1]  *  1;
      executableList[stage][Order1][2] = exampleMove [stage +  0][2];

      
      executableList[stage][Order2][0] = xConstant - (exampleMove [stage +  4][0]  *  1);
      executableList[stage][Order2][1] = exampleMove [stage +  4][1]  *  1;
      executableList[stage][Order2][2] = exampleMove [stage +  4][2];

      
      executableList[stage][Order3][0] = xConstant - (exampleMove [stage +  8][0]  * -1);
      executableList[stage][Order3][1] = exampleMove [stage +  8][1]  *  1;
      executableList[stage][Order3][2] = exampleMove [stage +  8][2]  *  1;

      executableList[stage][Order4][0] = xConstant - (exampleMove [stage + 12][0]  *  1);
      executableList[stage][Order4][1] = exampleMove [stage + 12][1]  *  1;
      executableList[stage][Order4][2] = exampleMove [stage + 12][2]  *  1; */
      
     

      
    }
    //xConstant = xConstant - PoseTable[16][1];
    moveExecuterPrimeReverse(executableList , numberOfSpeeds , balanceRadius);
  }



  if(directionOm == 3){
  for(int stage = 1; stage < 17; stage++){
  
   
      executableList[stage][Order1][0] = (exampleMove [stage +  0][0]  * -1) + xConstant;
      executableList[stage][Order1][1] = exampleMove [stage +  0][1]  *  1;
      executableList[stage][Order1][2] = exampleMove [stage +  0][2];

      
      executableList[stage][Order2][0] =(exampleMove [stage +  4][0]  *  1) + xConstant;
      executableList[stage][Order2][1] = exampleMove [stage +  4][1]  *  1;
      executableList[stage][Order2][2] = exampleMove [stage +  4][2];

      
      executableList[stage][Order3][0] =(exampleMove [stage +  8][0]  * -1) + xConstant;
      executableList[stage][Order3][1] = exampleMove [stage +  8][1]  *  1;
      executableList[stage][Order3][2] = exampleMove [stage +  8][2]  *  1;

      executableList[stage][Order4][0] =(exampleMove [stage + 12][0]  *  1) + xConstant;
      executableList[stage][Order4][1] = exampleMove [stage + 12][1]  *  1;
      executableList[stage][Order4][2] = exampleMove [stage + 12][2]  *  1;
      

      

      
    }
    moveExecuterPrimeButComunist(executableList , numberOfSpeeds , balanceRadius);
  }



   if(directionOm == 4){
    //Serial.println("IM_IN");
  //  xConstant = xConstant + PoseTable[16][1];
  for(int stage = 1; stage < 17; stage++){



      executableList[stage][Order1s][0] = (exampleMove [stage +  0][0]  * -1) + xConstant;
      executableList[stage][Order1s][1] = exampleMove [stage +  0][1]  *  1;
      executableList[stage][Order1s][2] = exampleMove [stage +  0][2];

      
      executableList[stage][Order2s][0] =(exampleMove [stage +  4][0]  *  1) + xConstant;
      executableList[stage][Order2s][1] = exampleMove [stage +  4][1]  *  1;
      executableList[stage][Order2s][2] = exampleMove [stage +  4][2];

      
      executableList[stage][Order3s][0] =(exampleMove [stage +  8][0]  * -1) + xConstant;
      executableList[stage][Order3s][1] = exampleMove [stage +  8][1]  *  1;
      executableList[stage][Order3s][2] = exampleMove [stage +  8][2]  *  1;

      executableList[stage][Order4s][0] =(exampleMove [stage + 12][0]  *  1) + xConstant;
      executableList[stage][Order4s][1] = exampleMove [stage + 12][1]  *  1;
      executableList[stage][Order4s][2] = exampleMove [stage + 12][2]  *  1;
   
   
   /*   executableList[stage][Order1][0] = xConstant - (exampleMove [stage +  0][0]  * -1);
      executableList[stage][Order1][1] = exampleMove [stage +  0][1]  *  1;
      executableList[stage][Order1][2] = exampleMove [stage +  0][2];

      
      executableList[stage][Order2][0] = xConstant - (exampleMove [stage +  4][0]  *  1);
      executableList[stage][Order2][1] = exampleMove [stage +  4][1]  *  1;
      executableList[stage][Order2][2] = exampleMove [stage +  4][2];

      
      executableList[stage][Order3][0] = xConstant - (exampleMove [stage +  8][0]  * -1);
      executableList[stage][Order3][1] = exampleMove [stage +  8][1]  *  1;
      executableList[stage][Order3][2] = exampleMove [stage +  8][2]  *  1;

      executableList[stage][Order4][0] = xConstant - (exampleMove [stage + 12][0]  *  1);
      executableList[stage][Order4][1] = exampleMove [stage + 12][1]  *  1;
      executableList[stage][Order4][2] = exampleMove [stage + 12][2]  *  1; */
      
     

      
    }
    //xConstant = xConstant - PoseTable[16][1];
    moveExecuterPrimeReverseCapitalist(executableList , numberOfSpeeds , balanceRadius);
  }
  
  //Serial.println("");Serial.println("");
  
  
}








