 const char HEADER       = 'H';
const char A_TAG    = 'M';
const char B_TAG    = 'X';
const char C_TAG    = 'O';
const char D_TAG    = 'L';
const int  TOTAL_BYTES  = 16  ; // the total bytes in a message


double Azymuth;
double Azymuth1;
 double Increase;

 double maxAngle = 0.645772;

  double Increase1;
   double Increase2;
    double Increase3;
int valZero;
        int valZero1;
 
 int ile = 30;
  void setup() {
   //  lcd.begin(16,2); 
   //  lcd.setCursor(0,0);
   //  lcd.print("Setup");
Serial.begin(115200);
//gyroSerial.begin(19200);
pwm.begin();
pwm.setPWMFreq(100);

//Runner RunnerP1(P11,P12,P13,staP11,staP12,staP13,1,1);
//Runner RunnerP2(P21,P22,P23,staP21,staP22,staP23,1,2);

//Runner RunnerL1(L11,L12,L13,staL11,staL12,staL13,0,3);
//Runner RunnerL2(L21,L22,L23,staL21,staL22,staL23,0,4);

 posP11 = 373-150;/////P////
 posP12 = 579;
 posP13 = 691;
 posP21 = 318+150;
 posP22 = 596;
 posP23 = 318;

int testedSpeeds = 8;

 posL11 = 455;/////L/////
 posL12 = 566;
 posL13 = 565;
 posL21 = 278;
 posL22 = 325;
 posL23 = 686;
  //Run();
  delay(10);
 /*
 int tAngle = 300;
 posP13 = posP13 - 0;
 posP23 = posP23 + tAngle;
 posL13 = posL13 - tAngle;
 posL23 = posL23 - 0;
// lcd.clear();
 */
YposT = 0;
ZposT = 0;

makeListOneSteep(68, 90);
}

void loop() {



  if ( Serial.available() >= TOTAL_BYTES)
  {
     if( Serial.read() == HEADER)
    {
      char tag = Serial.read();
      if(tag == A_TAG)
      {
        //Collect integers
        int valA1 = Serial.read() * 256; 
        valA1 = valA1 + Serial.read();
        int valA2 = Serial.read() * 256;
        valA2 = valA2 + Serial.read();
        int valA3 = Serial.read() * 256;
        valA3 = valA3 + Serial.read();
        int valA4 = Serial.read() * 256;
        valA4 = valA4 + Serial.read();
        int valA5 = Serial.read() * 256;
        valA5 = valA5 + Serial.read();

        Serial.print(valA1); Serial.println(" - valA1");
        Serial.print(valA2); Serial.println(" - valA2");
        Serial.print(valA3); Serial.println(" - valA3");
        Serial.print(valA4); Serial.println(" - valA4");

        

        //Azymuth =  map(valA1 , 0 , 1000 , 0 , 2*PI);
       // Azymuth1 = valA1/1000;
        Azymuth =  valA1 * 2*PI;
        Azymuth = Azymuth / 1000;

        
        //Increase = map(valA2 , 0 , 1000 , -0.523599 , 0.523599);
        

        if((Azymuth >= 0 ) && (Azymuth <  1.5708)){
          Increase1 = valA2-500;
          Increase2 = Increase1 / 500;
          Increase = Increase2 * maxAngle;
  
         valZero  = 1;
         valZero1 = 1;
        Azymuth = Azymuth - 0.78;
        rotate2axed(120, valA3 + 100 ,120, Azymuth , Increase , valZero,  valZero1);

        Serial.print(Azymuth); Serial.println(" - Azymuth 1");
        Serial.print(Increase); Serial.println(" - Increase");
        Serial.print(valZero); Serial.println(" - valZero");
        Serial.print(valZero1); Serial.println(" - valZero1");
  
      }else if((Azymuth >=  1.57 ) && (Azymuth < 3.14)){

        Increase1 = valA2-500;
          Increase2 = Increase1 / 500;
          Increase = Increase2 * maxAngle;
  
        valZero  = -1;
        valZero1 = -1;
        Azymuth = Azymuth - 2.35619;
        rotate2axed(120, valA3 + 100 ,120, Azymuth , Increase , valZero,  valZero1);

        Serial.print(Azymuth); Serial.println(" - Azymuth 2");
        Serial.print(Increase); Serial.println(" - Increase");
        Serial.print(valZero); Serial.println(" - valZero");
        Serial.print(valZero1); Serial.println(" - valZero1");
  
      }else if((Azymuth >= 3.14 ) && (Azymuth < 4.71)){

        Increase1 = valA2-500;
          Increase2 = Increase1 / 500;
          Increase = Increase2 * maxAngle;
  
        valZero  = -1;
        valZero1 =  1;
       Azymuth = Azymuth - 3.92699;
       rotate2axed(120, valA3 + 100 ,120, Azymuth , Increase , valZero,  valZero1);

       Serial.print(Azymuth); Serial.println(" - Azymuth 3");
        Serial.print(Increase); Serial.println(" - Increase");
        Serial.print(valZero); Serial.println(" - valZero");
        Serial.print(valZero1); Serial.println(" - valZero1");
  
      }else if((Azymuth >= 4.71 ) && (Azymuth <= 6.28)){

       Increase1 = valA2-500;
          Increase2 = Increase1 / 500;
          Increase = Increase2 * maxAngle;
  
      valZero  =  1;
      valZero1 = -1;
      Azymuth = Azymuth - 5.49;
      rotate2axed(120, valA3 + 100 ,120, Azymuth , Increase , valZero,  valZero1);

      Serial.print(Azymuth); Serial.println(" - Azymuth 4");
        Serial.print(Increase); Serial.println(" - Increase");
        Serial.print(valZero); Serial.println(" - valZero");
        Serial.print(valZero1); Serial.println(" - valZero1");
  
      }else{
        Serial.println("INVALID_ANGLE");
      }

        

        
        
      }else if(tag == B_TAG)
      {
        double degreOf;
        //Collect integers
        int valB1 = Serial.read() * 256; 
        valB1 = valB1 + Serial.read();
        int valB2 = Serial.read() * 256;
        valB2 = valB2 + Serial.read();
        int valB3 = Serial.read() * 256;
        valB3 = valB3 + Serial.read();
        int valB4 = Serial.read() * 256;
        valB4 = valB4 + Serial.read();
        int valB5 = Serial.read() * 256;
        valB5 = valB5 + Serial.read();

        
        degreOf = degreOf - 500;
        degreOf = valB1 * 90;
        degreOf = degreOf / 500;

        Serial.println(degreOf);

        screwUrSelfWoof(120 , 100 , 120 ,degreOf , valB3);

      }else if(tag == C_TAG)
      {
        //makeOneStep( 100, 1, 120, 60 ,1 , 8 , 8);
        //makeOneStep( 100, 1, 120, 60 ,6 , 8 , 8);
        //Collect integers
        int valC1 = Serial.read() * 256; 
        valC1 = valC1 + Serial.read();
        int valC2 = Serial.read() * 256;
        valC2 = valC2 + Serial.read();
        int valC3 = Serial.read() * 256;
        valC3 = valC3 + Serial.read();
        int valC4 = Serial.read() * 256;
        valC4 = valC4 + Serial.read();
        int valC5 = Serial.read() * 256;
        valC5 = valC5 + Serial.read();

        if(valC5 == 1){
          makeOneStep( 100, 1, 120, 60 ,1 , 8 , 8);
          //makeOneStep( 100, 1, 120, 60 ,6 , 8 , 8);
        }
        if(valC5 == 2){
          //makeOneStep( 100, 1, 120, 60 ,1 , 8 , 8);
          makeOneStep( 100, 1, 120, 60 ,2 , 8 , 8);
        }








        
      }else if(tag == D_TAG)
      {
        //Collect integers
        int valD1 = Serial.read() * 256; 
        valD1 = valD1 + Serial.read();
        int valD2 = Serial.read() * 256;
        valD2 = valD2 + Serial.read();
        int valD3 = Serial.read() * 256;
        valD3 = valD3 + Serial.read();
        int valD4 = Serial.read() * 256;
        valD4 = valD4 + Serial.read();
        int valD5 = Serial.read() * 256;
        valD5 = valD5 + Serial.read();

        serialFlush();

        Serial.println(valD4);
           if(valD4 == 40){ 
                 demo();
                 //serialFlush();
           }
           if(valD4 == 50){ 
                 demo2();
                 //serialFlush();
           }
           if(valD4 == 60){ 
                 demo3();
                 //serialFlush();
           }

           
          if(valD4 == 30){

            for(double i = 0; i < 6.28319; i = i + 0.0872665){
              Serial.println(i);
              Azymuth = i;

              
               if((Azymuth >= 0 ) && (Azymuth <  1.5708)){
          Increase1 = valD2-500;
          Increase2 = Increase1 / 500;
          Increase = Increase2 * maxAngle;
  
         valZero  = 1;
         valZero1 = 1;
        Azymuth = Azymuth - 0.78;
        rotate2axed(120, valD3 + 100 ,120, Azymuth , Increase , valZero,  valZero1);
            /*
        Serial.print(Azymuth); Serial.println(" - Azymuth 1");
        Serial.print(Increase); Serial.println(" - Increase");
        Serial.print(valZero); Serial.println(" - valZero");
        Serial.print(valZero1); Serial.println(" - valZero1");
  */
      }else if((Azymuth >=  1.57 ) && (Azymuth < 3.14)){

        Increase1 = valD2-500;
          Increase2 = Increase1 / 500;
          Increase = Increase2 * maxAngle;
  
        valZero  = -1;
        valZero1 = -1;
        Azymuth = Azymuth - 2.35619;
        rotate2axed(120, valD3 + 100 ,120, Azymuth , Increase , valZero,  valZero1);
/*
        Serial.print(Azymuth); Serial.println(" - Azymuth 2");
        Serial.print(Increase); Serial.println(" - Increase");
        Serial.print(valZero); Serial.println(" - valZero");
        Serial.print(valZero1); Serial.println(" - valZero1");
  */
      }else if((Azymuth >= 3.14 ) && (Azymuth < 4.71)){

        Increase1 = valD2-500;
          Increase2 = Increase1 / 500;
          Increase = Increase2 * maxAngle;
  
        valZero  = -1;
        valZero1 =  1;
       Azymuth = Azymuth - 3.92699;
       rotate2axed(120, valD3 + 100 ,120, Azymuth , Increase , valZero,  valZero1);
/*
       Serial.print(Azymuth); Serial.println(" - Azymuth 3");
        Serial.print(Increase); Serial.println(" - Increase");
        Serial.print(valZero); Serial.println(" - valZero");
        Serial.print(valZero1); Serial.println(" - valZero1");
  */
      }else if((Azymuth >= 4.71 ) && (Azymuth <= 6.28)){

       Increase1 = valD2-500;
          Increase2 = Increase1 / 500;
          Increase = Increase2 * maxAngle;
  
      valZero  =  1;
      valZero1 = -1;
      Azymuth = Azymuth - 5.49;
      rotate2axed(120, valD3 + 100 ,120, Azymuth , Increase , valZero,  valZero1);
/*
      Serial.print(Azymuth); Serial.println(" - Azymuth 4");
        Serial.print(Increase); Serial.println(" - Increase");
        Serial.print(valZero); Serial.println(" - valZero");
        Serial.print(valZero1); Serial.println(" - valZero1");
  */
      }else{
        Serial.println("INVALID_ANGLE");
        return;
        serialFlush();
      }

            }
            
      }
      }
      else {
        Serial.print("got message with unknown tag ");
        Serial.write(tag);
      }
    }
  }
















/*
screwUrSelf(120 , 100 , 120 ,10 , 100);
screwUrSelf(120 , 100 , 120 ,10 , 100);
screwUrSelf(120 , 100 , 120 ,10 , 100);
screwUrSelf(120 , 100 , 120 ,10 , 100);
screwUrSelf(120 , 100 , 120 ,10 , 100);
screwUrSelf(120 , 100 , 120 ,10 , 100);

 RunnerP1.Step(120 , 100 , 120);
 RunnerP2.Step(120 , 100 , 120);
 RunnerL1.Step(120 , 100 , 120);
 RunnerL2.Step(120 , 100 , 120);
delay(300000000);
makeListOneSteepPrime(68, 90);
// makeOneStep( 100, 1, 120, 60 ,6 , testedSpeeds , 8);
// makeOneStep( 100, 1, 120, 60 ,6 , testedSpeeds , 8);
// makeOneStep( 100, 1, 120, 60 ,6 , testedSpeeds , 8);

/*
makeListOneSteep(68, 90);
for(int i = 2; i < 12; i = i + 2){
  Serial.print("Im Begining test for radius - "); Serial.println(i);
 makeOneStep( 100, 1, 120, 30 ,1 , testedSpeeds , i);
 Serial.print(".");
 makeOneStep( 100, 1, 120, 30 ,1 , testedSpeeds , i);
 Serial.print(".");
 makeOneStep( 100, 1, 120, 30 ,1 , testedSpeeds , i);
 Serial.print(".");
// makeOneStep( 100, 1, 120, 30 ,1 , 5 , i);
// makeOneStep( 100, 1, 120, 30 ,1 , 5 , i);

// makeOneStep( 100, 1, 120, 30 ,2, 5 , i);
// makeOneStep( 100, 1, 120, 30 ,2, 5 , i);
 makeOneStep( 100, 1, 120, 30 ,2, testedSpeeds , i);
 Serial.print(".");
 makeOneStep( 100, 1, 120, 30 ,2, testedSpeeds , i);
 Serial.print(".");
 makeOneStep( 100, 1, 120, 30 ,2, testedSpeeds , i);
 Serial.println("End of test");
}
 /*
 makeListOneSteep(68, 90);
 makeOneStep( 100, 1, 120, 30 ,2, 5 , 8);
 makeOneStep( 100, 1, 120, 30 ,2, 5 , 8);
 makeOneStep( 100, 1, 120, 30 ,2, 5 , 8);
 makeOneStep( 100, 1, 120, 30 ,2, 5 , 8);
 makeOneStep( 100, 1, 120, 30 ,2, 5 , 8);
 
/*
 makeListOneSteep(1, 1);
 Serial.println("NORMALSAGISH");
 makeOneStep( 100, 1, 120, 30 ,4);
 makeOneStep( 100, 1, 120, 30 ,4);
 makeOneStep( 100, 1, 120, 30 ,4);
  //makeOneStep( 100, 1, 120, 30 , 2);
  */



}
  /*
  makeListOneSteep(68, 90); // x : MAX 75
//Serial.println("Middle");
makeOneStep( 100, 1, 120, 30);
makeOneStep( 100, 1, 120, 30);
makeOneStep( 100, 1, 120, 30);

  
//   demo();
   if (Serial.available()){
BluetoothData=Serial.read();
  }
  if(BluetoothData == 't'){
 //if(true){
//moveExecuter(testPoseTable);
makeListOneSteep(55, 90); // x : MAX 75
//Serial.println("Middle");
makeOneStep( 100, 1, 120, 30);
makeOneStep( 100, 1, 120, 30);
makeOneStep( 100, 1, 120, 30);
   }

 //  if(BluetoothData == 'g'){
   if(true){
    /*
    while(true){
        for(double i = 0; i < 0.523599; i = i + 0.0104533){
rotate2axed(120,100,120, 0 , i , 1,  1);  // 15 . 30
//Serial.println("DONE1");
yield();
    }
    delay(1000);
for(double i = 0.523599; i > 0; i = i - 0.0104533){
rotate2axed(120,100,120, 0  , i , 1,   1);  // 15 . 30
//Serial.println("DONE4");
yield();
    }
    }
  
    while(true){
    for(double i = -0.785398; i < 0.785398; i = i + 0.1054533){
rotate2axed(120,100,120, i , 0.523599 , 1,  1);  // 15 . 30
//Serial.println("DONE1");
yield();
    }
for(double i = -0.785398; i < 0.785398; i = i + 0.1054533){
rotate2axed(120,100,120, i , 0.523599 , -1,   -1);  // 15 . 30
//Serial.println("DONE4");
yield();
    }
    
    /*
    for(double i = 0.785398; i > -0.785398; i = i - 0.0174533){
rotate2axed(120,100,120, i , 0.523599 , 1);
Serial.println("DONE2");
yield();
    }
    */

       /*
    for(double i = 0.785398; i > -0.785398; i = i - 0.0174533){
rotate2axed(120,100,120, i , 0.523599 , -1);
Serial.println("DONE3");
yield();
    }
   
    
     for(double i = -0.785398; i < 0.785398; i = i + 0.1054533){
rotate2axed(120,100,120, i , 0.523599 , -1,   1);  // 15 . 30
//Serial.println("DONE4");
yield();
    }

    
    for(double i = -0.785398; i < 0.785398; i = i + 0.1054533){
rotate2axed(120,100,120, i , 0.523599 , 1,  -1);  // 15 . 30
//  Serial.println("DONE1");
yield();
    }
    
   
    }
   BluetoothData = false;
   }
   
  if(BluetoothData == 'm'){
int con = true;
int read1;
int read2;
int read3;
int read4;
int bool1;
int bool2;

 int read1s = analogRead(analogPin1);
 int read2s = analogRead(analogPin2);
 int read3s = analogRead(analogPin3);
 int read4s = analogRead(analogPin4);

 RunnerP1.Step(120 , 100 , 120);
 RunnerP2.Step(120 , 100 , 120);
 RunnerL1.Step(120 , 100 , 120);
 RunnerL2.Step(120 , 100 , 120);
 

while(con == true){
  read1 = analogRead(analogPin1);
  read2 = analogRead(analogPin2);
  read3 = analogRead(analogPin3);
  read4 = analogRead(analogPin4);

  bool1 = digitalRead(digitalPin1);
  bool2 = digitalRead(digitalPin2);


  
}
   }
   
    if(BluetoothData == 'e'){
   if(isStable(100 , 100 , 100 , 100 , -60 , 100 , directionP1X, directionP1Z, directionP2X, directionP2Z, directionL1X, directionL1Z) == true){
      Serial.println("Yep");
   }else{
      Serial.println("Dope tho...");
   }
   }

if(BluetoothData == 's'){
RunnerP1.Step(115 , 120 , 115);
RunnerP2.Step(157 , 120 , 42);
RunnerL1.Step(60 , 120 , 157);
RunnerL2.Step(55 , 50 , 55);
delay(10000000);
   }
   
   if(BluetoothData == '1'){
    cinematicTest();
    BluetoothData=false;
   }
   if(BluetoothData == '2'){
    RunnerP1.Step(115 , 100 , 115);
    delay(100);
    RunnerP1.Step(115 , 30 , 115);
    delay(300);
    RunnerP1.Step(115 , 100 , 115);
    delay(100);
    BluetoothData=false;
   }

   if(BluetoothData == '3'){
    RunnerP2.Step(115 , 100 , 115);
    delay(100);
    RunnerP2.Step(115 , 30 , 115);
    delay(300);
    RunnerP2.Step(115 , 100 , 115);
    delay(100);
    BluetoothData=false;
   }

   if(BluetoothData == '4'){
    RunnerL1.Step(115 , 100 , 115);
    delay(100);
    RunnerL1.Step(115 , 30 , 115);
    delay(300);
    RunnerL1.Step(115 , 100 , 115);
    delay(100);
    BluetoothData=false;
   }

   if(BluetoothData == '5'){
    RunnerL2.Step(115 , 100 , 115);
    delay(100);
    RunnerL2.Step(115 , 30 , 115);
    delay(300);
    RunnerL2.Step(115 , 100 , 115);
    delay(100);
    BluetoothData=false;
   }
//demo();
  /*
while(true){
while(ZposT > -100){

RunnerP1.Step(107 , YposT , 121+ZposT);
RunnerP2.Step(107 , YposT , 121+ZposT);

RunnerL1.Step(107 , YposT , 121-ZposT);
RunnerL2.Step(107 , YposT , 121-ZposT);
YposT = YposT - 0;
ZposT = ZposT - 1;
Serial.println(YposT);
delay(0.5);
}

while(ZposT < 100){

RunnerP1.Step(107 , YposT , 121+ZposT);
RunnerP2.Step(107 , YposT , 121+ZposT);

RunnerL1.Step(107 , YposT , 121-ZposT);
RunnerL2.Step(107 , YposT , 121-ZposT);
YposT = YposT + 0;
ZposT = ZposT + 1;
Serial.println(YposT);
delay(0.5);
}
} 
  // Run();
delay(10);
// Run();
//Config();
//con = true;
}

*/
