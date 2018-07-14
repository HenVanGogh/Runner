// #define anal
void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}   

void rotate2axed(double xPos,double yPos,double zPos, double azymuth , double increase , int valZero, int valZero1){

double A1 , A2 , A3 , A4 , hX1 , hZ1 , hX2 , hZ2 , hX3 , hZ3 , hX4 , hZ4;

double a , b , c , d;
double H1a , H1b , H1 , H2 , vR , vRu , vRk;
double K1 ,K2 ,K3 ,K4 , L1 , L2 , L3 , L4 , yBis1, yBis2, yBis3, yBis4 , Lz1, Lz2;
double  xBis1 , xBis2 , xBis3 , xBis4;
double  zBis1 , zBis2 , zBis3 , zBis4;

double increaseBis1;
double increaseBis2;

double newXZ1;
double newXZ2;
double newXZ3;
double newXZ4;

double KR1 , Kx1;
double KR2 , Kx2;


/*
if((azymuth >= 0 ) && (azymuth <  1.5708)){
  
}else if((azymuth >=  1.5708 ) && (azymuth < 3.14159)){
  
}else if((azymuth >= 3.14159 ) && (azymuth < 4.71239)){
  
}else if((azymuth >= 4.71239 ) && (azymuth < 6.28319)){
  
}else{
  Serial.println("INVALID_ANGLE");
}

*/



A1 = azymuth + 0.785398;
A2 = 0.785398 - azymuth;

#ifdef anal
Serial.println("");
Serial.print("A1 :  "); Serial.println(A1);
Serial.print("A2 :  "); Serial.println(A2);
Serial.println("");
#endif

a =  absoluteRadius * sin(A1);
c = -absoluteRadius * sin(A2);

#ifdef anal
Serial.println("");
Serial.print("a :  "); Serial.println(a);
Serial.print("c :  "); Serial.println(c);
Serial.println("");
#endif

b =  absoluteRadius * cos(A1);
d = -absoluteRadius * cos(A2);

#ifdef anal
Serial.println("");
Serial.print("b :  "); Serial.println(b);
Serial.print("d :  "); Serial.println(d);
Serial.println("");
#endif

//H1a = sin(increase) * a;
//H1b = sin(increase) * b;

//H1 = H1a + yPos;
//H2 = H2a + yPos;

vRu = sqrt((xPos*xPos)+(zPos*zPos)) + absoluteRadius;
vR = sqrt((vRu*vRu)+(yPos*yPos));
//vRk = acos(((vRu*vRu)+(vR*vR) - (yPos*yPos))/ (2*vRu * vR));
vRk = asin(yPos / vR);
#ifdef anal
Serial.println("");
Serial.print("vRu :  "); Serial.println(vRu);
Serial.print("vR :  "); Serial.println(vR);
Serial.print("vRk :  "); Serial.println(vRk);
Serial.println("");
#endif

//Lz1 = sqrt((H1*H1)+(a*a)+(c*c));
//Lz2 = sqrt((H2*H2)+(b*b)+(d*d));
KR1 = sqrt((a*a)+(b*b));
KR2 = sqrt((c*c)+(d*d));

#ifdef anal
Serial.println("");
Serial.print("KR1 :  "); Serial.println(KR1);
Serial.print("KR2 :  "); Serial.println(KR2);
Serial.println("");
#endif

Kx1 = (sin(increase)*a) / 2;
Kx2 = (sin(increase)*c) / 2;

#ifdef anal
Serial.println("");
Serial.print("Kx1 :  "); Serial.println(Kx1);
Serial.print("Kx2 :  "); Serial.println(Kx2);
Serial.println("");
#endif

increaseBis1 = acos(1 - (Kx1 * Kx1) / 14400.06);
increaseBis2 = acos(1 - (Kx2 * Kx2) / 14400.06);

#ifdef anal
Serial.println("");
Serial.print("increaseBis1 :  "); Serial.println(increaseBis1);
Serial.print("increaseBis2 :  "); Serial.println(increaseBis2);
Serial.println("");
#endif


K1 = vRk + increaseBis1 * valZero;
K2 = vRk + increaseBis2 * valZero;
K3 = vRk - increaseBis2 * valZero;
K4 = vRk - increaseBis1 * valZero;

/*
if(azymuth < 0){
K1 = vRk - increaseBis1;
K2 = vRk - increaseBis2;
K3 = vRk + increaseBis2;
K4 = vRk + increaseBis1;
}
*/

#ifdef anal
Serial.println("");
Serial.print("K1 :  "); Serial.println(K1);
Serial.print("K2 :  "); Serial.println(K2);
Serial.print("K3 :  "); Serial.println(K3);
Serial.print("K4 :  "); Serial.println(K4);
Serial.println("");
#endif

L1 = cos(K1)*vR;
L2 = cos(K2)*vR;
L3 = cos(K3)*vR;
L4 = cos(K4)*vR;

#ifdef anal
Serial.println("");
Serial.print("L1 :  "); Serial.println(L1);
Serial.print("L2 :  "); Serial.println(L2);
Serial.print("L3 :  "); Serial.println(L3);
Serial.print("L4 :  "); Serial.println(L4);
Serial.println("");
#endif

newXZ1 = L1 - absoluteRadius;
newXZ2 = L2 - absoluteRadius;
newXZ3 = L3 - absoluteRadius;
newXZ4 = L4 - absoluteRadius;

#ifdef anal
Serial.println("");
Serial.print("newXZ1 :  "); Serial.println(newXZ1);
Serial.print("newXZ2 :  "); Serial.println(newXZ2);
Serial.print("newXZ3 :  "); Serial.println(newXZ3);
Serial.print("newXZ4 :  "); Serial.println(newXZ4);
Serial.println("");
#endif

yBis1 = sin(K1)*vR;
yBis2 = sin(K2)*vR;
yBis3 = sin(K3)*vR;
yBis4 = sin(K4)*vR;

#ifdef anal
Serial.println("");
Serial.print("yBis1 :  "); Serial.println(yBis1);
Serial.print("yBis2 :  "); Serial.println(yBis2);
Serial.print("yBis3 :  "); Serial.println(yBis3);
Serial.print("yBis4 :  "); Serial.println(yBis4);
Serial.println("");
#endif



yBis1 = sqrt((vR*vR)-(L1*L1));
yBis2 = sqrt((vR*vR)-(L2*L2));
yBis3 = sqrt((vR*vR)-(L3*L3));
yBis4 = sqrt((vR*vR)-(L4*L4));

 //Serial.print(yBis1); Serial.print(" ");
 //Serial.println(yBis2);

#ifdef anal
Serial.println("");
Serial.print("yBis1s :  "); Serial.println(yBis1);
Serial.print("yBis2s :  "); Serial.println(yBis2);
Serial.print("yBis3s :  "); Serial.println(yBis3);
Serial.print("yBis4s :  "); Serial.println(yBis4);
Serial.println("");
#endif

xBis1 = newXZ1 / sqrt(2);
xBis2 = newXZ2 / sqrt(2);
xBis3 = newXZ3 / sqrt(2);
xBis4 = newXZ4 / sqrt(2);



#ifdef anal
Serial.println("");
Serial.print("xBis1 :  "); Serial.println(xBis1);
Serial.print("xBis2 :  "); Serial.println(xBis2);
Serial.print("xBis3 :  "); Serial.println(xBis3);
Serial.print("xBis4 :  "); Serial.println(xBis4);
Serial.println("");
#endif

zBis1 = newXZ1 / sqrt(2);
zBis2 = newXZ2 / sqrt(2);
zBis3 = newXZ3 / sqrt(2);
zBis4 = newXZ4 / sqrt(2);

#ifdef anal
Serial.println("");
Serial.print("zBis1 :  "); Serial.println(zBis1);
Serial.print("zBis2 :  "); Serial.println(zBis2);
Serial.print("zBis3 :  "); Serial.println(zBis3);
Serial.print("zBis4 :  "); Serial.println(zBis4);
Serial.println("");
#endif
//Serial.println("WUT ZE FUK");


if(valZero1 == 1){
RunnerP1.Step(xBis1 , yBis1 , zBis1);
RunnerP2.Step(xBis2 , yBis2 , zBis2);

RunnerL1.Step(xBis3 , yBis3 , zBis3);
RunnerL2.Step(xBis4 , yBis4 , zBis4);
//Serial.println("WUT ZE FUK");
}else{

RunnerP2.Step(xBis1 , yBis1 , zBis1);
RunnerL2.Step(xBis2 , yBis2 , zBis2);

RunnerP1.Step(xBis3 , yBis3 , zBis3);
RunnerL1.Step(xBis4 , yBis4 , zBis4);
}


} 













void rotate3axed(double xPos,double yPos,double zPos, double azymuth , double increase, double increaseBiser , int valZero, int valZero1){

double A1 , A2 , A3 , A4 , hX1 , hZ1 , hX2 , hZ2 , hX3 , hZ3 , hX4 , hZ4;

double a , b , c , d;
double H1a , H1b , H1 , H2 , vR , vRu , vRk;
double K1 ,K2 ,K3 ,K4 , L1 , L2 , L3 , L4 , yBis1, yBis2, yBis3, yBis4 , Lz1, Lz2;
double  xBis1 , xBis2 , xBis3 , xBis4;
double  zBis1 , zBis2 , zBis3 , zBis4;

double increaseBis1;
double increaseBis2;

double newXZ1;
double newXZ2;
double newXZ3;
double newXZ4;

double KR1 , Kx1;
double KR2 , Kx2;


/*
if((azymuth >= 0 ) && (azymuth <  1.5708)){
  
}else if((azymuth >=  1.5708 ) && (azymuth < 3.14159)){
  
}else if((azymuth >= 3.14159 ) && (azymuth < 4.71239)){
  
}else if((azymuth >= 4.71239 ) && (azymuth < 6.28319)){
  
}else{
  Serial.println("INVALID_ANGLE");
}

*/



A1 = azymuth + 0.785398;
A2 = 0.785398 - azymuth;

#ifdef anal
Serial.println("");
Serial.print("A1 :  "); Serial.println(A1);
Serial.print("A2 :  "); Serial.println(A2);
Serial.println("");
#endif

a =  absoluteRadius * sin(A1);
c = -absoluteRadius * sin(A2);

#ifdef anal
Serial.println("");
Serial.print("a :  "); Serial.println(a);
Serial.print("c :  "); Serial.println(c);
Serial.println("");
#endif

b =  absoluteRadius * cos(A1);
d = -absoluteRadius * cos(A2);

#ifdef anal
Serial.println("");
Serial.print("b :  "); Serial.println(b);
Serial.print("d :  "); Serial.println(d);
Serial.println("");
#endif

//H1a = sin(increase) * a;
//H1b = sin(increase) * b;

//H1 = H1a + yPos;
//H2 = H2a + yPos;

vRu = sqrt((xPos*xPos)+(zPos*zPos)) + absoluteRadius;
vR = sqrt((vRu*vRu)+(yPos*yPos));
//vRk = acos(((vRu*vRu)+(vR*vR) - (yPos*yPos))/ (2*vRu * vR));
vRk = asin(yPos / vR);
#ifdef anal
Serial.println("");
Serial.print("vRu :  "); Serial.println(vRu);
Serial.print("vR :  "); Serial.println(vR);
Serial.print("vRk :  "); Serial.println(vRk);
Serial.println("");
#endif

//Lz1 = sqrt((H1*H1)+(a*a)+(c*c));
//Lz2 = sqrt((H2*H2)+(b*b)+(d*d));
KR1 = sqrt((a*a)+(b*b));
KR2 = sqrt((c*c)+(d*d));

#ifdef anal
Serial.println("");
Serial.print("KR1 :  "); Serial.println(KR1);
Serial.print("KR2 :  "); Serial.println(KR2);
Serial.println("");
#endif

Kx1 = (sin(increase)*a) / 2;
Kx2 = (sin(increase)*c) / 2;

#ifdef anal
Serial.println("");
Serial.print("Kx1 :  "); Serial.println(Kx1);
Serial.print("Kx2 :  "); Serial.println(Kx2);
Serial.println("");
#endif

increaseBis1 = acos(1 - (Kx1 * Kx1) / 14400.06);
increaseBis2 = acos(1 - (Kx2 * Kx2) / 14400.06);

#ifdef anal
Serial.println("");
Serial.print("increaseBis1 :  "); Serial.println(increaseBis1);
Serial.print("increaseBis2 :  "); Serial.println(increaseBis2);
Serial.println("");
#endif


K1 = vRk + increaseBis1 * valZero;
K2 = vRk + increaseBis2 * valZero;
K3 = vRk - increaseBis2 * valZero;
K4 = vRk - increaseBis1 * valZero;

/*
if(azymuth < 0){
K1 = vRk - increaseBis1;
K2 = vRk - increaseBis2;
K3 = vRk + increaseBis2;
K4 = vRk + increaseBis1;
}
*/

#ifdef anal
Serial.println("");
Serial.print("K1 :  "); Serial.println(K1);
Serial.print("K2 :  "); Serial.println(K2);
Serial.print("K3 :  "); Serial.println(K3);
Serial.print("K4 :  "); Serial.println(K4);
Serial.println("");
#endif

L1 = cos(K1)*vR;
L2 = cos(K2)*vR;
L3 = cos(K3)*vR;
L4 = cos(K4)*vR;

#ifdef anal
Serial.println("");
Serial.print("L1 :  "); Serial.println(L1);
Serial.print("L2 :  "); Serial.println(L2);
Serial.print("L3 :  "); Serial.println(L3);
Serial.print("L4 :  "); Serial.println(L4);
Serial.println("");
#endif

newXZ1 = L1 - absoluteRadius;
newXZ2 = L2 - absoluteRadius;
newXZ3 = L3 - absoluteRadius;
newXZ4 = L4 - absoluteRadius;

#ifdef anal
Serial.println("");
Serial.print("newXZ1 :  "); Serial.println(newXZ1);
Serial.print("newXZ2 :  "); Serial.println(newXZ2);
Serial.print("newXZ3 :  "); Serial.println(newXZ3);
Serial.print("newXZ4 :  "); Serial.println(newXZ4);
Serial.println("");
#endif

yBis1 = sin(K1)*vR;
yBis2 = sin(K2)*vR;
yBis3 = sin(K3)*vR;
yBis4 = sin(K4)*vR;

#ifdef anal
Serial.println("");
Serial.print("yBis1 :  "); Serial.println(yBis1);
Serial.print("yBis2 :  "); Serial.println(yBis2);
Serial.print("yBis3 :  "); Serial.println(yBis3);
Serial.print("yBis4 :  "); Serial.println(yBis4);
Serial.println("");
#endif



yBis1 = sqrt((vR*vR)-(L1*L1));
yBis2 = sqrt((vR*vR)-(L2*L2));
yBis3 = sqrt((vR*vR)-(L3*L3));
yBis4 = sqrt((vR*vR)-(L4*L4));

 //Serial.print(yBis1); Serial.print(" ");
 //Serial.println(yBis2);

#ifdef anal
Serial.println("");
Serial.print("yBis1s :  "); Serial.println(yBis1);
Serial.print("yBis2s :  "); Serial.println(yBis2);
Serial.print("yBis3s :  "); Serial.println(yBis3);
Serial.print("yBis4s :  "); Serial.println(yBis4);
Serial.println("");
#endif

xBis1 = newXZ1 / sqrt(2);
xBis2 = newXZ2 / sqrt(2);
xBis3 = newXZ3 / sqrt(2);
xBis4 = newXZ4 / sqrt(2);



#ifdef anal
Serial.println("");
Serial.print("xBis1 :  "); Serial.println(xBis1);
Serial.print("xBis2 :  "); Serial.println(xBis2);
Serial.print("xBis3 :  "); Serial.println(xBis3);
Serial.print("xBis4 :  "); Serial.println(xBis4);
Serial.println("");
#endif

zBis1 = newXZ1 / sqrt(2);
zBis2 = newXZ2 / sqrt(2);
zBis3 = newXZ3 / sqrt(2);
zBis4 = newXZ4 / sqrt(2);

#ifdef anal
Serial.println("");
Serial.print("zBis1 :  "); Serial.println(zBis1);
Serial.print("zBis2 :  "); Serial.println(zBis2);
Serial.print("zBis3 :  "); Serial.println(zBis3);
Serial.print("zBis4 :  "); Serial.println(zBis4);
Serial.println("");
#endif
//Serial.println("WUT ZE FUK");


if(valZero1 == 1){
RunnerP1.Step(xBis1 , yBis1 , zBis1);
RunnerP2.Step(xBis2 , yBis2 , zBis2);

RunnerL1.Step(xBis3 , yBis3 , zBis3);
RunnerL2.Step(xBis4 , yBis4 , zBis4);
//Serial.println("WUT ZE FUK");
}else{

RunnerP2.Step(xBis1 , yBis1 , zBis1);
RunnerL2.Step(xBis2 , yBis2 , zBis2);

RunnerP1.Step(xBis3 , yBis3 , zBis3);
RunnerL1.Step(xBis4 , yBis4 , zBis4);
}











//Lz = sqrt((H1*H1)+(xPos*xPos));
/*
hX1 = absoluteRadius * sin(A3);
hZ1 = absoluteRadius * cos(A3);

hX1 = absoluteRadius * sin(A4);
hZ1 = absoluteRadius * cos(A4);
*/

 /*
  
}
double azymuthGyro;
double inclinationGyro;

 void translateAngles(double xAxisAngle , double zAxisAngle ){
  double xLinear;
  double y1Linear;
  double y2Linear;
  double zLinear;

  double d;

  xLinear = cos(xAxisAngle);
  y1Linear = sin(xAxisAngle);

  zLinear = cos(zAxisAngle);
  y2Linear = sin(zAxisAngle);

  d = sqrt((xLinear*xLinear)+(zLinear*zLinear));
  azymuthGyro = tan(xLinear/zLinear);
  inclinationGyro = tan((y1Linear + y2Linear) / d);
  
 }
String myString1;
String myString2;
 
void maintain2AxedStability(){
   if (gyroSerial.available()) {
    //gyroSerial.read());
    myString1 = gyroSerial.readStringUntil(' ');
    myString2 = gyroSerial.readStringUntil('\n');
  }
  translateAngles(myString1.toInt() , myString2.toInt());
   */
}





void screwUrSelf(int Xdefault, int Ydefault, int Zdefault, double substractAngle, double deltaY){
 double firstGroup;
 double seconGroup;

 double heightPlus = 0;
 substractAngle = substractAngle / 57.2958;

for(int n = 0; n < 3; n = n + 1){

  
   for(double i = substractAngle; i < (1.5708 - substractAngle) ; i = i + 0.0134533){
    firstGroup = sin(i);
    seconGroup = cos(i);

    //heightPlus = map(i , substractAngle , 1.5708 - substractAngle , 0 , deltaY);

    heightPlus = deltaY * ((i + substractAngle) / (1.5708 - substractAngle));

    Serial.println(heightPlus);

    RunnerP1.Step(Xdefault * firstGroup , Ydefault + heightPlus , Zdefault * seconGroup );
    RunnerP2.Step(Xdefault * seconGroup , Ydefault + heightPlus , Zdefault * firstGroup );

    RunnerL1.Step(Xdefault * seconGroup , Ydefault + heightPlus , Zdefault * firstGroup );
    RunnerL2.Step(Xdefault * firstGroup , Ydefault + heightPlus , Zdefault * seconGroup );

    delay(10);
    
  }


  for(double i = 1.5708 - substractAngle; i > substractAngle ; i = i - 0.0134533){
    firstGroup = sin(i);
    seconGroup = cos(i);

    //heightPlus = map(i , substractAngle , 1.5708 - substractAngle , 0 , deltaY);

    heightPlus = deltaY * ((i + substractAngle) / (1.5708 - substractAngle));

    Serial.println(heightPlus);

    RunnerP1.Step(Xdefault * firstGroup , Ydefault + heightPlus , Zdefault * seconGroup );
    RunnerP2.Step(Xdefault * seconGroup , Ydefault + heightPlus , Zdefault * firstGroup );

    RunnerL1.Step(Xdefault * seconGroup , Ydefault + heightPlus , Zdefault * firstGroup );
    RunnerL2.Step(Xdefault * firstGroup , Ydefault + heightPlus , Zdefault * seconGroup );

    delay(10);
  }
}


}
 
 



 void screwUrSelfWoof(int Xdefault, int Ydefault, int Zdefault, double substractAngle, double heightPlus){
 double firstGroup;
 double seconGroup;

 substractAngle = substractAngle / 57.2958;



  
 
    firstGroup = sin(substractAngle);
    seconGroup = cos(substractAngle);

    RunnerP1.Step(Xdefault * firstGroup , Ydefault + heightPlus , Zdefault * seconGroup );
    RunnerP2.Step(Xdefault * seconGroup , Ydefault + heightPlus , Zdefault * firstGroup );

    RunnerL1.Step(Xdefault * seconGroup , Ydefault + heightPlus , Zdefault * firstGroup );
    RunnerL2.Step(Xdefault * firstGroup , Ydefault + heightPlus , Zdefault * seconGroup );

}
 
 
 void Prin(){
Serial.print("genericValL11 ="); Serial.print(valL11); Serial.println(";");
Serial.print("genericValL12 ="); Serial.print(valL12);Serial.println(";");
Serial.print("genericValL13 ="); Serial.print(valL13);Serial.println(";");
Serial.print("genericValL14 ="); Serial.print(valL13);Serial.println(";");

Serial.print("genericValL21 ="); Serial.print(valL21);Serial.println(";");
Serial.print("genericValL22 ="); Serial.print(valL22);Serial.println(";");
Serial.print("genericValL23 ="); Serial.print(valL23);Serial.println(";");
Serial.print("genericValL24 ="); Serial.print(valL23);Serial.println(";");



Serial.print("genericValP11 ="); Serial.print(valP11);Serial.println(";");
Serial.print("genericValP12 ="); Serial.print(valP12);Serial.println(";");
Serial.print("genericValP13 ="); Serial.print(valP13);Serial.println(";");
Serial.print("genericValP14 ="); Serial.print(valP13);Serial.println(";");

Serial.print("genericValP21 ="); Serial.print(valP21);Serial.println(";");
Serial.print("genericValP22 ="); Serial.print(valP22);Serial.println(";");
Serial.print("genericValP23 ="); Serial.print(valP23);Serial.println(";");
Serial.print("genericValP24 ="); Serial.print(valP23);Serial.println(";");

Serial.println("-----------------------------------------");
Serial.println();
}



void Config(){
   Serial.println("Robot Calibration Mode On");
   Serial.println("Command List Below");
   Serial.println("For L click X");
   Serial.println("For P click Z");
   Serial.println("For Go back F13 :)");
   while(con == true){
   delay(configDelayTime);
   
   if (Serial.available()){
   BluetoothData=Serial.read();
    
   if (BluetoothData=='Z'){   ///P///      ///---///
   Serial.println("For 1 click 1");
   Serial.println("For 2 click 2");
   Serial.println("For 3 click 3");
     
   while(con == true){
if (Serial.available()){
BluetoothData=Serial.read();

   if(BluetoothData=='1'){  
    Serial.println("For Servo 1 - I");
    Serial.println("For Servo 2 - O");
    Serial.println("For Servo 3 - P"); 
    while(con==true){  
 if (Serial.available()){
 BluetoothData=Serial.read();
   if(BluetoothData=='I'){  
    
    while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posP11=posP11+1;
    valP11=posP11;
    Serial.println(posP11);
    Run();
   }
   if(BluetoothData=='-'){  
    posP11=posP11-1;
    valP11=posP11;
    Serial.println(posP11);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='O'){  

    while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posP12=posP12+1;
    valP12=posP12;
    Serial.println(posP12);
    Run();
   }
   if(BluetoothData=='-'){  
    posP12=posP12-1;
    valP12=posP12;
    Serial.println(posP12);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='P'){  

    while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posP13=posP13+1;
    valP13=posP13;
    Serial.println(posP13);
    Run();
   }
   if(BluetoothData=='-'){  
    posP13=posP13-1;
    valP13=posP13;
    Serial.println(posP13);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='2'){  
    Serial.println("For Servo 1 - I");
    Serial.println("For Servo 2 - O");
    Serial.println("For Servo 3 - P");
    while(con==true){  
 if (Serial.available()){
 BluetoothData=Serial.read();
   if(BluetoothData=='I'){  

    while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posP21=posP21+1;
    valP21=posP21;
    Serial.println(posP21);
    Run();
   }
   if(BluetoothData=='-'){  
    posP21=posP21-1;
    valP21=posP21;
    Serial.println(posP21);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='O'){  

    while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posP22=posP22+1;
    valP22=posP22;
    Serial.println(posP22);
    Run();
   }
   if(BluetoothData=='-'){  
    posP22=posP22-1;
    valP22=posP22;
    Serial.println(posP22);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='P'){  

    while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posP23=posP23+1;
    valP23=posP23;
    Serial.println(posP23);
    Run();
   }
   if(BluetoothData=='-'){  
    posP23=posP23-1;
    valP23=posP23;
    Serial.println(posP23);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='3'){  
    Serial.println("For Servo 1 - I");
    Serial.println("For Servo 2 - O");
    Serial.println("For Servo 3 - P");
    while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='I'){  

    while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posP31=posP31+1;
    valP31=posP31;
    Serial.println(posP31);
    Run();
   }
   if(BluetoothData=='-'){  
    posP31=posP31-1;
    valP31=posP31;
    Serial.println(posP31);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='O'){  

    while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posP32=posP32+1;
    valP32=posP32;
    Serial.println(posP32);
    Run();
   }
   if(BluetoothData=='-'){  
    posP32=posP32-1;
    valP32=posP32;
    Serial.println(posP32);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='P'){  

    while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posP33=posP33+1;
    valP33=posP33;
    Serial.println(posP33);
    Run();
   }
   if(BluetoothData=='-'){  
    posP33=posP33-1;
    valP33=posP33;
    Serial.println(posP33);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
}
delay(configDelayTime);
}
   }
}
delay(configDelayTime);
   }
   }
   
   if (BluetoothData=='X'){   ///L///     ///---///
   Serial.println("For 1 click 1");
   Serial.println("For 2 click 2");
   Serial.println("For 3 click 3");
   
   while(con == true){
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='1'){  
    Serial.println("For Servo 1 - I");
    Serial.println("For Servo 2 - O");
    Serial.println("For Servo 3 - P");
    while(con==true){  
  if (Serial.available()){
  BluetoothData=Serial.read();
   if(BluetoothData=='I'){  

    while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posL11=posL11+1;
    valL11=posL11;
    Serial.println(posL11);
    Run();
   }
   if(BluetoothData=='-'){  
    posL11=posL11-1;
    valL11=posL11;
    Serial.println(posL11);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='O'){  
 while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posL12=posL12+1;
    valL12=posL12;
    Serial.println(posL12);
    Run();
   }
   if(BluetoothData=='-'){  
    posL12=posL12-1;
    valL12=posL12;
    Serial.println(posL12);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='P'){  

        while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posL13=posL13+1;
    valL13=posL13;
    Serial.println(posL13);
    Run();
   }
   if(BluetoothData=='-'){  
    posL13=posL13-1;
    valL13=posL13;
    Serial.println(posL13);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='2'){  
    Serial.println("For Servo 1 - I");
    Serial.println("For Servo 2 - O");
    Serial.println("For Servo 3 - P"); 
    while(con==true){  
  if (Serial.available()){
  BluetoothData=Serial.read();
   if(BluetoothData=='I'){  

        while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posL21=posL21+1;
    valL21=posL21;
    Serial.println(posL21);
    Run();
   }
   if(BluetoothData=='-'){  
    posL21=posL21-1;
    valL21=posL21;
    Serial.println(posL21);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='O'){  
        while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posL22=posL22+1;
    valL22=posL22;
    Serial.println(posL22);
    Run();
   }
   if(BluetoothData=='-'){  
    posL22=posL22-1;
    valL22=posL22;
    Serial.println(posL22);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='P'){  

        while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posL23=posL23+1;
    valL23=posL23;
    Serial.println(posL23);
    Run();
   }
   if(BluetoothData=='-'){  
    posL23=posL23-1;
    valL23=posL23;
    Serial.println(posL23);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
}
delay(configDelayTime);
}  
   }
   if(BluetoothData=='3'){  
    Serial.println("For Servo 1 - I");
    Serial.println("For Servo 2 - O");
    Serial.println("For Servo 3 - P");
    while(con==true){  
  if (Serial.available()){
  BluetoothData=Serial.read();
   if(BluetoothData=='I'){  

        while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posL31=posL31+1;
    valL31=posL31;
    Serial.println(posL31);
    Run();
   }
   if(BluetoothData=='-'){  
    posL31=posL31-1;
    valL31=posL31;
    Serial.println(posL31);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='O'){  

        while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posL32=posL32+1;
    valL32=posL32;
    Serial.println(posL32);
    Run();
   }
   if(BluetoothData=='-'){  
    posL32=posL32-1;
    valL32=posL32;
    Serial.println(posL32);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
   if(BluetoothData=='P'){  

        while(con==true){  
if (Serial.available()){
BluetoothData=Serial.read();
   if(BluetoothData=='+'){  
    posL33=posL33+1;
    valL33=posL33;
    Serial.println(posL33);
    Run();
   }
   if(BluetoothData=='-'){  
    posL33=posL33-1;
    valL33=posL33;
    Serial.println(posL33);
    Run();
   }
   if (BluetoothData=='B'){ 
  con=false;
 }
}
delay(configDelayTime);
}
   }
}
delay(configDelayTime);
}
   }
}
delay(configDelayTime);
   }
   }
   }
   }
   Prin();
  }




void testBalanceRadius(){
  makeListOneSteep(68, 90);
for(int i = 8; i < 18; i = i + 2){
  Serial.print("Im Begining test for radius - "); Serial.println(i);
 makeOneStep( 100, 1, 120, 30 ,1 , 5 , i);
 Serial.print(".");
 makeOneStep( 100, 1, 120, 30 ,1 , 5 , i);
 Serial.print(".");
 makeOneStep( 100, 1, 120, 30 ,1 , 5 , i);
 Serial.print(".");
// makeOneStep( 100, 1, 120, 30 ,1 , 5 , i);
// makeOneStep( 100, 1, 120, 30 ,1 , 5 , i);

// makeOneStep( 100, 1, 120, 30 ,2, 5 , i);
// makeOneStep( 100, 1, 120, 30 ,2, 5 , i);
 makeOneStep( 100, 1, 120, 30 ,2, 5 , i);
 Serial.print(".");
 makeOneStep( 100, 1, 120, 30 ,2, 5 , i);
 Serial.print(".");
 makeOneStep( 100, 1, 120, 30 ,2, 5 , i);
 Serial.print("End of test");
}
}











  
