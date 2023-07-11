#include <math.h>

//### parameters bluetooth sensor ###
bool goodWidth=false, goodLength=false;
bool again=false;
int value;
int val; //informations if the user set the width or the length
float rank=0.1;

//### parameters ultrasonic sensor ###
#define trigPin 7  //pin 7
#define echoPin 8  //pin 8

float USrange();
int minRange = 30;  //en centimetres

float distance;  //en centimetres
float parcours[2];
int orientation = 1;

float largRobot = 0.29; 
float largRoue = 0.06;

float turnSpeed = 3; // in seconds

int length, width; // longueur, largueur in meters

#define memorySize 10
float distMemory[memorySize];

//### parameters for motor driver ###
#define enA 2  //pin 2
#define motorL 0
#define motorR 2

#define forward 0
#define backward 1

float defaultSpeed = 2;  //speed of the robot in meters per second

int Hbridge[4] = { 5, 6, 10, 9 };  //in1 -> pin5, in2 -> pin6, in3 -> pin10, in4 -> pin9

void motorControl(int motor, int rotation, float mps);  //motor is for right and left motor
                                                     //rotation is for the direction, 0 -> forward, 1 -> backward
                                                     //mps is for the speed of the rotation, it's in meters per second

int robotState = 3;   // 0 : démarrage d'un cycle nettoyage
                      // 1 : Robot en fonctionnement
                      // 2 : Robot en pause (obstacle devant)
                      // 3 : Nettoyage terminé
                      // 4 : Robot en pause (application)

long timer[4];
long timeSave;

void setup() {
  Serial.begin(9600);

  //### setup bluetooth sensor ###
  Serial1.begin(9600);
  Serial1.print("*1"+String("")+"*");
  Serial1.print("*2"+String("")+"*");

  //### setup ultrasonic sensor ###
  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input

  //### setup driver motor ###
  pinMode(enA, OUTPUT);
  for (int i = 0; i < 4; i++) {
    pinMode(Hbridge[i], OUTPUT);
  }

  Serial.println(USrange());

  Serial.println("Setup done.");

  delay(500);
}



void loop() {

  if(robotState == 3){ // Nettoyage terminé

    reset(); // reset l'intégralité des variables
    
    for (int i = 0; i < memorySize; i++) {
      distMemory[i] = USrange();
      delay(1);
    }

    // ***** attendre les infos bluetooth et setup everything *****
    while (robotState==3){
    setSetup();          //configure la longueur et la largeur 
    checkStartCycle();   // change le statue du robot si le cycle de nettoyage peut commencer
    }
  }

  // si recois stop du bluetooth, mettre le nettoyage en pause
  changeRobotState();

  if(robotState != 3 && robotState != 4){

    distance = mesure();

    checkWay(distance);

    timeSave = timing(robotState);

    //parcours[0] = (float(length)/2 + (float(length)/2)*-orientation) + (timeSave * defaultSpeed/1000)*orientation;
    //parcours[0] = (timeSave * defaultSpeed/1000)*orientation;
    parcours[0] = timeSave * defaultSpeed/1000;

    Serial.println(timeSave);
    Serial.print("X : ");
    Serial.print((float(length)/2 + (float(length)/2)*-orientation) + parcours[0]*orientation);
    Serial.print(" m. / Y : ");
    Serial.print(parcours[1]);
    Serial.println(" m.");

    if(parcours[0] > length){
      turn(orientation);
      orientation = orientation*(-1);
    }

  }

  delay(10);
}


// ### FONCTION POUR MOTORISATION ###

void motorControl(int motor, int rotation, float mps) {
  digitalWrite(Hbridge[motor], rotation % 2);
  digitalWrite(Hbridge[motor + 1], (rotation + 1) % 2);

  int speed = (mps / ((150 / 60) * (260 * M_PI) / 1000)) * 255;
  analogWrite(enA, speed);
}

void checkWay(float distance){  
  if (distance > minRange) {
    if (robotState != 1){
      timer[0] = millis();
      robotState = 1;
      motorControl(motorL, forward, defaultSpeed);
      motorControl(motorR, forward, defaultSpeed);
      Serial.println("Move");
    }
  } else {
    if (robotState != 2){
      robotState = 2;
      motorControl(motorL, forward, 0);
      motorControl(motorR, forward, 0);
      Serial.println("Stop");
    }
  }
}

long timing(int robotState){
  if(robotState == 1){
      timer[1] = millis()-timer[0]; 
      timer[3] = timer[2] + timer[1];
  }
  else if(robotState == 2){
    timer[2] = timer[3];
  }
  return timer[3];
}

void turn(int orientation){ 

  float turnSpect[4]; // 0 : distance roue interieur ; 1 : distance roue exterieure ; 2 : vitesse roue interieur ; 3 : vitesse roue exterieur
  turnSpect[0] = M_PI * ((largRoue/2) + 0.25 - (largRobot/2));
  turnSpect[1] = M_PI * ((largRoue/2) + 0.25 + (largRobot/2));
  
  turnSpect[2] = ((turnSpect[0] / turnSpeed) / ((150 / 60) * (260 * M_PI) / 1000)) * 255;
  turnSpect[3] = ((turnSpect[1] / turnSpeed) / ((150 / 60) * (260 * M_PI) / 1000)) * 255;

  if(orientation > 0){
    motorControl(motorR, backward, turnSpect[2]);
    motorControl(motorL, forward, turnSpect[3]);
    Serial.println("Right turn end correcly");
  }
  
  else if(orientation < 0){
    motorControl(motorL, backward, turnSpect[2]);
    motorControl(motorR, forward, turnSpect[3]);
    Serial.println("Left turn end correcly");
  }

  delay(1000*turnSpeed);

  checkWay(0);

  delay(500);

  parcours[1] += largRobot; // largeur de la rampe

  if(parcours[1] - largRobot >= width){
    robotState = 3;
    Serial.println("Nettoyage terminé !");  
  }

  for(int i = 0; i < 4; i++){  // Sert à reset les timer de déplacement
    timer[i] = 0;              //
  }                            //
  timer[0] = millis();         //
}
// ### FONCTION POUR CATPEUR BLUETOOTH ###
bool checkZone(){   // verrifie si la valeur est bien un chiffre
  if  (value<0 || value>9){
    Serial.println("not good");
    Serial.println(val);
    Serial1.print("*"+String(val)+String("Incorrect values")+"*");
    return false;
  }
  return true;
}
void good(){
  Serial1.print("*"+String(val)+String("Good")+"*");
  if (val==1){goodWidth=true;}
  else{goodLength=true;}
}
void setValue(){
  if(val==1){width=width*rank+value;}
  else {length=length*rank+value;}
}
void setSetup(){
  if (Serial1.available()){  //si on capte le bluetooth : 
    value=Serial1.read()-48; //on lit la valeur que l'application envoie
    if(value==60){          //si c'est cette valeur, ca correspond à la largeur
      again=true;
      goodWidth=false;      // la largeur n'est plus bonne
      Serial.println("It is a width");
      val=1;
      rank=0.1;
      width=0;
    }
    else if (value==28){     //si c'est cette valeur, ca correspond à la longueur
      again=true;
      goodLength=false;      //la longueur n'est plus bonne
      Serial.println("It is a length");
      val=2;
      rank=0.1;
      length=0;
    } 
    else if (value==22){     //si c'est la fin de la reception 
      Serial.print("The width is : ");
      Serial.println(width);
      Serial.print("The length is : ");
      Serial.println(length);
    }
    else if (!checkZone()){// si la valeur n'est pas un chiffre correcte
      again=false;
    }
    else if (again){   // on regarde les chiffres envoyés
      rank*=10;
      setValue();
      good();
    }
  }
}
//Verrifie que tout soit bon pour commencer un cycle de nettoyage
void checkStartCycle(){
  if (Serial1.available()){
    val=Serial1.read();
    if(goodLength && goodWidth && val=="m"){ 
      robotState=0;
    }
  }
}
//Si la longueur et la largeur sont bonne regarde dans quel état est le robot (arret ou marche)
void changeRobotState(){
  if(goodLength && goodWidth){   
    if (Serial1.available()){
      val=Serial1.read();
      if (val=="m"){robotState=1;}
      else if (val=="p"){robotState=2;}
    }
  }
}


// ### FONCTION POUR CAPTEUR ULTRA-SON ###

float USrange() {
  long duration;
  float distance;
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  return distance;
}

void sort(float arr[]) {
  for (int i = 0; i < memorySize - 1; i++) {
    for (int j = 0; j < memorySize - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        int temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

float mesure(){
  float distSort[memorySize];

  for(int i=0; i<memorySize-1; i++){
    distMemory[i] = distMemory[i+1];
  }

  distMemory[memorySize-1] = USrange();

  for (int i = 0; i < memorySize; i++) {
    distSort[i] = distMemory[i];
  }

  sort(distSort);

  return distSort[int(memorySize / 2)];
}

void reset(){

  timeSave = 0;

  for(int i = 0; i < 4; i++){  // Sert à reset les timer de déplacement
    timer[i] = 0;              //
  }       

  for(int i = 0; i < 2; i++){  // Sert à reset les distances parcourus
    parcours[i] = 0;           //
  }  

  distance = 0;
  orientation = 1;

  for (int i = 0; i < memorySize; i++) {
    distMemory[i] = 0;
  }

  length = 0;
  width = 0;
}
