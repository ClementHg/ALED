#include "RTClib.h"
#include <SoftwareSerial.h>

#define DELAY 1000
#define VIN 5
#define R 10000

const byte BROCHE_CAPTEUR = 5;

const byte DHT_SUCCESS = 0;
const byte DHT_TIMEOUT_ERROR = 1;
const byte DHT_CHECKSUM_ERROR = 2;

const int sensorPin = A0;

int motor_state = 1;
int auto_on = 0;

int in1 = 12;
int in2 = 13;
int ena = 6;

int sensorVal;
int lux;

volatile  float temperature, humidity;


RTC_DS1307 rtc;

char val;

void setup (){
  Serial.begin(9600);
  pinMode(BROCHE_CAPTEUR, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);

  while (!Serial);
 
  
  while (! rtc.begin()) {
    Serial.println("Attente du module RTC...");
    delay(1000);
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
 
  Serial.println("Horloge du module RTC mise a jour");
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
}

void loop(){
  sensorVal = analogRead(sensorPin);
  lux=sensorRawToPhys(sensorVal);
  Serial.print("Raw value from sensor= ");
  Serial.println(sensorVal);
  Serial.print("Physical value from sensor = ");
  Serial.print(lux);
  Serial.println(" lumen");
  Serial.println("");
  DateTime now = rtc.now();
  char heure[10];
  
  sprintf(heure, "%02d:%02d:%02d %02d/%02d/%02d", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
  Serial.println(heure);
  Serial.println("");
  
 
  switch (readDHT22(BROCHE_CAPTEUR, &temperature, &humidity)) {
  case DHT_SUCCESS: 
     
    /* Affichage de la température et du taux d'humidité */
    Serial.print(F("Humidite (%): "));
    Serial.println(humidity, 2);
    Serial.print(F("Temperature (^C): "));
    Serial.println(temperature, 2);
    break;
 
  case DHT_TIMEOUT_ERROR: 
    Serial.println(F("Pas de reponse !")); 
    break;
 
  case DHT_CHECKSUM_ERROR: 
    Serial.println(F("Pb de communication !")); 
    break;
  }
  Serial.println("");

  if (Serial.available()) {
    val=Serial.read(); 
    Serial.println(val); // Use the IDE's Tools > Serial Monitor
    parseCommand(val, temperature, lux); // parse the input
  }
  
  if (digitalRead(8) == LOW || auto_on == 0){
    aled(temperature, lux);
  }
  
  if (digitalRead(9) == LOW){
    motor_down();
  }
  
  if (digitalRead(10) == LOW){
    motor_up();
  }
  
  Serial.println(motor_state);
  Serial.println("");
  
  delay(DELAY);
}

byte readDHT22(byte pin, float* temperature, float* humidity) {
  
  /* Lit le capteur */
  byte data[5];
  byte ret = readDHTxx(pin, data, 1, 1000);
  
  /* Détecte et retourne les erreurs de communication */
  if (ret != DHT_SUCCESS) 
    return ret;
    
  /* Calcul la vraie valeur de la température et de l'humidité */
  float fh = data[0];
  fh *= 256;
  fh += data[1];
  fh *= 0.1;
  *humidity = fh;
 
  float ft = data[2] & 0x7f;
  ft *= 256;
  ft += data[3];
  ft *= 0.1;
  if (data[2] & 0x80) {
    ft *= -1;
  }
  *temperature = ft;

  /* Ok */
  return DHT_SUCCESS;
}

byte readDHTxx(byte pin, byte* data, unsigned long start_time, unsigned long timeout) {
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  // start_time est en millisecondes
  // timeout est en microsecondes
 
  /* Conversion du numéro de broche Arduino en ports / masque binaire "bas niveau" */
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *ddr = portModeRegister(port);   // Registre MODE (INPUT / OUTPUT)
  volatile uint8_t *out = portOutputRegister(port); // Registre OUT (écriture)
  volatile uint8_t *in = portInputRegister(port);   // Registre IN (lecture)
  
  /* Conversion du temps de timeout en nombre de cycles processeur */
  unsigned long max_cycles = microsecondsToClockCycles(timeout);
 
  /* Evite les problèmes de pull-up */
  *out |= bit;  // PULLUP
  *ddr &= ~bit; // INPUT
  delay(100);   // Laisse le temps à la résistance de pullup de mettre la ligne de données à HIGH
 
  /* Réveil du capteur */
  *ddr |= bit;  // OUTPUT
  *out &= ~bit; // LOW
  delay(start_time); // Temps d'attente à LOW causant le réveil du capteur
  // N.B. Il est impossible d'utilise delayMicroseconds() ici car un délai
  // de plus de 16 millisecondes ne donne pas un timing assez précis.
  
  /* Portion de code critique - pas d'interruptions possibles */
  noInterrupts();
  
  /* Passage en écoute */
  *out |= bit;  // PULLUP
  delayMicroseconds(40);
  *ddr &= ~bit; // INPUT
 
  /* Attente de la réponse du capteur */
  timeout = 0;
  while(!(*in & bit)) { /* Attente d'un état LOW */
    if (++timeout == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
  }
    
  timeout = 0;
  while(*in & bit) { /* Attente d'un état HIGH */
    if (++timeout == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
  }

  /* Lecture des données du capteur (40 bits) */
  for (byte i = 0; i < 40; ++i) {
 
    /* Attente d'un état LOW */
    unsigned long cycles_low = 0;
    while(!(*in & bit)) {
      if (++cycles_low == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }

    /* Attente d'un état HIGH */
    unsigned long cycles_high = 0;
    while(*in & bit) {
      if (++cycles_high == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }
    
    /* Si le temps haut est supérieur au temps bas c'est un "1", sinon c'est un "0" */
    data[i / 8] <<= 1;
    if (cycles_high > cycles_low) {
      data[i / 8] |= 1;
    }
  }
  
  /* Fin de la portion de code critique */
  interrupts();
 
  /*
   * Format des données :
   * [1, 0] = humidité en %
   * [3, 2] = température en degrés Celsius
   * [4] = checksum (humidité + température)
   */
   
  /* Vérifie la checksum */
  byte checksum = (data[0] + data[1] + data[2] + data[3]) & 0xff;
  if (data[4] != checksum)
    return DHT_CHECKSUM_ERROR; /* Erreur de checksum */
  else
    return DHT_SUCCESS; /* Pas d'erreur */
}

int sensorRawToPhys(int raw){
  
  float Vout = float(raw) * (VIN / float(1023));
  float RLDR = (R * (VIN - Vout))/Vout;
  int phys=500/(RLDR/1000);
  return phys;
}

void parseCommand(char input,int temperature,int lux) {
  switch (input) {
    case '1': // LEFT
      //turnLeft();   
      break;
    case '2': // UP
      //setVelocity(1);
      break;
    case '3': // RIGHT
      //turnRight();
      break;
    case '4': // DOWN
      //setVelocity(-1)
      break;
    case '5': // SELECT
      //toggleMode();
      break;
    case '6': // START
      //pause();
      break;  
    case 's': // SQUARE
      motor_down();
      break; 
    case 't': // TRIANGLE
      motor_up();
      break;  
    case 'x': // X
      aled(temperature, lux);
      break;
    case 'c': // CIRCLE
      //crawl();
      break;
  }
}

void motor_up () {
  digitalWrite(3, HIGH);
  digitalWrite(2, LOW);
  if (motor_state == 1) {
    digitalWrite(in1, 0);
    digitalWrite(in2, 1);
    digitalWrite(ena, abs(200));
    delay(2000);
    digitalWrite(in1, 0);
    digitalWrite(in2, 1);
    digitalWrite(ena, abs(0));
  }
  motor_state = 2;
  auto_on = 1;
}

void motor_down () {
  digitalWrite(2, HIGH);
  digitalWrite(3, LOW);
  if (motor_state == 2) {
    digitalWrite(in1, 1);
    digitalWrite(in2, 0);
    digitalWrite(ena, abs(200));
    delay(2000);
    digitalWrite(in1, 0);
    digitalWrite(in2, 1);
    digitalWrite(ena, abs(0));
  }
  motor_state = 1;
  auto_on = 1;
}

void aled(int temperature,int  lux){
  DateTime now = rtc.now();

  if (now.hour() <= 7 || (now.hour() >= 19)){
    if (motor_state == 2) {
      motor_down(); 
    }
  }
  else{
    if (motor_state == 2){
        if (temperature > 19){
          if (lux > 90){
            motor_down();
              
          }
        }
    }
    if (motor_state == 1) {
      
        if (temperature > 19){
          if (lux < 90){
            motor_up();
          }
        }
        else {
          motor_up();
        }
    }
  }
  auto_on = 0;
}
