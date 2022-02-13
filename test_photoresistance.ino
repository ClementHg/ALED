#define DELAY 1000
#define VIN 5             // On défini la tension d'entrée
#define R 10000           // Ainsi que la valeur de la résistance

const int sensorPin = A0;

int sensorVal;
int lux;

void setup() {
  Serial.begin(9600);

}

void loop() {
  sensorVal = analogRead(sensorPin);              // On lit la valeur de renvoyé par le capteur
  lux=sensorRawToPhys(sensorVal);                 // On applique la fonction pour déterminer le nombre de lumen
  Serial.print("Raw value from sensor= ");
  Serial.println(sensorVal);
  Serial.print("Physical value from sensor = ");
  Serial.print(lux);
  Serial.println(" lumen");
  Serial.println("");
  delay(DELAY);
}

int sensorRawToPhys(int raw){
  
  float Vout = float(raw) * (VIN / float(1023));   // On fais plusieurs calcul en fonction de
  float RLDR = (R * (VIN - Vout))/Vout;            // La valeur relevée par le capteur, la
  int phys=500/(RLDR/1000);                        // Tension d'alimentation et la valeur de la
  return phys;                                     // Résistance
}
