#include "RTClib.h"   // On inclut la bibliothèque RTClib

RTC_DS1307 rtc;

void setup() {
  Serial.begin(9600);

  while (!Serial);
  
  while (! rtc.begin()) {
    Serial.println("Attente du module RTC...");       // On attend une réponse du module RTC lors de l'initialisation
    delay(1000);
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));     // On initialise le module RTC à la date au moment de la compilation
  
  Serial.println("Horloge du module RTC mise a jour");
}

void loop() {
  DateTime now = rtc.now();                           // On lit la valeur indiqué par le module
  char heure[10];
  
  sprintf(heure, "%02d:%02d:%02d %02d/%02d/%02d", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
  Serial.println(heure);                              // Puis on affiche cette valeur en séparant les heures, minutes, secondes, etc...
  Serial.println("");
}
