#include "FuntechStepper.h" // Bibliothèques des steppers
#include <APA102.h>         // Bibliothèque des leds
#include <SoftwareSerial.h>

FuntechStepper stepper1 = FuntechStepper(0, 0, 4, FTS_BACKWARD); // Stepper fixe
FuntechStepper stepper2 = FuntechStepper(0, 0, 5, FTS_BACKWARD); // Stepper mobile

SoftwareSerial ArduinoMaster(2, 3); // RX->D3 TX->D2
String msg;

/// Leds
const uint8_t dataPin = 11;         // pin DO
const uint8_t clockPin = 12;        // pin CO
APA102<dataPin, clockPin> ledStrip; // instance du bandeau de leds
const int ledCount = 92;            // 89 réelles + 3 virtuelles pour finir le motif
const uint8_t brightness = 5;
rgb_color colors[ledCount];

int tabIndiceLeds[16] = {0, 1, 12, 13, 23, 24, 35, 36, 46, 47, 58, 59, 69, 70, 81, 82};
int compteurLGBT = 0; // compteur pour la couleur des leds

/// Drapeaux
bool phareDepose = true;
bool phareDeploye = false;
bool phareAllume = false;

/// Pour les ordres bluetooth
bool ordreDeploiement = false;
bool ordreRangement = false;

void setup() {
  Serial.begin(9600);
  stepper1.enable();
  stepper2.enable();

  ledStrip.write(colors, ledCount, 0); // On éteint les leds (brightness nulle)
}

void translationMoteurs(int direction) {

  stepper1.setDirection(direction);
  stepper2.setDirection(direction);
  for (long i = 0; i < 150000; i++) { // 8 mm par tour environ - 200 stp/ tr
    stepper1.step();
    stepper2.step();
    delayMicroseconds(50);
  }
}

void balayageLumiere() {
  /// reset des couleurs
  // on éteint toutes les leds
  for (int b = 0; b < ledCount; b++) {
    colors[b] = rgb_color(0, 0, 0); // noir = éteint
  }

  /// LGBT
  compteurLGBT = (compteurLGBT + 1) % 6; // Position dans l'animation arc-en-ciel
  switch (compteurLGBT) {
  case 0:
    for (int i = 0; i < 16; i++) {
      colors[tabIndiceLeds[i]] = rgb_color(228, 3, 3);
    }
    break;

  case 1:
    for (int i = 0; i < 16; i++) {
      colors[tabIndiceLeds[i]] = rgb_color(255, 140, 0);
    }
    break;

  case 2:
    for (int i = 0; i < 16; i++) {
      colors[tabIndiceLeds[i]] = rgb_color(255, 237, 0);
    }
    break;

  case 3:
    for (int i = 0; i < 16; i++) {
      colors[tabIndiceLeds[i]] = rgb_color(0, 128, 38);
    }
    break;

  case 4:
    for (int i = 0; i < 16; i++) {
      colors[tabIndiceLeds[i]] = rgb_color(0, 77, 255);
    }
    break;

  case 5:
    for (int i = 0; i < 16; i++) {
      colors[tabIndiceLeds[i]] = rgb_color(117, 7, 135);
    }
    break;

  default:
    break;
  }

  ledStrip.write(colors, ledCount, brightness); /// Animation des leds
  delay(50);                                    // tempo dans la vitesse du balayage

  for (int i = 0; i < 16; i++) { // translation des leds , incrémentation des indices
    tabIndiceLeds[i] = (tabIndiceLeds[i] + 1) % ledCount;
  }
}

void loop() {
  while (ArduinoMaster.available()) {
    delay(10);
    if (ArduinoMaster.available() > 0) {
      char c = ArduinoMaster.read(); // gets one byte from serial buffer
      if (c == ';') {
        if (msg == "deploy") {
          if (!phareDeploye) {
            // Serial.println("Montée");
            translationMoteurs(FTS_BACKWARD); // montée
            phareDeploye = true;
          } else
            ;

          /// Allumage
          phareAllume = true;
          while (phareDeploye && !ordreRangement) {
            balayageLumiere();
          }

          if (phareDeploye && ordreRangement) { // Rangement

            /// Extinction du phare
            // On passe les leds en noir
            for (int b = 0; b < ledCount; b++) {
              colors[b] = rgb_color(0, 0, 0); // noir = éteint
            }
            // Extinction des leds
            ledStrip.write(colors, ledCount, brightness);

            // Serial.println("Descente");
            translationMoteurs(FTS_FORWARD); // translation vers le bas
          }
        }
        if (msg == "demandeStatus") {
          msg = " phare deployé : " + String(phareDeploye) + " phare allumé : " + String(phareAllume) + " Phare deposé : " + String(phareDepose);
          ArduinoMaster.print(msg);
        }
      }
      msg += c; // makes the string readString
    }
  }
  ArduinoMaster.flush();
}
