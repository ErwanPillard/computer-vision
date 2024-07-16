#include "ESP8266WiFi.h"
#include "ESP8266WebServer.h"

// Remplacez par les informations de votre réseau WiFi
const char* ssid = "YOUR-WIFI";
const char* password = "YOUR-PASSWORD";

// Crée un serveur web sur le port 80
ESP8266WebServer server(80);

// Variables pour stocker les données reçues
String angle;
String distance;
int vitesse_motor_1 = 160; //moteur différent il ne vont pas a la même vitesse avec une meme tension
int vitesse_motor_2 = 125;


const int motor1_pwm = 5;
const int motor1_dir = 0;
const int motor2_pwm = 4;
const int motor2_dir = 2;

void setup() {
  // Initialise la communication série pour le débogage
  Serial.begin(115200);

  // Initialisation des broches des moteurs en sortie
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
  pinMode(motor2_dir, OUTPUT);

  // Connexion au réseau WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.print(ssid);

  // Attente de la connexion
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Connexion réussie
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Configuration de la route /data pour recevoir les données
  server.on("/data", HTTP_GET, handleData);

  // Démarrage du serveur
  server.begin();
  Serial.println("Server started");
}

void loop() {
  // Gestion des requêtes HTTP
  server.handleClient();

  float angleValue = angle.toFloat();
  float distanceValue = distance.toFloat();

  Serial.println(angleValue);
  Serial.println(distanceValue);

  if (abs(angleValue)>15){
    tournerGauche(angleValue);
  }else {
    stop();
    if(distanceValue>10){
      avancer();
    }else{
      stop();
    }
  }
}

// Fonction pour gérer les requêtes à /data
void handleData() {
  if (server.hasArg("distance") && server.hasArg("angle")) {
    distance = server.arg("distance");
    angle = server.arg("angle");

    // Affiche les données reçues sur le moniteur série
    //Serial.print("Received distance: ");
    //Serial.println(distance);
    //Serial.print("Received angle: ");
    //Serial.println(angle);

    // Répond au client
    server.send(200, "text/plain", "Data received");
  } else {
    server.send(400, "text/plain", "Bad Request: Missing distance or angle");
  }
}


// Fonctions de contrôle des moteurs
void avancer() {
  analogWrite(motor1_pwm, vitesse_motor_1);
  digitalWrite(motor1_dir, HIGH);
  analogWrite(motor2_pwm, vitesse_motor_2);
  digitalWrite(motor2_dir, HIGH);
}

void reculer() {
  analogWrite(motor1_pwm, vitesse_motor_1);
  digitalWrite(motor1_dir, LOW);
  analogWrite(motor2_pwm, vitesse_motor_2);
  digitalWrite(motor2_dir, LOW);
}

void tournerGauche() {
  analogWrite(motor1_pwm, vitesse_motor_1);
  digitalWrite(motor1_dir, LOW);
  analogWrite(motor2_pwm, vitesse_motor_2);
  digitalWrite(motor2_dir, HIGH);
}

void tournerDroite() {
  analogWrite(motor1_pwm, vitesse_motor_1);
  digitalWrite(motor1_dir, HIGH);
  analogWrite(motor2_pwm, vitesse_motor_2);
  digitalWrite(motor2_dir, LOW);
}

void stop() {
  analogWrite(motor1_pwm, LOW);
  digitalWrite(motor1_dir, LOW);
  analogWrite(motor2_pwm, LOW);
  digitalWrite(motor2_dir, LOW);
}