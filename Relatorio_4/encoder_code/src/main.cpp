#include <Servo.h>
#include <Arduino.h>

Servo meuServo;

const int pinoServo = 9;

void setup() {
  Serial.begin(9600);

  meuServo.attach(pinoServo);
  meuServo.write(90); // Parado inicialmente

  Serial.println("Controle do Servo");
  Serial.println("0   - Gira em um sentido");
  Serial.println("180 - Gira no outro sentido");
  Serial.println("P   - PARAR");
}

void loop() {
  if (Serial.available() > 0) {

    String comando = Serial.readStringUntil('\n');
    comando.trim();
    comando.toUpperCase();

    // Comando de parada
    if (comando == "P" || comando == "PARAR") {
      meuServo.write(90);
      Serial.println("SERVO PARADO");
      return;
    }

    // Verifica se o comando e numerico
    if (comando.length() == 0) return;

    for (int i = 0; i < comando.length(); i++) {
      if (!isDigit(comando[i])) {
        Serial.println("Comando invalido!");
        return;
      }
    }

    int valor = comando.toInt();

    if (valor >= 0 && valor <= 180) {
      meuServo.writeMicroseconds(valor);

      Serial.print("Comando: ");
      Serial.println(valor);
    } else {
      Serial.println("Valor invalido!");
    }
  }
}