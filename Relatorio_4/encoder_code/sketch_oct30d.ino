
#include <Servo.h>
#include <Encoder.h>

Servo motor;
Encoder meuEncoder(2, 3);

const int PINO_SERVO = 9;
const int PULSO = 1000;

const float K = 0.6094; // mm por tick
const unsigned long INTERVALO = 50000; // 50 ms

long ticksAnteriores = 0;
unsigned long tempoAnterior = 0;

void setup() {
  Serial.begin(115200);

  motor.attach(PINO_SERVO);
  motor.writeMicroseconds(1500);

  meuEncoder.write(0);

  delay(2000);

  // Comeca o experimento
  ticksAnteriores = meuEncoder.read();
  tempoAnterior = micros();

  motor.writeMicroseconds(PULSO);

  Serial.println("Tempo_ms,Ticks,Velocidade_mm_s");
}

void loop() {
  unsigned long agora = micros();
  unsigned long dt = agora - tempoAnterior;

  if (dt >= INTERVALO) {
    long ticksAtuais = meuEncoder.read();
    long deltaTicks = ticksAtuais - ticksAnteriores;

    float velocidade =
        (deltaTicks * K) / (dt / 1000000.0);

    Serial.print(millis());
    Serial.print(",");

    Serial.print(ticksAtuais);
    Serial.print(",");

    Serial.println(velocidade, 2);

    ticksAnteriores = ticksAtuais;
    tempoAnterior = agora;
  }
}
