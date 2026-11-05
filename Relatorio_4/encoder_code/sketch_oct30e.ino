
#include <Servo.h>
#include <Encoder.h>

Servo motor;
Encoder meuEncoder(2, 3);

const int PINO_SERVO = 9;

// Calibracao
const float K = 0.6094;

// Servo
const int NEUTRO = 1500;

// Ganhos PI
const float Kp = 5.0;
const float Ki = 2.0;
const float Kd = 0.0;

// Controle
float integral = 0.0;
float setpoint = 0.0;
float setpointAnterior = 0.0;

// Amostragem
const unsigned long INTERVALO = 50000UL;

// Media movel
const int JANELA = 5;
float buffer[JANELA] = {0};
float soma = 0.0;
int indice = 0;

// Encoder
long ticksAnteriores = 0;

// Tempo
unsigned long tempoAnterior = 0;
unsigned long inicioTeste = 0;

bool terminou = false;

void setup() {
  Serial.begin(115200);

  motor.attach(PINO_SERVO);
  motor.writeMicroseconds(NEUTRO);

  meuEncoder.write(0);

  Serial.println("Teste PI - Inversao de sentido");

  delay(3000);

  ticksAnteriores = meuEncoder.read();
  tempoAnterior = micros();
  inicioTeste = millis();

  Serial.println(
    "Tempo_ms,Setpoint,VelocidadeBruta,"
    "VelocidadeFiltrada,Erro,Pulso,Ticks,Distancia_mm"
  );
}

void loop() {
  if (terminou) {
    return;
  }

  unsigned long tempo = millis() - inicioTeste;

  // Referencia de velocidade
  if (tempo < 1000) {
    setpoint = 0.0;
  }
  else if (tempo < 5000) {
    setpoint = 100.0;
  }
  else if (tempo < 9000) {
    setpoint = -100.0;
  }
  else {
    motor.writeMicroseconds(NEUTRO);
    terminou = true;
    Serial.println("FIM");
    return;
  }

  // Zera integral nas mudancas
  if (setpoint != setpointAnterior) {
    integral = 0.0;
    setpointAnterior = setpoint;
  }

  unsigned long agora = micros();
  unsigned long dt = agora - tempoAnterior;

  if (dt >= INTERVALO) {
    long ticksAtuais = meuEncoder.read();

    long deltaTicks =
      ticksAtuais - ticksAnteriores;

    float dtSegundos =
      dt / 1000000.0;

    // Velocidade bruta
    float velocidadeBruta =
      (deltaTicks * K) / dtSegundos;

    // Media movel
    soma -= buffer[indice];
    buffer[indice] = velocidadeBruta;
    soma += buffer[indice];

    indice = (indice + 1) % JANELA;

    float velocidadeFiltrada =
      soma / JANELA;

    // Erro
    float erro =
      setpoint - velocidadeFiltrada;

    // Integral
    integral += erro * dtSegundos;

    float limiteIntegral = 250.0 / Ki;

    integral = constrain(
      integral,
      -limiteIntegral,
      limiteIntegral
    );

    // Controlador PI
    float correcao =
      Kp * erro + Ki * integral;

    correcao = constrain(
      correcao,
      -250.0,
      250.0
    );

    int pulso =
      NEUTRO + (int)correcao;

    pulso = constrain(
      pulso,
      1000,
      2000
    );

    if (setpoint == 0.0) {
      pulso = NEUTRO;
      integral = 0.0;
    }

    motor.writeMicroseconds(pulso);

    // Distancia assinada
    float distancia =
      ticksAtuais * K;

    // Registro CSV
    Serial.print(tempo);
    Serial.print(",");

    Serial.print(setpoint, 2);
    Serial.print(",");

    Serial.print(velocidadeBruta, 2);
    Serial.print(",");

    Serial.print(velocidadeFiltrada, 2);
    Serial.print(",");

    Serial.print(erro, 2);
    Serial.print(",");

    Serial.print(pulso);
    Serial.print(",");

    Serial.print(ticksAtuais);
    Serial.print(",");

    Serial.println(distancia, 3);

    ticksAnteriores = ticksAtuais;
    tempoAnterior = agora;
  }
}
