
#include <Encoder.h>

Encoder meuEncoder(2, 3);

// Constantes medidas na montagem
const float K = 0.6094;  // mm por tick
const unsigned long INTERVALO = 50000; // 50 ms

// Variaveis
long ticksAnteriores = 0;
unsigned long tempoAnterior = 0;

void setup() {
  Serial.begin(115200);

  meuEncoder.write(0);
  tempoAnterior = micros();


  Serial.println("Ticks,Distancia_mm,Velocidade_mm_s,DeltaT_ms");
}

void loop() {

  unsigned long agora = micros();
  unsigned long dt = agora - tempoAnterior;

  // Executa a cada 50 ms
  if (dt >= INTERVALO) {

    // Leitura atual do encoder
    long ticksAtuais = meuEncoder.read();

    // Diferenca entre leituras
    long deltaTicks = ticksAtuais - ticksAnteriores;

    // Distancia acumulada em mm
    float distancia = ticksAtuais * K;

    // Intervalo real em segundos
    float dtSegundos = dt / 1000000.0;

    // Velocidade em mm/s
    float velocidade = (deltaTicks * K) / dtSegundos;

    // Mostra apenas quando houver movimento
    if (deltaTicks != 0) {

      Serial.print(ticksAtuais);
      Serial.print(",");

      Serial.print(distancia, 3);
      Serial.print(",");

      Serial.print(velocidade, 2);
      Serial.print(",");

      Serial.println(dt / 1000.0, 2);
    }

    // Atualiza as leituras anteriores
    ticksAnteriores = ticksAtuais;
    tempoAnterior = agora;
  }
}
