
#include <Encoder.h>

Encoder meuEncoder(2, 3);

long ticks = 0;

void setup() {
  Serial.begin(115200);

  meuEncoder.write(0);

  Serial.println("Contador de ticks");
}

void loop() {
  long leitura = meuEncoder.read();

  if (leitura != ticks) {
    ticks = leitura;

    Serial.print("Ticks: ");
    Serial.println(ticks);
  }
}
