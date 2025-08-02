#include "dht_sensor/dht_sensor.h"
#include "screens/screens.h" // Para drawMainScreen()

// Função para ler o sensor DHT11
void readDHTSensor() {
  // A leitura do sensor pode levar alguns milissegundos.
  // Não use delays longos no loop principal.
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Verifica se alguma leitura falhou e exibe um erro
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Falha ao ler do sensor DHT!"));
    temperature = NAN; // Define como Not-a-Number para indicar erro
    humidity = NAN;
  } else {
    temperature = t;
    humidity = h;
    Serial.print(F("Umidade: "));
    Serial.print(humidity);
    Serial.print(F("%  Temperatura: "));
    Serial.print(temperature);
    Serial.println(F("°C"));
  }
  
  // Redesenha a tela principal para atualizar os valores
  // Esta chamada é importante para que a UI reflita as leituras.
  drawMainScreen();
}
