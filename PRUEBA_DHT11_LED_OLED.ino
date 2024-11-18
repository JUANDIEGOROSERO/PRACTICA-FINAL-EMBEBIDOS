#include <heltec_unofficial.h>
#include <Wire.h>
#include <DHT.h>

#define oledDisplay display
#define DHTPIN 19              // Pin donde está conectado el DHT11
#define DHTTYPE DHT11          // Tipo de sensor DHT
#define TEMP_MAX 25.00         // Temperatura máxima en grados Celsius
#define HUM_MIN 40.00          // Humedad mínima en %

DHT dht(DHTPIN, DHTTYPE);      // Inicializa el sensor DHT11

const int ledRojo = 21;        // Pin del LED rojo
const int ledAzul = 20;        // Pin del LED azul

void setup() {
  Serial.begin(115200);

  // Inicialización de la pantalla OLED
  heltec_setup(); // Configuración específica de Heltec
  oledDisplay.setFont(ArialMT_Plain_10);
  oledDisplay.clear();
  
  // Inicia el sensor DHT11
  dht.begin();
  
  // Configuración de pines de los LEDs
  pinMode(ledRojo, OUTPUT);
  pinMode(ledAzul, OUTPUT);
}

void drawFontFaceDemo(float temp, float hum) {
  oledDisplay.clear();
  oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);

  oledDisplay.drawString(0, 0, "Temperatura: " + String(temp) + " C");
  oledDisplay.drawString(0, 15, "Humedad: " + String(hum) + " %");
  oledDisplay.display();
}

void loop() {
  float temp = dht.readTemperature();  // Lee la temperatura
  float hum = dht.readHumidity();      // Lee la humedad

  // Validación de errores en las lecturas del sensor
  if (isnan(temp) || isnan(hum)) {
    Serial.println("Error al leer del sensor DHT!");
    return;
  }

  // Formatea los valores a 2 decimales
  temp = round(temp * 100) / 100.0;
  hum = round(hum * 100) / 100.0;

  // Muestra los valores en el monitor serial
  Serial.print("Temperatura: ");
  Serial.print(temp);
  Serial.print(" °C, Humedad: ");
  Serial.print(hum);
  Serial.println(" %");

  // Controla los LEDs según las condiciones de temperatura y humedad
  digitalWrite(ledRojo, temp > TEMP_MAX ? HIGH : LOW);
  digitalWrite(ledAzul, hum < HUM_MIN ? HIGH : LOW);

  // Llama a la función pasando las variables
  drawFontFaceDemo(temp, hum);

  delay(10000); // Retraso de 10 segundos entre lecturas
}