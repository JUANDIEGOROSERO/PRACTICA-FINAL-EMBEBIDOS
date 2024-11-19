#include "LoRaWan_APP.h"
#include <DHT.h>
#include "HT_SSD1306Wire.h"
#include <TinyGPS++.h>

#define DHTPIN 19              // Pin donde está conectado el DHT11
#define DHTTYPE DHT11          // Tipo de sensor DHT
#define TEMP_MAX 25.00         // Temperatura máxima en grados Celsius
#define HUM_MIN 40.00          // Humedad mínima en %

#define PIN_SENSOR_IR 34      // Pin para el sensor infrarrojo FC-51
#define PIN_BOTON_RESET 35    // Pin para el botón de reset
#define PIN_LED 47            // Pin para el LED de estado

// Define the RX and TX pins for Serial 2
#define RXD2 6
#define TXD2 7
#define GPS_BAUD 9600

TinyGPSPlus gps;               // Objeto TinyGPS
HardwareSerial gpsSerial(2);
DHT dht(DHTPIN, DHTTYPE);      // Inicializa el sensor DHT11

const int ledRojo = 45;        // Pin del LED rojo
const int ledAzul = 46;        // Pin del LED azul

const int pinPotenciometro = 5;       // Pin GPIO-5 donde está conectado el potenciómetro
const float referenciaVoltaje = 3.3;  // Voltaje de referencia del ESP32 (3.3V)
float voltaje = 0.0;

// Variables globales
volatile int conteoBloqueos = 0;          // Conteo de bloqueos detectados
volatile bool reiniciarConteo = false;    // Indicador para reiniciar el conteo


// Configuración del temporizador
hw_timer_t * myTimer = NULL;
volatile bool ledEstado = false;

// Variable para controlar el cambio de pantalla
volatile bool mostrarPantalla2 = false;
unsigned long tiempoInicioPantalla2 = 0;        // Tiempo de inicio de la pantalla 2
volatile unsigned long ultimaInterrupcion = 0;  // Tiempo de la última interrupción
const unsigned long tiempoDebounce = 200;       // Tiempo mínimo entre detecciones en ms

// Prototipos de funciones
void IRAM_ATTR detectarBloqueo();  // Interrupción para el sensor infrarrojo
void IRAM_ATTR resetearConteo();   // Interrupción para el botón de reset
void IRAM_ATTR gestionarLED();     // Interrupción para el Led

void showTHV(float temp, float hum, float volt);            // Prototipo de la función mostrar temperatura, humedad y voltaje
void showBLL(int conteoBloqueos, double lat, double log);  // Prototipo de mostrar bloqueos, latitud y longitud


// Configuracion de pantalla
SSD1306Wire oledDisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

void setup() {
  Serial.begin(115200);

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

  // Inicializar la pantalla oled
  oledDisplay.init();
  oledDisplay.setFont(ArialMT_Plain_10);

  // Configurar pines
  pinMode(PIN_SENSOR_IR, INPUT);           // Pin del sensor como entrada
  pinMode(PIN_BOTON_RESET, INPUT_PULLUP);  // Pin del botón como entrada con pull-up
  pinMode(PIN_LED, OUTPUT);                // Pin del led como salida

  // Configurar interrupciones
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_IR), detectarBloqueo, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON_RESET), resetearConteo, FALLING);

  // Configurar monitor serial
  Serial.begin(115200);
  Serial.println("Sistema iniciado. Esperando detección...");

  // Inicialización del temporizador de hardware del ESP32
  myTimer = timerBegin(0, 80, true);                  // Temporizador 0, prescaler 80 (1 tick = 1us)
  timerAttachInterrupt(myTimer, &gestionarLED, true); // Asignar la interrupción
  timerAlarmWrite(myTimer, 750000, true);             // Temporizador para 750 ms
  timerAlarmEnable(myTimer);                          // Habilitar alarma del temporizador

  // Inicia el sensor DHT11
  dht.begin();
  
  // Configuración de pines de los LEDs
  pinMode(ledRojo, OUTPUT);
  pinMode(ledAzul, OUTPUT);
}

void loop() {

  if (reiniciarConteo) {
    noInterrupts();                         // Deshabilitar interrupciones temporalmente
    conteoBloqueos = 0;                     // Reiniciar el conteo
    Serial.print("Bloqueos detectados: ");
    Serial.println(conteoBloqueos);
    reiniciarConteo = false;
    interrupts();                           // Habilitar interrupciones nuevamente
    Serial.println("Conteo reiniciado.");
  }
  // Espera activa; interrupciones manejan los eventos.
  
  delay(10);  // Pequeño retraso para evitar uso excesivo de CPU
  
  while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
  }
  double latitud = gps.location.lat();  // Lee la latitud
  double longitud = gps.location.lng(); // Lee la longitud
  
  float temp = dht.readTemperature();  // Lee la temperatura
  float hum = dht.readHumidity();      // Lee la humedad

  // Validación de errores en las lecturas del sensor
  if (isnan(temp) || isnan(hum)) {
    Serial.println("Error al leer del sensor DHT!");
    return;
  }

  // Controla los LEDs según las condiciones de temperatura y humedad
  digitalWrite(ledRojo, temp > TEMP_MAX ? HIGH : LOW);
  digitalWrite(ledAzul, hum < HUM_MIN ? LOW : HIGH);

  int lecturaADC = analogRead(pinPotenciometro);              // Lee el valor del ADC (0-4095)
  float voltaje = (lecturaADC / 4095.0) * referenciaVoltaje;  // Convierte la lectura a voltaje
  
  // Muestra los valores en el monitor serial
  Serial.print("Temperatura: ");
  Serial.print(temp);
  Serial.print(" °C, Humedad: ");
  Serial.print(hum);
  Serial.println(" %");
  Serial.print(longitud, 6);
  Serial.println("..");
  Serial.print(latitud, 6);
  Serial.println("..");
  Serial.print("Voltaje: ");
  Serial.print(voltaje, 2);
  Serial.println(" V");

  // Maneja el cambio de pantalla
  if(mostrarPantalla2){
    showBLL(conteoBloqueos, longitud, latitud);
    if (millis() - tiempoInicioPantalla2 > 250) {
      mostrarPantalla2 = false;   // Vuelve a pantalla 1 después de 250 ms
    }
  }else{
    showTHV(temp, hum, voltaje);  // Muestra pantalla 1
  }

}

// Función de interrupción para aumentar bloqueos
void IRAM_ATTR detectarBloqueo() {
  
  unsigned long tiempoActual = millis();
  // Verifica si ha pasado suficiente tiempo desde la última detección
  
  if (tiempoActual - ultimaInterrupcion > tiempoDebounce) {
    ultimaInterrupcion = tiempoActual;  // Actualiza el tiempo de la última interrupción
    conteoBloqueos++;                   // Incrementa el conteo
    mostrarPantalla2 = true;
    tiempoInicioPantalla2 = tiempoActual;
    Serial.print("Bloqueos detectados: ");
    Serial.println(conteoBloqueos);
  }

}

// Función de interrupción para reiniciar el conteo
void IRAM_ATTR resetearConteo() {

  reiniciarConteo = true;  // Bandera para el reinicio del contador

}

// Función de interrupción del temporizador para gestionar el LED
void IRAM_ATTR gestionarLED() {

  ledEstado = !ledEstado;
  digitalWrite(PIN_LED, ledEstado);  // Cambia el estado del LED

  if (ledEstado) {
    timerAlarmWrite(myTimer, 250000, true);  // Configurar en 250 ms para encender
  }else {
    timerAlarmWrite(myTimer, 750000, true);  // Configurar en 750 ms para apagar
  }

}

// Función para mostrar la pantalla 1
void showTHV(float temp, float hum, float volt) {

  oledDisplay.clear();
  oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
  oledDisplay.drawString(0, 00, "Grupo 5");
  oledDisplay.drawString(0, 10, "Tarjeta 1");
  oledDisplay.drawString(0, 20, "Temperatura: " + String(temp, 2) + " C");
  oledDisplay.drawString(0, 30, "Humedad: " + String(hum, 2) + " %");
  oledDisplay.drawString(0, 40, "Voltaje: " + String(volt, 2)); 
  oledDisplay.display();

}

// Función para mostrar la Pantalla 2
void showBLL(int conteoBloqueos, double lat, double log) {

  oledDisplay.clear();
  oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
  oledDisplay.drawString(0, 0, "Bloqueos detectados: " + String(conteoBloqueos) );
  oledDisplay.drawString(0, 10, "Longitud: " + String(log, 6));
  oledDisplay.drawString(0, 20, "Latencia: " + String(lat, 6)); 
  oledDisplay.display();

}
