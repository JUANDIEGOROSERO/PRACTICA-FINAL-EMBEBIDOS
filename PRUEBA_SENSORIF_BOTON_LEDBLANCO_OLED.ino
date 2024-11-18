#include <heltec_unofficial.h>
#include <Wire.h>
#include <DHT.h>

#define oledDisplay display
// Configuración de pines
#define PIN_SENSOR_IR 34      // Pin para el sensor infrarrojo FC-51
#define PIN_BOTON_RESET 35    // Pin para el botón de reset
#define PIN_LED 33            // Pin para el LED de estado


// Variables globales
volatile int conteoBloqueos = 0;  // Conteo de bloqueos detectados
volatile bool reiniciarConteo = false;  // Indicador para reiniciar el conteo

// Configuración del temporizador
hw_timer_t * timer = NULL;
volatile bool ledEstado = false;


// Prototipos de funciones
void IRAM_ATTR detectarBloqueo();  // Interrupción para el sensor infrarrojo
void IRAM_ATTR resetearConteo();   // Interrupción para el botón de reset
void IRAM_ATTR gestionarLED();     // Interrupción para el Led

void setup() {
  
  heltec_setup(); // Configuración específica de Heltec
  oledDisplay.setFont(ArialMT_Plain_10);
  oledDisplay.clear();

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
  timer = timerBegin(0, 80, true);              // Temporizador 0, prescaler 80 (1 tick = 1us)
  timerAttachInterrupt(timer, &gestionarLED, true); // Asignar la interrupción
  timerAlarmWrite(timer, 750000, true);         // Temporizador para 750 ms
  timerAlarmEnable(timer);                      // Habilitar alarma del temporizador
}

void loop() {
  // Verificar si se solicitó reiniciar el conteo
  
  if (reiniciarConteo) {
    noInterrupts();         // Deshabilitar interrupciones temporalmente
    conteoBloqueos = 0;     // Reiniciar el conteo
    Serial.print("Bloqueos detectados: ");
    Serial.println(conteoBloqueos);
    reiniciarConteo = false;
    interrupts();           // Habilitar interrupciones nuevamente
    Serial.println("Conteo reiniciado.");
  }

  drawFontFaceDemo(conteoBloqueos);
  // Espera activa; interrupciones manejan los eventos.
  delay(10);  // Pequeño retraso para evitar uso excesivo de CPU
}

// Función de interrupción para detectar bloqueos
void IRAM_ATTR detectarBloqueo() {
  conteoBloqueos++;  // Incrementar el conteo de bloqueos
  Serial.print("Bloqueos detectados: ");
  Serial.println(conteoBloqueos);
}

// Función de interrupción para reiniciar el conteo
void IRAM_ATTR resetearConteo() {
  reiniciarConteo = true;  // Señalar que se debe reiniciar el conteo
}

// Función de interrupción del temporizador para gestionar el LED
void IRAM_ATTR gestionarLED() {
  ledEstado = !ledEstado;
  digitalWrite(PIN_LED, ledEstado);  // Cambia el estado del LED

  if (ledEstado) {
    timerAlarmWrite(timer, 250000, true);  // Configurar en 250 ms para encender
  } else {
    timerAlarmWrite(timer, 750000, true);  // Configurar en 750 ms para apagar
  }
}

void drawFontFaceDemo(int conteoBloqueos) {
  oledDisplay.clear();
  oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);

  oledDisplay.drawString(0, 0, "Bloqueos detectados: " + String(conteoBloqueos) );
  oledDisplay.display();
}