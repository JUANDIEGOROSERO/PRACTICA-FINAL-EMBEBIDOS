#include "LoRaWan_APP.h"
#include <HTTPClient.h>
#include "HT_SSD1306Wire.h"
#include <TinyGPS++.h>
#include <Wire.h>
#include "DHT.h"

// Configuracion GPS
#define RXD2 6
#define TXD2 7
#define GPS_BAUD 9600
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// Configuracion SENSORES 
#define DHTPIN 19
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

const int pinPotenciometro = 5;   // PIN Potenciometro
const int PIN_BOTON_RESET = 35;   // PIN Boton
const int PIN_SENSOR_IR = 34;     // PIN Sensor infrarojo

// PINES LEDS
const int ledRojo = 45;  // LED Temperatura
const int ledAzul = 46;  // LED Humedad
const int PIN_LED = 47;  // LED Estado

// Configuracion de Pantalla OLED
SSD1306Wire oledDisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); 

// Configuracion LoRaWAN
uint8_t devEui[] = { 0x5B, 0x3B, 0x9F, 0xFA, 0x12, 0xF4, 0x00, 0x00 }; 
uint8_t appEui[] = { 0xEC, 0xFF, 0x9F, 0xFA, 0x12, 0xF4, 0x00, 0x00 }; 
uint8_t appKey[] = { 0xA5, 0x3F, 0x9F, 0x32, 0x47, 0x1A, 0xA2, 0x4A, 0xF6, 0x2E, 0x74, 0xAC, 0xDA, 0x96, 0xBC, 0xA4 };  // TARJETA 1!!

uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;
uint16_t userChannelsMask[6]={ 0xFF00,0x0000,0x0000,0x0000,0x0000,0x0000 };
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t  loraWanClass = CLASS_C;
uint32_t appTxDutyCycle = 10000;
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = true;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

// Configuracion del temporizador
unsigned long now = millis();
unsigned long lastTrigger = 0;
hw_timer_t *myTimer = NULL;   // Puntero al temporizador

// Banderas
boolean IRbandera = false;     // Bandera Sensor Infrarojo para mostrar Pantalla

// Variables globales
volatile unsigned long contadorLed = 0;  // Contador Estado LED

// RTC
RTC_DATA_ATTR int conteoBloqueos;
RTC_DATA_ATTR bool firstrun = true; 

// INFORMACION DEL EQUIPO Y TARJETA
int grupo = 5;
int num_tarjeta = 1;

// VARIABLES A MONITOREAR
double lat =  0;
double logt = 0;
int voltaje = 0;
double hum = 0;
double tem = 0;

// Prototipo de funciones
void IRAM_ATTR gestionarLED();
void IRAM_ATTR detectarBloqueo();
void IRAM_ATTR resetearConteo();
void showBK();


/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
  while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
  }
  
  if (gps.location.isUpdated()) {  
    logt = gps.location.lng();
    lat = gps.location.lat();
  } else{
    lat = 2.445848 ; 
    logt = -76.598559;  // Para las pruebas coordenadas fijas
  }

  voltaje = analogRead(pinPotenciometro);
  hum = dht.readHumidity();
  tem = dht.readTemperature();

  if (hum < 40){
      digitalWrite(ledAzul, HIGH);
  } else{
    digitalWrite(ledAzul, LOW);
  }

  if (tem > 25){
      digitalWrite(ledRojo, HIGH);
  } else{
    digitalWrite(ledRojo, LOW);
  }

  long lati_s;
  long longi_s;
  long hum_s;
  long temp_s;
  lati_s = lat*10000000;
  longi_s= logt*10000000;
  hum_s = hum*10000000;
  temp_s= tem*10000000;
  
      appDataSize = 32;//AppDataSize max value is 64
      // [0..3] 8 bytes
      appData[0] = (byte) ((lati_s & 0xFF000000) >> 24 );
      appData[1] = (byte) ((lati_s & 0x00FF0000) >> 16 );
      appData[2] = (byte) ((lati_s & 0x0000FF00) >> 8 );
      appData[3] = (byte) ((lati_s & 0X000000FF));
      appData[4] = (byte) ((longi_s & 0xFF000000) >> 24 );
      appData[5] = (byte) ((longi_s & 0x00FF0000) >> 16 );
      appData[6] = (byte) ((longi_s & 0x0000FF00) >> 8 );
      appData[7] = (byte) ((longi_s & 0X000000FF));
      appData[8] = (byte) ((voltaje & 0xFF000000) >> 24 );
      appData[9] = (byte) ((voltaje & 0x00FF0000) >> 16 );
      appData[10] = (byte) ((voltaje & 0x0000FF00) >> 8 );
      appData[11] = (byte) ((voltaje & 0X000000FF));
      appData[12] = (byte) ((hum_s & 0xFF000000) >> 24 );
      appData[13] = (byte) ((hum_s & 0x00FF0000) >> 16 );
      appData[14] = (byte) ((hum_s & 0x0000FF00) >> 8 );
      appData[15] = (byte) ((hum_s & 0X000000FF));
      appData[16] = (byte) ((temp_s & 0xFF000000) >> 24 );
      appData[17] = (byte) ((temp_s & 0x00FF0000) >> 16 );
      appData[18] = (byte) ((temp_s & 0x0000FF00) >> 8 );
      appData[19] = (byte) ((temp_s & 0X000000FF));
      appData[20] = (byte) ((grupo & 0xFF000000) >> 24 );
      appData[21] = (byte) ((grupo & 0x00FF0000) >> 16 );
      appData[22] = (byte) ((grupo & 0x0000FF00) >> 8 );
      appData[23] = (byte) ((grupo & 0X000000FF));
      appData[24] = (byte) ((num_tarjeta & 0xFF000000) >> 24 );
      appData[25] = (byte) ((num_tarjeta & 0x00FF0000) >> 16 );
      appData[26] = (byte) ((num_tarjeta & 0x0000FF00) >> 8 );
      appData[27] = (byte) ((num_tarjeta & 0X000000FF));
      appData[28] = (byte) ((conteoBloqueos & 0xFF000000) >> 24 );
      appData[29] = (byte) ((conteoBloqueos & 0x00FF0000) >> 16 );
      appData[30] = (byte) ((conteoBloqueos & 0x0000FF00) >> 8 );
      appData[31] = (byte) ((conteoBloqueos & 0X000000FF));
}

void setup() {

	Serial.begin(115200);

  Mcu.begin();
  
  // Configuracion OLED  
  oledDisplay.init();
  oledDisplay.setFont(ArialMT_Plain_10);

  // Configuracion de Pines
  dht.begin();  
  pinMode(PIN_SENSOR_IR, INPUT); // INPUT-PULL
  pinMode(PIN_BOTON_RESET, INPUT);
 
 // Configuracion GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

 // configuracion de LEDS
  pinMode(ledRojo, OUTPUT);
  pinMode(ledAzul, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  digitalWrite(ledRojo, LOW);
  digitalWrite(PIN_LED, LOW);

  // Configurar el temporizador de Hardware del ESP32
  myTimer = timerBegin(0, 80, true);  // timer 0, prescaler 80 para microsegundos
  timerAttachInterrupt(myTimer, &gestionarLED, true);  // Adjuntar la interrupción
  timerAlarmWrite(myTimer, 250000, true);  // Genera una interrupción cada 250ms
  timerAlarmEnable(myTimer);  // Habilitar la alarma

  // Configuraciones de Interrupciones
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_IR), detectarBloqueo, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON_RESET), resetearConteo, RISING);
  
  // LoRaWAN
  if(firstrun)
  {
    LoRaWAN.displayMcuInit();
    firstrun = false;
  }
	deviceState = DEVICE_STATE_INIT;

}


void loop()
{

  showBK();
	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{

#if(LORAWAN_DEVEUI_AUTO)
			LoRaWAN.generateDeveuiByChipID();
#endif
			LoRaWAN.init(loraWanClass,loraWanRegion);
      break;

		}
		case DEVICE_STATE_JOIN:
		{
      
      LoRaWAN.displayJoining();
			LoRaWAN.join();
			break;

		}
		case DEVICE_STATE_SEND:
		{

      oledDisplay.clear();
      LoRaWAN.displaySending();
			prepareTxFrame( appPort );
			LoRaWAN.send();
      
      // Mostrar los datos recibidos en la pantalla OLED
      showOG();
      // -----------------------------------------------

			deviceState = DEVICE_STATE_CYCLE;
			break;

		}
		case DEVICE_STATE_CYCLE:
		{

			// Schedule next packet transmission
			// txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(10000); // ENVIO CADA 10 SEGUNDO
			deviceState = DEVICE_STATE_SLEEP;
			break;

		}
		case DEVICE_STATE_SLEEP:
		{
      
      oledDisplay.clear();
			LoRaWAN.sleep(loraWanClass);
      oledDisplay.clear();
			break;
      
		}
		default:
		{

			deviceState = DEVICE_STATE_INIT;
			break;
		
    }
	}

}

// FUNCIONES A USAR
// Función de interrupción del temporizador para gestionar el LED
void IRAM_ATTR gestionarLED() {

  contadorLed++;  // Incrementar contador en cada milisegundo
  if (digitalRead(PIN_LED) && contadorLed >= 1) {  // Encendido por 250 ms
    contadorLed = 0;
    digitalWrite(PIN_LED, LOW);
  } else if (!digitalRead(PIN_LED) && contadorLed >= 3) {  // Apagado por 750 ms
    contadorLed = 0;
    digitalWrite(PIN_LED, HIGH);
  }

}

// Función de interrupción para aumentar bloqueos
void IRAM_ATTR detectarBloqueo() {

  if (IRbandera == false){
    IRbandera = true;
    conteoBloqueos++;
  }

}

// Función de interrupción para reiniciar el conteo
void IRAM_ATTR resetearConteo() {

  conteoBloqueos=0;
  IRbandera = true;

}

// Función mostrar pantalla bloqueos
void showBK(){

  if (IRbandera == true){
    oledDisplay.clear();
    oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
    oledDisplay.drawString(0, 0, "BLOQUEOOO");
    oledDisplay.drawString(0, 16, "Bloqueos detectados: " + String(conteoBloqueos));
    oledDisplay.display(); 
    delay(100); 
    oledDisplay.clear();
    IRbandera = false;
  }

}

// Función mostrar pantalla bloqueos
void showOG(){

      oledDisplay.clear();
      oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
      oledDisplay.drawString(0, 0, "Datos subidos,Grup5,Trj1");
      oledDisplay.drawString(0, 16, "Vol: "+ String(voltaje));
      oledDisplay.drawString(0, 32, "Lat: " + String(lat) + "Lon: " + String(logt));
      oledDisplay.drawString(0, 48, "Hum: " + String(hum) + "Temp: " + String(tem));
      oledDisplay.display();

}