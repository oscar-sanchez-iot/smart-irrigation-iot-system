// Inclusión de bibliotecas necesarias
#include <AT24CX.h>               // Librería para interactuar con memorias EEPROM AT24CX.
#include <SD.h>                   // Librería para trabajar con tarjetas SD.
#include <SPI.h>                  // Librería SPI para comunicación serial.
#include <Adafruit_Sensor.h>      // Librería para sensores de Adafruit (para sensores específicos).
#include <ETH.h>                  // Librería Ethernet para placas compatibles con Ethernet.
#include <WiFi.h>                 // Librería para la conexión Wi-Fi en placas compatibles con Wi-Fi.
#include <WiFiAP.h>               // Librería para configurar un punto de acceso Wi-Fi.
#include <WiFiClient.h>           // Librería para crear un cliente Wi-Fi.
#include <WiFiGeneric.h>          // Librería genérica de Wi-Fi.
#include <WiFiMulti.h>            // Librería para administrar múltiples conexiones Wi-Fi.
#include <WiFiScan.h>             // Librería para escanear redes Wi-Fi disponibles.
#include <WiFiServer.h>           // Librería para crear un servidor Wi-Fi.
#include <WiFiSTA.h>              // Librería para configurar una estación Wi-Fi.
#include <WiFiType.h>             // Librería para definir tipos de Wi-Fi.
#include <WiFiUdp.h>              // Librería para la comunicación UDP sobre Wi-Fi.
#include <Wire.h>                 // Librería para la comunicación I2C (bus de dos hilos).
#include <Adafruit_GFX.h>         // Librería para gráficos de Adafruit.
#include <Adafruit_SSD1306.h>     // Librería para pantallas OLED SSD1306 de Adafruit.
#include "DHTesp.h"               // Librería para sensores de temperatura y humedad DHT.
#include <I2C_eeprom.h>           // Librería para EEPROMs I2C.
#include <EEPROM.h>               // Librería para interactuar con la EEPROM interna del microcontrolador.
#define DS1307_I2C_ADDRESS 0X68   // Dirección I2C del RTC DS1307.
#include <PubSubClient.h>         // Librería para la comunicación con servidores MQTT.
#include "FS.h"                   // Librería para el sistema de archivos SPIFFS en ESP8266 y ESP32.
#include <Arduino.h>              // Librería principal de Arduino.
#include "freertos/FreeRTOS.h"    // Librería para sistemas basados en FreeRTOS.
#include "freertos/task.h"        // Librería para la creación de tareas en FreeRTOS.
#define SD_CS_PIN 5               // Definir el pin CS (Chip Select) de la tarjeta MICROSD
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire); // esta línea inicializa y configura el display OLED 
AT24C32 Eeprom; //Este objeto se utiliza para interactuar con una memoria EEPROM AT24C32.
DHTesp dht;
int estado = 0; //estado que indica en que posicion se encuentra el motor de la cortina en modo automatico
int estado1 = 0;//estado que indica en que posicion se encuentra el motor de la cortina en modo manual
int AIN1 = 13; // pin contol de giro de motor izq
int AIN2 = 12; // pin contol de giro de motor der
int ENA = 35; // pin de de pulsos para control de velocidad del motor de la cortina
int pinDHT = 15; //pin recepion analogica de humedad relativa y temperatura
int pinMan = 16; // pin ppara el control manual
int pin_aire = 2;// pin contol de encendido de aire del invernadero
int pin_alarma = 4; //pin de activacion de alarma cuando las condiciones del sistema se salgan de los parametros
int pin_led = 14;  //pin de avtivacion manual o por dashboard de la cortina de invernadero
int led_bomba = 26; //pin de activacion manual o por dasboard de la bomba de riego 
int led_aire = 17; // pin de activacion manual o por dashboard de los disipadores de calor del invernadero
int pin_relay = 27; // pin de acrivacion automatica de la bomba de riego (activacion de rele No1)
int valor = 0; //variable utilizada para mostrar el valor de humedad temperatura en pantalla
float tempMax; //temperatura máxima que puede tomar según el cultivo seleccionado
float hrMax; //humedad relativa máxima que puede tomar según el cultivo seleccionado
float hsMax; //humedad de suelo máxima que puede tomar según el cultivo seleccionado
float phMax; //ph máximo que puede tomar según el cultivo seleccionado
float tempMin; //temperatura minima que puede tomar según el cultivo seleccionado
float hrMin; //humedad relativa minima que puede tomar según el cultivo seleccionado
float hsMin; //humedad de suelo máxima que puede tomar según el cultivo seleccionado
float phMin; //ph minimo que puede tomar según el cultivo seleccionado
float ph_act; //variable utilizada para la obtencion del ph activo del sustraro
const int humsuelo = 33; // pin de Lectura del sensor //HW-080 
int valHumsuelo; // Variable utiliazada para la obtencion de la humedad de suelo
int Tempe = 0; // variable que se usa para obtener el valor de la temperatura del sensor en formato int para poder ser reconoccida en node red
int humed = 0;// variable que se usa para obtener el valor de la temperatura del sensor en formato int para poder ser reconoccida en node red
int valHum = 0; //variable de operacion para convesion de humedad 
int ph=0; //variable utilizada para poder reconocer el ph_acctivo en la dashboard
int HumSu=0;// variable utilizada para poder reconocer el ph_acctivo en la dashboard
char buffer1[10];// buffer que almacena el valor de la temperatura para ser leida en lenguaje de nodos de node red
char buffer2[10];// buffer que almacena el valor de la humedad realtiva para ser leida en lenguaje de nodos de node red
char buffer3[10]; // buffer que almacena el valor de la humedad de suelo para ser leida en lenguaje de nodos de node red
char buffer4[10]; // buffer que almacena el valor del ph para ser leida en lenguaje de nodos de node red
File myFile; //variable que genera el archivo de reportes en la micro sd

///////////Sensor PH///////////
float calibration_value = 21.34; //base de calibraccion del sensor ph
int phval = 0; //variables de operacion del sensor
unsigned long int avgval; //variable de operacion del sensor
int buffer_arr[10], temp;
///////////////////////////////
// Variables y objetos relacionados con la comunicación WiFi y MQTT
const char *ssid = "SamsungOscar"; //red wifi conexion con la esp32
const char *password = "oscarSan"; //contraseña red wifi
//const char* ssid = "oscarsam"; //red wifi alterna
//const char* password = "oscar001";//contraseña red wifi alterna
const char *mqtt_server = "test.mosquitto.org"; //vinculacion con el broker 
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
 
 // Configurar la conexión WiFi
void setup_wifi()
{

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Función de callback para manejar mensajes MQTT recibe los topicos y las variables aplicados en la dashboard
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  
  // Controlar el LED en el pin 16
  if (strcmp(topic, "manual") == 0)
  {
    if ((char)payload[0] == '0')
    {
          digitalWrite(16, LOW); // se utiliza para desactivar el control manual
        
    }
    else
    {
      digitalWrite(16, HIGH); //se utiliza para activar el control manual
        
      
    }
  }

   if (strcmp(topic, "Ingesa-2") == 0)
  {
    if ((char)payload[0] == '0')
    {
          digitalWrite(14, LOW); //se utiliza para apagar de modo manual del aire
        
    }
    else
    {
      digitalWrite(14, HIGH); //se utiliza para encender de modo manual del aire
      
    }
  }
  
  // Controlar el LED en el pin 26
  if (strcmp(topic, "Ingesa-1") == 0)
  {
    if ((char)payload[0] == '0')
    {
      digitalWrite(26, LOW); //se utiliza para apagar de modo manual el riego
      Serial.print(topic);
    }
    else
    {
      digitalWrite(26, HIGH); //se utiliza para encender de modo manual el riego
      Serial.print(topic);
    }
  }

    // Controlar el LED en el pin 17
  if (strcmp(topic, "Ingesa-3") == 0)
  {
    if ((char)payload[0] == '0')
    {
      digitalWrite(17, LOW); // se utiliza para activar cotrol manual al sistema
      Serial.print(topic);
    }
    else
    {
      digitalWrite(17, HIGH); // se utiliza para desactivar cotrol manual al sistema
      Serial.print(topic);
    }
  }
 //recibe el topico de seleccion de cultivo "lechuga"  del dashboard y asigna los parametros de este a las variables de control de actuadores
   if (strcmp(topic, "cultivo") == 0)
{
  Serial.println("Topic is 'cultivo'");
  Serial.print("Payload[0]: "); Serial.println((char)payload[0]);
  
  if ((char)payload[0] == '1')
  {
      tempMax = 24;
      hrMax = 70;
      hsMax = 80;
      phMax = 6.8;
      tempMin = 7;
      hrMin = 45;
      hsMin = 60;
      phMin = 6;

      // Imprimir los valores configurados en el puerto serie
      Serial.println("Lechuga");
      Serial.println("Valores configurados para la opción 1:");
      Serial.print("Temp Max: "); Serial.println(tempMax);
      Serial.print("HR Max: "); Serial.println(hrMax);
      Serial.print("HS Max: "); Serial.println(hsMax);
      Serial.print("pH Max: "); Serial.println(phMax);
      Serial.print("Temp Min: "); Serial.println(tempMin);
      Serial.print("HR Min: "); Serial.println(hrMin);
      Serial.print("HS Min: "); Serial.println(hsMin);
      Serial.print("pH Min: "); Serial.println(phMin);
  }
}
 //recibe el topico de seleccion de cultivo "papa"  del dashboard y asigna los parametros de este a las variables de control de actuadores
 if (strcmp(topic, "cultivo") == 0)
{
  Serial.println("Topic is 'cultivo'");
  Serial.print("Payload[0]: "); Serial.println((char)payload[0]);
  
  if ((char)payload[0] == '2')
  {
      tempMax = 18;
      hrMax = 70;
      hsMax = 80;
      phMax = 7.0;
      tempMin = 12;
      hrMin = 50;
      hsMin = 70;
      phMin = 5;

      // Imprimir los valores configurados en el puerto serie
      Serial.println("Papa");
      Serial.println("Valores configurados para la opción 1:");
      Serial.print("Temp Max: "); Serial.println(tempMax);
      Serial.print("HR Max: "); Serial.println(hrMax);
      Serial.print("HS Max: "); Serial.println(hsMax);
      Serial.print("pH Max: "); Serial.println(phMax);
      Serial.print("Temp Min: "); Serial.println(tempMin);
      Serial.print("HR Min: "); Serial.println(hrMin);
      Serial.print("HS Min: "); Serial.println(hsMin);
      Serial.print("pH Min: "); Serial.println(phMin);
  }
}

 //recibe el topico de seleccion de cultivo "arveja"  del dashboard y asigna los parametros de este a las variables sensadas para control de actuadores
 if (strcmp(topic, "cultivo") == 0)
{
  Serial.println("Topic is 'cultivo'");
  Serial.print("Payload[0]: "); Serial.println((char)payload[0]);
  
  if ((char)payload[0] == '3')
  {
      tempMax = 25;
      hrMax = 70;
      hsMax = 80;
      phMax = 6.5;
      tempMin = 10;
      hrMin = 50;
      hsMin = 60;
      phMin = 5.5;

      // Imprimir los valores configurados en el puerto serie
      Serial.println("Arveja");
      Serial.println("Valores configurados para la opción 1:");
      Serial.print("Temp Max: "); Serial.println(tempMax);
      Serial.print("HR Max: "); Serial.println(hrMax);
      Serial.print("HS Max: "); Serial.println(hsMax);
      Serial.print("pH Max: "); Serial.println(phMax);
      Serial.print("Temp Min: "); Serial.println(tempMin);
      Serial.print("HR Min: "); Serial.println(hrMin);
      Serial.print("HS Min: "); Serial.println(hsMin);
      Serial.print("pH Min: "); Serial.println(phMin);
  }
}
}

// Función para intentar reconectar al servidor MQTT
void reconnect()
{
   while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
   
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
   
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic-Ingesa", "hello world");
      // ... and resubscribe
      client.subscribe("manual"); //subsribe el topico de control manual
      client.subscribe("Ingesa-2"); //subsribe el topico de control manual del aire
      client.subscribe("Ingesa-1"); //subsribe el topico de control manual de riego
      client.subscribe("Ingesa-3"); //subsribe el topico de control manual de cortinas
      client.subscribe("cultivo"); //subscribe el topico de seleccion de cultivo
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Funciones para escribir y leer desde una EEPROM I2
/////////////ESCRIBIR///////////////
void i2c_eeprom_write_byte(int deviceaddress, unsigned int eeaddress, byte data)
{
  int rdata = data;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission();
}

/////////////LEER///////////////
byte i2c_eeprom_read_byte(int deviceaddress, unsigned int eeaddress)
{
  byte rdata = 0xFF;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress, 1);
  if (Wire.available())
  {
    rdata = Wire.read();
    return rdata;
  }
}

/// convierte numeros decimales a BCD
byte decToBcd(byte val)
{
  return ((val / 10 * 16) + (val % 10));
}
/// convierte BCD a numeros decimales
byte bcdToDec(byte val)
{
  return ((val / 16 * 10) + (val % 16));
}
// Funciones para configurar y leer la fecha y hora de un DS1307
/////////////////////////////////VOID ESCRIB
void setDateDs1307(byte second,     // 0-59
                   byte minute,     // 0-59
                   byte hour,       // 0-23
                   byte dayOfWeek,  // 1-7
                   byte dayOfMonth, // 1-28/29/30/31
                   byte month,      // 1-12
                   byte year)       // 0-99

{
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0);
  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));
  Wire.write(decToBcd(dayOfWeek));
  Wire.write(decToBcd(dayOfMonth));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));
  Wire.endTransmission();
}
////LEER RELOJ////
void getDateDs1307(byte *second,
                   byte *minute,
                   byte *hour,
                   byte *dayOfWeek,
                   byte *dayOfMonth,
                   byte *month,
                   byte *year)

{
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);

  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

//variables de proposito general
int tellstate = 0;
float variable[16];
unsigned int resultado, respuesta, respuesta2;
char datom, datoi[2];
long segundos, auxsegundos;
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
short esclavo;
int i, datos[16], a, contador, datoe[6], clave[4], clavei[6], dir, adres;

//////////////////////////////////////SENSOR PH///////////////////////////////////////////

void sensorPh()
{
  for (int i = 0; i < 10; i++)
  {
    buffer_arr[i] = analogRead(32);
    delay(30);
  }
  for (int i = 0; i < 9; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      if (buffer_arr[i] > buffer_arr[j])
      {
        temp = buffer_arr[i];
        buffer_arr[i] = buffer_arr[j];
        buffer_arr[j] = temp;
      }
    }
  }
  avgval = 0;
  for (int i = 2; i < 8; i++)
    avgval += buffer_arr[i];
  float volt = ((float)avgval * 5.0 / 1024 / 6) * 0.141;
  // float ph_act =((-5.70 * volt + calibration_value)*2.05);
  float ph_act = -5.70 * volt + calibration_value;
}


void setup()
{
  //confguracion pantalla oled
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  //configuracion de pines digitales y analogicos como entradas y salidas
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  dht.setup(pinDHT, DHTesp::DHT11);
  pinMode(pin_relay, OUTPUT);
  pinMode(pin_alarma, OUTPUT);
  pinMode(pin_aire, OUTPUT);
  pinMode(pin_led, OUTPUT); 
  pinMode(humsuelo, INPUT);
  pinMode(led_bomba, OUTPUT);
  pinMode(led_aire, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(pinMan, OUTPUT);
  digitalWrite(ENA, HIGH);
  Serial.begin(115200); //velocidad de trasmision en baudios
  setup_wifi(); //llamado de metodo wifi
  client.setServer(mqtt_server, 1883); //configuracion protocolo mqtt a través del puerto 1883
  client.setCallback(callback);
 
}

void reporte() //metodo de generación de reportes
{
  datos[1] = 1;
  datos[2] = dayOfMonth;
  datos[3] = month;
  datos[4] = year;
  datos[5] = hour;
  datos[6] = minute;
  datos[7] = second;

  // aqui se genera el numero del reporte
  a = Eeprom.read(0);
  if (a == 0xFF)
  {
    contador = 0;
  }
  datos[0] = a;
  datos[0] = datos[0] + 1;
  Eeprom.write(0, datos[0]);
  delay(5);
  if (!SD.begin(SD_CS_PIN))
  {                                                       // Inicializar la tarjeta SD utilizando el pin CS definido anteriormente
    Serial.println("Error al inicializar la tarjeta SD"); // Si la inicialización falla, mostrar un mensaje de error y terminar
    return;
  }
  Serial.println("Tarjeta SD inicializada correctamente"); // Si la inicialización tiene éxito, mostrar un mensaje de confirmación
  // cambiar la palabra write para crear el encabezado y appende para dar el salto de linea a cada uno de los repores
  File myFile = SD.open("/Reportes.csv", FILE_APPEND); // Crear o abrir un archivo en la tarjeta SD con el nombre "test.txt" y modo de escritura
  if (!myFile)
  {
    Serial.println("Error al abrir el archivo"); // Si no se puede abrir el archivo, mostrar un mensaje de error y terminar
    return;
  }
  // Escribir datos en el archivo (en este caso, una cadena de texto)

  //Genera el encabezaddo del archivo csv en la micro sd
   //myFile.println("#REPORTE        IDEQUIPO               DD                  MM                   AA                    HH                     MIN                    SEG                     S.H                     S.T                     S.H.S                     S.PH");
   //myFile.close(); //generar la primeara vez  y luego comentar
   // myFile = SD.open("Reportes.csv", FILE_WRITE);
  // myFile.println("");
  if (myFile)
  {
    getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year); //resive las variables del reloj y de los sensores para los reportes 

    myFile.print(datos[0], DEC); // ##
    myFile.print(";");
    myFile.print(datos[1], DEC); // ID
    myFile.print(";");
    myFile.print(datos[2], DEC); // DD
    myFile.print(";");
    myFile.print(datos[3], DEC); // MM
    myFile.print(";");
    myFile.print(datos[4], DEC); // AA
    myFile.print(";");
    myFile.print(datos[5], DEC); // HH
    myFile.print(";");
    myFile.print(datos[6], DEC); // MIN
    myFile.print(";");
    myFile.print(datos[7], DEC); // SEG
    myFile.print(";");
    myFile.print(variable[0], 2); // S.H
    myFile.print(";");
    myFile.print(variable[1], 2); // S.T
    myFile.print(";");
    myFile.print(variable[2], 2); // S.H.S
    myFile.print(";");
    myFile.print(variable[3], 2); // S.PH
    myFile.println(";");
    myFile.close();
    // Cerrar el archivo para asegurar que los datos se guarden correctamente

    myFile.close();

    // Mostrar un mensaje indicando que los datos se han escrito en el archivo
    Serial.println("Datos escritos en el archivo");

    // Abrir el archivo "Reportes.csv" en modo de lectura
    myFile = SD.open("/Reportes.csv", FILE_READ);
    if (!myFile)
    {

      // Si no se puede abrir el archivo, mostrar un mensaje de error y terminar
      Serial.println("Error al abrir el archivo");
      return;
    }

    // Leer los datos del archivo y enviarlos a través de la comunicación serie
    while (myFile.available())
    {

      // Leer un byte del archivo y enviarlo a través de la comunicación serie
      Serial.write(myFile.read());
    }
  }
}
//actualizaccion de hora y fecha
void actualizar() // colocar la primera vez
{
  second = 0x00;     // segundo 53
  minute = 0x23;     // minuto 42
  hour = 0x0C;       // hora 7:00
  dayOfWeek = 0x06;  // dia de la semana 02
  dayOfMonth = 0x10; // dia del mes 16
  month = 0x09;      // mes 09
  year = 0x17;       // año = 2023

  setDateDs1307(second, minute, hour, dayOfWeek, dayOfMonth, month, year);
}

//metodo de obtencion de hora y fecha y variables sensadas
void fecha_hora()
{
  for (i = 0; i < 12; i++)
  {
    getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year); //&apuntador
    myFile.print(datos[0], DEC);                                                    // ##
    myFile.print(";");
    myFile.print(datos[1], DEC); // ID
    myFile.print(";");
    myFile.print(datos[2], DEC); // DD
    myFile.print(";");
    myFile.print(datos[3], DEC); // MM
    myFile.print(";");
    myFile.print(datos[4], DEC); // AA
    myFile.print(";");
    myFile.print(datos[5], DEC); // HH
    myFile.print(";");
    myFile.print(datos[6], DEC); // MIN
    myFile.print(";");
    myFile.print(datos[7], DEC); // SEG
    myFile.print(";");
    myFile.print(variable[0], 2); // S.H
    myFile.print(";");
    myFile.print(variable[1], 2); // S.T
    myFile.print(";");
    myFile.print(variable[2], 2); // S.H.S
    myFile.print(";");
    myFile.print(variable[3], 2); // S.PH
    myFile.print(";");
    myFile.close();
  }
}

//metodo para leer la memoria EEPROM
void leerExterna()
{
  for (i = 0; i < 6; i++)
  {
    datoe[i] = i2c_eeprom_read_byte(0x50, i);
    delay(3);
  }
}

//metodo para escribir la memoria EEPROM
void escribirExterna()
{
  for (i = 0; i < 2; i++)
  {
    i2c_eeprom_write_byte(0x50, i, clave[i]);
    delay(3);
    dir++;
  }
}
//metodo para escribir la memoria EEPROM
void leerMemoria()
{
  for (i = 0; i < 6; i++)
    clave[i] = EEPROM.read(i);
  delay(3);
}

void escribirMemoria()
{
  for (i = 0; i < 2; i++)
  {
    EEPROM.write(adres, clave[i]);
    adres++;
    delay(5);
  }
}
//método de lectura de sensores y de activacion de actuadores
void leerSensores()
{
  
  TempAndHumidity data = dht.getTempAndHumidity();
  variable[0] = data.temperature;
  variable[1] = data.humidity;
  variable[2] = valHum;
 
  Tempe = data.temperature;
  humed = data.humidity;
  ph = ph_act;
  HumSu = valHum;
  Serial.println("Temperatura: " + String(data.temperature, 2) + "°C");
  Serial.println("Humedad: " + String(data.humidity, 1) + "%");
  // Convertir el valor a porcentaje
  valHumsuelo = map(analogRead(humsuelo), 4092, 0, 0, 100);
  valHum= valHumsuelo*2;
  Serial.print("Humedad del suelo: ");
  Serial.print(valHum);
  Serial.println(" %");
  // metodo para Sensor Ph
   for (int i = 0; i < 10; i++)
  {
    buffer_arr[i] = analogRead(32);
    delay(30);
  }
  for (int i = 0; i < 9; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      if (buffer_arr[i] > buffer_arr[j])
      {
        temp = buffer_arr[i];
        buffer_arr[i] = buffer_arr[j];
        buffer_arr[j] = temp;
      }
    }
  }
  avgval = 0;
  for (int i = 2; i < 8; i++)
    avgval += buffer_arr[i];
  float volt = ((float)avgval * 5.0 / 1024 / 6) * 0.141;
  // float ph_act =((-5.70 * volt + calibration_value)*2.05);
 ph_act = -5.70 * volt + calibration_value;
  //
  //mostrar las variables sensadas en el puerto serial 
  Serial.print("Ph Suelo: ");
  Serial.println(String(ph_act, 2));
  Serial.print("Vol: ");
  Serial.println(volt);
  Serial.println("--");
  variable[3] = ph_act;
  delay(1000);
  //mostrar las cariable sensadas en la pantalla oled
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.setTextSize(1);
  display.println(data.temperature);
  display.println(data.humidity);
  display.println(valHum);
  display.println(ph_act);
  delay(2000);
  //activa o desactiva el aire si la humedad relativa llega a los limmites segun el cultivo seleccionado
  if (data.humidity < hrMax)
  {
    digitalWrite(pin_aire, HIGH); 
    display.display();
    delay(2000);
  }
  else if (data.humidity >= hrMin)
  {
    digitalWrite(pin_aire, LOW);
    display.display();
    delay(2000);
  }
  //activa o desactiva alarma si la humedad relativa o ph sobre pasa los limites 
  if (data.humidity > hrMax+5 || ph_act > phMax || ph_act < phMin)
  {
    digitalWrite(pin_alarma, HIGH);
    delay(100);
    display.display();
    delay(2000);
  }
  else 
  {
    digitalWrite(pin_alarma, LOW);
    display.display();
    delay(2000);
  }
  //abre o cierra las cortinas si la temperatura llega a los limmites segun el cultivo seleccionado
  if (data.temperature > tempMax & estado == 0)
  {
    digitalWrite(AIN2, HIGH);
    digitalWrite(AIN1, LOW);
    delay(2000);
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, LOW);
    display.display();
    delay(2000);
    estado = 1;
  }
  else if (data.temperature < tempMin & estado == 1)
  {
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, HIGH);
    //analogWrite(ENA, 128); 
    display.display();
    delay(2000);
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, LOW);
    //analogWrite(ENA, 128); 
    delay(2000);
    estado = 0;
    
    }
   //activa o desactiva el riego si la humedad llega a los limmites segun el cultivo seleccionado
    if (valHum < hsMin)
  {
     digitalWrite(pin_relay, HIGH);
     display.display();
     delay(2000);    
  }
  else if(valHum <= hsMax)
  {
    digitalWrite(pin_relay, HIGH);
    display.display();
    delay(2000);
    
  }
  

  }
 
  

void loop()
{

  // Verifica si el cliente está conectado
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  // asigna variable de estado de los pines mencionados
  int pinLedState = digitalRead(pin_led);
  int pinManState = digitalRead(pinMan);
  int led_aireState = digitalRead(led_aire);
  int led_bombaState = digitalRead(led_bomba);


  // Verificar el estado del pin

  if (pinManState == LOW)
  {
    leerSensores(); //desactiva el método sensores por que se seleccionó el método manual
    reporte(); //desactiva el método reportes por que se seleccionó el método manual
  
  }
// si esta activo el modo manual puede controlar de este modo la cortina
 else{
 if(pinLedState == HIGH & pinManState== HIGH & estado1==0)
   { estado1=1;
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, HIGH);
    delay(1500);
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, LOW);
    delay(1000);
   }
 else if(pinLedState == LOW & pinManState== HIGH & estado1==1)
 {
    estado1=0;
    digitalWrite(AIN2, HIGH);
    digitalWrite(AIN1, LOW);
    delay(1500);
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, LOW);
    delay(2000);
    
 }

 // si esta activo el modo manual puede controlar de este modo aire
 if(led_aireState == HIGH & pinManState== HIGH)
   {
    
    digitalWrite(pin_aire, HIGH);   
    
   }
 else if(led_aireState == LOW & pinManState== HIGH)
 {
    
    digitalWrite(pin_aire, LOW);;
    
    
 }
// si esta activo el modo manual puede controlar de este modo el riego 
if(led_bombaState == HIGH & pinManState== HIGH)
   {
    
    digitalWrite(pin_relay, HIGH);   
    
   }
 else if(led_bombaState == LOW & pinManState== HIGH)
 {
    
    digitalWrite(pin_relay, LOW);;
    
    
 }
 }
  unsigned long now = millis();
  if (now - lastMsg > 2000)
  {
    lastMsg = now;
    ++value;
    snprintf(msg, MSG_BUFFER_SIZE, "SISE #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);

    // Publica la temperatura en el tema "Temperatura-Ingesa" en dashboard
   itoa(Tempe, buffer1, 10);
    client.publish("Temperatura-Ingesa", buffer1);

    // Publica la humedad en el tema "Humedad-Ingesa"  en dashboard
    itoa(humed, buffer2, 10);
    client.publish("Humedad-Ingesa", buffer2);

     // Publica la humedad de suelo en el tema "Temperatura-Ingesa"  en dashboard
    itoa(HumSu, buffer3, 10);
    client.publish("HSuelo_Ingesa", buffer3);

    // Publica ph en el tema "Humedad-Ingesa"  en dashboard
    itoa(ph, buffer4, 10);
    client.publish("PH_Ingesa", buffer4);
  }

 
  
  // actualizar(); // activar metodo setear fecha y hora para inciar por primera vez el sistema
  // fecha_hora(); // activar metodo Guardar la fecha y hora
}

