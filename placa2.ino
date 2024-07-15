#include <OneWire.h>
#include <DallasTemperature.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <PID_v1.h>

//Prototipos de funciones para evitar errores
// Funciones de comunicación
void recibirDatos(void);  // Recibe datos (JSON) de la placa 1
void enviarDatosPlaca2(void); // Envía datos (JSON) a la placa 1
// Funciones de sensores
void leerTemperaturaSensor(void); // Lee la temperatura del sensor DS18B20
void leerEstadoGrifo(void); // Lee el estado del grifo (abierto o cerrado)
// Funciones de actuadores
void ActivarBomba(void); // Activa o desactiva la bomba según las condiciones
void activarCalentador(void); // Controla el calentador mediante PID



//////////////DEFINICION DE PINES/////////////
//Comunicacion entre placas
#define ONE_WIRE_BUS 32 // Pin de datos del DS18B20
#define TX_PIN 27 // Pin TX (GPIO 1)
#define RX_PIN 14 // Pin RX (GPIO 3) 
//Actuadores y sensores
#define PIN_BOMBA 2 //Pin activar relé bomba
#define PIN_CALENTADOR 4 //Pin activar SSR, salida del PID
#define PIN_GRIFO 16 //Pin sensor XKC-Y28, que indica el grifo abierto


/////////////Variables globales recibidas de la otra placa////////////////
bool emergenciaRecibida = false;
bool bombaRecibida = false;
int temperaturaRecibida = 0;
//Por si falla el JSON
bool emergenciaAnterior = false;
bool bombaAnterior = false;
float temperaturaAnterior = 0.0;

//////Variables globales enviadas///////
bool grifoAbiertoEnviado = 0;
float temperaturaLeida = 0.0;

////// Variables para el PID*///////
double Setpoint, Input, Output;
double Kp = 5, Ki = 10, Kd = 1;
int OutputMin = 0, OutputMax = 255; // Límites de salida (0-255 para PWM)


//Crear instancias
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);



void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  sensors.begin();

  pinMode(PIN_GRIFO, INPUT);
  pinMode(PIN_BOMBA, OUTPUT);
  pinMode(PIN_CALENTADOR, OUTPUT);


  digitalWrite(PIN_BOMBA, LOW);
  digitalWrite(PIN_CALENTADOR, LOW);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(OutputMin, OutputMax);
  myPID.SetSampleTime(100); 
}


void loop() {
  leerTemperaturaSensor();
  leerEstadoGrifo();
  enviarDatosPlaca2(); // Enviar datos a la placa 1
  recibirDatos(); // Recibir datos de la placa 1
  if (emergenciaRecibida) {
    // Parada de emergencia: apagar bomba y calentador
    digitalWrite(PIN_BOMBA, LOW);
    digitalWrite(PIN_CALENTADOR, LOW);
  }
  else{
    ActivarBomba();
    activarCalentador();
  }

  delay(5); // Retardo igual al de la placa CYD
}


////////////////////////////////////FUNCIONES AÑADIDAS///////////////////////////////////////
/////////FUNCIONES DE COMUNICACION/////////////////////

//Función para recibir
/*void recibirDatos() {
  if (Serial2.available()) {
    String jsonRecibido = Serial2.readStringUntil('\n');

    StaticJsonDocument<128> doc; 
    DeserializationError error = deserializeJson(doc, jsonRecibido);
    //Serial.println(jsonRecibido); //ver que llega por si hay errores

    if (error) {
      Serial.print(F("Error al deserializar JSON: "));
      Serial.println(error.f_str());
      return; 
    }

    // Leer los valores del JSON
    emergenciaRecibida = doc["ParadaEmergencia"];
    bombaRecibida = doc["ActivarBomba"];
    temperaturaRecibida = doc["temperaturaEnviada"];

    // Imprimir los datos recibidos 
    Serial.println("Datos recibidos:");
    Serial.print("  Emergencia: "); Serial.println(emergenciaRecibida);
    Serial.print("  Bomba: "); Serial.println(bombaRecibida);
    Serial.print("  Temperatura: "); Serial.println(temperaturaRecibida);
  }
}*/

void recibirDatos() {
    if (Serial2.available()) {
        String jsonRecibido = Serial2.readStringUntil('\n');

        StaticJsonDocument<128> doc; 
        DeserializationError error = deserializeJson(doc, jsonRecibido);

        if (!error) {
            // Solo actualizar si la deserialización fue exitosa
            emergenciaAnterior = doc["ParadaEmergencia"];
            bombaAnterior = doc["ActivarBomba"];
            temperaturaAnterior = doc["temperaturaEnviada"];

            // Imprimir los datos recibidos 
            Serial.println("Datos recibidos:");
            Serial.print(" Emergencia: "); Serial.println(emergenciaAnterior);
            Serial.print(" Bomba: "); Serial.println(bombaAnterior);
            Serial.print(" Temperatura: "); Serial.println(temperaturaAnterior);
        } else {
            // Usar los valores anteriores en caso de error
            emergenciaRecibida = emergenciaAnterior;
            bombaRecibida = bombaAnterior;
            temperaturaRecibida = temperaturaAnterior;

            Serial.println("Error en el JSON, usando valores anteriores:");
            Serial.print(" Emergencia: "); Serial.println(emergenciaRecibida);
            Serial.print(" Bomba: "); Serial.println(bombaRecibida);
            Serial.print(" Temperatura: "); Serial.println(temperaturaRecibida);
        }
    }
}


//Funcion para enviar datos desde la placa 2
void enviarDatosPlaca2() {
  StaticJsonDocument<128> datosEnviar; // Igual que lo recibido

  datosEnviar["temperaturaLeida"] = temperaturaLeida; //hay que poner lo que la otra placa espera recibir
  datosEnviar["grifoAbierto"] = grifoAbiertoEnviado;

  String jsonEnviar;
  serializeJson(datosEnviar, jsonEnviar);
  Serial2.println(jsonEnviar);

  //Imprimir los datos enviados para depuración
  Serial.println("Datos enviados:");
  Serial.print("  Temperatura: "); Serial.println(temperaturaLeida);
  Serial.print("  Grifo abierto: "); Serial.println(grifoAbiertoEnviado);
}




////////////////////////////FUNCIONES DE LOS SENSORES//////////////
//Leer la temperatura
void leerTemperaturaSensor() {
  sensors.requestTemperatures(); // Solicitar la medición de temperatura
  temperaturaLeida = sensors.getTempCByIndex(0); // Leer la temperatura del primer sensor
  
 
      /* Mostrar la temperatura en el Monitor Serie
    char tempString[10];
    dtostrf(temperaturaLeida, 6, 2, tempString);
    Serial2.print(tempString);
    Serial2.println(" C");*/
}

//Leer el estado del grifo
void leerEstadoGrifo() {
  grifoAbiertoEnviado = digitalRead(PIN_GRIFO) == 1; //Grifo abierto = 1
}


////////////////////////////////FUNCIONES DE LOS ACTUADORES////////////////////////////////
//Activar la bomba de recirculado
void ActivarBomba() {
  if (bombaRecibida && grifoAbiertoEnviado==1) {
    digitalWrite(PIN_BOMBA, HIGH); // Activar la bomba
    //Serial.println("Bomba activada"); // Mensaje de depuración 
  } 
  else {
    digitalWrite(PIN_BOMBA, LOW);  // Desactivar la bomba
    //Serial.println("Bomba desactivada"); // Mensaje de depuración
  }
}

//Activar el calentador//
void activarCalentador() {
  Input = temperaturaLeida;
  Setpoint = temperaturaRecibida;
  myPID.Compute();              // Calcular la salida del PID
  analogWrite(PIN_CALENTADOR, Output); // Controlar el calentador con PWM
}