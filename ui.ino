#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>
#include <HardwareSerial.h> // Incluir la librería HardwareSerial, para enviar los datos con los pines que quiero
#include <ArduinoJson.h> // Para enviar los datos en formato JSON, más robusto


//Prototipos de funciones no incluidas en el ui
//Funciones de comunicacion entre placas
void enviarDatos(bool emergencia, bool bomba, int temperaturaEnviada); 
void recibirDatos(void);
//funciones de tiempo
void iniciar_cuenta_atras(lv_obj_t * label, lv_obj_t * spinbox, int minutos); // iniciar cuenta atras manual
void timer_callback_manual(lv_timer_t * timer_manual);
void timer_callback_maceracion(lv_timer_t * timer_maceracion);
void timer_callback_ebullicion(lv_timer_t * timer_ebullicion);
//funciones interaccion con usuario
void Alarma(void); // Prototipo de la función Alarma(), para que suene el buzzer
void reiniciarSpinboxesTiempo(void); //puesta a 0 de todos los spinboxes, al acabar los procesos o después de la seta
void actualizarLabelsTemperatura(void);


/*Don't forget to set Sketchbook location in File/Preferences to the path of your UI project (the parent foder of this INO file)*/
/*Change to your screen resolution*/
static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 320;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

// Touchscreen pins
#define XPT2046_IRQ 36  // T_IRQ
#define XPT2046_MOSI 32 // T_DIN
#define XPT2046_MISO 39 // T_OUT
#define XPT2046_CLK 25  // T_CLK
#define XPT2046_CS 33  // T_CS

SPIClass tsSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

lv_timer_t * timer_manual; // Define timer_manual para el modo manual
lv_timer_t * timer_maceracion; // Temporizador para la maceración
lv_timer_t * timer_ebullicion; // Temporizador para la ebullición


//DEFINICION DE PINES LIBRES///
// Comunicación entre placas
#define RX_PIN 22       // Pin RX (GPIO 22)
#define TX_PIN 27       // Pin TX (GPIO 27)
//El pin de la alarma
#define PIN_BUZZER 26 // Pin GPIO al que está conectado el buzzer


//DEFINICION DE VARIABLES GLOBALES///
//variables globales para enviar, se van a usar en varias funciones
bool bomba = 0; // Variable para enviar si se activa la bomba o no
int temperaturaEnviada = 0; //Variable para enviar la temperatura manual
bool emergencia = 0;

//variables globales para recibir
float temperaturaRecibida = 0.0;
bool grifoAbierto = 0;

//Para los procesos
int temperaturasAuto[4] = {0,0,0,0};
int tiempoMaceracion[4] = {0,0,0,0};
int tiempoAvisoEbull[4] = {0,0,0,0};
bool bombaMaceracion[4] = {0,0,0,0};

// Variables para la cuenta atrás
static int segundos_restantes = 0; //Segundos restantes manual
static int segundosRestantesMaceracion = 0; // Segundos restantes para la maceración
static int segundosRestantesEbullicion = 0; // Segundos restantes para la ebullición
static int segundosRestantesPeriodoEbullicion = 0; // Nuevo contador para el periodo en curso
int tiempoTotalMaceracion = 0;
bool maceradofin=0;
int indiceMaceracion = 0;
int indiceEbullicion = 0; 


#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    uint16_t touchX = 0, touchY = 0;
    TS_Point p = touchscreen.getPoint(); //yo
    /*Serial.print("Raw X: "); Serial.print(p.x);
    Serial.print(", Raw Y: "); Serial.print(p.y);
    Serial.print(", Pressure: "); Serial.println(p.z);*/

    data->point.x = map(p.x, 271, 3880, 0, screenWidth); //valores sacados mediante calibración
    data->point.y = map(p.y, 183, 3796, 0, screenHeight); //valores sacados mediante calibración

    if( p.z>60 ) //se está apretando la pantalla.
    {
        data->state = LV_INDEV_STATE_PR; //Pressed
        /*Serial.print("Raw X: "); Serial.print(p.x);
        Serial.print(", Raw Y: "); Serial.print(p.y);
        Serial.print(", Pressure: "); Serial.println(p.z);*/
    }
    else
    {
        data->state = LV_INDEV_STATE_REL; //Released       
    }
}

void setup(){
    //Serial.begin( 115200 ); /* prepare for possible serial debug */

    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

//código inicializar touchscreen, los parámetros están definidos arriba, se usa la libreria XPT2046
    tsSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    touchscreen.begin(tsSPI);
    touchscreen.setRotation(0); // Rotación del táctil a 0

    tft.begin();          /* TFT init */
    tft.setRotation(0); // Rotación pantalla a 0, debe coincidir con la táctil
    tft.invertDisplay(1); //Para que los colores no salgan invertidos

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    ui_init();
    Serial.println( "Setup done" );

///Timers
  timer_manual = lv_timer_create(timer_callback_manual, 1000, NULL); // Crear el temporizador (1 segundo de intervalo) para que no se bloquee la pantalla en la cuenta atrás
  timer_maceracion = lv_timer_create(timer_callback_maceracion, 1000, NULL); // Temporizador maceración
  timer_ebullicion = lv_timer_create(timer_callback_ebullicion, 1000, NULL); // Temporizador ebullición
  //timer_verificar_grifo = lv_timer_create(verificar_grifo_callback, 100, NULL);

  lv_timer_pause(timer_manual); // Pausar el temporizador al inicio para que no vaya restando
  lv_timer_pause(timer_maceracion);
  lv_timer_pause(timer_ebullicion); 
 // lv_timer_pause(timer_verificar_grifo); // Pausar el temporizador al inicio

//Monitores serie
  Serial.begin( 115200 ); /* prepare for possible serial debug */
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);//recibir y enviar datos por el serial 2, comunicación entre placas
}




void loop(){
 // Serial.println("Inicio del bucle loop()");
  recibirDatos();
  enviarDatos(emergencia,bomba,temperaturaEnviada);
	lv_timer_handler(); /* let the GUI do its work */
	delay(5);

}



///////////////////////////////////////////////////////FUNCIONES AÑADIDAS/////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////FUNCIONES DE CONTROL DE TIEMPO/////////////////////////////////////////////////////////
// Función para iniciar la cuenta atrás
void iniciar_cuenta_atras(lv_obj_t * label, lv_obj_t * spinbox, int minutos) {
    segundos_restantes = minutos * 60;
    lv_timer_resume(timer_manual); // Reanudar el temporizador de LVGL
}


void timer_callback_manual(lv_timer_t * timer_manual) {
    if (segundos_restantes > 0) {
        segundos_restantes--;
        // Actualizar el label de cuenta atrás manual
        int horas = segundos_restantes / 3600;
        int minutos = (segundos_restantes % 3600) / 60;
        int segundos = segundos_restantes % 60;
        lv_label_set_text_fmt(ui_LCuentaAtrasManual, "%02d:%02d:%02d", horas, minutos, segundos);
        //Serial.print("timer_callback: ");Serial.println(segundos_restantes);
    } else {
        // Cuenta atrás manual terminada
        reiniciarSpinboxesTiempo();
        Serial.println("fin");
        lv_timer_pause(timer_manual);
        enviarDatos(false, false, 0);
        //Serial.println("cuenta atras terminada");
    }
}

void timer_callback_maceracion(lv_timer_t * timer_maceracion) {
    // Verificar si la maceración ha finalizado
    if (maceradofin == 1) {
        lv_label_set_text_fmt(ui_LTiempoRestante, "MACERACION FINALIZADA,\n CIERRA EL GRIFO");
        lv_obj_add_state(ui_BHervirAuto1, LV_STATE_DISABLED);
        bomba=0;
        if (!grifoAbierto) {
            lv_obj_clear_state(ui_BHervirAuto1, LV_STATE_DISABLED); // Habilitar botón
            lv_label_set_text_fmt(ui_LTiempoRestante, "GRIFO CERRADO");
            return; // Salir del callback si el grifo está cerrado
        }
    } 
    else {
        // Verificar si la temperatura está en rango o si tiempoMaceracion[indiceMaceracion] es cero
        if ((temperaturaRecibida >= temperaturaEnviada - 3 && temperaturaRecibida <= temperaturaEnviada + 3) || tiempoMaceracion[indiceMaceracion] == 0) {

            // Restar tiempo al periodo actual (solo si tiempoMaceracion[indiceMaceracion] no es cero)
            if (tiempoMaceracion[indiceMaceracion] != 0) {
                segundosRestantesMaceracion--;
                tiempoTotalMaceracion--;
            }

            // Actualizar los labels de tiempo (solo si tiempoMaceracion[indiceMaceracion] no es cero)
            if (tiempoMaceracion[indiceMaceracion] != 0) {
                lv_label_set_text_fmt(ui_LCuentaAtrasAuto, "%02d:%02d:%02d", segundosRestantesMaceracion / 3600, (segundosRestantesMaceracion % 3600) / 60, segundosRestantesMaceracion % 60);
                lv_label_set_text_fmt(ui_LTiempoRestante, "%02d:%02d:%02d", tiempoTotalMaceracion / 3600, (tiempoTotalMaceracion % 3600) / 60, tiempoTotalMaceracion % 60);
            }

            // Verificar si el periodo actual ha terminado o si tiempoMaceracion[indiceMaceracion] es cero
            if (segundosRestantesMaceracion == 0 || tiempoMaceracion[indiceMaceracion] == 0) {
                // Incrementar el índice del periodo
                indiceMaceracion++;

                // Verificar si quedan periodos
                if (indiceMaceracion < 4) {
                    // Iniciar el siguiente periodo
                    segundosRestantesMaceracion = tiempoMaceracion[indiceMaceracion] * 60;
                    bomba = bombaMaceracion[indiceMaceracion];
                    temperaturaEnviada = temperaturasAuto[indiceMaceracion];
                    enviarDatos(emergencia, bomba, temperaturaEnviada);
                } else {
                    // Finalizar la maceración
                    maceradofin = 1;
                    enviarDatos(false, false, 0); // Apagar bomba y calentador
                    lv_label_set_text_fmt(ui_LCuentaAtrasAuto, "COMPLETADO");
                }
            }
        } else {
            // La temperatura no está en rango, mostrar mensaje
            lv_label_set_text_fmt(ui_LTiempoRestante, "MODFICANDO TEMPERATURA");
        }
    }
}


void timer_callback_ebullicion(lv_timer_t * timer_ebullicion) {

    // Restar tiempo al periodo actual y al tiempo total si no se desfasa
    if (segundosRestantesEbullicion > 0) {
        segundosRestantesEbullicion--;
    }
    if (segundosRestantesPeriodoEbullicion > 0) {
        segundosRestantesPeriodoEbullicion--;
    }

    // Actualizar los labels de tiempo
    lv_label_set_text_fmt(ui_LCuentaAtrasAutoEbuTotal, "%02d:%02d:%02d", segundosRestantesEbullicion / 3600, (segundosRestantesEbullicion % 3600) / 60, segundosRestantesEbullicion % 60);
    lv_label_set_text_fmt(ui_LCuentaAtrasAutoEbuPeriodo, "%02d:%02d:%02d", segundosRestantesPeriodoEbullicion / 3600, (segundosRestantesPeriodoEbullicion % 3600) / 60, segundosRestantesPeriodoEbullicion % 60);

    // Verificar si el periodo actual ha terminado
    if (segundosRestantesPeriodoEbullicion == 0) {
        // Activar la alarma
        Alarma();

        // Incrementar el índice del periodo
        indiceEbullicion++;

        // Verificar si quedan periodos
        if (indiceEbullicion < 4 && tiempoAvisoEbull[indiceEbullicion] > 0) { // <-- Verificar si el tiempo del siguiente periodo es mayor que cero
            // Iniciar el siguiente periodo
            segundosRestantesPeriodoEbullicion = tiempoAvisoEbull[indiceEbullicion] * 60;
        } else {
            // Finalizar la ebullición
            lv_timer_pause(timer_ebullicion); // Detener el temporizador
            lv_label_set_text_fmt(ui_LCuentaAtrasAutoEbuTotal, "COMPLETADO");
            lv_label_set_text_fmt(ui_LCuentaAtrasAutoEbuPeriodo, "COMPLETADO");
            lv_obj_clear_state(ui_BFin, LV_STATE_DISABLED);
        }
    }
}


///////////////////////////////////////////////////FUNCIONES DE ACTUALIZACIÓN IU Y AVISO////////////////////////////////////////////////////////////////////////////////
void reiniciarSpinboxesTiempo() {
  // Spinboxes de maceración
  lv_spinbox_set_value(ui_STpoAuto1, 0);
  lv_spinbox_set_value(ui_STpoAuto2, 0);
  lv_spinbox_set_value(ui_STpoAuto3, 0);
  lv_spinbox_set_value(ui_STpoAuto4, 0);

  // Spinboxes de ebullición
  lv_spinbox_set_value(ui_STpoAutoAviso1, 0);
  lv_spinbox_set_value(ui_STpoAutoAviso2, 0);
  lv_spinbox_set_value(ui_STpoAutoAviso3, 0);
  lv_spinbox_set_value(ui_STpoAutoAviso4, 0);

  // Spinbox manual
  lv_spinbox_set_value(ui_STpo, 0);
}


void actualizarLabelsTemperatura(void) {
    // Labels en modo manual
  char tempStr[10]; // Buffer para almacenar la cadena de texto, que si no no actualiza los labels..
  dtostrf(temperaturaRecibida, 6, 1, tempStr);

  lv_label_set_text(ui_LTempLeida, tempStr);
    lv_label_set_text(ui_LTempLeidaCaleManual, tempStr);
    lv_label_set_text(ui_LTempLeidaMaceAuto, tempStr);
    lv_label_set_text(ui_LTempLeidaEbullAuto, tempStr);
}

void Alarma() {
  tone(PIN_BUZZER, 1000); // Emitir un tono continuo de 1000 Hz
  delay(1000);           // Duración del tono (1 segundo)
  noTone(PIN_BUZZER);    // Detener el tono
}


//////////////////////////////////////FUNCIONES AÑADIDAS EN SQUARELINE SE INVOCAN DESDE LA IU//////////////////////////////////////////////////////////////////////////////////////////////

//Funcion de pantalla CaleManualActivado, se activa al acceder a la pantalla, mira a ver si hay temporizador, lo activa y activa el calefactor a la temperatura solicitada. 
void CalefactorManual (lv_event_t * e){ 

    lv_label_set_text_fmt(ui_LTempObjCaleManual, "%d", lv_spinbox_get_value(ui_STemp));
    int tempoManu = lv_obj_has_state(ui_CTemporizador, LV_STATE_CHECKED);//checkbox de temporizador
    temperaturaEnviada = lv_spinbox_get_value(ui_STemp);
    if (tempoManu){
        int tpo_manu = lv_spinbox_get_value(ui_STpo);
        iniciar_cuenta_atras(ui_LCuentaAtrasManual, ui_STpo, tpo_manu); // Iniciar la cuenta atrás si no se declara así peta, brujería!!
        enviarDatos(emergencia,bomba,temperaturaEnviada);
    }
    else{
      enviarDatos(emergencia,bomba,temperaturaEnviada);
    }
}

//Función de parada de emergencia, en los distintos botones de STOP
void ParadaEmergencia(lv_event_t * e) {
  emergencia = 1;
  reiniciarSpinboxesTiempo();
  lv_timer_pause(timer_manual);
  lv_timer_pause(timer_maceracion);
  lv_timer_pause(timer_ebullicion);
  enviarDatos(emergencia,bomba,0);
}


//Funcion para resetear la emergencia, se reseta al volver a la pantalla de inicio
void resetEmergencia(lv_event_t * e) {
  emergencia = 0;
}


//Boton activar bomba en pantalla TempManual, si grifoAbierto=0, no debería de estar habilitado el botón, pero se hace un doble check
void FactivarBomba(lv_event_t * e){
    if(grifoAbierto==1){
    bomba=1;
    }
}


//Boton parar bomba en pantalla TempManual, solamente se habilita al pulsr el de activar bomba
void FpararBomba(lv_event_t * e){
  bomba=0;
}


//Funcion de tomar datos, se activa al darle al botón Empezar en la pantalla "AutoHervido"
void TomarDatosAuto (lv_event_t * e){
// Obtener valores de los spinboxes de temperatura y tiempo
  temperaturasAuto[0] = lv_spinbox_get_value(ui_STempAuto1);
  tiempoMaceracion[0] = lv_spinbox_get_value(ui_STpoAuto1);
  //si no hay tiempo de maceración, ponemos la temperatura a 0 para no activar el ssr
  if(tiempoMaceracion[0]==0){
    temperaturasAuto[0]=0;
  }
  else if (temperaturasAuto[0]==0){
    tiempoMaceracion[0]=0;
  }


  temperaturasAuto[1] = lv_spinbox_get_value(ui_STempAuto2);
  tiempoMaceracion[1] = lv_spinbox_get_value(ui_STpoAuto2);
  if(tiempoMaceracion[1]==0){
    temperaturasAuto[1]=0;
  }
  else if (temperaturasAuto[1]==0){
    tiempoMaceracion[1]=0;
  }

  temperaturasAuto[2] = lv_spinbox_get_value(ui_STempAuto3);
  tiempoMaceracion[2] = lv_spinbox_get_value(ui_STpoAuto3);
    if(tiempoMaceracion[2]==0){
    temperaturasAuto[2]=0;
  }
  else if (temperaturasAuto[2]==0){
    tiempoMaceracion[2]=0;
  }

  temperaturasAuto[3] = lv_spinbox_get_value(ui_STempAuto4);
  tiempoMaceracion[3] = lv_spinbox_get_value(ui_STpoAuto4);
    if(tiempoMaceracion[3]==0){
    temperaturasAuto[3]=0;
  }
  else if (temperaturasAuto[3]==0){
    tiempoMaceracion[3]=0;
  }

   //recirculado si o no
  bombaMaceracion[0] = lv_obj_has_state(ui_CRecirculadoAuto1, LV_STATE_CHECKED);
  bombaMaceracion[1] = lv_obj_has_state(ui_CRecirculadoAuto2, LV_STATE_CHECKED);
  bombaMaceracion[2] = lv_obj_has_state(ui_CRecirculadoAuto3, LV_STATE_CHECKED);
  bombaMaceracion[3] = lv_obj_has_state(ui_CRecirculadoAuto4, LV_STATE_CHECKED);
  
  //cada cuanto avisa durante la ebullicion
  tiempoAvisoEbull[0] = lv_spinbox_get_value(ui_STpoAutoAviso1);
  tiempoAvisoEbull[1] = lv_spinbox_get_value(ui_STpoAutoAviso2);
  tiempoAvisoEbull[2] = lv_spinbox_get_value(ui_STpoAutoAviso3);
  tiempoAvisoEbull[3] = lv_spinbox_get_value(ui_STpoAutoAviso4);

  tiempoTotalMaceracion = 0;
  for (int i = 0; i < 4; i++) {
      tiempoTotalMaceracion += tiempoMaceracion[i] * 60;
  }
  lv_obj_add_state(ui_BFin, LV_STATE_DISABLED);
}


//Función de iniciar la maceración, se acitva al cargar la pantalla AutoMaceradoStatus
void IniciarMaceracion(lv_event_t * e) {
    maceradofin=0;
    indiceMaceracion = 0; // Índice del periodo actual (0 al inicio)
    segundosRestantesMaceracion = tiempoMaceracion[indiceMaceracion] * 60; // Inicializar el tiempo del primer periodo
    bomba=bombaMaceracion[indiceMaceracion];
    temperaturaEnviada=temperaturasAuto[indiceMaceracion];
    // Enviar datos del primer periodo
    enviarDatos(emergencia, bomba, temperaturaEnviada);
    //Serial.print(emergencia);Serial.print(bombaMaceracion[indiceMaceracion]);Serial.println(temperaturasAuto[indiceMaceracion]);
    // Iniciar el temporizador
    lv_timer_resume(timer_maceracion);
}


void Ebullicion(lv_event_t * e) {
    static int indiceEbullicion = 0; // Índice del periodo actual
    // Calcular el tiempo total de ebullición y reiniciar el contador
    segundosRestantesEbullicion = 0;
    
    for (int i = 0; i < 4; i++) {
        segundosRestantesEbullicion += tiempoAvisoEbull[i] * 60;
    }
    // Inicializar el tiempo restante del primer periodo
    segundosRestantesPeriodoEbullicion = tiempoAvisoEbull[indiceEbullicion] * 60;

    // Verificar temperatura antes de iniciar
    if (temperaturaRecibida >= 97.0) {
        // Enviar datos a la otra placa (sin emergencia, sin bomba, temperatura objetivo alta)
        enviarDatos(0, 0, 100);
        lv_timer_resume(timer_ebullicion);
   } else {
        // Mostrar mensaje si la temperatura no es suficiente
        lv_label_set_text_fmt(ui_LCuentaAtrasAutoEbuTotal, "MODIFICANDO\n TEMPERATURA");
    }
}



/////////////////////////////FUNCIONES DE COMUNICACION ENTRE PLACAS//////////////////////////////////////////////
void enviarDatos(bool emergencia, bool bomba, int temperaturaEnviada){ 
  StaticJsonDocument<128> datosEnviar; // Buffer de 256 bytes, más vale que sobre
  
  datosEnviar["ParadaEmergencia"] = emergencia;// Para el calefactor y corta los reles
  datosEnviar["ActivarBomba"] = bomba; // Activación manual bomba
  datosEnviar["temperaturaEnviada"] = temperaturaEnviada;// Temperatura a enviar

  String jsonEnviar;
  serializeJson( datosEnviar, jsonEnviar);
  Serial2.println(jsonEnviar);

  //Imprimir los datos enviados para depuración
  /*Serial.println("Datos enviados:");
  Serial.print("  Parada Emergencia: "); Serial.println(emergencia);
  Serial.print("  Activar Bomba: "); Serial.println(bomba);
  Serial.print("  temperaturaEnviada: "); Serial.println(temperaturaEnviada);*/
}


void recibirDatos() {//
  if (Serial2.available()) {
    String jsonRecibido = Serial2.readStringUntil('\n');

    StaticJsonDocument<128> doc; // Hay que ajustar para que no de muchos fallos
    DeserializationError error = deserializeJson(doc, jsonRecibido);

    if (error) {
      Serial.print(F("Error al deserializar JSON: "));
      Serial.println(error.f_str());
      return;
    }
    else{
      temperaturaRecibida = doc["temperaturaLeida"];
      grifoAbierto = doc["grifoAbierto"];
      /*Serial.println("Datos recibidos:");
      Serial.print("  Temperatura: "); Serial.println(temperaturaRecibida);
      Serial.print("  grifoAbierto: "); Serial.println(grifoAbierto);*/
      actualizarLabelsTemperatura();
      if (grifoAbierto==1){
        lv_obj_set_style_bg_color(ui_BGrifoLeer, lv_color_hex(0x007400), 0);
        lv_obj_clear_state(ui_BActivarBombaManual, LV_STATE_DISABLED);
        if(bomba==1){
          lv_obj_add_state(ui_BActivarBombaManual, LV_STATE_DISABLED);
          lv_obj_clear_state(ui_BPararBombaManual, LV_STATE_DISABLED);

        }
      }
      else{
        lv_obj_set_style_bg_color(ui_BGrifoLeer, lv_color_hex(0xFC0000), 0);
        lv_obj_add_state(ui_BActivarBombaManual, LV_STATE_DISABLED);
      }
  
    }
  }
}


