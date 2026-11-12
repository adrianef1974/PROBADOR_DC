
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Preferences.h>

// ==================================================
// PANTALLA ESP32-C6-LCD-1.47
// ==================================================

#define TFT_MOSI 6
#define TFT_SCLK 7
#define TFT_CS   14
#define TFT_DC   15
#define TFT_RST  21
#define TFT_BL   22

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ==================================================
// ENTRADAS Y SALIDA RESERVADAS
// ==================================================

#define PIN_TENSION   0
#define PIN_CORRIENTE 1
#define PIN_NTC       2
#define PIN_GATE      3
#define PIN_BOTON_TEST 23
#define PIN_BOTON_TANDA 4

const unsigned long ANTIRREBOTE_TEST = 50UL;
const unsigned long ANTIRREBOTE_TANDA = 50UL;
bool botonTandaAnterior = HIGH;
bool estadoEstableBotonTanda = HIGH;
unsigned long ultimoCambioBotonTanda = 0;
int ultimaCorrienteSolicitada = 100;
int ultimoTiempoSolicitado = 10;
int pwmRealPorcentaje = 0;
int pwmMaximoAlcanzado = 0;
unsigned long ultimaActualizacionRegulacion = 0;
float limiteCorrienteTrabajo = 0.0f;
bool buscandoCorriente = false;
int muestrasConsecutivasEnBanda = 0;
int muestrasConsecutivasBajoBanda = 0;
const unsigned long TIMEOUT_BUSQUEDA_CORRIENTE = 10000UL;
const float BANDA_CORRIENTE_MIN = 0.97f;
const float BANDA_CORRIENTE_MAX = 1.03f;
bool modoTest = false;
bool botonTestAnterior = HIGH;
unsigned long ultimoCambioBotonTest = 0;
int pwmTestPorcentaje = 0;
float tensionFicticia = 12.0f;
const int FRECUENCIA_PWM = 5000;
const int RESOLUCION_PWM = 8;

const float DIVISOR_TENSION = 13.5f;
const float SHUNT_OHMIOS = 0.1f;

float factorTension = 1.000f;
float factorCorriente = 1.000f;

float toleranciaPorcentaje = 2.0f;
// Tension nominal autorizada por el supervisor; cero significa sin configurar.
float tensionAutorizada = 0.0f;
float tensionMinimaEntrada = 3.0f;
float tensionMaximaEntrada = 48.0f;

// ==================================================
// CONFIGURACION FUTURA DE CORRIENTE
// ==================================================

const int CORRIENTE_MINIMA_CONFIGURABLE = 100;
const int CORRIENTE_MAXIMA_CONFIGURABLE = 10000;

// Limite inicial: 1 amper.
// Se puede configurar hasta 10 amper desde la web.

int corrienteMaximaConfigurada = 1000;
int limiteSupervisorDurantePrueba = 1000;

// ==================================================
// HABILITACIONES DE FUNCIONES
// ==================================================

bool controlPWMHabilitado = false;
bool regulacionCorrienteHabilitada = false;
bool shuntHabilitado = false;
bool salidaMOSFETHabilitada = false;
bool temperaturaHabilitada = false;
bool pruebaCargaHabilitada = false;
bool proteccionesPotenciaHabilitadas = false;
bool calibracionCorrienteHabilitada = false;

// ==================================================
// SEGURIDAD DE LA CARGA
// ==================================================

bool cargaHabilitada = false;

// ==================================================
// PRUEBA MANUAL DEL GATE
// ==================================================

bool pruebaGateActiva = false;
unsigned long inicioPruebaGate = 0;

const unsigned long DURACION_PRUEBA_GATE = 500UL;

const float TENSION_GATE_MINIMA = 3.0f;
const float TENSION_GATE_MAXIMA = 48.0f;

float corrienteMaximaPrueba = 0.0f;

// ==================================================
// LIMITE DE PROTECCION DE CORRIENTE
// ==================================================

float obtenerLimiteProteccionCorriente() {

  return corrienteMaximaConfigurada / 1000.0f;
}

// ==================================================
// WIFI
// ==================================================

const char* ssid = "PROBADOR_DC";
const char* password = "12345678";

WebServer server(80);
Preferences preferencias;

// ==================================================
// ESTADOS
// ==================================================

enum Estado {
  LISTO,
  EN_PRUEBA,
  APROBADO,
  RECHAZADO
};

Estado estado = LISTO;

// ==================================================
// VARIABLES
// ==================================================

int etapa = 0;

int corriente = 100;
int tiempoPrueba = 10;

int aprobadas = 0;
int rechazadas = 0;

float tensionReferencia = 0.0f;
float tensionMedida = 0.0f;
float corrienteMedida = 0.0f;

float tensionMinima = 0.0f;
float tensionMaxima = 0.0f;

float tensionCargaMinima = 99.0f;
float tensionCargaMaxima = 0.0f;
float tensionFinal = 0.0f;

bool configuracionAutorizada = false;
bool fuenteDetectada = false;
bool pruebaActiva = false;

unsigned long tiempoEtapa = 0;
unsigned long tiempoInicio = 0;
unsigned long ultimaPantalla = 0;
unsigned long ultimaLectura = 0;

unsigned long ultimaLecturaPrueba = 0;

String motivoFallo = "";

// ==================================================
// CONTROL DE TANDA, DESCARGA Y RESET
// ==================================================

bool tandaConResultados = false;
bool tandaFinalizada = false;

bool archivoCSVServido = false;
bool descargaConfirmada = false;

// ==================================================
// HISTORIAL CSV
// ==================================================

String historialCSV =
"Prueba,Tiempo_s,Tension_referencia_V,Tolerancia_pct,"
"Corriente_maxima_autorizada_supervisor_mA,Corriente_seleccionada_operario_mA,"
"Corriente_maxima_alcanzada_A,Tension_minima_carga_V,Tension_maxima_carga_V,"
"Tension_final_V,PWM_maximo_pct,Resultado,Motivo\r\n";

int numeroPrueba = 0;
int registrosTanda = 0;
int registrosTotales = 0;

// ==================================================
// FUNCIONES AUXILIARES
// ==================================================

String nombreEstado() {

  switch (estado) {

    case LISTO: return "LISTO";
    case EN_PRUEBA: return "EN PRUEBA";
    case APROBADO: return "APROBADO";
    case RECHAZADO: return "RECHAZADO";
  }

  return "LISTO";
}

float leerPromedioADC(int pin) {

  uint32_t suma = 0;
  const int muestras = 16;

  for (int i = 0; i < muestras; i++) {

    suma += analogReadMilliVolts(pin);
    delayMicroseconds(150);
  }

  return (float)suma / (float)muestras;
}

// ==================================================
// MEDICION SIN FACTOR DE CORRIENTE
// ==================================================

float leerCorrienteSinCalibrar() {

  float mvShunt = leerPromedioADC(PIN_CORRIENTE);

  float tensionShunt = mvShunt / 1000.0f;

  return tensionShunt / SHUNT_OHMIOS;
}

// ==================================================
// CALIBRACION AUTOMATICA DE CORRIENTE
// ==================================================

bool calibrarCorriente(float corrienteReferenciaA) {

  if (!calibracionCorrienteHabilitada) {
    Serial.println("CALIBRACION BLOQUEADA: DESHABILITADA");
    return false;
  }

  if (!shuntHabilitado) {
    Serial.println("CALIBRACION BLOQUEADA: SHUNT DESHABILITADO");
    return false;
  }

  if (modoTest || pruebaActiva || pruebaGateActiva) {
    Serial.println("CALIBRACION BLOQUEADA: MODO TEST O PRUEBA ACTIVA");
    return false;
  }

  // RANGO DE CALIBRACION PREPARADO:
  // 100 mA A 10 A.

  if (corrienteReferenciaA < 0.100f ||
      corrienteReferenciaA > 10.000f) {

    Serial.println("CALIBRACION BLOQUEADA: REFERENCIA FUERA DE RANGO");
    return false;
  }

  float corrienteSinCalibrar = leerCorrienteSinCalibrar();

  if (corrienteSinCalibrar < 0.005f) {

    Serial.println("CALIBRACION BLOQUEADA: LECTURA INSUFICIENTE");
    return false;
  }

  float nuevoFactor =
    corrienteReferenciaA / corrienteSinCalibrar;

  if (nuevoFactor < 0.500f ||
      nuevoFactor > 1.500f) {

    Serial.println("CALIBRACION BLOQUEADA: FACTOR FUERA DE RANGO");
    return false;
  }

  factorCorriente = nuevoFactor;

  preferencias.putFloat("factorI", factorCorriente);

  actualizarMediciones();

  Serial.println("CALIBRACION DE CORRIENTE COMPLETADA");

  Serial.print("Corriente de referencia: ");
  Serial.print(corrienteReferenciaA, 3);
  Serial.println(" A");

  Serial.print("Corriente sin calibrar: ");
  Serial.print(corrienteSinCalibrar, 3);
  Serial.println(" A");

  Serial.print("Nuevo factor: ");
  Serial.println(factorCorriente, 4);

  return true;
}

// ==================================================
// PWM INVERTIDO Y MODO TEST
// ==================================================

void establecerPWM(uint8_t porcentajeActivacion) {

  porcentajeActivacion = constrain(porcentajeActivacion, 0, 100);

  if (porcentajeActivacion == 0) {
    digitalWrite(PIN_GATE, HIGH);   // Totem-pole inactivo
    return;
  }

  if (porcentajeActivacion == 100) {
    digitalWrite(PIN_GATE, LOW);    // Totem-pole activo
    return;
  }

  uint8_t dutyGPIO = 255 - (uint8_t)round(
    (porcentajeActivacion * 255.0f) / 100.0f
  );

  analogWrite(PIN_GATE, dutyGPIO);
}

void entrarModoTest() {

  if (pruebaActiva || pruebaGateActiva || tandaFinalizada) return;

  modoTest = true;
  pwmTestPorcentaje = 0;
  establecerPWM(0);

  // No se utilizan las entradas ADC reales en modo TEST.
  tensionMedida = tensionFicticia;
  corrienteMedida = 0.0f;
  fuenteDetectada = false;

  Serial.println("MODO TEST ACTIVADO");
  Serial.println("LECTURAS REALES ADC SUSPENDIDAS");
  Serial.println("PWM MANUAL DISPONIBLE: 0 A 100%");
}

void salirModoTest() {

  establecerPWM(0);
  pwmTestPorcentaje = 0;
  modoTest = false;

  detectarFuenteReal();

  Serial.println("MODO TEST DESACTIVADO");
  Serial.println("PWM EN 0% - MOSFET APAGADO");
}

void actualizarBotonTest() {

  bool lectura = digitalRead(PIN_BOTON_TEST);

  if (lectura != botonTestAnterior) {
    ultimoCambioBotonTest = millis();
    botonTestAnterior = lectura;
  }

  static bool estadoEstable = HIGH;

  if ((millis() - ultimoCambioBotonTest) >= ANTIRREBOTE_TEST &&
      lectura != estadoEstable) {

    estadoEstable = lectura;

    if (estadoEstable == LOW) {
      if (modoTest) salirModoTest();
      else entrarModoTest();
    }
  }
}

// ==================================================
// MEDICIONES REALES
// ==================================================

void actualizarMediciones() {

  if (modoTest) return;

  float mvTension = leerPromedioADC(PIN_TENSION);

  tensionMedida = (mvTension / 1000.0f) *
                  DIVISOR_TENSION * factorTension;

  if (shuntHabilitado) {

    float corrienteSinCalibrar =
      leerCorrienteSinCalibrar();

    corrienteMedida =
      corrienteSinCalibrar * factorCorriente;

  } else {

    corrienteMedida = 0.0f;
  }

  fuenteDetectada = (tensionMedida >= 1.0f);
}

// ==================================================
// DETECTAR FUENTE REAL
// ==================================================

void detectarFuenteReal() {

  actualizarMediciones();

  tensionReferencia = 0.0f;
  tensionMinima = 0.0f;
  tensionMaxima = 0.0f;

  tensionCargaMinima = 99.0f;
  tensionCargaMaxima = 0.0f;
  tensionFinal = 0.0f;

  corrienteMaximaPrueba = 0.0f;

  estado = LISTO;
  etapa = 0;
  pruebaActiva = false;

  establecerPWM(0);

  Serial.println();

  if (fuenteDetectada) {

    Serial.println("FUENTE REAL DETECTADA");

    Serial.print("Tension medida: ");
    Serial.print(tensionMedida, 2);
    Serial.println(" V");

  } else {

    Serial.println("SIN FUENTE DETECTADA");
  }
}

// ==================================================
// PRUEBA MANUAL DEL GATE
// ==================================================

bool iniciarPruebaGate() {

  if (modoTest) return false;

  if (!configuracionAutorizada) {
    Serial.println("GATE BLOQUEADO: FALTA AUTORIZACION");
    return false;
  }

  if (pruebaActiva || pruebaGateActiva) {
    Serial.println("GATE BLOQUEADO: OTRA PRUEBA ACTIVA");
    return false;
  }

  actualizarMediciones();

  if (!fuenteDetectada ||
      tensionMedida < TENSION_GATE_MINIMA ||
      tensionMedida > TENSION_GATE_MAXIMA) {

    Serial.println("GATE BLOQUEADO: SE REQUIEREN ENTRE 3 Y 48 V");
    return false;
  }

  establecerPWM(100);

  pruebaGateActiva = true;
  inicioPruebaGate = millis();

  Serial.println("PRUEBA GATE INICIADA");
  Serial.println("MOSFET ACTIVADO AL 100% DURANTE 500 ms (GPIO3 LOW)");

  return true;
}

void actualizarPruebaGate() {

  if (!pruebaGateActiva) return;

  if (millis() - inicioPruebaGate >= DURACION_PRUEBA_GATE) {

    establecerPWM(0);

    pruebaGateActiva = false;

    Serial.println("PRUEBA GATE FINALIZADA");
    Serial.println("GPIO3 HIGH - MOSFET APAGADO");
  }
}

// ==================================================
// REGULACION PWM REAL POR REALIMENTACION DE CORRIENTE
// ==================================================

const unsigned long INTERVALO_REGULACION_PWM = 100UL;
const float MARGEN_CORTE_CORRIENTE = 1.10f;

void regularPWMReal(unsigned long ahora) {
  if (!pruebaActiva || etapa != 2) return;
  if (!controlPWMHabilitado || !regulacionCorrienteHabilitada) return;
  if (ahora - ultimaActualizacionRegulacion < INTERVALO_REGULACION_PWM) return;
  ultimaActualizacionRegulacion = ahora;

  const float objetivoA = corriente / 1000.0f;
  if (objetivoA <= 0.0f) { pwmRealPorcentaje = 0; establecerPWM(0); return; }

  // Rampa gradual y correccion alrededor de la consigna.
  if (corrienteMedida < objetivoA * 0.90f) pwmRealPorcentaje = min(100, pwmRealPorcentaje + 1);
  else if (corrienteMedida > objetivoA * 1.03f) pwmRealPorcentaje = max(0, pwmRealPorcentaje - 2);
  else if (corrienteMedida > objetivoA) pwmRealPorcentaje = max(0, pwmRealPorcentaje - 1);
  else if (corrienteMedida < objetivoA * 0.97f) pwmRealPorcentaje = min(100, pwmRealPorcentaje + 1);

  pwmRealPorcentaje = constrain(pwmRealPorcentaje, 0, 100);
  establecerPWM((uint8_t)pwmRealPorcentaje);
  if (pwmRealPorcentaje > pwmMaximoAlcanzado) pwmMaximoAlcanzado = pwmRealPorcentaje;
}

// ==================================================
// INICIAR PRUEBA
// ==================================================

bool iniciarPrueba(int corrienteSolicitada, int tiempoSolicitado) {

  if (modoTest || pruebaActiva || pruebaGateActiva || tandaFinalizada) return false;

  if (!cargaHabilitada || !pruebaCargaHabilitada || !shuntHabilitado ||
      !salidaMOSFETHabilitada || !proteccionesPotenciaHabilitadas ||
      !controlPWMHabilitado || !regulacionCorrienteHabilitada) {
    Serial.println("INICIO BLOQUEADO: REVISAR HABILITACIONES DE SEGURIDAD");
    return false;
  }

  if (tensionAutorizada < 3.0f || tensionAutorizada > 48.0f) {
    Serial.println("INICIO BLOQUEADO: EL SUPERVISOR NO CONFIGURO LA TENSION AUTORIZADA");
    return false;
  }

  if (corrienteSolicitada < CORRIENTE_MINIMA_CONFIGURABLE ||
      corrienteSolicitada > corrienteMaximaConfigurada ||
      tiempoSolicitado < 1 || tiempoSolicitado > 10) {
    Serial.println("INICIO BLOQUEADO: CORRIENTE O TIEMPO FUERA DE RANGO");
    return false;
  }

  // Comprobacion de la fuente con el MOSFET apagado.
  actualizarMediciones();
  float margen = tensionAutorizada * toleranciaPorcentaje / 100.0f;
  float minimo = tensionAutorizada - margen;
  float maximo = tensionAutorizada + margen;

  if (!fuenteDetectada || tensionMedida < minimo || tensionMedida > maximo) {
    Serial.println("INICIO NO REGISTRADO: FUENTE FUERA DEL RANGO AUTORIZADO");
    Serial.print("Esperada: "); Serial.print(tensionAutorizada, 2);
    Serial.print(" V +/- "); Serial.print(toleranciaPorcentaje, 1);
    Serial.print(" % | Medida: "); Serial.print(tensionMedida, 2);
    Serial.println(" V");
    establecerPWM(0);
    estado = LISTO;
    etapa = 0;
    pruebaActiva = false;
    motivoFallo = "";
    return false;
  }

  corriente = corrienteSolicitada;
  tiempoPrueba = constrain(tiempoSolicitado, 1, 10);
  ultimaCorrienteSolicitada = corriente;
  ultimoTiempoSolicitado = tiempoPrueba;
  preferencias.putInt("ultCorriente", ultimaCorrienteSolicitada);
  preferencias.putInt("ultTiempo", ultimoTiempoSolicitado);

  // Referencia fija del supervisor, no la lectura instantanea.
  tensionReferencia = tensionAutorizada;
  tensionMinima = minimo;
  tensionMaxima = maximo;
  tensionCargaMinima = tensionMedida;
  tensionCargaMaxima = tensionMedida;
  tensionFinal = 0.0f;
  corrienteMaximaPrueba = 0.0f;
  pwmRealPorcentaje = 0;
  pwmMaximoAlcanzado = 0;
  ultimaActualizacionRegulacion = 0;
  limiteSupervisorDurantePrueba = corrienteMaximaConfigurada;
  limiteCorrienteTrabajo = min(obtenerLimiteProteccionCorriente(),
                               (corriente / 1000.0f) * MARGEN_CORTE_CORRIENTE);

  muestrasConsecutivasEnBanda = 0;
  muestrasConsecutivasBajoBanda = 0;
  buscandoCorriente = true;
  motivoFallo = "";
  establecerPWM(0);

  etapa = 1;
  estado = EN_PRUEBA;
  pruebaActiva = true;
  tiempoInicio = millis();
  tiempoEtapa = tiempoInicio;
  ultimaLecturaPrueba = 0;

  Serial.println("INICIO DE PRUEBA CV");
  Serial.print("Referencia supervisor: "); Serial.print(tensionReferencia, 2);
  Serial.print(" V +/- "); Serial.print(toleranciaPorcentaje, 1); Serial.println(" %");
  Serial.print("Corriente elegida por operario: "); Serial.print(corriente);
  Serial.println(" mA");
  Serial.println("ETAPA 1: VACIO INICIAL - 2 SEGUNDOS");
  return true;
}

// ==================================================
// REGISTRAR RESULTADO
// ==================================================

void guardarEstadoTanda() {
  preferencias.putInt("numPrueba", numeroPrueba);
  preferencias.putInt("regTanda", registrosTanda);
  preferencias.putInt("regTotal", registrosTotales);
  preferencias.putInt("aprobadas", aprobadas);
  preferencias.putInt("rechazadas", rechazadas);
  preferencias.putBool("tandaHay", tandaConResultados);
  preferencias.putBool("tandaFin", tandaFinalizada);
  preferencias.putBool("csvServido", archivoCSVServido);
  preferencias.putBool("csvConf", descargaConfirmada);
}

void registrarResultado() {
  unsigned long duracion = (millis() - tiempoInicio) / 1000UL;
  String resultado = nombreEstado();
  String motivo = (estado == RECHAZADO) ? motivoFallo : "NINGUNO";

  // El correlativo se asigna al finalizar, nunca al iniciar.
  numeroPrueba++;
  registrosTanda++;
  registrosTotales++;
  if (estado == RECHAZADO) rechazadas++;
  else if (estado == APROBADO) aprobadas++;

  String fila = String(numeroPrueba) + ",";
  fila += String(duracion) + ",";
  fila += String(tensionReferencia, 2) + ",";
  fila += String(toleranciaPorcentaje, 1) + ",";
  fila += String(limiteSupervisorDurantePrueba) + ",";
  fila += String(corriente) + ",";
  fila += String(corrienteMaximaPrueba, 3) + ",";
  fila += String(tensionCargaMinima, 2) + ",";
  fila += String(tensionCargaMaxima, 2) + ",";
  fila += String(tensionFinal, 2) + ",";
  fila += String(pwmMaximoAlcanzado) + ",";
  fila += resultado + ",";
  fila += motivo + "\r\n";

  historialCSV += fila;
  String clave = "r" + String(registrosTotales);
  preferencias.putString(clave.c_str(), fila);
  tandaConResultados = true;
  guardarEstadoTanda();

  Serial.println("RESULTADO REGISTRADO Y GUARDADO");
  Serial.println(resultado);
}

void terminarPrueba(bool aprobada) {
  establecerPWM(0);
  pruebaActiva = false;
  buscandoCorriente = false;
  estado = aprobada ? APROBADO : RECHAZADO;
  registrarResultado();
  Serial.println("PRUEBA FINALIZADA");
  Serial.println("GPIO3 HIGH - MOSFET APAGADO");
}

// ==================================================
// ACTUALIZAR PRUEBA
// ==================================================

void actualizarPrueba() {
  if (modoTest || !pruebaActiva) return;

  unsigned long ahora = millis();
  bool muestraNueva = false;
  if (ahora - ultimaLecturaPrueba >= 50UL) {
    ultimaLecturaPrueba = ahora;
    actualizarMediciones();
    muestraNueva = true;
  }

  // ETAPA 1: VACIO INICIAL - 2 SEGUNDOS.
  if (etapa == 1) {
    if (ahora - tiempoEtapa >= 2000UL) {
      if (!fuenteDetectada || tensionMedida < tensionMinima || tensionMedida > tensionMaxima) {
        motivoFallo = "FUENTE_DESCONECTADA_O_TENSION_FUERA_DE_TOLERANCIA_EN_VACIO_INICIAL";
        terminarPrueba(false);
        return;
      }

      etapa = 2;
      tiempoEtapa = ahora;
      buscandoCorriente = true;
      muestrasConsecutivasEnBanda = 0;
      muestrasConsecutivasBajoBanda = 0;
      tensionCargaMinima = 99.0f;
      tensionCargaMaxima = 0.0f;
      corrienteMaximaPrueba = 0.0f;
      pwmRealPorcentaje = 0;
      pwmMaximoAlcanzado = 0;
      ultimaActualizacionRegulacion = ahora;
      establecerPWM(0);
      Serial.println("ETAPA 2A: BUSQUEDA DE CORRIENTE - MAXIMO 10 SEGUNDOS");
    }
    return;
  }

  // ETAPA 2: BUSQUEDA (2A) Y MANTENIMIENTO (2B).
  if (etapa == 2) {
    if (!pruebaCargaHabilitada || !cargaHabilitada || !shuntHabilitado ||
        !salidaMOSFETHabilitada || !proteccionesPotenciaHabilitadas ||
        !controlPWMHabilitado || !regulacionCorrienteHabilitada) {
      motivoFallo = "HABILITACION_DESACTIVADA_DURANTE_PRUEBA";
      terminarPrueba(false);
      return;
    }

    if (muestraNueva) {
      if (tensionMedida < tensionCargaMinima) tensionCargaMinima = tensionMedida;
      if (tensionMedida > tensionCargaMaxima) tensionCargaMaxima = tensionMedida;
      if (corrienteMedida > corrienteMaximaPrueba) corrienteMaximaPrueba = corrienteMedida;

      // Supervision continua de tension durante busqueda y mantenimiento.
      if (!fuenteDetectada || tensionMedida < tensionMinima || tensionMedida > tensionMaxima) {
        motivoFallo = !fuenteDetectada ? "FUENTE_DESCONECTADA_DURANTE_CARGA" :
                                         "TENSION_FUERA_DE_TOLERANCIA_DURANTE_CARGA";
        terminarPrueba(false);
        return;
      }

      // Dos muestras nuevas consecutivas confirman una caida real de corriente.
      if (!buscandoCorriente) {
        float objetivo = corriente / 1000.0f;
        if (corrienteMedida < objetivo * BANDA_CORRIENTE_MIN) {
          muestrasConsecutivasBajoBanda++;
          if (muestrasConsecutivasBajoBanda >= 2) {
            motivoFallo = "CORRIENTE_NO_SOSTENIDA_DURANTE_MANTENIMIENTO";
            terminarPrueba(false);
            return;
          }
        } else {
          muestrasConsecutivasBajoBanda = 0;
        }
      }
    }

    if (corrienteMedida > limiteCorrienteTrabajo) {
      motivoFallo = "SOBRECORRIENTE_LIMITE_DE_TRABAJO";
      terminarPrueba(false);
      return;
    }

    regularPWMReal(ahora);

    if (buscandoCorriente) {
      if (muestraNueva) {
        float objetivo = corriente / 1000.0f;
        if (corrienteMedida >= objetivo * BANDA_CORRIENTE_MIN &&
            corrienteMedida <= objetivo * BANDA_CORRIENTE_MAX) {
          muestrasConsecutivasEnBanda++;
        } else {
          muestrasConsecutivasEnBanda = 0;
        }

        if (muestrasConsecutivasEnBanda >= 3) {
          buscandoCorriente = false;
          muestrasConsecutivasBajoBanda = 0;
          tiempoEtapa = ahora; // Aqui comienza el tiempo de mantenimiento.
          Serial.println("ETAPA 2B: CORRIENTE ALCANZADA - INICIA MANTENIMIENTO");
        }
      }

      if (buscandoCorriente && ahora - tiempoEtapa >= TIMEOUT_BUSQUEDA_CORRIENTE) {
        motivoFallo = "NO_ALCANZO_CORRIENTE_OBJETIVO_EN_10_SEGUNDOS";
        terminarPrueba(false);
        return;
      }
    } else if (ahora - tiempoEtapa >= (unsigned long)tiempoPrueba * 1000UL) {
      establecerPWM(0);
      etapa = 3;
      tiempoEtapa = ahora;
      Serial.println("ETAPA 2B FINALIZADA: ETAPA 3 VACIO FINAL");
    }
    return;
  }

  // ETAPA 3: VACIO FINAL - 2 SEGUNDOS.
  if (etapa == 3 && ahora - tiempoEtapa >= 2000UL) {
    tensionFinal = tensionMedida;
    if (!fuenteDetectada || tensionFinal < tensionMinima || tensionFinal > tensionMaxima) {
      motivoFallo = "FALLO_RECUPERACION_TENSION_EN_VACIO_FINAL";
      terminarPrueba(false);
    } else {
      terminarPrueba(true);
    }
  }
}

// ==================================================
// BOTON GPIO4 - CICLO DE PRUEBA / TANDA / CSV / RESET
// ==================================================

void actualizarBotonTanda() {
  bool lectura = digitalRead(PIN_BOTON_TANDA);
  if (lectura != botonTandaAnterior) {
    ultimoCambioBotonTanda = millis();
    botonTandaAnterior = lectura;
  }
  if (millis() - ultimoCambioBotonTanda < ANTIRREBOTE_TANDA) return;
  if (lectura == estadoEstableBotonTanda) return;
  estadoEstableBotonTanda = lectura;
  if (estadoEstableBotonTanda != LOW) return;
  if (modoTest || pruebaActiva || pruebaGateActiva) return;

  if (!tandaConResultados && !tandaFinalizada) {
    if (!iniciarPrueba(ultimaCorrienteSolicitada, ultimoTiempoSolicitado))
      Serial.println("GPIO4: INICIO BLOQUEADO - REVISAR CONFIGURACION Y FUENTE");
    return;
  }
  if (tandaConResultados && !tandaFinalizada) { finalizarTanda(); return; }
  if (tandaFinalizada) {
    if (archivoCSVServido && descargaConfirmada) resetearTanda();
    else Serial.println("GPIO4: DESCARGAR RESULTADOS ANTES DE CONTINUAR");
  }
}

// ==================================================
// BOTON FIN - FINALIZAR TANDA
// ==================================================

void finalizarTanda() {

  if (!tandaConResultados || pruebaActiva || tandaFinalizada) {

    Serial.println("FIN BLOQUEADO");
    return;
  }

  tandaFinalizada = true;
  archivoCSVServido = false;
  descargaConfirmada = false;
  guardarEstadoTanda();

  Serial.println("TANDA FINALIZADA - DESCARGA CSV HABILITADA");
}

// ==================================================
// BOTON RESET - NUEVA TANDA
// ==================================================

void resetearTanda() {

  if (!tandaFinalizada || !archivoCSVServido ||
      !descargaConfirmada || pruebaActiva) {

    Serial.println("RESET BLOQUEADO");
    return;
  }

  pruebaActiva = false;
  pruebaGateActiva = false;

  establecerPWM(0);

  estado = LISTO;
  etapa = 0;
  aprobadas = 0;
  rechazadas = 0;
  registrosTanda = 0;

  tandaConResultados = false;
  tandaFinalizada = false;
  archivoCSVServido = false;
  descargaConfirmada = false;
  guardarEstadoTanda();

  detectarFuenteReal();

  Serial.println("NUEVA TANDA PREPARADA - HISTORIAL CSV CONSERVADO");
}

// ==================================================
// PANTALLA LCD
// ==================================================

void mostrarPantalla() {

  tft.fillScreen(ST77XX_BLACK);

  if (modoTest) {

    tft.setTextColor(ST77XX_MAGENTA);
    tft.setTextSize(2);
    tft.setCursor(10, 12);
    tft.println("MODO TEST");
    tft.drawFastHLine(5, 42, 162, ST77XX_WHITE);

    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.setCursor(10, 58);
    tft.println("TENSION FICTICIA");

    tft.setTextColor(ST77XX_CYAN);
    tft.setTextSize(3);
    tft.setCursor(10, 78);
    tft.print(tensionFicticia, 1);
    tft.println(" V");

    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 132);
    tft.println("PWM GATE");

    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(3);
    tft.setCursor(10, 158);
    tft.print(pwmTestPorcentaje);
    tft.println(" %");

    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.setCursor(10, 210);
    tft.println("FRECUENCIA: 5 kHz");
    tft.setCursor(10, 230);
    tft.println("RESOLUCION: 8 bits");
    tft.setCursor(10, 250);
    tft.println(pwmTestPorcentaje == 0 ?
                "GATE: APAGADO" : "GATE: PWM ACTIVO");
    tft.setCursor(10, 285);
    tft.println("GPIO23: CAMBIAR MODO");
    return;
  }

  if (tandaFinalizada && (!archivoCSVServido || !descargaConfirmada)) {
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(2);
    tft.setCursor(8, 35);  tft.println("DESCARGAR");
    tft.setCursor(8, 60);  tft.println("RESULTADOS");
    tft.setCursor(8, 85);  tft.println("ANTES DE");
    tft.setCursor(8, 110); tft.println("CONTINUAR");
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.setCursor(8, 160); tft.println("Descarga el CSV desde");
    tft.setCursor(8, 175); tft.println("la pagina web.");
    tft.setCursor(8, 210); tft.println("Luego pulsa GPIO4");
    tft.setCursor(8, 230); tft.println("para reiniciar tanda.");
    return;
  }

  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 12);
  tft.println("PROBADOR DC");
  tft.drawFastHLine(5, 42, 162, ST77XX_WHITE);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 55);
  tft.println("TENSION");

  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(3);
  tft.setCursor(10, 82);
  tft.print(tensionMedida, 2);
  tft.println(" V");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 130);
  tft.println("CORRIENTE");

  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(10, 155);
  tft.print(corriente / 1000.0f, 3);
  tft.println(" A");

  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(1);
  tft.setCursor(10, 180);
  tft.print("LIMITE CONFIG: ");
  tft.print(corrienteMaximaConfigurada);
  tft.println(" mA");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 195);
  tft.println("ESTADO:");
  tft.setCursor(10, 220);

  if (estado == LISTO) {

    tft.setTextColor(ST77XX_WHITE);
    tft.println("LISTO");

  } else if (estado == EN_PRUEBA) {

    tft.setTextColor(ST77XX_YELLOW);
    tft.println("EN PRUEBA");

  } else if (estado == APROBADO) {

    tft.setTextColor(ST77XX_GREEN);
    tft.println("APROBADO");

  } else {

    tft.setTextColor(ST77XX_RED);
    tft.println("RECHAZADO");
  }

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(10, 265);
  tft.print("ETAPA: ");
  tft.print(etapa);
  tft.println("/3");

  tft.setCursor(10, 250);
  tft.print("PWM: ");
  tft.print(pwmRealPorcentaje);
  tft.print("% MAX: ");
  tft.print(pwmMaximoAlcanzado);
  tft.println("%");

  tft.setCursor(10, 285);

  if (cargaHabilitada) tft.println("CARGA HABILITADA");
  else tft.println("CARGA DESHABILITADA");
}

// ==================================================
// PAGINA WEB
// ==================================================

const char pagina[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Probador DC</title>

<style>
body{background:#18212b;color:white;font-family:Arial;text-align:center;margin:0;padding:15px}
.contenedor{max-width:450px;margin:auto}
.caja{background:#263544;border-radius:12px;padding:15px;margin:12px 0}
h2{color:#55c8ff}
.valor{font-size:32px;color:#55c8ff;font-weight:bold}
input{width:90%;margin:12px 0}
input[type=checkbox]{width:auto;margin:8px}
button{width:45%;padding:15px;margin:5px;border:0;border-radius:8px;font-size:16px;font-weight:bold}
.iniciar{background:#20b875;color:white}
.fin{background:#dc4545;color:white}
.reset{background:#e6a23c;color:#18212b}
.descargar{width:95%;background:#258bd5;color:white}
.pruebaGate{width:95%;background:#d78b27;color:white}
.estado{font-size:23px;font-weight:bold}
.contadores{display:flex;justify-content:space-around}
button:disabled{background:#555;color:#aaa;opacity:.55;cursor:not-allowed}
.mensaje{font-size:14px;color:#ffd166;min-height:20px}
.opcion{display:block;text-align:left;background:#1c2935;padding:12px;border-radius:8px;margin:9px 0}
.numero{width:85%;padding:10px;font-size:18px;box-sizing:border-box}
.testOnly{display:none}
body.testMode .normalOnly{display:none!important}
body.testMode .testOnly{display:block}
.pwmValor{font-size:30px;color:#ffd166;font-weight:bold}
</style>
</head>

<body>
<div class="contenedor">

<h2 class="normalOnly">PROBADOR DE FUENTES DC</h2>
<h2 class="testOnly">PROBADOR DC - MODO TEST</h2>

<div class="caja testOnly">
<h3>CONTROL DE LABORATORIO</h3>
<p>TENSION FICTICIA: <b id="tensionFicticiaTxt">12.0 V</b></p>
<input type="range" id="tensionFicticia" min="0" max="48" step="0.1" value="12"
onpointerdown="moviendoTensionTest=true" onpointerup="moviendoTensionTest=false"
oninput="tensionFicticiaTxt.innerText=Number(this.value).toFixed(1)+' V'; enviarTensionTest(this.value)">
<p>PWM DE ACTIVACION MOSFET</p>
<div class="pwmValor"><span id="pwmTestTxt">0</span> %</div>
<input type="range" id="pwmTest" min="0" max="100" step="1" value="0"
onpointerdown="moviendoPWMTest=true" onpointerup="moviendoPWMTest=false"
oninput="pwmTestTxt.innerText=this.value; enviarPWMTest(this.value)">
<p>Frecuencia: <b>5 kHz</b> | Resolucion: <b>8 bits</b></p>
<p id="estadoGateTest">Gate apagado (GPIO3 HIGH)</p>

</div>

<div class="caja normalOnly">
<p>TENSION MEDIDA</p>
<div class="valor" id="tension">-- V</div>
<p>Referencia: <span id="referencia">--</span></p>
<p>Objetivo configurado: <b id="corrienteConfigurada">100 mA</b></p>
<p>Corriente medida: <span id="corrienteMedida">-- A</span></p>
<p id="fuente">Comprobando entrada...</p>
<p id="seguridad">POTENCIA BLOQUEADA</p>
</div>

<div class="caja normalOnly">

<h3>MENU DE CONFIGURACION</h3>

<p>Ingresar PIN para modificar los ajustes</p>

<input type="password" id="pinConfig" inputmode="numeric"
placeholder="PIN de configuracion" style="width:70%">

<button class="descargar" onclick="ingresarConfig()">INGRESAR</button>

<div id="panelConfig" style="display:none">

<hr>
<h3>1. LIMITE MAXIMO DE CORRIENTE</h3>

<p>Rango configurable: 100 mA a 10.000 mA</p>

<input class="numero" type="number" id="corrienteMaxima"
min="100" max="10000" step="100" value="1000">



<hr>
<h3>2. CARGA DE POTENCIA</h3>

<label class="opcion">
<input type="checkbox" id="habilitarCarga">
Habilitar carga de potencia
</label>

<hr>
<h3>3. CONTROL PWM REAL</h3>

<label class="opcion">
<input type="checkbox" id="controlPWM">
Habilitar control PWM
</label>

<hr>
<h3>4. REGULACION DE CORRIENTE</h3>

<label class="opcion">
<input type="checkbox" id="regulacionCorriente">
Habilitar regulacion de corriente
</label>



<hr>
<h3>5. MEDICION DE CORRIENTE</h3>

<p>Shunt nominal: <b>0,1 ohm</b></p>
<p>Rango configurable: <b>100 mA a 10 A</b></p>

<label class="opcion">
<input type="checkbox" id="shunt">
Habilitar medicion mediante shunt
</label>

<hr>
<h3>6. SALIDA MOSFET</h3>

<label class="opcion">
<input type="checkbox" id="mosfet">
Habilitar salida MOSFET
</label>



<h3>PRUEBA MANUAL DEL GATE</h3>



<button class="pruebaGate" id="botonGate" onclick="probarGate()">
PROBAR GATE 500 ms
</button>

<p class="mensaje" id="mensajeGate">Gate en reposo.</p>

<hr>
<h3>7. SENSOR DE TEMPERATURA</h3>

<label class="opcion">
<input type="checkbox" id="temperatura">
Habilitar sensor NTC
</label>



<hr>
<h3>8. PRUEBA REAL DE CARGA</h3>

<label class="opcion">
<input type="checkbox" id="pruebaCarga">
Habilitar prueba real de carga
</label>

<hr>
<h3>9. PROTECCIONES DE POTENCIA</h3>

<label class="opcion">
<input type="checkbox" id="protecciones">
Habilitar protecciones de potencia
</label>

<hr>
<h3>10. MEDICION DE TENSION</h3>

<p>Tension nominal autorizada por el supervisor (3 a 48 V)</p>
<input type="number" class="numero" id="tensionAutorizada" min="3" max="48" step="0.1" value="0">
<p>La fuente solo se admite dentro de la tolerancia configurada.</p>
<p>Factor de calibracion</p>

<input type="number" class="numero" id="factorTension"
min="0.500" max="1.500" step="0.001" value="1.000">

<p>Tolerancia: <span id="toleranciaTxt">2.0</span> %</p>

<input type="range" id="tolerancia"
min="0.1" max="10" step="0.1" value="2"
oninput="toleranciaTxt.innerText=Number(this.value).toFixed(1)">

<hr>
<h3>11. CALIBRACION DE CORRIENTE</h3>

<label class="opcion">
<input type="checkbox" id="calibracionCorriente">
Habilitar calibracion de corriente
</label>

<p>Factor de corriente</p>

<input type="number" class="numero" id="factorCorriente"
min="0.500" max="1.500" step="0.001" value="1.000">

<p>Factor actual: <b id="factorActual">1.000</b></p>

<p>Corriente real medida con multimetro (mA)</p>

<input type="number" class="numero" id="corrienteReferencia"
min="100" max="10000" step="1" value="100"
placeholder="Ejemplo: 100">



<button class="descargar" onclick="calibrarCorriente()">
CALIBRAR CORRIENTE
</button>

<p class="mensaje" id="mensajeCalibracion"></p>

<hr>

<button class="descargar" onclick="guardarConfig()">
GUARDAR CONFIGURACION
</button>

<p id="mensajeConfig" class="mensaje"></p>

</div>
</div>

<div class="caja normalOnly">

<h3>CONFIGURACION DE PRUEBA</h3>

<p>Limite maximo configurado: <b id="limitePrueba">1000 mA</b></p>

<p>Corriente objetivo:</p>

<input class="numero" type="number" id="corrientePrueba"
min="100" max="1000" step="100" value="100"
onchange="guardarConsignaFisica()">

<p>Tiempo de carga:
<span id="tiempoTxt">10</span> segundos</p>

<input type="range" id="tiempo"
min="1" max="10" value="10"
oninput="tiempoTxt.innerText=this.value"
onchange="guardarConsignaFisica()">



</div>

<div class="caja normalOnly">

<p>ESTADO</p>

<div class="estado" id="estado">LISTO</div>

<p>Etapa: <span id="etapa">0 / 3</span></p>
<p>Tiempo restante: <span id="restante">0</span> s</p>

<button class="iniciar" id="botonIniciar" onclick="iniciar()">INICIAR</button>
<button class="fin" id="botonFin" onclick="fin()" disabled>FIN</button>
<button class="reset" id="botonReset" onclick="reset()" disabled>RESET</button>

<p class="mensaje" id="mensaje">
Lecturas reales; configurar la carga desde el menu.
</p>

</div>

<div class="caja normalOnly">

<h3>CONTADORES</h3>

<div class="contadores">
<div><h2 id="aprobadas">0</h2>APROBADAS</div>
<div><h2 id="rechazadas">0</h2>RECHAZADAS</div>
<div><h2 id="total">0</h2>TOTAL</div>
</div>

</div>

<div class="caja normalOnly">

<h3>HISTORIAL</h3>
<p>Descarga todas las pruebas registradas.</p>

<button class="descargar" id="botonDescargar"
onclick="descargarResultados()" disabled>
DESCARGAR RESULTADOS CSV
</button>

</div>
</div>

<script>

let limiteCorrienteActual=1000;
let timerPWMTest=null;
let timerTensionTest=null;
let moviendoPWMTest=false;
let moviendoTensionTest=false;

function enviarPWMTest(valor){
  clearTimeout(timerPWMTest);
  timerPWMTest=setTimeout(async()=>{
    try{await fetch("/test/pwm?valor="+Number(valor),{method:"POST"});}catch(e){}
  },80);
}

function enviarTensionTest(valor){
  clearTimeout(timerTensionTest);
  timerTensionTest=setTimeout(async()=>{
    try{await fetch("/test/tension?valor="+encodeURIComponent(Number(valor)),{method:"POST"});}catch(e){}
  },80);
}

function actualizarLimiteVisual(){

  let limite=Number(document.getElementById("corrienteMaxima").value);

  if(!Number.isFinite(limite))return;

  limite=Math.max(100,Math.min(10000,limite));

  let selector=document.getElementById("corrientePrueba");

  selector.max=limite;

  if(Number(selector.value)>limite){
    selector.value=limite;
  }

  document.getElementById("limitePrueba").innerText=
    limite+" mA";
}

document.getElementById("corrienteMaxima").addEventListener(
  "input",actualizarLimiteVisual
);

async function ingresarConfig(){

  let pin=document.getElementById("pinConfig").value;

  try{

    let r=await fetch("/config/login?pin="+encodeURIComponent(pin));
    let d=await r.json();

    if(!r.ok){
      alert(d.mensaje||"PIN incorrecto");
      return;
    }

    document.getElementById("panelConfig").style.display="block";
    document.getElementById("pinConfig").value="";

    document.getElementById("factorTension").value=d.factorTension.toFixed(3);
    document.getElementById("factorCorriente").value=d.factorCorriente.toFixed(3);

    document.getElementById("factorActual").innerText=
      d.factorCorriente.toFixed(3);

    document.getElementById("tolerancia").value=d.tolerancia;
    document.getElementById("tensionAutorizada").value=d.tensionAutorizada;

    document.getElementById("toleranciaTxt").innerText=
      Number(d.tolerancia).toFixed(1);

    document.getElementById("corrienteMaxima").value=
      d.corrienteMaxima;

    actualizarLimiteVisual();

    document.getElementById("habilitarCarga").checked=d.cargaHabilitada;
    document.getElementById("controlPWM").checked=d.controlPWM;
    document.getElementById("regulacionCorriente").checked=d.regulacionCorriente;
    document.getElementById("shunt").checked=d.shunt;
    document.getElementById("mosfet").checked=d.mosfet;
    document.getElementById("temperatura").checked=d.temperatura;
    document.getElementById("pruebaCarga").checked=d.pruebaCarga;
    document.getElementById("protecciones").checked=d.protecciones;
    document.getElementById("calibracionCorriente").checked=d.calibracionCorriente;

  }catch(e){

    alert("Error de comunicacion.");
  }
}

async function guardarConfig(){

  let limite=Number(document.getElementById("corrienteMaxima").value);
  let tensionAut=Number(document.getElementById("tensionAutorizada").value);
  if(!Number.isFinite(tensionAut)||tensionAut<3||tensionAut>48){alert("La tension autorizada debe estar entre 3 y 48 V.");return;}

  if(!Number.isInteger(limite)||limite<100||limite>10000){

    alert("El limite debe estar entre 100 y 10000 mA.");
    return;
  }

  let parametros = new URLSearchParams({

    factor: document.getElementById("factorTension").value,

    factorCorriente:
      document.getElementById("factorCorriente").value,

    tolerancia:
      document.getElementById("tolerancia").value,

    tensionAutorizada:
      document.getElementById("tensionAutorizada").value,

    corrienteMaxima: String(limite),

    carga:
      document.getElementById("habilitarCarga").checked ? "1" : "0",

    pwm:
      document.getElementById("controlPWM").checked ? "1" : "0",

    regulacion:
      document.getElementById("regulacionCorriente").checked ? "1" : "0",

    shunt:
      document.getElementById("shunt").checked ? "1" : "0",

    mosfet:
      document.getElementById("mosfet").checked ? "1" : "0",

    temperatura:
      document.getElementById("temperatura").checked ? "1" : "0",

    pruebaCarga:
      document.getElementById("pruebaCarga").checked ? "1" : "0",

    protecciones:
      document.getElementById("protecciones").checked ? "1" : "0",

    calibracionCorriente:
      document.getElementById("calibracionCorriente").checked ? "1" : "0"
  });

  try{

    let r=await fetch("/config/save?"+parametros.toString(),
                      {method:"POST"});

    let d=await r.json();

    document.getElementById("mensajeConfig").innerText=d.mensaje;

    if(!r.ok)alert(d.mensaje);

    if(r.ok){

      limiteCorrienteActual=limite;
      actualizarLimiteVisual();

      let selector=document.getElementById("corrientePrueba");
      selector.max=limite;

      if(Number(selector.value)>limite){
        selector.value=limite;
      }
    }

    actualizar();

  }catch(e){

    document.getElementById("mensajeConfig").innerText=
      "Error al guardar.";
  }
}

async function calibrarCorriente(){

  let referencia =
    Number(document.getElementById("corrienteReferencia").value);

  if(!Number.isFinite(referencia) || referencia<100 || referencia>10000){

    alert("Ingresar una corriente de referencia entre 100 y 10000 mA.");
    return;
  }

  if(!document.getElementById("calibracionCorriente").checked){

    alert("Primero habilita la calibracion de corriente.");
    return;
  }

  if(!document.getElementById("shunt").checked){

    alert("Primero habilita la medicion mediante shunt.");
    return;
  }

  try{

    let r=await fetch(
      "/config/calibrar?referencia="+referencia,
      {method:"POST"}
    );

    let d=await r.json();

    document.getElementById("mensajeCalibracion").innerText=d.mensaje;

    if(!r.ok){

      alert(d.mensaje);
      return;
    }

    document.getElementById("factorCorriente").value=
      Number(d.factor).toFixed(3);

    document.getElementById("factorActual").innerText=
      Number(d.factor).toFixed(3);

    actualizar();

  }catch(e){

    alert("Error de comunicacion durante la calibracion.");
  }
}

async function probarGate(){

  if(!confirm(
    "CONFIRMAR PRUEBA DEL GATE:\n\n"+
    "La entrada debe medir entre 3 y 48 V.\n"+
    "El Gate se activara durante 500 ms.\n"+
    "¿Continuar?"
  ))return;

  let boton=document.getElementById("botonGate");
  boton.disabled=true;

  try{

    let r=await fetch("/gate/test",{method:"POST"});
    let d=await r.json();

    document.getElementById("mensajeGate").innerText=d.mensaje;

    if(!r.ok){
      alert(d.mensaje);
    }

    actualizar();

  }catch(e){

    document.getElementById("mensajeGate").innerText=
      "Error de comunicacion.";

  }finally{

    setTimeout(()=>{boton.disabled=false;},700);
  }
}

async function guardarConsignaFisica(){
  let t=parseInt(document.getElementById("tiempo").value,10);
  let c=parseInt(document.getElementById("corrientePrueba").value,10);
  if(!Number.isInteger(c)||c<100||c>limiteCorrienteActual||!Number.isFinite(t)||t<1||t>10) return;
  try{
    await fetch("/consigna?corriente="+c+"&tiempo="+t,{method:"POST"});
  }catch(e){}
}

async function iniciar(){

  let t=parseInt(document.getElementById("tiempo").value,10);
  let c=parseInt(document.getElementById("corrientePrueba").value,10);

  if(!Number.isFinite(t)||t<1||t>10){

    alert("Tiempo permitido: 1 a 10 segundos");
    return;
  }

  if(!Number.isInteger(c)||c<100||c>limiteCorrienteActual){

    alert("La corriente debe estar entre 100 mA y el limite configurado.");
    return;
  }

  try{

    let r=await fetch("/start?corriente="+c+"&tiempo="+t);

    if(!r.ok){

      let m=await r.text();
      alert(m);
    }

    actualizar();

  }catch(e){

    alert("Error de comunicacion con el probador.");
  }
}

async function fin(){

  if(document.getElementById("botonFin").disabled)return;

  if(!confirm("FIN cerrara la tanda. Luego deberas descargar el CSV antes de RESET. ¿Continuar?"))return;

  try{

    let r=await fetch("/fin");

    if(!r.ok){

      alert("FIN todavia no esta habilitado.");
      return;
    }

    actualizar();

  }catch(e){

    alert("Error de comunicacion.");
  }
}

async function descargarResultados(){

  let boton=document.getElementById("botonDescargar");

  if(boton.disabled)return;

  boton.disabled=true;

  try{

    let respuesta=await fetch("/download");

    if(!respuesta.ok)throw new Error("No se pudo obtener el CSV");

    let archivo=await respuesta.blob();

    let url=URL.createObjectURL(archivo);

    let enlace=document.createElement("a");

    enlace.href=url;
    enlace.download="resultados_probador.csv";

    document.body.appendChild(enlace);
    enlace.click();
    enlace.remove();

    setTimeout(()=>URL.revokeObjectURL(url),2000);

    let confirmacion=await fetch("/downloaded",{method:"POST"});

    if(!confirmacion.ok)throw new Error("No se pudo confirmar la descarga");

    document.getElementById("mensaje").innerText=
      "CSV recibido. RESET habilitado.";

    actualizar();

  }catch(e){

    document.getElementById("mensaje").innerText=
      "Error en la descarga. RESET sigue bloqueado.";

    alert("No se pudo completar la descarga. Intenta nuevamente.");

  }finally{

    boton.disabled=false;
  }
}

async function reset(){

  if(document.getElementById("botonReset").disabled)return;

  if(!confirm("RESET pondra los contadores en cero. El historial CSV se conservara. ¿Continuar?"))return;

  try{

    let r=await fetch("/reset");

    if(!r.ok){

      alert("RESET bloqueado. Primero debes descargar los resultados.");
      return;
    }

    actualizar();

  }catch(e){

    alert("Error de comunicacion.");
  }
}

async function actualizar(){

  try{

    let r=await fetch("/data");
    let d=await r.json();

    document.body.classList.toggle("testMode",d.modoTest);

    if(d.modoTest){
      if(!moviendoTensionTest){
        document.getElementById("tensionFicticia").value=d.tensionFicticia;
        document.getElementById("tensionFicticiaTxt").innerText=Number(d.tensionFicticia).toFixed(1)+" V";
      }
      if(!moviendoPWMTest){
        document.getElementById("pwmTest").value=d.pwmTest;
        document.getElementById("pwmTestTxt").innerText=d.pwmTest;
      }
      document.getElementById("estadoGateTest").innerText=
        d.pwmTest===0 ? "Gate apagado (GPIO3 HIGH)" : "PWM activo (GPIO3 invertido)";
      return;
    }

    document.getElementById("tension").innerText=
      d.tension.toFixed(2)+" V";

    document.getElementById("referencia").innerText=
      d.referencia>0 ? d.referencia.toFixed(2)+" V" : "--";

    if(document.activeElement.id!=="corrientePrueba"){
      document.getElementById("corrientePrueba").value=d.corrienteConfigurada;
    }
    if(document.activeElement.id!=="tiempo"){
      document.getElementById("tiempo").value=d.tiempoConfigurado;
      document.getElementById("tiempoTxt").innerText=d.tiempoConfigurado;
    }

    document.getElementById("corrienteMedida").innerText=
      d.corrienteMedida.toFixed(3)+" A";

    document.getElementById("corrienteConfigurada").innerText=
      d.corrienteConfigurada+" mA";

    document.getElementById("limitePrueba").innerText=
      d.corrienteMaxima+" mA";
    document.getElementById("corrientePrueba").max=d.corrienteMaxima;
    if(Number(document.getElementById("corrientePrueba").value)>d.corrienteMaxima) document.getElementById("corrientePrueba").value=d.corrienteMaxima;

    limiteCorrienteActual=d.corrienteMaxima;

    document.getElementById("fuente").innerText=
      d.fuenteDetectada ? "FUENTE DETECTADA" : "SIN FUENTE";

    document.getElementById("seguridad").innerText=
      d.cargaHabilitada ? "CARGA HABILITADA" : "CARGA DESHABILITADA";

    document.getElementById("estado").innerText=d.estado;
    document.getElementById("etapa").innerText=d.etapa+" / 3";
    document.getElementById("restante").innerText=d.restante;

    document.getElementById("aprobadas").innerText=d.aprobadas;
    document.getElementById("rechazadas").innerText=d.rechazadas;

    document.getElementById("total").innerText=
      d.aprobadas+d.rechazadas;

    document.getElementById("botonIniciar").disabled=!d.puedeIniciar;
    document.getElementById("botonFin").disabled=!d.finHabilitado;
    document.getElementById("botonDescargar").disabled=!d.descargaHabilitada;
    document.getElementById("botonReset").disabled=!d.resetHabilitado;

    if(d.pruebaGateActiva){

      document.getElementById("mensajeGate").innerText=
        "GATE ACTIVO: esperando apagado automatico.";

    }else if(document.getElementById("mensajeGate").innerText.includes("ACTIVO")){

      document.getElementById("mensajeGate").innerText=
        "Gate en reposo. GPIO3 HIGH.";

    }

    if(!d.cargaHabilitada){

      document.getElementById("mensaje").innerText=
        "Carga deshabilitada desde configuracion.";

    }else if(d.pruebaActiva){

      document.getElementById("mensaje").innerText=
        d.buscandoCorriente ? "Buscando corriente objetivo (maximo 10 s)." : "Mantenimiento de corriente en curso.";

    }else if(d.tandaFinalizada&&!d.descargaConfirmada){

      document.getElementById("mensaje").innerText=
        "DESCARGAR RESULTADOS ANTES DE CONTINUAR.";

    }else if(d.resetHabilitado){

      document.getElementById("mensaje").innerText=
        "Descarga confirmada. RESET habilitado.";

    }else if(d.finHabilitado){

      document.getElementById("mensaje").innerText=
        "Prueba terminada. Puedes iniciar otra o pulsar FIN.";
    }

  }catch(e){}
}

setInterval(actualizar,700);
actualizar();

</script>
</body>
</html>
)HTML";

// ==================================================
// CONFIGURAR SERVIDOR WEB
// ==================================================

void configurarWeb() {

  server.on("/", HTTP_GET, []() {

    server.send(200, "text/html", pagina);
  });

  // ----------------------------------------------
  // INGRESO A CONFIGURACION
  // ----------------------------------------------

  server.on("/config/login", HTTP_GET, []() {

    if (!server.hasArg("pin") || server.arg("pin") != "1813") {

      configuracionAutorizada = false;

      server.send(401, "application/json",
                  "{\"mensaje\":\"PIN incorrecto\"}");

      return;
    }

    configuracionAutorizada = true;

    String json = "{\"factorTension\":" +
                  String(factorTension, 3);

    json += ",\"factorCorriente\":" +
            String(factorCorriente, 3);

    json += ",\"corrienteMaxima\":" +
            String(corrienteMaximaConfigurada);

    json += ",\"tolerancia\":" +
            String(toleranciaPorcentaje, 1);
    json += ",\"tensionAutorizada\":" + String(tensionAutorizada, 2);

    json += ",\"cargaHabilitada\":" +
            String(cargaHabilitada ? "true" : "false");

    json += ",\"controlPWM\":" +
            String(controlPWMHabilitado ? "true" : "false");

    json += ",\"regulacionCorriente\":" +
            String(regulacionCorrienteHabilitada ? "true" : "false");

    json += ",\"shunt\":" +
            String(shuntHabilitado ? "true" : "false");

    json += ",\"mosfet\":" +
            String(salidaMOSFETHabilitada ? "true" : "false");

    json += ",\"temperatura\":" +
            String(temperaturaHabilitada ? "true" : "false");

    json += ",\"pruebaCarga\":" +
            String(pruebaCargaHabilitada ? "true" : "false");

    json += ",\"protecciones\":" +
            String(proteccionesPotenciaHabilitadas ? "true" : "false");

    json += ",\"calibracionCorriente\":" +
            String(calibracionCorrienteHabilitada ? "true" : "false");

    json += "}";

    server.send(200, "application/json", json);
  });

  // ----------------------------------------------
  // GUARDAR CONFIGURACION
  // ----------------------------------------------

  server.on("/config/save", HTTP_POST, []() {

    if (!configuracionAutorizada) {
      server.send(401, "application/json",
                  "{\"mensaje\":\"Ingresar PIN primero\"}");
      return;
    }

    if (pruebaActiva || pruebaGateActiva || modoTest) {
      server.send(423, "application/json",
                  "{\"mensaje\":\"Configuracion bloqueada durante una prueba o modo TEST\"}");
      return;
    }

    if (!server.hasArg("factor") ||
        !server.hasArg("factorCorriente") ||
        !server.hasArg("corrienteMaxima") ||
        !server.hasArg("tolerancia") ||
        !server.hasArg("tensionAutorizada") ||
        !server.hasArg("carga") ||
        !server.hasArg("pwm") ||
        !server.hasArg("regulacion") ||
        !server.hasArg("shunt") ||
        !server.hasArg("mosfet") ||
        !server.hasArg("temperatura") ||
        !server.hasArg("pruebaCarga") ||
        !server.hasArg("protecciones") ||
        !server.hasArg("calibracionCorriente")) {

      server.send(400, "application/json",
                  "{\"mensaje\":\"Faltan parametros\"}");

      return;
    }

    float nuevoFactor = server.arg("factor").toFloat();

    float nuevoFactorCorriente =
      server.arg("factorCorriente").toFloat();

    float nuevaTolerancia =
      server.arg("tolerancia").toFloat();

    int nuevoLimiteCorriente =
      server.arg("corrienteMaxima").toInt();
    float nuevaTensionAutorizada = server.arg("tensionAutorizada").toFloat();

    if (nuevoFactor < 0.500f || nuevoFactor > 1.500f ||
        nuevoFactorCorriente < 0.500f ||
        nuevoFactorCorriente > 1.500f ||
        nuevaTolerancia < 0.1f ||
        nuevaTolerancia > 10.0f ||
        nuevaTensionAutorizada < 3.0f || nuevaTensionAutorizada > 48.0f ||
        nuevoLimiteCorriente < CORRIENTE_MINIMA_CONFIGURABLE ||
        nuevoLimiteCorriente > CORRIENTE_MAXIMA_CONFIGURABLE) {

      server.send(400, "application/json",
                  "{\"mensaje\":\"Valor fuera de rango\"}");

      return;
    }

    cargaHabilitada = (server.arg("carga") == "1");

    controlPWMHabilitado = (server.arg("pwm") == "1");

    regulacionCorrienteHabilitada =
      (server.arg("regulacion") == "1");

    shuntHabilitado = (server.arg("shunt") == "1");

    salidaMOSFETHabilitada =
      (server.arg("mosfet") == "1");

    temperaturaHabilitada =
      (server.arg("temperatura") == "1");

    pruebaCargaHabilitada =
      (server.arg("pruebaCarga") == "1");

    proteccionesPotenciaHabilitadas =
      (server.arg("protecciones") == "1");

    calibracionCorrienteHabilitada =
      (server.arg("calibracionCorriente") == "1");

    factorTension = nuevoFactor;
    factorCorriente = nuevoFactorCorriente;
    toleranciaPorcentaje = nuevaTolerancia;
    tensionAutorizada = nuevaTensionAutorizada;

    corrienteMaximaConfigurada = nuevoLimiteCorriente;

    establecerPWM(0);

    preferencias.putFloat("factorV", factorTension);
    preferencias.putFloat("factorI", factorCorriente);
    preferencias.putFloat("tolerancia", toleranciaPorcentaje);
    preferencias.putFloat("tensionAut", tensionAutorizada);

    preferencias.putInt("maxCorriente", corrienteMaximaConfigurada);
    if (ultimaCorrienteSolicitada > corrienteMaximaConfigurada) {
      ultimaCorrienteSolicitada = corrienteMaximaConfigurada;
      corriente = ultimaCorrienteSolicitada;
      preferencias.putInt("ultCorriente", ultimaCorrienteSolicitada);
    }

    preferencias.putBool("cargaOn", cargaHabilitada);
    preferencias.putBool("controlPWM", controlPWMHabilitado);
    preferencias.putBool("regCorriente", regulacionCorrienteHabilitada);
    preferencias.putBool("shuntOn", shuntHabilitado);
    preferencias.putBool("mosfetOn", salidaMOSFETHabilitada);
    preferencias.putBool("ntcOn", temperaturaHabilitada);
    preferencias.putBool("pruebaCarga", pruebaCargaHabilitada);
    preferencias.putBool("protecciones", proteccionesPotenciaHabilitadas);
    preferencias.putBool("calibCorriente", calibracionCorrienteHabilitada);

    server.send(200, "application/json",
                "{\"mensaje\":\"Configuracion guardada correctamente\"}");
  });

  // ----------------------------------------------
  // CALIBRACION DE CORRIENTE
  // ----------------------------------------------

  server.on("/config/calibrar", HTTP_POST, []() {

    if (pruebaActiva || pruebaGateActiva || modoTest) {
      server.send(423, "application/json",
                  "{\"mensaje\":\"Calibracion bloqueada durante prueba o modo TEST\"}");
      return;
    }

    if (!configuracionAutorizada) {

      server.send(401, "application/json",
                  "{\"mensaje\":\"Ingresar PIN primero\"}");

      return;
    }

    if (!server.hasArg("referencia")) {

      server.send(400, "application/json",
                  "{\"mensaje\":\"Falta corriente de referencia\"}");

      return;
    }

    float referenciaA =
      server.arg("referencia").toFloat() / 1000.0f;

    if (!calibrarCorriente(referenciaA)) {

      server.send(400, "application/json",
                  "{\"mensaje\":\"Calibracion no realizada. Verificar rango, habilitaciones, lectura y factor.\"}");

      return;
    }

    String json = "{\"mensaje\":\"Calibracion completada y guardada\",";
    json += "\"factor\":";
    json += String(factorCorriente, 4);
    json += "}";

    server.send(200, "application/json", json);
  });

  // ----------------------------------------------
  // PRUEBA MANUAL DEL GATE
  // ----------------------------------------------

  server.on("/gate/test", HTTP_POST, []() {

    if (!configuracionAutorizada) {

      server.send(401, "application/json",
                  "{\"mensaje\":\"Ingresar PIN primero\"}");

      return;
    }

    if (iniciarPruebaGate()) {

      server.send(200, "application/json",
                  "{\"mensaje\":\"Prueba iniciada: MOSFET activado al 100% durante 500 ms (GPIO3 LOW)\"}");

    } else {

      server.send(423, "application/json",
                  "{\"mensaje\":\"Prueba bloqueada. Verificar que no haya otra prueba activa y que la entrada mida entre 3 y 48 V.\"}");
    }
  });

  // ----------------------------------------------
  // CONTROL PWM Y TENSION FICTICIA - MODO TEST
  // ----------------------------------------------

  server.on("/test/pwm", HTTP_POST, []() {

    if (!modoTest || !server.hasArg("valor")) {
      server.send(423, "application/json",
                  "{\"mensaje\":\"Modo TEST no activo o falta valor\"}");
      return;
    }

    int valor = constrain(server.arg("valor").toInt(), 0, 100);
    pwmTestPorcentaje = valor;
    establecerPWM(pwmTestPorcentaje);

    server.send(200, "application/json",
                "{\"mensaje\":\"PWM actualizado\"}");
  });

  server.on("/test/tension", HTTP_POST, []() {

    if (!modoTest || !server.hasArg("valor")) {
      server.send(423, "application/json",
                  "{\"mensaje\":\"Modo TEST no activo o falta valor\"}");
      return;
    }

    float valor = server.arg("valor").toFloat();

    if (valor < 0.0f || valor > 48.0f) {
      server.send(400, "application/json",
                  "{\"mensaje\":\"Tension ficticia fuera de rango\"}");
      return;
    }

    tensionFicticia = valor;
    tensionMedida = tensionFicticia;

    server.send(200, "application/json",
                "{\"mensaje\":\"Tension ficticia actualizada\"}");
  });

  // ----------------------------------------------
  // GUARDAR CONSIGNA PARA EL PULSADOR GPIO4
  // ----------------------------------------------

  server.on("/consigna", HTTP_POST, []() {
    if (pruebaActiva || tandaFinalizada || modoTest ||
        !server.hasArg("corriente") || !server.hasArg("tiempo")) {
      server.send(423, "text/plain", "CONSIGNA BLOQUEADA");
      return;
    }
    int c = server.arg("corriente").toInt();
    int t = server.arg("tiempo").toInt();
    if (c < CORRIENTE_MINIMA_CONFIGURABLE ||
        c > corrienteMaximaConfigurada || t < 1 || t > 10) {
      server.send(400, "text/plain", "CONSIGNA FUERA DE RANGO");
      return;
    }
    ultimaCorrienteSolicitada = c;
    ultimoTiempoSolicitado = t;
    corriente = c;
    tiempoPrueba = t;
    preferencias.putInt("ultCorriente", c);
    preferencias.putInt("ultTiempo", t);
    server.send(200, "text/plain", "CONSIGNA GUARDADA");
  });

  // ----------------------------------------------
  // INICIAR PRUEBA
  // ----------------------------------------------

  server.on("/start", HTTP_GET, []() {

    if (!server.hasArg("corriente") || !server.hasArg("tiempo")) {
      server.send(400, "text/plain", "FALTAN PARAMETROS");
      return;
    }
    int corrienteSolicitada = server.arg("corriente").toInt();
    int tiempoSolicitado = server.arg("tiempo").toInt();

    if (iniciarPrueba(corrienteSolicitada, tiempoSolicitado)) {

      server.send(200, "text/plain", "OK");

    } else {

      server.send(423, "text/plain",
        "INICIO BLOQUEADO: verificar habilitaciones, corriente seleccionada o fuente fuera de rango.");
    }
  });

  // ----------------------------------------------
  // FINALIZAR TANDA
  // ----------------------------------------------

  server.on("/fin", HTTP_GET, []() {

    if (modoTest || !tandaConResultados || pruebaActiva || tandaFinalizada) {

      server.send(403, "text/plain", "FIN BLOQUEADO");
      return;
    }

    finalizarTanda();

    server.send(200, "text/plain", "OK");
  });

  // ----------------------------------------------
  // RESET
  // ----------------------------------------------

  server.on("/reset", HTTP_GET, []() {

    if (modoTest || !tandaFinalizada || !archivoCSVServido ||
        !descargaConfirmada || pruebaActiva) {

      server.send(403, "text/plain",
                  "RESET BLOQUEADO: DESCARGA PENDIENTE");

      return;
    }

    resetearTanda();

    server.send(200, "text/plain", "OK");
  });

  // ----------------------------------------------
  // DATOS PARA LA WEB
  // ----------------------------------------------

  server.on("/data", HTTP_GET, []() {

    int restante = 0;

    if (pruebaActiva && etapa == 2) {
      unsigned long transcurrido = (millis() - tiempoEtapa) / 1000UL;
      restante = buscandoCorriente ? max(0, 10 - (int)transcurrido) : max(0, tiempoPrueba - (int)transcurrido);
    }

    bool finHabilitado =
      tandaConResultados && !tandaFinalizada && !pruebaActiva;

    bool puedeIniciar =
      !pruebaActiva &&
      !pruebaGateActiva &&
      !tandaFinalizada &&
      cargaHabilitada &&
      pruebaCargaHabilitada &&
      shuntHabilitado &&
      salidaMOSFETHabilitada &&
      proteccionesPotenciaHabilitadas &&
      controlPWMHabilitado &&
      regulacionCorrienteHabilitada &&
      fuenteDetectada && tensionAutorizada >= 3.0f && tensionAutorizada <= 48.0f &&
      tensionMedida >= tensionAutorizada * (1.0f - toleranciaPorcentaje / 100.0f) &&
      tensionMedida <= tensionAutorizada * (1.0f + toleranciaPorcentaje / 100.0f);

    bool descargaHabilitada =
      tandaFinalizada && !pruebaActiva;

    bool resetHabilitado =
      tandaFinalizada &&
      archivoCSVServido &&
      descargaConfirmada &&
      !pruebaActiva;

    String json = "{";

    json += "\"modoTest\":" + String(modoTest ? "true" : "false") + ",";
    json += "\"pwmTest\":" + String(pwmTestPorcentaje) + ",";
    json += "\"pwmReal\":" + String(pwmRealPorcentaje) + ",";
    json += "\"pwmMaximo\":" + String(pwmMaximoAlcanzado) + ",";
    json += "\"tensionFicticia\":" + String(tensionFicticia, 1) + ",";
    json += "\"estado\":\"" + nombreEstado() + "\",";
    json += "\"etapa\":" + String(etapa) + ",";
    json += "\"tension\":" + String(tensionMedida, 3) + ",";
    json += "\"corrienteConfigurada\":" + String(corriente) + ",";
    json += "\"tiempoConfigurado\":" + String(tiempoPrueba) + ",";
    json += "\"corrienteMaxima\":" + String(corrienteMaximaConfigurada) + ",";
    json += "\"tensionAutorizada\":" + String(tensionAutorizada, 2) + ",";
    json += "\"buscandoCorriente\":" + String(buscandoCorriente ? "true" : "false") + ",";
    json += "\"corrienteMedida\":" + String(corrienteMedida, 4) + ",";
    json += "\"referencia\":" + String(tensionReferencia > 0 ? tensionReferencia : tensionAutorizada, 3) + ",";
    json += "\"restante\":" + String(restante) + ",";
    json += "\"aprobadas\":" + String(aprobadas) + ",";
    json += "\"rechazadas\":" + String(rechazadas) + ",";
    json += "\"fuenteDetectada\":" + String(fuenteDetectada ? "true" : "false") + ",";
    json += "\"cargaHabilitada\":" + String(cargaHabilitada ? "true" : "false") + ",";
    json += "\"pruebaActiva\":" + String(pruebaActiva ? "true" : "false") + ",";
    json += "\"pruebaGateActiva\":" + String(pruebaGateActiva ? "true" : "false") + ",";
    json += "\"tandaFinalizada\":" + String(tandaFinalizada ? "true" : "false") + ",";
    json += "\"descargaConfirmada\":" + String(descargaConfirmada ? "true" : "false") + ",";
    json += "\"finHabilitado\":" + String(finHabilitado ? "true" : "false") + ",";
    json += "\"puedeIniciar\":" + String(puedeIniciar ? "true" : "false") + ",";
    json += "\"descargaHabilitada\":" + String(descargaHabilitada ? "true" : "false") + ",";
    json += "\"resetHabilitado\":" + String(resetHabilitado ? "true" : "false");

    json += "}";

    server.send(200, "application/json", json);
  });

  // ----------------------------------------------
  // DESCARGA CSV
  // ----------------------------------------------

  server.on("/download", HTTP_GET, []() {

    if (!tandaFinalizada || pruebaActiva) {

      server.send(403, "text/plain", "DESCARGA NO HABILITADA");
      return;
    }

    archivoCSVServido = true;
    descargaConfirmada = false;
    guardarEstadoTanda();

    server.sendHeader("Content-Disposition",
                      "attachment; filename=resultados_probador.csv");

    server.sendHeader("Cache-Control", "no-store");

    server.send(200, "text/csv; charset=utf-8",
                String("\xEF\xBB\xBF") + historialCSV);
  });

  // ----------------------------------------------
  // CONFIRMAR DESCARGA
  // ----------------------------------------------

  server.on("/downloaded", HTTP_POST, []() {

    if (tandaFinalizada && archivoCSVServido && !pruebaActiva) {

      descargaConfirmada = true;
      guardarEstadoTanda();

      server.send(200, "text/plain", "DESCARGA CONFIRMADA");

    } else {

      server.send(409, "text/plain", "NO HAY DESCARGA PENDIENTE");
    }
  });

  server.begin();
}

// ==================================================
// SETUP
// ==================================================

void setup() {

  Serial.begin(115200);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  pinMode(PIN_GATE, OUTPUT);
  analogWriteResolution(PIN_GATE, RESOLUCION_PWM);
  analogWriteFrequency(PIN_GATE, FRECUENCIA_PWM);
  establecerPWM(0);

  pinMode(PIN_BOTON_TEST, INPUT_PULLUP);
  pinMode(PIN_BOTON_TANDA, INPUT_PULLUP);
  botonTandaAnterior = digitalRead(PIN_BOTON_TANDA);
  estadoEstableBotonTanda = botonTandaAnterior;

  preferencias.begin("probador", false);

  // CARGA DE CONFIGURACION GUARDADA

  cargaHabilitada =
    preferencias.getBool("cargaOn", false);

  factorTension =
    preferencias.getFloat("factorV", 1.000f);

  factorCorriente =
    preferencias.getFloat("factorI", 1.000f);

  toleranciaPorcentaje = preferencias.getFloat("tolerancia", 2.0f);
  tensionAutorizada = preferencias.getFloat("tensionAut", 0.0f);
  ultimaCorrienteSolicitada = preferencias.getInt("ultCorriente", 100);
  ultimoTiempoSolicitado = preferencias.getInt("ultTiempo", 10);
  corriente = ultimaCorrienteSolicitada;
  tiempoPrueba = ultimoTiempoSolicitado;

  corrienteMaximaConfigurada =
    preferencias.getInt("maxCorriente", 1000);

  corrienteMaximaConfigurada = constrain(corrienteMaximaConfigurada,
              CORRIENTE_MINIMA_CONFIGURABLE, CORRIENTE_MAXIMA_CONFIGURABLE);
  ultimaCorrienteSolicitada = constrain(ultimaCorrienteSolicitada, CORRIENTE_MINIMA_CONFIGURABLE, corrienteMaximaConfigurada);
  ultimoTiempoSolicitado = constrain(ultimoTiempoSolicitado, 1, 10);
  corriente = ultimaCorrienteSolicitada;
  tiempoPrueba = ultimoTiempoSolicitado;
  numeroPrueba = preferencias.getInt("numPrueba", 0);
  registrosTanda = preferencias.getInt("regTanda", 0);
  registrosTotales = preferencias.getInt("regTotal", registrosTanda);
  aprobadas = preferencias.getInt("aprobadas", 0);
  rechazadas = preferencias.getInt("rechazadas", 0);
  tandaConResultados = preferencias.getBool("tandaHay", false);
  tandaFinalizada = preferencias.getBool("tandaFin", false);
  archivoCSVServido = preferencias.getBool("csvServido", false);
  descargaConfirmada = preferencias.getBool("csvConf", false);
  for (int i = 1; i <= registrosTotales; i++) {
    String clave = "r" + String(i);
    historialCSV += preferencias.getString(clave.c_str(), "");
  }

  controlPWMHabilitado =
    preferencias.getBool("controlPWM", false);

  regulacionCorrienteHabilitada =
    preferencias.getBool("regCorriente", false);

  shuntHabilitado =
    preferencias.getBool("shuntOn", false);

  salidaMOSFETHabilitada =
    preferencias.getBool("mosfetOn", false);

  temperaturaHabilitada =
    preferencias.getBool("ntcOn", false);

  pruebaCargaHabilitada =
    preferencias.getBool("pruebaCarga", false);

  proteccionesPotenciaHabilitadas =
    preferencias.getBool("protecciones", false);

  calibracionCorrienteHabilitada =
    preferencias.getBool("calibCorriente", false);

  analogReadResolution(12);

  analogSetPinAttenuation(PIN_TENSION, ADC_11db);
  analogSetPinAttenuation(PIN_CORRIENTE, ADC_11db);
  analogSetPinAttenuation(PIN_NTC, ADC_11db);

  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

  tft.init(172, 320);
  tft.setRotation(0);

  detectarFuenteReal();
  mostrarPantalla();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  configurarWeb();

  Serial.println();
  Serial.println("PROBADOR DC - CONFIGURACION MODULAR");

  Serial.print("Limite maximo de corriente configurado: ");
  Serial.print(corrienteMaximaConfigurada);
  Serial.println(" mA");

  Serial.println(cargaHabilitada ?
    "Carga habilitada." : "Carga deshabilitada.");

  Serial.println(controlPWMHabilitado ?
    "Control PWM habilitado en configuracion." :
    "Control PWM pendiente.");

  Serial.println(regulacionCorrienteHabilitada ?
    "Regulacion de corriente habilitada en configuracion." :
    "Regulacion de corriente deshabilitada.");

  Serial.println(shuntHabilitado ?
    "Medicion shunt habilitada." :
    "Medicion shunt deshabilitada.");

  Serial.println(salidaMOSFETHabilitada ?
    "Salida MOSFET habilitada en configuracion." :
    "Salida MOSFET deshabilitada.");

  Serial.println(temperaturaHabilitada ?
    "NTC habilitado en configuracion." :
    "NTC deshabilitado.");

  Serial.println(pruebaCargaHabilitada ?
    "Prueba de carga habilitada en configuracion." :
    "Prueba de carga deshabilitada.");

  Serial.println(proteccionesPotenciaHabilitadas ?
    "Protecciones habilitadas en configuracion." :
    "Protecciones deshabilitadas.");

  Serial.println(calibracionCorrienteHabilitada ?
    "Calibracion de corriente habilitada." :
    "Calibracion de corriente deshabilitada.");

  Serial.println("SALIDA FISICA MOSFET: APAGADA");
  Serial.println("PWM INVERTIDO: 5 kHz / 8 bits");
  Serial.println("GPIO3 HIGH = MOSFET APAGADO");
  Serial.println("GPIO3 LOW = MOSFET ACTIVADO");
  Serial.println("PULSADOR MODO TEST: GPIO23 A GND");
  Serial.println("UNA PULSACION ENTRA; OTRA PULSACION SALE");
  Serial.println("PULSADOR TANDA: GPIO4 A GND");
  Serial.println("GPIO4: INICIAR / FIN TANDA / RESET TRAS CSV");
  Serial.println("PRUEBA MANUAL GATE: DISPONIBLE CON PIN");
  Serial.println("RANGO DE CORRIENTE CONFIGURABLE: 100 mA A 10 A");
  Serial.println("PROTECCION DE SOBRECORRIENTE: SEGUN LIMITE CONFIGURADO");
  Serial.println("REGULACION PWM AUTOMATICA: REALIMENTACION SHUNT");
  Serial.println("MEDICION PREVIA DE RESISTENCIA DESACTIVADA - GPIO2 PARA NTC");

  Serial.print("Red WiFi: ");
  Serial.println(ssid);

  Serial.print("Direccion web: ");
  Serial.println(WiFi.softAPIP());
}

// ==================================================
// LOOP
// ==================================================

void loop() {

  actualizarBotonTest();
  actualizarBotonTanda();
  server.handleClient();

  if (!modoTest && millis() - ultimaLectura >= 250) {

    ultimaLectura = millis();
    actualizarMediciones();
  }

  actualizarPrueba();
  actualizarPruebaGate();

  if (millis() - ultimaPantalla >= 500) {

    ultimaPantalla = millis();
    mostrarPantalla();
  }
}
