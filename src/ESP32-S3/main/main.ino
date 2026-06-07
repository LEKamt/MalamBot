// Firmware principal del microcontrolador ESP32 encardado de leer los datos de los 
// micrófonos y generar alerta de detección de fuga.

// ---  IMPORTANDO LIBRERÍAS ---
#include <Arduino.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>

// --- PINES DE CONTROL DEL MULTIPLEXOR 74HC151 ---
#define MUX_S0 16
#define MUX_S1 17
#define MUX_S2 18

// --- PINES I2S PARA EL ESP32 DEV (Maestro) ---
#define BCLK_0    15 
#define WS_0      2  
#define SD_0      32

// --- PINES HACIA EL myRIO / LABVIEW --- (Por cambiar a I2C)
#define RX_MYRIO  5
#define TX_MYRIO  4 

// --- PIN DE ALARMA ---
#define PIN_LED   13 

// --- CONFIGURACIÓN DE AUDIO Y FFT --- 
#define SAMPLES        256     // 256 muestras para barrido ultra rápido (~30 Hz)
#define SAMPLING_FREQ  44100 

// --- CONFIGURACIÓN DEL FILTRO DE RUIDO (AFINADO) ---
const double FREQ_MINIMA    = 750.0;   // Evita el ruido rosa de baja frecuencia
const double FREQ_MAXIMA    = 30000.0;  
const double UMBRAL_MAGNITUD = 800000.0; // Ignora la estática fantasma

double vReal[SAMPLES], vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

int32_t buffer[SAMPLES * 2]; 
int32_t dummyBuffer[SAMPLES * 2]; 
double frecuenciasDominantes[6];

// --- VARIABLES PARA EL TEMPORIZADOR DEL LED ---
unsigned long tiempoInicioFuga = 0;
bool fugaEnProceso = false;
const unsigned long TIEMPO_CONFIRMACION = 1000; // 1 segundo (1000 ms)

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_MYRIO, TX_MYRIO); 
  
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);

  // Configurar el pin del LED como salida
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.println("Iniciando ESP32 Dev Maestro Acústico de Alta Velocidad...");

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLING_FREQ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, 
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = SAMPLES,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = BCLK_0,
    .ws_io_num = WS_0,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = SD_0
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void loop() {
  bool hayFugaActual = false; // Rastrea si algún micrófono detecta fuga en esta vuelta

  // --- BARRIDO DE AUDIO MÁXIMA VELOCIDAD ---
  for (int canalMux = 0; canalMux < 6; canalMux++) {
    
    // 1. Conmutar el Multiplexor
    digitalWrite(MUX_S0, canalMux & 0x01);
    digitalWrite(MUX_S1, (canalMux >> 1) & 0x01);
    digitalWrite(MUX_S2, (canalMux >> 2) & 0x01);
    
    delayMicroseconds(500); 

    // 2. Limpieza DMA Rápida (64 muestras)
    size_t bytesRead;
    i2s_read(I2S_NUM_0, dummyBuffer, 64 * 4, &bytesRead, pdMS_TO_TICKS(10));

    // 3. Lectura Real
    esp_err_t result = i2s_read(I2S_NUM_0, buffer, sizeof(buffer), &bytesRead, pdMS_TO_TICKS(100));

    if (result == ESP_OK && bytesRead > 0) {
      
      // 4. Preparar datos para FFT
      for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = (double)(buffer[i] >> 8);     
        vImag[i] = 0.0; 
      }

      // Remoción DC Offset
      double mean = 0;
      for (int i = 0; i < SAMPLES; i++) mean += vReal[i];
      mean /= SAMPLES;
      for (int i = 0; i < SAMPLES; i++) vReal[i] -= mean;

      // 5. Calcular Transformada de Fourier
      FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT.compute(FFTDirection::Forward);
      FFT.complexToMagnitude(); 

      // 6. Búsqueda con Filtro Pasa-Banda
      double maxMagnitud = 0.0;
      int indicePico = 0;

      for (int i = 1; i < (SAMPLES / 2); i++) {
          double freqActual = (i * 1.0 * SAMPLING_FREQ) / SAMPLES;
          
          if (freqActual >= FREQ_MINIMA && freqActual <= FREQ_MAXIMA) {
              if (vReal[i] > maxMagnitud) {
                  maxMagnitud = vReal[i];
                  indicePico = i;
              }
          }
      }

      // 7. Aplicar umbral de silencio
      double pico = 0.0;
      if (maxMagnitud > UMBRAL_MAGNITUD) { 
          pico = (indicePico * 1.0 * SAMPLING_FREQ) / SAMPLES;
          
          // Si el pico supera los 1500 Hz, registramos detección de fuga
          if (pico >= 1000.0) {
            hayFugaActual = true;
          }
      }

      frecuenciasDominantes[canalMux] = pico;
    }
    
    yield(); // Alimentar al Watchdog
  }

  // --- LÓGICA DEL TEMPORIZADOR DEL LED ---
  if (hayFugaActual) {
    if (!fugaEnProceso) {
      // Comienza el conteo de los 3 segundos
      fugaEnProceso = true;
      tiempoInicioFuga = millis();
    } else {
      // Si el silbido persiste más de 3000 ms, encendemos el LED
      if (millis() - tiempoInicioFuga >= TIEMPO_CONFIRMACION) {
        digitalWrite(PIN_LED, HIGH); 
      }
    }
  } else {
    // Si la fuga desaparece, se apaga la alarma inmediatamente y se limpia el conteo
    fugaEnProceso = false;
    digitalWrite(PIN_LED, LOW);
  }

  // --- EMPAQUETAR Y ENVIAR AL myRIO ---
  String trama = String(frecuenciasDominantes[0], 1) + "," +
                 String(frecuenciasDominantes[1], 1) + "," +
                 String(frecuenciasDominantes[2], 1) + "," +
                 String(frecuenciasDominantes[3], 1) + "," +
                 String(frecuenciasDominantes[4], 1) + "," +
                 String(frecuenciasDominantes[5], 1);

  Serial.println(trama);   // Monitor Serie (PC)
  Serial1.println(trama);  // UART (myRIO)
}