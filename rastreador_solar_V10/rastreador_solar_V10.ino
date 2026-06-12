#include <WiFi.h>
#include <esp_sleep.h>
#include <WebServer.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <LittleFS.h>    
#include <time.h>         

// protótipos
void rastrearSol();
void lerDHT();
void atualizarOLED();
void salvarHistorico();
void salvarPerformance();
void adicionarLog(int nivel, const char* funcao, const char* mensagem);
void carregarConfig();
void salvarConfig();
void iniciarWiFi();
void gerenciarWiFi();
void sincronizarNTP();       
String sendFile(const char* path, const char* contentType);
void handleRoot();
void handleDados();
void handleHistorico();
void handleGraficos();
void handlePerformance();
void handlePerfData();
void handleLogs();
void handleLogsData();
void handleLogsClear();
void handleConfig();
void handleConfigData();
void handleConfigSave();
void handleSobre();
void handleChangelog();
void taskSensores(void* pv);
void taskManutencao(void* pv);
void gerenciarEnergia();
void persistirHistorico();
void restaurarHistorico();
unsigned long tempoSistema();
time_t epochAtual();           

// wifi
String ssid     = "INTELBRAS";
String password = "Comput@dor2020";

// pinos
#define LDR_TL  34
#define LDR_TR  35
#define LDR_BL  32
#define LDR_BR  33
#define DHT_PIN  4
#define DHT_TYPE DHT11
#define SERVO_H  13
#define SERVO_V  12
#define BOTAO_PIN 0

// objetos
WebServer server(80);
DHT dht(DHT_PIN, DHT_TYPE);
Servo servoH, servoV;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Preferences prefs;
Preferences prefsHist;

// leituras
float posH = 90.0, posV = 90.0;
float alvoH = 90.0, alvoV = 90.0;
float temperatura = 0.0, umidade = 0.0;
int   ldrTL, ldrTR, ldrBL, ldrBR;

// parâmetros
float kp         = 0.008;
float zonaMorta  = 300;
float passoMax   = 6.0;
float suavizacao = 0.12;

// intervalos
long intervaloLDR       = 30;
long intervaloDHT       = 2000;
long intervaloHistorico = 300000;
long intervaloPerf      = 5000;

// timers
unsigned long ultimaLeituraDHT  = 0;
unsigned long ultimaLeituraLDR  = 0;
unsigned long ultimoRegistro    = 0;
unsigned long ultimaPerformance = 0;

// concorrência e tasks FreeRTOS
portMUX_TYPE dadosMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t hTaskSensores = nullptr, hTaskManutencao = nullptr;

// interrupção de hardware
volatile bool    botaoEvento       = false;
volatile bool    rastreamentoAtivo = true;
volatile int64_t botaoUltimoUs     = 0;

void IRAM_ATTR botaoISR() {
  int64_t agora = esp_timer_get_time();
  if (agora - botaoUltimoUs > 250000) {
    botaoUltimoUs = agora;
    botaoEvento   = true;
  }
}

// gerenciamento de energia
bool economiaAtiva    = false;
int  limiarNoite      = 200;
long minutosDeepSleep = 30;
unsigned long inicioEscuro = 0;
#define TEMPO_CONFIRMA_NOITE 600000UL

// medição CPU
volatile unsigned long busyAcumuladoUs = 0;
unsigned long janelaCpuInicio = 0;

// linha do tempo — epoch NTP + offset de boot
unsigned long tempoOffsetMs = 0;
unsigned long tempoSistema() { return tempoOffsetMs + millis(); }

// retorna o epoch Unix atual. Se o NTP já sincronizou, usa time(nullptr), caso contrário usa o offset relativo como estimativa.
time_t epochAtual() {
  time_t t = time(nullptr);
  if (t > 1700000000UL) return t;  
  return (time_t)(tempoSistema() / 1000UL);
}

bool ntpSincronizado = false;
unsigned long ultimoTentativaNTP = 0;
#define INTERVALO_NTP 30000UL  

// buffer circular - histório 24h
#define MAX_HISTORICO 288
struct Registro {
  time_t        epoch;        
  unsigned long tempo;        
  float temperatura, umidade;
  int   ldrMedia, servoH, servoV;
};
Registro historico[MAX_HISTORICO];
int indiceAtual = 0, totalRegistros = 0;

// buffer de performance
#define MAX_PERF 60
struct RegistroPerf {
  unsigned long tempo;
  uint32_t heapLivre, minHeap;
  float cpuUsage;
};
RegistroPerf perfHist[MAX_PERF];
int perfIdx = 0, perfTotal = 0;

// tempos de execução
unsigned long tempoRastrearSol = 0;
unsigned long tempoLerDHT      = 0;
unsigned long tempoOLED        = 0;
unsigned long tempoHistorico   = 0;
unsigned long tempoWebServer   = 0;
float cpuUsage = 0.0;

// sistema de logs
#define MAX_LOGS   150
#define LOG_INFO   0
#define LOG_WARN   1
#define LOG_ERRO   2

struct LogEntry {
  unsigned long tempo;
  int           nivel;
  char          funcao[24];
  char          mensagem[80];
};

LogEntry logs[MAX_LOGS];
int logIdx   = 0;
int logTotal = 0;

void adicionarLog(int nivel, const char* funcao, const char* mensagem) {
  portENTER_CRITICAL(&dadosMux);
  logs[logIdx].tempo = tempoSistema();
  logs[logIdx].nivel = nivel;
  strncpy(logs[logIdx].funcao,   funcao,   23); logs[logIdx].funcao[23]   = 0;
  strncpy(logs[logIdx].mensagem, mensagem, 79); logs[logIdx].mensagem[79] = 0;
  logIdx = (logIdx + 1) % MAX_LOGS;
  if (logTotal < MAX_LOGS) logTotal++;
  portEXIT_CRITICAL(&dadosMux);

  const char* prefixo = nivel == LOG_ERRO ? "[ERRO]" : nivel == LOG_WARN ? "[WARN]" : "[INFO]";
  Serial.printf("%s [%s] %s\n", prefixo, funcao, mensagem);
}

// NTP — sincronização não-bloqueante
void sincronizarNTP() {
  if (!ntpSincronizado && WiFi.status() == WL_CONNECTED) {
    if (millis() - ultimoTentativaNTP >= INTERVALO_NTP) {
      ultimoTentativaNTP = millis();
      configTime(-3 * 3600, 0, "pool.ntp.org", "time.google.com");
      time_t t = time(nullptr);
      if (t > 1700000000UL) {
        ntpSincronizado = true;
        adicionarLog(LOG_INFO, "sincronizarNTP", "NTP sincronizado com sucesso");
        prefs.begin("rastreador", false);
        prefs.putULong("ntpEpoch", (unsigned long)t);
        prefs.putULong("ntpMillis", millis());
        prefs.end();
      }
    }
  }
}

void salvarHistorico() {
  unsigned long t0 = micros();
  int ldrMedia = (ldrTL + ldrTR + ldrBL + ldrBR) / 4;
  historico[indiceAtual] = {
    epochAtual(),           
    tempoSistema(),
    temperatura, umidade,
    ldrMedia, (int)posH, (int)posV
  };
  indiceAtual = (indiceAtual + 1) % MAX_HISTORICO;
  if (totalRegistros < MAX_HISTORICO) totalRegistros++;
  tempoHistorico = micros() - t0;
  adicionarLog(LOG_INFO, "salvarHistorico", ("Registro " + String(totalRegistros) + " salvo").c_str());

  if (totalRegistros % 6 == 0) persistirHistorico();
}

// persistência do histórico 24h (NVS)
void persistirHistorico() {
  prefsHist.begin("hist", false);
  prefsHist.putULong("offset", tempoSistema());
  prefsHist.putInt("idx", indiceAtual);
  prefsHist.putInt("total", totalRegistros);
  size_t gravado = prefsHist.putBytes("buf", historico, sizeof(historico));
  prefsHist.end();
  if (gravado != sizeof(historico)) {
    adicionarLog(LOG_ERRO, "persistirHistorico", "Falha ao gravar historico na NVS");
  } else {
    adicionarLog(LOG_INFO, "persistirHistorico", "Historico 24h gravado na NVS");
  }
}

void restaurarHistorico() {
  prefsHist.begin("hist", true);
  if (prefsHist.isKey("buf") && prefsHist.getBytesLength("buf") == sizeof(historico)) {
    prefsHist.getBytes("buf", historico, sizeof(historico));
    indiceAtual    = prefsHist.getInt("idx", 0);
    totalRegistros = prefsHist.getInt("total", 0);
    tempoOffsetMs  = prefsHist.getULong("offset", 0);
    adicionarLog(LOG_INFO, "restaurarHistorico", ("Historico restaurado: " + String(totalRegistros) + " registros").c_str());
  } else {
    adicionarLog(LOG_INFO, "restaurarHistorico", "Sem historico previo na NVS");
  }
  prefsHist.end();

  // restaura estimativa de epoch para os registros anteriores ao NTP
  prefs.begin("rastreador", true);
  unsigned long savedEpoch  = prefs.getULong("ntpEpoch",  0);
  unsigned long savedMillis = prefs.getULong("ntpMillis", 0);
  prefs.end();
  if (savedEpoch > 1700000000UL) {
    unsigned long deltaS = (millis() - savedMillis) / 1000UL;
    struct timeval tv = { .tv_sec = (time_t)(savedEpoch + deltaS), .tv_usec = 0 };
    settimeofday(&tv, nullptr);
    adicionarLog(LOG_INFO, "restaurarHistorico", "Epoch estimado restaurado da NVS");
  }
}

void salvarPerformance() {
  perfHist[perfIdx] = { millis(), ESP.getFreeHeap(), ESP.getMinFreeHeap(), cpuUsage };
  perfIdx = (perfIdx + 1) % MAX_PERF;
  if (perfTotal < MAX_PERF) perfTotal++;
}

// configuração persistente
void carregarConfig() {
  prefs.begin("rastreador", true);
  ssid       = prefs.getString("ssid", ssid);
  password   = prefs.getString("pass", password);
  kp         = prefs.getFloat("kp", kp);
  zonaMorta  = prefs.getFloat("zona", zonaMorta);
  passoMax   = prefs.getFloat("passo", passoMax);
  suavizacao = prefs.getFloat("suav", suavizacao);
  intervaloLDR       = prefs.getLong("iLDR", intervaloLDR);
  intervaloDHT       = prefs.getLong("iDHT", intervaloDHT);
  intervaloHistorico = prefs.getLong("iHist", intervaloHistorico);
  intervaloPerf      = prefs.getLong("iPerf", intervaloPerf);
  economiaAtiva      = prefs.getBool("eco", economiaAtiva);
  limiarNoite        = prefs.getInt("ecoLim", limiarNoite);
  minutosDeepSleep   = prefs.getLong("ecoMin", minutosDeepSleep);
  prefs.end();
  adicionarLog(LOG_INFO, "carregarConfig", "Configuracao carregada da NVS");
}

void salvarConfig() {
  prefs.begin("rastreador", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", password);
  prefs.putFloat("kp", kp);
  prefs.putFloat("zona", zonaMorta);
  prefs.putFloat("passo", passoMax);
  prefs.putFloat("suav", suavizacao);
  prefs.putLong("iLDR", intervaloLDR);
  prefs.putLong("iDHT", intervaloDHT);
  prefs.putLong("iHist", intervaloHistorico);
  prefs.putLong("iPerf", intervaloPerf);
  prefs.putBool("eco", economiaAtiva);
  prefs.putInt("ecoLim", limiarNoite);
  prefs.putLong("ecoMin", minutosDeepSleep);
  prefs.end();
  adicionarLog(LOG_INFO, "salvarConfig", "Configuracao gravada na NVS");
}

// wifi não bloqueante
unsigned long wifiUltimaTentativa = 0;
const unsigned long WIFI_INTERVALO_RECONEXAO = 10000;
bool wifiConectadoAnterior = false;

void iniciarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  WiFi.begin(ssid.c_str(), password.c_str());
  wifiUltimaTentativa = millis();
  adicionarLog(LOG_INFO, "iniciarWiFi", ("Conectando ao Wi-Fi: " + ssid).c_str());
}

void gerenciarWiFi() {
  bool conectado = (WiFi.status() == WL_CONNECTED);
  if (conectado && !wifiConectadoAnterior) {
    adicionarLog(LOG_INFO, "gerenciarWiFi", ("Wi-Fi conectado. IP: " + WiFi.localIP().toString()).c_str());
  }
  wifiConectadoAnterior = conectado;
  if (!conectado && millis() - wifiUltimaTentativa >= WIFI_INTERVALO_RECONEXAO) {
    wifiUltimaTentativa = millis();
    WiFi.begin(ssid.c_str(), password.c_str());
    adicionarLog(LOG_WARN, "gerenciarWiFi", "Tentando reconectar ao Wi-Fi...");
  }
}

// LITTLE FS
String sendFile(const char* path, const char* contentType) {
  if (LittleFS.exists(path)) {
    File f = LittleFS.open(path, "r");
    server.streamFile(f, contentType);
    f.close();
    return "";
  }
  String msg = "<h2>Arquivo não encontrado: ";
  msg += path;
  msg += "</h2><p>Faça o upload dos arquivos HTML via LittleFS (Data Upload Tool).</p>";
  server.send(404, "text/html", msg);
  return "";
}

void setup() {
  Serial.begin(115200);
  adicionarLog(LOG_INFO, "setup", "Iniciando sistema v11...");

  esp_sleep_wakeup_cause_t causa = esp_sleep_get_wakeup_cause();
  if (causa == ESP_SLEEP_WAKEUP_TIMER) {
    adicionarLog(LOG_INFO, "setup", "Acordou do deep sleep noturno (timer)");
  } else if (causa == ESP_SLEEP_WAKEUP_EXT0) {
    adicionarLog(LOG_INFO, "setup", "Acordou do deep sleep pelo botao BOOT");
  }

  // inicia o little fs
  if (!LittleFS.begin(true)) {  
    adicionarLog(LOG_ERRO, "setup", "Falha ao montar LittleFS");
  } else {
    adicionarLog(LOG_INFO, "setup", "LittleFS montado com sucesso");
  }

  carregarConfig();
  restaurarHistorico();

  dht.begin();
  ultimaLeituraDHT = millis();

  servoH.attach(SERVO_H);
  servoV.attach(SERVO_V);
  servoH.write((int)posH);
  servoV.write((int)posV);
  adicionarLog(LOG_INFO, "setup", "Servos inicializados em 90 graus");

  pinMode(BOTAO_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BOTAO_PIN), botaoISR, FALLING);
  adicionarLog(LOG_INFO, "setup", "Interrupcao de hardware no botao BOOT habilitada");

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    adicionarLog(LOG_ERRO, "setup", "OLED SSD1306 nao encontrado no endereco 0x3C");
  } else {
    adicionarLog(LOG_INFO, "setup", "OLED inicializado com sucesso");
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("Iniciando v11...");
  display.display();

  iniciarWiFi();
  WiFi.setSleep(true);
  adicionarLog(LOG_INFO, "setup", "Modem sleep do Wi-Fi habilitado");

  // -rotas do servidor web
  server.on("/",            handleRoot);
  server.on("/dados",       handleDados);
  server.on("/historico",   handleHistorico);
  server.on("/graficos",    handleGraficos);
  server.on("/performance", handlePerformance);
  server.on("/perfdata",    handlePerfData);
  server.on("/logs",        handleLogs);
  server.on("/logsdata",    handleLogsData);
  server.on("/logsclear",   handleLogsClear);
  server.on("/config",      handleConfig);
  server.on("/configdata",  handleConfigData);
  server.on("/configsave",  HTTP_POST, handleConfigSave);
  server.on("/sobre",       handleSobre);
  server.on("/changelog",   handleChangelog);
  server.begin();
  adicionarLog(LOG_INFO, "setup", "Servidor web iniciado em porta 80");

  xTaskCreatePinnedToCore(taskSensores,   "sensores",   6144, nullptr, 2, &hTaskSensores,   1);
  xTaskCreatePinnedToCore(taskManutencao, "manutencao", 6144, nullptr, 1, &hTaskManutencao, 0);
  adicionarLog(LOG_INFO, "setup", "Tasks criadas: sensores (core 1) e manutencao (core 0)");
  janelaCpuInicio = millis();
}

// loop - servidor web, wifi e botão
void loop() {
  unsigned long t0 = micros();
  server.handleClient();
  unsigned long gasto = micros() - t0;
  tempoWebServer = gasto;
  portENTER_CRITICAL(&dadosMux);
  busyAcumuladoUs += gasto;
  portEXIT_CRITICAL(&dadosMux);

  gerenciarWiFi();

  if (botaoEvento) {
    botaoEvento = false;
    rastreamentoAtivo = !rastreamentoAtivo;
    adicionarLog(LOG_INFO, "botaoISR", rastreamentoAtivo
      ? "Rastreamento retomado pelo botao BOOT"
      : "Rastreamento pausado pelo botao BOOT");
  }
  vTaskDelay(pdMS_TO_TICKS(2));  
}

// task sensores (core 1)
void taskSensores(void* pv) {
  for (;;) {
    unsigned long agora = millis();
    unsigned long t0 = micros();
    bool trabalhou = false;

    if (rastreamentoAtivo && agora - ultimaLeituraLDR >= (unsigned long)intervaloLDR) {
      ultimaLeituraLDR = agora;
      rastrearSol();
      atualizarOLED();
      trabalhou = true;
    }
    if (agora - ultimaLeituraDHT >= (unsigned long)intervaloDHT) {
      ultimaLeituraDHT = agora;
      lerDHT();
      trabalhou = true;
    }
    if (trabalhou) {
      unsigned long gasto = micros() - t0;
      portENTER_CRITICAL(&dadosMux);
      busyAcumuladoUs += gasto;
      portEXIT_CRITICAL(&dadosMux);
    }
    vTaskDelay(pdMS_TO_TICKS(5));  // NÃO BLOQUEANTE
  }
}

// task manutencao (core 0)
void taskManutencao(void* pv) {
  for (;;) {
    unsigned long agora = millis();

    sincronizarNTP();

    if (agora - ultimoRegistro >= (unsigned long)intervaloHistorico) {
      ultimoRegistro = agora;
      salvarHistorico();
    }
    if (agora - ultimaPerformance >= (unsigned long)intervaloPerf) {
      ultimaPerformance = agora;
      unsigned long janelaUs = (agora - janelaCpuInicio) * 1000UL;
      portENTER_CRITICAL(&dadosMux);
      unsigned long busy = busyAcumuladoUs;
      busyAcumuladoUs = 0;
      portEXIT_CRITICAL(&dadosMux);
      janelaCpuInicio = agora;
      cpuUsage = janelaUs > 0 ? constrain((float)busy / (float)(janelaUs * 2) * 100.0f, 0.0f, 100.0f) : 0.0f;
      salvarPerformance();
    }
    gerenciarEnergia();
    vTaskDelay(pdMS_TO_TICKS(50));  
  }
}

// gerenciamento de energia
void gerenciarEnergia() {
  if (!economiaAtiva) { inicioEscuro = 0; return; }

  int media = (ldrTL + ldrTR + ldrBL + ldrBR) / 4;
  if (media >= limiarNoite) { inicioEscuro = 0; return; }

  if (inicioEscuro == 0) {
    inicioEscuro = millis();
    adicionarLog(LOG_WARN, "gerenciarEnergia", "Escuridao detectada; deep sleep em 10 min se persistir");
    return;
  }
  if (millis() - inicioEscuro >= TEMPO_CONFIRMA_NOITE) {
    adicionarLog(LOG_WARN, "gerenciarEnergia", "Entrando em deep sleep noturno");
    persistirHistorico();
    esp_sleep_enable_timer_wakeup((uint64_t)minutosDeepSleep * 60ULL * 1000000ULL);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
    esp_deep_sleep_start();
  }
}

void rastrearSol() {
  unsigned long t0 = micros();
  int tl = analogRead(LDR_TL);
  int tr = analogRead(LDR_TR);
  int bl = analogRead(LDR_BL);
  int br = analogRead(LDR_BR);

  int mediaTop = (tl + tr) / 2;
  int mediaBot = (bl + br) / 2;
  float erro   = mediaBot - mediaTop;

  float novoAlvoV = alvoV;
  if (abs(erro) > zonaMorta) {
    float passo = constrain(erro * kp, -passoMax, passoMax);
    novoAlvoV = constrain(alvoV + passo, 20, 160);
  }
  float novaPosV = posV + (novoAlvoV - posV) * suavizacao;

  portENTER_CRITICAL(&dadosMux);
  ldrTL = tl; ldrTR = tr; ldrBL = bl; ldrBR = br;
  alvoV = novoAlvoV;
  posV  = novaPosV;
  portEXIT_CRITICAL(&dadosMux);

  servoV.write((int)novaPosV);
  servoH.write(90);
  tempoRastrearSol = micros() - t0;
}

void lerDHT() {
  unsigned long t0 = micros();
  float t = dht.readTemperature();
  float u = dht.readHumidity();

  if (!isnan(t) && t >= 0.0 && t <= 60.0) {
    portENTER_CRITICAL(&dadosMux);
    temperatura = t;
    portEXIT_CRITICAL(&dadosMux);
  } else {
    adicionarLog(LOG_WARN, "lerDHT", "Leitura de temperatura invalida descartada");
  }
  if (!isnan(u) && u >= 0.0 && u <= 100.0) {
    portENTER_CRITICAL(&dadosMux);
    umidade = u;
    portEXIT_CRITICAL(&dadosMux);
  } else {
    adicionarLog(LOG_WARN, "lerDHT", "Leitura de umidade invalida descartada");
  }
  tempoLerDHT = micros() - t0;
}

void atualizarOLED() {
  unsigned long t0 = micros();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Temp: %.1fC\n", temperatura);
  display.printf("Umid: %.1f%%\n", umidade);
  display.printf("V:%.0f H:%.0f\n", posV, posH);
  display.printf("IP:%s\n", WiFi.localIP().toString().c_str());
  display.display();
  tempoOLED = micros() - t0;
}

// rotas das páginas
void handleRoot()        { sendFile("/index.html",       "text/html"); }
void handleGraficos()    { sendFile("/graficos.html",    "text/html"); }
void handlePerformance() { sendFile("/performance.html", "text/html"); }
void handleLogs()        { sendFile("/logs.html",        "text/html"); }
void handleConfig()      { sendFile("/config.html",      "text/html"); }
void handleSobre()       { sendFile("/sobre.html",       "text/html"); }
void handleChangelog()   { sendFile("/changelog.html",   "text/html"); }

// rota dados
void handleDados() {
  StaticJsonDocument<300> doc;
  doc["temperatura"] = temperatura; doc["umidade"] = umidade;
  doc["servoH"] = (int)posH; doc["servoV"] = (int)posV;
  doc["ldrTL"] = ldrTL; doc["ldrTR"] = ldrTR;
  doc["ldrBL"] = ldrBL; doc["ldrBR"] = ldrBR;
  doc["ntpOk"] = ntpSincronizado;
  String json; serializeJson(doc, json);
  server.send(200, "application/json", json);
}

// rota histórico
void handleHistorico() {
  int inicio = (totalRegistros < MAX_HISTORICO) ? 0 : indiceAtual;
  String json = "{\"total\":"; json += totalRegistros; json += ",\"ntpOk\":";
  json += ntpSincronizado ? "true" : "false";
  json += ",\"dados\":[";
  for (int i = 0; i < totalRegistros; i++) {
    int idx = (inicio + i) % MAX_HISTORICO;
    if (i > 0) json += ",";
    json += "{\"t\":"    + String(historico[idx].tempo/1000)
          + ",\"epoch\":" + String((unsigned long)historico[idx].epoch)  // ← NOVO
          + ",\"temp\":" + String(historico[idx].temperatura)
          + ",\"umid\":" + String(historico[idx].umidade)
          + ",\"ldr\":"  + String(historico[idx].ldrMedia)
          + ",\"sh\":"   + String(historico[idx].servoH)
          + ",\"sv\":"   + String(historico[idx].servoV) + "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// rota logs data
void handleLogsData() {
  int inicio = (logTotal < MAX_LOGS) ? 0 : logIdx;
  String json = "{\"total\":" + String(logTotal) + ",\"logs\":[";
  for (int i = 0; i < logTotal; i++) {
    int idx = (inicio + i) % MAX_LOGS;
    if (i > 0) json += ",";
    String nivel = logs[idx].nivel == LOG_ERRO ? "ERRO" :
                   logs[idx].nivel == LOG_WARN ? "WARN" : "INFO";
    json += "{\"t\":" + String(logs[idx].tempo)
          + ",\"nivel\":\"" + nivel + "\""
          + ",\"funcao\":\"" + String(logs[idx].funcao) + "\""
          + ",\"msg\":\"" + String(logs[idx].mensagem) + "\"}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// rota logs clear
void handleLogsClear() {
  logIdx = 0; logTotal = 0;
  adicionarLog(LOG_INFO, "handleLogsClear", "Logs limpos pelo usuario via web");
  server.send(200, "application/json", "{\"ok\":true}");
}

//rota perf data
void handlePerfData() {
  uint32_t heapTotal = ESP.getHeapSize(), heapLivre = ESP.getFreeHeap();
  uint32_t heapMin = ESP.getMinFreeHeap(), heapUsado = heapTotal - heapLivre;
  uint32_t flashTotal = ESP.getFlashChipSize();
  uint32_t sketchUsado = ESP.getSketchSize(), sketchLivre = ESP.getFreeSketchSpace();
  uint32_t psramTotal = ESP.getPsramSize(), psramLivre = ESP.getFreePsram();
  int rssi = WiFi.RSSI();

  String json = "{";
  json += "\"uptime_s\":"    + String(tempoSistema()/1000) + ",";
  json += "\"cpu_freq_mhz\":"+ String(getCpuFrequencyMhz()) + ",";
  json += "\"cpu_pct\":"     + String(cpuUsage, 1) + ",";
  json += "\"rastreamento\":"+ String(rastreamentoAtivo ? "true" : "false") + ",";
  json += "\"num_tasks\":"   + String(uxTaskGetNumberOfTasks()) + ",";
  json += "\"ntpOk\":"       + String(ntpSincronizado ? "true" : "false") + ",";
  json += "\"heap\":{\"total\":"   + String(heapTotal)
        + ",\"livre\":"  + String(heapLivre)
        + ",\"usado\":"  + String(heapUsado)
        + ",\"minimo\":" + String(heapMin)
        + ",\"pct_usado\":" + String((int)((float)heapUsado/heapTotal*100)) + "},";
  json += "\"stack\":{\"loop\":" + String((uint32_t)uxTaskGetStackHighWaterMark(nullptr))
        + ",\"sensores\":"   + String(hTaskSensores   ? (uint32_t)uxTaskGetStackHighWaterMark(hTaskSensores)   : 0)
        + ",\"manutencao\":" + String(hTaskManutencao ? (uint32_t)uxTaskGetStackHighWaterMark(hTaskManutencao) : 0) + "},";
  json += "\"flash\":{\"total\":"        + String(flashTotal)
        + ",\"sketch_usado\":"+ String(sketchUsado)
        + ",\"sketch_livre\":"+ String(sketchLivre) + "},";
  json += "\"psram\":{\"total\":"+ String(psramTotal)
        + ",\"livre\":"  + String(psramLivre) + "},";
  json += "\"wifi\":{\"ssid\":\"" + String(ssid) + "\""
        + ",\"ip\":\"" + WiFi.localIP().toString() + "\""
        + ",\"rssi\":" + String(rssi)
        + ",\"canal\":" + String(WiFi.channel())
        + ",\"status\":\"" + (WiFi.status()==WL_CONNECTED?"Conectado":"Desconectado") + "\"},";
  json += "\"tempos_us\":{\"rastrearSol\":"   + String(tempoRastrearSol)
        + ",\"lerDHT\":"         + String(tempoLerDHT)
        + ",\"atualizarOLED\":"  + String(tempoOLED)
        + ",\"salvarHistorico\":"+ String(tempoHistorico)
        + ",\"webServer\":"      + String(tempoWebServer) + "},";
  json += "\"sensores\":{\"temperatura\":" + String(temperatura)
        + ",\"umidade\":"  + String(umidade)
        + ",\"ldrTL\":"    + String(ldrTL)
        + ",\"ldrTR\":"    + String(ldrTR)
        + ",\"ldrBL\":"    + String(ldrBL)
        + ",\"ldrBR\":"    + String(ldrBR)
        + ",\"servoV\":"   + String((int)posV)
        + ",\"servoH\":"   + String((int)posH) + "},";

  struct InfoTask { const char* nome; const char* resp; TaskHandle_t h; int core; };
  InfoTask infos[] = {
    { "loopTask",   "Servidor web, Wi-Fi e botao",       xTaskGetCurrentTaskHandle(), 1 },
    { "sensores",   "LDRs, servos, DHT e OLED",          hTaskSensores,               1 },
    { "manutencao", "Historico, performance e energia",  hTaskManutencao,             0 },
  };
  json += "\"tasks\":[";
  for (int i = 0; i < 3; i++) {
    if (i > 0) json += ",";
    int prio = infos[i].h ? (int)uxTaskPriorityGet(infos[i].h) : -1;
    uint32_t hwm = infos[i].h ? (uint32_t)uxTaskGetStackHighWaterMark(infos[i].h) : 0;
    const char* estado = "?";
    if (infos[i].h) {
      switch (eTaskGetState(infos[i].h)) {
        case eRunning:   estado = "Running";   break;
        case eReady:     estado = "Ready";     break;
        case eBlocked:   estado = "Blocked";   break;
        case eSuspended: estado = "Suspended"; break;
        default:         estado = "Outro";     break;
      }
    }
    json += "{\"nome\":\"" + String(infos[i].nome)
          + "\",\"resp\":\"" + String(infos[i].resp)
          + "\",\"core\":" + String(infos[i].core)
          + ",\"prio\":" + String(prio)
          + ",\"stack_livre\":" + String(hwm)
          + ",\"estado\":\"" + String(estado) + "\"}";
  }
  json += "],";

  int inicio = (perfTotal < MAX_PERF) ? 0 : perfIdx;
  json += "\"historico_perf\":[";
  for (int i = 0; i < perfTotal; i++) {
    int idx = (inicio + i) % MAX_PERF;
    if (i > 0) json += ",";
    json += "{\"t\":"+ String(perfHist[idx].tempo/1000)
          + ",\"heap\":"+ String(perfHist[idx].heapLivre)
          + ",\"minheap\":"+ String(perfHist[idx].minHeap) + "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// rota config data
void handleConfigData() {
  StaticJsonDocument<512> doc;
  doc["ssid"]  = ssid;
  doc["pass"]  = password;
  doc["iLDR"]  = intervaloLDR;
  doc["iDHT"]  = intervaloDHT;
  doc["iHist"] = intervaloHistorico;
  doc["iPerf"] = intervaloPerf;
  doc["kp"]    = kp;
  doc["zona"]  = zonaMorta;
  doc["passo"] = passoMax;
  doc["suav"]  = suavizacao;
  doc["eco"]    = economiaAtiva ? 1 : 0;
  doc["ecoLim"] = limiarNoite;
  doc["ecoMin"] = minutosDeepSleep;
  String json; serializeJson(doc, json);
  server.send(200, "application/json", json);
}

// rota config save
void handleConfigSave() {
  bool trocouWiFi = false;

  if (server.hasArg("ssid")) {
    String novo = server.arg("ssid");
    if (novo != ssid) { ssid = novo; trocouWiFi = true; }
  }
  if (server.hasArg("pass")) {
    String novo = server.arg("pass");
    if (novo != password) { password = novo; trocouWiFi = true; }
  }
  if (server.hasArg("iLDR"))  intervaloLDR       = server.arg("iLDR").toInt();
  if (server.hasArg("iDHT"))  intervaloDHT       = server.arg("iDHT").toInt();
  if (server.hasArg("iHist")) intervaloHistorico = server.arg("iHist").toInt();
  if (server.hasArg("iPerf")) intervaloPerf      = server.arg("iPerf").toInt();
  if (server.hasArg("kp"))    kp         = server.arg("kp").toFloat();
  if (server.hasArg("zona"))  zonaMorta  = server.arg("zona").toFloat();
  if (server.hasArg("passo")) passoMax   = server.arg("passo").toFloat();
  if (server.hasArg("suav"))  suavizacao = server.arg("suav").toFloat();
  if (server.hasArg("eco"))    economiaAtiva    = server.arg("eco").toInt() == 1;
  if (server.hasArg("ecoLim")) limiarNoite      = server.arg("ecoLim").toInt();
  if (server.hasArg("ecoMin")) minutosDeepSleep = server.arg("ecoMin").toInt();

  salvarConfig();
  server.send(200, "application/json", "{\"ok\":true}");

  if (trocouWiFi) {
    adicionarLog(LOG_INFO, "handleConfigSave", "Credenciais alteradas, reconectando Wi-Fi");
    iniciarWiFi();
  }
}
