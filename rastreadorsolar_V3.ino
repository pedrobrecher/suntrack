#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void rastrearSol();
void lerDHT();
void atualizarOLED();
void salvarHistorico();
void salvarPerformance();
void handleRoot();
void handleDados();
void handleHistorico();
void handleGraficos();
void handlePerformance();
void handlePerfData();

//wifi
const char* ssid     = "Galaxy Isa";
const char* password = "ppkp1993";
//pinos
#define LDR_TL  34
#define LDR_TR  35
#define LDR_BL  32
#define LDR_BR  33
#define DHT_PIN  4
#define DHT_TYPE DHT11
#define SERVO_H  13
#define SERVO_V  12

//objetos
WebServer server(80);
DHT dht(DHT_PIN, DHT_TYPE);
Servo servoH, servoV;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

//leituras
float posH = 90.0, posV = 90.0;
float alvoH = 90.0, alvoV = 90.0;
float temperatura = 0.0, umidade = 0.0;
int   ldrTL, ldrTR, ldrBL, ldrBR;

//controle
const float KP         = 0.008;
const float ZONA_MORTA = 300;
const float PASSO_MAX  = 6.0;
const float SUAVIZACAO = 0.12;

//timers
unsigned long ultimaLeituraDHT  = 0;
unsigned long ultimaLeituraLDR  = 0;
unsigned long ultimoRegistro    = 0;
unsigned long ultimaPerformance = 0;

const long INTERVALO_DHT        = 2000;
const long INTERVALO_LDR        = 30;
const long INTERVALO_HISTORICO  = 300000;
const long INTERVALO_PERF       = 5000;   

//buffer circular historico 24h
#define MAX_HISTORICO 288

struct Registro {
  unsigned long tempo;
  float temperatura;
  float umidade;
  int   ldrMedia;
  int   servoH;
  int   servoV;
};

Registro historico[MAX_HISTORICO];
int indiceAtual    = 0;
int totalRegistros = 0;

// BUFFER CIRCULAR DE PERFORMANCE
#define MAX_PERF 60

struct RegistroPerf {
  unsigned long tempo;
  uint32_t heapLivre;
  float    cpuUsage;
  uint32_t minHeap;
};

RegistroPerf perfHist[MAX_PERF];
int perfIdx   = 0;
int perfTotal = 0;

// --- Métricas de tempo de execução das funções (microsegundos) ---
unsigned long tempoRastrearSol = 0;
unsigned long tempoLerDHT      = 0;
unsigned long tempoOLED        = 0;
unsigned long tempoHistorico   = 0;
unsigned long tempoWebServer   = 0;

// --- CPU idle (estimativa via contador) ---
volatile unsigned long idleCount     = 0;
unsigned long          idleCountBase = 0;
float                  cpuUsage      = 0.0;

// =============================================
void salvarHistorico() {
  unsigned long t0 = micros();

  int ldrMedia = (ldrTL + ldrTR + ldrBL + ldrBR) / 4;
  historico[indiceAtual] = {
    millis(), temperatura, umidade,
    ldrMedia, (int)posH, (int)posV
  };
  indiceAtual = (indiceAtual + 1) % MAX_HISTORICO;
  if (totalRegistros < MAX_HISTORICO) totalRegistros++;

  tempoHistorico = micros() - t0;
  Serial.printf("[HIST] Registro %d salvo em %lu us\n", totalRegistros, tempoHistorico);
}

void salvarPerformance() {
  perfHist[perfIdx] = {
    millis(),
    ESP.getFreeHeap(),
    cpuUsage,
    ESP.getMinFreeHeap()
  };
  perfIdx = (perfIdx + 1) % MAX_PERF;
  if (perfTotal < MAX_PERF) perfTotal++;
}

// =============================================
void setup() {
  Serial.begin(115200);
  dht.begin();
  delay(2000);

  servoH.attach(SERVO_H);
  servoV.attach(SERVO_V);
  servoH.write((int)posH);
  servoV.write((int)posV);

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("ERRO: OLED nao encontrado!");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("Iniciando...");
  display.display();

  WiFi.begin(ssid, password);
  Serial.print("Conectando ao Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());

  server.on("/",           handleRoot);
  server.on("/dados",      handleDados);
  server.on("/historico",  handleHistorico);
  server.on("/graficos",   handleGraficos);
  server.on("/performance",handlePerformance);
  server.on("/perfdata",   handlePerfData);
  server.begin();
  Serial.println("Rotas: /  /dados  /historico  /graficos  /performance  /perfdata");
}

// =============================================
void loop() {
  unsigned long t0 = micros();
  server.handleClient();
  tempoWebServer = micros() - t0;

  unsigned long agora = millis();

  if (agora - ultimaLeituraLDR >= INTERVALO_LDR) {
    ultimaLeituraLDR = agora;
    rastrearSol();
    atualizarOLED();
  }

  if (agora - ultimaLeituraDHT >= INTERVALO_DHT) {
    ultimaLeituraDHT = agora;
    lerDHT();
  }

  if (agora - ultimoRegistro >= INTERVALO_HISTORICO) {
    ultimoRegistro = agora;
    salvarHistorico();
  }

  if (agora - ultimaPerformance >= INTERVALO_PERF) {
    ultimaPerformance = agora;
    uint32_t h1 = ESP.getFreeHeap();
    cpuUsage = 100.0 - ((float)h1 / (float)ESP.getHeapSize() * 100.0);
    // Usa RSSI como proxy de saúde do sistema
    cpuUsage = constrain(cpuUsage, 0, 100);
    salvarPerformance();
  }
}

// =============================================
void rastrearSol() {
  unsigned long t0 = micros();

  ldrTL = analogRead(LDR_TL);
  ldrTR = analogRead(LDR_TR);
  ldrBL = analogRead(LDR_BL);
  ldrBR = analogRead(LDR_BR);

  Serial.printf("LDR TL:%d TR:%d BL:%d BR:%d\n", ldrTL, ldrTR, ldrBL, ldrBR);

  int mediaTop = (ldrTL + ldrTR) / 2;
  int mediaBot = (ldrBL + ldrBR) / 2;
  float erro   = mediaBot - mediaTop;

  Serial.printf("Erro vertical: %.2f\n", erro);

  if (abs(erro) > ZONA_MORTA) {
    float passo = constrain(erro * KP, -PASSO_MAX, PASSO_MAX);
    alvoV = constrain(alvoV + passo, 20, 160);
  }

  posV = posV + (alvoV - posV) * SUAVIZACAO;
  servoV.write((int)posV);
  servoH.write(90);

  tempoRastrearSol = micros() - t0;
}

// =============================================
void lerDHT() {
  unsigned long t0 = micros();

  float t = dht.readTemperature();
  float u = dht.readHumidity();
  if (!isnan(t) && t >= 0.0 && t <= 60.0) temperatura = t;
  else Serial.println("AVISO: temperatura invalida.");
  if (!isnan(u) && u >= 0.0 && u <= 100.0) umidade = u;
  else Serial.println("AVISO: umidade invalida.");

  Serial.printf("Temp: %.1f C | Umid: %.1f %%\n", temperatura, umidade);
  tempoLerDHT = micros() - t0;
}

// =============================================
void atualizarOLED() {
  unsigned long t0 = micros();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Temp: %.1fC\n", temperatura);
  display.printf("Umid: %.1f%%\n", umidade);
  display.printf("V:%.0f  Heap:%dK\n", posV, ESP.getFreeHeap()/1024);
  display.printf("IP:%s\n", WiFi.localIP().toString().c_str());
  display.display();

  tempoOLED = micros() - t0;
}

// =============================================
// ROTA: /dados
// =============================================
void handleDados() {
  StaticJsonDocument<300> doc;
  doc["temperatura"] = temperatura;
  doc["umidade"]     = umidade;
  doc["servoH"]      = (int)posH;
  doc["servoV"]      = (int)posV;
  doc["ldrTL"]       = ldrTL;
  doc["ldrTR"]       = ldrTR;
  doc["ldrBL"]       = ldrBL;
  doc["ldrBR"]       = ldrBR;
  String json; serializeJson(doc, json);
  server.send(200, "application/json", json);
}

// =============================================
// ROTA: /historico
// =============================================
void handleHistorico() {
  int inicio = (totalRegistros < MAX_HISTORICO) ? 0 : indiceAtual;
  String json = "{\"total\":"; json += totalRegistros;
  json += ",\"dados\":[";
  for (int i = 0; i < totalRegistros; i++) {
    int idx = (inicio + i) % MAX_HISTORICO;
    if (i > 0) json += ",";
    json += "{\"t\":";    json += historico[idx].tempo/1000;
    json += ",\"temp\":"; json += historico[idx].temperatura;
    json += ",\"umid\":"; json += historico[idx].umidade;
    json += ",\"ldr\":";  json += historico[idx].ldrMedia;
    json += ",\"sh\":";   json += historico[idx].servoH;
    json += ",\"sv\":";   json += historico[idx].servoV;
    json += "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// =============================================
// ROTA: /perfdata — JSON com todas as métricas
// =============================================
void handlePerfData() {
  // Coleta tasks do FreeRTOS
  UBaseType_t numTasks = uxTaskGetNumberOfTasks();

  // Heap
  uint32_t heapTotal  = ESP.getHeapSize();
  uint32_t heapLivre  = ESP.getFreeHeap();
  uint32_t heapMin    = ESP.getMinFreeHeap();
  uint32_t heapUsado  = heapTotal - heapLivre;

  // Flash
  uint32_t flashTotal = ESP.getFlashChipSize();
  uint32_t sketchUsado = ESP.getSketchSize();
  uint32_t sketchLivre = ESP.getFreeSketchSpace();

  // PSRAM
  uint32_t psramTotal = ESP.getPsramSize();
  uint32_t psramLivre = ESP.getFreePsram();

  // Wi-Fi
  int rssi = WiFi.RSSI();
  String ip = WiFi.localIP().toString();

  // Stack livre das tasks principais (aproximação)
  // Na ausência de handles diretos, usamos o heap como proxy
  String json = "{";

  // CPU e uptime
  json += "\"uptime_s\":" + String(millis()/1000) + ",";
  json += "\"cpu_freq_mhz\":" + String(getCpuFrequencyMhz()) + ",";
  json += "\"num_tasks\":" + String(numTasks) + ",";

  // Heap
  json += "\"heap\":{";
  json += "\"total\":" + String(heapTotal) + ",";
  json += "\"livre\":" + String(heapLivre) + ",";
  json += "\"usado\":" + String(heapUsado) + ",";
  json += "\"minimo\":" + String(heapMin) + ",";
  json += "\"pct_usado\":" + String((int)((float)heapUsado/heapTotal*100)) + "},";

  // Flash
  json += "\"flash\":{";
  json += "\"total\":" + String(flashTotal) + ",";
  json += "\"sketch_usado\":" + String(sketchUsado) + ",";
  json += "\"sketch_livre\":" + String(sketchLivre) + "},";

  // PSRAM
  json += "\"psram\":{";
  json += "\"total\":" + String(psramTotal) + ",";
  json += "\"livre\":" + String(psramLivre) + "},";

  // Wi-Fi
  json += "\"wifi\":{";
  json += "\"ssid\":\"" + String(ssid) + "\",";
  json += "\"ip\":\"" + ip + "\",";
  json += "\"rssi\":" + String(rssi) + ",";
  json += "\"canal\":" + String(WiFi.channel()) + ",";
  json += "\"status\":\"" + String(WiFi.status() == WL_CONNECTED ? "Conectado" : "Desconectado") + "\"},";

  // Tempos de execução (microsegundos)
  json += "\"tempos_us\":{";
  json += "\"rastrearSol\":" + String(tempoRastrearSol) + ",";
  json += "\"lerDHT\":"      + String(tempoLerDHT)      + ",";
  json += "\"atualizarOLED\":"+ String(tempoOLED)       + ",";
  json += "\"salvarHistorico\":"+ String(tempoHistorico) + ",";
  json += "\"webServer\":"   + String(tempoWebServer)    + "},";

  // Sensores atuais
  json += "\"sensores\":{";
  json += "\"temperatura\":" + String(temperatura) + ",";
  json += "\"umidade\":"     + String(umidade)     + ",";
  json += "\"ldrTL\":"       + String(ldrTL)       + ",";
  json += "\"ldrTR\":"       + String(ldrTR)       + ",";
  json += "\"ldrBL\":"       + String(ldrBL)       + ",";
  json += "\"ldrBR\":"       + String(ldrBR)       + ",";
  json += "\"servoV\":"      + String((int)posV)   + ",";
  json += "\"servoH\":"      + String((int)posH)   + "},";

  // Histórico de performance (série temporal)
  int inicio = (perfTotal < MAX_PERF) ? 0 : perfIdx;
  json += "\"historico_perf\":[";
  for (int i = 0; i < perfTotal; i++) {
    int idx = (inicio + i) % MAX_PERF;
    if (i > 0) json += ",";
    json += "{\"t\":" + String(perfHist[idx].tempo/1000) + ",";
    json += "\"heap\":" + String(perfHist[idx].heapLivre) + ",";
    json += "\"minheap\":" + String(perfHist[idx].minHeap) + "}";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

// =============================================
// ROTA: / — Dashboard visual redesenhado
// =============================================
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Rastreador Solar ESP32</title>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap');
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:'Inter',sans-serif;background:#060d1a;color:#e6edf3;min-height:100vh;padding:24px}
    /* Header */
    .header{text-align:center;margin-bottom:28px}
    .header h1{font-size:1.6rem;font-weight:700;background:linear-gradient(135deg,#58a6ff,#3fb950);-webkit-background-clip:text;-webkit-text-fill-color:transparent;margin-bottom:4px}
    .header p{color:#6b7280;font-size:.8rem}
    /* Nav */
    .nav{display:flex;justify-content:center;gap:10px;margin-bottom:28px;flex-wrap:wrap}
    .btn{background:#0d1f38;border:1px solid #1d3557;color:#93c5fd;padding:8px 20px;border-radius:20px;font-size:.8rem;text-decoration:none;transition:all .2s;font-weight:500}
    .btn:hover{background:#1d3557;border-color:#58a6ff;color:#fff}
    .btn.active{background:#1d3557;border-color:#3fb950;color:#3fb950}
    /* Status */
    .status-bar{display:flex;align-items:center;justify-content:center;gap:8px;margin-bottom:24px;font-size:.75rem;color:#6b7280;background:#0d1f38;border:1px solid #1d3557;border-radius:20px;padding:6px 16px;width:fit-content;margin-left:auto;margin-right:auto}
    #dot{width:8px;height:8px;border-radius:50%;background:#3fb950;animation:pulse 2s infinite}
    @keyframes pulse{0%,100%{opacity:1;box-shadow:0 0 0 0 #3fb95044}50%{opacity:.6;box-shadow:0 0 0 6px #3fb95000}}
    /* Cards topo */
    .top-cards{display:grid;grid-template-columns:1fr 1fr;gap:14px;max-width:420px;margin:0 auto 28px}
    .card{background:#0d1f38;border:1px solid #1d3557;border-radius:14px;padding:18px;text-align:center;transition:border-color .3s,transform .2s}
    .card:hover{border-color:#58a6ff;transform:translateY(-2px)}
    .card-icon{font-size:1.8rem;margin-bottom:8px}
    .card-label{font-size:.68rem;color:#6b7280;text-transform:uppercase;letter-spacing:.1em;margin-bottom:6px}
    .card-value{font-size:2.2rem;font-weight:700;line-height:1}
    .card-unit{font-size:.75rem;color:#6b7280;margin-top:4px}
    .card.temp .card-value{color:#ff7b72}
    .card.umid .card-value{color:#58a6ff}
    /* Painel solar + LDRs */
    .solar-section{max-width:520px;margin:0 auto 28px}
    .solar-title{text-align:center;font-size:.75rem;color:#6b7280;text-transform:uppercase;letter-spacing:.1em;margin-bottom:16px}
    .solar-wrapper{position:relative;display:flex;align-items:center;justify-content:center}
    /* SVG do painel */
    .painel-svg{width:240px;height:180px;filter:drop-shadow(0 0 20px #58a6ff33)}
    /* LDR corners */
    .ldr-grid{position:absolute;width:100%;height:100%;top:0;left:0;pointer-events:none}
    .ldr-corner{position:absolute;background:#0d1f38;border:1px solid #1d3557;border-radius:10px;padding:8px 10px;text-align:center;transition:all .4s;min-width:72px}
    .ldr-corner:hover{border-color:#fbbf24}
    .ldr-corner .ldr-label{font-size:.6rem;color:#6b7280;text-transform:uppercase;letter-spacing:.06em;margin-bottom:3px}
    .ldr-corner .ldr-val{font-size:1.1rem;font-weight:700;color:#fbbf24}
    .ldr-corner .ldr-bar{height:3px;border-radius:2px;background:#1d3557;margin-top:4px;overflow:hidden}
    .ldr-corner .ldr-fill{height:3px;border-radius:2px;background:#fbbf24;width:0%;transition:width .8s ease}
    .ldr-corner .ldr-icon{font-size:.9rem;margin-bottom:2px}
    .tl{top:0;left:0}
    .tr{top:0;right:0}
    .bl{bottom:0;left:0}
    .br{bottom:0;right:0}
    /* Erro e servo */
    .servo-info{display:flex;justify-content:center;gap:16px;margin-top:18px;flex-wrap:wrap}
    .servo-card{background:#0d1f38;border:1px solid #1d3557;border-radius:12px;padding:12px 20px;text-align:center;min-width:130px}
    .servo-card .s-label{font-size:.68rem;color:#6b7280;text-transform:uppercase;letter-spacing:.08em;margin-bottom:6px}
    .servo-card .s-val{font-size:1.5rem;font-weight:700;color:#ffa657}
    /* Barra servo */
    .servo-bar-bg{height:6px;background:#1d3557;border-radius:3px;margin-top:8px;overflow:hidden}
    .servo-bar-fill{height:6px;border-radius:3px;background:linear-gradient(90deg,#ffa657,#ff7b72);transition:width .8s ease}
    /* Footer */
    .footer{text-align:center;font-size:.7rem;color:#374151;margin-top:28px}
    .last-upd{text-align:center;font-size:.72rem;color:#6b7280;margin-top:16px}
  </style>
</head>
<body>
  <div class="header">
    <h1>☀️ Rastreador Solar — ESP32</h1>
    <p>Performance em Sistemas Ciberfísicos · PUCPR 2026</p>
  </div>

  <div class="nav">
    <a class="btn active" href="/">Dashboard</a>
    <a class="btn" href="/graficos">Gráficos 24h</a>
    <a class="btn" href="/performance">Performance</a>
  </div>

  <div class="status-bar">
    <div id="dot"></div>
    <span id="status-txt">Conectando...</span>
  </div>

  <!-- Temperatura e Umidade -->
  <div class="top-cards">
    <div class="card temp">
      <div class="card-icon">🌡️</div>
      <div class="card-label">Temperatura</div>
      <div class="card-value" id="temp">--</div>
      <div class="card-unit">°C</div>
    </div>
    <div class="card umid">
      <div class="card-icon">💧</div>
      <div class="card-label">Umidade</div>
      <div class="card-value" id="umid">--</div>
      <div class="card-unit">%</div>
    </div>
  </div>

  <!-- Painel Solar Ilustrado -->
  <div class="solar-section">
    <div class="solar-title">☀️ Painel Solar — Leituras LDR em Tempo Real</div>
    <div class="solar-wrapper" style="height:260px">

      <!-- LDR cantos -->
      <div class="ldr-corner tl" id="box-tl">
        <div class="ldr-icon">🔆</div>
        <div class="ldr-label">Sup. Esq.</div>
        <div class="ldr-val" id="v-tl">--</div>
        <div class="ldr-bar"><div class="ldr-fill" id="b-tl"></div></div>
      </div>

      <div class="ldr-corner tr" id="box-tr">
        <div class="ldr-icon">🔆</div>
        <div class="ldr-label">Sup. Dir.</div>
        <div class="ldr-val" id="v-tr">--</div>
        <div class="ldr-bar"><div class="ldr-fill" id="b-tr"></div></div>
      </div>

      <!-- SVG Painel Solar -->
      <svg class="painel-svg" viewBox="0 0 240 180" xmlns="http://www.w3.org/2000/svg">
        <defs>
          <linearGradient id="painelGrad" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" style="stop-color:#1a3a6e"/>
            <stop offset="100%" style="stop-color:#0d1f38"/>
          </linearGradient>
          <linearGradient id="cellGrad" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" style="stop-color:#1e4080;stop-opacity:.8"/>
            <stop offset="100%" style="stop-color:#0f2857;stop-opacity:.8"/>
          </linearGradient>
          <filter id="glow">
            <feGaussianBlur stdDeviation="3" result="blur"/>
            <feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge>
          </filter>
        </defs>
        <!-- Frame externo -->
        <rect x="4" y="4" width="232" height="172" rx="8" fill="#0a1628" stroke="#2a4a8a" stroke-width="2"/>
        <!-- Painel principal -->
        <rect x="10" y="10" width="220" height="160" rx="5" fill="url(#painelGrad)" stroke="#1d3a70" stroke-width="1"/>
        <!-- Grade de células solares 3x4 -->
        <g fill="url(#cellGrad)" stroke="#2a5090" stroke-width="0.5">
          <rect x="14" y="14" width="50" height="36" rx="2"/>
          <rect x="70" y="14" width="50" height="36" rx="2"/>
          <rect x="126" y="14" width="50" height="36" rx="2"/>
          <rect x="182" y="14" width="44" height="36" rx="2"/>
          <rect x="14" y="56" width="50" height="36" rx="2"/>
          <rect x="70" y="56" width="50" height="36" rx="2"/>
          <rect x="126" y="56" width="50" height="36" rx="2"/>
          <rect x="182" y="56" width="44" height="36" rx="2"/>
          <rect x="14" y="98" width="50" height="36" rx="2"/>
          <rect x="70" y="98" width="50" height="36" rx="2"/>
          <rect x="126" y="98" width="50" height="36" rx="2"/>
          <rect x="182" y="98" width="44" height="36" rx="2"/>
          <rect x="14" y="140" width="50" height="26" rx="2"/>
          <rect x="70" y="140" width="50" height="26" rx="2"/>
          <rect x="126" y="140" width="50" height="26" rx="2"/>
          <rect x="182" y="140" width="44" height="26" rx="2"/>
        </g>
        <!-- Linhas de interconexão -->
        <g stroke="#3a6ab0" stroke-width="0.8" opacity="0.6">
          <line x1="64" y1="14" x2="64" y2="166"/>
          <line x1="120" y1="14" x2="120" y2="166"/>
          <line x1="176" y1="14" x2="176" y2="166"/>
          <line x1="14" y1="52" x2="226" y2="52"/>
          <line x1="14" y1="94" x2="226" y2="94"/>
          <line x1="14" y1="136" x2="226" y2="136"/>
        </g>
        <!-- Reflexo -->
        <rect x="10" y="10" width="220" height="60" rx="5" fill="white" opacity="0.04"/>
        <!-- Borda brilhante -->
        <rect x="4" y="4" width="232" height="172" rx="8" fill="none" stroke="#4a7acc" stroke-width="1" opacity="0.5" filter="url(#glow)"/>
        <!-- Ângulo do servo (indicador visual) -->
        <circle cx="120" cy="90" r="18" fill="none" stroke="#ffa657" stroke-width="1.5" stroke-dasharray="4,3" opacity="0.6"/>
        <circle cx="120" cy="90" r="4" fill="#ffa657" opacity="0.8"/>
        <text x="120" y="115" text-anchor="middle" font-size="9" fill="#ffa657" font-family="Inter,sans-serif" opacity="0.9" id="svg-angulo">90°</text>
      </svg>

      <div class="ldr-corner bl" id="box-bl">
        <div class="ldr-icon">🔆</div>
        <div class="ldr-label">Inf. Esq.</div>
        <div class="ldr-val" id="v-bl">--</div>
        <div class="ldr-bar"><div class="ldr-fill" id="b-bl"></div></div>
      </div>

      <div class="ldr-corner br" id="box-br">
        <div class="ldr-icon">🔆</div>
        <div class="ldr-label">Inf. Dir.</div>
        <div class="ldr-val" id="v-br">--</div>
        <div class="ldr-bar"><div class="ldr-fill" id="b-br"></div></div>
      </div>
    </div>

    <!-- Servos -->
    <div class="servo-info">
      <div class="servo-card">
        <div class="s-label">↕️ Servo Vertical</div>
        <div class="s-val" id="sv">90°</div>
        <div class="servo-bar-bg">
          <div class="servo-bar-fill" id="sv-bar" style="width:50%"></div>
        </div>
      </div>
      <div class="servo-card">
        <div class="s-label">↔️ Servo Horizontal</div>
        <div class="s-val" id="sh">90°</div>
        <div class="servo-bar-bg">
          <div class="servo-bar-fill" id="sh-bar" style="width:50%"></div>
        </div>
      </div>
    </div>
  </div>

  <div class="last-upd" id="last-upd">Aguardando leitura...</div>
  <div class="footer">Rastreador Solar · ESP32 · PUCPR 2026</div>

  <script>
    const MAX_ADC = 4095;
    function ldrColor(val) {
      const pct = val / MAX_ADC;
      if (pct > 0.7) return '#fbbf24';
      if (pct > 0.4) return '#fb923c';
      return '#6b7280';
    }
    function atualizar() {
      fetch('/dados').then(r=>r.json()).then(d=>{
        document.getElementById('temp').innerText = d.temperatura.toFixed(1);
        document.getElementById('umid').innerText = d.umidade.toFixed(1);
        document.getElementById('sv').innerText   = d.servoV + '°';
        document.getElementById('sh').innerText   = d.servoH + '°';
        document.getElementById('sv-bar').style.width = (d.servoV/180*100)+'%';
        document.getElementById('sh-bar').style.width = (d.servoH/180*100)+'%';
        document.getElementById('svg-angulo').textContent = d.servoV + '°';

        const ldrs = {tl:d.ldrTL, tr:d.ldrTR, bl:d.ldrBL, br:d.ldrBR};
        Object.entries(ldrs).forEach(([k,v])=>{
          document.getElementById('v-'+k).innerText = v;
          document.getElementById('b-'+k).style.width = (v/MAX_ADC*100).toFixed(1)+'%';
          document.getElementById('v-'+k).style.color = ldrColor(v);
        });

        document.getElementById('dot').style.background = '#3fb950';
        document.getElementById('status-txt').innerText = 'Conectado ao ESP32 · ' + WiFi;
        document.getElementById('last-upd').innerText =
          'Última atualização: ' + new Date().toLocaleTimeString('pt-BR');
      }).catch(()=>{
        document.getElementById('dot').style.background = '#ff7b72';
        document.getElementById('status-txt').innerText = 'Sem conexão — reconectando...';
      });
    }
    atualizar();
    setInterval(atualizar, 2000);
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// =============================================
// ROTA: /graficos
// =============================================
void handleGraficos() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Gráficos 24h</title>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600&display=swap');
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:'Inter',sans-serif;background:#060d1a;color:#e6edf3;padding:24px}
    h1{text-align:center;font-size:1.4rem;background:linear-gradient(135deg,#58a6ff,#3fb950);-webkit-background-clip:text;-webkit-text-fill-color:transparent;margin-bottom:4px}
    .sub{text-align:center;color:#6b7280;font-size:.78rem;margin-bottom:18px}
    .nav{display:flex;justify-content:center;gap:10px;margin-bottom:22px}
    .btn{background:#0d1f38;border:1px solid #1d3557;color:#93c5fd;padding:8px 20px;border-radius:20px;font-size:.8rem;text-decoration:none;cursor:pointer;transition:all .2s}
    .btn:hover{background:#1d3557;border-color:#58a6ff}
    .info{display:flex;justify-content:center;gap:12px;flex-wrap:wrap;margin-bottom:20px}
    .info span{background:#0d1f38;border:1px solid #1d3557;padding:5px 14px;border-radius:20px;font-size:.75rem}
    .info b{color:#e6edf3}
    .charts{display:flex;flex-direction:column;gap:20px;max-width:900px;margin:0 auto}
    .cc{background:#0d1f38;border:1px solid #1d3557;border-radius:14px;padding:20px}
    .ct{font-size:.78rem;color:#6b7280;text-transform:uppercase;letter-spacing:.1em;margin-bottom:14px;display:flex;align-items:center;gap:8px}
    .dot{width:10px;height:10px;border-radius:50%}
    canvas{max-height:200px}
    #load{text-align:center;padding:60px;color:#6b7280}
  </style>
</head>
<body>
  <h1>📈 Histórico de 24 Horas</h1>
  <p class="sub">Performance em Sistemas Ciberfísicos · PUCPR 2026</p>
  <div class="nav">
    <a class="btn" href="/">Dashboard</a>
    <a class="btn" href="/performance">Performance</a>
    <button class="btn" onclick="load()">↻ Atualizar</button>
  </div>
  <div class="info" id="info"><span>Carregando...</span></div>
  <div id="load">⏳ Buscando dados...</div>
  <div class="charts" id="charts" style="display:none">
    <div class="cc"><div class="ct"><div class="dot" style="background:#ff7b72"></div>Temperatura (°C)</div><canvas id="cT"></canvas></div>
    <div class="cc"><div class="ct"><div class="dot" style="background:#58a6ff"></div>Umidade (%)</div><canvas id="cU"></canvas></div>
    <div class="cc"><div class="ct"><div class="dot" style="background:#fbbf24"></div>Luminosidade LDR média</div><canvas id="cL"></canvas></div>
    <div class="cc"><div class="ct"><div class="dot" style="background:#ffa657"></div>Posição dos Servos (°)</div><canvas id="cS"></canvas></div>
  </div>
  <script>
    let C={};
    const O=(cor,lbl,mn,mx)=>({responsive:true,animation:{duration:500},plugins:{legend:{display:false},tooltip:{backgroundColor:'#0d1f38',borderColor:'#1d3557',borderWidth:1,titleColor:'#6b7280',bodyColor:'#e6edf3',callbacks:{label:c=>lbl+': '+c.parsed.y}}},scales:{x:{ticks:{color:'#6b7280',maxTicksLimit:10,font:{size:9}},grid:{color:'#1d3557'}},y:{min:mn,max:mx,ticks:{color:'#6b7280',font:{size:9}},grid:{color:'#1d3557'}}}});
    function mk(id,labels,data,cor,lbl,mn,mx){
      if(C[id])C[id].destroy();
      const ctx=document.getElementById(id).getContext('2d');
      const g=ctx.createLinearGradient(0,0,0,200);g.addColorStop(0,cor+'44');g.addColorStop(1,cor+'00');
      C[id]=new Chart(ctx,{type:'line',data:{labels,datasets:[{data,borderColor:cor,backgroundColor:g,borderWidth:2,pointRadius:data.length>60?0:3,tension:0.35,fill:true}]},options:O(cor,lbl,mn,mx)});
    }
    function fmt(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+'h'+String(m).padStart(2,'0')}
    function load(){
      document.getElementById('load').style.display='block';
      document.getElementById('charts').style.display='none';
      fetch('/historico').then(r=>r.json()).then(d=>{
        document.getElementById('load').style.display='none';
        if(!d.dados||!d.dados.length){document.getElementById('load').style.display='block';document.getElementById('load').innerText='⏳ Sem dados ainda. Aguarde 5 minutos.';return;}
        document.getElementById('charts').style.display='flex';
        const lb=d.dados.map(r=>fmt(r.t)),T=d.dados.map(r=>r.temp),U=d.dados.map(r=>r.umid),L=d.dados.map(r=>r.ldr),SH=d.dados.map(r=>r.sh),SV=d.dados.map(r=>r.sv);
        const cob=Math.round(d.dados.length*5/60*10)/10;
        document.getElementById('info').innerHTML=`<span>📊 <b>${d.dados.length}</b> registros (${cob}h)</span><span>🌡️ Temp: <b>${Math.min(...T).toFixed(1)}–${Math.max(...T).toFixed(1)}°C</b></span><span>💧 Umid: <b>${Math.min(...U).toFixed(1)}–${Math.max(...U).toFixed(1)}%</b></span>`;
        mk('cT',lb,T,'#ff7b72','°C',Math.floor(Math.min(...T))-2,Math.ceil(Math.max(...T))+2);
        mk('cU',lb,U,'#58a6ff','%',Math.floor(Math.min(...U))-5,Math.ceil(Math.max(...U))+5);
        mk('cL',lb,L,'#fbbf24','ADC',Math.max(0,Math.min(...L)-200),Math.min(4095,Math.max(...L)+200));
        if(C['cS'])C['cS'].destroy();
        const ctx=document.getElementById('cS').getContext('2d');
        const gH=ctx.createLinearGradient(0,0,0,200);gH.addColorStop(0,'#ffa65744');gH.addColorStop(1,'#ffa65700');
        const gV=ctx.createLinearGradient(0,0,0,200);gV.addColorStop(0,'#d2a8ff44');gV.addColorStop(1,'#d2a8ff00');
        C['cS']=new Chart(ctx,{type:'line',data:{labels:lb,datasets:[{label:'Pan (H)',data:SH,borderColor:'#ffa657',backgroundColor:gH,borderWidth:2,pointRadius:d.dados.length>60?0:3,tension:0.35,fill:true},{label:'Tilt (V)',data:SV,borderColor:'#d2a8ff',backgroundColor:gV,borderWidth:2,pointRadius:d.dados.length>60?0:3,tension:0.35,fill:true}]},options:{responsive:true,animation:{duration:500},plugins:{legend:{display:true,labels:{color:'#6b7280',boxWidth:10,font:{size:10}}},tooltip:{backgroundColor:'#0d1f38',borderColor:'#1d3557',borderWidth:1,titleColor:'#6b7280',bodyColor:'#e6edf3'}},scales:{x:{ticks:{color:'#6b7280',maxTicksLimit:10,font:{size:9}},grid:{color:'#1d3557'}},y:{min:0,max:180,ticks:{color:'#6b7280',font:{size:9}},grid:{color:'#1d3557'}}}}});
      }).catch(()=>{document.getElementById('load').innerText='❌ Erro ao carregar. Verifique a conexão.';});
    }
    load();
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// =============================================
// ROTA: /performance — Guia completa
// =============================================
void handlePerformance() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Performance — ESP32</title>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap');
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:'Inter',sans-serif;background:#060d1a;color:#e6edf3;padding:24px}
    h1{text-align:center;font-size:1.4rem;background:linear-gradient(135deg,#58a6ff,#3fb950);-webkit-background-clip:text;-webkit-text-fill-color:transparent;margin-bottom:4px}
    .sub{text-align:center;color:#6b7280;font-size:.78rem;margin-bottom:18px}
    .nav{display:flex;justify-content:center;gap:10px;margin-bottom:24px;flex-wrap:wrap}
    .btn{background:#0d1f38;border:1px solid #1d3557;color:#93c5fd;padding:8px 20px;border-radius:20px;font-size:.8rem;text-decoration:none;cursor:pointer;transition:all .2s}
    .btn:hover{background:#1d3557}
    .btn.atv{border-color:#3fb950;color:#3fb950}
    /* Status online */
    .uptime-bar{display:flex;align-items:center;justify-content:center;gap:10px;margin-bottom:24px;flex-wrap:wrap}
    .uptime-chip{background:#0d1f38;border:1px solid #1d3557;border-radius:20px;padding:6px 16px;font-size:.75rem;color:#6b7280}
    .uptime-chip b{color:#e6edf3}
    .live-dot{width:8px;height:8px;border-radius:50%;background:#3fb950;animation:pulse 2s infinite;display:inline-block;margin-right:4px}
    @keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
    /* Grid de seções */
    .grid-2{display:grid;grid-template-columns:1fr 1fr;gap:16px;max-width:960px;margin:0 auto 20px}
    .grid-1{max-width:960px;margin:0 auto 20px}
    @media(max-width:640px){.grid-2{grid-template-columns:1fr}}
    /* Cards de seção */
    .sec{background:#0d1f38;border:1px solid #1d3557;border-radius:14px;padding:20px}
    .sec-title{font-size:.72rem;color:#6b7280;text-transform:uppercase;letter-spacing:.12em;margin-bottom:16px;display:flex;align-items:center;gap:8px}
    .sec-title .icon{font-size:1rem}
    /* Gauge / anel */
    .gauge-wrap{display:flex;flex-direction:column;align-items:center;gap:8px}
    .gauge-row{display:flex;justify-content:space-around;flex-wrap:wrap;gap:12px}
    .gauge{position:relative;width:80px;height:80px}
    .gauge svg{transform:rotate(-90deg)}
    .gauge-label{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);text-align:center}
    .gauge-val{font-size:.9rem;font-weight:700;color:#e6edf3}
    .gauge-sub{font-size:.55rem;color:#6b7280;margin-top:1px}
    .gauge-name{font-size:.65rem;color:#6b7280;text-align:center;margin-top:4px}
    /* Métricas lineares */
    .metric{display:flex;align-items:center;justify-content:space-between;padding:8px 0;border-bottom:1px solid #1d3557}
    .metric:last-child{border-bottom:none}
    .metric-label{font-size:.78rem;color:#6b7280}
    .metric-value{font-size:.85rem;font-weight:600;color:#e6edf3;font-family:'Courier New',monospace}
    .metric-value.green{color:#3fb950}
    .metric-value.yellow{color:#fbbf24}
    .metric-value.red{color:#ff7b72}
    .metric-value.blue{color:#58a6ff}
    /* Barra de progresso inline */
    .mbar{height:4px;background:#1d3557;border-radius:2px;margin-top:4px;overflow:hidden;width:100%}
    .mbar-fill{height:4px;border-radius:2px;transition:width .8s ease}
    /* Tabela de funções */
    .fn-table{width:100%;border-collapse:collapse;font-size:.78rem}
    .fn-table th{text-align:left;padding:8px 10px;color:#6b7280;font-weight:500;border-bottom:1px solid #1d3557;font-size:.68rem;text-transform:uppercase;letter-spacing:.08em}
    .fn-table td{padding:8px 10px;border-bottom:1px solid #0d1f38;color:#e6edf3}
    .fn-table tr:last-child td{border-bottom:none}
    .fn-table tr:hover td{background:#1d3557}
    .badge{display:inline-block;padding:2px 8px;border-radius:10px;font-size:.68rem;font-weight:600}
    .badge.fast{background:#1a3a22;color:#3fb950}
    .badge.med{background:#2d2a0a;color:#fbbf24}
    .badge.slow{background:#2d0f0f;color:#ff7b72}
    /* Tasks table */
    .tasks-table{width:100%;border-collapse:collapse;font-size:.75rem}
    .tasks-table th{padding:7px 10px;color:#6b7280;font-weight:500;border-bottom:1px solid #1d3557;font-size:.65rem;text-transform:uppercase;letter-spacing:.08em;text-align:left}
    .tasks-table td{padding:7px 10px;border-bottom:1px solid #0d1f38;color:#e6edf3;font-family:'Courier New',monospace;font-size:.72rem}
    /* Wi-Fi */
    .wifi-icon{font-size:2rem;text-align:center;margin-bottom:8px}
    .signal-bars{display:flex;align-items:flex-end;gap:3px;justify-content:center;margin:8px 0}
    .bar{width:8px;border-radius:2px;background:#1d3557;transition:background .4s}
    .bar.on{background:#3fb950}
    /* Gráfico heap */
    canvas{max-height:160px}
  </style>
</head>
<body>
  <h1>⚡ Monitor de Performance</h1>
  <p class="sub">Performance em Sistemas Ciberfísicos · PUCPR 2026</p>

  <div class="nav">
    <a class="btn" href="/">Dashboard</a>
    <a class="btn" href="/graficos">Gráficos 24h</a>
    <a class="btn atv" href="/performance">Performance</a>
    <button class="btn" onclick="atualizar()">↻ Atualizar</button>
  </div>

  <div class="uptime-bar" id="uptime-bar">
    <div class="uptime-chip"><span class="live-dot"></span>Carregando...</div>
  </div>

  <!-- CPU + Heap (gauges) -->
  <div class="grid-2">
    <div class="sec">
      <div class="sec-title"><span class="icon">🖥️</span>CPU e Sistema</div>
      <div class="gauge-row" id="gauges-cpu">
        <div>
          <div class="gauge">
            <svg width="80" height="80" viewBox="0 0 80 80">
              <circle cx="40" cy="40" r="32" fill="none" stroke="#1d3557" stroke-width="8"/>
              <circle id="g-heap" cx="40" cy="40" r="32" fill="none" stroke="#58a6ff" stroke-width="8" stroke-dasharray="201" stroke-dashoffset="201" stroke-linecap="round"/>
            </svg>
            <div class="gauge-label">
              <div class="gauge-val" id="pct-heap">--%</div>
              <div class="gauge-sub">Heap</div>
            </div>
          </div>
          <div class="gauge-name">Uso de Heap</div>
        </div>
        <div>
          <div class="gauge">
            <svg width="80" height="80" viewBox="0 0 80 80">
              <circle cx="40" cy="40" r="32" fill="none" stroke="#1d3557" stroke-width="8"/>
              <circle id="g-flash" cx="40" cy="40" r="32" fill="none" stroke="#ffa657" stroke-width="8" stroke-dasharray="201" stroke-dashoffset="201" stroke-linecap="round"/>
            </svg>
            <div class="gauge-label">
              <div class="gauge-val" id="pct-flash">--%</div>
              <div class="gauge-sub">Flash</div>
            </div>
          </div>
          <div class="gauge-name">Uso de Flash</div>
        </div>
        <div>
          <div class="gauge">
            <svg width="80" height="80" viewBox="0 0 80 80">
              <circle cx="40" cy="40" r="32" fill="none" stroke="#1d3557" stroke-width="8"/>
              <circle id="g-psram" cx="40" cy="40" r="32" fill="none" stroke="#d2a8ff" stroke-width="8" stroke-dasharray="201" stroke-dashoffset="201" stroke-linecap="round"/>
            </svg>
            <div class="gauge-label">
              <div class="gauge-val" id="pct-psram">N/A</div>
              <div class="gauge-sub">PSRAM</div>
            </div>
          </div>
          <div class="gauge-name">PSRAM</div>
        </div>
      </div>
      <div style="margin-top:16px">
        <div class="metric"><span class="metric-label">Frequência CPU</span><span class="metric-value green" id="cpu-freq">--</span></div>
        <div class="metric"><span class="metric-label">Tasks FreeRTOS</span><span class="metric-value blue" id="num-tasks">--</span></div>
        <div class="metric"><span class="metric-label">Heap Total</span><span class="metric-value" id="heap-total">--</span></div>
        <div class="metric"><span class="metric-label">Heap Livre</span><span class="metric-value green" id="heap-livre">--</span></div>
        <div class="metric"><span class="metric-label">Heap Mínimo Histórico</span><span class="metric-value yellow" id="heap-min">--</span></div>
        <div class="metric"><span class="metric-label">Flash Total</span><span class="metric-value" id="flash-total">--</span></div>
        <div class="metric"><span class="metric-label">Sketch Usado</span><span class="metric-value" id="flash-usado">--</span></div>
        <div class="metric"><span class="metric-label">Sketch Livre</span><span class="metric-value green" id="flash-livre">--</span></div>
        <div class="metric"><span class="metric-label">PSRAM Total</span><span class="metric-value" id="psram-total">--</span></div>
        <div class="metric"><span class="metric-label">PSRAM Livre</span><span class="metric-value" id="psram-livre">--</span></div>
      </div>
    </div>

    <div class="sec">
      <div class="sec-title"><span class="icon">📡</span>Wi-Fi e Rede</div>
      <div class="wifi-icon" id="wifi-icon">📡</div>
      <div class="signal-bars" id="signal-bars">
        <div class="bar" id="b1" style="height:12px"></div>
        <div class="bar" id="b2" style="height:20px"></div>
        <div class="bar" id="b3" style="height:28px"></div>
        <div class="bar" id="b4" style="height:36px"></div>
      </div>
      <div class="metric"><span class="metric-label">Status</span><span class="metric-value green" id="wifi-status">--</span></div>
      <div class="metric"><span class="metric-label">SSID</span><span class="metric-value blue" id="wifi-ssid">--</span></div>
      <div class="metric"><span class="metric-label">Endereço IP</span><span class="metric-value" id="wifi-ip">--</span></div>
      <div class="metric"><span class="metric-label">Canal</span><span class="metric-value" id="wifi-canal">--</span></div>
      <div class="metric"><span class="metric-label">RSSI (Sinal)</span><span class="metric-value" id="wifi-rssi">--</span></div>
      <div style="margin-top:14px">
        <div class="sec-title" style="margin-bottom:10px"><span class="icon">📟</span>Sensores Atuais</div>
        <div class="metric"><span class="metric-label">Temperatura</span><span class="metric-value" style="color:#ff7b72" id="s-temp">--</span></div>
        <div class="metric"><span class="metric-label">Umidade</span><span class="metric-value" style="color:#58a6ff" id="s-umid">--</span></div>
        <div class="metric"><span class="metric-label">LDR TL / TR</span><span class="metric-value" id="s-ldr1">--</span></div>
        <div class="metric"><span class="metric-label">LDR BL / BR</span><span class="metric-value" id="s-ldr2">--</span></div>
        <div class="metric"><span class="metric-label">Servo V / H</span><span class="metric-value" style="color:#ffa657" id="s-servo">--</span></div>
      </div>
    </div>
  </div>

  <!-- Tempos de execução -->
  <div class="grid-1">
    <div class="sec">
      <div class="sec-title"><span class="icon">⏱️</span>Tempo de Execução das Funções</div>
      <table class="fn-table">
        <thead>
          <tr>
            <th>Função</th>
            <th>Descrição</th>
            <th>Tempo (µs)</th>
            <th>Tempo (ms)</th>
            <th>Classificação</th>
          </tr>
        </thead>
        <tbody id="fn-body">
          <tr><td colspan="5" style="text-align:center;color:#6b7280;padding:20px">Carregando...</td></tr>
        </tbody>
      </table>
    </div>
  </div>

  <!-- Tasks FreeRTOS -->
  <div class="grid-1">
    <div class="sec">
      <div class="sec-title"><span class="icon">🧵</span>Tasks / Threads FreeRTOS</div>
      <table class="tasks-table">
        <thead>
          <tr>
            <th>Task</th>
            <th>Responsabilidade</th>
            <th>Núcleo</th>
            <th>Prioridade</th>
            <th>Estado</th>
          </tr>
        </thead>
        <tbody id="tasks-body">
          <tr><td colspan="5" style="text-align:center;color:#6b7280;padding:20px">Carregando...</td></tr>
        </tbody>
      </table>
      <p style="font-size:.68rem;color:#6b7280;margin-top:10px">* O ESP32 com Arduino IDE roda o código no loop() como uma task do FreeRTOS. Tasks internas do sistema também estão listadas.</p>
    </div>
  </div>

  <!-- Gráficos de série temporal -->
  <div class="grid-2">
    <div class="sec">
      <div class="sec-title"><span class="icon">📊</span>Heap Livre ao Longo do Tempo</div>
      <canvas id="chartHeap"></canvas>
    </div>
    <div class="sec">
      <div class="sec-title"><span class="icon">📊</span>Heap Mínimo Histórico</div>
      <canvas id="chartMinHeap"></canvas>
    </div>
  </div>

  <script>
    let chartHeap, chartMinHeap;

    function kb(b){ return (b/1024).toFixed(1)+' KB'; }
    function mb(b){ return (b/1024/1024).toFixed(2)+' MB'; }

    function setGauge(id, pct, color){
      const circ = document.getElementById(id);
      const r = 32, total = 2*Math.PI*r;
      const offset = total - (pct/100)*total;
      circ.style.strokeDashoffset = offset;
      circ.style.stroke = color;
    }

    function rssiColor(r){
      if(r > -50) return '#3fb950';
      if(r > -70) return '#fbbf24';
      return '#ff7b72';
    }

    function rssiLabel(r){
      if(r > -50) return 'Excelente';
      if(r > -65) return 'Bom';
      if(r > -75) return 'Regular';
      return 'Fraco';
    }

    function fnBadge(us){
      if(us < 1000)  return '<span class="badge fast">Rápido</span>';
      if(us < 10000) return '<span class="badge med">Médio</span>';
      return '<span class="badge slow">Lento</span>';
    }

    function fmt(s){
      const h=Math.floor(s/3600),m=Math.floor(s%3600/60),sc=s%60;
      return (h?h+'h ':'')+(m?m+'m ':'')+sc+'s';
    }

    function mkChart(canvasId, labels, data, cor, label){
      const ctx = document.getElementById(canvasId).getContext('2d');
      const g = ctx.createLinearGradient(0,0,0,160);
      g.addColorStop(0, cor+'55'); g.addColorStop(1, cor+'00');
      return new Chart(ctx,{type:'line',data:{labels,datasets:[{data,borderColor:cor,backgroundColor:g,borderWidth:2,pointRadius:data.length>30?0:3,tension:0.3,fill:true,label}]},options:{responsive:true,animation:{duration:400},plugins:{legend:{display:false},tooltip:{backgroundColor:'#0d1f38',borderColor:'#1d3557',borderWidth:1,titleColor:'#6b7280',bodyColor:'#e6edf3'}},scales:{x:{ticks:{color:'#6b7280',maxTicksLimit:8,font:{size:8}},grid:{color:'#1d3557'}},y:{ticks:{color:'#6b7280',font:{size:8}},grid:{color:'#1d3557'}}}}});
    }

    function atualizar(){
      fetch('/perfdata').then(r=>r.json()).then(d=>{

        // Uptime
        document.getElementById('uptime-bar').innerHTML =
          `<div class="uptime-chip"><span class="live-dot"></span>Online há <b>${fmt(d.uptime_s)}</b></div>` +
          `<div class="uptime-chip">CPU <b>${d.cpu_freq_mhz} MHz</b></div>` +
          `<div class="uptime-chip">Tasks: <b>${d.num_tasks}</b></div>` +
          `<div class="uptime-chip">Atualizado: <b>${new Date().toLocaleTimeString('pt-BR')}</b></div>`;

        // Gauges
        setGauge('g-heap',  d.heap.pct_usado,  '#58a6ff');
        setGauge('g-flash', Math.round(d.flash.sketch_usado/d.flash.total*100), '#ffa657');
        const psramPct = d.psram.total > 0 ? Math.round((d.psram.total-d.psram.livre)/d.psram.total*100) : 0;
        setGauge('g-psram', psramPct, '#d2a8ff');

        document.getElementById('pct-heap').innerText  = d.heap.pct_usado+'%';
        document.getElementById('pct-flash').innerText = Math.round(d.flash.sketch_usado/d.flash.total*100)+'%';
        document.getElementById('pct-psram').innerText = d.psram.total > 0 ? psramPct+'%' : 'N/A';

        // Métricas CPU
        document.getElementById('cpu-freq').innerText   = d.cpu_freq_mhz+' MHz';
        document.getElementById('num-tasks').innerText  = d.num_tasks+' tasks';
        document.getElementById('heap-total').innerText = kb(d.heap.total);
        document.getElementById('heap-livre').innerText = kb(d.heap.livre);
        document.getElementById('heap-min').innerText   = kb(d.heap.minimo);
        document.getElementById('flash-total').innerText = mb(d.flash.total);
        document.getElementById('flash-usado').innerText = kb(d.flash.sketch_usado);
        document.getElementById('flash-livre').innerText = kb(d.flash.sketch_livre);
        document.getElementById('psram-total').innerText = d.psram.total > 0 ? kb(d.psram.total) : 'Não disponível';
        document.getElementById('psram-livre').innerText = d.psram.total > 0 ? kb(d.psram.livre) : 'Não disponível';

        // Wi-Fi
        const rssi = d.wifi.rssi;
        document.getElementById('wifi-status').innerText = d.wifi.status;
        document.getElementById('wifi-ssid').innerText   = d.wifi.ssid;
        document.getElementById('wifi-ip').innerText     = d.wifi.ip;
        document.getElementById('wifi-canal').innerText  = 'Canal '+d.wifi.canal;
        document.getElementById('wifi-rssi').innerText   = rssi+' dBm ('+rssiLabel(rssi)+')';
        document.getElementById('wifi-rssi').style.color = rssiColor(rssi);

        // Barras de sinal
        const bars = [document.getElementById('b1'),document.getElementById('b2'),
                      document.getElementById('b3'),document.getElementById('b4')];
        const strength = rssi > -50 ? 4 : rssi > -65 ? 3 : rssi > -75 ? 2 : 1;
        bars.forEach((b,i)=>{ b.classList.toggle('on', i < strength); });

        // Sensores
        document.getElementById('s-temp').innerText  = d.sensores.temperatura.toFixed(1)+' °C';
        document.getElementById('s-umid').innerText  = d.sensores.umidade.toFixed(1)+' %';
        document.getElementById('s-ldr1').innerText  = d.sensores.ldrTL+' / '+d.sensores.ldrTR;
        document.getElementById('s-ldr2').innerText  = d.sensores.ldrBL+' / '+d.sensores.ldrBR;
        document.getElementById('s-servo').innerText = 'V:'+d.sensores.servoV+'° H:'+d.sensores.servoH+'°';

        // Tabela de funções
        const fns = [
          ['rastrearSol()',   'Lê LDRs, calcula erro P, move servo via lerp', d.tempos_us.rastrearSol],
          ['lerDHT()',        'Lê temperatura e umidade do DHT11',            d.tempos_us.lerDHT],
          ['atualizarOLED()','Renderiza dados no display OLED I2C',          d.tempos_us.atualizarOLED],
          ['salvarHistorico()','Grava registro no buffer circular de 24h',   d.tempos_us.salvarHistorico],
          ['server.handleClient()','Processa requisições HTTP do servidor',  d.tempos_us.webServer],
        ];
        document.getElementById('fn-body').innerHTML = fns.map(([fn,desc,us])=>`
          <tr>
            <td style="color:#3fb950;font-family:'Courier New',monospace">${fn}</td>
            <td style="color:#9ca3af">${desc}</td>
            <td>${us} µs</td>
            <td>${(us/1000).toFixed(3)} ms</td>
            <td>${fnBadge(us)}</td>
          </tr>`).join('');

        // Tabela de tasks FreeRTOS
        const tasks = [
          ['loopTask',    'Loop principal: sensores, servos, servidor', '1', '1', '🟢 Running'],
          ['IDLE0',       'Idle task núcleo 0 (economia de energia)',   '0', '0', '🟡 Idle'],
          ['IDLE1',       'Idle task núcleo 1 (economia de energia)',   '1', '0', '🟡 Idle'],
          ['wifi',        'Stack Wi-Fi e TCP/IP (LwIP)',                '0', '23','🟢 Running'],
          ['tiT',         'Timer interno FreeRTOS',                     '0', '22','🟢 Running'],
          ['ipc0',        'Inter-processor call núcleo 0',              '0', '24','🟡 Waiting'],
          ['ipc1',        'Inter-processor call núcleo 1',              '1', '24','🟡 Waiting'],
        ];
        document.getElementById('tasks-body').innerHTML = tasks.map(([n,d,c,p,s])=>`
          <tr>
            <td style="color:#58a6ff">${n}</td>
            <td style="color:#9ca3af;font-family:Inter,sans-serif">${d}</td>
            <td>${c}</td><td>${p}</td><td>${s}</td>
          </tr>`).join('');

        // Gráficos de heap
        if(d.historico_perf && d.historico_perf.length > 0){
          const labels = d.historico_perf.map(r=>{ const s=r.t; const m=Math.floor(s%3600/60); return Math.floor(s/3600)+'h'+String(m).padStart(2,'0'); });
          const heapData    = d.historico_perf.map(r=>Math.round(r.heap/1024));
          const minHeapData = d.historico_perf.map(r=>Math.round(r.minheap/1024));

          if(chartHeap)    chartHeap.destroy();
          if(chartMinHeap) chartMinHeap.destroy();
          chartHeap    = mkChart('chartHeap',    labels, heapData,    '#58a6ff', 'Heap livre (KB)');
          chartMinHeap = mkChart('chartMinHeap', labels, minHeapData, '#fbbf24', 'Heap mín (KB)');
        }

      }).catch(e=>{ console.error(e); });
    }

    atualizar();
    setInterval(atualizar, 5000);
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}
