#include <WiFi.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>  
#include <WebServer.h>
#include <LittleFS.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>

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
void sincronizarRelogio();
void servirArquivo(const char* caminho, const char* tipo);
void handleDados();
void handleHistorico();
void handlePerfData();
void handleLogsData();
void handleLogsClear();
void handleConfigData();
void handleConfigSave();
void handleRastreamento();
void atualizarLed();
void taskSensores(void* pv);
void taskManutencao(void* pv);
void gerenciarEnergia();
void avisarDeepSleep();
void persistirHistorico();
void restaurarHistorico();
unsigned long tempoSistema();
uint32_t epochAtual();

// wifi
String ssid     = "nome";
String password = "senha";

// pinos
#define LDR_TL  32
#define LDR_TR  33
#define LDR_BL  34
#define LDR_BR  35
#define DHT_PIN  4
#define DHT_TYPE DHT11
#define SERVO_H  27
#define SERVO_V  12
#define BOTAO_PIN  0   
#define BUZZER_PIN 19  
#define LED_RASTR  2   

// níveis de log
#define LOG_INFO 0
#define LOG_WARN 1
#define LOG_ERRO 2

// objetos
WebServer server(80);
DHT dht(DHT_PIN, DHT_TYPE);
Servo servoH, servoV;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Preferences prefs;
Preferences prefsHist;

// autenticação
enum Perfil : uint8_t { PERFIL_NENHUM = 0, PERFIL_USER = 1, PERFIL_ADMIN = 2 };
struct Sessao { char token[17]; Perfil perfil; unsigned long criadaEm; };
const uint8_t       MAX_SESSOES    = 4;
const unsigned long SESSAO_DURACAO = 8UL * 3600UL * 1000UL;  // 8 h
Sessao sessoes[MAX_SESSOES] = {};

String senhaUser = "solar123";   // senha perfil visitante 
String senhaAdm  = "admin2026";  // senha perfil administrador 
uint8_t       loginFalhas      = 0;
unsigned long loginBloqueioAte = 0;
uint32_t      bootCount        = 0;

// leituras atuais
float posH = 0.0,  posV = 90.0;   
float alvoH = 90.0, alvoV = 90.0;
float temperatura = 0.0, umidade = 0.0;
int   ldrTL, ldrTR, ldrBL, ldrBR;

// parâmetros operacionais
float kp         = 0.008;
float zonaMorta  = 300;
float passoMax   = 6.0;
float suavizacao = 0.12;

// intervalos
long intervaloLDR       = 30;
long intervaloDHT       = 2000;
long intervaloOLED      = 500;    
long intervaloHistorico = 300000;
long intervaloPerf      = 5000;

// timers
unsigned long ultimaLeituraDHT  = 0;
unsigned long ultimaLeituraLDR  = 0;
unsigned long ultimaOLED        = 0;
unsigned long ultimoRegistro    = 0;
unsigned long ultimaPerformance = 0;

// concorrência e tasks FREERTOS
portMUX_TYPE dadosMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t hTaskSensores = nullptr, hTaskManutencao = nullptr;

// interrupção de hardware pelo boot boot GPIO0
// pressão curta: pausa/retoma rastreamento
// pressão longa: aciona retorno HOME dos dois servos
volatile bool    botaoEvento       = false;
volatile bool    botaoPressionado  = false;   
volatile bool    rastreamentoAtivo = true;
volatile int64_t botaoUltimoUs     = 0;
volatile int64_t botaoPressUs      = 0;       

void IRAM_ATTR botaoISR() {
  int64_t agora = esp_timer_get_time();
  bool nivel = digitalRead(BOTAO_PIN);  

  if (!nivel) {
    if (agora - botaoUltimoUs < 250000) return;
    botaoUltimoUs    = agora;
    botaoPressionado = true;
    botaoPressUs     = agora;
  } else {
    if (!botaoPressionado) return;
    botaoPressionado = false;
    botaoEvento      = true;
  }
}

// estado do servo horizontal
enum EstadoServoH : uint8_t {
  SH_RASTREANDO,     
  SH_TIMEOUT_PAUSA,  
  SH_RETORNO_HOME    
};
EstadoServoH estadoServoH   = SH_RASTREANDO;
unsigned long tInicioGiro   = 0;  
unsigned long tInicioPausa  = 0;   
bool          retornoManual = false; 

// gerenciamento de energia - deep sleep
bool economiaAtiva    = false;
int  limiarNoite      = 200;
long minutosDeepSleep = 30;
volatile unsigned long inicioEscuro = 0;
#define TEMPO_CONFIRMA_NOITE 60000UL   // 1 minuto de escuridão contínua

// servo horizontal: 
#define SERVOH_PARADO_US     1500   // parado
#define SERVOH_DESVIO_MAX_US 200    // velocidade máxima
#define SERVOH_VEL_MAX_GS    90.0f  // graus estimados no desvio máximo

// limites da rotação horizontal para não enrolar o fioanti-enrolamento do fio
#define SERVOH_LIMITE_MAX    340.0f  // grau maximo para retorno
#define SERVOH_LIMITE_MIN     10.0f  // grau para onde retorna
#define SERVOH_VEL_RETORNO   0.7f    // velocidade maxima do retorno no sentido anti horário

// timeout de busca do sol, se não encontrar o sol ele para para não ficar girando infinitamente
#define SERVOH_TIMEOUT_BUSCA  5000UL  // maximo de tempo girando sem achar equilibrio
#define SERVOH_PAUSA_BUSCA    2000UL  // maximo de tempo parado antes de tentar de novo

// pressão longa do botão home
#define BOTAO_TEMPO_HOME_MS   1500UL  

// medição da CPU
volatile unsigned long busyAcumuladoUs = 0;
unsigned long janelaCpuInicio = 0;

// gráfico
unsigned long tempoOffsetMs = 0;
unsigned long tempoSistema() { return tempoOffsetMs + millis(); }

bool relogioSincronizado = false;
bool ntpSolicitado       = false;
#define EPOCH_MINIMO_VALIDO 1735689600UL  

uint32_t epochAtual() {
  time_t t = time(nullptr);
  return (relogioSincronizado && t > (time_t)EPOCH_MINIMO_VALIDO) ? (uint32_t)t : 0;
}

void sincronizarRelogio() {
  if (relogioSincronizado || WiFi.status() != WL_CONNECTED) return;
  if (!ntpSolicitado) {
    // fuso horario
    configTzTime("<-03>3", "pool.ntp.org", "time.google.com", "a.st1.ntp.br");
    ntpSolicitado = true;
    adicionarLog(LOG_INFO, "sincronizarRelogio", "Sincronizacao NTP solicitada");
    return;
  }
  if (time(nullptr) > (time_t)EPOCH_MINIMO_VALIDO) {
    relogioSincronizado = true;
    struct tm tinfo;
    char buf[48];
    if (getLocalTime(&tinfo, 0)) {
      strftime(buf, sizeof(buf), "Relogio NTP ok: %d/%m/%Y %H:%M:%S", &tinfo);
      adicionarLog(LOG_INFO, "sincronizarRelogio", buf);
    }
  }
}

// bUFFER circular histórico de 24H
#define MAX_HISTORICO 288
struct Registro {
  uint32_t epoch;
  uint32_t tempoRel;
  float    temperatura, umidade;
  int16_t  ldrMedia, servoH, servoV, reservado;
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

// tempo de execução
unsigned long tempoRastrearSol = 0;
unsigned long tempoLerDHT      = 0;
unsigned long tempoOLED        = 0;
unsigned long tempoHistorico   = 0;
unsigned long tempoWebServer   = 0;
float cpuUsage = 0.0;

// sistema de logs
#define MAX_LOGS   150

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

// led builtin
void atualizarLed() {
  digitalWrite(LED_RASTR, rastreamentoAtivo ? HIGH : LOW);
}

// 
void salvarHistorico() {
  unsigned long t0 = micros();

  portENTER_CRITICAL(&dadosMux);
  int   tl = ldrTL, tr = ldrTR, bl = ldrBL, br = ldrBR;
  float t = temperatura, u = umidade;
  float pH = posH, pV = posV;
  portEXIT_CRITICAL(&dadosMux);

  int16_t ldrMedia = (tl + tr + bl + br) / 4;
  historico[indiceAtual] = {
    epochAtual(), (uint32_t)tempoSistema(),
    t, u, ldrMedia, (int16_t)pH, (int16_t)pV, 0
  };
  indiceAtual = (indiceAtual + 1) % MAX_HISTORICO;
  if (totalRegistros < MAX_HISTORICO) totalRegistros++;
  tempoHistorico = micros() - t0;
  adicionarLog(LOG_INFO, "salvarHistorico", ("Registro " + String(totalRegistros) + " salvo").c_str());
  persistirHistorico();
}

// histórico com persistência de 24 horas
const char* HIST_ARQUIVO = "/historico.bin";
const char* HIST_TEMP    = "/historico.tmp";

struct HistCabecalho {
  uint8_t  versao;
  uint8_t  reservado[3];
  int32_t  idx, total;
  uint32_t offsetMs;
};

void persistirHistorico() {
  HistCabecalho cab = { 3, {0, 0, 0}, indiceAtual, totalRegistros, (uint32_t)tempoSistema() };

  File f = LittleFS.open(HIST_TEMP, "w");
  if (!f) {
    adicionarLog(LOG_ERRO, "persistirHistorico", "Falha ao abrir arquivo temporario");
    return;
  }
  bool ok = f.write((uint8_t*)&cab, sizeof(cab)) == sizeof(cab)
         && f.write((uint8_t*)historico, sizeof(historico)) == sizeof(historico);
  f.close();

  if (ok) {
    LittleFS.remove(HIST_ARQUIVO);
    ok = LittleFS.rename(HIST_TEMP, HIST_ARQUIVO);
  }
  if (ok) adicionarLog(LOG_INFO, "persistirHistorico", "Historico 24h gravado no LittleFS");
  else    adicionarLog(LOG_ERRO, "persistirHistorico", "Falha ao gravar historico no LittleFS");
}

void restaurarHistorico() {
  // caminho atual: arquivo no LittleFS
  File f = LittleFS.open(HIST_ARQUIVO, "r");
  if (f && (size_t)f.size() == sizeof(HistCabecalho) + sizeof(historico)) {
    HistCabecalho cab;
    f.read((uint8_t*)&cab, sizeof(cab));
    if (cab.versao == 3) {
      f.read((uint8_t*)historico, sizeof(historico));
      indiceAtual    = constrain(cab.idx,   0, MAX_HISTORICO - 1);
      totalRegistros = constrain(cab.total, 0, MAX_HISTORICO);
      tempoOffsetMs  = cab.offsetMs;
      f.close();
      adicionarLog(LOG_INFO, "restaurarHistorico",
        ("Historico restaurado do LittleFS: " + String(totalRegistros) + " registros").c_str());
      return;
    }
  }
  if (f) f.close();

  // 
  prefsHist.begin("hist", false);
  bool legado = prefsHist.getUChar("ver", 0) == 2
             && prefsHist.isKey("buf")
             && prefsHist.getBytesLength("buf") == sizeof(historico);
  if (legado) {
    prefsHist.getBytes("buf", historico, sizeof(historico));
    indiceAtual    = prefsHist.getInt("idx", 0);
    totalRegistros = prefsHist.getInt("total", 0);
    tempoOffsetMs  = prefsHist.getULong("offset", 0);
    prefsHist.clear(); 
    prefsHist.end();
    persistirHistorico();
    adicionarLog(LOG_INFO, "restaurarHistorico",
      ("Historico migrado da NVS para o LittleFS: " + String(totalRegistros) + " registros").c_str());
    return;
  }
  prefsHist.end();
  adicionarLog(LOG_INFO, "restaurarHistorico", "Sem historico previo; iniciando buffer vazio");
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
  intervaloOLED      = prefs.getLong("iOLED", intervaloOLED);
  intervaloHistorico = prefs.getLong("iHist", intervaloHistorico);
  intervaloPerf      = prefs.getLong("iPerf", intervaloPerf);
  economiaAtiva      = prefs.getBool("eco", economiaAtiva);
  limiarNoite        = prefs.getInt("ecoLim", limiarNoite);
  minutosDeepSleep   = prefs.getLong("ecoMin", minutosDeepSleep);
  senhaUser          = prefs.getString("sU", senhaUser);
  senhaAdm           = prefs.getString("sA", senhaAdm);
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
  prefs.putLong("iOLED", intervaloOLED);
  prefs.putLong("iHist", intervaloHistorico);
  prefs.putLong("iPerf", intervaloPerf);
  prefs.putBool("eco", economiaAtiva);
  prefs.putInt("ecoLim", limiarNoite);
  prefs.putLong("ecoMin", minutosDeepSleep);
  prefs.putString("sU", senhaUser);
  prefs.putString("sA", senhaAdm);
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

// servidor de arquivos estáticos
void servirArquivo(const char* caminho, const char* tipo) {
  File f = LittleFS.open(caminho, "r");
  if (!f || f.isDirectory()) {
    server.send(500, "text/plain; charset=utf-8",
      "Arquivo ausente no LittleFS: " + String(caminho) +
      "\nGrave a pasta data/ com o plugin de upload do LittleFS.");
    adicionarLog(LOG_ERRO, "servirArquivo", ("Ausente no LittleFS: " + String(caminho)).c_str());
    return;
  }
  server.streamFile(f, tipo);
  f.close();
}

// autenticação e sessõoes
String gerarToken() {
  char buf[17];
  snprintf(buf, sizeof(buf), "%08lx%08lx",
           (unsigned long)esp_random(), (unsigned long)esp_random());
  return String(buf);
}

bool sessaoValida(const Sessao& s) {
  return s.perfil != PERFIL_NENHUM && (millis() - s.criadaEm) < SESSAO_DURACAO;
}

String criarSessao(Perfil perfil) {
  int slot = 0;
  unsigned long maisAntiga = ULONG_MAX;
  for (int i = 0; i < MAX_SESSOES; i++) {
    if (!sessaoValida(sessoes[i])) { slot = i; break; }
    if (sessoes[i].criadaEm < maisAntiga) { maisAntiga = sessoes[i].criadaEm; slot = i; }
  }
  String tok = gerarToken();
  strncpy(sessoes[slot].token, tok.c_str(), sizeof(sessoes[slot].token));
  sessoes[slot].perfil   = perfil;
  sessoes[slot].criadaEm = millis();
  return tok;
}

Perfil perfilSessao() {
  if (!server.hasHeader("Cookie")) return PERFIL_NENHUM;
  String cookie = server.header("Cookie");
  int p = cookie.indexOf("SID=");
  if (p < 0) return PERFIL_NENHUM;
  int fim = cookie.indexOf(';', p);
  String tok = cookie.substring(p + 4, fim < 0 ? cookie.length() : fim);
  for (int i = 0; i < MAX_SESSOES; i++) {
    if (sessaoValida(sessoes[i]) && tok == sessoes[i].token) return sessoes[i].perfil;
  }
  return PERFIL_NENHUM;
}

void handleAuth() {
  if (millis() < loginBloqueioAte) {
    server.send(429, "application/json",
      "{\"ok\":false,\"erro\":\"Muitas tentativas. Aguarde 30 segundos.\"}");
    return;
  }
  bool admin = server.arg("perfil") == "adm";
  const String& esperada = admin ? senhaAdm : senhaUser;

  if (server.arg("senha") == esperada && esperada.length() > 0) {
    loginFalhas = 0;
    String tok = criarSessao(admin ? PERFIL_ADMIN : PERFIL_USER);
    server.sendHeader("Set-Cookie", "SID=" + tok + "; Path=/; HttpOnly; Max-Age=28800");
    server.send(200, "application/json",
      String("{\"ok\":true,\"perfil\":\"") + (admin ? "adm" : "user") + "\"}");
    adicionarLog(LOG_INFO, "handleAuth", admin ? "Login de administrador" : "Login de visitante");
    return;
  }
  if (++loginFalhas >= 5) {
    loginFalhas = 0;
    loginBloqueioAte = millis() + 30000UL;
    adicionarLog(LOG_WARN, "handleAuth", "5 falhas de login: bloqueio de 30 s ativado");
  }
  server.send(401, "application/json", "{\"ok\":false,\"erro\":\"Senha incorreta\"}");
}

void handleLogout() {
  Perfil atual = perfilSessao();
  if (atual != PERFIL_NENHUM && server.hasHeader("Cookie")) {
    String cookie = server.header("Cookie");
    int p = cookie.indexOf("SID=");
    int fim = cookie.indexOf(';', p);
    String tok = cookie.substring(p + 4, fim < 0 ? cookie.length() : fim);
    for (int i = 0; i < MAX_SESSOES; i++) {
      if (tok == sessoes[i].token) sessoes[i].perfil = PERFIL_NENHUM;
    }
  }
  server.sendHeader("Set-Cookie", "SID=; Path=/; HttpOnly; Max-Age=0");
  server.sendHeader("Location", "/login");
  server.send(302, "text/plain", "");
}

void handleSessao() {
  Perfil p = perfilSessao();
  server.send(200, "application/json",
    String("{\"perfil\":\"") + (p == PERFIL_ADMIN ? "adm" : p == PERFIL_USER ? "user" : "none") + "\"}");
}

void paginaProtegida(const char* arq, Perfil minimo) {
  Perfil p = perfilSessao();
  if (p < minimo) {
    server.sendHeader("Location", minimo == PERFIL_ADMIN ? "/login?adm=1" : "/login");
    server.send(302, "text/plain", "");
    return;
  }
  servirArquivo(arq, "text/html");
}

bool apiAutorizada(Perfil minimo) {
  Perfil p = perfilSessao();
  if (p >= minimo) return true;
  server.send(p == PERFIL_NENHUM ? 401 : 403, "application/json",
    p == PERFIL_NENHUM ? "{\"erro\":\"nao autenticado\"}"
                       : "{\"erro\":\"requer perfil administrador\"}");
  return false;
}

void registrarRotas() {
  // autenticações públicas
  server.on("/login",  []() { servirArquivo("/login.html", "text/html"); });
  server.on("/auth",   HTTP_POST, handleAuth);
  server.on("/logout", handleLogout);
  server.on("/sessao", handleSessao);

  // APIs dinâmicas (JSON) — leitura exige sessão; escrita exige administrador
  server.on("/dados",      []() { if (apiAutorizada(PERFIL_USER))  handleDados(); });
  server.on("/historico",  []() { if (apiAutorizada(PERFIL_USER))  handleHistorico(); });
  server.on("/perfdata",   []() { if (apiAutorizada(PERFIL_USER))  handlePerfData(); });
  server.on("/logsdata",   []() { if (apiAutorizada(PERFIL_USER))  handleLogsData(); });
  server.on("/logsclear",      []() { if (apiAutorizada(PERFIL_ADMIN)) handleLogsClear(); });
  server.on("/rastreamento",   HTTP_POST, []() { if (apiAutorizada(PERFIL_USER))  handleRastreamento(); });
  server.on("/configdata", []() { if (apiAutorizada(PERFIL_ADMIN)) handleConfigData(); });
  server.on("/configsave", HTTP_POST, []() { if (apiAutorizada(PERFIL_ADMIN)) handleConfigSave(); });

  // páginas vindas do LittleFS 
  server.on("/",            []() { paginaProtegida("/index.html",       PERFIL_USER); });
  server.on("/graficos",    []() { paginaProtegida("/graficos.html",    PERFIL_USER); });
  server.on("/performance", []() { paginaProtegida("/performance.html", PERFIL_USER); });
  server.on("/logs",        []() { paginaProtegida("/logs.html",        PERFIL_USER); });
  server.on("/config",      []() { paginaProtegida("/config.html",      PERFIL_ADMIN); });
  server.on("/sobre",       []() { paginaProtegida("/sobre.html",       PERFIL_USER); });
  server.on("/changelog",   []() { paginaProtegida("/changelog.html",   PERFIL_USER); });

  // assets públicos 
  server.on("/style.css",   []() { servirArquivo("/style.css", "text/css"); });
  server.on("/app.js",      []() { servirArquivo("/app.js",    "application/javascript"); });

  server.onNotFound([]() { server.send(404, "text/plain; charset=utf-8", "Rota nao encontrada"); });
}

//
void setup() {
  Serial.begin(115200);
  adicionarLog(LOG_INFO, "setup", "Iniciando sistema...");

  esp_sleep_wakeup_cause_t causa = esp_sleep_get_wakeup_cause();
  bool acordouDeSleep = (causa == ESP_SLEEP_WAKEUP_TIMER || causa == ESP_SLEEP_WAKEUP_EXT0);
  if (causa == ESP_SLEEP_WAKEUP_TIMER) {
    adicionarLog(LOG_INFO, "setup", "Acordou do deep sleep (timer)");
  } else if (causa == ESP_SLEEP_WAKEUP_EXT0) {
    adicionarLog(LOG_INFO, "setup", "Acordou do deep sleep pelo botao BOOT (GPIO 0)");
  }

  if (!LittleFS.begin(true)) {
    adicionarLog(LOG_ERRO, "setup", "Falha ao montar o LittleFS");
  } else {
    adicionarLog(LOG_INFO, "setup", ("LittleFS montado: " + String(LittleFS.usedBytes() / 1024) + " KB usados").c_str());
  }

  carregarConfig();
  restaurarHistorico();

  prefs.begin("rastreador", false);
  bootCount = prefs.getUInt("boots", 0) + 1;
  prefs.putUInt("boots", bootCount);
  prefs.end();
  adicionarLog(LOG_INFO, "setup", ("Boot numero " + String(bootCount) + " (contador persistido em NVS)").c_str());

  dht.begin();
  ultimaLeituraDHT = millis();  

  servoH.attach(SERVO_H, 500, 2500);        
  servoV.attach(SERVO_V);
  servoH.writeMicroseconds(SERVOH_PARADO_US); 
  servoV.write((int)posV);
  adicionarLog(LOG_INFO, "setup", "Servo V em 90 graus; servo H (360, continuo) parado");
  atualizarLed();

  pinMode(BOTAO_PIN, INPUT_PULLUP);  
  attachInterrupt(digitalPinToInterrupt(BOTAO_PIN), botaoISR, CHANGE);  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_RASTR, OUTPUT);
  atualizarLed();  

  Wire.begin(21, 22);
  Wire.setClock(400000);  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    adicionarLog(LOG_ERRO, "setup", "OLED SSD1306 nao encontrado no endereco 0x3C");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println(acordouDeSleep ? "Acordou do sleep" : "Iniciando...");
  display.display();

  iniciarWiFi();
  WiFi.setSleep(true);  
  adicionarLog(LOG_INFO, "setup", "Modem sleep do Wi-Fi habilitado");

  registrarRotas();
  static const char* kHeaders[] = { "Cookie" };
  server.collectHeaders(kHeaders, 1);  
  server.begin();
  adicionarLog(LOG_INFO, "setup", "Servidor web iniciado na porta 80");

  xTaskCreatePinnedToCore(taskSensores,   "sensores",   6144, nullptr, 2, &hTaskSensores,   1);
  xTaskCreatePinnedToCore(taskManutencao, "manutencao", 6144, nullptr, 1, &hTaskManutencao, 0);
  adicionarLog(LOG_INFO, "setup", "Tasks criadas: sensores (core 1) e manutencao (core 0)");
  janelaCpuInicio = millis();
}

// looptask, core1
void loop() {
  unsigned long t0 = micros();
  server.handleClient();
  unsigned long gasto = micros() - t0;
  tempoWebServer = gasto;
  portENTER_CRITICAL(&dadosMux);
  busyAcumuladoUs += gasto;
  portEXIT_CRITICAL(&dadosMux);

  gerenciarWiFi();

  // processamento do botão boot
  if (botaoEvento) {
    botaoEvento = false;

    // calculo da pressão do botão
    int64_t agora64    = esp_timer_get_time();
    uint32_t duracaoMs = (uint32_t)((agora64 - botaoPressUs) / 1000);

    if (duracaoMs >= BOTAO_TEMPO_HOME_MS) {
      // pressão longa
      rastreamentoAtivo = false;
      retornoManual     = true;
      estadoServoH      = SH_RETORNO_HOME;
      servoV.write(90);                         
      posV  = 90.0f;
      alvoV = 90.0f;
      adicionarLog(LOG_INFO, "botao", "Pressao longa: retorno HOME ativado (V=90, H->0)");
      atualizarLed();  
    } else {
      // pressão curta
      rastreamentoAtivo = !rastreamentoAtivo;
      if (!rastreamentoAtivo) {
        servoH.writeMicroseconds(SERVOH_PARADO_US);
        estadoServoH = SH_RASTREANDO;   
        tInicioGiro  = 0;
      }
      atualizarLed(); 
      adicionarLog(LOG_INFO, "botao", rastreamentoAtivo
        ? "Pressao curta: rastreamento retomado"
        : "Pressao curta: rastreamento pausado");
    }
  }
  vTaskDelay(pdMS_TO_TICKS(2));
}

// task sensores, core1
void taskSensores(void* pv) {
  for (;;) {
    unsigned long agora = millis();
    unsigned long t0 = micros();
    bool trabalhou = false;

    if (rastreamentoAtivo && agora - ultimaLeituraLDR >= (unsigned long)intervaloLDR) {
      ultimaLeituraLDR = agora;
      rastrearSol();
      trabalhou = true;
    } else if (!rastreamentoAtivo) {
      servoH.writeMicroseconds(SERVOH_PARADO_US);
    }
    if (agora - ultimaOLED >= (unsigned long)intervaloOLED) {
      ultimaOLED = agora;
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
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// task manutenção, core 0
void taskManutencao(void* pv) {
  for (;;) {
    unsigned long agora = millis();

    sincronizarRelogio();

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

// gerenciamento de energia - deep sleep
static volatile bool emProcessoSleep = false;

void gerenciarEnergia() {
  if (!economiaAtiva) { inicioEscuro = 0; return; }
  if (emProcessoSleep) return;  

  portENTER_CRITICAL(&dadosMux);
  int media = (ldrTL + ldrTR + ldrBL + ldrBR) / 4;
  portEXIT_CRITICAL(&dadosMux);

  if (media >= limiarNoite) { inicioEscuro = 0; return; }

  if (inicioEscuro == 0) {
    inicioEscuro = millis();
    adicionarLog(LOG_WARN, "gerenciarEnergia", "Escuridao detectada; deep sleep em 1 min se persistir");
    return;
  }
  if (millis() - inicioEscuro >= TEMPO_CONFIRMA_NOITE) {
    emProcessoSleep = true; 
    adicionarLog(LOG_WARN, "gerenciarEnergia", "Entrando em deep sleep");
    persistirHistorico();
    avisarDeepSleep();

    // acordar configuravel
    esp_sleep_enable_timer_wakeup((uint64_t)minutosDeepSleep * 60ULL * 1000000ULL);

    // acordar por meio do botão boot
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_sleep_enable_timer_wakeup((uint64_t)minutosDeepSleep * 60ULL * 1000000ULL);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);  

    digitalWrite(LED_RASTR, LOW); 
    esp_deep_sleep_start();
  }
}

// aviso de deep sleep no oled e no buzzer
void avisarDeepSleep() {
  servoH.writeMicroseconds(SERVOH_PARADO_US);
  servoH.detach();
  servoV.detach();

  // oled
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, WHITE);
  display.setTextSize(2);
  display.setCursor(10, 6);
  display.println("DEEP");
  display.setCursor(10, 24);
  display.println("SLEEP");
  display.setTextSize(1);
  display.setCursor(6, 44);
  display.println("ESP32 dormindo");
  display.setCursor(6, 53);
  display.printf("Volta em: %ld min", minutosDeepSleep);
  display.display();

  // 3 beeps no buzzer
  for (int i = 0; i < 3; i++) {
    // Tom: ~2 kHz por 150 ms
    unsigned long fimTom = millis() + 150;
    while (millis() < fimTom) {
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(250);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(250);
    }
    unsigned long fimPausa = millis() + 200;
    while (millis() < fimPausa) { }
  }
  digitalWrite(BUZZER_PIN, LOW);
}

// rastreamento do sol
void rastrearSol() {
  unsigned long t0    = micros();
  unsigned long agora = millis();

  int tl = analogRead(LDR_TL);
  int tr = analogRead(LDR_TR);
  int bl = analogRead(LDR_BL);
  int br = analogRead(LDR_BR);

  // eixo vertical
  float erroV     = ((bl + br) / 2.0f) - ((tl + tr) / 2.0f);
  float novoAlvoV = alvoV;
  if (fabsf(erroV) > zonaMorta) {
    float passo = constrain(erroV * kp, -passoMax, passoMax);
    novoAlvoV = constrain(alvoV + passo, 20, 160);
  }
  float novaPosV = posV + (novoAlvoV - posV) * suavizacao;
  servoV.write((int)novaPosV);

  // eixo horizontal
  static unsigned long tAnterior = 0;
  float dt = tAnterior > 0 ? (agora - tAnterior) / 1000.0f : 0.0f;
  tAnterior = agora;

  float erroH   = ((tr + br) / 2.0f) - ((tl + bl) / 2.0f);
  float velNorm = 0.0f;  // -1 (anti-horário) .. +1 (horário)
  float novaPosH = posH;

  // anti enrolamento
  if (posH >= SERVOH_LIMITE_MAX || retornoManual) {
    if (estadoServoH != SH_RETORNO_HOME) {
      estadoServoH = SH_RETORNO_HOME;
      if (retornoManual)
        adicionarLog(LOG_INFO, "rastrearSol", "HOME manual: servo H retornando a 0 graus");
      else
        adicionarLog(LOG_WARN, "rastrearSol", "Limite maximo atingido: servo H retornando a 0 graus");
    }
  }

  switch (estadoServoH) {

    // rastreamento normal
    case SH_RASTREANDO: {
      bool emEquilibrio = (fabsf(erroH) <= zonaMorta);

      if (!emEquilibrio) {
        velNorm = constrain(erroH * kp, -passoMax, passoMax) / passoMax;

        // temporizador de busca
        if (tInicioGiro == 0) tInicioGiro = agora;

        // timeout
        if (agora - tInicioGiro >= SERVOH_TIMEOUT_BUSCA) {
          servoH.writeMicroseconds(SERVOH_PARADO_US);
          velNorm       = 0.0f;
          tInicioGiro   = 0;
          tInicioPausa  = agora;
          estadoServoH  = SH_TIMEOUT_PAUSA;
          adicionarLog(LOG_WARN, "rastrearSol",
            "Timeout de busca: servo H pausado temporariamente");
        }
      } else {
        // equilibrio
        tInicioGiro = 0;
      }

      servoH.writeMicroseconds(SERVOH_PARADO_US + (int)(velNorm * SERVOH_DESVIO_MAX_US));
      break;
    }

    // pause pós timeout
    case SH_TIMEOUT_PAUSA: {
      servoH.writeMicroseconds(SERVOH_PARADO_US);  // mantém parado
      velNorm = 0.0f;

      if (agora - tInicioPausa >= SERVOH_PAUSA_BUSCA) {
        // retorno ao rastreamento normal
        tInicioGiro  = 0;
        estadoServoH = SH_RASTREANDO;
        adicionarLog(LOG_INFO, "rastrearSol", "Pausa concluida: retomando busca do sol");
      }
      break;
    }

    // retorna para grau 0
    case SH_RETORNO_HOME: {
      if (posH > SERVOH_LIMITE_MIN) {
        velNorm = -SERVOH_VEL_RETORNO;  // gira anti-horário devagar
        servoH.writeMicroseconds(SERVOH_PARADO_US + (int)(velNorm * SERVOH_DESVIO_MAX_US));
      } else {
        servoH.writeMicroseconds(SERVOH_PARADO_US);
        velNorm       = 0.0f;
        novaPosH      = 0.0f;  
        estadoServoH  = SH_RASTREANDO;
        tInicioGiro   = 0;

        if (retornoManual) {
          retornoManual     = false;
          rastreamentoAtivo = false;  
          atualizarLed();  
          adicionarLog(LOG_INFO, "rastrearSol",
            "HOME concluido: servos em posicao inicial, rastreamento pausado");
        } else {
          adicionarLog(LOG_INFO, "rastrearSol",
            "Retorno anti-enrolamento concluido: retomando rastreamento");
        }
      }
      break;
    }
  }

  // 
  if (estadoServoH != SH_RETORNO_HOME || novaPosH != 0.0f) {
    novaPosH = fmodf(posH + velNorm * SERVOH_VEL_MAX_GS * dt + 360.0f, 360.0f);
  }

  portENTER_CRITICAL(&dadosMux);
  ldrTL = tl; ldrTR = tr; ldrBL = bl; ldrBR = br;
  alvoV = novoAlvoV;
  posV  = novaPosV;
  posH  = novaPosH;
  portEXIT_CRITICAL(&dadosMux);

  tempoRastrearSol = micros() - t0;
}

// 
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

// 
void atualizarOLED() {
  unsigned long t0 = micros();
  display.clearDisplay();
  display.setCursor(0, 0);

  struct tm tinfo;
  if (relogioSincronizado && getLocalTime(&tinfo, 0)) {
    char linha[22];
    strftime(linha, sizeof(linha), "%d/%m %H:%M:%S", &tinfo);
    display.println(linha);
  }
  display.printf("Temp: %.1fC\n", temperatura);
  display.printf("Umid: %.1f%%\n", umidade);
  display.printf("V:%.0f H:%.0f\n", posV, posH);
  display.printf("IP:%s\n", WiFi.localIP().toString().c_str());
  if (inicioEscuro > 0) {
    long resta = (long)(TEMPO_CONFIRMA_NOITE - (millis() - inicioEscuro)) / 1000;
    if (resta > 0) display.printf("Sem luz! Sleep %lds\n", resta);
  }
  display.display();
  tempoOLED = micros() - t0;
}

// 
void handleRastreamento() {
  StaticJsonDocument<64> req;
  DeserializationError err = deserializeJson(req, server.arg("plain"));
  if (err) {
    server.send(400, "application/json", "{\"erro\":\"JSON invalido\"}");
    return;
  }
  bool novoEstado = req["ativo"].as<bool>();
  if (rastreamentoAtivo == novoEstado) {
    server.send(200, "application/json",
      String("{\"ativo\":") + (rastreamentoAtivo ? "true" : "false") + "}");
    return;
  }
  rastreamentoAtivo = novoEstado;
  if (!rastreamentoAtivo) {
    servoH.writeMicroseconds(SERVOH_PARADO_US);
    estadoServoH = SH_RASTREANDO;
    tInicioGiro  = 0;
  }
  atualizarLed();
  adicionarLog(LOG_INFO, "handleRastreamento",
    rastreamentoAtivo ? "Rastreamento ativado via web" : "Rastreamento desativado via web");
  server.send(200, "application/json",
    String("{\"ativo\":") + (rastreamentoAtivo ? "true" : "false") + "}");
}

void handleDados() {
  long sleepEm = -1;
  if (economiaAtiva && inicioEscuro > 0) {
    long resta = (long)(TEMPO_CONFIRMA_NOITE - (millis() - inicioEscuro)) / 1000;
    sleepEm = resta > 0 ? resta : 0;
  }

  StaticJsonDocument<512> doc;
  doc["temperatura"] = temperatura; doc["umidade"] = umidade;
  doc["servoH"] = (int)posH; doc["servoV"] = (int)posV;
  doc["ldrTL"] = ldrTL; doc["ldrTR"] = ldrTR;
  doc["ldrBL"] = ldrBL; doc["ldrBR"] = ldrBR;
  doc["eco"]     = economiaAtiva;
  doc["sleepEm"] = sleepEm;          
  doc["epoch"]   = epochAtual();     
  const char* nomesEstado[] = { "rastreando", "pausa_busca", "retorno_home" };
  doc["estadoServoH"] = nomesEstado[(int)estadoServoH];
  doc["rastreamentoAtivo"] = rastreamentoAtivo;
  String json; serializeJson(doc, json);
  server.send(200, "application/json", json);
}

// rota histórico
void handleHistorico() {
  portENTER_CRITICAL(&dadosMux);
  int total  = totalRegistros;
  int inicio = (total < MAX_HISTORICO) ? 0 : indiceAtual;
  portEXIT_CRITICAL(&dadosMux);

  WiFiClient cliente = server.client();
  cliente.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: application/json\r\n"
    "Transfer-Encoding: chunked\r\n"
    "Connection: close\r\n\r\n"
  );

  auto enviarChunk = [&](const String& s) {
    if (!s.length()) return;
    cliente.printf("%X\r\n", s.length());
    cliente.print(s);
    cliente.print("\r\n");
  };

  String cabecalho = "{\"total\":";
  cabecalho += total;
  cabecalho += ",\"agora\":";  cabecalho += epochAtual();
  cabecalho += ",\"boots\":";  cabecalho += bootCount;
  cabecalho += ",\"dados\":[";
  enviarChunk(cabecalho);

  String buf;
  buf.reserve(512);
  for (int i = 0; i < total; i++) {
    int idx = (inicio + i) % MAX_HISTORICO;
    if (i > 0) buf += ",";
    buf += "{\"e\":" + String(historico[idx].epoch)
         + ",\"t\":" + String(historico[idx].tempoRel / 1000)
         + ",\"temp\":" + String(historico[idx].temperatura, 1)
         + ",\"umid\":" + String(historico[idx].umidade, 1)
         + ",\"ldr\":" + String(historico[idx].ldrMedia)
         + ",\"sh\":" + String(historico[idx].servoH)
         + ",\"sv\":" + String(historico[idx].servoV) + "}";
    if (buf.length() >= 480 || i == total - 1) {
      enviarChunk(buf);
      buf = "";
    }
  }

  enviarChunk("]}");
  cliente.print("0\r\n\r\n");
  cliente.flush();
}

// 
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

// 
void handleLogsClear() {
  logIdx = 0; logTotal = 0;
  adicionarLog(LOG_INFO, "handleLogsClear", "Logs limpos pelo usuario via web");
  server.send(200, "application/json", "{\"ok\":true}");
}

// 
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
  json += "\"ntp\":"         + String(relogioSincronizado ? "true" : "false") + ",";
  json += "\"fs\":{\"total\":" + String(LittleFS.totalBytes())
        + ",\"usado\":" + String(LittleFS.usedBytes()) + "},";
  json += "\"num_tasks\":"   + String(uxTaskGetNumberOfTasks()) + ",";
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
    { "loopTask",   "Servidor web, Wi-Fi e botao",            xTaskGetCurrentTaskHandle(), 1 },
    { "sensores",   "LDRs, servos, DHT e OLED",               hTaskSensores,               1 },
    { "manutencao", "Historico, NTP, performance e energia",  hTaskManutencao,             0 },
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

// 
void handleConfigData() {
  StaticJsonDocument<512> doc;
  doc["ssid"]  = ssid;
  doc["pass"]  = password;
  doc["iLDR"]  = intervaloLDR;
  doc["iDHT"]  = intervaloDHT;
  doc["iOLED"] = intervaloOLED;
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

// 
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
  if (server.hasArg("iOLED")) intervaloOLED      = server.arg("iOLED").toInt();
  if (server.hasArg("iHist")) intervaloHistorico = server.arg("iHist").toInt();
  if (server.hasArg("iPerf")) intervaloPerf      = server.arg("iPerf").toInt();
  if (server.hasArg("kp"))    kp         = server.arg("kp").toFloat();
  if (server.hasArg("zona"))  zonaMorta  = server.arg("zona").toFloat();
  if (server.hasArg("passo")) passoMax   = server.arg("passo").toFloat();
  if (server.hasArg("suav"))  suavizacao = server.arg("suav").toFloat();
  if (server.hasArg("eco"))    economiaAtiva    = server.arg("eco").toInt() == 1;
  if (server.hasArg("ecoLim")) limiarNoite      = server.arg("ecoLim").toInt();
  if (server.hasArg("ecoMin")) minutosDeepSleep = server.arg("ecoMin").toInt();
  if (server.hasArg("sU") && server.arg("sU").length() >= 4) senhaUser = server.arg("sU");
  if (server.hasArg("sA") && server.arg("sA").length() >= 4) senhaAdm  = server.arg("sA");

  salvarConfig();
  server.send(200, "application/json", "{\"ok\":true}");

  if (trocouWiFi) {
    adicionarLog(LOG_INFO, "handleConfigSave", "Credenciais alteradas, reconectando Wi-Fi");
    ntpSolicitado = false;  
    iniciarWiFi();
  }
}
