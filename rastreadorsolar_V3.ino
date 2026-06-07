#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>

// wifi
const char* ssid     = "Galaxy Isa";
const char* password = "ppkp1993";

// pinos principais
#define LDR_TL  34
#define LDR_TR  35
#define LDR_BL  32
#define LDR_BR  33
#define DHT_PIN  4
#define DHT_TYPE DHT11
#define SERVO_H  13
#define SERVO_V  12

// objetos
WebServer server(80);
DHT dht(DHT_PIN, DHT_TYPE);
Servo servoH, servoV;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// leituras
float posH = 90.0;
float posV = 90.0;
float temperatura = 0.0;
float umidade     = 0.0;
int   ldrTL, ldrTR, ldrBL, ldrBR;

// ---
float alvoH = 90.0;
float alvoV = 90.0;

const float SUAVIZACAO = 0.12;

const int PASSO = 8;

unsigned long ultimaLeituraDHT  = 0;
unsigned long ultimaLeituraLDR  = 0;
unsigned long ultimoRegistro    = 0;

const int  TOLERANCIA          = 50;
const long INTERVALO_DHT       = 2000;    
const long INTERVALO_LDR       = 30;      
const long INTERVALO_HISTORICO = 300000;  

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

void salvarHistorico() {
  int ldrMedia = (ldrTL + ldrTR + ldrBL + ldrBR) / 4;
  historico[indiceAtual].tempo       = millis();
  historico[indiceAtual].temperatura = temperatura;
  historico[indiceAtual].umidade     = umidade;
  historico[indiceAtual].ldrMedia    = ldrMedia;
  historico[indiceAtual].servoH      = (int)posH;
  historico[indiceAtual].servoV      = (int)posV;
  indiceAtual = (indiceAtual + 1) % MAX_HISTORICO;
  if (totalRegistros < MAX_HISTORICO) totalRegistros++;
  Serial.print("[HIST] Registro salvo. Total: ");
  Serial.println(totalRegistros);
}

// ===
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
  display.setCursor(0, 0);
  display.println("Iniciando...");
  display.display();

  WiFi.begin(ssid, password);
  Serial.print("Conectando ao Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.on("/",          handleRoot);
  server.on("/dados",     handleDados);
  server.on("/historico", handleHistorico);
  server.on("/graficos",  handleGraficos);
  server.begin();
  Serial.println("Servidor web iniciado!");
  Serial.println("Rotas: /  /dados  /historico  /graficos");
}

// ===
void loop() {
  server.handleClient();

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
}

const float KP         = 0.008;
const float ZONA_MORTA = 300;
const float PASSO_MAX  = 6.0;

void rastrearSol() {
  ldrTL = analogRead(LDR_TL);
  ldrTR = analogRead(LDR_TR);
  ldrBL = analogRead(LDR_BL);
  ldrBR = analogRead(LDR_BR);

  Serial.print("LDR TL: "); Serial.print(ldrTL);
  Serial.print(" | TR: ");  Serial.print(ldrTR);
  Serial.print(" | BL: ");  Serial.print(ldrBL);
  Serial.print(" | BR: ");  Serial.println(ldrBR);

  int mediaTop = (ldrTL + ldrTR) / 2;
  int mediaBot = (ldrBL + ldrBR) / 2;

  float erro = mediaBot - mediaTop; 
  Serial.print("Erro vertical: "); Serial.println(erro);

  if (abs(erro) > ZONA_MORTA) {
    float passo = erro * KP;
    passo = constrain(passo, -PASSO_MAX, PASSO_MAX);
    alvoV = constrain(alvoV + passo, 20, 160);
  }

  posV = posV + (alvoV - posV) * SUAVIZACAO;
  servoV.write((int)posV);
  servoH.write(90);
}

// ===
void lerDHT() {
  float t = dht.readTemperature();
  float u = dht.readHumidity();

  if (!isnan(t) && t >= 0.0 && t <= 60.0) {
    temperatura = t;
  } else {
    Serial.println("AVISO: temperatura invalida descartada.");
  }

  if (!isnan(u) && u >= 0.0 && u <= 100.0) {
    umidade = u;
  } else {
    Serial.println("AVISO: umidade invalida descartada.");
  }

  Serial.print("Temp: "); Serial.print(temperatura);
  Serial.print(" C | Umid: "); Serial.print(umidade);
  Serial.println(" %");
}

// ===
void atualizarOLED() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Temp: "); display.print(temperatura, 1); display.println("C");
  display.print("Umid: "); display.print(umidade, 1);     display.println("%");
  display.print("H:"); display.print((int)posH);
  display.print("  V:"); display.println((int)posV);
  display.print("Reg:"); display.println(totalRegistros);
  display.print("IP:"); display.println(WiFi.localIP());
  display.display();
}

// ===
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Rastreador Solar ESP32</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:'Segoe UI',Arial,sans-serif;background:#0d1117;color:#e6edf3;min-height:100vh;padding:20px}
    h1{text-align:center;color:#58a6ff;font-size:1.5rem;margin-bottom:4px}
    .subtitle{text-align:center;color:#8b949e;font-size:.8rem;margin-bottom:14px}
    #status{display:flex;align-items:center;justify-content:center;gap:6px;margin-bottom:14px;font-size:.75rem;color:#8b949e}
    #status-dot{width:8px;height:8px;border-radius:50%;background:#3fb950;animation:pulse 2s infinite}
    @keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
    .nav{display:flex;justify-content:center;margin-bottom:20px}
    .btn-graf{background:#21262d;border:1px solid #30363d;color:#58a6ff;padding:9px 22px;border-radius:8px;cursor:pointer;font-size:.85rem;text-decoration:none;transition:background .2s,border-color .2s}
    .btn-graf:hover{background:#30363d;border-color:#58a6ff}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:14px;max-width:800px;margin:0 auto}
    .card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:18px 14px;text-align:center;transition:border-color .3s}
    .card:hover{border-color:#58a6ff}
    .card-label{font-size:.72rem;color:#8b949e;text-transform:uppercase;letter-spacing:.08em;margin-bottom:10px}
    .card-icon{font-size:1.6rem;margin-bottom:6px}
    .card-value{font-size:2rem;font-weight:bold;color:#3fb950;transition:color .4s ease}
    .card-unit{font-size:.8rem;color:#8b949e;margin-top:2px}
    .card.temp .card-value{color:#ff7b72}
    .card.umid .card-value{color:#58a6ff}
    .card.servo .card-value{color:#ffa657}
    .card.ldr .card-value{color:#d2a8ff;font-size:1.4rem}
    .bar-bg{background:#21262d;border-radius:4px;height:6px;margin-top:8px;overflow:hidden}
    .bar{height:6px;border-radius:4px;transition:width .8s ease}
    .bar.ldr-c{background:#d2a8ff}
    .bar.srv-c{background:#ffa657}
    .last-update{text-align:center;font-size:.72rem;color:#8b949e;margin-top:18px}
  </style>
</head>
<body>
  <h1>&#9728;&#65039; Rastreador Solar &mdash; ESP32</h1>
  <p class="subtitle">Performance em Sistemas Ciberfísicos &middot; PUCPR 2026</p>
  <div id="status"><div id="status-dot"></div><span id="status-text">Conectando...</span></div>
  <div class="nav"><a class="btn-graf" href="/graficos">&#128200; Ver Gráficos de 24h</a></div>
  <div class="grid">
    <div class="card temp">
      <div class="card-icon">🌡️</div>
      <div class="card-label">Temperatura</div>
      <div class="card-value" id="temp">--</div>
      <div class="card-unit">&deg;C</div>
    </div>
    <div class="card umid">
      <div class="card-icon">💧</div>
      <div class="card-label">Umidade</div>
      <div class="card-value" id="umid">--</div>
      <div class="card-unit">%</div>
    </div>
    <div class="card servo">
      <div class="card-icon">&#8596;&#65039;</div>
      <div class="card-label">Servo Horizontal</div>
      <div class="card-value" id="sh">--</div>
      <div class="card-unit">graus</div>
      <div class="bar-bg"><div class="bar srv-c" id="sh-bar" style="width:50%"></div></div>
    </div>
    <div class="card servo">
      <div class="card-icon">&#8597;&#65039;</div>
      <div class="card-label">Servo Vertical</div>
      <div class="card-value" id="sv">--</div>
      <div class="card-unit">graus</div>
      <div class="bar-bg"><div class="bar srv-c" id="sv-bar" style="width:50%"></div></div>
    </div>
    <div class="card ldr">
      <div class="card-icon">&#9728;&#65039;</div>
      <div class="card-label">LDR Sup. Esq.</div>
      <div class="card-value" id="tl">--</div>
      <div class="bar-bg"><div class="bar ldr-c" id="tl-bar"></div></div>
    </div>
    <div class="card ldr">
      <div class="card-icon">&#9728;&#65039;</div>
      <div class="card-label">LDR Sup. Dir.</div>
      <div class="card-value" id="tr">--</div>
      <div class="bar-bg"><div class="bar ldr-c" id="tr-bar"></div></div>
    </div>
    <div class="card ldr">
      <div class="card-icon">&#9728;&#65039;</div>
      <div class="card-label">LDR Inf. Esq.</div>
      <div class="card-value" id="bl">--</div>
      <div class="bar-bg"><div class="bar ldr-c" id="bl-bar"></div></div>
    </div>
    <div class="card ldr">
      <div class="card-icon">&#9728;&#65039;</div>
      <div class="card-label">LDR Inf. Dir.</div>
      <div class="card-value" id="br">--</div>
      <div class="bar-bg"><div class="bar ldr-c" id="br-bar"></div></div>
    </div>
  </div>
  <div class="last-update" id="last-update">Aguardando primeira leitura...</div>
  <script>
    const MAX_ADC = 4095;
    function atualizar() {
      fetch('/dados')
        .then(r=>{ if(!r.ok) throw 0; return r.json(); })
        .then(d=>{
          document.getElementById('temp').innerText = d.temperatura.toFixed(1);
          document.getElementById('umid').innerText = d.umidade.toFixed(1);
          document.getElementById('sh').innerText   = d.servoH;
          document.getElementById('sv').innerText   = d.servoV;
          document.getElementById('sh-bar').style.width = (d.servoH/180*100)+'%';
          document.getElementById('sv-bar').style.width = (d.servoV/180*100)+'%';
          ['tl','tr','bl','br'].forEach((id,i)=>{
            const v=[d.ldrTL,d.ldrTR,d.ldrBL,d.ldrBR][i];
            document.getElementById(id).innerText = v;
            document.getElementById(id+'-bar').style.width=(v/MAX_ADC*100).toFixed(1)+'%';
          });
          document.getElementById('status-dot').style.background='#3fb950';
          document.getElementById('status-text').innerText='Conectado ao ESP32';
          document.getElementById('last-update').innerText=
            'Última atualização: '+new Date().toLocaleTimeString('pt-BR');
        })
        .catch(()=>{
          document.getElementById('status-dot').style.background='#ff7b72';
          document.getElementById('status-text').innerText='Sem conexão — tentando reconectar...';
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

void handleDados() {
  StaticJsonDocument<256> doc;
  doc["temperatura"] = temperatura;
  doc["umidade"]     = umidade;
  doc["servoH"]      = (int)posH;
  doc["servoV"]      = (int)posV;
  doc["ldrTL"]       = ldrTL;
  doc["ldrTR"]       = ldrTR;
  doc["ldrBL"]       = ldrBL;
  doc["ldrBR"]       = ldrBR;
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleHistorico() {
  int inicio = (totalRegistros < MAX_HISTORICO) ? 0 : indiceAtual;
  int n = totalRegistros;

  String json = "{\"total\":";
  json += n;
  json += ",\"intervalo_min\":5,\"dados\":[";

  for (int i = 0; i < n; i++) {
    int idx = (inicio + i) % MAX_HISTORICO;
    if (i > 0) json += ",";
    json += "{";
    json += "\"t\":";    json += historico[idx].tempo / 1000;
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

void handleGraficos() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Gráficos 24h — Rastreador Solar</title>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:'Segoe UI',Arial,sans-serif;background:#0d1117;color:#e6edf3;min-height:100vh;padding:20px}
    h1{text-align:center;color:#58a6ff;font-size:1.4rem;margin-bottom:4px}
    .subtitle{text-align:center;color:#8b949e;font-size:.78rem;margin-bottom:18px}
    .nav{display:flex;justify-content:center;gap:10px;margin-bottom:22px}
    .btn{background:#21262d;border:1px solid #30363d;color:#e6edf3;padding:8px 18px;border-radius:8px;cursor:pointer;font-size:.82rem;text-decoration:none;transition:background .2s,border-color .2s}
    .btn:hover{background:#30363d;border-color:#58a6ff}
    .info-bar{display:flex;justify-content:center;gap:20px;flex-wrap:wrap;margin-bottom:20px;font-size:.78rem;color:#8b949e}
    .info-bar span{background:#161b22;padding:5px 12px;border-radius:6px;border:1px solid #30363d}
    .info-bar b{color:#e6edf3}
    .charts{display:flex;flex-direction:column;gap:24px;max-width:900px;margin:0 auto}
    .chart-card{background:#161b22;border:1px solid #30363d;border-radius:12px;padding:20px}
    .chart-title{font-size:.85rem;color:#8b949e;text-transform:uppercase;letter-spacing:.08em;margin-bottom:14px;display:flex;align-items:center;gap:8px}
    .chart-title .dot{width:10px;height:10px;border-radius:50%}
    canvas{max-height:220px}
    #loading{text-align:center;padding:60px;color:#8b949e;font-size:1rem}
    .sem-dados{text-align:center;padding:40px;color:#8b949e}
  </style>
</head>
<body>
  <h1>&#128200; Histórico de 24 Horas</h1>
  <p class="subtitle">Performance em Sistemas Ciberfísicos &middot; PUCPR 2026</p>
  <div class="nav">
    <a class="btn" href="/">&#9646; Dashboard</a>
    <button class="btn" onclick="carregarDados()">&#8635; Atualizar</button>
  </div>
  <div class="info-bar" id="info-bar"><span>Carregando dados...</span></div>
  <div id="loading">&#9203; Buscando dados do ESP32...</div>
  <div class="charts" id="charts" style="display:none">
    <div class="chart-card">
      <div class="chart-title"><div class="dot" style="background:#ff7b72"></div>Temperatura (&deg;C)</div>
      <canvas id="chartTemp"></canvas>
    </div>
    <div class="chart-card">
      <div class="chart-title"><div class="dot" style="background:#58a6ff"></div>Umidade Relativa (%)</div>
      <canvas id="chartUmid"></canvas>
    </div>
    <div class="chart-card">
      <div class="chart-title"><div class="dot" style="background:#d2a8ff"></div>Luminosidade média LDR (0&ndash;4095)</div>
      <canvas id="chartLDR"></canvas>
    </div>
    <div class="chart-card">
      <div class="chart-title"><div class="dot" style="background:#ffa657"></div>Posição dos Servos (graus)</div>
      <canvas id="chartServos"></canvas>
    </div>
  </div>
  <script>
    let charts = {};

    function formatarTempo(segundos) {
      const h = Math.floor(segundos / 3600);
      const m = Math.floor((segundos % 3600) / 60);
      return h + 'h' + String(m).padStart(2,'0');
    }

    function opcoesBase(cor, label, yMin, yMax) {
      return {
        responsive: true,
        animation: { duration: 600 },
        plugins: {
          legend: { display: false },
          tooltip: {
            backgroundColor: '#161b22', borderColor: '#30363d', borderWidth: 1,
            titleColor: '#8b949e', bodyColor: '#e6edf3',
            callbacks: { label: ctx => label + ': ' + ctx.parsed.y }
          }
        },
        scales: {
          x: { ticks:{ color:'#8b949e', maxTicksLimit:12, font:{size:10} }, grid:{ color:'#21262d' } },
          y: { min:yMin, max:yMax, ticks:{ color:'#8b949e', font:{size:10} }, grid:{ color:'#21262d' } }
        }
      };
    }

    function criarGrafico(id, labels, dados, cor, label, yMin, yMax) {
      if (charts[id]) charts[id].destroy();
      const ctx = document.getElementById(id).getContext('2d');
      const grad = ctx.createLinearGradient(0, 0, 0, 220);
      grad.addColorStop(0, cor + '55');
      grad.addColorStop(1, cor + '00');
      charts[id] = new Chart(ctx, {
        type: 'line',
        data: {
          labels: labels,
          datasets: [{ data:dados, borderColor:cor, backgroundColor:grad, borderWidth:2,
            pointRadius: dados.length > 60 ? 0 : 3, pointHoverRadius:5, tension:0.3, fill:true }]
        },
        options: opcoesBase(cor, label, yMin, yMax)
      });
    }

    function carregarDados() {
      document.getElementById('loading').style.display = 'block';
      document.getElementById('charts').style.display  = 'none';
      document.getElementById('info-bar').innerHTML    = '<span>Carregando...</span>';

      fetch('/historico')
        .then(r => { if(!r.ok) throw new Error('Erro HTTP'); return r.json(); })
        .then(d => {
          document.getElementById('loading').style.display = 'none';

          if (!d.dados || d.dados.length === 0) {
            document.getElementById('loading').style.display = 'block';
            document.getElementById('loading').innerHTML =
              '<div class="sem-dados">&#9203; Ainda sem dados suficientes.<br>O primeiro registro aparece após 5 minutos de operação.</div>';
            return;
          }

          document.getElementById('charts').style.display = 'flex';

          const n      = d.dados.length;
          const labels = d.dados.map(r => formatarTempo(r.t));
          const temps  = d.dados.map(r => r.temp);
          const umids  = d.dados.map(r => r.umid);
          const ldrs   = d.dados.map(r => r.ldr);
          const shArr  = d.dados.map(r => r.sh);
          const svArr  = d.dados.map(r => r.sv);

          const tMin = Math.min(...temps).toFixed(1);
          const tMax = Math.max(...temps).toFixed(1);
          const uMin = Math.min(...umids).toFixed(1);
          const uMax = Math.max(...umids).toFixed(1);
          const cobertura = Math.round(n * 5 / 60 * 10) / 10;

          document.getElementById('info-bar').innerHTML =
            '<span>&#128202; <b>' + n + '</b> registros (' + cobertura + 'h)</span>' +
            '<span>&#127777;&#65039; Temp: <b>' + tMin + '&ndash;' + tMax + '&deg;C</b></span>' +
            '<span>&#128167; Umid: <b>' + uMin + '&ndash;' + uMax + '%</b></span>';

          const tPad = 2, uPad = 5, lPad = 200;

          criarGrafico('chartTemp', labels, temps, '#ff7b72', '°C',
            Math.floor(Math.min(...temps)) - tPad, Math.ceil(Math.max(...temps)) + tPad);

          criarGrafico('chartUmid', labels, umids, '#58a6ff', '%',
            Math.floor(Math.min(...umids)) - uPad, Math.ceil(Math.max(...umids)) + uPad);

          criarGrafico('chartLDR', labels, ldrs, '#d2a8ff', 'ADC',
            Math.max(0, Math.min(...ldrs) - lPad), Math.min(4095, Math.max(...ldrs) + lPad));

          // Gráfico servos com 2 linhas
          if (charts['chartServos']) charts['chartServos'].destroy();
          const ctx = document.getElementById('chartServos').getContext('2d');
          const gH = ctx.createLinearGradient(0,0,0,220);
          gH.addColorStop(0,'#ffa65744'); gH.addColorStop(1,'#ffa65700');
          const gV = ctx.createLinearGradient(0,0,0,220);
          gV.addColorStop(0,'#d2a8ff44'); gV.addColorStop(1,'#d2a8ff00');

          charts['chartServos'] = new Chart(ctx, {
            type: 'line',
            data: {
              labels: labels,
              datasets: [
                { label:'Horizontal (Pan)', data:shArr, borderColor:'#ffa657', backgroundColor:gH,
                  borderWidth:2, pointRadius: n > 60 ? 0 : 3, tension:0.3, fill:true },
                { label:'Vertical (Tilt)',  data:svArr, borderColor:'#d2a8ff', backgroundColor:gV,
                  borderWidth:2, pointRadius: n > 60 ? 0 : 3, tension:0.3, fill:true }
              ]
            },
            options: {
              responsive: true, animation:{ duration:600 },
              plugins: {
                legend: { display:true, labels:{ color:'#8b949e', boxWidth:12, font:{size:11} } },
                tooltip: { backgroundColor:'#161b22', borderColor:'#30363d', borderWidth:1,
                  titleColor:'#8b949e', bodyColor:'#e6edf3' }
              },
              scales: {
                x: { ticks:{ color:'#8b949e', maxTicksLimit:12, font:{size:10} }, grid:{ color:'#21262d' } },
                y: { min:0, max:180, ticks:{ color:'#8b949e', font:{size:10} }, grid:{ color:'#21262d' } }
              }
            }
          });
        })
        .catch(err => {
          document.getElementById('loading').innerHTML =
            '<div class="sem-dados">&#10060; Erro ao carregar dados.<br>Verifique a conexão com o ESP32.</div>';
          console.error(err);
        });
    }

    carregarDados();
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}
