
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

// ==========================================
// CONFIGURACOES WIFI
// ==========================================

const char* ap_ssid = "ESP32_CONTROL";
const char* ap_password = "admin123";

// ==========================================
// LOGIN
// ==========================================

const char* login_user = "admin";
const char* login_password = "123456";

// ==========================================
// PINOS
// ==========================================

#define PIN_PORTAS  2
#define PIN_PAINEL  25
#define PIN_PARTIDA 26
#define PIN_AR      27
#define PIN_RX      18   // Novo rele: Liga RX

// ==========================================
// CONTROLE DA SESSAO
// ==========================================

const unsigned long TEMPO_SESSAO = 120000; // 2 minutos sem atividade
bool sessaoAtiva = false;
unsigned long ultimaAtividade = 0;

void ativarSessao() {
  sessaoAtiva = true;
  ultimaAtividade = millis();
  digitalWrite(PIN_RX, HIGH);
}

void encerrarSessao() {
  sessaoAtiva = false;
  digitalWrite(PIN_RX, LOW);
  Serial.println("SESSAO ENCERRADA - RELE RX DESLIGADO");
}

void atualizarSessao() {
  if (sessaoAtiva &&
      (unsigned long)(millis() - ultimaAtividade) >= TEMPO_SESSAO) {
    encerrarSessao();
    Serial.println("SESSAO EXPIRADA POR INATIVIDADE");
  }
}

// ==========================================
// CONFIGURACAO DOS PULSOS
// ==========================================

const unsigned long TEMPO_PULSO = 1000;

struct Saida {
  uint8_t pino;
  bool ativa;
  unsigned long inicio;
};

Saida saidas[] = {
  {PIN_PORTAS, false, 0},
  {PIN_PAINEL, false, 0},
  {PIN_PARTIDA, false, 0},
  {PIN_AR, false, 0}
};

const int TOTAL_SAIDAS = 4;

// ==========================================
// SERVIDOR
// ==========================================

WebServer server(80);
DNSServer dnsServer;

IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

const byte DNS_PORT = 53;

// ==========================================
// AUTENTICACAO
// ==========================================

bool autenticado() {
  if (!server.authenticate(login_user, login_password)) {
    server.requestAuthentication(
      BASIC_AUTH,
      "ESP32 Controle"
    );
    return false;
  }

  // Credenciais aceitas: manter o rele RX ligado durante a sessao.
  ativarSessao();
  return true;
}

// ==========================================
// PAGINA WEB
// ==========================================

const char pagina[] PROGMEM = R"rawliteral(

<!DOCTYPE html>
<html lang="pt-BR">

<head>

<meta charset="UTF-8">

<meta name="viewport"
content="width=device-width, initial-scale=1.0">

<title>Controle ESP32</title>

<style>

* {
  box-sizing: border-box;
}

body {
  margin: 0;
  font-family: Arial, sans-serif;
  background: #101827;
  color: white;
  min-height: 100vh;
  padding: 20px;
}

.container {
  max-width: 430px;
  margin: auto;
}

h1 {
  text-align: center;
  font-size: 25px;
  margin-bottom: 5px;
}

.subtitulo {
  text-align: center;
  color: #94a3b8;
  margin-bottom: 30px;
}

.status {
  background: #1e293b;
  padding: 15px;
  border-radius: 12px;
  margin-bottom: 20px;
  text-align: center;
  color: #4ade80;
}

.grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 15px;
}

.botao {
  border: none;
  border-radius: 15px;
  min-height: 135px;
  padding: 15px 8px;
  color: white;
  background: #263449;
  cursor: pointer;
  font-size: 17px;
  font-weight: bold;
  transition: 0.15s;
}

.botao:active {
  transform: scale(0.96);
}

.botao:disabled {
  opacity: 0.5;
  cursor: wait;
}

.icone {
  display: block;
  font-size: 35px;
  margin-bottom: 12px;
}

.door { border-bottom: 5px solid #3b82f6; }
.painel { border-bottom: 5px solid #eab308; }
.partida { border-bottom: 5px solid #22c55e; }
.ar { border-bottom: 5px solid #06b6d4; }

.botao-sair {
  width: 100%;
  margin-top: 18px;
  padding: 13px;
  border: 1px solid #ef4444;
  border-radius: 10px;
  color: white;
  background: #7f1d1d;
  font-size: 15px;
  font-weight: bold;
  cursor: pointer;
}

.mensagem {
  margin-top: 25px;
  text-align: center;
  padding: 15px;
  border-radius: 10px;
  background: #1e293b;
  color: #cbd5e1;
  min-height: 48px;
}

.rodape {
  text-align: center;
  margin-top: 25px;
  font-size: 12px;
  color: #64748b;
}

</style>

</head>

<body>

<div class="container">

<h1>CONTROLE VEICULAR</h1>

<div class="subtitulo">
ESP32 - Central de comandos
</div>

<div class="status">
● ESP32 CONECTADO
</div>

<div class="grid">

<button class="botao door"
onclick="acionar('portas',this)">

<span class="icone">🔓</span>
Destravar<br>Portas

</button>

<button class="botao painel"
onclick="acionar('painel',this)">

<span class="icone">🚘</span>
Ligar<br>Painel

</button>

<button class="botao partida"
onclick="acionar('partida',this)">

<span class="icone">🔑</span>
Partida

</button>

<button class="botao ar"
onclick="acionar('ar',this)">

<span class="icone">❄️</span>
Ar-condicionado

</button>

</div>

<button class="botao-sair" onclick="sair()">Sair do sistema</button>

<div class="mensagem" id="mensagem">
Aguardando comando...
</div>

<div class="rodape">
ESP32 ACCESS POINT<br>
192.168.4.1
</div>

</div>

<script>

async function acionar(comando, botao) {

  botao.disabled = true;

  document.getElementById("mensagem").innerText =
    "Enviando comando...";

  try {

    const resposta = await fetch(
      "/acionar/" + comando,
      {
        method: "POST",
        credentials: "same-origin"
      }
    );

    const dados = await resposta.text();

    if (!resposta.ok) {
      throw new Error(dados);
    }

    document.getElementById("mensagem").innerText =
      dados;

  } catch (erro) {

    document.getElementById("mensagem").innerText =
      "Erro: " + erro.message;

  }

  setTimeout(() => {
    botao.disabled = false;
  }, 1100);

}

// Mantem a sessao ativa enquanto a pagina estiver aberta.
// Se a pagina for fechada, o RX desliga apos 2 minutos.
const monitorSessao = setInterval(async () => {
  try {
    const resposta = await fetch("/atividade", {
      method: "GET",
      credentials: "same-origin",
      cache: "no-store"
    });

    if (!resposta.ok) {
      clearInterval(monitorSessao);
    }
  } catch (erro) {
    // Sem comunicacao: o ESP32 encerrara a sessao por tempo limite.
  }
}, 30000);

async function sair() {
  clearInterval(monitorSessao);

  document.getElementById("mensagem").innerText =
    "Encerrando sessao...";

  try {
    const resposta = await fetch("/logout", {
      method: "POST",
      credentials: "same-origin"
    });

    const dados = await resposta.text();
    document.getElementById("mensagem").innerText = dados;

    document.querySelectorAll(".botao").forEach(botao => {
      botao.disabled = true;
    });
    document.querySelector(".botao-sair").disabled = true;

  } catch (erro) {
    document.getElementById("mensagem").innerText =
      "Nao foi possivel confirmar o logout.";
  }
}

</script>

</body>
</html>

)rawliteral";

// ==========================================
// ACIONAMENTO DAS SAIDAS
// ==========================================

void acionarSaida(int indice) {

  if (indice < 0 || indice >= TOTAL_SAIDAS) {
    return;
  }

  Saida &saida = saidas[indice];

  // Evita novo acionamento enquanto ativo
  if (saida.ativa) {
    server.send(409, "text/plain",
                "Comando ja esta em execucao");
    return;
  }

  digitalWrite(saida.pino, HIGH);

  saida.ativa = true;
  saida.inicio = millis();

  Serial.print("ACIONADO GPIO: ");
  Serial.println(saida.pino);

  server.send(200, "text/plain",
              "Comando executado - pulso de 1 segundo");
}

// ==========================================
// CONTROLE AUTOMATICO DOS PULSOS
// ==========================================

void atualizarSaidas() {

  unsigned long agora = millis();

  for (int i = 0; i < TOTAL_SAIDAS; i++) {

    if (saidas[i].ativa &&
        (unsigned long)(agora - saidas[i].inicio)
          >= TEMPO_PULSO) {

      digitalWrite(saidas[i].pino, LOW);

      saidas[i].ativa = false;

      Serial.print("DESLIGADO GPIO: ");
      Serial.println(saidas[i].pino);
    }
  }
}

// ==========================================
// CONFIGURAR ROTAS WEB
// ==========================================

void configurarServidor() {

  server.on("/", HTTP_GET, []() {

    if (!autenticado()) return;

    server.send_P(200, "text/html", pagina);

  });

  // Heartbeat: confirma que a pagina continua aberta e autenticada.
  server.on("/atividade", HTTP_GET, []() {
    if (!autenticado()) return;
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/plain", "Sessao ativa");
  });

  // Logout: desliga o rele RX imediatamente.
  server.on("/logout", HTTP_POST, []() {
    if (!autenticado()) return;
    encerrarSessao();
    server.send(200, "text/plain", "Sessao encerrada. Rele RX desligado.");
  });

  server.on("/acionar/portas", HTTP_POST, []() {

    if (!autenticado()) return;

    acionarSaida(0);

  });

  server.on("/acionar/painel", HTTP_POST, []() {

    if (!autenticado()) return;

    acionarSaida(1);

  });

  server.on("/acionar/partida", HTTP_POST, []() {

    if (!autenticado()) return;

    acionarSaida(2);

  });

  server.on("/acionar/ar", HTTP_POST, []() {

    if (!autenticado()) return;

    acionarSaida(3);

  });

  server.onNotFound([]() {

    if (!autenticado()) return;

    server.sendHeader("Location", "http://192.168.4.1/");
    server.send(302, "text/plain", "");

  });

  server.begin();

  Serial.println("Servidor HTTP iniciado");
}

// ==========================================
// SETUP
// ==========================================

void setup() {

  Serial.begin(115200);

  // Inicializar saidas em nivel baixo

  for (int i = 0; i < TOTAL_SAIDAS; i++) {

    pinMode(saidas[i].pino, OUTPUT);

    digitalWrite(saidas[i].pino, LOW);

  }

  // Rele RX: desligado ate um login valido.
  pinMode(PIN_RX, OUTPUT);
  digitalWrite(PIN_RX, LOW);

  // Criar rede Wi-Fi

  WiFi.mode(WIFI_AP);

  WiFi.softAPConfig(localIP, gateway, subnet);

  WiFi.softAP(ap_ssid, ap_password);

  Serial.println();
  Serial.println("==============================");
  Serial.println(" ESP32 CONTROLE VEICULAR");
  Serial.println("==============================");

  Serial.print("WiFi: ");
  Serial.println(ap_ssid);

  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // DNS para portal cativo

  dnsServer.start(
    DNS_PORT,
    "*",
    localIP
  );

  // Iniciar servidor

  configurarServidor();

}

// ==========================================
// LOOP
// ==========================================

void loop() {

  dnsServer.processNextRequest();

  server.handleClient();

  atualizarSaidas();
  atualizarSessao();

}
