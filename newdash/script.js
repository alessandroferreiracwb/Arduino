// IMPORTANTE: SUBSTITUA 'SEU_IP_DO_ESP32' PELO ENDEREÇO IP REAL DO SEU ESP32
// Exemplo: var gateway = `ws://192.168.1.65:81/`;
var gateway = `ws://10.0.0.13:81/`;

var websocket;
window.addEventListener('load', onLoad);

function onLoad(event) {
  initWebSocket();
  // Inicializa os valores dos spans ao carregar a página
  document.querySelectorAll('input[type="range"]').forEach(slider => {
    document.getElementById(slider.id + '_value').innerText = slider.value;
  });
  generateJ2Buttons(); // Gera os botões do Joystick 2 dinamicamente
}

function initWebSocket() {
  console.log('Tentando abrir uma conexão WebSocket...');
  websocket = new WebSocket(gateway);
  websocket.onopen = onOpen;
  websocket.onclose = onClose;
  websocket.onmessage = onMessage;
}

function onOpen(event) {
  console.log('Conexão WebSocket aberta');
  document.getElementById('status').innerText = 'Conectado ao ESP32!';
  sendData(); // Envia o estado inicial ao conectar
}

function onClose(event) {
  console.log('Conexão WebSocket fechada');
  document.getElementById('status').innerText = 'Desconectado. Tentando reconectar...';
  setTimeout(initWebSocket, 2000); // Tenta reconectar após 2 segundos
}

function onMessage(event) {
  console.log("Mensagem do ESP32: " + event.data);
}

// Função para gerar os 24 botões do Joystick 2 dinamicamente
function generateJ2Buttons() {
  const buttonGrid = document.getElementById('j2-button-grid');
  for (let i = 1; i <= 24; i++) {
    const button = document.createElement('button');
    button.id = 'j2_btn' + i;
    button.innerText = 'T ' + i;
    buttonGrid.appendChild(button);
  }
}

// Função para coletar e enviar dados
function sendData() {
  let data = {
    joystickPrincipal: {
      eixos: {
        eixo1: { x: parseFloat(document.getElementById('jp_eixo1x').value) },
        eixo2: { x: parseFloat(document.getElementById('jp_eixo2x').value) },
        eixo3: { x: parseFloat(document.getElementById('jp_eixo3x').value) },
        eixo4: { x: parseFloat(document.getElementById('jp_eixo4x').value) },
        eixo5: { x: parseFloat(document.getElementById('jp_eixo5x').value) },
        eixo6: { x: parseFloat(document.getElementById('jp_eixo6x').value) },
        eixo7: { x: parseFloat(document.getElementById('jp_eixo7x').value) },
        eixo8: { x: parseFloat(document.getElementById('jp_eixo8x').value) }
      },
      botoes: {}
    },
    joystick2: {
      eixos: {
        eixo1: { x: parseFloat(document.getElementById('j2_eixo1x').value) },
        eixo2: { x: parseFloat(document.getElementById('j2_eixo2x').value) },
        eixo3: { x: parseFloat(document.getElementById('j2_eixo3x').value) },
        eixo4: { x: parseFloat(document.getElementById('j2_eixo4x').value) },
        eixo5: { x: parseFloat(document.getElementById('j2_eixo5x').value) }
      },
      botoes: {}
    }
  };

  // Coleta o estado dos botões do Joystick Principal
  for (let i = 1; i <= 12; i++) {
    let btn = document.getElementById('jp_btn' + i);
    if (btn) { // Verifica se o botão existe
      data.joystickPrincipal.botoes['btn' + i] = btn.classList.contains('active');
    }
  }

  // Coleta o estado dos botões do Joystick 2
  for (let i = 1; i <= 24; i++) {
    let btn = document.getElementById('j2_btn' + i);
    if (btn) { // Verifica se o botão existe
      data.joystick2.botoes['btn' + i] = btn.classList.contains('active');
    }
  }

  // Só envia se a conexão WebSocket estiver aberta
  if (websocket.readyState === WebSocket.OPEN) {
    websocket.send(JSON.stringify(data));
  }
}

// Adiciona listeners para os sliders (eixos)
document.addEventListener('DOMContentLoaded', () => {
  document.querySelectorAll('input[type="range"]').forEach(slider => {
    slider.addEventListener('input', () => {
      // Atualiza o texto do span com o valor atual do slider
      document.getElementById(slider.id + '_value').innerText = slider.value;
      sendData(); // Envia dados ao arrastar o slider
    });
  });

  // Adiciona listeners para os botões (clique toggle)
  // O event listener para os botões agora é adicionado após a geração dinâmica
  document.getElementById('j2-button-grid').addEventListener('click', (event) => {
    if (event.target.tagName === 'BUTTON') {
      event.target.classList.toggle('active'); // Alterna a classe 'active'
      sendData(); // Envia o novo estado
    }
  });

  document.getElementById('j2-button-grid').addEventListener('touchend', (event) => {
    if (event.target.tagName === 'BUTTON') {
      event.preventDefault(); // Evita o disparo duplo em alguns browsers (touch e click)
      event.target.classList.toggle('active');
      sendData();
    }
  }, { passive: false });

  // Listeners para os botões do Joystick Principal
  document.querySelectorAll('.joystick-panel:first-of-type .button-grid button').forEach(button => {
    button.addEventListener('click', () => {
      button.classList.toggle('active');
      sendData();
    });
    button.addEventListener('touchend', (e) => {
      if (e.target === button) {
        e.preventDefault();
        button.classList.toggle('active');
        sendData();
      }
    }, { passive: false });
  });
});


// Envia dados periodicamente para garantir que o ESP32 receba atualizações
setInterval(sendData, 500); // Envia a cada 500ms (meio segundo)