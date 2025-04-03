// === 상태 출력용 명령 전송 함수 ===
function sendCommand(command) {
  console.log("보낸 명령:", command);
  const status = document.getElementById("status-text");
  if (status) {
    status.innerText = `명령 전송됨: ${command}`;
  }
  // TODO: WebSocket 또는 HTTP 요청 등 통신 구현 예정
}

// === CLI 관련 ===
let cliHistory = [];
let historyIndex = -1;

function runCLI() {
  const cliInput = document.getElementById("cli-input");
  const cliLog = document.getElementById("cli-log");

  const command = cliInput.value.trim();
  if (command === "") return;

  cliHistory.push(command);
  historyIndex = cliHistory.length;

  logCLI(`> ${command}`);

  // 예시 명령 처리
  if (command === "auto start") {
    logCLI("구현 안 됨 ㅋㅋ");
  } else {
    logCLI("(가상 처리됨)");
  }

  cliInput.value = "";
}

function logCLI(text) {
  const cliLog = document.getElementById("cli-log");
  cliLog.innerHTML += `${text}<br>`;
  cliLog.scrollTop = cliLog.scrollHeight;
}

// === 키보드 이벤트 처리 ===
function setupKeyboardControl() {
  const toggle = document.getElementById("keyboard-toggle");

  document.addEventListener("keydown", function (event) {
    if (!toggle?.checked) return;

    const key = event.key.toLowerCase();
    if (key === "w") sendCommand("forward");
    else if (key === "s") sendCommand("backward");
    else if (key === "a") sendCommand("left");
    else if (key === "d") sendCommand("right");
    else if (key === " ") sendCommand("stop");
  });
}

function setupCLIKeyboard() {
  const cliInput = document.getElementById("cli-input");

  cliInput.addEventListener("keydown", (e) => {
    if (e.key === "Enter") {
      e.preventDefault();
      runCLI();
    } else if (e.key === "ArrowUp") {
      e.preventDefault();
      if (historyIndex > 0) {
        historyIndex--;
        cliInput.value = cliHistory[historyIndex];
      }
    } else if (e.key === "ArrowDown") {
      e.preventDefault();
      if (historyIndex < cliHistory.length - 1) {
        historyIndex++;
        cliInput.value = cliHistory[historyIndex];
      } else {
        historyIndex = cliHistory.length;
        cliInput.value = "";
      }
    }
  });
}

// === 초기 모델 선택 처리 ===
function confirmModel() {
  const model = document.getElementById("modelSelect").value;
  if (model) {
    document.getElementById("modelOverlay").style.display = "none";
    document.getElementById("mainContent").classList.remove("blurred");
  } else {
    alert("모델을 선택해주세요.");
  }
}

// === 초기화 함수 ===
function init() {
  document.getElementById("mainContent").classList.add("blurred");
  setupKeyboardControl();
  setupCLIKeyboard();
}

// === DOM 로드 이후 실행 ===
document.addEventListener("DOMContentLoaded", init);
