let socket;

function connect() {
  socket = new WebSocket("ws://" + location.host + "/ws");
  socket.binaryType = "arraybuffer";
  socket.onmessage = onTelemetry;
  socket.onclose = function () {
    document.getElementById("status").innerHTML = "--";
    setTimeout(connect, 1000);
  };
}

function onTelemetry(event) {
  let data = new DataView(event.data);

  let sequence  = data.getUint32(4, true);
  let angle     = data.getFloat32(12, true);
  let target    = data.getFloat32(16, true);
  let flags     = data.getUint8(20);

  let encoderValid = (flags & 0x01) !== 0;
  let motorMoving  = (flags & 0x02) !== 0;
  let stalled      = (flags & 0x04) !== 0;
  let fault        = (flags & 0x08) !== 0;
  let lockout      = (flags & 0x10) !== 0;

  document.getElementById("angle").innerHTML = angle.toFixed(2) + "&deg;";
  document.getElementById("seq").innerHTML = sequence;
  document.getElementById("targetDisplay").innerHTML = target.toFixed(2);
  document.getElementById("status").innerHTML = encoderValid ? "OK" : "invalid";

  let state = motorMoving ? "moving" : "idle";
  if (stalled) state = "stalled";
  if (fault)   state = "fault";
  if (lockout) state = "locked out";

  let movingEl = document.getElementById("moving");
  movingEl.innerHTML = state;
  movingEl.className = state === "idle" ? "moving-no" : "moving-yes";
}

function sendTarget() {
  let input = document.getElementById("targetInput");
  let val = parseFloat(input.value);
  if (!isNaN(val) && socket.readyState === WebSocket.OPEN) {
    socket.send(val.toString());
  }
}

document.getElementById("sendBtn").addEventListener("click", sendTarget);
document.getElementById("targetInput").addEventListener("keydown", function (e) {
  if (e.key === "Enter") sendTarget();
});

connect();
