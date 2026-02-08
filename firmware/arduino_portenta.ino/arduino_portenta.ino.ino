#include <Ethernet.h>

// ---------------- CONFIGURACIÓN ----------------
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 50);
EthernetServer server(80);

// Pines PERMITIDOS
int allowed_pins[] = {2, 3, 4, 5, 6, 7};
const int N_ALLOWED = sizeof(allowed_pins) / sizeof(allowed_pins[0]);
// ------------------------------------------------

// Comprueba si un pin está permitido
bool is_allowed(int pin) {
  for (int i = 0; i < N_ALLOWED; i++) {
    if (allowed_pins[i] == pin) return true;
  }
  return false;
}

void setup() {
  Ethernet.begin(mac, ip);
  server.begin();

  for (int i = 0; i < N_ALLOWED; i++) {
    pinMode(allowed_pins[i], OUTPUT);
    digitalWrite(allowed_pins[i], LOW);
  }
}

void loop() {
  EthernetClient client = server.available();
  if (!client) return;

  String req = client.readStringUntil('\r');
  client.flush();

  int pin = -1;
  bool set_on = false;
  bool set_off = false;

  // Parse simple: /digital/<pin>/on|off
  int idx = req.indexOf("/digital/");
  if (idx >= 0) {
    int p_start = idx + 9;
    int p_end = req.indexOf('/', p_start);
    pin = req.substring(p_start, p_end).toInt();

    if (req.indexOf("/on") >= 0)  set_on = true;
    if (req.indexOf("/off") >= 0) set_off = true;
  }

  bool success = false;
  String message = "invalid_request";

  if (pin >= 0 && is_allowed(pin)) {
    if (set_on) {
      digitalWrite(pin, HIGH);
      success = true;
      message = "on";
    }
    if (set_off) {
      digitalWrite(pin, LOW);
      success = true;
      message = "off";
    }
  } else if (pin >= 0) {
    message = "pin_not_allowed";
  }

  // -------- JSON RESPONSE --------
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();

  client.print("{");
  client.print("\"success\":"); client.print(success ? "true" : "false"); client.print(",");
  client.print("\"pin\":"); client.print(pin); client.print(",");
  client.print("\"state\":");

  if (pin >= 0 && is_allowed(pin)) {
    client.print(digitalRead(pin) == HIGH ? "\"on\"" : "\"off\"");
  } else {
    client.print("\"unknown\"");
  }

  client.print(",");
  client.print("\"message\":\""); client.print(message); client.print("\"");
  client.print("}");

  delay(1);
  client.stop();
}
