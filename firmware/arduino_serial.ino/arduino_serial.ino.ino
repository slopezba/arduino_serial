/*
  Arduino ADC + Digital Outputs + JSON Serial Protocol

  - ADC A0-A3 -> JSON
  - Digital outputs: D8, D9
  - All outputs start LOW
  - Commands via JSON:
      {"cmd":"set_output","pin":8,"value":1}
      {"cmd":"set_period","period_ms":100}
*/

const unsigned long BAUDRATE = 115200;

// ADC
unsigned long period_ms = 50;
unsigned long last_ms = 0;

// Digital outputs
const int OUT1 = 8;
const int OUT2 = 9;

void setup() {
  Serial.begin(BAUDRATE);

  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);

  // Estado seguro
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
}

void handle_command(String cmd) {
  cmd.trim();

  // -------- SET OUTPUT --------
  if (cmd.indexOf("\"cmd\"") >= 0 && cmd.indexOf("set_output") >= 0) {

    int pin = -1;
    int value = 0;

    int pin_idx = cmd.indexOf("\"pin\"");
    int val_idx = cmd.indexOf("\"value\"");

    if (pin_idx >= 0 && val_idx >= 0) {
      pin = cmd.substring(cmd.indexOf(":", pin_idx) + 1).toInt();
      value = cmd.substring(cmd.indexOf(":", val_idx) + 1).toInt();

      if (pin == 8 || pin == 9) {
        digitalWrite(pin, value ? HIGH : LOW);
      }
    }
  }

  // -------- SET PERIOD --------
  if (cmd.indexOf("set_period") >= 0) {
    int p_idx = cmd.indexOf("period_ms");
    if (p_idx >= 0) {
      unsigned long new_period =
        cmd.substring(cmd.indexOf(":", p_idx) + 1).toInt();

      if (new_period >= 5 && new_period <= 5000) {
        period_ms = new_period;
      }
    }
  }
}

void loop() {
  // ---------- SERIAL INPUT ----------
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handle_command(cmd);
  }

  // ---------- ADC OUTPUT ----------
  unsigned long now = millis();
  if (now - last_ms < period_ms) return;
  last_ms = now;

  int a0 = analogRead(A0);
  int a1 = analogRead(A1);
  int a2 = analogRead(A2);
  int a3 = analogRead(A3);

  Serial.print("{\"A0\":");
  Serial.print(a0);
  Serial.print(",\"A1\":");
  Serial.print(a1);
  Serial.print(",\"A2\":");
  Serial.print(a2);
  Serial.print(",\"A3\":");
  Serial.print(a3);
  Serial.print("}\n");
}
