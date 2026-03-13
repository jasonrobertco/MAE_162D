#include "InterruptAndTimerSetup.h"

//////////////////////////////////////Parameters for DC Motor 1/////////////////////////////////////
#define Motor1_ENA_PIN 6
#define Motor1_IN1_PIN 8
#define Motor1_IN2_PIN 43
DC_Motor Motor1;
#define Encoder1_PhaseA_Pin 18 // must be 2, 3, 18, or 19
#define Encoder1_PhaseB_Pin 19 // must be 2, 3, 18, or 19
float Motor1Pos;
//////////////////////////////////////Parameters for DC Motor 2 /////////////////////////////////////
#define Motor2_ENA_PIN 7
#define Motor2_IN1_PIN 30
#define Motor2_IN2_PIN 4
DC_Motor Motor2;
#define Encoder2_PhaseA_Pin 3 // must be 2, 3, 18, or 19
#define Encoder2_PhaseB_Pin 2 // must be 2, 3, 18, or 19
float Motor2Pos;
//////

float ISR1_dt = 0.01;

// Test phases:
// Phase 1 (0-3s):  Motor1 forward at 50% speed
// Phase 2 (3-6s):  Motor1 stopped
// Phase 3 (6-9s):  Motor2 forward at 50% speed
// Phase 4 (9-12s): Motor2 stopped
// Phase 5 (12-15s): Both motors forward at 50% speed
// Phase 6 (15+s):  Both motors stopped
int lastPhase = -1;

void setup() {
  Serial.begin(115200);
  DC_Motor_Init(&Motor1, Motor1_IN1_PIN, Motor1_IN2_PIN, Motor1_ENA_PIN);
  Encoder1ExtISRSetup(Encoder1_PhaseA_Pin, Encoder1_PhaseB_Pin, &Motor1.CurrentEncoderCount);
  DC_Motor_Init(&Motor2, Motor2_IN1_PIN, Motor2_IN2_PIN, Motor2_ENA_PIN);
  Encoder2ExtISRSetup(Encoder2_PhaseA_Pin, Encoder2_PhaseB_Pin, &Motor2.CurrentEncoderCount);
  delay(100);
  HighPriorityInterruptInit(ISR1, 1/ISR1_dt);
  Serial.println("=== Motor & Encoder Direction Test ===");
  Serial.println("Check that encoder counts INCREASE when each motor spins FORWARD.");
  Serial.println("If encoder counts decrease, swap Phase A/B pins for that motor.");
  Serial.println("If a motor spins backward, swap IN1/IN2 pins for that motor.");
  Serial.println("Phase\tM1_Enc\tM2_Enc\tM1_Cmd\tM2_Cmd");
}

void loop() {
  float t = millis() / 1000.0;
  int phase;
  float m1cmd = 0;
  float m2cmd = 0;

  if (t < 3) {
    phase = 1; m1cmd = 0.5; m2cmd = 0;
  } else if (t < 6) {
    phase = 2; m1cmd = 0; m2cmd = 0;
  } else if (t < 9) {
    phase = 3; m1cmd = 0; m2cmd = 0.5;
  } else if (t < 12) {
    phase = 4; m1cmd = 0; m2cmd = 0;
  } else if (t < 15) {
    phase = 5; m1cmd = 0.5; m2cmd = 0.5;
  } else {
    phase = 6; m1cmd = 0; m2cmd = 0;
  }

  if (phase != lastPhase) {
    lastPhase = phase;
    switch (phase) {
      case 1: Serial.println("\n--- Phase 1: Motor1 (RIGHT) forward only ---"); break;
      case 2: Serial.println("\n--- Phase 2: Stopped ---"); break;
      case 3: Serial.println("\n--- Phase 3: Motor2 (LEFT) forward only ---"); break;
      case 4: Serial.println("\n--- Phase 4: Stopped ---"); break;
      case 5: Serial.println("\n--- Phase 5: Both motors forward ---"); break;
      case 6: Serial.println("\n--- Phase 6: Done - All stopped ---"); break;
    }
  }

  Serial.print(phase);
  Serial.print("\t");
  Serial.print(Motor1.CurrentEncoderCount);
  Serial.print("\t");
  Serial.print(Motor2.CurrentEncoderCount);
  Serial.print("\t");
  Serial.print(m1cmd);
  Serial.print("\t");
  Serial.println(m2cmd);

  DC_Motor_Go(&Motor1, m1cmd);
  DC_Motor_Go(&Motor2, m2cmd);

  delay(100);
}

void ISR1(void) {
  Motor1Pos = DC_Motor_GetPos(&Motor1);
  Motor2Pos = DC_Motor_GetPos(&Motor2);
}
