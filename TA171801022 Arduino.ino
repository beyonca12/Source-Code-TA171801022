 // Library
#include <Servo.h>

// Define Pin Pengendali Motor
#define DIR_YAW   5
#define DIR_ROLL  6
#define DIR_PITCH 7

// Define Pin ESC Motor
Servo esc_signal_yaw;
Servo esc_signal_roll;
Servo esc_signal_pitch;

// Variabel Komunikasi
String yaw;
String roll;
String pitch;
String state;
String junk;
float yaw2,roll2,pitch2;
float yaw3,roll3,pitch3;
int state2;

// Variabel Motor
float error_yaw;
float error_roll;
float error_pitch;
float angle_yaw;
float angle_roll;
float angle_pitch;
float angle_ref_yaw;
float angle_ref_roll;
float angle_ref_pitch;
float lastangle_yaw;
float lastangle_roll;
float lastangle_pitch;

// Variabel PID
float kP = 2;
float kI = 0.5;
float kD = 0.2;
float P_yaw, P_roll, P_pitch;
float I_yaw, I_roll, I_pitch;
float D_yaw, D_roll, D_pitch;
float pid_yaw, pid_roll, pid_pitch;
long lastProcess;

// State Raspi dan Motor
int i = 0;

// Variabel acak
int k = 0;

// Setup Baudrate
void setup() {
  // Pin Motor
  pinMode(DIR_YAW,OUTPUT);
  pinMode(DIR_ROLL,OUTPUT);
  pinMode(DIR_PITCH,OUTPUT);
  // Setup Motor
  esc_signal_yaw.attach(9);
  esc_signal_yaw.writeMicroseconds(1000);
  esc_signal_roll.attach(10);
  esc_signal_roll.writeMicroseconds(1000);
  esc_signal_pitch.attach(11);
  esc_signal_pitch.writeMicroseconds(1000);
  delay(3000);
  // Setup Serial
  Serial.begin(38400);
  Serial.println("Waiting for Raspberry Pi to send a signal...\n");
}

// Looping
void loop() {
    // Looping Serial
    while (Serial.available() > 0)
    {        
      
//  	  // Substring
      yaw = Serial.readStringUntil('a');
      Serial.read();
	    junk = Serial.readStringUntil('x');
      Serial.read();
      roll = Serial.readStringUntil('b');
      Serial.read();
      pitch = Serial.readStringUntil('c');
      Serial.read();
      state = Serial.readStringUntil('\0');
      
  	  // Menyimpan Nilai Yaw, Roll, Pitch
      if (state == "0")
      {
        yaw2 = yaw.toFloat();
        roll2 = roll.toFloat();
        pitch2 = pitch.toFloat();
      }
      if (state == "1")
      {
        yaw3 = yaw.toFloat();
        roll3 = roll.toFloat();
        pitch3 = pitch.toFloat();  
      }
      // Ganti State
      i=1;
    }

    if (i==1)
    {
      // Motor      
      motor();
      i=0;
    }
}

void motor()
{
  // Nilai Yaw Roll Pitch Sensor
  angle_yaw = yaw3;
  angle_roll = roll3;
  angle_pitch = pitch3;

//  Serial.print("Angle Yaw = ");
//  Serial.print(angle_yaw);
//  Serial.print("\t");
//  Serial.print("Angle Roll = ");
//  Serial.print(angle_roll);
//  Serial.print("\t");
//  Serial.print("Angle Pitch = ");
//  Serial.print(angle_pitch);
//  Serial.print("\t");

  // Nilai Yaw Roll Pitch Referensi
  angle_ref_yaw = yaw2;
  angle_ref_roll = roll2;
  angle_ref_pitch = pitch2;

  // Selisih Sudut Yaw Roll Pitch
  if (angle_yaw > 0)
  {error_yaw = (angle_yaw - angle_ref_yaw);}
  else
  {error_yaw = (angle_ref_yaw - angle_yaw);}
  
  if (angle_roll > 0)
  {error_roll = (angle_roll - angle_ref_roll);}
  else
  {error_roll = (angle_ref_roll - angle_roll);}

  if (angle_pitch > 0)
  {error_pitch = (angle_pitch - angle_ref_pitch);}
  else
  {error_pitch = (angle_ref_pitch - angle_pitch);}

  // Hitung Selisih Waktu
  float deltaTime = (millis() - lastProcess) / 1000.0;
  lastProcess = millis();
    
  // Hitung P
  P_yaw = error_yaw * kP;
  P_roll = error_roll * kP;
  P_pitch = error_pitch * kP;
    
  // Hitung I
  I_yaw = I_yaw + (error_yaw * kI) * deltaTime;
  I_roll = I_roll + (error_roll * kI) * deltaTime;
  I_pitch = I_pitch + (error_pitch * kI) * deltaTime;
    
  // Hitung D
  D_yaw = (lastangle_yaw - angle_yaw) * kD / deltaTime;
  D_roll = (lastangle_roll - angle_roll) * kD / deltaTime;
  D_pitch = (lastangle_pitch - angle_pitch) * kD / deltaTime;
  
  lastangle_yaw = angle_yaw;
  lastangle_roll = angle_roll;
  lastangle_pitch = angle_pitch;

  // Hitung PID Yaw Roll Pitch
  pid_yaw = P_yaw + I_yaw + D_yaw;
  pid_roll = P_roll + I_roll + D_roll;
  pid_pitch = P_pitch + I_pitch + D_pitch;

  // Scaling PID
  pid_yaw = pid_yaw * 200;
  pid_roll = pid_roll * 200;
  pid_pitch = pid_pitch * 200;

  // Batasan Nilai PID
  pid_yaw = constrain(pid_yaw, -3000, 3000);
  pid_roll = constrain(pid_roll, -1250, 1250);
  pid_pitch = constrain(pid_pitch, -1250, 1250);

//  Serial.print("pid Yaw = ");
//  Serial.print(pid_yaw);
//  Serial.print("\t");
//  Serial.print("pid Roll = ");
//  Serial.print(pid_roll);
//  Serial.print("\t");
//  Serial.print("pid Pitch = ");
//  Serial.print(pid_pitch);
//  Serial.print("\t");

  // Motor Yaw
  if (angle_ref_yaw - angle_yaw > 0)
  {
    digitalWrite(DIR_YAW, LOW) ;
    esc_signal_yaw.writeMicroseconds(pid_yaw + k);
    delay(10);
//    Serial.println("Arah 1 Yaw");
  }
  else
  {
    digitalWrite(DIR_YAW, HIGH);
    esc_signal_yaw.writeMicroseconds(pid_yaw + k);
    delay(10);
//    Serial.println("Arah 2 Yaw");
  }

  // Motor Roll
  if (angle_ref_roll - angle_roll > 0)
  {
    digitalWrite(DIR_ROLL, LOW) ;
    esc_signal_roll.writeMicroseconds(pid_roll + k);
    delay(10);
//    Serial.println("Arah 1 Roll");
  }
  else
  {
    digitalWrite(DIR_ROLL, HIGH);
    esc_signal_roll.writeMicroseconds(pid_roll + k);
    delay(10);
//    Serial.println("Arah 2 Roll");
  }

  // Motor Pitch
  if (angle_ref_pitch - angle_pitch > 0)
  {
    digitalWrite(DIR_PITCH, LOW) ;
    esc_signal_pitch.writeMicroseconds(pid_pitch + k);
    delay(10);
//    Serial.println("Arah 1 Pitch");
  }
  else
  {
    digitalWrite(DIR_PITCH, HIGH);
    esc_signal_pitch.writeMicroseconds(pid_pitch + k);
    delay(10);
//    Serial.println("Arah 2 Pitch");
  }
  
  k = k * -1;
//  Serial.print(deltaTime);
//  Serial.println(" ");
  delay(40);
}
