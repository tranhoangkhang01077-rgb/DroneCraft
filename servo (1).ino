#include <Arduino.h>
#include <SCServo.h>

// ===== Chân và servo =====
#define PWM_PIN    34      // xung PWM từ FC
#define SERVO_RX   16
#define SERVO_TX   17
#define SERVO_ID    1

SMS_STS st;

// ===== Góc kẹp =====
// Servo Feetech STS: 4096 bước = 360° (1° ≈ 11,38 bước)
const int   POS_ZERO    = 2048;   // vị trí raw ứng với 0° (MỞ) - hiệu chỉnh theo cơ khí thực tế
const int   DIR         = 1;      // đổi thành -1 nếu kẹp đóng sai chiều
const float ANGLE_OPEN  = 5.0f;   // độ
const float ANGLE_CLOSE = 60.0f;  // độ

const int SERVO_SPEED = 1800;
const int SERVO_ACC   = 30;

// true  = nhả torque sau khi di chuyển (servo không giữ cứng)
// false = giữ torque (nên dùng khi kẹp hàng, tránh rơi hàng)
const bool RELEASE_TORQUE_AFTER_MOVE = false;
const unsigned long MOVE_TIME_MS = 1500;

// ===== Ngưỡng PWM (us) =====
const int PWM_LOW_MAX  = 1400;   // < ngưỡng này -> MỞ
const int PWM_HIGH_MIN = 1600;   // > ngưỡng này -> ĐÓNG
const int PWM_VALID_MIN = 800;
const int PWM_VALID_MAX = 2200;

const int  STABLE_COUNT_READY = 10;  // số lần đọc ổn định để xác nhận trạng thái ban đầu
const int  STABLE_COUNT_MOVE  = 5;   // số lần đọc ổn định để ra lệnh
const unsigned long SIGNAL_TIMEOUT_MS = 500;

// ===== Trạng thái =====
int  candidateState = -1;
int  candidateCount = 0;
int  lastRCState    = -1;   // 0 = mở, 1 = đóng
bool rcReady        = false;
unsigned long lastValidMs = 0;
unsigned long lastPrintMs = 0;

bool torqueReleasePending = false;
unsigned long torqueOffDeadline = 0;

// ===== Hàm tiện ích =====
int angleToPos(float deg) {
  int pos = POS_ZERO + DIR * (int)(deg * 4096.0f / 360.0f + 0.5f);
  return constrain(pos, 0, 4095);
}

// 0 = mở, 1 = đóng, -1 = vùng giữa / không hợp lệ
int classifyPWM(int pwm) {
  if (pwm < PWM_VALID_MIN || pwm > PWM_VALID_MAX) return -1;
  if (pwm < PWM_LOW_MAX)  return 0;
  if (pwm > PWM_HIGH_MIN) return 1;
  return -1;
}

void moveGripper(int state) {
  float angle = (state == 0) ? ANGLE_OPEN : ANGLE_CLOSE;
  int target  = angleToPos(angle);

  Serial.printf("LENH: %s KEP -> %.0f do (raw %d)\n",
                state == 0 ? "MO" : "DONG", angle, target);

  st.EnableTorque(SERVO_ID, 1);
  delay(20);
  int result = st.WritePosEx(SERVO_ID, target, SERVO_SPEED, SERVO_ACC);
  Serial.printf("WritePosEx result = %d\n", result);

  if (RELEASE_TORQUE_AFTER_MOVE) {
    torqueOffDeadline = millis() + MOVE_TIME_MS;
    torqueReleasePending = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("==========================================");
  Serial.println("  DIEU KHIEN KEP: MO 0 DO / DONG 60 DO");
  Serial.println("==========================================");

  Serial2.begin(1000000, SERIAL_8N1, SERVO_RX, SERVO_TX);
  st.pSerial = &Serial2;

  pinMode(PWM_PIN, INPUT);

  int ping = st.Ping(SERVO_ID);
  Serial.printf("Servo Ping = %d %s\n", ping, ping >= 0 ? "(OK)" : "(KHONG PHAN HOI)");

  // Không gửi lệnh vị trí lúc khởi động để kẹp không tự di chuyển
  Serial.println("Cho PWM on dinh tu FC...");
  Serial.println("Go O + Enter: mo kep, C + Enter: dong kep (thu tay).");
}

void loop() {
  // 1. Thử bằng tay qua Serial Monitor
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'O' || cmd == 'o') moveGripper(0);
    else if (cmd == 'C' || cmd == 'c') moveGripper(1);
  }

  // 2. Đọc PWM (timeout 30 ms)
  int pwm = (int)pulseIn(PWM_PIN, HIGH, 30000);
  unsigned long nowMs = millis();

  bool pulseValid = (pwm >= PWM_VALID_MIN && pwm <= PWM_VALID_MAX);
  if (pulseValid) lastValidMs = nowMs;
  bool signalFresh = lastValidMs != 0 && (nowMs - lastValidMs) < SIGNAL_TIMEOUT_MS;

  // 3. Log định kỳ (mỗi 500 ms)
  if (nowMs - lastPrintMs >= 500) {
    lastPrintMs = nowMs;
    if (pwm == 0)          Serial.println("PWM = 0 us (KHONG CO XUNG)");
    else if (!pulseValid)  Serial.printf("PWM = %d us (NGOAI DAI RC - kiem tra FC co xuat PWM khong)\n", pwm);
    else                   Serial.printf("PWM = %d us | trang thai: %s\n", pwm,
                                         lastRCState == 0 ? "MO" : lastRCState == 1 ? "DONG" : "chua xac dinh");
  }

  // 4. Mất tín hiệu: giữ nguyên kẹp, không ra lệnh mới.
  //    Khi có lại tín hiệu phải xác nhận trạng thái ban đầu lại.
  if (!signalFresh) {
    if (rcReady) Serial.println("MAT TIN HIEU - giu nguyen kep");
    rcReady = false;
    candidateState = -1;
    candidateCount = 0;
    lastRCState = -1;
  }

  // 5. Phân loại và chống nhiễu
  if (pulseValid) {
    int state = classifyPWM(pwm);

    if (state == -1) {
      candidateState = -1;
      candidateCount = 0;
    } else {
      if (state == candidateState) candidateCount++;
      else { candidateState = state; candidateCount = 1; }

      // Ghi nhận trạng thái công tắc ban đầu, KHÔNG di chuyển
      if (!rcReady && candidateCount >= STABLE_COUNT_READY) {
        rcReady = true;
        lastRCState = candidateState;
        Serial.printf("RC SAN SANG - cong tac dang o muc %s, servo khong di chuyen\n",
                      lastRCState == 0 ? "MO" : "DONG");
      }
      // Chỉ ra lệnh khi trạng thái mới ổn định và khác trạng thái trước
      else if (rcReady && candidateCount >= STABLE_COUNT_MOVE && candidateState != lastRCState) {
        lastRCState = candidateState;
        moveGripper(lastRCState);
        candidateCount = 0;
      }
    }
  }

  // 6. Nhả torque sau khi di chuyển (nếu bật tùy chọn)
  if (torqueReleasePending && (long)(nowMs - torqueOffDeadline) >= 0) {
    st.EnableTorque(SERVO_ID, 0);
    torqueReleasePending = false;
    Serial.println("Da tat torque.");
  }
}