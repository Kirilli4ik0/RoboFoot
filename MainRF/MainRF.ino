class Timer {
  long period;
  mutable long last;
  bool chk() const {
    long tme = millis();
    if (abs(tme - last) > period) {
      last = tme;
      return true;
    }
    return false;
  }
public:
  Timer(long period)
    : period(period) {
    last = millis();
  };
  void reset() {
    last = millis();
  }
  operator bool() const {
    return chk();
  }
};


volatile int cnt1 = 0;
volatile int cnt2 = 0;
volatile int cnt3 = 0;
volatile int cnt4 = 0;

void inter1() {
  if (digitalRead(38)) cnt1--;
  else cnt1++;
}
void inter2() {
  if (digitalRead(36)) cnt2++;
  else cnt2--;
}
void inter3() {
  if (digitalRead(A2)) cnt3--;
  else cnt3++;
}
void inter4() {
  if (!digitalRead(A0)) cnt4--;
  else cnt4++;
}

void setupEncoders() {
  pinMode(38, INPUT);
  pinMode(36, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(18, INPUT);
  pinMode(20, INPUT);

  attachInterrupt(digitalPinToInterrupt(2), inter1, RISING);
  attachInterrupt(digitalPinToInterrupt(3), inter2, RISING);
  attachInterrupt(digitalPinToInterrupt(18), inter3, RISING);
  attachInterrupt(digitalPinToInterrupt(20), inter4, RISING);
}
Timer print(50);

struct Motor {
  const int portA, portB, portPWM;
  int tar;
  volatile int* cnt;

  float Kp = 25;
  float Kd = 0;
  float Ki = 0.5;
  float integral = 0;
  float derivative = 0;
  int difLst = 0;


  Timer compoundTimer{ 20 };

  Motor(int portA, int portB, int portPWM, volatile int* cnt)
    : portA(portA), portB(portB), portPWM(portPWM), cnt(cnt) {
    pinMode(portA, OUTPUT);
    pinMode(portB, OUTPUT);
    pinMode(portPWM, OUTPUT);
  }

  void move(int spd) {
    digitalWrite(portA, spd > 0);
    digitalWrite(portB, spd < 0);
    analogWrite(portPWM, min(abs(spd), 255));
  }

  void tick() {
    if (compoundTimer) {
      // Serial.println(*cnt - tar);

      int pos = *cnt;
      int dif = (tar - pos);

      derivative = derivative * 0.9 + (dif - difLst) * 0.1;

      float currKd = Kd;  // * (1 - min(1,abs(dif)/50));

      move(dif * Kp + Ki * integral + currKd * derivative);

      // if (print && millis() < 5000) {
      //   Serial.print(*cnt);
      //   Serial.print(" ");
      //   Serial.print(dif * Kp);
      //   Serial.print(" ");
      //   Serial.print(Ki * integral);
      //   Serial.print(" ");
      //   Serial.print(dif * Kp + Ki * integral + currKd * derivative);
      //   Serial.print(" ");
      //   Serial.println(currKd * derivative);
      // }

      integral += dif;
      difLst = dif;
    }
  }
};

Motor mot1A(41, 40, 6, &cnt1);  //M1 side
Motor mot1B(42, 43, 7, &cnt2);  //M2 fw

Motor mot2A(A5, A4, 4, &cnt3);    //M3 side
Motor mot2B(A10, A11, 8, &cnt4);  //M4 fw


void movePos(float pos) {
  pos *= 5.6;
  pos = min(150, max(0, pos));
  int offs1 = 50;
  int offs2 = 100;

  if (pos > offs2) pos -= offs2;
  else if (pos > offs1) pos -= offs1;

  mot1A.tar = pos;
}

void movePos2(float pos) {
  pos *= 5.6;
  pos = min(100, max(50, pos));

  mot2A.tar = pos - 50;
}

void setupCoeficients() {
  mot1B.Kp = 7;
  mot1B.Kd = -5;
  mot1B.Ki = 0.1;

  mot2B.Kp = 15;
  mot2B.Kd = -5;
  mot2B.Ki = 0.1;
}

bool isCameraView = false;


void checkSerial() {
  String s = "S3:";
  Serial.println("check Serail");
  while (Serial3.available() > 0) {
    s += (char)Serial3.read();
  }
  Serial.println(s);
  s = "S2:";
  Serial.println("check Serail");
  while (Serial2.available() > 0) {
    s += (char)Serial2.read();
  }
  Serial.println(s);
}

int pos_x = 0;
int pos_y = 0;

bool err_x = true;
bool err_y = true;

int checkPos() {
  //Serial.println("check Serail");
  String s = "";
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();
    if (s.length() != 0 || c == 'c')
      s += c;
  }
  s.trim();
  //Serial.println("S2---" + s);
  if (s.substring(0, 6) == "camera" && s.length() > 8) {
    int spacePos = s.indexOf('\n', 8);
    if (spacePos != -1) {
      err_x = (s.substring(spacePos + 1) != "ok");
      // if (!err_x)
      pos_x = (pos_x * 9 + s.substring(8, spacePos).toInt()) / 10;
    }
  } else if (s.length() > 0)
    ;
  // Serial.println("cam1" + s + "!");
  // Serial.println("S2---" + String(err_x) + " " + String(pos_x));

  s = "";
  while (Serial3.available() > 0) {
    char c = (char)Serial3.read();
    if (s.length() != 0 || c == 'c')
      s += c;
  }
  s.trim();
  if (s.substring(0, 6) == "camera" && s.length() > 8) {
    int spacePos = s.indexOf('\n', 8);
    if (spacePos != -1) {
      err_y = (s.substring(spacePos + 1) != "ok");
      // if (!err_y)
      pos_y = (pos_y * 9 + s.substring(8, spacePos).toInt()) / 10;
    }
    // Serial.println("cam2!");
  } else if (s.length() > 0)
    ;
  // Serial.println("cam2" + s + "!");
}

int glPx;

void calculate(float pos1, float pos2) {  // pos1 = pos2 = 400;
  Serial.println("calculate " + String(pos1) + " " + String(pos2));
  float Ga = 90.0 - pos1 * 0.113;  //90/800
  float Gb = pos2 * 0.113;
  float Gc = 180.0 - Ga - Gb;
  // Serial.println(String(Ga) + " " + String(Gb) + " " + String(Gc));
  float c = 470;
  Ga = sin(Ga * DEG_TO_RAD);
  Gc = sin(Gc * DEG_TO_RAD);
  Gb = sin(Gb * DEG_TO_RAD);
  // Serial.println(String(Ga) + " " + String(Gb) + " " + String(Gc));
  float a = sin(Ga * DEG_TO_RAD) * c / sin(Gc * DEG_TO_RAD);
  float b = sin(Gb * DEG_TO_RAD) * c / sin(Gc * DEG_TO_RAD);
  //Serial.println(String(a) + " " + String(b) + " " + String(c) );
  float pp = (a + b + c) / 2;
  float ss = sqrt(pp * (pp - a) * (pp - b) * (pp - c));
  int x = (int)(2 * ss / c);    //mm
  int y = sqrt(b * b - x * x);  //mm
  glPx = x;
  Serial.println(String(x) + " " + String(y));
}

void setup() {
  delay(1000);
  Serial3.begin(115200);
  Serial2.begin(115200);
  Serial.begin(115200);
  delay(1000);

  if (isCameraView) {
    Serial3.println("debug");
    Serial2.println("debug");
  } else {
    Serial3.println("rel");
    Serial2.println("rel");
  }
  setupCoeficients();
  setupEncoders();

  // mot2B.move(-200);
  // mot1B.move(-200);

  delay(1500);
  mot2A.move(-255);
  mot1A.move(-255);
  delay(800);
  *mot1A.cnt = 0;
  *mot2A.cnt = 0;
  mot1A.move(0);
  mot2A.move(0);
  // // delay(1000);

  // mot2A.tar = 30;
  // mot2B.tar = 3;
  // mot1B.tar = 0;


  // mot1B.tar = 0;
}

Timer punch(4000);

Timer stop(300);

float state = 4;

Timer stopPWM(1);
int stopCnt = 0;





void loop() {
  //checkSerial();
  checkPos();
  // Serial.println("err:" + String(err_x) + " " + String(err_y));
  //if (!err_x && !err_y)
  calculate(pos_x, pos_y);
  mot1A.tick();
  mot2A.tick();
  // mot2B.tick();.
  mot1B.tick();
  // movePos(((long)millis() - 5000) / 100);
  movePos((glPx - 30) / 5 * 3);
  movePos2((glPx - 30) / 5 * 3);
  Serial.println((glPx - 30) / 5 * 3);
  delay(20);
  return;
  //mot1A.tick(); // hold position
  //movePos(((long)millis() - 5000) / 100); move horisontally
  //return;
  if (state == 0) {  // start punch
    mot1B.move(255);
    stop.reset();
    state = 0.5;
  }
  if (state == 0.5 && stop) state = 1;  // wait for punch
  if (state == 1 && stopPWM) {
    if (stopCnt == 80) {  // stopping
      stopCnt = 0;
      state = 2;
      punch.reset();
      mot1B.move(0);
      mot1B.tar = *mot1B.cnt + 60 - ((*mot1B.cnt + 60) % 45);  // put vertical target
    } else {
      if (stopCnt % 2 == 0) mot1B.move(255);
      else mot1B.move(-255);
      stopCnt++;
    }
  }

  if (state == 2) {  // move to vertical poition
    Serial.println(*mot1B.cnt - mot1B.tar);
    mot1B.tick();
    if (punch) state = 0;
  }
}
