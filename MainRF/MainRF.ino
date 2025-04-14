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
  pinMode(A1, INPUT);

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

class Motor {
public:
  const int portA, portB, portPWM;
  int tar;
  volatile int* cnt;

  float Kp = 5;
  float Kd = 0;
  float Ki = 0.1;
  float integral = 0;
  float derivative = 0;
  int difLst = 0;

  int offset = 0;
  bool lastEnd = false;
  int pinEnd;

  Timer compoundTimer{ 20 };

  Motor(Motor&) = delete;
  void operator=(Motor&) = delete;

  Motor(int portA, int portB, int portPWM, volatile int* cnt, int pinEnd, int offset)
    : portA(portA), portB(portB), portPWM(portPWM), cnt(cnt), offset(offset), pinEnd(pinEnd) {
    pinMode(portA, OUTPUT);
    pinMode(portB, OUTPUT);
    pinMode(portPWM, OUTPUT);
    pinMode(pinEnd, INPUT);
  }

  void move(int spd) {
    digitalWrite(portA, spd > 0);
    digitalWrite(portB, spd < 0);
    if (spd != 0)
      spd = abs(spd) + 125;
    analogWrite(portPWM, min(spd, 255));
  }

  void recalcPos() {
    bool curEnd = digitalRead(pinEnd);
    if (lastEnd != curEnd) {
      Serial.println("END corrected from " + String(*cnt) + " to " + String(offset));
      *cnt = offset;
    }
    lastEnd = curEnd;
  }

  void tick() {
    recalcPos();

    if (compoundTimer) {
      // Serial.println(*cnt - tar);

      int pos = *cnt;
      int dif = (tar - pos);

      derivative = derivative * 0.9 + (dif - difLst) * 0.1;

      float currKd = Kd;  // * (1 - min(1,abs(dif)/50));

      integral = constrain(integral, -1000, 1000);

      if (abs(dif) <= 1 && derivative < 0.2)
        move(0);
      else
        move(dif * Kp + Ki * integral + currKd * derivative);


      // if (dif < 0)
      //   move(-50);
      // else if (dif > 0)
      //   move(50);
      // else move(0);
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

//-------------------------------------------------------------------------------------------------
Motor mot1A(41, 40, 6, &cnt1, 30, 12);  //M2 side
Motor mot1B(42, 43, 7, &cnt2, 27, 0);   //M1 round

Motor mot2A(A5, A4, 4, &cnt3, 29, 13);   //M4 side  vorota
Motor mot2B(A10, A11, 8, &cnt4, 26, 0);  //M3 round  vorota
//------------------------------  -------------------------------------------------------------------

void movePos(float pos) {
  int offs1 = 110;
  int offs2 = 220;

  if (pos > offs2) pos = map(constrain(pos, 0, 110), 0, 110, 0, 57);
  else if (pos > offs1) pos = map(constrain(pos, 110, 220), 110, 220, 0, 57);
  else pos = map(constrain(pos, 220, 330), 220, 330, 0, 57);

  // Serial.println("movePos1 " + String(pos));
  mot1A.tar = pos;
}

void movePos2(float pos) {
  //pos *= 5.6;
  // pos = min(80, max(50, pos));

  pos = map(constrain(pos, 120, 200), 120, 200, 0, 57);

  //Serial.println("movePos2 " + String(pos));
  mot2A.tar = pos;
}

void setupCoeficients() {
  mot1B.Kp = 7;
  mot1B.Kd = -5;
  mot1B.Ki = 0.1;

  mot2B.Kp = 15;
  mot2B.Kd = -5;
  mot2B.Ki = 0.1;
}


void checkSerial() {
  String s = "check Serail S3: ";
  int av = 0;
  while (Serial3.available() > 0) {
    s += (char)Serial3.read();
    av++;
  }
  if (av > 0) Serial.println(s);
  s = "check Serail S2: ";
  av = 0;
  while (Serial2.available() > 0) {
    s += (char)Serial2.read();
    av++;
  }
  if (av > 0) Serial.println(s);
}

int pos_x = 0;
int pos_y = 0;

bool err_x = true;
bool err_y = true;

int diff_x = true;
int diff_y = true;

int x = 0;
int y = 0;


void readPos() {
  delay(5);
  //Serial.println("check Serail");
  String s = "";
  while (Serial2.available() > 0) {
    char c = (char)Serial2.read();
    if (s.length() != 0 || c == 'c')
      s += c;
  }
  s.trim();
  //if (s.length()>0) Serial.println("S2---" + s);
  if (s.substring(0, 6) == "camera" && s.length() > 8) {
    //Serial.println("S2---" + s);
    int spacePos1 = s.indexOf(' ', 7);
    if (spacePos1 != -1) {
      int spacePos2 = s.indexOf(' ', spacePos1 + 1);
      if (spacePos1 != -1) {
        err_x = s.substring(spacePos2 + 1) != "ok";
        if (!err_x) {
          diff_x = s.substring(spacePos1 + 1, spacePos2).toInt();
          pos_x = s.substring(7, spacePos1).toInt();
        }
      }
      //Serial.println("pos_x=" + String(pos_x) + "err= " + String(err_x));
      //pos_x = (pos_x * 9 + s.substring(8, spacePos).toInt()) / 10;
    }
  }
  //Serial.println("cam1" + s + "!");
  //Serial.println("S2---" + String(err_x) + " " + String(pos_x));

  s = "";
  while (Serial3.available() > 0) {
    char c = (char)Serial3.read();
    if (s.length() != 0 || c == 'c')
      s += c;
  }
  s.trim();
  //if (s.length()>0) Serial.println("S3---" + s);
  if (s.substring(0, 6) == "camera" && s.length() > 8) {
    //Serial.println("S3---" + s);
    int spacePos1 = s.indexOf(' ', 7);
    if (spacePos1 != -1) {
      int spacePos2 = s.indexOf(' ', spacePos1 + 1);
      if (spacePos1 != -1) {
        err_y = s.substring(spacePos2 + 1) != "ok";
        if (!err_y) {
          diff_y = s.substring(spacePos1 + 1, spacePos2).toInt();
          pos_y = s.substring(7, spacePos1).toInt();
        }
      }
      //pos_y = (pos_y * 9 + s.substring(8, spacePos).toInt()) / 10;
    }
  }
  //Serial.println("cam2" + s + "!");
}

void calculate(float pos1, float pos2) {
  float add = 1;
  float a = (90 - (pos1 / add) * 90 / 800);
  float b = (pos2 * add) * 90 / 800;
  a = a * DEG_TO_RAD;
  b = b * DEG_TO_RAD;
  float c = 470;

  float xT = c * tan(b) / (tan(a) + tan(b));  //mm
  float yT = xT * tan(a);                     //mm
  x = xT;
  y = yT;
  Serial.println("calculate: " + String(pos_x) + "(" + String(diff_x) + "):" + String(pos_y) + "(" + String(diff_x) + ") >>>  " + String(x) + ":" + String(y));
}


void correctInitPosMA(class Motor& mot) {
  mot.move(-70);
  while (digitalRead(mot.pinEnd) == 1) {}
  *mot.cnt = mot.offset;
  Serial.println("Offset = " + String(mot.offset));
  mot.move(0);
  delay(300);

  mot.move(70);
  while (digitalRead(mot.pinEnd) == 0) {}
  Serial.println("Offset = " + String(mot.offset));
  mot.move(0);
  delay(300);
}

void testTOFiindOffser(class Motor& mot) {
  mot.move(-50);
  delay(1000);
  mot.move(0);
  delay(1000);
  *mot.cnt = -5;
  for (int i = 0; i < 20; i++) {
    Serial.println(*mot.cnt);
    delay(1000);
  }
  delay(10000);
}

void setup() {

  //S3 - 15 pin serial; R6 Hall(D26 pin)
  //S2 - 17 PIN serial; R5 Hall(D27 pin)
  pinMode(28, INPUT);
  pinMode(27, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);

  delay(1000);
  Serial3.begin(115200);
  Serial2.begin(115200);
  Serial.begin(115200);
  delay(1000);

  setupCoeficients();
  setupEncoders();

  // mot2B.move(-200);
  // mot1B.move(-200);

  // delay(1500);

  // mot1B.move(-255);

  //move to see offset
  //testTOFiindOffser(mot1A);

  //correct positopn end
  correctInitPosMA(mot2A);
  correctInitPosMA(mot1A);

  // mot2B.move(-255);
  // delay(5000);

  // while (digitalRead(mot2A.pinEnd)==0){}
  // Serial.println(2);
  // mot2A.move(0);
  // mot2A.offset = *mot2A.cnt;
  // delay(5000);

  // delay(5000);
  // Serial.println(String(cnt1) + " " + String(cnt2) + " " + String(cnt3) + " " + String(cnt4) + " ");
  // delay(100009);

  // mot2A.tar = 30;
  // mot2B.tar = 3;
  // mot1B.tar = 0;


  // mot1B.tar = 0;
  Serial.println("end setup");
}

Timer punch(4000);

Timer stop(300);

float state = 2;

Timer stopPWM(1);
int stopCnt = 0;


int prev_x;
int prev_y;


// void loop2() {
//   delay(30);
//   //movePos2 (100);
//   mot2A.tick();
//   Serial.println(String(cnt1) + " " + String(cnt2) + " " + String(cnt3) + " " + String(cnt4) + " ");
// }

Timer serialRead(20);


void loop() {
  // Serial.println(digitalRead(20));
  // return;
  //checkSerial();

  if (serialRead) {
    prev_x = pos_x;
    prev_y = pos_y;
    readPos();
    if (pos_x != prev_x || prev_y != pos_y) {
      if (pos_x != 304)  //какая-то глючная точка!! надо посмотреть на месте с освещением
        calculate(pos_x, pos_y);
      // Serial.print(2);
    }
  }



  // delay(200);
  mot1A.tick();
  mot2A.tick();
  movePos2(y);
  movePos(y);

  if (x > 270) {
    if (*mot2B.cnt < 0) mot2B.move(80);
    else if (*mot2B.cnt > 0) mot2B.move(-80);
  } else mot2B.move(0);
  //Serial.println(*mot2B.cnt);

  mot2B.recalcPos();

  // Serial.println(*mot2A.cnt);
  // // mot2B.tick();.
  // mot1B.tick();
  // // movePos(((long)millis() - 5000) / 100);
  // mot2A.tar = y/10;
  // movePos2((glPx - 30) / 5 * 3);
  // Serial.println((glPx - 30) / 5 * 3);
  // delay(20);
  // return;
  //mot1A.tick(); // hold position
  //movePos(((long)millis() - 5000) / 100); move horisontally
  //return;

  if (x < 210 && x > 140) {
    state = 0;
    x = 0;
  }

  mot1B.recalcPos();
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
      *mot1B.cnt += 45;
      mot1B.tar = 0;  //*mot1B.cnt + 60 - ((*mot1B.cnt + 60) % 45);  // put vertical target
    } else {
      if (stopCnt % 2 == 0) mot1B.move(255);
      else mot1B.move(-255);
      stopCnt++;
    }
  }

  if (state == 2) {  // move to vertical poition
    // Serial.println(*mot1B.cnt - mot1B.tar);
    mot1B.tick();
    // if (punch) state = 0;
  }
}
