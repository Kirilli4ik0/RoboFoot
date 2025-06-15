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
Timer print(50);

class Motor {
public:
  const int portA, portB, portPWM;
  int tar  = 0;
  volatile int* cnt;

  float Kp = 3.5;//5;
  float Kd = 0;
  float Ki = 0.1;
  float integral = 0;
  float derivative = 0;
  int difLst = 0;

  int offset = 0;
  int minSpeed;
  bool lastEnd = false;
  int pinEnd;
  String name;

  int spd;
  long t;

  Timer compoundTimer{ 20 };

  Motor(Motor&) = delete;
  void operator=(Motor&) = delete;

  Motor(int portA, int portB, int portPWM, volatile int* cnt, int pinEnd, int offset,  String name, int minSpeed)
    : portA(portA), portB(portB), portPWM(portPWM), cnt(cnt), offset(offset), pinEnd(pinEnd), name(name), minSpeed(minSpeed) {
    pinMode(portA, OUTPUT);
    pinMode(portB, OUTPUT);
    pinMode(portPWM, OUTPUT);
    pinMode(pinEnd, INPUT);
  }

  void move(int nspd) {
    
    digitalWrite(portA, nspd > 0);
    digitalWrite(portB, nspd < 0);
    if (nspd != 0)
      nspd = abs(nspd)*2 + minSpeed;//125;//70;//nspd = abs(nspd) + 125;
    analogWrite(portPWM, min(nspd, 255));
    spd = nspd;
  }

  void recalcPos() {
    bool curEnd = digitalRead(pinEnd);
    if (lastEnd != curEnd && *cnt != offset) {
      Serial.println("END corrected from " + String(*cnt) + " to " + String(offset));
      *cnt = offset;
    }
    lastEnd = curEnd;
  }

  bool isBlock(){
    return (millis() - t>200 && spd!=0);
  }

  void tick() {
    recalcPos();//correct  end position

    if (compoundTimer) {
      // Serial.println(*cnt - tar);

      int pos = *cnt;
      int dif = (tar - pos);

      derivative = derivative * 0.9 + (dif - difLst) * 0.1;

      float currKd = Kd;  // * (1 - min(1,abs(dif)/50));

      integral = constrain(integral, -1000, 1000);

      if (abs(dif) <= 3 && derivative < 0.2)//if (abs(dif) <= 3 && derivative < 0.2)
        move(0);
      else  {
        int m =  dif * Kp + Ki * integral + currKd * derivative;
        move(m);
        //Serial.print("." + String(m));
      }


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
Motor mot1A(41, 40, 6, &cnt1, 30, 18, "mot1A", 125);  //M2 side
Motor mot1B(42, 43, 7, &cnt2, 27, 0, "mot1B", 20);   //M1 round

Motor mot2A(A5, A4, 4, &cnt3, 29, 13, "mot2A", 125);   //M4 side  vorota
Motor mot2B(A10, A11, 8, &cnt4, 26, 0, "mot2B", 20);  //M7 round  vorota
//------------------------------  -------------------------------------------------------------------


void inter1() {
  mot1A.t  = millis();
  if (digitalRead(38)) cnt1--;
  else cnt1++;
}
void inter2() {
  mot1B.t  = millis();
  if (digitalRead(36)) cnt2++;
  else cnt2--;
}
void inter3() {
  mot2A.t  = millis();
  if (digitalRead(A2)) cnt3--;
  else cnt3++;
}
void inter4() {
  mot2B.t  = millis();
  if (!digitalRead(A0)) cnt4--;
  else cnt4++;
}

long long t1=0, t2=0, t3=0, t4=0;
void checkBlock(){
  if(mot1A.isBlock()){
    Serial.println("block@@@@@@@@@@@@@@@@@@@@@@@@@@@");
    Serial.println("spd=" +  String(mot1A.spd) + " tar=" + String(mot1A.tar) + " cnt=" + String(*mot1A.cnt) );
    /*mot1A.move(-70);
    delay(100);
    *mot1A.cnt = 0;
    mot1A.move(0);
    Serial.println("spd=" +  String(mot1A.spd) + " tar=" + String(mot1A.tar) + " cnt=" + String(*mot1A.cnt) );*/
  }
  //mot1B.checkBlock();
  if(mot2A.isBlock() ){
    mot2A.tar = *mot1A.cnt;
  }
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

//нападающий
void movePos(float pos) {
  if (pos>115) pos-=100;
  if (pos>115) pos-=100;

  if (pos<35) pos = 35;
  if (pos>115) pos = 115;

  pos =  map(pos, 35, 115,  0, 50);

  // int offs1 = 120;
  // int offs2 = 220;

  // if (pos > offs2) pos = map(constrain(pos+10, offs2, 300), offs2, 300, 0, 57);
  // else if (pos > offs1) pos = map(constrain(pos, offs1, offs2), offs1, offs2, 0, 57);
  // else pos = map(constrain(pos-20, 0, offs1), 0, offs1, 0, 57);

  if (abs(mot1A.tar - pos) > 1) {
    Serial.println("movePos " + String(pos));
    mot1A.tar = pos;
  }
}

//вротарь
void movePos2(float pos) {
  //pos *= 5.6;
  // pos = min(80, max(50, pos));

  pos = map(constrain(pos, 120, 220), 140, 220, 5, 50);

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
  delay(50);
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


bool readPos(int minD) {
  bool  res = false;
  
  int tD=0;
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
        tD = s.substring(spacePos1 + 1, spacePos2).toInt();
        if (!err_x && tD>minD) {
          diff_x = tD;
          pos_x = s.substring(7, spacePos1).toInt();
          res = true;
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
        tD = s.substring(spacePos1 + 1, spacePos2).toInt();
        if (!err_y && tD>minD) {
          diff_y = tD;
          pos_y = s.substring(7, spacePos1).toInt();
          res = true;
        }
      }
      //pos_y = (pos_y * 9 + s.substring(8, spacePos).toInt()) / 10;
    }
  }
  return res;
  //Serial.println("cam2" + s + "!");
}
void calculate(float pos1, float pos2) {
  float tgA = (805- pos1) / (pos1-64);
  float tgB = pos2/(751-pos2);

  int angA = RAD_TO_DEG*atan(tgA);
  int angB = RAD_TO_DEG*atan(tgB);
  Serial.print(String(angA) + ":" + String(angB) + "____");
  float c = 470;
  float xT = c * tgB / (tgA + tgB);  //mm
  float yT = xT * tgA;                     //mm
  x = xT;
  y = yT;
  Serial.println("calculate: S2_" + String(int(pos1)) + "(" + String(diff_x) + "):S3_" + String(int(pos2)) + "(" + String(diff_x) + ") >>>  " + String(x) + ":" + String(y));
}

void calculateOld(float pos1, float pos2) {
  int posA = map(constrain(pos1, 80, 750), 80, 770, 0, 90);
  int posB = map(constrain(pos2, 20, 680), 20, 700, 5, 90);//int posB = map(constrain(pos2, 20, 680), 0, 700, 0, 90);
  Serial.print(String(pos1) + ":" + String(pos2) + "____");
  float a = (90 - posA);
  float b = posB;
  a = a * DEG_TO_RAD;
  b = b * DEG_TO_RAD;
  float c = 470;

  float xT = c * tan(b) / (tan(a) + tan(b));  //mm
  float yT = xT * tan(a);                     //mm
  x = xT;
  y = yT;
  Serial.println("calculate: S2_" + String(posA) + "(" + String(diff_x) + "):S3_" + String(posB) + "(" + String(diff_x) + ") >>>  " + String(x) + ":" + String(y));
}


void correctInitPosMA(class Motor& mot) {

  mot.move(-70);
  while (digitalRead(mot.pinEnd) == 1) {}
  mot.move(0);
  *mot.cnt = mot.offset;
  Serial.println("Offset = " + String(mot.offset));

  delay(300);

  mot.move(70);
  while (digitalRead(mot.pinEnd) == 0) {}
  mot.move(0);
  Serial.println("correctInitPosMA END");
}

void correctInitV(class Motor& mot) {
  mot.move(-20);
  delay(200);
  mot.move(0);
  mot.move(20);
  while (digitalRead(mot.pinEnd) == 1) {}
  mot.move(0);
  *mot.cnt = mot2B.offset;
  Serial.println("correctInitV Offset = " + String(mot.offset));
}

void testTOFindOffser(class Motor& mot) {
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
  pinMode(29, INPUT);
  pinMode(27, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);

  delay(1000);
  Serial3.begin(115200);
  Serial2.begin(115200);
  Serial.begin(115200);
  delay(1000);

  setupCoeficients();
  setupEncoders();

  //move to see offset
  //testTOFindOffser(mot1A);

  //test line sensors!
    // while(1){
    //   Serial.print(digitalRead(mot2A.pinEnd)); 
    //   Serial.print(digitalRead(mot1A.pinEnd)); 
    //   delay(1000);
    // }

  //correct positopn end
  // correctInitPosMA(mot2A);
  // correctInitPosMA(mot1A);

  correctInitV(mot2B);

  // Serial.println(String(cnt1) + " " + String(cnt2) + " " + String(cnt3) + " " + String(cnt4) + " ");
  // delay(100009);

  // mot2A.tar = 30;
  // mot2B.tar = 3;
  // mot1B.tar = 0;
  // mot1B.tar = 0;


  Serial.println("end setup");
}

Timer punch(500);//4000

Timer stop(100);//300

float state = 2;

Timer stopPWM(1);
int stopCnt = 0;


int prev_x;
int prev_y;




Timer serialRead(20);

void handleStateMot(){

  //вратарь
  // if (x > 350) {
  //   if (*mot2B.cnt < 0) mot2B.move(80);
  //   else if (*mot2B.cnt > 0) mot2B.move(-80);
  // } else mot2B.move(0);
  mot2B.tick();
  //Serial.println(*mot2B.cnt);
  if (x > 250) {
    if (mot2B.tar == 5) mot2B.tar = -5;
    else if (x>350 && y>100 && y<260) mot2B.tar = 5;
    else mot2B.tar = -5;
  } 

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



  mot1B.recalcPos();
   //обратно в луп если не хочу постоянно вертеть
    if (state == 0) {  // start punch
      mot1B.move(255);//255
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
      if (stopCnt % 2 == 0) mot1B.move(255);//255
      else mot1B.move(-255);//255
      stopCnt++;
    }
  }

  if (state == 2) {  // move to vertical poition
    // Serial.println(*mot1B.cnt - mot1B.tar);
    mot1B.tick();
      //сброс чтоб не били пока новый кадр не придет
    if (x < 200 && x > 115) {
      state = 0;
      x = 0;
    }
    // if (punch) state = 0;
  }
}
int  st1 = 0;
int oldc  = 0;
void loopT(){
  st1++;
  if (st1==1) {
      movePos(115);
      Serial.println(String(cnt1) + " " + String(cnt2) + " " + String(cnt3) + " " + String(cnt4) + " ");
  }else if (st1==1000)  {
    Serial.println("move 0");
    movePos(0);
    Serial.println(String(cnt1) + " " + String(cnt2) + " " + String(cnt3) + " " + String(cnt4) + " ");
  }else if (st1==2000)  {
    st1=0;
    delay(2000);
    Serial.println(String(cnt1) + " " + String(cnt2) + " " + String(cnt3) + " " + String(cnt4) + " ");
  }
  if (oldc != cnt1) {
    Serial.print("_" + String(cnt1) );
    oldc = cnt1;
  }
  mot1A.tick();
  delay(5);
}

// void loop() {
//   delay(300);
//   //movePos2 (100);
//   //mot2A.tick();
//   Serial.println(String(cnt1) + " " + String(cnt2) + " " + String(cnt3) + " " + String(cnt4) + " ");
// }

int  prognozeVr = 0;
void loop() {
  // checkSerial();
  // return;

  if (serialRead) {
    prev_x = x;
    prev_y = y;
    //-------KKKK ----------- если ярко то 70/ если темно то 25 
    if(readPos(40)) {
      calculate(pos_x, pos_y);
    }
    if (x!=prev_x || y!=prev_y){
      prognozeVr = (y - prev_y)*(420-prev_x)/(x -prev_x) + prev_y;
      Serial.println(String(x) + "," + String(y) + " prev " + String(prev_x) + "," + String(prev_y) +"->" + String(prognozeVr));
      if (prognozeVr<100 || prognozeVr>250){
        prognozeVr=0;
      }
    }
  }

  //слежение за мячем вправо-влево
  mot1A.tick();//нападающие
  mot2A.tick();//вратарь

  // //вратарь
  if (prev_x < x || abs(prev_y - y)<3 || prognozeVr == 0)
    movePos2(y);
  else {
    movePos2(prognozeVr);
  }

  movePos(y);//нападающие

  //checkBlock();//останавливаем  мотор если он заел и не двигается 

  handleStateMot();
}
