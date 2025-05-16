
#define CAMERA_MODEL_ESP32S3_EYE  // Has PSRAM
#include "camera_pins.h"

#include <HardwareSerial.h>
#include "readJPEG.h"

//---KKK------------------------------------------------------------------
bool isDebugView = false;

uint8_t buf[800];
int pos = 0;
int diff = 0;
int aec = 600;
int gain = 64;

int iFb = 0;
unsigned long t = 0;
camera_fb_t *fb = NULL;


uint8_t bufOld[800];
uint8_t* oldB;
uint8_t* curB;


void pD(String s) {
  if (isDebugView) Serial.print(s);
}

void pDln(String s) {
  if (isDebugView) Serial.println(s);
}


void saveOr(){
  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb);

  delay(20);
  fb = esp_camera_fb_get();
  if (!fb) {
    pDln("Camera capture failed");
  } else {
    imageJPEG::decode(fb, bufOld);
  }
  esp_camera_fb_return(fb);
}

void setup() {

  Serial.begin(115200);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 40000000;
  config.frame_size = FRAMESIZE_CIF;     //FRAMESIZE_VGA;//FRAMESIZE_SVGA;//FRAMESIZE_QVGA;//FRAMESIZE_240X240;//FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  //PIXFORMAT_GRAYSCALE;//
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_LATEST;  //CAMERA_GRAB_WHEN_EMPTY;//
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 8;  //12
  config.fb_count = 2;

  //camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  if (!psramFound()) {
    delay(5000);
    Serial.printf("Limit the frame size when PSRAM is not available");
  }

  sensor_t *s = esp_camera_sensor_get();

  s->set_dcw(s, 0);            //    dcw = 0
  s->set_exposure_ctrl(s, 0);  //handle enable
  s->set_aec_value(s, 600);    // 400 //aec_value = 108
  s->set_whitebal(s, 0);       //??? awb = 0
  s->set_gain_ctrl(s, 0);
  s->set_agc_gain(s, 64);  //??? agc = 64
  s->set_raw_gma(s, 0);    //  raw_gma = 0
  s->set_lenc(s, 0);       //  lenc = 0
  s->set_bpc(s, 0);        // bpc = 0
  s->set_wpc(s, 1);        // wpc = 0
  s->set_dcw(s, 0);

  s->set_vflip(s, 0);
  s->set_hmirror(s, 1);
  //old camera
  //s->set_vflip(s, 1);
  //s->set_hmirror(s, 0);

  //int res = s->set_res_raw(s, 0, 940, 2560, 988, 0, 0, 2644, 1000, 800, 16, true, true);///LL  S2!!!!
  //int res = s->set_res_raw(s, 0, 900+10, 2560, 948+10, 0, 0, 2644, 1000, 800, 16, true, true);//RR S3 old


  int h = 16;//64;//16;
  //KKK ===================================================
  int up = 930;//s2

  //s3  new
  //int up = 940 - 35;//s3
  int res = s->set_res_raw(s, 0, up, 2560, up+3*h, 0, 0, 2644, up+3*h+20, 800, h, true, true);
  Serial.println(res);
  delay(500);
  
}



void calculateFPS() {
  if (iFb == 0) {
    t = millis();
    Serial.println("---------start 20---------------------");
  }

  if (iFb > 20) {
    Serial.println();
    Serial.println("--------RES 20 F----------------------");
    Serial.println(millis() - t);
    Serial.println("");
    iFb = 0;
  } else {
    iFb++;
  }
  // Take Picture with Camera
  unsigned long tf = millis();
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
  } else {
    Serial.print("len:");
    Serial.print(fb->len);
    Serial.print(" time:");
    Serial.println(millis() - tf);
    esp_camera_fb_return(fb);
  }
}

int MID_AV = 85;//95
void changeAec(int mid) {
  
  pDln("mid  pixel = " + String(mid) + " " + String(aec));
  if (mid<MID_AV-10 || mid>MID_AV+10) {

    if (mid > MID_AV+30 ) mid = MID_AV+30;
    else if (mid < MID_AV-30 ) mid = MID_AV-30;

    sensor_t *s = esp_camera_sensor_get();
    aec += (MID_AV - mid)*2;
    if (aec > 1900) {aec = 1900; gain=64; s->set_agc_gain(s, gain); }
    else if (aec < 300) {aec = 300;  gain-=5; s->set_agc_gain(s, gain);  }//??? agc = 64
    s->set_aec_value(s, aec);

    //Serial.print(" m=" +String(mid)+ " aec=" + String(aec));
  }
}

int averPixel = 0;
bool getMaxDif(){
  int m = 0;
  int cur = 0;
  int a = 0;

  long midPixel = 0;
  int avC=0;
  for (int i=20; i<=780; i++){
      if (i%10 == 0) {
        midPixel+=curB[i];
        avC ++;
      }
      a = curB[i] - oldB[i];
      if (a > m) {
        cur = i;
        m = curB[i];
      }
  }
  if (cur>0){
    pos = cur;
  }

  averPixel = midPixel/avC;
  diff = curB[pos] - oldB[pos];
  //Serial.print(" d_" + String(diff));

  //center ball
  if (diff>20){
    int t = pos;
    int c = curB[t];
    while (curB[t] >= c - diff/2 && t > 2) {
        if (curB[t] > c)
            c = curB[t];
        t -= 2;
    }
    int minC = t;
    t = pos;
    while (curB[t] >= c - diff/2 && t < 780){
        if (curB[t] > c)
            c = curB[t];
        t += 2;
    }
    int maxC = t;
    pos = (minC + maxC)/2;
  }

  return diff>20;
}

bool sw = true;

void switchBuf(){
  if (sw){
    oldB = bufOld;
    curB = buf;
  } else {
    oldB = buf;
    curB = bufOld;
  }
  sw = !sw;
}

int c = 0;
void loop() {
  switchBuf();
  //Take Picture with Camera
  unsigned long tf = millis();
  fb = esp_camera_fb_get();
  c++;
  if (!fb) {
    pDln("Camera capture failed");
    delay(5000);
  } else {

    imageJPEG::decode(fb, curB);
    //bool res = calculateBall();
    bool res = getMaxDif();

    //if(c > 2) 
    {
      int av = MID_AV;
      if (imageJPEG::midCount>0)
        av = imageJPEG::mid/imageJPEG::midCount;
      //if (!res) 
      {
        changeAec(av);
      }
      c = 0;
    }

    if (isDebugView) {
      Serial.println();
      Serial.println("0 255 0 255");
      Serial.print(fb->len);
      Serial.println("");
      // if (res == true) Serial.print(pos);
      // else Serial.print(0);
      //Serial.println("");

      Serial.println(aec);
      Serial.write((byte *)buf, 800);
      //Serial.print(fb->width);Serial.print(" ");Serial.println(fb->height);//Serial.println("5 5");//
      Serial.write((byte *)fb->buf, fb->len);  //Serial.write((byte*)fb->buf, 25);//
    }

    if (res) {
      //Serial.println("");
      Serial.println("camera " +  String(pos) + " " + String(diff) + " ok");
    }

    //if (res != true) Serial.print(" e " + String(pos) );
    //else  {Serial.println("");Serial.println(" !!! " + String(pos) );}

  }

  esp_camera_fb_return(fb);
}
