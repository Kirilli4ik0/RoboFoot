

#define CAMERA_MODEL_ESP32S3_EYE  // Has PSRAM
#include "camera_pins.h"

#include <HardwareSerial.h>
#include "readJPEG.h"


#define numBlock 15
byte midBlackArr[numBlock + 1];  // = [60]*(numBlock+1)
float koff = 1.5;
int width = 780;
int start = 20;
int block = width / numBlock;
int line = 8;  // #8+16 #8 # 67

uint8_t buf[800];
int pos = 0;
int aec = 600;
bool isDebugView = true;


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
  for (int i = 0; i < numBlock + 1; i++) {
    midBlackArr[i] = 60;
  }

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

  //camera_sensor_info_t *info = esp_camera_sensor_get_info(&s->id);
  //Serial.print("Detected camera ");
  //Serial.print(info->name);Serial.print(" pid=");Serial.println(s->id.PID);

  //Set Window: Start: 0 900, End: 2623 1200, Offset: 16 4, Total: 2844 1968, Output: 800 100, Scale: 1, Binning: 0
  //GOOD!!!!
  //int res = s->set_res_raw(s, 0, 0, 2623, 1951, 32, 16, 2844, 1968, 800, 600, false, false);
  //int ret = s->set_pll(s, false, 10, 1, 2, false, 1, true, 2);

  //int res = s->set_res_raw(s, 0, 750, 2560, 1050, 0, 0, 2644, 1200, 800, 96, true, true); GOOD !!!
  //KKKKKKKKKK
  int res = s->set_res_raw(s, 0, 940, 2560, 988, 0, 0, 2644, 1000, 800, 16, true, true);///LL  S2!!!!
  //int res = s->set_res_raw(s, 0, 900, 2560, 948, 0, 0, 2644, 1000, 800, 16, true, true);//RR S3!!!!
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

int MID_AV = 100;
void changeAec(int mid) {
  
  pDln("mid  pixel = " + String(mid) + " " + String(aec));
  if (mid<MID_AV-10 || mid>MID_AV+10) {

    if (mid > MID_AV+30 ) mid = MID_AV+30;
    else if (mid < MID_AV-30 ) mid = MID_AV-30;

    sensor_t *s = esp_camera_sensor_get();
    aec += (MID_AV - mid)*2;
    if (aec > 1900) aec = 1900;
    else if (aec < 300) aec = 300;
    s->set_aec_value(s, aec);

    pDln("aec=" + String(aec));
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
  Serial.print(" d:" + String(curB[pos] - oldB[pos]));
  return curB[pos] - oldB[pos]>20;
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
    
    // delayMicroseconds(10);
    Serial.println("camera");
    Serial.println(pos);
    if (res != true) Serial.println("error");
    else Serial.println("ok");

    //if (res != true) Serial.print(" e " + String(pos) );
    //else  {Serial.println("");Serial.println(" !!! " + String(pos) );}

  }

  esp_camera_fb_return(fb);
}
