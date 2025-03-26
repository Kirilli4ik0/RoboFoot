#include "esp_camera.h"

#define CAMERA_MODEL_ESP32S3_EYE  // Has PSRAM
#include "camera_pins.h"

#include <HardwareSerial.h>


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


void saveOr(){
  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb);

  delay(20);
  fb = esp_camera_fb_get();
  if (!fb) {
    pDln("Camera capture failed");
  } else {
    decode(fb, bufOld);
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

typedef struct {
  uint16_t width;
  uint16_t height;
  uint16_t data_offset;
  const uint8_t *input;
  uint8_t *output;
} rgb_jpg_decoder;

static unsigned int _jpg_read(void *arg, size_t index, uint8_t *buf, size_t len) {
  rgb_jpg_decoder *jpeg = (rgb_jpg_decoder *)arg;
  if (buf) {
    memcpy(buf, jpeg->input + index, len);
  }
  return len;
}

int jpgW = 0;
int jpgH = 0;
static bool decode(camera_fb_t *fb, uint8_t *out_buf) {
  //s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
  rgb_jpg_decoder jpeg;
  jpeg.width = 0;
  jpeg.height = 0;
  jpeg.input = fb->buf;
  jpeg.output = out_buf;
  jpeg.data_offset = 0;

  if (esp_jpg_decode(fb->len, JPG_SCALE_NONE, _jpg_read, _rgb_write, (void *)&jpeg) != ESP_OK) {
    return false;
  }
  jpgW = jpeg.width;
  jpgH = jpeg.height;
  return true;
}
#define BYTE_PIX 1
//output buffer and image width
static bool _rgb_write(void *arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data) {
  rgb_jpg_decoder *jpeg = (rgb_jpg_decoder *)arg;
  if (!data) {
    pDln("");
    pDln("size: " + String(x) + "_" + String(y));
    if (x == 0 && y == 0) {
      //write start
      jpeg->width = w;
      jpeg->height = h;
      //Serial.println("");
      //Serial.println("size: " + String(x) + "_" + String(y));
    } else {
      //write end
      //Serial.println("");
      //Serial.println("end");
    }
    return true;
  }
  //Serial.print(String(x) + "_" + String(y) + " " + String(w) + "_" + String(h) + " " + String(data[0]) + " : ");
  size_t jw = jpeg->width * BYTE_PIX;
  size_t t = y * jw;
  size_t b = t + (h * jw);
  size_t l = x * BYTE_PIX;
  uint8_t *out = jpeg->output + jpeg->data_offset;
  uint8_t *o = out;
  size_t iy, ix;

  w = w * BYTE_PIX;

  //Serial.print("(");
  if (y == 8) {
    for (int i = 0; i < w; i++) {
      out[x + i] = data[i];
      //Serial.print(data[i]); Serial.print(" ");
    }
  }
  //Serial.print(")");
  // int tt=0;
  // for(iy=t; iy<b; iy+=jw) {
  //     o = out+iy+l;
  //     for(ix=0; ix<w; ix+= BYTE_PIX) {
  //         //o[ix] = data[ix+2];
  //         //o[ix+1] = data[ix+1];
  //         //o[ix+2] = data[ix];
  //         //o[ix] = data[ix+1];

  //         tt = data[ix+1];
  //     }
  //     data+=w;
  // }
  return true;
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
void pD(String s) {
  if (isDebugView) Serial.print(s);
}

void pDln(String s) {
  if (isDebugView) Serial.println(s);
}

bool calculateBall() {
  int iP = start;
  int sumBL = 0;
  int numBL = 0;
  int sect = 0;
  int found = 0;
  int err = 0;
  int ballSection = -1;
  int cur;

  for (int i = 0; i <= numBlock; i++) {
    pD(String(midBlackArr[i]) + " ");
  }
  pDln("");

  while (iP < width) {
    cur = buf[iP];
    if (cur < midBlackArr[sect]) {
      iP++;
      if (iP % block == 0) {
        if (numBL != 0 and ballSection != sect) {
          midBlackArr[sect] = (int)((float)sumBL * koff / numBL);
          pDln("sect=" + String(sect) + " new val=" + String(midBlackArr[sect]));
          if (midBlackArr[sect] < 20) {
            midBlackArr[sect] = 20;
          } else if (midBlackArr[sect] > 90) {
            midBlackArr[sect] = 90;
          }
        }
        sumBL = 0;
        numBL = 0;
        sect = iP / block;
      } else {
        numBL++;
        sumBL += cur;
      }
    } else {
      int start = iP;
      iP++;
      cur = buf[iP];
      while (cur >= midBlackArr[sect] and iP < width) {
        iP++;
        cur = buf[iP];
        if (sect != iP / block) {
          sumBL = 0;
          numBL = 0;
          sect = iP / block;
        }
      }
      if (iP - start > 10) {
        ballSection = sect;
        found++;
        pDln("found=" + String(found) + " pos: " + String(start) + " " + String(iP));
        if (found > 1 and err == 0) {
          koff += 0.1;
          pDln("koff += 0.5 koff=" + String(koff));
          if (koff>3) koff = 3;
          for (int k; k < numBlock; k++) {
            if (midBlackArr[k] < 110) {
              midBlackArr[k] += 5;
            }
          }
          iP = start;
          sumBL = 0;
          numBL = 0;
          sect = 0;
          found = 0;
          err = 1;
        } else {
          int next = (iP + start) / 2;
          if (found > 1) {
            //found several
            int dif1 = buf[pos] - midBlackArr[pos / block];
            int dif2 = buf[next] - midBlackArr[next / block];
            pDln("choose diif1=" + String(dif1) + " diff2=" + String(dif2));
            if (dif1 < dif2)
              pos = next;
          } else {
            pos = next;
          }
        }
      }
    }
  }
  if (found == 0) {
    koff = 1.5;
  }
  return found == 1;
}

void changeAec() {
  for (int i = 0; i <= numBlock; i++) {
    pD(String(midBlackArr[i]) + " ");
  }
  pDln("");
  int sum = 0;
  for (int j = 0; j <= numBlock; j++) {
    sum += midBlackArr[j];
  }
  int mid = sum / (numBlock + 1);
  pDln("changeAec mid=" + String(mid) + " sum=" + String(sum));
  if (mid < 40) {
    aec += 20;
    if (aec > 1000)
      aec = 1000;
  } else {
    aec -= 20;
    if (aec < 400)
      aec = 400;
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_aec_value(s, aec);

  pDln("aec=" + String(aec));
}
bool getMaxDif(){
  int m = 0;
  int cur = 0;
  int a = 0;
  for (int i=20; i<780; i++){
      a = curB[i] - oldB[i];
      if (a > m) {
        cur = i;
        m = curB[i];
      }
  }
  if (cur>0){
    pos = cur;
  }
  Serial.print(" d:" + String(curB[pos] - oldB[pos]));
  return curB[pos] - oldB[pos]>20;
}

bool sw = true;

void swiitchBuf(){
  if (sw){
    oldB = bufOld;
    curB = buf;
  } else {
    oldB = buf;
    curB = bufOld;
  }
  sw = !sw;
}

void loop() {
  swiitchBuf();
  //Take Picture with Camera
  unsigned long tf = millis();
  fb = esp_camera_fb_get();
  if (!fb) {
    pDln("Camera capture failed");
  } else {

    decode(fb, curB);
    //bool res = calculateBall();
    bool res = getMaxDif();
    if (!res) {
      changeAec();
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
