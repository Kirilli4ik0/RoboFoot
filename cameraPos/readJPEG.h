#include "esp_camera.h"

namespace imageJPEG {


  typedef struct {
    uint16_t width;
    uint16_t height;
    uint16_t data_offset;
    const uint8_t *input;
    uint8_t *output;
  } rgb_jpg_decoder;

  #define BYTE_PIX 1

  long mid = 0;
  int midCount = 0;

  //output buffer and image width
  static bool _rgb_write(void *arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data) {
    rgb_jpg_decoder *jpeg = (rgb_jpg_decoder *)arg;
    if (!data) {
      // pDln("");
      // pDln("size: " + String(x) + "_" + String(y));
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
    if (y == 0) {
      //Serial.print(data[0]);Serial.print(" ");
      mid+= data[0];
      midCount++;
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

    mid = 0;
    midCount = 0;

    if (esp_jpg_decode(fb->len, JPG_SCALE_NONE, _jpg_read, _rgb_write, (void *)&jpeg) != ESP_OK) {
      return false;
    }

    //Serial.println(String(mid) + " " + String(midCount));
    jpgW = jpeg.width;
    jpgH = jpeg.height;
    return true;
  }

}
