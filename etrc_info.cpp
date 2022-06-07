#include "etrc_info.h"
#include <math.h>
#include "app.h"

Luminous::Luminous(SensorIo* sensor_io, Camera* camera)
    : color_(kInvalidColor), hsv_({0, 0, 0}), sensor_io_(sensor_io), camera_(camera) {
}

void Luminous::Update() {
  UpdateRgb();
  UpdateHsv();
  UpdateColor();
}

void Luminous::SetColorReference(Color c, Hsv hsv) {
  color_ref_[c] = hsv;
}

void Luminous::UpdateRgb() {
  rgb_raw_t val = sensor_io_->color_rgb_raw_;
  
  rgb_.r = val.r;
  rgb_.g = val.g;
  rgb_.b = val.b;
}

void Luminous::UpdateHsv() {
  float r = static_cast<float>(rgb_.r);
  float g = static_cast<float>(rgb_.g);
  float b = static_cast<float>(rgb_.b);

  float max = r > g ? r : g;
  max = max > b ? max : b;
  float min = r < g ? r : g;
  min = min < b ? min : b;
  float c = max - min;

  float h;
  if (c == 0) {
    h = -1;
  } else if (max == r) {
    h = fmodf(((g - b) / c), 6);
  } else if (max == g) {
    h = ((b - r) / c) + 2;
  } else if (max == b) {
    h = ((r - g) / c) + 4;
  } else {
    h = -1;
  }

  if (h != -1) {
    h = 60 * h;
  }

  float s;
  if (max == 0) {
    s = 0;
  } else {
    s = c / max;
  }

  float v = max;
  if (v > 100) {
    v = 100;
  }

  hsv_.h = h;
  hsv_.s = s * 100;
  hsv_.v = v;
}

void Luminous::UpdateColor() {
}

Localize::Localize(MotorIo* motor_io)
    : motor_io_(motor_io) {
}

void Localize::Update() {
  int32_t counts_r_ = motor_io_->counts_r_;
  int32_t counts_l_ = motor_io_->counts_l_;

  // if(counts_l_ > 50 && counts_r_ > 50) {
  //   counts_l_=0;
  //   counts_r_=0;
  // }

  curr_index += 1;
  counts_rs[curr_index] = counts_r_;
  counts_ls[curr_index] = counts_l_;

  double Ll = R * (counts_ls[curr_index] - counts_ls[curr_index - 1]) * M_PI / 180;
  double Lr = R * (counts_rs[curr_index] - counts_rs[curr_index - 1]) * M_PI / 180;

  double micro_theta = (Lr - Ll) / D;
  theta_wa += micro_theta;
  theta = theta_wa;
  double A = (Lr + Ll) / 2 * (1 - 0);
  double dx = A * cos(theta_wa + micro_theta / 2);
  double dy = A * sin(theta_wa + micro_theta / 2);
  double dd = sqrt(dx * dx + dy * dy);

  x += dx;
  y += dy;
  distance_ += dd;

  char str[264];
  sprintf(str, "x: %f y: %f distance: %f\n", x, y, distance_);
  syslog(LOG_NOTICE, str);
}

void Localize::SaveOdometri() {
  char str [256];
  FILE* fp = fopen("Odome.csv", "w");

  for (int i=0; i<curr_index; i++) {
    sprintf(str, "%d, %d\n", counts_ls[i], counts_rs[i]);
    fprintf(fp, str);
  }

  fclose(fp);
}
