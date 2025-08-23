#define enA 18
#define enB 19

#define in1_left 5
#define in2_left 6
#define in1_right 9
#define in2_right 10

#define WHEEL_RADIUS 0.03f   
#define TRACK_WIDTH  0.18f  

float Kp = 1.2, Ki = 0.01, Kd = 0.05;

float vmax = 90.0;
float amax = 180.0; 

const float dt = 0.02f;

float integral = 0, prevError = 0, prevDeriv = 0;


void setup() {
  pinMode(in1_left, OUTPUT);
  pinMode(in2_left, OUTPUT);
  pinMode(in1_right, OUTPUT);
  pinMode(in2_right, Output);
}

void loop() {

}

void trapezoidLinear(float totalDist, float t, float &pos, float &vel) {
  float D = fabs(totalDist);
  float sign = (totalDist >= 0) ? 1.0 : -1.0;

  float t_acc = vmax_lin / amax_lin;
  float dist_acc = 0.5 * amax_lin * t_acc * t_acc;

  float t_flat, t_total;

  if (2 * dist_acc >= D) {
    t_acc = sqrt(D / amax_lin);
    t_flat = 0;
    t_total = 2 * t_acc;
  } else {
    t_flat = (D - 2 * dist_acc) / vmax_lin;
    t_total = 2 * t_acc + t_flat;
  }

  if (t >= t_total) {
    pos = D * sign;
    vel = 0;
  } else if (t < t_acc) {
    vel = amax_lin * t;
    pos = 0.5 * amax_lin * t * t;
  } else if (t < (t_acc + t_flat)) {
    vel = vmax_lin;
    pos = dist_acc + vmax_lin * (t - t_acc);
  } else {
    float td = t - (t_acc + t_flat);
    vel = vmax_lin - amax_lin * td;
    pos = dist_acc + vmax_lin * t_flat + vmax_lin * td - 0.5 * amax_lin * td * td;
  }

  vel *= sign;
  pos *= sign;
}

void linearToWheels(float v, float &vL, float &vR) {
  vL = v;
  vR = v;
}

void moveDistance(float targetDist) {
  integral_lin = 0;
  prevError_lin = 0;

  float startTime = millis() / 1000.0;
  float pos_cmd, vel_cmd;
  float measuredDist = 0;

  while (true) {
    float t = (millis() / 1000.0) - startTime;

    trapezoidLinear(targetDist, t, pos_cmd, vel_cmd);

    measuredDist = getDistance(); 

    float error = pos_cmd - measuredDist;

    if (fabs(error) < 0.01 && fabs(vel_cmd) < 0.01) {
      setMotorSpeeds(0, 0);
      break;
    }

    float u = pidUpdate(error);

    float v_cmd = vel_cmd + u;

    float vL, vR;
    linearToWheels(v_cmd, vL, vR);

    int pwmL = velToPWM(vL);
    int pwmR = velToPWM(vR);

    setMotorSpeeds(pwmL, pwmR);

    delay((int)(dt * 1000));
  }
}

float wrapAngle(float e) {
  while (e > 180) e -= 360;
  while (e < -180) e += 360;
  return e;
}

float pidUpdate(float error) {
  integral += error * dt;
  float deriv = (error - prevError) / dt;

  float u = Kp * error + Ki * integral + Kd * deriv;

  prevError = error;
  return u;
}

void angularVelToWheels(float omegaDeg, float &vL, float &vR, String mode, String pivotSide) {
  float omega = omegaDeg * PI / 180.0;

  if (mode == "inplace") {
    vL = -omega * (TRACK_WIDTH / 2.0);
    vR =  omega * (TRACK_WIDTH / 2.0);
  }
  else if (mode == "pivot") {
    if (pivotSide == "left") {
      vL = 0;
      vR = omega * TRACK_WIDTH;
    } else {
      vR = 0;
      vL = -omega * TRACK_WIDTH;
    }
  }
}

int velToPWM(float v) {
  float maxWheelSpeed = 0.5; 
  int pwm = (int)(255.0 * v / maxWheelSpeed);
  if (pwm > 255) pwm = 255;
  if (pwm < -255) pwm = -255;
  return pwm;
}


void setMotorSpeeds(int pwmL, int pwmR) {
  if (pwmL >= 0) {
    analogWrite(in1_left, pwmL);  
    analogWrite(in2_left, 0);
  } else {
    analogWrite(in1_left, 0);
    analogWrite(in2_left, -pwmL);
  }

  if (pwmR >= 0) {
    analogWrite(in1_right, pwmR); 
    analogWrite(in2_right, 0);
  } else {
    analogWrite(in1_right, 0);
    analogWrite(in2_right, -pwmR);
  }
}

void turnTheta(float targetTheta, String mode="inplace", String pivotSide="left") {
  integral = 0;
  prevError = 0;

  float startTime = millis() / 1000.0;
  float duration = fabs(targetTheta) / vmax * 2.0;

  while (true) {
    float t = (millis()/1000.0) - startTime;

    float currentAngle = getHeading(); 

    float error = wrapAngle(targetTheta - currentAngle);

    if (fabs(error) < 2.0 && t > duration) {
      setMotorSpeeds(0, 0);
      break;
    }

    float u = pidUpdate(error);

    float omegaCmd = u;

    float vL, vR;
    angularVelToWheels(omegaCmd, vL, vR, mode, pivotSide);

    int pwmL = velToPWM(vL);
    int pwmR = velToPWM(vR);

    setMotorSpeeds(pwmL, pwmR);

    delay((int)(dt*1000));
  }
}

void getHeaing(){
}
void getHeading(){
}