// =============================================================
//  Inertial Navigation System
// =============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
#include <esp_task_wdt.h>
#include <math.h>

#ifndef sq
#define sq(x) ((x)*(x))
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295f
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232f
#endif

// ── Pins ──────────────────────────────────────────────────────
#define SDA_PIN    21
#define SCL_PIN    22
#define GPS_RX     16
#define GPS_TX     17
#define PIX_RX     32
#define PIX_TX     33
#define LED_DRIFT   2   // built‑in LED (drift warning)
#define LED_GPS     4   // GPS fix indicator
#define LED_STATIC  5   // ZUPT active indicator
#define BUZZER_PIN 26   // target arrival beep
#define NS         15

HardwareSerial SerialGPS(1);
HardwareSerial SerialPix(2);
Adafruit_BMP280  bmp;
QMC5883LCompass  compass;
TinyGPSPlus      gps;
WebServer        server(80);

// ── MPU6050 raw driver with rate‑of‑change rejection ─────────
#define MPU_ADDR 0x68
#define ACCEL_SC (9.80665f / 8192.0f)
#define GYRO_SC  (0.00106422f)

bool mpuWR(uint8_t r, uint8_t v) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v);
    return Wire.endTransmission() == 0;
}
bool mpuRR(uint8_t r, uint8_t* b, uint8_t n) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(r);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU_ADDR, n);
    if (Wire.available() < n) return false;
    for (uint8_t i = 0; i < n; i++) b[i] = Wire.read();
    return true;
}
bool mpuInit() {
    mpuWR(0x6B, 0x80); delay(150);
    if (!mpuWR(0x6B, 0x00)) return false; delay(100);
    uint8_t w = 0;
    if (!mpuRR(0x75, &w, 1)) return false;
    Serial.printf("  WHO=0x%02X\n", w);
    if (w!=0x68 && w!=0x70 && w!=0x71 && w!=0x19 && w!=0x98) return false;
    mpuWR(0x1A, 0x04);
    mpuWR(0x1B, 0x08);
    mpuWR(0x1C, 0x08);
    delay(50); return true;
}
static float prev_gx=0,prev_gy=0,prev_gz=0;
static bool  mpuHasPrev=false;
static float sm_gx=0,sm_gy=0,sm_gz=0;
static float sm_ax=0,sm_ay=0,sm_az=0;
#define GYRO_DELTA_MAX  2.0f

bool mpuRead(float& ax, float& ay, float& az,
             float& gx, float& gy, float& gz) {
    uint8_t b[6];
    if (!mpuRR(0x3B, b, 6)) return false;
    float rax = (int16_t)(b[0]<<8|b[1]) * ACCEL_SC;
    float ray = (int16_t)(b[2]<<8|b[3]) * ACCEL_SC;
    float raz = (int16_t)(b[4]<<8|b[5]) * ACCEL_SC;
    if (!mpuRR(0x43, b, 6)) return false;
    float rgx = (int16_t)(b[0]<<8|b[1]) * GYRO_SC;
    float rgy = (int16_t)(b[2]<<8|b[3]) * GYRO_SC;
    float rgz = (int16_t)(b[4]<<8|b[5]) * GYRO_SC;

    if (fabsf(rax)>50||fabsf(ray)>50||fabsf(raz)>50) return false;
    if (fabsf(rgx)>20||fabsf(rgy)>20||fabsf(rgz)>20) return false;

    if (mpuHasPrev) {
        if (fabsf(rgx-prev_gx)>GYRO_DELTA_MAX ||
            fabsf(rgy-prev_gy)>GYRO_DELTA_MAX ||
            fabsf(rgz-prev_gz)>GYRO_DELTA_MAX) {
            ax=sm_ax; ay=sm_ay; az=sm_az;
            gx=sm_gx; gy=sm_gy; gz=sm_gz;
            return true;
        }
    }
    prev_gx=rgx; prev_gy=rgy; prev_gz=rgz;
    mpuHasPrev=true;

    sm_gx = 0.6f*sm_gx + 0.4f*rgx;
    sm_gy = 0.6f*sm_gy + 0.4f*rgy;
    sm_gz = 0.6f*sm_gz + 0.4f*rgz;
    sm_ax = 0.5f*sm_ax + 0.5f*rax;
    sm_ay = 0.5f*sm_ay + 0.5f*ray;
    sm_az = 0.5f*sm_az + 0.5f*raz;

    ax=sm_ax; ay=sm_ay; az=sm_az;
    gx=sm_gx; gy=sm_gy; gz=sm_gz;
    return true;
}

// ── EKF arrays (DRAM_ATTR) ────────────────────────────────────
DRAM_ATTR static float ekf_x[NS];
DRAM_ATTR static float ekf_P[NS][NS];
DRAM_ATTR static float ekf_Q[NS];
DRAM_ATTR static float sc_F[NS][NS];
DRAM_ATTR static float sc_FP[NS][NS];
DRAM_ATTR static float sc_FPFt[NS][NS];
DRAM_ATTR static float sc_KH[NS*NS];
DRAM_ATTR static float sc_nP[NS*NS];

// ── Calibration ───────────────────────────────────────────────
float abx=0, aby=0, abz=0;
float gbx=0, gby=0, gbz=0;
bool  calDone=false;
int   calProg=0;

void insertionSort(float* a, int n) {
    for (int i=1; i<n; i++) {
        float k=a[i]; int j=i-1;
        while (j>=0 && a[j]>k) { a[j+1]=a[j]; j--; }
        a[j+1]=k;
    }
}
#define CAL_SAMPLES 3000
static float calBufGz[CAL_SAMPLES];

// ── Navigation state ──────────────────────────────────────────
double phoneLat=0, phoneLon=0;
float  phoneAlt=0;
double gpsLatOff=0, gpsLonOff=0;
bool   gpsCal=false, gpsJammed=false, gpsConnected=false;
int    gpsWarmup=0;
float  baroOff=0, startBaro=0;
unsigned long lastGpsMs=0;
#define GPS_TMO 5000UL
bool   running=false;
unsigned long lastUs=0, insStartMs=0;

// ── Target coordinates ────────────────────────────────────────
double targetLat=0, targetLon=0;
bool   targetSet=false;
float  targetRadiusM=10.0f;
bool   targetReached=false;
unsigned long targetBeepEnd=0;

// ── Complementary filter ──────────────────────────────────────
float cf_roll=0, cf_pitch=0;
#define CF_ALPHA 0.96f

// ── Telemetry ─────────────────────────────────────────────────
double tLat=0, tLon=0;
float  tAlt=0, tVn=0, tVe=0, tVd=0;
float  tRoll=0, tPitch=0, tYaw=0, tHdg=0, tGpsErr=0;
float  tBaroAlt=0, tGyrX=0, tGyrY=0, tGyrZ=0;
float  tTempC=0, tPressPa=0;
float  tTargetDistM=0;
bool   tJam=false, tFix=false, tGpsCal=false, tStat=false, tTargetSet=false;
int    gSats=0;
float  gHdop=99, gSpeedKmh=0, gCourseDeg=0;
uint32_t gUtcTime=0;
float  rawAx=0, rawAy=0, rawAz=0;
float  rawGx=0, rawGy=0, rawGz=0;
float  rawBaro=0, rawGpsHdop=0;
double rawGpsLat=0, rawGpsLon=0;

// ── Drift monitor & LEDs ──────────────────────────────────────
static unsigned long lastDebugMs  = 0;
static unsigned long lastDriftLog = 0;
static float lastPosN=0, lastPosE=0;

// ── Bump test ─────────────────────────────────────────────────
bool   bumpTestActive=false;
double bumpTestStartLat=0, bumpTestStartLon=0;
float  bumpTestError=0;

// ── ZUPT ──────────────────────────────────────────────────────
#define ZW 32
static float zbuf[ZW];
static int   zidx=0;
static bool  zfull=false;

bool motionDetect(float ax, float ay, float az) {
    float m = sqrtf(ax*ax + ay*ay + az*az);
    if (m < 1.0f || m > 25.0f) return false;
    zbuf[zidx]=m; zidx=(zidx+1)%ZW;
    if (zidx==0) zfull=true;
    int n = zfull ? ZW : zidx;
    if (n < 8) return false;
    float mean=0;
    for (int i=0;i<n;i++) mean+=zbuf[i]; mean/=n;
    float var=0;
    for (int i=0;i<n;i++) var+=sq(zbuf[i]-mean); var/=n;
    return (var < 0.003f) && (fabsf(mean-9.80665f) < 0.4f);
}

// ── EKF forward declarations ──────────────────────────────────
void ekf_update(int n, float* H, float* z, float* R);

void ekfSanitize() {
    for (int i=0; i<NS; i++) {
        if (!isfinite(ekf_x[i]) || fabsf(ekf_x[i]) > 1e6f) {
            ekf_x[i]=0;
            for (int j=0;j<NS;j++) ekf_P[i][j]=ekf_P[j][i]=0;
            ekf_P[i][i]=1.0f;
        }
    }
    for (int i=0;i<NS;i++)
        if (!isfinite(ekf_P[i][i])||ekf_P[i][i]<0||ekf_P[i][i]>1e6f)
            ekf_P[i][i]=0.5f;
    while (ekf_x[8] >  M_PI) ekf_x[8] -= 2.0f*M_PI;
    while (ekf_x[8] < -M_PI) ekf_x[8] += 2.0f*M_PI;
}

void updateZUPT() {
    float H[3*NS]; memset(H,0,sizeof(H));
    H[0*NS+3]=1; H[1*NS+4]=1; H[2*NS+5]=1;
    float z[3]={0,0,0}, R[3]={0.0001f,0.0001f,0.0005f};
    ekf_update(3,H,z,R);
    ekf_x[3]=0; ekf_x[4]=0; ekf_x[5]=0;
    ekf_P[3][3]=1e-6f; ekf_P[4][4]=1e-6f; ekf_P[5][5]=1e-6f;
    ekf_P[0][0]=fminf(ekf_P[0][0],0.01f);
    ekf_P[1][1]=fminf(ekf_P[1][1],0.01f);
    ekf_P[2][2]=fminf(ekf_P[2][2],0.01f);
}

void ekf_init() {
    for (int i=0;i<NS;i++) ekf_x[i]=0;
    for (int i=0;i<NS;i++) for (int j=0;j<NS;j++) ekf_P[i][j]=(i==j)?0.5f:0;
    ekf_Q[0]=ekf_Q[1]=ekf_Q[2]=1e-9f;
    ekf_Q[3]=ekf_Q[4]=ekf_Q[5]=1e-6f;
    ekf_Q[6]=ekf_Q[7]=ekf_Q[8]=1e-5f;
    ekf_Q[9]=ekf_Q[10]=ekf_Q[11]=1e-9f;
    ekf_Q[12]=ekf_Q[13]=ekf_Q[14]=1e-10f;
}
void mat_mul(float C[NS][NS],float A[NS][NS],float B[NS][NS]){
    for(int i=0;i<NS;i++) for(int j=0;j<NS;j++){float v=0;for(int k=0;k<NS;k++)v+=A[i][k]*B[k][j];C[i][j]=v;}
}
void mat_mulT(float C[NS][NS],float A[NS][NS],float B[NS][NS]){
    for(int i=0;i<NS;i++) for(int j=0;j<NS;j++){float v=0;for(int k=0;k<NS;k++)v+=A[i][k]*B[j][k];C[i][j]=v;}
}
void propagate_P(float F[NS][NS]){
    mat_mul(sc_FP,F,ekf_P); mat_mulT(sc_FPFt,sc_FP,F);
    for(int i=0;i<NS;i++) for(int j=0;j<NS;j++)
        ekf_P[i][j]=sc_FPFt[i][j]+(i==j?ekf_Q[i]:0);
}
void ekf_update(int n,float*H,float*z,float*R){
    float S9[9]={};
    for(int i=0;i<n;i++) for(int j=0;j<n;j++){
        float v=0;
        for(int a=0;a<NS;a++){float hp=0;for(int b2=0;b2<NS;b2++)hp+=H[i*NS+b2]*ekf_P[b2][a];v+=hp*H[j*NS+a];}
        S9[i*3+j]=v+(i==j?R[i]:0);
    }
    float Si9[9]={};
    if(n==1){float d=S9[0];if(fabsf(d)<1e-12f)d=1e-12f;Si9[0]=1.0f/d;}
    else if(n==2){float d=S9[0]*S9[4]-S9[1]*S9[3];if(fabsf(d)<1e-12f)d=1e-12f;Si9[0]=S9[4]/d;Si9[1]=-S9[1]/d;Si9[3]=-S9[3]/d;Si9[4]=S9[0]/d;}
    else{
        float d=S9[0]*(S9[4]*S9[8]-S9[5]*S9[7])-S9[1]*(S9[3]*S9[8]-S9[5]*S9[6])+S9[2]*(S9[3]*S9[7]-S9[4]*S9[6]);
        if(fabsf(d)<1e-12f)d=1e-12f;
        Si9[0]=(S9[4]*S9[8]-S9[5]*S9[7])/d;Si9[1]=(S9[2]*S9[7]-S9[1]*S9[8])/d;Si9[2]=(S9[1]*S9[5]-S9[2]*S9[4])/d;
        Si9[3]=(S9[5]*S9[6]-S9[3]*S9[8])/d;Si9[4]=(S9[0]*S9[8]-S9[2]*S9[6])/d;Si9[5]=(S9[2]*S9[3]-S9[0]*S9[5])/d;
        Si9[6]=(S9[3]*S9[7]-S9[4]*S9[6])/d;Si9[7]=(S9[1]*S9[6]-S9[0]*S9[7])/d;Si9[8]=(S9[0]*S9[4]-S9[1]*S9[3])/d;
    }
    float K45[NS*3];memset(K45,0,sizeof(K45));
    for(int i=0;i<NS;i++) for(int j=0;j<n;j++){
        float v=0;for(int m=0;m<n;m++){float ph=0;for(int b2=0;b2<NS;b2++)ph+=ekf_P[i][b2]*H[m*NS+b2];v+=ph*Si9[m*3+j];}
        K45[i*3+j]=v;
    }
    float inn[3]={};
    for(int i=0;i<n;i++){float hx=0;for(int b2=0;b2<NS;b2++)hx+=H[i*NS+b2]*ekf_x[b2];inn[i]=z[i]-hx;}
    for(int i=0;i<NS;i++) for(int j=0;j<n;j++) ekf_x[i]+=K45[i*3+j]*inn[j];
    memset(sc_KH,0,sizeof(sc_KH));
    for(int i=0;i<NS;i++) for(int j=0;j<NS;j++) for(int m=0;m<n;m++) sc_KH[i*NS+j]+=K45[i*3+m]*H[m*NS+j];
    memset(sc_nP,0,sizeof(sc_nP));
    for(int i=0;i<NS;i++) for(int j=0;j<NS;j++){
        float v=0;for(int k=0;k<NS;k++)v+=((i==k?1.0f:0.0f)-sc_KH[i*NS+k])*ekf_P[k][j];sc_nP[i*NS+j]=v;
    }
    for(int i=0;i<NS;i++) for(int j=0;j<NS;j++) ekf_P[i][j]=sc_nP[i*NS+j];
}
float distM(double la1,double lo1,double la2,double lo2){
    double dN=(la2-la1)*111111.0, dE=(lo2-lo1)*111111.0*cos(la1*DEG_TO_RAD);
    return (float)sqrt(dN*dN+dE*dE);
}
uint8_t nmea_ck(const char*s){uint8_t c=0;while(*s)c^=(uint8_t)(*s++);return c;}

// ── Web page (normal C string, no raw literal issues) ─────────
static const char PAGE[] PROGMEM = 
"<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1,maximum-scale=1\">"
"<title>Inertial Navigation System</title><style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{background:#060c10;color:#b8dce8;font-family:monospace;font-size:13px;padding:6px}"
"h1{color:#00cfff;text-align:center;font-size:14px;letter-spacing:3px;padding:8px 0 2px}"
".sub{text-align:center;font-size:10px;color:#1e3a4a;margin-bottom:8px}"
".p{background:#09141c;border:1px solid #0c3050;border-radius:3px;padding:7px;margin-bottom:5px}"
".ph{font-size:10px;color:#00cfff;letter-spacing:2px;border-bottom:1px solid #0c3050;padding-bottom:3px;margin-bottom:5px}"
".rw{display:flex;justify-content:space-between;padding:2px 0}"
".lb{font-size:10px;color:#1e3a4a}"
".vl{font-size:12px;font-weight:bold;color:#00ff88}"
".vc{color:#00cfff}.va{color:#ffb800}.vr{color:#ff3355}"
".sb{display:flex;justify-content:space-between;align-items:center;background:#09141c;border:1px solid #0c3050;border-radius:3px;padding:5px 8px;margin-bottom:5px;gap:4px;flex-wrap:wrap}"
".bd{padding:2px 7px;border-radius:2px;font-size:10px}"
".ok{background:#002a18;color:#00ff88;border:1px solid #00ff88}"
".wt{background:#150f00;color:#ffb800;border:1px solid #ffb800}"
".jm{background:#150005;color:#ff3355;border:1px solid #ff3355}"
".g2{display:grid;grid-template-columns:1fr 1fr;gap:4px}"
".btn{width:100%;padding:10px;margin:4px 0;background:transparent;font-family:monospace;font-size:11px;border-radius:3px;cursor:pointer}"
".bc{color:#00cfff;border:1px solid #00cfff}"
".bg{color:#00ff88;border:1px solid #00ff88;font-weight:bold}"
".ba{color:#ffb800;border:1px solid #ffb800}"
".br{color:#ff3355;border:1px solid #ff3355}"
"input{width:100%;background:#09141c;color:#00ff88;border:1px solid #0c3050;border-radius:3px;padding:7px;margin:3px 0;font-family:monospace;font-size:12px}"
".cb{height:4px;background:#0c3050;border-radius:2px;margin:4px 0}"
".cf{height:100%;background:#00ff88;border-radius:2px;transition:width .3s}"
"hr{border:none;border-top:1px solid #0c3050;margin:5px 0}"
"#msg{text-align:center;font-size:10px;color:#ffb800;min-height:14px;margin-top:3px}"
".warn{background:#150005;border:1px solid #ff3355;color:#ff3355;border-radius:3px;text-align:center;padding:5px;font-size:10px;margin-bottom:5px}"
"canvas{display:block}"
"#LD{text-align:center;padding:30px;color:#00cfff;font-size:11px}"
"</style></head><body>"
"<h1>Inertial Navigation System</h1><div class=\"sub\">// UAV || MISSILE || ROV \\</div>"
"<div id=\"LD\">CONNECTING...</div>"
"<div id=\"SU\" style=\"display:none\"><div class=\"p\"><div class=\"ph\">CALIBRATION</div>"
"<div class=\"warn\" id=\"ws\">&#9888; KEEP DEVICE ON FLAT SURFACE. DO NOT TOUCH OR VIBRATE &#9888;</div>"
"<div class=\"rw\"><span class=\"lb\">STATUS</span><span class=\"vl va\" id=\"cs\">PENDING</span></div>"
"<div class=\"cb\"><div class=\"cf\" id=\"cf\" style=\"width:0%\"></div></div>"
"<div class=\"rw\"><span class=\"lb\">BARO ALT</span><span class=\"vl vc\" id=\"bL\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">TEMP</span><span class=\"vl\" id=\"tL\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">PRESSURE</span><span class=\"vl\" id=\"pL\">--</span></div></div>"
"<div class=\"p\"><div class=\"ph\">DATE / TIME</div><div style=\"display:flex;gap:4px\">"
"<input id=\"dtI\" placeholder=\"YYYYMMDD_HHMMSS\" style=\"flex:1;margin:0\">"
"<button class=\"btn bc\" style=\"width:auto;padding:7px 10px;margin:0;font-size:10px\" onclick=\"syncDT()\">AUTO</button></div></div>"
"<div class=\"p\"><div class=\"ph\">TARGET COORDINATES (OPTIONAL)</div>"
"<input id=\"tgLat\" placeholder=\"Target Latitude\" type=\"number\" step=\"any\">"
"<input id=\"tgLon\" placeholder=\"Target Longitude\" type=\"number\" step=\"any\">"
"<input id=\"tgRad\" placeholder=\"Arrival radius (m, default 10)\" type=\"number\" step=\"any\" value=\"10\">"
"<div style=\"font-size:10px;color:#1e3a4a;margin-top:3px\">LED+beep when within radius. Leave blank to disable.</div></div>"
"<button class=\"btn bc\" onclick=\"phoneGPS()\">USE PHONE GPS FOR ORIGIN</button>"
"<button class=\"btn bc\" onclick=\"openMaps()\">OR PICK IN GOOGLE MAPS</button>"
"<input id=\"la\" placeholder=\"Origin Latitude\" type=\"number\" step=\"any\">"
"<input id=\"lo\" placeholder=\"Origin Longitude\" type=\"number\" step=\"any\">"
"<input id=\"al\" placeholder=\"Altitude (m MSL)\" type=\"number\" step=\"any\">"
"<hr><button class=\"btn bg\" onclick=\"startINS()\">START INS</button><div id=\"msg\"></div></div>"
"<div id=\"DA\" style=\"display:none\"><div class=\"sb\"><span id=\"sbd\" class=\"bd wt\">INIT</span>"
"<span id=\"gst\" style=\"color:#ffb800\">GPS:--</span><span id=\"clk\" style=\"color:#00cfff\">--:--:--</span>"
"<span id=\"upt\" style=\"color:#1e3a4a\">T+0s</span>"
"<button class=\"btn br\" id=\"rB\" style=\"width:auto;padding:3px 9px;margin:0;font-size:10px\" onclick=\"doReset()\">RESET</button></div>"
"<div class=\"g2\" style=\"margin-bottom:4px\"><button class=\"btn ba\" id=\"bumpBtn\" onclick=\"startBumpTest()\" style=\"margin:0;padding:8px;font-size:10px\">BUMP TEST</button>"
"<button class=\"btn bc\" onclick=\"viewMap()\" style=\"margin:0;padding:8px;font-size:10px\">VIEW MAP</button></div>"
"<div class=\"g2\" style=\"margin-bottom:4px\"><div class=\"p\" style=\"padding:4px\"><div class=\"ph\">HORIZON</div><canvas id=\"cH\" width=\"148\" height=\"112\"></canvas></div>"
"<div class=\"p\" style=\"padding:4px\"><div class=\"ph\">COMPASS</div><canvas id=\"cC\" width=\"148\" height=\"112\"></canvas></div></div>"
"<div class=\"p\" style=\"padding:4px;margin-bottom:4px\"><div class=\"ph\">TURN RATE</div><canvas id=\"cT\" width=\"290\" height=\"36\"></canvas></div>"
"<div class=\"p\" style=\"padding:4px;margin-bottom:4px\"><div class=\"ph\" style=\"display:flex;justify-content:space-between\"><span>LIVE TRAIL MAP</span>"
"<span style=\"display:flex;gap:6px\"><button onclick=\"mapZoomIn()\" style=\"background:transparent;color:#00cfff;border:1px solid #0c3050;border-radius:2px;padding:1px 7px;cursor:pointer;font-size:11px\">+</button>"
"<button onclick=\"mapZoomOut()\" style=\"background:transparent;color:#00cfff;border:1px solid #0c3050;border-radius:2px;padding:1px 7px;cursor:pointer;font-size:11px\">-</button>"
"<button onclick=\"mapClear()\" style=\"background:transparent;color:#ff3355;border:1px solid #ff3355;border-radius:2px;padding:1px 7px;cursor:pointer;font-size:10px\">CLR</button></span></div>"
"<canvas id=\"cMap\" width=\"290\" height=\"200\" style=\"width:100%;border:1px solid #0c3050;border-radius:2px;margin-top:3px\"></canvas>"
"<div id=\"mapScale\" style=\"font-size:9px;color:#1e3a4a;text-align:right;margin-top:2px\">scale: --</div></div>"
"<div id=\"tgtPanel\" class=\"p\" style=\"margin-bottom:4px;display:none\"><div class=\"ph\">TARGET</div>"
"<div class=\"rw\"><span class=\"lb\">DISTANCE</span><span class=\"vl va\" id=\"tgtDist\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">STATUS</span><span class=\"vl\" id=\"tgtStat\">EN ROUTE</span></div></div>"
"<div class=\"g2\"><div class=\"p\"><div class=\"ph\">POSITION</div>"
"<div class=\"rw\"><span class=\"lb\">LAT</span><span class=\"vl vc\" id=\"dA\" style=\"font-size:10px\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">LON</span><span class=\"vl vc\" id=\"dO\" style=\"font-size:10px\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">ALT MSL</span><span class=\"vl\" id=\"dL\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">BARO</span><span class=\"vl vc\" id=\"dB\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">GPS ERR</span><span class=\"vl va\" id=\"dE\">--</span></div></div>"
"<div class=\"p\"><div class=\"ph\" style=\"display:flex;justify-content:space-between;align-items:center\"><span>VELOCITY</span>"
"<button id=\"unitBtn\" onclick=\"togUnit()\" style=\"background:transparent;color:#00cfff;border:1px solid #0c3050;border-radius:2px;padding:1px 7px;cursor:pointer;font-size:9px\">m/s</button></div>"
"<div class=\"rw\"><span class=\"lb\">NORTH</span><span class=\"vl\" id=\"dVn\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">EAST</span><span class=\"vl\" id=\"dVe\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">DOWN</span><span class=\"vl vc\" id=\"dVd\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">H-SPD</span><span class=\"vl\" id=\"dSp\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">3D-SPD</span><span class=\"vl\" id=\"dS3\">--</span></div></div>"
"<div class=\"p\"><div class=\"ph\">ATTITUDE</div>"
"<div class=\"rw\"><span class=\"lb\">ROLL</span><span class=\"vl\" id=\"dRo\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">PITCH</span><span class=\"vl\" id=\"dPi\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">YAW</span><span class=\"vl vc\" id=\"dYa\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">R-RATE</span><span class=\"vl\" id=\"dRr\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">P-RATE</span><span class=\"vl\" id=\"dPr\">--</span></div></div>"
"<div class=\"p\"><div class=\"ph\">ENVIRONMENT</div>"
"<div class=\"rw\"><span class=\"lb\">TEMP</span><span class=\"vl\" id=\"dTe\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">hPa</span><span class=\"vl vc\" id=\"dHp\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">mmHg</span><span class=\"vl\" id=\"dMh\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">STATIC</span><span class=\"vl\" id=\"dSt\">--</span></div></div></div>"
"<div class=\"p\" style=\"margin-top:4px\"><div class=\"ph\">GPS RECEIVER</div>"
"<div id=\"gNC\" style=\"display:none\"><span class=\"vl vr\">NOT CONNECTED</span></div>"
"<div id=\"gNF\" style=\"display:none\"><span class=\"vl va\">SEARCHING FOR SATELLITES...</span></div>"
"<div id=\"gFX\" style=\"display:none\"><div class=\"g2\"><div>"
"<div class=\"rw\"><span class=\"lb\">SATS</span><span class=\"vl\" id=\"gSt\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">HDOP</span><span class=\"vl\" id=\"gHd\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">SPD</span><span class=\"vl\" id=\"gSp\">--</span></div></div><div>"
"<div class=\"rw\"><span class=\"lb\">UTC</span><span class=\"vl vc\" id=\"gUt\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">BST+6</span><span class=\"vl\" id=\"gLc\">--</span></div>"
"<div class=\"rw\"><span class=\"lb\">COURSE</span><span class=\"vl\" id=\"gCr\">--</span></div></div></div></div></div>"
"<div id=\"bumpResult\" class=\"p\" style=\"display:none;margin-top:4px\"><div class=\"ph\">BUMP TEST RESULT</div>"
"<div class=\"rw\"><span class=\"lb\">DRIFT ERROR</span><span class=\"vl va\" id=\"bumpError\">--</span><span class=\"lb\"> m</span>"
"<span class=\"vl\" id=\"bumpRating\" style=\"margin-left:8px\"></span></div>"
"<button class=\"btn bc\" onclick=\"closeBumpResult()\" style=\"padding:5px;margin-top:4px;font-size:10px\">CLOSE</button></div></div>"
"<script>"
"var t0=null,calPoll=null,rT=null,clkInt=null,gpsSynced=false;"
"var cLat=0,cLon=0,bumpState=0;"
"var uIdx=0,uN=['m/s','km/h','mph','kn'],uM=[1,3.6,2.23694,1.94384];"
"var originLat=0,originLon=0,originSet=false;"
"var targetLat2=0,targetLon2=0,targetSet2=false;"
"var trail=[],mapScale=5,mapOriginN=0,mapOriginE=0;"
"function mapZoomIn(){mapScale=Math.min(mapScale*1.5,200);drawMap();}"
"function mapZoomOut(){mapScale=Math.max(mapScale/1.5,0.1);drawMap();}"
"function mapClear(){trail=[];mapOriginN=0;mapOriginE=0;drawMap();}"
"function addTrailPoint(n,e){trail.push({n:n,e:e});if(trail.length>2000)trail.shift();}"
"function drawMap(){"
"var cv=document.getElementById('cMap'),c=cv.getContext('2d');"
"var W=cv.offsetWidth||290,H=200;cv.width=W;cv.height=H;"
"c.fillStyle='#060c10';c.fillRect(0,0,W,H);"
"var cx2=W/2,cy2=H/2,gridM=10,gridPx=gridM*mapScale;"
"c.strokeStyle='#0c1a28';c.lineWidth=1;"
"for(var x=cx2%gridPx;x<W;x+=gridPx){c.beginPath();c.moveTo(x,0);c.lineTo(x,H);c.stroke();}"
"for(var y=cy2%gridPx;y<H;y+=gridPx){c.beginPath();c.moveTo(0,y);c.lineTo(W,y);c.stroke();}"
"c.strokeStyle='#1e3a4a';c.lineWidth=1;"
"c.beginPath();c.moveTo(cx2-8,cy2);c.lineTo(cx2+8,cy2);c.moveTo(cx2,cy2-8);c.lineTo(cx2,cy2+8);c.stroke();"
"if(targetSet2){"
"var tdN=(targetLat2-originLat)*111111,tdE=(targetLon2-originLon)*111111*Math.cos(originLat*Math.PI/180);"
"var tx=cx2+(tdE-mapOriginE)*mapScale,ty=cy2-(tdN-mapOriginN)*mapScale;"
"c.beginPath();c.arc(tx,ty,5,0,2*Math.PI);c.fillStyle='#ff3355';c.fill();"
"c.strokeStyle='rgba(255,51,85,0.3)';c.lineWidth=1;c.beginPath();c.arc(tx,ty,10*mapScale,0,2*Math.PI);c.stroke();}"
"if(trail.length>1){"
"c.beginPath();c.strokeStyle='#00ff88';c.lineWidth=1.5;c.lineJoin='round';"
"var p0=trail[0];c.moveTo(cx2+(p0.e-mapOriginE)*mapScale,cy2-(p0.n-mapOriginN)*mapScale);"
"for(var i=1;i<trail.length;i++){var p=trail[i];c.lineTo(cx2+(p.e-mapOriginE)*mapScale,cy2-(p.n-mapOriginN)*mapScale);}"
"c.stroke();}"
"if(trail.length>0){"
"var lp=trail[trail.length-1];var px=cx2+(lp.e-mapOriginE)*mapScale,py=cy2-(lp.n-mapOriginN)*mapScale;"
"c.beginPath();c.arc(px,py,4,0,2*Math.PI);c.fillStyle='#00cfff';c.fill();"
"var hdRad=cLat===0?0:(document.getElementById('dYa').textContent.replace(' deg','')*Math.PI/180)||0;"
"c.strokeStyle='#00cfff';c.lineWidth=1.5;c.beginPath();c.moveTo(px,py);c.lineTo(px+Math.sin(hdRad)*12,py-Math.cos(hdRad)*12);c.stroke();}"
"document.getElementById('mapScale').textContent='scale: '+Math.round(50/mapScale)+'m per 50px';}"
"function p2(n){return n<10?'0'+n:''+n;}"
"function startClock(h,m,s){if(clkInt)clearInterval(clkInt);var tot=h*3600+m*60+s;"
"clkInt=setInterval(function(){tot++;var hh=Math.floor(tot/3600)%24,mm=Math.floor((tot%3600)/60),ss=tot%60;"
"document.getElementById('clk').textContent=p2(hh)+':'+p2(mm)+':'+p2(ss);},1000);}"
"function syncDT(){var n=new Date();startClock(n.getHours(),n.getMinutes(),n.getSeconds());"
"var s=n.getFullYear()+p2(n.getMonth()+1)+p2(n.getDate())+'_'+p2(n.getHours())+p2(n.getMinutes())+p2(n.getSeconds());"
"var el=document.getElementById('dtI');if(el)el.value=s;}"
"window.addEventListener('load',function(){syncDT();fetch('/status').then(r=>r.json()).then(d=>{"
"document.getElementById('LD').style.display='none';if(d.running){"
"document.getElementById('DA').style.display='block';t0=Date.now()-d.uptime;setInterval(poll,400);"
"}else{document.getElementById('SU').style.display='block';if(d.calDone){"
"document.getElementById('ws').style.display='none';document.getElementById('cs').textContent='COMPLETE';"
"document.getElementById('cs').className='vl';}startCalPoll();}}).catch(()=>{"
"document.getElementById('LD').style.display='none';document.getElementById('SU').style.display='block';startCalPoll();});});"
"function startCalPoll(){calPoll=setInterval(()=>{fetch('/caldata').then(r=>r.json()).then(d=>{"
"var p=Math.min(100,Math.round(d.p/3000*100));document.getElementById('cf').style.width=p+'%';"
"document.getElementById('cs').textContent=d.done?'COMPLETE':'RUNNING '+p+'%';if(d.done){"
"document.getElementById('ws').style.display='none';document.getElementById('cs').className='vl';clearInterval(calPoll);}});"
"fetch('/env').then(r=>r.json()).then(d=>{document.getElementById('bL').textContent=d.al.toFixed(1)+' m';"
"document.getElementById('tL').textContent=d.tc.toFixed(1)+' C';"
"document.getElementById('pL').textContent=(d.pp/100).toFixed(1)+' hPa';});},1000);}"
"function phoneGPS(){if(!navigator.geolocation){document.getElementById('msg').textContent='No geolocation';return;}"
"document.getElementById('msg').textContent='Acquiring...';"
"navigator.geolocation.getCurrentPosition(p=>{document.getElementById('la').value=p.coords.latitude.toFixed(8);"
"document.getElementById('lo').value=p.coords.longitude.toFixed(8);"
"if(p.coords.altitude)document.getElementById('al').value=p.coords.altitude.toFixed(1);"
"document.getElementById('msg').textContent='GPS acquired.';},e=>document.getElementById('msg').textContent='Err:'+e.message,"
"{enableHighAccuracy:true,timeout:15000});}"
"function openMaps(){window.open('https://maps.google.com','_blank');document.getElementById('msg').textContent='Long-press in Maps to drop pin, copy coords.';}"
"function startINS(){"
"var la=document.getElementById('la').value.trim(),lo=document.getElementById('lo').value.trim(),al=document.getElementById('al').value.trim();"
"if(!la||!lo||!al){document.getElementById('msg').textContent='Fill origin fields.';return;}"
"var tgLa=document.getElementById('tgLat').value.trim(),tgLo=document.getElementById('tgLon').value.trim(),tgRd=document.getElementById('tgRad').value.trim()||'10';"
"originLat=parseFloat(la);originLon=parseFloat(lo);originSet=true;"
"if(tgLa&&tgLo){targetLat2=parseFloat(tgLa);targetLon2=parseFloat(tgLo);targetSet2=true;}"
"document.getElementById('msg').textContent='Starting...';"
"var body='lat='+la+'&lon='+lo+'&alt='+al;if(tgLa&&tgLo)body+='&tgLat='+tgLa+'&tgLon='+tgLo+'&tgRad='+tgRd;"
"fetch('/start',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body})"
".then(r=>r.text()).then(t=>{if(t==='OK'){if(calPoll)clearInterval(calPoll);"
"document.getElementById('SU').style.display='none';document.getElementById('DA').style.display='block';"
"if(targetSet2)document.getElementById('tgtPanel').style.display='block';t0=Date.now();setInterval(poll,400);"
"}else document.getElementById('msg').textContent='Err:'+t;}).catch(()=>document.getElementById('msg').textContent='Comm err.');}"
"function doReset(){if(rT){clearInterval(rT);rT=null;document.getElementById('rB').textContent='RESET';return;}"
"document.getElementById('rB').textContent='CANCEL(5s)';var cnt=5;"
"rT=setInterval(()=>{cnt--;if(cnt<=0){clearInterval(rT);rT=null;fetch('/reset',{method:'POST'}).then(()=>window.location.reload());}"
"else document.getElementById('rB').textContent='CANCEL('+cnt+'s)';},1000);}"
"function togUnit(){uIdx=(uIdx+1)%4;document.getElementById('unitBtn').textContent=uN[uIdx];}"
"function fV(v){return(v*uM[uIdx]).toFixed(3)+' '+uN[uIdx];}"
"function startBumpTest(){var btn=document.getElementById('bumpBtn');"
"if(bumpState===0){fetch('/bump_start',{method:'POST'}).then(r=>r.text()).then(t=>{if(t==='OK'){"
"bumpState=1;btn.textContent='RETURN TO START';btn.style.color='#00ff88';btn.style.borderColor='#00ff88';"
"document.getElementById('msg').textContent='Move ~1m then press RETURN TO START';}});"
"}else{fetch('/bump_return',{method:'POST'}).then(r=>r.json()).then(d=>{"
"bumpState=0;btn.textContent='BUMP TEST';btn.style.color='';btn.style.borderColor='';"
"document.getElementById('bumpResult').style.display='block';"
"document.getElementById('bumpError').textContent=d.error.toFixed(3);"
"var e=d.error,rat=e<0.1?'EXCELLENT':e<0.3?'GOOD':e<0.5?'ACCEPTABLE':'FIX ZUPT/Q';"
"document.getElementById('bumpRating').textContent=rat;"
"document.getElementById('bumpRating').className='vl'+(e<0.3?' vc':e<0.5?' va':' vr');"
"document.getElementById('msg').textContent='Drift: '+d.error.toFixed(3)+'m - '+rat;});}}"
"function closeBumpResult(){document.getElementById('bumpResult').style.display='none';}"
"function poll(){fetch('/data').then(r=>r.json()).then(upd);if(t0)document.getElementById('upt').textContent='T+'+Math.floor((Date.now()-t0)/1000)+'s';}"
"function upd(d){"
"cLat=d.la;cLon=d.lo;if(originSet)addTrailPoint(d.pn,d.pe);drawMap();"
"document.getElementById('dA').textContent=d.la.toFixed(7);"
"document.getElementById('dO').textContent=d.lo.toFixed(7);"
"document.getElementById('dL').textContent=d.al.toFixed(1)+' m';"
"document.getElementById('dB').textContent=d.ba.toFixed(1)+' m';"
"document.getElementById('dE').textContent=d.ge.toFixed(1)+' m';"
"document.getElementById('dVn').textContent=fV(d.vn);"
"document.getElementById('dVe').textContent=fV(d.ve);"
"document.getElementById('dVd').textContent=fV(d.vd);"
"document.getElementById('dSp').textContent=fV(Math.sqrt(d.vn*d.vn+d.ve*d.ve));"
"document.getElementById('dS3').textContent=fV(Math.sqrt(d.vn*d.vn+d.ve*d.ve+d.vd*d.vd));"
"document.getElementById('dRo').textContent=d.ro.toFixed(1)+' deg';"
"document.getElementById('dPi').textContent=d.pi.toFixed(1)+' deg';"
"document.getElementById('dYa').textContent=d.ya.toFixed(1)+' deg';"
"document.getElementById('dRr').textContent=d.gx.toFixed(2)+' d/s';"
"document.getElementById('dPr').textContent=d.gy.toFixed(2)+' d/s';"
"document.getElementById('dTe').textContent=d.tc.toFixed(1)+' C';"
"document.getElementById('dHp').textContent=(d.pp/100).toFixed(1);"
"document.getElementById('dMh').textContent=(d.pp*0.00750062).toFixed(1);"
"var driftSpd=Math.sqrt(d.vn*d.vn+d.ve*d.ve+d.vd*d.vd),stEl=document.getElementById('dSt');"
"if(d.st){stEl.textContent='YES';stEl.className='vl vc';}"
"else if(driftSpd>0.05&&!d.fx){stEl.textContent='DRIFT?';stEl.className='vl vr';}"
"else{stEl.textContent='NO';stEl.className='vl va';}"
"var sb=document.getElementById('sbd'),gs=document.getElementById('gst');"
"if(d.jam){sb.textContent='GPS JAMMED';sb.className='bd jm';gs.textContent='GPS:JAM';gs.style.color='#ff3355';}"
"else if(d.fx){sb.textContent='GPS+INS';sb.className='bd ok';gs.textContent='GPS:OK';gs.style.color='#00ff88';}"
"else{sb.textContent='INS ONLY';sb.className='bd wt';gs.textContent='GPS:SRCH';gs.style.color='#ffb800';}"
"var nc=document.getElementById('gNC'),nf=document.getElementById('gNF'),fx=document.getElementById('gFX');"
"if(!d.gconn){nc.style.display='block';nf.style.display='none';fx.style.display='none';}"
"else if(!d.fx){nc.style.display='none';nf.style.display='block';fx.style.display='none';}"
"else{nc.style.display='none';nf.style.display='none';fx.style.display='block';"
"document.getElementById('gSt').textContent=d.gs;"
"document.getElementById('gHd').textContent=d.gh.toFixed(1);"
"document.getElementById('gSp').textContent=d.gsk.toFixed(1)+' km/h';"
"document.getElementById('gCr').textContent=d.gco.toFixed(1)+' deg';"
"var ut=d.gut,uh=Math.floor(ut/10000),um2=Math.floor((ut%10000)/100),us2=ut%100;"
"document.getElementById('gUt').textContent=p2(uh)+':'+p2(um2)+':'+p2(us2)+' UTC';"
"document.getElementById('gLc').textContent=p2((uh+6)%24)+':'+p2(um2)+':'+p2(us2)+' BST';"
"if(!gpsSynced){gpsSynced=true;startClock((uh+6)%24,um2,us2);}}"
"if(d.tset){document.getElementById('tgtDist').textContent=d.tdist.toFixed(1)+' m';"
"document.getElementById('tgtStat').textContent=d.treach?'ARRIVED!':'EN ROUTE';"
"document.getElementById('tgtStat').className='vl'+(d.treach?' vc':' va');}"
"drawH(d.ro,d.pi);drawC(d.hd);drawT(d.gz);}"
"function viewMap(){if(cLat===0&&cLon===0)return;window.open('https://maps.google.com/?q='+cLat.toFixed(7)+','+cLon.toFixed(7),'_blank');}"
"function drawH(roll,pitch){"
"var cv=document.getElementById('cH'),c=cv.getContext('2d');"
"var W=cv.width,H=cv.height,cx=W/2,cy=H/2;"
"c.clearRect(0,0,W,H);c.save();c.beginPath();c.rect(0,0,W,H);c.clip();c.save();"
"c.translate(cx,cy);c.rotate(roll*Math.PI/180);"
"var ph=pitch*1.5;"
"c.fillStyle='#003355';c.fillRect(-W,-H*3,W*2,H*3-ph);"
"c.fillStyle='#2a1500';c.fillRect(-W,-ph,W*2,H*3);"
"c.strokeStyle='#00ff88';c.lineWidth=1.5;c.beginPath();c.moveTo(-W,-ph);c.lineTo(W,-ph);c.stroke();"
"c.font='8px monospace';c.textAlign='left';c.textBaseline='middle';"
"for(var deg=-20;deg<=20;deg+=10){if(deg===0)continue;var py=-ph-deg*1.5,lw=deg%20===0?30:20;"
"c.strokeStyle='rgba(0,255,136,0.45)';c.lineWidth=1;c.beginPath();c.moveTo(-lw,py);c.lineTo(lw,py);c.stroke();"
"c.fillStyle='rgba(184,220,232,0.65)';c.fillText((deg>0?'+':'')+deg,lw+3,py);}"
"c.restore();"
"c.strokeStyle='#ffb800';c.lineWidth=2;"
"c.beginPath();c.moveTo(cx-30,cy);c.lineTo(cx-12,cy);c.moveTo(cx+12,cy);c.lineTo(cx+30,cy);c.moveTo(cx,cy-5);c.lineTo(cx,cy+5);c.stroke();"
"c.font='bold 9px monospace';c.textAlign='center';c.textBaseline='top';"
"if(pitch>2){c.fillStyle='#00ff88';c.fillText('NOSE UP +'+pitch.toFixed(1)+'deg',cx,2);}"
"else if(pitch<-2){c.fillStyle='#ff3355';c.fillText('NOSE DN '+pitch.toFixed(1)+'deg',cx,2);}"
"else{c.fillStyle='rgba(30,58,74,0.8)';c.fillText('LEVEL',cx,2);}"
"c.restore();c.strokeStyle='#0c3050';c.lineWidth=1;c.strokeRect(0,0,W,H);}"
"function drawC(mag){"
"var cv=document.getElementById('cC'),c=cv.getContext('2d');"
"var W=cv.width,H=cv.height,cx=W/2,cy=H/2+4,R=Math.min(cx,cy-4)-2;"
"c.clearRect(0,0,W,H);c.beginPath();c.arc(cx,cy,R,0,2*Math.PI);c.fillStyle='#09141c';c.fill();c.strokeStyle='#0c3050';c.lineWidth=1.5;c.stroke();"
"for(var i=0;i<36;i++){var a=i*10*Math.PI/180,r1=R-(i%3===0?9:4),r2=R-1;c.beginPath();c.moveTo(cx+Math.sin(a)*r1,cy-Math.cos(a)*r1);c.lineTo(cx+Math.sin(a)*r2,cy-Math.cos(a)*r2);c.strokeStyle=i%3===0?'#1e3a4a':'#0c2030';c.lineWidth=1;c.stroke();}"
"var cd=['N','E','S','W'];c.font='bold 9px monospace';c.textAlign='center';c.textBaseline='middle';"
"for(var i=0;i<4;i++){var a2=i*90*Math.PI/180,lr=R-14;c.fillStyle=i===0?'#ff3355':'#b8dce8';c.fillText(cd[i],cx+Math.sin(a2)*lr,cy-Math.cos(a2)*lr);}"
"var na=mag*Math.PI/180;c.beginPath();c.moveTo(cx,cy);c.lineTo(cx+Math.sin(na)*(R-16),cy-Math.cos(na)*(R-16));c.strokeStyle='#ff3355';c.lineWidth=2.5;c.lineCap='round';c.stroke();"
"c.beginPath();c.moveTo(cx,cy);c.lineTo(cx-Math.sin(na)*13,cy+Math.cos(na)*13);c.strokeStyle='#1e3a4a';c.lineWidth=2;c.stroke();"
"c.beginPath();c.arc(cx,cy,3,0,2*Math.PI);c.fillStyle='#00cfff';c.fill();"
"c.fillStyle='#00ff88';c.font='9px monospace';c.textAlign='center';c.textBaseline='top';c.fillText(Math.round(mag)+' deg',cx,2);}"
"function drawT(yr){"
"var cv=document.getElementById('cT'),c=cv.getContext('2d'),W=cv.width,H=cv.height,cx=W/2,cy=H/2;"
"if(!W||!H)return;c.clearRect(0,0,W,H);"
"var mx=6.0;for(var i=-3;i<=3;i++){var x=cx+i*(W/2/mx)*2;c.beginPath();c.moveTo(x,cy-7);c.lineTo(x,cy+7);c.strokeStyle=i===0?'#00ff88':'#1e3a4a';c.lineWidth=i===0?1.5:1;c.stroke();}"
"var cl=Math.max(-mx,Math.min(mx,yr)),nx=cx+cl*(W/2)/mx;"
"c.fillStyle=Math.abs(yr)>0.5?'#ffb800':'#00ff88';c.fillRect(nx-2,cy-9,4,18);"
"c.fillStyle='#1e3a4a';c.font='9px monospace';c.textAlign='center';c.fillText('L',10,cy+4);c.fillText('R',W-10,cy+4);"
"c.fillStyle='#00cfff';c.fillText(yr.toFixed(1)+' d/s',cx,H-1);}"
"</script></body></html>";

// ── Forward declarations ───────────────────────────────────────
void runCalibration();
void predictStep(float dt);
void updateBaro();
void updateGPS(double cLat, double cLon);
void sendNMEA();
void handleStart();
void handleData();

void setup() {
    Serial.begin(115200); delay(600);
    Serial.println("\n===== INS v14.6 =====");
    Serial.printf("Heap: %u\n", ESP.getFreeHeap());

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t wc={.timeout_ms=15000,.idle_core_mask=0,.trigger_panic=true};
    esp_task_wdt_reconfigure(&wc);
#else
    esp_task_wdt_init(15,true);
#endif
    esp_task_wdt_add(NULL);

    SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    SerialPix.begin(38400, SERIAL_8N1, PIX_RX, PIX_TX);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    delay(300);

    Serial.println("I2C scan:");
    for (uint8_t a=1; a<127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission()==0) Serial.printf("  0x%02X\n",a);
    }

    Serial.print("MPU6050 ");
    bool mok=false;
    for (int i=0;i<5;i++) { if (mpuInit()){mok=true;break;} Serial.print("."); delay(300); }
    if (!mok) { Serial.println("FAIL"); while(1){esp_task_wdt_reset();delay(500);} }
    Serial.println("OK");

    Serial.print("BMP280 ");
    bool bok=bmp.begin(0x76); if(!bok) bok=bmp.begin(0x77);
    if (!bok) { Serial.println("FAIL"); while(1){esp_task_wdt_reset();delay(500);} }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,   Adafruit_BMP280::STANDBY_MS_1);
    Serial.println("OK");

    Serial.print("QMC5883 "); compass.init(); Serial.println("OK");
    Wire.setClock(400000);

    pinMode(LED_DRIFT,  OUTPUT); digitalWrite(LED_DRIFT,  LOW);
    pinMode(LED_GPS,    OUTPUT); digitalWrite(LED_GPS,    LOW);
    pinMode(LED_STATIC, OUTPUT); digitalWrite(LED_STATIC, LOW);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

    ekf_init();

    WiFi.softAP("INS by Jubair", "98765ins");
    Serial.printf("AP: %s\n", WiFi.softAPIP().toString().c_str());

    server.on("/",HTTP_GET,[](){server.send_P(200,"text/html",PAGE);});
    server.on("/status",HTTP_GET,[](){
        char b[80];
        snprintf(b,sizeof(b),"{\"running\":%s,\"calDone\":%s,\"uptime\":%lu}",
            running?"true":"false", calDone?"true":"false",
            (unsigned long)(running?millis()-insStartMs:0));
        server.send(200,"application/json",b);
    });
    server.on("/caldata",HTTP_GET,[](){
        char b[48];
        snprintf(b,sizeof(b),"{\"done\":%s,\"p\":%d}",calDone?"true":"false",calProg);
        server.send(200,"application/json",b);
    });
    server.on("/env",HTTP_GET,[](){
        char b[80];
        snprintf(b,sizeof(b),"{\"al\":%.2f,\"tc\":%.2f,\"pp\":%.1f}",
            (double)bmp.readAltitude(1013.25f),(double)bmp.readTemperature(),(double)bmp.readPressure());
        server.send(200,"application/json",b);
    });
    server.on("/reset",HTTP_POST,[](){
        running=false; gpsCal=false; gpsJammed=false; gpsWarmup=0;
        tFix=false; tJam=false; tGpsErr=0;
        targetSet=false; targetReached=false;
        digitalWrite(BUZZER_PIN,LOW);
        server.send(200,"text/plain","OK");
    });
    server.on("/bump_start",HTTP_POST,[](){
        bumpTestStartLat=tLat; bumpTestStartLon=tLon;
        bumpTestActive=true;
        server.send(200,"text/plain","OK");
    });
    server.on("/bump_return",HTTP_POST,[](){
        bumpTestError=distM(bumpTestStartLat,bumpTestStartLon,tLat,tLon);
        bumpTestActive=false;
        char b[48];
        snprintf(b,sizeof(b),"{\"error\":%.4f}",(double)bumpTestError);
        server.send(200,"application/json",b);
    });
    server.on("/start",HTTP_POST,handleStart);
    server.on("/data", HTTP_GET, handleData);
    server.begin();

    runCalibration();
    Serial.println("Ready. SSID:INS by Jubair Pass:98765ins");
}

void loop() {
    esp_task_wdt_reset();
    server.handleClient();
    while (SerialGPS.available()) gps.encode(SerialGPS.read());
    if (!gpsConnected && gps.charsProcessed()>20) {
        gpsConnected=true;
        Serial.printf("GPS connected\n");
    }
    if (millis() < targetBeepEnd) {
        digitalWrite(BUZZER_PIN, (millis()/250)%2 ? HIGH : LOW);
    } else {
        digitalWrite(BUZZER_PIN, LOW);
    }
    if (!running) return;

    unsigned long now=micros();
    float dt=(float)(now-lastUs)*1e-6f;
    if (dt<0.005f) return;
    if (dt>0.10f) dt=0.02f;
    lastUs=now;

    predictStep(dt);
    ekfSanitize();

    bool st=motionDetect(rawAx, rawAy, rawAz);
    tStat=st;
    if (st) {
        updateZUPT();
    } else if (!tFix) {
        ekf_x[3]*=0.9990f;
        ekf_x[4]*=0.9990f;
        ekf_x[5]*=0.9998f;
    }

    static unsigned long lastCompassMs=0;
    if (millis()-lastCompassMs>=50) {
        lastCompassMs=millis();
        compass.read();
        tHdg=(float)compass.getAzimuth();
    }
    tTempC=bmp.readTemperature();
    tPressPa=bmp.readPressure();
    updateBaro();

    bool used=false;
    if (gps.location.isValid() && gps.location.age()<GPS_TMO) {
        lastGpsMs=millis();
        rawGpsLat=gps.location.lat(); rawGpsLon=gps.location.lng();
        rawGpsHdop=gps.hdop.isValid()?(float)gps.hdop.value()/100.0f:99.0f;
        gSats=gps.satellites.isValid()?gps.satellites.value():0;
        gHdop=rawGpsHdop;
        if (gps.time.isValid())   gUtcTime=gps.time.value();
        if (gps.speed.isValid())  gSpeedKmh=(float)gps.speed.kmph();
        if (gps.course.isValid()) gCourseDeg=(float)gps.course.deg();
        if (!gpsCal) {
            gpsLatOff=phoneLat-gps.location.lat();
            gpsLonOff=phoneLon-gps.location.lng();
            gpsCal=true; gpsWarmup=0;
        }
        double cLat=gps.location.lat()+gpsLatOff;
        double cLon=gps.location.lng()+gpsLonOff;
        if (gpsWarmup<15) {
            gpsWarmup++; gpsJammed=false;
            updateGPS(cLat,cLon); used=true;
        } else {
            double eLat=phoneLat+(ekf_x[0]/111111.0);
            double eLon=phoneLon+(ekf_x[1]/(111111.0*cos(phoneLat*DEG_TO_RAD)));
            tGpsErr=distM(eLat,eLon,cLat,cLon);
            if (tGpsErr<2000.0f) {
                gpsJammed=false; updateGPS(cLat,cLon); used=true;
            } else {
                gpsJammed=true;
            }
        }
    } else if (lastGpsMs && (millis()-lastGpsMs)>GPS_TMO) {
        gpsJammed=true;
    }

    tLat=phoneLat+(ekf_x[0]/111111.0);
    tLon=phoneLon+(ekf_x[1]/(111111.0*cos(phoneLat*DEG_TO_RAD)));
    tAlt=phoneAlt-ekf_x[2];
    tBaroAlt=rawBaro+baroOff;
    tVn=ekf_x[3]; tVe=ekf_x[4]; tVd=ekf_x[5];
    tRoll=cf_roll; tPitch=cf_pitch;
    tYaw=ekf_x[8]*RAD_TO_DEG;
    tGyrX=rawGx*RAD_TO_DEG; tGyrY=rawGy*RAD_TO_DEG; tGyrZ=rawGz*RAD_TO_DEG;
    tJam=gpsJammed; tFix=used; tGpsCal=gpsCal;

    if (targetSet) {
        tTargetDistM=distM(tLat,tLon,targetLat,targetLon);
        tTargetSet=true;
        if (!targetReached && tTargetDistM<targetRadiusM) {
            targetReached=true;
            targetBeepEnd=millis()+5000;
            Serial.printf("TARGET REACHED! dist=%.1fm\n",tTargetDistM);
        }
    }

    digitalWrite(LED_GPS,    tFix  ? HIGH : LOW);
    digitalWrite(LED_STATIC, tStat ? HIGH : LOW);
    float posDrift=sqrtf(sq(ekf_x[0]-lastPosN)+sq(ekf_x[1]-lastPosE));
    if (tStat && posDrift>0.3f) { digitalWrite(LED_DRIFT,HIGH); }
    else if (!tStat) { digitalWrite(LED_DRIFT,LOW); lastPosN=ekf_x[0]; lastPosE=ekf_x[1]; }

    if (millis()-lastDebugMs>5000) {
        lastDebugMs=millis();
        Serial.printf("POS:%.3f,%.3f,%.1f VEL:%.3f,%.3f,%.3f ST:%d GPS:%d YAW:%.1f\n",
            ekf_x[0],ekf_x[1],ekf_x[2],ekf_x[3],ekf_x[4],ekf_x[5],
            (int)tStat,(int)tFix,tYaw);
    }
    if (millis()-lastDriftLog>60000) {
        lastDriftLog=millis();
        Serial.printf("DRIFT,%lu,%.3f,%.3f,%.3f,%d,%d\n",
            (unsigned long)(millis()-insStartMs),
            ekf_x[0],ekf_x[1],ekf_x[2],(int)tStat,(int)tFix);
    }

    sendNMEA();
}

void runCalibration() {
    Serial.println("--- 60s Cal. FLAT SURFACE. DO NOT VIBRATE ---");
    calDone=false; calProg=0;
    float sumAx=0,sumAy=0,sumAz=0,sumGx=0,sumGy=0;
    int ok=0;
    for (int i=0; i<CAL_SAMPLES; i++) {
        esp_task_wdt_reset();
        server.handleClient();
        while (SerialGPS.available()) gps.encode(SerialGPS.read());
        if (!gpsConnected && gps.charsProcessed()>20) gpsConnected=true;
        float cx2,cy2,cz2,gx2,gy2,gz2;
        if (mpuRead(cx2,cy2,cz2,gx2,gy2,gz2)) {
            sumAx+=cx2; sumAy+=cy2; sumAz+=cz2;
            sumGx+=gx2; sumGy+=gy2;
            calBufGz[ok<CAL_SAMPLES?ok:CAL_SAMPLES-1]=gz2;
            ok++;
        }
        calProg=i+1;
        delay(20);
        if (i%600==0) Serial.printf("  %d%% heap=%u\n",i*100/CAL_SAMPLES,ESP.getFreeHeap());
    }
    ok=max(ok,1);
    abx=sumAx/ok; aby=sumAy/ok; abz=sumAz/ok-9.80665f;
    gbx=sumGx/ok; gby=sumGy/ok;
    int n=min(ok,CAL_SAMPLES);
    insertionSort(calBufGz,n);
    gbz=calBufGz[n/2];
    calDone=true;
    Serial.printf("  Abias:%.4f %.4f %.4f\n",abx,aby,abz);
    Serial.printf("  Gbias: mean_gx=%.5f mean_gy=%.5f MEDIAN_gz=%.5f\n",gbx,gby,gbz);
    float nx2=abx,ny2=aby,nz2=abz+9.80665f;
    float am=sqrtf(nx2*nx2+ny2*ny2+nz2*nz2);
    if (am>0.5f) {
        cf_roll =atan2f(ny2,nz2)*RAD_TO_DEG;
        cf_pitch=atan2f(-nx2,sqrtf(ny2*ny2+nz2*nz2))*RAD_TO_DEG;
    }
    Serial.printf("  CF init roll=%.2f pitch=%.2f\n",cf_roll,cf_pitch);
    Serial.println("--- Cal DONE ---");
}

void handleStart() {
    if (!server.hasArg("lat")||!server.hasArg("lon")||!server.hasArg("alt")) {
        server.send(400,"text/plain","Missing args"); return;
    }
    phoneLat=server.arg("lat").toDouble();
    phoneLon=server.arg("lon").toDouble();
    phoneAlt=server.arg("alt").toFloat();

    if (server.hasArg("tgLat")&&server.hasArg("tgLon")) {
        targetLat=server.arg("tgLat").toDouble();
        targetLon=server.arg("tgLon").toDouble();
        targetRadiusM=server.hasArg("tgRad")?server.arg("tgRad").toFloat():10.0f;
        targetSet=true; targetReached=false;
        Serial.printf("Target: %.7f,%.7f r=%.1fm\n",targetLat,targetLon,targetRadiusM);
    }

    float bs=0;
    for (int i=0;i<20;i++){bs+=bmp.readAltitude(1013.25f);delay(50);}
    float rb=bs/20.0f;
    baroOff=phoneAlt-rb; startBaro=phoneAlt;
    Serial.printf("Baro: raw=%.1f phone=%.1f off=%.2f\n",rb,phoneAlt,baroOff);

    ekf_init();
    memset(zbuf,0,sizeof(zbuf)); zidx=0; zfull=false;
    mpuHasPrev=false;
    sm_ax=sm_ay=sm_az=sm_gx=sm_gy=sm_gz=0;
    gpsCal=false; gpsJammed=false; gpsWarmup=0;
    tGpsErr=0; tFix=false;
    insStartMs=millis(); lastGpsMs=millis(); lastUs=micros();
    running=true;
    server.send(200,"text/plain","OK");
    Serial.println("INS ENGAGED");
}

void handleData() {
    char b[640];
    snprintf(b,sizeof(b),
        "{\"la\":%.7f,\"lo\":%.7f,\"al\":%.2f,\"ba\":%.2f,"
        "\"pn\":%.4f,\"pe\":%.4f,"
        "\"vn\":%.4f,\"ve\":%.4f,\"vd\":%.4f,"
        "\"ro\":%.2f,\"pi\":%.2f,\"ya\":%.2f,\"hd\":%.1f,"
        "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
        "\"ge\":%.2f,\"fx\":%s,\"jam\":%s,\"gc\":%s,"
        "\"tc\":%.2f,\"pp\":%.1f,\"st\":%s,"
        "\"gs\":%d,\"gh\":%.1f,\"gsk\":%.1f,\"gco\":%.1f,"
        "\"gut\":%lu,\"glat\":%.7f,\"glon\":%.7f,\"gconn\":%s,"
        "\"tset\":%s,\"tdist\":%.2f,\"treach\":%s}",
        tLat,tLon,(double)tAlt,(double)tBaroAlt,
        (double)ekf_x[0],(double)ekf_x[1],
        (double)tVn,(double)tVe,(double)tVd,
        (double)tRoll,(double)tPitch,(double)tYaw,(double)tHdg,
        (double)tGyrX,(double)tGyrY,(double)tGyrZ,(double)tGpsErr,
        tFix?"true":"false",tJam?"true":"false",tGpsCal?"true":"false",
        (double)tTempC,(double)tPressPa,tStat?"true":"false",
        gSats,(double)gHdop,(double)gSpeedKmh,(double)gCourseDeg,
        (unsigned long)gUtcTime,(double)rawGpsLat,(double)rawGpsLon,
        gpsConnected?"true":"false",
        tTargetSet?"true":"false",(double)tTargetDistM,targetReached?"true":"false");
    server.send(200,"application/json",b);
}

void predictStep(float dt) {
    float ax,ay,az,gx,gy,gz;
    if (!mpuRead(ax,ay,az,gx,gy,gz)) return;
    rawAx=ax; rawAy=ay; rawAz=az;
    rawGx=gx; rawGy=gy; rawGz=gz;
    rawBaro=bmp.readAltitude(1013.25f);

    float am=sqrtf(ax*ax+ay*ay+az*az);
    if (am>7.0f && am<13.0f) {
        float aR=atan2f(ay,az)*RAD_TO_DEG;
        float aP=atan2f(-ax,sqrtf(ay*ay+az*az))*RAD_TO_DEG;
        cf_roll =CF_ALPHA*(cf_roll +(gx-gbx)*dt*RAD_TO_DEG)+(1.0f-CF_ALPHA)*aR;
        cf_pitch=CF_ALPHA*(cf_pitch+(gy-gby)*dt*RAD_TO_DEG)+(1.0f-CF_ALPHA)*aP;
    } else {
        cf_roll +=(gx-gbx)*dt*RAD_TO_DEG;
        cf_pitch+=(gy-gby)*dt*RAD_TO_DEG;
    }

    float axC=ax-abx-ekf_x[9],  ayC=ay-aby-ekf_x[10], azC=az-abz-ekf_x[11];
    float gxC=gx-gbx-ekf_x[12], gyC=gy-gby-ekf_x[13], gzC=gz-gbz-ekf_x[14];

    float r=ekf_x[6],p=ekf_x[7],y2=ekf_x[8];
    float cr=cosf(r),sr=sinf(r),cp=cosf(p),sp=sinf(p);
    float tp=(fabsf(cp)>0.05f)?sp/cp:0.0f;
    float cpS=(fabsf(cp)>0.05f)?cp:0.05f;
    float cy=cosf(y2),sy=sinf(y2);

    ekf_x[6]+=(gxC+(gyC*sr+gzC*cr)*tp)*dt;
    ekf_x[7]+=(gyC*cr-gzC*sr)*dt;
    ekf_x[8]+=(gyC*sr+gzC*cr)/cpS*dt;

    r=ekf_x[6];p=ekf_x[7];y2=ekf_x[8];
    cr=cosf(r);sr=sinf(r);cp=cosf(p);sp=sinf(p);cy=cosf(y2);sy=sinf(y2);

    float aN=(cy*cp)*axC+(cy*sp*sr-sy*cr)*ayC+(cy*sp*cr+sy*sr)*azC;
    float aE=(sy*cp)*axC+(sy*sp*sr+cy*cr)*ayC+(sy*sp*cr-cy*sr)*azC;
    float aD=(-sp)*axC+(cp*sr)*ayC+(cp*cr)*azC-9.80665f;

    ekf_x[3]+=aN*dt; ekf_x[4]+=aE*dt; ekf_x[5]+=aD*dt;
    ekf_x[0]+=ekf_x[3]*dt; ekf_x[1]+=ekf_x[4]*dt; ekf_x[2]+=ekf_x[5]*dt;

    for(int i=0;i<NS;i++) for(int j=0;j<NS;j++) sc_F[i][j]=(i==j)?1.0f:0.0f;
    sc_F[0][3]=dt; sc_F[1][4]=dt; sc_F[2][5]=dt;
    sc_F[3][7]=aD*dt; sc_F[4][6]=-aD*dt; sc_F[5][6]=aE*dt; sc_F[5][7]=-aN*dt;
    sc_F[3][9]=-dt; sc_F[4][10]=-dt; sc_F[5][11]=-dt;
    sc_F[6][12]=-dt; sc_F[7][13]=-dt; sc_F[8][14]=-dt;
    propagate_P(sc_F);
}

void updateBaro() {
    float ba=rawBaro+baroOff;
    float H[NS]; memset(H,0,sizeof(H)); H[2]=1;
    float z[1]={-(ba-startBaro)}, R[1]={0.25f};
    ekf_update(1,H,z,R);
}

void updateGPS(double cLat, double cLon) {
    float nM=(float)((cLat-phoneLat)*111111.0);
    float eM=(float)((cLon-phoneLon)*111111.0*cos(phoneLat*DEG_TO_RAD));
    float H[2*NS]; memset(H,0,sizeof(H));
    H[0*NS+0]=1; H[1*NS+1]=1;
    float z[2]={nM,eM};
    float hd=(rawGpsHdop>0.1f&&rawGpsHdop<10.0f)?rawGpsHdop:3.0f;
    float Rv=fmaxf(2.5f,2.5f*hd*hd), R[2]={Rv,Rv};
    ekf_update(2,H,z,R);
    ekf_P[0][0]=fminf(ekf_P[0][0],Rv*1.5f);
    ekf_P[1][1]=fminf(ekf_P[1][1],Rv*1.5f);
}

void sendNMEA() {
    static unsigned long last=0;
    if (millis()-last<200) return; last=millis();
    double lat=tLat,lon=tLon; float alt=tAlt;
    char ld=(lat>=0)?'N':'S', lod2=(lon>=0)?'E':'W';
    double aLa=fabs(lat),aLo=fabs(lon);
    int lad=(int)aLa; double lam=(aLa-lad)*60.0;
    int lod3=(int)aLo; double lom=(aLo-lod3)*60.0;
    float spd=sqrtf(tVn*tVn+tVe*tVe)*1.94384f;
    float cog=fmodf(tHdg+360.0f,360.0f);
    int fix=tJam?6:(tFix?1:6);
    char body[160];
    snprintf(body,sizeof(body),
        "GPGGA,120000.00,%02d%09.6f,%c,%03d%09.6f,%c,%d,12,0.8,%.1f,M,0.0,M,,",
        lad,lam,ld,lod3,lom,lod2,fix,(double)alt);
    SerialPix.printf("$%s*%02X\r\n",body,nmea_ck(body));
    snprintf(body,sizeof(body),
        "GPRMC,120000.00,A,%02d%09.6f,%c,%03d%09.6f,%c,%.2f,%.2f,010101,,,A",
        lad,lam,ld,lod3,lom,lod2,(double)spd,(double)cog);
    SerialPix.printf("$%s*%02X\r\n",body,nmea_ck(body));
}