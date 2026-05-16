#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

#define PIN_SDA 8
#define PIN_SCL 9

#define SERVO_DEFAULT_MIN_US    500
#define SERVO_DEFAULT_MAX_US   2500
#define SERVO_DEFAULT_MIN_DEG    0.0f
#define SERVO_DEFAULT_MAX_DEG  270.0f
#define DEFAULT_SERVO_REST_ANGLE 135
#define UNSET_ANGLE 271

#define PCA9685_ADDR 0x40
#define PCA9685_FREQ_HZ 50
#define PCA9685_OSC_FREQ 25000000UL  // 25 MHz internal oscillator

Adafruit_PWMServoDriver pca(PCA9685_ADDR);
uint16_t g_servoAngles[16];
float g_servoZero[16];
int8_t g_servoSign[16];  // +1 or -1
Preferences prefs;

bool parseFloat(char* tok, float& out) {
    if (!tok) return false;
    char* end = nullptr;
    float v = strtof(tok, &end);
    if (end == tok) return false;
    out = v;
    return true;
}

bool parseInt(char* tok, long& out) {
    if (!tok) return false;
    char* end = nullptr;
    long v = strtol(tok, &end, 10);
    if (end == tok) return false;
    out = v;
    return true;
}

template <typename T>
constexpr const T clamp(const T x, const T min, const T max) {
    return x > max ? max : x < min ? min : x;
}

inline void getServoRestAngleKey(char* key, uint8_t channel) {
    snprintf(key, 64, "servo-rest-%d", channel);
}

bool getSavedRestServoAngle(uint8_t channel, uint16_t& angle_out) {
    char key[64];
    getServoRestAngleKey(key, channel);
    if (!prefs.isKey(key)) return false;
    angle_out = prefs.getUInt(key);
    return true;
}

void saveRestServoAngle(uint8_t channel, uint16_t angle) {
    char key[64];
    getServoRestAngleKey(key, channel);
    prefs.putUInt(key, angle);
}

inline void getServoZeroKey(char* key, uint8_t channel) {
    snprintf(key, 64, "servo-zero-%d", channel);
}

bool getSavedServoZero(uint8_t channel, float& out) {
    char key[64];
    getServoZeroKey(key, channel);
    if (!prefs.isKey(key)) return false;
    out = prefs.getFloat(key);
    return true;
}

void saveServoZero(uint8_t channel, float angle) {
    char key[64];
    getServoZeroKey(key, channel);
    prefs.putFloat(key, angle);
}

inline void getServoSignKey(char* key, uint8_t channel) {
    snprintf(key, 64, "servo-sign-%d", channel);
}

bool getSavedServoSign(uint8_t channel, int8_t& out) {
    char key[64];
    getServoSignKey(key, channel);
    if (!prefs.isKey(key)) return false;
    out = (int8_t)prefs.getChar(key);
    return true;
}

void saveServoSign(uint8_t channel, int8_t sign) {
    char key[64];
    getServoSignKey(key, channel);
    prefs.putChar(key, (int8_t)sign);
}

void driveServo(uint8_t channel, uint16_t angle) {
    angle = clamp<uint16_t>(angle, 0, 270);
    uint16_t pulse = map(angle, 0, 270, SERVO_DEFAULT_MIN_US, SERVO_DEFAULT_MAX_US);
    pca.writeMicroseconds(channel, pulse);
    g_servoAngles[channel] = angle;
}

#define SERIAL_BUF_LEN 96

char g_buf[SERIAL_BUF_LEN];
uint16_t g_len = 0;

// Strip leading whitespace and return pointer to next token, plus advance *p
// past it. Returns nullptr when no token remains.
char* nextToken(char** p) {
    if (*p == nullptr) return nullptr;
    while (**p && isspace((unsigned char)**p)) (*p)++;
    if (**p == '\0') return nullptr;
    char* start = *p;
    while (**p && !isspace((unsigned char)**p)) (*p)++;
    if (**p) { **p = '\0'; (*p)++; }
    return start;
}

void handleServo(char* rest) {
    long ch;
    float angle;
    char* t1 = nextToken(&rest);
    char* t2 = nextToken(&rest);
    if (!parseInt(t1, ch) || !parseFloat(t2, angle)) {
        Serial.println(F("ERR usage: SERVO ch angle"));
        return;
    }
    driveServo((uint8_t)ch, (uint16_t)angle);
    Serial.printf("OK SERVO %ld %.2f\n", ch, angle);
}

void handleServoPos(char* rest) {
    long ch;
    char* t1 = nextToken(&rest);
    if (!parseInt(t1, ch)) {
        Serial.println(F("ERR usage: SERVOPOS ch"));
        return;
    }
    if (!(0 <= ch && ch <= 15)) {
        Serial.println(F("ERR usage: expected 0 <= `ch` <= 15"));
        return;
    }
    Serial.println(g_servoAngles[ch]);
    Serial.printf("OK SERVOPOS %ld\n", ch);
}

void handleServoSave(char* rest) {
    long ch;
    char* t1 = nextToken(&rest);
    if (!parseInt(t1, ch)) {
        Serial.println(F("ERR usage: SERVOSAVE ch"));
        return;
    }
    if (!(0 <= ch && ch <= 15)) {
        Serial.println(F("ERR expected 0 <= `ch` <= 15"));
        return;
    }
    uint16_t angle = g_servoAngles[ch];
    if (angle == UNSET_ANGLE) {
        Serial.printf("ERR SERVOSAVE %ld: servo has not been driven yet\n", ch);
        return;
    }
    saveRestServoAngle((uint8_t)ch, angle);
    Serial.printf("OK SERVOSAVE %ld %u\n", ch, angle);
}

void handleZero(char* rest) {
    long ch;
    float angle;
    char* t1 = nextToken(&rest);
    char* t2 = nextToken(&rest);
    if (!parseInt(t1, ch) || !parseFloat(t2, angle)) {
        Serial.println(F("ERR usage: ZERO ch angle"));
        return;
    }
    if (!(0 <= ch && ch <= 15)) {
        Serial.println(F("ERR expected 0 <= ch <= 15"));
        return;
    }
    g_servoZero[ch] = angle;
    Serial.printf("OK ZERO %ld %.2f\n", ch, angle);
}

void handleZeroGet(char* rest) {
    long ch;
    char* t1 = nextToken(&rest);
    if (!parseInt(t1, ch)) {
        Serial.println(F("ERR usage: ZEROGET ch"));
        return;
    }
    if (!(0 <= ch && ch <= 15)) {
        Serial.println(F("ERR expected 0 <= ch <= 15"));
        return;
    }
    Serial.printf("%.2f\n", g_servoZero[ch]);
    Serial.printf("OK ZEROGET %ld\n", ch);
}

void handleZeroSave(char* rest) {
    long ch;
    char* t1 = nextToken(&rest);
    if (!parseInt(t1, ch)) {
        Serial.println(F("ERR usage: ZEROSAVE ch"));
        return;
    }
    if (!(0 <= ch && ch <= 15)) {
        Serial.println(F("ERR expected 0 <= ch <= 15"));
        return;
    }
    float angle = g_servoZero[ch];
    saveServoZero((uint8_t)ch, angle);
    Serial.printf("OK ZEROSAVE %ld %.2f\n", ch, angle);
}

void handleRestGet(char* rest) {
    long ch;
    char* t1 = nextToken(&rest);
    if (!parseInt(t1, ch)) {
        Serial.println(F("ERR usage: RESTGET ch"));
        return;
    }
    if (!(0 <= ch && ch <= 15)) {
        Serial.println(F("ERR expected 0 <= ch <= 15"));
        return;
    }
    uint16_t angle;
    if (getSavedRestServoAngle((uint8_t)ch, angle)) {
        Serial.println(angle);
    } else {
        Serial.println(F("NONE"));
    }
    Serial.printf("OK RESTGET %ld\n", ch);
}

void handleSign(char* rest) {
    long ch, val;
    char* t1 = nextToken(&rest);
    char* t2 = nextToken(&rest);
    if (!parseInt(t1, ch) || !parseInt(t2, val)) {
        Serial.println(F("ERR usage: SIGN ch (1 or -1)"));
        return;
    }
    if (!(0 <= ch && ch <= 15)) {
        Serial.println(F("ERR expected 0 <= ch <= 15"));
        return;
    }
    g_servoSign[ch] = (val < 0) ? -1 : 1;
    Serial.printf("OK SIGN %ld %d\n", ch, g_servoSign[ch]);
}

void handleSignGet(char* rest) {
    long ch;
    char* t1 = nextToken(&rest);
    if (!parseInt(t1, ch)) {
        Serial.println(F("ERR usage: SIGNGET ch"));
        return;
    }
    if (!(0 <= ch && ch <= 15)) {
        Serial.println(F("ERR expected 0 <= ch <= 15"));
        return;
    }
    Serial.printf("%d\n", g_servoSign[ch]);
    Serial.printf("OK SIGNGET %ld\n", ch);
}

void handleSignSave(char* rest) {
    long ch;
    char* t1 = nextToken(&rest);
    if (!parseInt(t1, ch)) {
        Serial.println(F("ERR usage: SIGNSAVE ch"));
        return;
    }
    if (!(0 <= ch && ch <= 15)) {
        Serial.println(F("ERR expected 0 <= ch <= 15"));
        return;
    }
    saveServoSign((uint8_t)ch, g_servoSign[ch]);
    Serial.printf("OK SIGNSAVE %ld %d\n", ch, g_servoSign[ch]);
}

void dispatch(char* line) {
    while (*line && isspace((unsigned char)*line)) line++;
    if (*line == '\0' || *line == '#') return;

    char* cmd = nextToken(&line);
    if (!cmd) return;

    for (char* p = cmd; *p; ++p) *p = (char)toupper((unsigned char)*p);

    if (!strcmp(cmd, "SERVO"))           handleServo(line);
    else if (!strcmp(cmd, "SERVOPOS"))  handleServoPos(line);
    else if (!strcmp(cmd, "SERVOSAVE")) handleServoSave(line);
    else if (!strcmp(cmd, "ZERO"))      handleZero(line);
    else if (!strcmp(cmd, "ZEROGET"))   handleZeroGet(line);
    else if (!strcmp(cmd, "ZEROSAVE"))  handleZeroSave(line);
    else if (!strcmp(cmd, "RESTGET"))   handleRestGet(line);
    else if (!strcmp(cmd, "SIGN"))      handleSign(line);
    else if (!strcmp(cmd, "SIGNGET"))   handleSignGet(line);
    else if (!strcmp(cmd, "SIGNSAVE"))  handleSignSave(line);
    else {
        Serial.print(F("ERR unknown command: "));
        Serial.println(cmd);
    }
}

void poll() {
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c < 0) break;

        if (c == '\r') continue;
        if (c == '\n') {
            g_buf[g_len] = '\0';
            if (g_len > 0) dispatch(g_buf);
            g_len = 0;
            continue;
        }
        if (g_len < (SERIAL_BUF_LEN - 1)) {
            g_buf[g_len++] = (char)c;
        } else {
            g_len = 0;
            Serial.println(F("ERR line too long"));
        }
    }
}

void initServoPositions() {
    for (uint8_t channel = 0; channel <= 15; channel++) {
        g_servoAngles[channel] = UNSET_ANGLE;

        // Load zero offset from NVS, default to 135.0
        float zero;
        if (getSavedServoZero(channel, zero)) {
            g_servoZero[channel] = zero;
        } else {
            g_servoZero[channel] = 135.0f;
        }

        // Load sign from NVS, default to +1
        int8_t sign;
        if (getSavedServoSign(channel, sign)) {
            g_servoSign[channel] = sign;
        } else {
            g_servoSign[channel] = 1;
        }

        uint16_t angle;
        if (getSavedRestServoAngle(channel, angle)) {
            driveServo(channel, angle);
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting!");

    prefs.begin("robodawg");

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    if (!pca.begin()) {
        Serial.println("[servo] ERROR: PCA9685 not found on I2C bus");
    }
    pca.setOscillatorFrequency(PCA9685_OSC_FREQ);
    pca.setPWMFreq(PCA9685_FREQ_HZ);

    initServoPositions();
}

void loop() {
    poll();
    delayMicroseconds(1000);
}
