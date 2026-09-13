#include <Arduino.h>
#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(20);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// --- Reducteur ---
const float GEAR_RATIO = 20.0;   // 20 tours moteur = 1 tour bras

// --- Protections ---
float STALL_CURRENT = 2.5;
float STALL_TIME = 2.0;
bool fault = false;
unsigned long stall_start = 0;
bool stall_active = false;

// --- Recalage ---
// position_bras = (angle_moteur / GEAR_RATIO) + offset_bras
float offset_bras = 0;           // recale par la commande Z
float target_bras = 0;           // consigne en angle bras (rad)

// Buffer reception commandes
char cmdBuf[32];
int cmdIdx = 0;

// Convertit une consigne bras -> consigne moteur pour SimpleFOC
float brasToMoteur(float angle_bras) {
    return (angle_bras - offset_bras) * GEAR_RATIO;
}
// Position bras actuelle (lue depuis le moteur)
float getPositionBras() {
    return (motor.shaft_angle / GEAR_RATIO) + offset_bras;
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Wire.setSDA(PB7);
    Wire.setSCL(PB8);
    Wire.begin();
    Wire.setClock(400000);
    sensor.init(&Wire);

    driver.voltage_power_supply = 24;
    driver.voltage_limit = 24;
    driver.init();

    motor.linkDriver(&driver);
    motor.linkSensor(&sensor);
    currentSense.linkDriver(&driver);

    motor.voltage_limit = 6;
    motor.current_limit = 3.0;
    motor.velocity_limit = 15;
    motor.controller = MotionControlType::angle;

    motor.PID_velocity.P = 0.3;
    motor.PID_velocity.I = 5;
    motor.LPF_velocity.Tf = 0.02;
    motor.P_angle.P = 8;
    motor.P_angle.output_ramp = 100;

    motor.init();
    currentSense.init();
    motor.linkCurrentSense(&currentSense);
    motor.initFOC();

    // Au demarrage : consigne = position actuelle (pas de saut)
    target_bras = getPositionBras();

    Serial.println("READY");
    // Protocole :
    //  P<angle>  -> va a la position bras <angle> (rad)
    //  Z<angle>  -> recale : "tu es actuellement a <angle>" (pas de mouvement)
    //  S         -> stop (disable)
    //  E         -> enable
    //  R         -> reset defaut
    //  ?         -> renvoie position actuelle
}

void traiterCommande(char* cmd) {
    char type = cmd[0];
    float val = atof(cmd + 1);

    switch (type) {
        case 'P':   // consigne position bras
            target_bras = val;
            break;

        case 'Z': {  // recalage sans mouvement
            // On veut que getPositionBras() == val maintenant
            // val = shaft_angle/RATIO + offset  ->  offset = val - shaft_angle/RATIO
            offset_bras = val - (motor.shaft_angle / GEAR_RATIO);
            target_bras = val;   // la consigne suit, pour ne pas bouger
            Serial.print("RECALE ");
            Serial.println(val, 4);
            break;
        }

        case 'S':
            motor.disable();
            Serial.println("DISABLED");
            break;

        case 'E':
            motor.enable();
            target_bras = getPositionBras();  // repart de la position actuelle
            Serial.println("ENABLED");
            break;

        case 'R':
            fault = false;
            stall_active = false;
            motor.enable();
            target_bras = getPositionBras();
            Serial.println("RESET");
            break;

        case '?':
            Serial.print("POS ");
            Serial.println(getPositionBras(), 4);
            break;
    }
}

unsigned long last_status = 0;

void loop() {
    motor.loopFOC();

    if (fault) {
        motor.setPhaseVoltage(0, 0, 0);
    } else {
        motor.move(brasToMoteur(target_bras));
    }

    // --- Watchdog stall ---
    float I = currentSense.getDCCurrent();
    if (!fault) {
        if (fabs(I) > STALL_CURRENT) {
            if (!stall_active) { stall_active = true; stall_start = millis(); }
            else if ((millis() - stall_start) > (STALL_TIME * 1000)) {
                fault = true;
                motor.disable();
                Serial.println("FAULT STALL");
            }
        } else {
            stall_active = false;
        }
    }

    // --- Reception commandes serie (non-bloquant) ---
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdIdx > 0) {
                cmdBuf[cmdIdx] = '\0';
                traiterCommande(cmdBuf);
                cmdIdx = 0;
            }
        } else if (cmdIdx < 31) {
            cmdBuf[cmdIdx++] = c;
        }
    }

    // --- Status periodique vers le Pi (50Hz) ---
    if (millis() - last_status > 20) {
        last_status = millis();
        Serial.print("S ");
        Serial.print(getPositionBras(), 4);   // position bras (rad)
        Serial.print(" ");
        Serial.print(I, 2);                    // courant
        Serial.print(" ");
        Serial.println(fault ? 1 : 0);         // flag defaut
    }
}