 #include <FalconRobot.h>

FalconRobotLineSensor left(A3);
FalconRobotLineSensor middle_l(A2);
FalconRobotLineSensor middle_r(A1);
FalconRobotLineSensor right(A0);

FalconRobotMotors motors(5, 7, 6, 8);

int leftValue;
int middle_lValue;
int middle_rValue;
int rightValue;

int digitalLeft;
int digitalMiddleL;
int digitalMiddleR;
int digitalRight;

#define LINETHRESHOLD 750
#define SPEED 55//Ajustar Velocidade
#define AJUSTE 8 //Ajustar ajuste

#define desvioDesejado 0

int leftSpeed;
int rightSpeed;

void leituraSensores() {
    leftValue = left.read();
    middle_lValue = middle_l.read();
    middle_rValue = middle_r.read();
    rightValue = right.read();
}

void Bsensorsvalue() {
    digitalLeft = (leftValue < LINETHRESHOLD) ? 1 : 0;
    digitalMiddleL = (middle_lValue < LINETHRESHOLD) ? 1 : 0;
    digitalMiddleR = (middle_rValue < LINETHRESHOLD) ? 1 : 0;
    digitalRight = (rightValue < LINETHRESHOLD) ? 1 : 0;
    Serial.print("Valores_digitais_sensores");
    Serial.print(digitalLeft);
    Serial.print(digitalMiddleL);
    Serial.print(digitalMiddleR);
    Serial.print(digitalRight);
    Serial.print("\n");
}

int calcularDesvioAtual() {
    int WeightedSum = (-4 * digitalLeft) + (-2 * digitalMiddleL) + (2 * digitalMiddleR) + (4 * digitalRight);
    int NsensoresL0 = 4 - (digitalLeft + digitalMiddleL + digitalMiddleR + digitalRight); // Número de sensores lendo zero
    if (NsensoresL0 == 0) { // evita a divisão por zero
        NsensoresL0 = 1;
    }
    int desvioAtual = WeightedSum / NsensoresL0;

    Serial.print("Soma ponderada, n sensores 0, desvioAtual");
    Serial.println(WeightedSum);
    Serial.println(NsensoresL0);
    Serial.println(desvioAtual);
    return desvioAtual;
}

void loop() {
    leituraSensores();
    Bsensorsvalue();
    int desvioAtual = calcularDesvioAtual();

    int erro =  desvioDesejado - desvioAtual;
    Serial.print("erro:");
    Serial.println(erro);

    if (erro == 0) { // seguir reto
      leftSpeed = SPEED;
      rightSpeed = SPEED;
    } else if (erro > -4 && erro < -1) { // curva esquerda inicial
      leftSpeed = AJUSTE *2;
      rightSpeed = SPEED;
    } else if (erro == -4) { // curva esquerda agravado
      leftSpeed = 0;
      rightSpeed = SPEED;
    } else if (erro < 4 && erro > 1) { // curva esquerda inicial
      leftSpeed = SPEED;
      rightSpeed = AJUSTE*2;     
    } else if (erro == 4) { // curva esquerda agravado
      leftSpeed = SPEED;
      rightSpeed = 0;
    } else {                //Cenários não mapeados
      leftSpeed = 0;
      rightSpeed = 0;
    }

    motors.leftDrive(leftSpeed, BACKWARD);
    motors.rightDrive(rightSpeed, FORWARD);

    delay(50);
}

void setup() {
    Serial.begin(9600);
}
