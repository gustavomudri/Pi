#include <FalconRobot.h>

//Setup sensores IR
FalconRobotLineSensor left(A3);
FalconRobotLineSensor middle_l(A2);
FalconRobotLineSensor middle_r(A1);
FalconRobotLineSensor right(A0);

int leftValue;
int middle_lValue;
int middle_rValue;
int rightValue;

int digitalLeft;
int digitalMiddleL;
int digitalMiddleR;
int digitalRight;

#define LINETHRESHOLD 850
// Fim setup sensore IR 

// Setup motores
FalconRobotMotors motors(5, 7, 6, 8);

#define base_speed 20 // Ajustar Velocidade
#define ajuste 10 // Ajustar ajuste

float vme_atual;
float vmd_atual;
float vme_desejada;
float vmd_desejada;

int leftSpeed;
int rightSpeed;

// Saturação e mapeamento para controlador dos motores
int v_max = 160;
int v_min = 0;
int um_min = 0;
int um_max = 100;
 
// Fim setup Motores 

#define WINDOW_SIZE 1  // Tamanho da janela para a média móvel


float ume_history[WINDOW_SIZE];  // Array para armazenar os valores de controle recentes
int ume_index = 0;  // Índice para inserir o próximo valor de controle

float umd_history[WINDOW_SIZE];  // Array para armazenar os valores de controle recentes
int umd_index = 0;  // Índice para inserir o próximo valor de controle

// Var. Desvio 
#define desvioDesejado 0

// Fim setup desvio Atual

// Declaração das variáveis auxiliares para o cálculo da velocidade
volatile unsigned long contador_L = 0;
volatile unsigned long contador_R = 0;
const int N_pulsos_rot = 20; 

// Declaração das variáveis auxiliares para a temporização
const long interval = 50; // Intervalo de tempo em milissegundos para a medição
unsigned long previousMillis = 0;
unsigned long startMillis = 0;

//declaração de var. para controlador PID 
int erro_ant1 = 0;
int erro_ant2 = 0;
int erro_ant3 = 0;
int erro_ant4 = 0;

// Buffer para média móvel
const int bufferSize = 10;
float speedLBuffer[bufferSize] = {0};
float speedRBuffer[bufferSize] = {0};
int bufferIndex = 0;

//Parâmetro controlador MD
float umd_ant = 0;
float erromd_ant = 0;

//parâmetros controlador ME
float ume_ant = 0;
float errome_ant=0;

#define umax 100
#define umin -100

// leitura dos sensores
void leituraSensores() {
    leftValue = left.read();
    middle_lValue = middle_l.read();
    middle_rValue = middle_r.read();
    rightValue = right.read();
}

// Calculo dos valores binários dos sensores 
void Bsensorsvalue() {
    digitalLeft = (leftValue < LINETHRESHOLD) ? 1 : 0;
    digitalMiddleL = (middle_lValue < LINETHRESHOLD) ? 1 : 0;
    digitalMiddleR = (middle_rValue < LINETHRESHOLD) ? 1 : 0;
    digitalRight = (rightValue < LINETHRESHOLD) ? 1 : 0;
  
}

// Calculo do desvio atual 
int calcularErroAtual() {
    int WeightedSum = (-6 * digitalLeft) + (-2 * digitalMiddleL) + (2 * digitalMiddleR) + (6 * digitalRight);
    int NsensoresL0 = 4 - (digitalLeft + digitalMiddleL + digitalMiddleR + digitalRight); // Número de sensores lendo zero
    if (NsensoresL0 == 0) { // evita a divisão por zero
        NsensoresL0 = 1;
    }
    int desvioAtual = WeightedSum / NsensoresL0;

    int erro = - desvioAtual;
    return erro;
}

// Função de interrupção para contar os pulsos do encoder esquerdo
void contador_pulso_L() {
    contador_L++;
}

// Função de interrupção para contar os pulsos do encoder direito
void contador_pulso_R() {
    contador_R++;
}

// Função para calcular a média móvel
float calcularMediaMovel(float buffer[], int size) {
    float soma = 0;
    for (int i = 0; i < size; i++) {
        soma += buffer[i];
    }
    return soma / size;
}

// Função para calcular a velocidade das rodas
void calcularVelocidade() {
    vme_atual = (contador_L / (float)N_pulsos_rot) * (60000.0 / interval);
    vmd_atual = (contador_R / (float)N_pulsos_rot) * (60000.0 / interval);

    contador_L = 0;
    contador_R = 0;

    previousMillis = millis();
}

void controladorMD(float vmd_desejada){
  
  int erromd_atual = vmd_desejada - vmd_atual;
  float umd = 1.762*erromd_atual - 1.338*erromd_ant + umd_ant;
 
  //Saturação motor direito
  umd = constrain(umd, v_min, v_max);

  float map(float umd, float v_min, float v_max, float um_min, float um_max);

    motors.rightDrive(umd, FORWARD);
    // Atualização das variáveis 
    umd_ant = umd;
    erromd_ant = erromd_atual;
} 

// Controlador Motor Esquerdo 
void controladorME(int vme_desejada){

  int errome_atual = vme_desejada - vme_atual;
  float ume = 1.818* errome_atual- 1.382*errome_ant + ume_ant;

  //Saturação motor direito
  ume = constrain(ume, v_min, v_max);
  
 //Mapeamento para saída do motor: 

  float map(float ume, float v_min, float v_max, float um_min, float um_max);

  motors.leftDrive(ume, BACKWARD);

  // Atualização das variáveis 
  ume_ant = ume;
  errome_ant = errome_atual;
}


void PID() {
  int i;
  float Kp = 50, Ki = 0, Kd = 0;
  float erro = calcularErroAtual();

  int proportional = erro * Kp;
  int integral = Ki * (erro + erro_ant1 + erro_ant2); 
  int derivative = Kd * (erro - erro_ant1);

  int udesv = proportional + integral + derivative; 

  if (udesv > 0){
    vme_desejada =  udesv;
    vmd_desejada =  base_speed - udesv;
  } else if (udesv < 0){
    vme_desejada = udesv;
    vmd_desejada = base_speed - udesv;
  } else if (udesv == 0) {
    vme_desejada = base_speed + ajuste;
    vmd_desejada = base_speed + ajuste; 
  }

  vme_desejada = constrain(vme_desejada, v_min, v_max);
  vmd_desejada = constrain(vmd_desejada, v_min, v_max);
  
  controladorME(vme_desejada);
  controladorMD(vmd_desejada);

  // Atualização das variáveis 
  erro_ant4 = erro_ant3;
  erro_ant3 = erro_ant2;
  erro_ant2 = erro_ant1;
  erro_ant1 = erro; 
}

// Configuração inicial
void setup() {
    Serial.begin(115200);

    // Configuração dos pinos conectados aos canais do encoder como entrada
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(2), contador_pulso_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), contador_pulso_R, CHANGE);

    startMillis = millis();
}

// Loop principal
void loop() {
    unsigned long currentMillis = millis();
    // Sensores leitura
    leituraSensores();
    Bsensorsvalue();
   
    // Calculo PID
    if ((currentMillis - previousMillis) >= interval) {
        calcularVelocidade();
        PID();
    }
}
