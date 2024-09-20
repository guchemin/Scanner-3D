// Pinos do driver A4988 para o motor NEMA 17 (Motor X)
#define dirPinNEMA 5
#define stepPinNEMA 2
#define enablePinNEMA 8

// Pinos do driver A4988 para o motor 28BYJ-48 (Motor Y)
#define dirPinBYJ 6
#define stepPinBYJ 3
#define enablePinBYJ 8

// Pino do sensor de distância Sharp
#define sensorPin A0

// Pino do botão de pulso
#define buttonPin 12

// Variáveis de controle
int stepsNema = 100;
int stepsBYJ = 11605; // 5 voltas e 2/3 de volta
int voltasNEMA = 0;
int totalVoltasNEMA = 33; // 33 subidas de 4mm cada = 132mm = 13,2cm de subida
int readInterval = stepsBYJ / 211; // Intervalo de passos para fazer uma leitura do sensor
int mediu = 0;
bool isRunning = false;
bool buttonState = false;
bool lastButtonState = false;

void setup() {
  // Configura os pinos do NEMA 17 (Motor X)
  pinMode(dirPinNEMA, OUTPUT);
  pinMode(stepPinNEMA, OUTPUT);
  pinMode(enablePinNEMA, OUTPUT);
  digitalWrite(enablePinNEMA, LOW);

  // Configura os pinos do 28BYJ-48 (Motor Y)
  pinMode(dirPinBYJ, OUTPUT);
  pinMode(stepPinBYJ, OUTPUT);
  pinMode(enablePinBYJ, OUTPUT);
  digitalWrite(enablePinBYJ, LOW);

  // Configura os pinos de microstepping
  pinMode(9, OUTPUT); // MS1 para NEMA 17 (Motor X)
  pinMode(10, OUTPUT); // MS2 para NEMA 17 (Motor X)
  pinMode(11, OUTPUT); // MS3 para NEMA 17 (Motor X)
  pinMode(12, OUTPUT); // MS1 para 28BYJ-48 (Motor Y)
  pinMode(13, OUTPUT); // MS2 para 28BYJ-48 (Motor Y)
  pinMode(14, OUTPUT); // MS3 para 28BYJ-48 (Motor Y)

  digitalWrite(9, HIGH); // MS1 para NEMA 17 (Motor X)
  digitalWrite(10, HIGH); // MS2 para NEMA 17 (Motor X)
  digitalWrite(11, HIGH); // MS3 para NEMA 17 (Motor X)
  digitalWrite(12, HIGH); // MS1 para 28BYJ-48 (Motor Y)
  digitalWrite(13, HIGH); // MS2 para 28BYJ-48 (Motor Y)
  digitalWrite(14, HIGH); // MS3 para 28BYJ-48 (Motor Y)

  // Configura o botão de pulso
  pinMode(buttonPin, INPUT_PULLUP);

  // Inicia a comunicação serial para depuração
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
 
  delay(5000);
}

void loop() {
  // Lê o estado do botão
  buttonState = digitalRead(buttonPin);

  // Verifica se o botão foi pressionado (mudança de estado)
  if (buttonState == LOW && lastButtonState == HIGH) {
    if (isRunning) {
      // Se estiver em execução, para e volta para a base
      isRunning = false;
      returnNEMA();
    } else {
      // Se não estiver em execução, inicia o processo
      isRunning = true;
      voltasNEMA = 0;  // Reinicia a contagem de voltas
    }
  }
  lastButtonState = buttonState;

  if (isRunning) {
    if (voltasNEMA < totalVoltasNEMA) {
      // Motor 28BYJ-48 (Motor Y) dá 5 voltas e 2/3 de volta com leituras do sensor
      mediu = moveMotorWithSensor(stepPinBYJ, dirPinBYJ, stepsBYJ, readInterval);
       
      delay(1000);

      // Motor NEMA 17 (Motor X) dá meia volta 
      if (mediu) {
        moveMotor(stepPinNEMA, dirPinNEMA, stepsNema);
      } else {
        returnNEMA();
      }

      // Repete o ciclo
      delay(1000); // Espera 1 segundo antes de repetir o ciclo
     
      voltasNEMA++;
    } else {
      if (mediu) {
        returnNEMA(); // Volta para a base
      }
      isRunning = false;  // Para o funcionamento
    }
  }
}

// Função para mover um motor com leituras do sensor
int moveMotorWithSensor(int stepPin, int dirPin, int steps, int interval) {
  mediu = 0;
  digitalWrite(dirPin, HIGH); // Define a direção
  for (int i = 0; i < steps; i++) {
    // Verifica o estado do botão durante o movimento
    buttonState = digitalRead(buttonPin);
    if (buttonState == LOW && lastButtonState == HIGH) {
      isRunning = false;  // Para a execução
      Serial.println(voltasNEMA);
      return 0;  // Retorna imediatamente, interrompendo o movimento
    }
    lastButtonState = buttonState;

    // Continua o movimento do motor
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500); // Ajuste este valor para controlar a velocidade
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500); // Ajuste este valor para controlar a velocidade

    // Faz a leitura do sensor a cada intervalo de passos
    if (i % interval == 0) {
      int sensorValue = analogRead(sensorPin);
      float distCm = 10650.08 * pow(sensorValue, -0.935) - 10.0;
      float raio = 38.5 - distCm;
      if (raio > 0 && raio < 10) {
        mediu = true;
      }
      Serial.println(raio); // Imprime o valor do sensor na serial para depuração
      delay(50);
    }
  }
  Serial.println(-1); // Para reconhecer a mudança de altura
  return mediu;
}

// Função para mover um motor
void moveMotor(int stepPin, int dirPin, int steps) {
  digitalWrite(dirPin, HIGH); // Define a direção
  for (int i = 0; i < steps; i++) {
        digitalWrite(stepPin, HIGH);
    delayMicroseconds(500); // Ajuste este valor para controlar a velocidade
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500); // Ajuste este valor para controlar a velocidade
  }
}


// Função para o motor NEMA 17 (Motor X) voltar à posição original
void returnNEMA() {
  Serial.println(-10);
  digitalWrite(dirPinNEMA, LOW); // Define a direção oposta
  while (voltasNEMA > 0) {
    for (int i = 0; i < stepsNema; i++) {
      digitalWrite(stepPinNEMA, HIGH);
      delayMicroseconds(500); // Ajuste este valor para controlar a velocidade
      digitalWrite(stepPinNEMA, LOW);
      delayMicroseconds(500); // Ajuste este valor para controlar a velocidade
    }
    voltasNEMA--;
  }
  voltasNEMA = totalVoltasNEMA;
}
