#include "RA.h"
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <SPIFFS.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <EEPROM.h>


//#include <Wire.h>
//#include <Adafruit_ADS1X15.h>

// Declare file object
File file;

BluetoothSerial SerialBT; // Instância para comunicação Bluetooth

RA::RA() {}


//Fim das entradas analógicas
//Variáveis para aquisição de dados
int ValorFoto1 = 0;                                   //Variável para fotodetector de referência (foto1)
int ValorFoto2 = 0;                                   //Variável para fotodetector de saída (foto2)
int N;                                                //Número de médias para obtenção de um ponto na curva de reflectância
float R = 0;                                          //Valor médio de reflectância (Será plotado no gráfico)
float soma = 0;                                       //Variável auxiliar para cálculo de média
float reflect[2];                                     //Vetor para armazenar valores adquiridos de reflectância(Tem que ter o tamanho de N)
//Fim das variáveis para aquisição de dados

float pi = 3.14159;
// Definição de parâmetros do prisma
float alpha = pi/3;                                     //Ângulo em rad
float beta = pi/3;
float gama = pi/3;
float w = 0;                                            //Posição atual do feixe na face superior do prisma, inicialmente 0
float a = 50;                                           //Dimensões do prisma (em mm)
float b = 50;
float c = 50;
float n = 1.5;                                          //Índice de refração do prisma
//Fim da definição de parâmetros do prisma

//Definições de variáveis úteis para correção da posição do feixe (Proveniente do artigo)
float theta = 6;                                      //Ângulo a ser rotacionado
float theta_atual;                                      //Ângulo que o sistema está, essa variável vai ser usada para plotar o gráfico.
float l;                                                //Variável auxiliar no cálculo (pode ser excluída depois)
float d;                                                //Distância necessária transladar para manter feixe no mesmo ponto
float theta_r;                                          //Ângulo refratado. Útil no cálculo de d
float r;                                                //Raio da circunferência inscrita
//Fim das definições de variáveis
//Definição de variáveis para movimentação 2D
float esp = 2;                                     // Espaçamento entre células em mm
float x,y = 0;                                     // Posição em x e y em mm, sempre inicia-se em (0.0)
int celula;                                        // Célula que o reflectômetro deve se posicionar, receber valo de 0 a 120 do usuário
//Fim da definição de variáveis para movimentação 2D

//definição das variáveis de movimentação eixo W
float ww = 0;                                       // Posição W inicial, sempre inicia em 0. i.e., o mais abaixo possível
float w_atual;                                      //posição atual w

//fim das variáveis de movimentação do eixo W


float distance = 0.0;


void RA::Begin() {

  Serial.begin(115200);


  SerialBT.begin("ReflectometroAutomatizado"); // Nome do dispositivo Bluetooth

  pinMode(stepPinW, OUTPUT);
  pinMode(dirPinX, OUTPUT);
  pinMode(homePinW, INPUT);
  pinMode(enableW, OUTPUT);
  
  pinMode(stepPinT, OUTPUT);
  pinMode(dirPinT, OUTPUT);
  pinMode(enableT, OUTPUT);

  pinMode(stepPinX, OUTPUT);
  pinMode(dirPinX, OUTPUT);
  pinMode(enableX, OUTPUT);

  pinMode(stepPinY, OUTPUT);
  pinMode(dirPinY, OUTPUT);
  pinMode(enableY, OUTPUT);
  
  digitalWrite(enableW, HIGH);
  digitalWrite(enableT, HIGH);
  digitalWrite(enableX, HIGH);
  digitalWrite(enableY, HIGH);
  
/*
    // Inicializa o ADS1115 no endereço padrão (0x48 para ADDR = GND)
  if (!ads.begin()) {
    Serial.println("Erro ao inicializar o ADS1115!");
    while (1);
  }
  */
    if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
    }
  Serial.println("Fim Begin");
}

// Função para armazenar dados em SPIFFS
void RA::storeInSPIFFS(const char* path, float angulo, float w) {
    File file = SPIFFS.open(path, FILE_APPEND);
    if (!file) {
        Serial.println("Falha ao abrir o arquivo para escrita.");
        return;
    }
    file.printf("%f,%f\n", angulo, w);
    file.close();
}

// Função para ler dados de SPIFFS (para uso frequente)
void RA::readFromSPIFFS(const char* path) {
    File file = SPIFFS.open(path, FILE_READ);
    if (!file) {
        Serial.println("Falha ao abrir o arquivo para leitura.");
        return;
    }

        while (file.available()) {
        String line = file.readStringUntil('\n');
        Serial.println(line); // Aqui você pode processar a linha lida conforme necessário
    }
    file.close();
}


void RA::processFunction(String functionName, String argument1, String argument2, String argument3) {
  Serial.print("Funcao recebida: ");
  Serial.print(functionName);
  Serial.print(", Argumento1: ");
  Serial.println(argument1);
  Serial.print(", Argumento2: ");
  Serial.println(argument2);
  Serial.print(", Argumento3: ");
  Serial.println(argument3);

  // Execute a função com base no nome

  if (functionName == "IrHome") {

    if(argument1 == "W")
    {
      IrHome(motorW);
    }

    else if (argument1 == "T")
    {
      IrHome(motorT);
    }
    else if (argument1 == "X")
    {
      IrHome(motorX);
    }
    else if (argument1 == "Y")
    {
      IrHome(motorY);
    }

    else
    {
      SerialBT.print("Argumento inválido");
    }
    SerialBT.print("IrHome: ");SerialBT.print(argument1); SerialBT.println(" Executado");
  } 
  
  
  
  else if (functionName == "passo") {
     if(argument1 == "W")
    {
      passo(motorW);
    }

    else if (argument1 == "T")
    {
      passo(motorT);
    }
    else if (argument1 == "X")
    {
      passo(motorX);
    }
    else if (argument1 == "Y")
    {
      passo(motorY);
    }

    else
    {
      SerialBT.print("Argumento inválido");
    }
    SerialBT.print("passo : ");SerialBT.print(argument1); SerialBT.println(" Executado");
  } 
  
  else if (functionName == "mover1mm") {
         if(argument1 == "W")
    {
      mover1mm(motorW);
    }

    else if (argument1 == "T")
    {
      mover1mm(motorT);
    }
    else if (argument1 == "X")
    {
      mover1mm(motorX);
    }
    else if (argument1 == "Y")
    {
      mover1mm(motorY);
    }

    else
    {
      SerialBT.print("Argumento inválido");
    }
    SerialBT.print("mover1mm : ");SerialBT.print(argument1); SerialBT.println(" Executado");
  } 
  
  else if (functionName == "mover10mm") {
     if(argument1 == "W")
    {
      mover10mm(motorW);
    }

    else if (argument1 == "T")
    {
      mover10mm(motorT);
    }
    else if (argument1 == "X")
    {
      mover10mm(motorX);
    }
    else if (argument1 == "Y")
    {
      mover10mm(motorY);
    }

    else
    {
      SerialBT.print("Argumento inválido");
    }
    SerialBT.print("mover10mm : ");SerialBT.print(argument1); SerialBT.println(" Executado");
  } 
  
  else if (functionName == "mover100mm") {
    if(argument1 == "W")
    {
      mover100mm(motorW);
    }

    else if (argument1 == "T")
    {
      mover100mm(motorT);
    }
    else if (argument1 == "X")
    {
      mover100mm(motorX);
    }
    else if (argument1 == "Y")
    {
      mover100mm(motorY);
    }

    else
    {
      SerialBT.print("Argumento inválido");
    }
    SerialBT.print("mover100mm : ");SerialBT.print(argument1); SerialBT.println(" Executado");

  } 
  
  else if (functionName == "moverDistancia") {


    float distance = argument2.toFloat();  // Converte a string do argumento para float

     if(argument1 == "W")
    {
     moverDistancia(distance, motorW);
    }

    else if (argument1 == "T")
    {
      moverDistancia(distance, motorT);
    }
    else if (argument1 == "X")
    {
      moverDistancia(distance, motorX);
    }
    else if (argument1 == "Y")
    {
      moverDistancia(distance, motorY);
    }

    else
    {
      SerialBT.print("Argumento inválido");
    }
    SerialBT.print("moverDistancia : ");SerialBT.print(argument1); SerialBT.println(" Executado");

    
 
  } 
  
  else if (functionName == "ConfigParaCima") {

         if(argument1 == "W")
    {
     ConfigParaCima(motorW);
    }

    else if (argument1 == "T")
    {
      ConfigParaCima(motorT);
    }
    else if (argument1 == "X")
    {
      ConfigParaCima(motorX);
    }
    else if (argument1 == "Y")
    {
      ConfigParaCima(motorY);
    }

    else
    {
      SerialBT.print("Argumento inválido");
    }
    SerialBT.print("configParaCima : ");SerialBT.print(argument1); SerialBT.println(" Executado");
    

  } 
 else if (functionName == "ConfigParabaixo") {
            if(argument1 == "W")
    {
     ConfigParabaixo(motorW);
    }

    else if (argument1 == "T")
    {
      ConfigParabaixo(motorT);
    }
    else if (argument1 == "X")
    {
      ConfigParabaixo(motorX);
    }
    else if (argument1 == "Y")
    {
      ConfigParabaixo(motorY);
    }

    else
    {
      SerialBT.print("Argumento inválido");
    }
    SerialBT.print("configParabaixo : ");SerialBT.print(argument1); SerialBT.println(" Executado");
    
  } 
  
  else if (functionName == "MoverUmPonto") {
    MoverUmPontoW();
    SerialBT.print("MoverUmPonto "); SerialBT.println("Executado");
  } 
 
  
  else if (functionName == "ConstroiArray") {
    ConstroiArray();
    SerialBT.print("ConstroiArray "); SerialBT.println("Executado");
  }

  else if (functionName == "reflectancia") {
    reflectancia();
    SerialBT.print("reflectancia "); SerialBT.println("Executado");
  }

  else if (functionName == "varreduraAngular") {
    varreduraAngular(argument1.toFloat(),argument2.toFloat(), argument3.toFloat());

    SerialBT.print("varreduraAngular "); SerialBT.println("Executado");
  }
  else if (functionName == "percorrerSuperficie") {
    percorrerSuperficie(argument1.toFloat(),argument2.toFloat(), argument3.toFloat());

    SerialBT.print("percorrerSuperficie "); SerialBT.println("Executado");
  }

  else if (functionName == "imprimirValoresMedios") {
    imprimirValoresMedios();

    SerialBT.print("imprimirValoresMedios "); SerialBT.println("Executado");
  }

   else {
    Serial.println("Funcao desconhecida");
    SerialBT.println("Funcao desconhecida ");
  }
}

void RA::EsperaMensagem() {
  if (SerialBT.available()) {
    String incomingData = SerialBT.readStringUntil('\n'); // Leia a string até encontrar uma quebra de linha
    
    // Encontre a posição do primeiro caractere de espaço
    int firstSpacePos = incomingData.indexOf(' ');

    // Verifique se há espaço na string
    if (firstSpacePos != -1) {
      // Separe o nome da função
      String functionName = incomingData.substring( firstSpacePos);

      // Encontre a posição do segundo caractere de espaço
      int secondSpacePos = incomingData.indexOf(' ', firstSpacePos + 1);

      // Verifique se há um segundo espaço na string
      if (secondSpacePos != -1) {
        // Separe o primeiro argumento
        String argument1 = incomingData.substring(firstSpacePos + 1, secondSpacePos);

        // Encontre a posição do terceiro caractere de espaço
        int thirdSpacePos = incomingData.indexOf(' ', secondSpacePos + 1);

        // Verifique se há um terceiro espaço na string
        if (thirdSpacePos != -1) {
          // Separe o segundo e terceiro argumento
          String argument2 = incomingData.substring(secondSpacePos + 1, thirdSpacePos);
          String argument3 = incomingData.substring(thirdSpacePos + 1);

          // Execute a função com base no nome e nos argumentos
          processFunction(functionName, argument1, argument2, argument3);
        } else {
          Serial.println("Formato inválido. Use 'funcao argumento1 argumento2 argumento3'");
        }
      } else {
        Serial.println("Formato inválido. Use 'funcao argumento1 argumento2 argumento3'");
      }
    } else {
      Serial.println("Formato inválido. Use 'funcao argumento1 argumento2 argumento3'");
    }
  }
}


void RA::ConfigParaCima(int motor)
{
  if(motor == motorW)
  {
      digitalWrite(dirPinX, HIGH);
  }

 else if(motor == motorT)
  {
      digitalWrite(dirPinT, HIGH);
  }
else if(motor == motorX)
  {
      digitalWrite(dirPinX, HIGH);
  }
else if(motor == motorY)
  {
      digitalWrite(dirPinY, HIGH);
  }
  else{}
  
}


void RA::ConfigParabaixo(int motor)
{
    if(motor == motorW)
  {
      digitalWrite(dirPinX, LOW);
  }

 else if(motor == motorT)
  {
      digitalWrite(dirPinT, LOW);
  }
else if(motor == motorX)
  {
      digitalWrite(dirPinX, LOW);
  }
else if(motor == motorY)
  {
      digitalWrite(dirPinY, LOW);
  }
  else{}
}


void RA::IrHome(int motor)
{

}

void RA::passo(int motor)
{

      if ((w_atual < wMin_mm || w_atual > wMax_mm) ||
        (theta_atual < angMinGraus || theta_atual > angMaxGraus) ||
        (x < xMin_mm || x > xMax_mm) ||
        (y < yMin_mm || y > yMax_mm)) {
        
        Serial.println("Valor fora dos limites! Movimento interrompido.");
        return;  // Interrompe a execução da função
    }

    if(motor == motorW)
  {
      digitalWrite(enableW, LOW);

      //dá um pulso no pino step
		  delayMicroseconds(delayPassoW);
		  digitalWrite(stepPinW, HIGH);
		  delayMicroseconds(delayPassoW);
		  digitalWrite(stepPinW, LOW);

      digitalWrite(enableW, HIGH);
  }

 else if(motor == motorT)
  {
      digitalWrite(enableT, LOW);
      
      //dá um pulso no pino step
		  delayMicroseconds(delayPassoT);
		  digitalWrite(stepPinT, HIGH);
		  delayMicroseconds(delayPassoT);
		  digitalWrite(stepPinT, LOW);

      digitalWrite(enableT, HIGH);
  }
else if(motor == motorX)
  {
      digitalWrite(enableX, LOW);
      
      //dá um pulso no pino step
		  delayMicroseconds(delayPassoX);
		  digitalWrite(stepPinX, HIGH);
		  delayMicroseconds(delayPassoX);
		  digitalWrite(stepPinX, LOW);

      digitalWrite(enableX, HIGH);
  }
else if(motor == motorY)
  {
      digitalWrite(enableY, LOW);
      
      //dá um pulso no pino step
		  delayMicroseconds(delayPassoY);
		  digitalWrite(stepPinY, HIGH);
		  delayMicroseconds(delayPassoY);
		  digitalWrite(stepPinY, LOW);

      digitalWrite(enableY, HIGH);
  }
  else{}
	

	
}

void RA::mover1mm(int motor)
{
  //faz um loop para mover 1mm

      if(motor == motorW)
  {
    for(int i=0; i< (int)(1/kW); i++)
     {
        passo(motor);
      }
  }

 else if(motor == motorT)
  {
    for(int i=0; i< (int)(1/kT); i++)
     {
        passo(motor);
      }
  }
else if(motor == motorX)
  {
    for(int i=0; i< (int)(1/kX); i++)
     {
        passo(motor);
      }
  }
else if(motor == motorY)
  {
    for(int i=0; i< (int)(1/kY); i++)
     {
        passo(motor);
      }
  }
  else{}

}

void RA::mover10mm(int motor)
{
  //faz um loop para mover 10mm
      if(motor == motorW)
  {
    for(int i=0; i< (int)(10/kW); i++)
     {
        passo(motor);
      }
  }

 else if(motor == motorT)
  {
    for(int i=0; i< (int)(10/kT); i++)
     {
        passo(motor);
      }
  }
else if(motor == motorX)
  {
    for(int i=0; i< (int)(10/kX); i++)
     {
        passo(motor);
      }
  }
else if(motor == motorY)
  {
    for(int i=0; i< (int)(10/kY); i++)
     {
        passo(motor);
      }
  }
  else{}
}

void RA::mover100mm(int motor)
{
      if(motor == motorW)
  {
    for(int i=0; i< (int)(100/kW); i++)
     {
        passo(motor);
      }
  }

 else if(motor == motorT)
  {
    for(int i=0; i< (int)(100/kT); i++)
     {
        passo(motor);
      }
  }
else if(motor == motorX)
  {
    for(int i=0; i< (int)(100/kX); i++)
     {
        passo(motor);
      }
  }
else if(motor == motorY)
  {
    for(int i=0; i< (int)(100/kY); i++)
     {
        passo(motor);
      }
  }
  else{}
}


void RA::avisaMovimentoInsuficiente(float movimento, int motor)
{
        if(motor == motorW)
  {
      if(movimento<kW)
  {
    Serial.println("Movimento Insuficiente");
  }
  }

 else if(motor == motorT)
  {
        if(movimento<kT)
  {
    Serial.println("Movimento Insuficiente");
  }
  }
else if(motor == motorX)
  {
      if(movimento<kX)
  {
    Serial.println("Movimento Insuficiente");
  }
  }
else if(motor == motorY)
  {
        if(movimento<kY)
  {
    Serial.println("Movimento Insuficiente");
  }
  }
  else{}

}


void RA::MoverUmPontoW()
{
    avisaMovimentoInsuficiente((float)(dL/nPossible), motorW);
    //faz um loop para mover um ponto
  for(int i=0; i< (int)((dL/nPossible)/kW); i++)
  {
    passo(motorW);
  }
}

void RA::moverDistancia(float d, int motor)
{
          if(motor == motorW)
  {
      if (d>0)
	{
		ConfigParaCima(motor);
	}
	
	else
	{
		ConfigParabaixo(motor);
	}
       //faz um loop para mover uma distancia d
	for(int i=0; i< (int)(d/kW); i++)
 	 {
 		   passo(motor);
  } 
  w_atual += d;
  }

 else if(motor == motorT)
  {
        if (d>0)
	{
		ConfigParaCima(motor);
	}
	
	else
	{
		ConfigParabaixo(motor);
	}
       //faz um loop para mover uma distancia d
	for(int i=0; i< (int)(d/kT); i++)
 	 {
 		   passo(motor);
  } 

  }
else if(motor == motorX)
  {
        if (d>0)
	{
		ConfigParaCima(motor);
	}
	
	else
	{
		ConfigParabaixo(motor);
	}
       //faz um loop para mover uma distancia d
	for(int i=0; i< (int)(d/kX); i++)
 	 {
 		   passo(motor);
  } 

      
  }
else if(motor == motorY)
  {
          if (d>0)
	{
		ConfigParaCima(motor);
	}
	
	else
	{
		ConfigParabaixo(motor);
	}
       //faz um loop para mover uma distancia d
	for(int i=0; i< (int)(d/kY); i++)
 	 {
 		   passo(motor);
  } 

  }
  else{}
  
	
}




// Função principal para procurar o ponto W de maior reflectância e armazena na flash
void RA::ConstroiArray(){
    float maiorReflectancia;
    float anguloMaiorReflectancia;
    const char* filePath = "/coordenadas.txt";

    moverDistancia(w_atual - ww, motorW); // Vai para o ponto mais abaixo de rotação

    for (int i = 0; i < nPossible; i++) {
        moverDistancia(i * (dL / nPossible), motorW); // Para no ponto i do eixo W

        if (i == 0) {
            // Grava a posição inicial
            storeInSPIFFS(filePath, theta_atual, w_atual);
        } else {
            maiorReflectancia = reflectancia();

            for (int j = 0; j < passoGrauBusca; j++) {
                rotacionar(grauBusca / passoGrauBusca);

                if (reflectancia() > maiorReflectancia) {
                    maiorReflectancia = reflectancia();
                    anguloMaiorReflectancia = theta_atual;
                }
            }

            // Grava o ângulo de maior reflectância no ponto i
            storeInSPIFFS(filePath, anguloMaiorReflectancia, w_atual);

            // Rotacionar de volta para o último ponto cadastrado
            irParaAngulo(anguloMaiorReflectancia);
        }
    }

}

void RA::alinharMotorW(float theta) {
    if (!SPIFFS.begin(true)) {
        Serial.println("Falha ao montar o sistema de arquivos");
        return;
    }

    File file = SPIFFS.open("/coordenadas.txt", FILE_READ);
    if (!file) {
        Serial.println("Falha ao abrir o arquivo para leitura");
        return;
    }

    float storedTheta, storedW;
    float closestTheta = 0;
    float closestW = 0;
    float minDifference = 360.0;  // Inicializa com um valor grande (360 graus é o máximo de diferença possível)
    bool coordenadaEncontrada = false;

    while (file.available()) {
        String line = file.readStringUntil('\n');  // Lê uma linha do arquivo
        int commaIndex = line.indexOf(',');  // Encontra a posição da vírgula
        storedTheta = line.substring(0, commaIndex).toFloat();  // Extrai e converte o theta
        storedW = line.substring(commaIndex + 1).toFloat();  // Extrai e converte o W

        float difference = abs(storedTheta - theta);  // Calcula a diferença absoluta entre o theta armazenado e o theta recebido

        if (difference < minDifference) {
            closestTheta = storedTheta;
            closestW = storedW;
            minDifference = difference;
            coordenadaEncontrada = true;
        }
    }

    file.close();

    if (coordenadaEncontrada) {
        moverDistancia(closestW - w_atual, motorW);  // Move o motor para o W correspondente ao theta mais próximo
        w_atual = closestW;  // Atualiza a posição atual de W
        Serial.print("Coordenada W ajustada para o theta mais próximo: ");
        Serial.println(closestTheta);
    } else {
        Serial.println("Nenhuma coordenada W encontrada para o theta fornecido.");
    }
}


float RA::reflectancia() {
    int32_t soma0 = 0;
    int32_t soma1 = 0;

    for (int i = 0; i < mediaADC; i++) {
        //soma0 += ads.readADC_SingleEnded(0);            //sinal foto entrada
        //soma1 += ads.readADC_SingleEnded(1);            //sinal foto saida
        delay(10);
    }

    if (soma1 == 0) {
        Serial.println("Erro: Divisão por zero na função reflectancia.");
        return 0.0;  // Retorna um valor neutro ou qualquer outro que faça sentido no seu contexto
    }

    return static_cast<float>(soma0) / soma1;
}


void RA::imprimirValoresMedios() {
    int32_t soma0 = 0;
    int32_t soma1 = 0;

    for (int i = 0; i < mediaADC; i++) {
        // soma0 += ads.readADC_SingleEnded(0);  // sinal foto entrada
        // soma1 += ads.readADC_SingleEnded(1);  // sinal foto saída
        delay(10);
    }

    // Proteção contra divisão por zero
    float reflectancia = (soma1 != 0) ? static_cast<float>(soma0) / soma1 : 0.0;

    // Calcula médias
    float mediaEntrada = static_cast<float>(soma0) / mediaADC;
    float mediaSaida = static_cast<float>(soma1) / mediaADC;

    // Imprime valores
    SerialBT.print("Valor de Entrada (média): ");
    SerialBT.println(mediaEntrada);
    SerialBT.print("Valor de Saída (média): ");
    SerialBT.println(mediaSaida);
    SerialBT.print("Reflectância: ");
    SerialBT.println(reflectancia);
}




void RA::irParaAngulo(float thetaAtual)
{
    //funcao irparaangulo
    rotacionar(theta_atual - thetaAtual);
    //alinha o motor W com o laser refletido após a rotação  
    alinharMotorW(thetaAtual); 
}


void RA::varreduraAngular(float anguloMin, float anguloMax, float resolucaoAngular) {
    // Vai para o ângulo mínimo
    irParaAngulo(anguloMin);

    // Realiza a varredura de ângulo mínimo até ângulo máximo
    for (float anguloAtual = anguloMin; anguloAtual <= anguloMax; anguloAtual += resolucaoAngular) {
        // Rotaciona para o ângulo atual
        irParaAngulo(anguloAtual);

        // Obtém a reflectância no ângulo atual
        float valorReflectancia = reflectancia();

        // Imprime o par ângulo, reflectância via Serial Bluetooth
        SerialBT.print(anguloAtual);
        SerialBT.print(", ");
        SerialBT.println(valorReflectancia);
    }
}

void RA::percorrerSuperficie(float max, float min, float res) {

    const int totalCelulas = 121;

    for (int celula = 0; celula < totalCelulas; celula++) {
        ir_para(celula); // Chama a função para ir para a célula atual

        // Printa a célula atual
        SerialBT.print(celula);
        SerialBT.println(":");
        // printa a varredura angular na célula atual
        varreduraAngular(max, min, res);

    }
}




void RA::rotacionar (float theta)
{
  if (theta<0){                                         //Verifica para qual lado deve rotacionar
  digitalWrite(dirPinT, HIGH);                           //Rotaciona para região negativa(Esquerda)
  }
  else{
    digitalWrite(dirPinT, LOW);                          //Rotaciona para região positiva (Direita)
  }
  if(abs(theta)<=0.003)                                       //Passo subdividido em 1/16
  {
    digitalWrite(MS1,HIGH);
    digitalWrite(MS2,HIGH);
    digitalWrite(MS3,HIGH);
  }
  if(abs(theta)>0.003 && abs(theta)<=0.007)                                       //Passo subdividido em 1/8
  {
    digitalWrite(MS1,HIGH);
    digitalWrite(MS2,HIGH);
    digitalWrite(MS3,LOW);
  }
  if(abs(theta)>0.007 && abs(theta)<=0.015)                                       //Passo subdividido em 1/4
  {
    digitalWrite(MS1,LOW);
    digitalWrite(MS2,HIGH);
    digitalWrite(MS3,LOW); 
  }
  if(abs(theta)>0.015 && abs(theta)<=0.03)                                        //Passo subdividido em 1/2
  {
    digitalWrite(MS1,HIGH);
    digitalWrite(MS2,LOW);
    digitalWrite(MS3,LOW);
  }
  else                                                  //Passo completo
  {
    digitalWrite(MS1, LOW);
    digitalWrite(MS2, LOW);
    digitalWrite(MS3, LOW);
  }
  for(int i = 0; i < (int)(theta/0.061); i++)           //Dá o número de passos relacionado ao ângulo a ser rotacionado - Atenção para a imprecisão nesse for
  {
    digitalWrite(stepPinT,HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinT,LOW);
    delayMicroseconds(500);
  }
  
  l = a*sin(beta/2)*cos(alpha/2)/sin((alpha+beta)/2);
  r = a/(cos(alpha/2)/sin(alpha/2) + cos(beta/2)/sin(beta/2));
  theta_r = asin(sin(theta)/n);
  d = -w + (1/cos(alpha+theta))*(2*sin(theta/2)*((-l+a/2)*sin(alpha+theta/2)+r*cos(alpha+theta/2)) - (w+alpha/2)*sin(alpha)*sin(theta - theta_r)/cos(theta_r));

  if (d<0)                                             //Verifica se translada no sentido +x ou -x (VERIFICAR ISSO NO MOTOR)
  {                   
    digitalWrite(dirPinX, HIGH);                       //Translada para região negativa(Esquerda)
  }
  else{
    digitalWrite(dirPinX, LOW);                        //Translada para região positiva (Direita)
  }
  for (int i=0; i < (int)(d/0.011); i++)               //Dá o número de passos relacionado a distância a ser transladada em mm
  {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinX,LOW);
    delayMicroseconds(500);
  }
  theta_atual = theta_atual+theta;
   x = x+d;
}


void RA::ir_para(int celula) {
  
  switch(celula){
    case 120:
    {
    float posicao120_x = -5*esp;
    float posicao120_y = 5*esp;

    if ((posicao120_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao120_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao120_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao120_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao120_x;                            // Atualiza a posição em x e y
    y = posicao120_y;
    }
    break;
    case 119:
    {
    float posicao119_x = -4*esp;
    float posicao119_y = 5*esp;

    if ((posicao119_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao119_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao119_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao119_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao119_x;                            // Atualiza a posição em x e y
    y = posicao119_y;
    }
    break;
    case 118:
    {
    float posicao118_x = -3*esp;
    float posicao118_y = 5*esp;

    if ((posicao118_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao118_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao118_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao118_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao118_x;                            // Atualiza a posição em x e y
    y = posicao118_y;
    }
    break;
    case 117:
    {
    float posicao117_x = -2*esp;
    float posicao117_y = 5*esp;

    if ((posicao117_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao117_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao117_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao117_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao117_x;                            // Atualiza a posição em x e y
    y = posicao117_y;
    }
    break;
    case 116:
    {
    float posicao116_x = -esp;
    float posicao116_y = 5*esp;

    if ((posicao116_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao116_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao116_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao116_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao116_x;                            // Atualiza a posição em x e y
    y = posicao116_y;
    }
    break;
    case 115:
    {
    float posicao115_x = 0;
    float posicao115_y = 5*esp;

    if ((posicao115_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao115_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao115_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao115_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao115_x;                            // Atualiza a posição em x e y
    y = posicao115_y;
    }
    break;
    case 114:
    {
    float posicao114_x = esp;
    float posicao114_y = 5*esp;

    if ((posicao114_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao114_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao114_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao114_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao114_x;                            // Atualiza a posição em x e y
    y = posicao114_y;
    }
    break;
    case 113:
    {
    float posicao113_x = 2*esp;
    float posicao113_y = 5*esp;

    if ((posicao113_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao113_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao113_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao113_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao113_x;                            // Atualiza a posição em x e y
    y = posicao113_y;
    }
    break;
    case 112:
    {
    float posicao112_x = 3*esp;
    float posicao112_y = 5*esp;

    if ((posicao112_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao112_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao112_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao112_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao112_x;                            // Atualiza a posição em x e y
    y = posicao112_y;

    }
    break;

    case 111:
  
    {
    float posicao111_x = 4*esp;
    float posicao111_y = 5*esp;

    if ((posicao111_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao111_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao111_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao111_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao111_x;                            // Atualiza a posição em x e y
    y = posicao111_y;
    }
    break;
    case 110:
    {
    float posicao110_x = 5*esp;
    float posicao110_y = 5*esp;

    if ((posicao110_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao110_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao110_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao110_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao110_x;                            // Atualiza a posição em x e y
    y = posicao110_y;
    }
    break;
    case 109:
    {
    float posicao109_x = -5*esp;
    float posicao109_y = 4*esp;

    if ((posicao109_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao109_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao109_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao109_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao109_x;                            // Atualiza a posição em x e y
    y = posicao109_y;
    }
    break;
    case 108:
    {
    float posicao108_x = -4*esp;
    float posicao108_y = 4*esp;

    if ((posicao108_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao108_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao108_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao108_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao108_x;                            // Atualiza a posição em x e y
    y = posicao108_y;
    }
    break;
    case 107:
    {
    float posicao107_x = -3*esp;
    float posicao107_y = 4*esp;

    if ((posicao107_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao107_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao107_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao107_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao107_x;                            // Atualiza a posição em x e y
    y = posicao107_y;
    }
    break;
    case 106:
    {
    float posicao106_x = -2*esp;
    float posicao106_y = 4*esp;

    if ((posicao106_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao106_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao106_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao106_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao106_x;                            // Atualiza a posição em x e y
    y = posicao106_y;
    }
    break;
    case 105:
    {
    float posicao105_x = -esp;
    float posicao105_y = 4*esp;

    if ((posicao105_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao105_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao105_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao105_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao105_x;                            // Atualiza a posição em x e y
    y = posicao105_y;
    }
    break;
    case 104:
    {
    float posicao104_x = 0;
    float posicao104_y = 4*esp;

    if ((posicao104_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao104_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao104_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao104_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao104_x;                            // Atualiza a posição em x e y
    y = posicao104_y;
    }
    break;
    case 103:
    {
    float posicao103_x = esp;
    float posicao103_y = 4*esp;

    if ((posicao103_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao103_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao103_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao103_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao103_x;                            // Atualiza a posição em x e y
    y = posicao103_y;
    }
    break;
    case 102:
    {
    float posicao102_x = 2*esp;
    float posicao102_y = 4*esp;

    if ((posicao102_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao102_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao102_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao102_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao102_x;                            // Atualiza a posição em x e y
    y = posicao102_y;
    }
    break;
    case 101:
    {
    float posicao101_x = 3*esp;
    float posicao101_y = 4*esp;

    if ((posicao101_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao101_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao101_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao101_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao101_x;                            // Atualiza a posição em x e y
    y = posicao101_y;
    }
    break;
    case 100:
    {
    float posicao100_x = 4*esp;
    float posicao100_y = 4*esp;

    if ((posicao100_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao100_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao100_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao100_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao100_x;                            // Atualiza a posição em x e y
    y = posicao100_y;
    }
    break;
    case 99:
    {
    float posicao99_x = 5*esp;
    float posicao99_y = 4*esp;

    if ((posicao99_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao99_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao99_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao99_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao99_x;                            // Atualiza a posição em x e y
    y = posicao99_y;
    }
    break;
    case 98:
    {
    float posicao98_x = -5*esp;
    float posicao98_y = 3*esp;

    if ((posicao98_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao98_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao98_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao98_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao98_x;                            // Atualiza a posição em x e y
    y = posicao98_y;
    }
    break;
    case 97:
    {
    float posicao97_x = -4*esp;
    float posicao97_y = 3*esp;

    if ((posicao97_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao97_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao97_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao97_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao97_x;                            // Atualiza a posição em x e y
    y = posicao97_y;
    }
    break;
    case 96:
    {
    float posicao96_x = -3*esp;
    float posicao96_y = 3*esp;

    if ((posicao96_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao96_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao96_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao96_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao96_x;                            // Atualiza a posição em x e y
    y = posicao96_y;
    }
    break;
    case 95:
    {
    float posicao95_x = -2*esp;
    float posicao95_y = 3*esp;

    if ((posicao95_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao95_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao95_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao95_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao95_x;                            // Atualiza a posição em x e y
    y = posicao95_y;
    }
    break;
    case 94:
    {
    float posicao94_x = -esp;
    float posicao94_y = 3*esp;

    if ((posicao94_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao94_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao94_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao94_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao94_x;                            // Atualiza a posição em x e y
    y = posicao94_y;
    }
    break;
    case 93:
    {
    float posicao93_x = 0;
    float posicao93_y = 3*esp;

    if ((posicao93_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao93_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao93_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao93_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao93_x;                            // Atualiza a posição em x e y
    y = posicao93_y;
    }
    break;
    case 92:
    {
    float posicao92_x = esp;
    float posicao92_y = 3*esp;

    if ((posicao92_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao92_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao92_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao92_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao92_x;                            // Atualiza a posição em x e y
    y = posicao92_y;
    }
    break;
    case 91:
    {
    float posicao91_x = 2*esp;
    float posicao91_y = 3*esp;

    if ((posicao91_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao91_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao91_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao91_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao91_x;                            // Atualiza a posição em x e y
    y = posicao91_y;
    }
    break;
    case 90:
    {
    float posicao90_x = 3*esp;
    float posicao90_y = 3*esp;

    if ((posicao90_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao90_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao90_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao90_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao90_x;                            // Atualiza a posição em x e y
    y = posicao90_y;
    }
    break;
    case 89:
    {
    float posicao89_x = 4*esp;
    float posicao89_y = 3*esp;

    if ((posicao89_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao89_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao89_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao89_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao89_x;                            // Atualiza a posição em x e y
    y = posicao89_y;
    }
    break;
    case 88:
    {
    float posicao88_x = 5*esp;
    float posicao88_y = 3*esp;

    if ((posicao88_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao88_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao88_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao88_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao88_x;                            // Atualiza a posição em x e y
    y = posicao88_y;
    }
    break;
    case 87:
    {
    float posicao87_x = -5*esp;
    float posicao87_y = 2*esp;

    if ((posicao87_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao87_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao87_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao87_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao87_x;                            // Atualiza a posição em x e y
    y = posicao87_y;
    }
    break;
    case 86:
    {
    float posicao86_x = -4*esp;
    float posicao86_y = 2*esp;

    if ((posicao86_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao86_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao86_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao86_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao86_x;                            // Atualiza a posição em x e y
    y = posicao86_y;
    }
    break;
    case 85:
    {
    float posicao85_x = -3*esp;
    float posicao85_y = 2*esp;

    if ((posicao85_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao85_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao85_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao85_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao85_x;                            // Atualiza a posição em x e y
    y = posicao85_y;
    }
    break;
    case 84:
    {
    float posicao84_x = -2*esp;
    float posicao84_y = 2*esp;

    if ((posicao84_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao84_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao84_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao84_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao84_x;                            // Atualiza a posição em x e y
    y = posicao84_y;
    }
    break;
    case 83:
    {
    float posicao83_x = -esp;
    float posicao83_y = 2*esp;

    if ((posicao83_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao83_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao83_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao83_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao83_x;                            // Atualiza a posição em x e y
    y = posicao83_y;
    }
    break;
    case 82:
    {
    float posicao82_x = 0;
    float posicao82_y = 2*esp;

    if ((posicao82_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao82_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao82_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao82_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao82_x;                            // Atualiza a posição em x e y
    y = posicao82_y;
    }
    break;
    case 81:
    {
    float posicao81_x = esp;
    float posicao81_y = 2*esp;

    if ((posicao81_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao81_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao81_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao81_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao81_x;                            // Atualiza a posição em x e y
    y = posicao81_y;
    }
    break;
    case 80:
    {
    float posicao80_x = 2*esp;
    float posicao80_y = 2*esp;

    if ((posicao80_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao80_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao80_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao80_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao80_x;                            // Atualiza a posição em x e y
    y = posicao80_y;
    }
    break;
    case 79:
    {
    float posicao79_x = 3*esp;
    float posicao79_y = 2*esp;

    if ((posicao79_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao79_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao79_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao79_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao79_x;                            // Atualiza a posição em x e y
    y = posicao79_y;
    }
    break;
    case 78:
    {
    float posicao78_x = 4*esp;
    float posicao78_y = 2*esp;

    if ((posicao78_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao78_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao78_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao78_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao78_x;                            // Atualiza a posição em x e y
    y = posicao78_y;
    }
    break;
    case 77:
    {
    float posicao77_x = 5*esp;
    float posicao77_y = 2*esp;

    if ((posicao77_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao77_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao77_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao77_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao77_x;                            // Atualiza a posição em x e y
    y = posicao77_y;
    }
    break;
    case 76:
    {
    float posicao76_x = -5*esp;
    float posicao76_y = esp;

    if ((posicao76_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao76_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao76_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao76_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao76_x;                            // Atualiza a posição em x e y
    y = posicao76_y;
    }
    break;
    case 75:
    {
    float posicao75_x = -4*esp;
    float posicao75_y =esp;

    if ((posicao75_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao75_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao75_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao75_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao75_x;                            // Atualiza a posição em x e y
    y = posicao75_y;
    }
    break;
    case 74:
    {
    float posicao74_x = -3*esp;
    float posicao74_y = esp;

    if ((posicao74_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao74_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao74_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao74_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao74_x;                            // Atualiza a posição em x e y
    y = posicao74_y;
    }
    break;
    case 73:
    {
    float posicao73_x = -2*esp;
    float posicao73_y = esp;

    if ((posicao73_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao73_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao73_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao73_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao73_x;                            // Atualiza a posição em x e y
    y = posicao73_y;
    }
    break;
    case 72:
    {
    float posicao72_x = -esp;
    float posicao72_y = esp;

    if ((posicao72_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao72_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao72_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao72_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao72_x;                            // Atualiza a posição em x e y
    y = posicao72_y;
    }
    break;
    case 71:
    {
    float posicao71_x = 0;
    float posicao71_y = esp;

    if ((posicao71_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao71_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao71_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao71_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao71_x;                            // Atualiza a posição em x e y
    y = posicao71_y;
    }
    break;
    case 70:
    {
    float posicao70_x = esp;
    float posicao70_y = esp;

    if ((posicao70_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao70_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao70_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao70_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao70_x;                            // Atualiza a posição em x e y
    y = posicao70_y;
    }
    break;
    case 69:
    {
    float posicao69_x = 2*esp;
    float posicao69_y = esp;

    if ((posicao69_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao69_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao69_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao69_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao69_x;                            // Atualiza a posição em x e y
    y = posicao69_y;
    }
    break;
    case 68:
    {
    float posicao68_x = 3*esp;
    float posicao68_y = esp;

    if ((posicao68_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao68_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao68_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao68_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao68_x;                            // Atualiza a posição em x e y
    y = posicao68_y;
    }
    break;
    case 67:
    {
    float posicao67_x = 4*esp;
    float posicao67_y = esp;

    if ((posicao67_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao67_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao67_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao67_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao67_x;                            // Atualiza a posição em x e y
    y = posicao67_y;
    }
    break;
    case 66:
    {
    float posicao66_x = 5*esp;
    float posicao66_y = esp;

    if ((posicao66_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao66_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao66_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao66_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao66_x;                            // Atualiza a posição em x e y
    y = posicao66_y;
    }
    break;
    case 65:
    {
    float posicao65_x = -5*esp;
    float posicao65_y = 0;

    if ((posicao65_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao65_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao65_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao65_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao65_x;                            // Atualiza a posição em x e y
    y = posicao65_y;
    }
    break;
    case 64:
    {
    float posicao64_x = -4*esp;
    float posicao64_y = 0;

    if ((posicao64_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao64_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao64_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao64_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao64_x;                            // Atualiza a posição em x e y
    y = posicao64_y;
    }
    break;
    case 63:
    {
    float posicao63_x = -3*esp;
    float posicao63_y = 0;

    if ((posicao63_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao63_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao63_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao63_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao63_x;                            // Atualiza a posição em x e y
    y = posicao63_y;
    }
    break;
    case 62:
    {
    float posicao62_x = -2*esp;
    float posicao62_y = 0;

    if ((posicao62_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao62_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao62_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao62_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao62_x;                            // Atualiza a posição em x e y
    y = posicao62_y;
    }
    break;
    case 61:
    {
    float posicao61_x = -esp;
    float posicao61_y=0;

    if ((posicao61_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao61_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao61_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao61_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao61_x;                            // Atualiza a posição em x e y
    y = posicao61_y;
    }
    break;
    case 60:
    {
    float posicao60_x=0;                            // Define a posição em x e y para a célula em questão
    float posicao60_y=0;
    
    if ((posicao60_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao60_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao60_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao60_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao60_x;                            // Atualiza a posição em x e y
    y = posicao60_y;
    }
    break;
    
    case 59:
    {
    float posicao59_x= esp;                         // Define a posição em x e y para a célula em questão
    float posicao59_y=0;
    
    if ((posicao59_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao59_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao59_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao59_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao59_x;                            // Atualiza a posição em x e y
    y = posicao59_y;
    }
    break;
    case 58:
    {
    float posicao58_x= 2*esp;                         // Define a posição em x e y para a célula em questão
    float posicao58_y=0;
    
    if ((posicao58_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao58_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao58_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao58_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao58_x;                            // Atualiza a posição em x e y
    y = posicao58_y;
    }
    break;
    case 57:
    {
    float posicao57_x= 3*esp;                         // Define a posição em x e y para a célula em questão
    float posicao57_y=0;

    if ((posicao57_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao57_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao57_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao57_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao57_x;                            // Atualiza a posição em x e y
    y = posicao57_y;
    }
    break;
    case 56:
    {
    float posicao56_x= 4*esp;                         // Define a posição em x e y para a célula em questão
    float posicao56_y=0;

    if ((posicao56_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao56_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao56_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao56_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao56_x;                            // Atualiza a posição em x e y
    y = posicao56_y;
    }
    break;
    case 55:
    {
    float posicao55_x= 5*esp;                         // Define a posição em x e y para a célula em questão
    float posicao55_y=0;

    if ((posicao55_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao55_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao55_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao55_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao55_x;                            // Atualiza a posição em x e y
    y = posicao55_y;
    }
    break;
    case 54:
    {
    float posicao54_x= -5*esp;                         // Define a posição em x e y para a célula em questão
    float posicao54_y= -esp;

    if ((posicao54_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao54_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao54_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao54_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao54_x;                            // Atualiza a posição em x e y
    y = posicao54_y;
    }
    break;
    case 53:
    {
    float posicao53_x= -4*esp;                         // Define a posição em x e y para a célula em questão
    float posicao53_y= -esp;

    if ((posicao53_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao53_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao53_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao53_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao53_x;                            // Atualiza a posição em x e y
    y = posicao53_y;
    }
    break;
    case 52:
    {
    float posicao52_x= -3*esp;                         // Define a posição em x e y para a célula em questão
    float posicao52_y= -esp;

    if ((posicao52_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao52_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao52_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao52_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao52_x;                            // Atualiza a posição em x e y
    y = posicao52_y;
    }
    break;
    case 51:
    {
    float posicao51_x= -2*esp;                         // Define a posição em x e y para a célula em questão
    float posicao51_y= -esp;

    if ((posicao51_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao51_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao51_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao51_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao51_x;                            // Atualiza a posição em x e y
    y = posicao51_y;
    }
    break;
    case 50:
    {
    float posicao50_x= -esp;                         // Define a posição em x e y para a célula em questão
    float posicao50_y= -esp;

    if ((posicao50_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao50_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao50_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao50_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao50_x;                            // Atualiza a posição em x e y
    y = posicao50_y;
    }
    break;
    case 49:
    {
    float posicao49_x= 0;                         // Define a posição em x e y para a célula em questão
    float posicao49_y= -esp;

    if ((posicao49_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao49_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao49_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao49_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao49_x;                            // Atualiza a posição em x e y
    y = posicao49_y;
    }
    break;
    case 48:
    {
    float posicao48_x= esp;                         // Define a posição em x e y para a célula em questão
    float posicao48_y= -esp;

    if ((posicao48_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao48_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao48_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao48_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao48_x;                            // Atualiza a posição em x e y
    y = posicao48_y;
    }
    break;
    case 47:
    {
    float posicao47_x= 2*esp;                         // Define a posição em x e y para a célula em questão
    float posicao47_y= -esp;

    if ((posicao47_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao47_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao47_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao47_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao47_x;                            // Atualiza a posição em x e y
    y = posicao47_y;
    }
    break;
    case 46:
    {
    float posicao46_x= 3*esp;                         // Define a posição em x e y para a célula em questão
    float posicao46_y= -esp;

    if ((posicao46_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao46_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao46_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao46_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao46_x;                            // Atualiza a posição em x e y
    y = posicao46_y;
    }
    break;
    case 45:
    {
    float posicao45_x= 4*esp;                         // Define a posição em x e y para a célula em questão
    float posicao45_y= -esp;

    if ((posicao45_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao45_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao45_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao45_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao45_x;                            // Atualiza a posição em x e y
    y = posicao45_y;
    }
    break;
    case 44:
    {
    float posicao44_x= 5*esp;                         // Define a posição em x e y para a célula em questão
    float posicao44_y= -esp;

    if ((posicao44_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao44_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao44_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao44_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao44_x;                            // Atualiza a posição em x e y
    y = posicao44_y;
    }
    break;
    case 43:
    {
    float posicao43_x= -5*esp;                         // Define a posição em x e y para a célula em questão
    float posicao43_y= -2*esp;

    if ((posicao43_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao43_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao43_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao43_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao43_x;                            // Atualiza a posição em x e y
    y = posicao43_y;
    }
    break;
    case 42:
    {
    float posicao42_x= -4*esp;                         // Define a posição em x e y para a célula em questão
    float posicao42_y= -2*esp;

    if ((posicao42_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao42_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao42_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao42_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao42_x;                            // Atualiza a posição em x e y
    y = posicao42_y;
    }
    break;
    case 41:
    {
    float posicao41_x= -3*esp;                         // Define a posição em x e y para a célula em questão
    float posicao41_y= -2*esp;

    if ((posicao41_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao41_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao41_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao41_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao41_x;                            // Atualiza a posição em x e y
    y = posicao41_y;
    }
    break;
    case 40:
    {
    float posicao40_x= -2*esp;                         // Define a posição em x e y para a célula em questão
    float posicao40_y= -2*esp;

    if ((posicao40_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao40_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao40_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao40_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao40_x;                            // Atualiza a posição em x e y
    y = posicao40_y;
    }
    break;
    case 39:
    {
    float posicao39_x= -esp;                         // Define a posição em x e y para a célula em questão
    float posicao39_y= -2*esp;

    if ((posicao39_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao39_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao39_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao39_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao39_x;                            // Atualiza a posição em x e y
    y = posicao39_y;
    }
    break;
    case 38:
    {
    float posicao38_x= 0;                         // Define a posição em x e y para a célula em questão
    float posicao38_y= -2*esp;

    if ((posicao38_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao38_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao38_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao38_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao38_x;                            // Atualiza a posição em x e y
    y = posicao38_y;
    }
    break;
    case 37:
    {
    float posicao37_x= esp;                         // Define a posição em x e y para a célula em questão
    float posicao37_y= -2*esp;

    if ((posicao37_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao37_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao37_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao37_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao37_x;                            // Atualiza a posição em x e y
    y = posicao37_y;
    }
    break;
    case 36:
    {
    float posicao36_x= 2*esp;                         // Define a posição em x e y para a célula em questão
    float posicao36_y= -2*esp;

    if ((posicao36_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao36_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao36_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao36_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao36_x;                            // Atualiza a posição em x e y
    y = posicao36_y;
    }
    break;
    case 35:
    {
    float posicao35_x= 3*esp;                         // Define a posição em x e y para a célula em questão
    float posicao35_y= -2*esp;

    if ((posicao35_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao35_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao35_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao35_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao35_x;                            // Atualiza a posição em x e y
    y = posicao35_y;
    }
    break;
    case 34:
    {
    float posicao34_x= 4*esp;                         // Define a posição em x e y para a célula em questão
    float posicao34_y= -2*esp;

    if ((posicao34_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao34_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao34_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao34_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao34_x;                            // Atualiza a posição em x e y
    y = posicao34_y;
    }
    break;
    case 33:
    {
    float posicao33_x= 5*esp;                         // Define a posição em x e y para a célula em questão
    float posicao33_y= -2*esp;

    if ((posicao33_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao33_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao33_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao33_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao33_x;                            // Atualiza a posição em x e y
    y = posicao33_y;
    }
    break;
    case 32:
    {
    float posicao32_x= -5*esp;                         // Define a posição em x e y para a célula em questão
    float posicao32_y= -3*esp;

    if ((posicao32_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao32_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao32_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao32_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao32_x;                            // Atualiza a posição em x e y
    y = posicao32_y;
    }
    break;
    case 31:
    {
    float posicao31_x= -4*esp;                         // Define a posição em x e y para a célula em questão
    float posicao31_y= -3*esp;

    if ((posicao31_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao31_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao31_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao31_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao31_x;                            // Atualiza a posição em x e y
    y = posicao31_y;
    }
    break;
    case 30:
    {
    float posicao30_x= -3*esp;                         // Define a posição em x e y para a célula em questão
    float posicao30_y= -3*esp;

    if ((posicao30_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao30_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao30_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao30_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao30_x;                            // Atualiza a posição em x e y
    y = posicao30_y;
    }
    break;
    case 29:
    {
    float posicao29_x= -2*esp;                         // Define a posição em x e y para a célula em questão
    float posicao29_y= -3*esp;

    if ((posicao29_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao29_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao29_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao29_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao29_x;                            // Atualiza a posição em x e y
    y = posicao29_y;
    }
    break;
    case 28:
    {
    float posicao28_x= -esp;                         // Define a posição em x e y para a célula em questão
    float posicao28_y= -3*esp;

    if ((posicao28_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao28_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao28_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao28_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao28_x;                            // Atualiza a posição em x e y
    y = posicao28_y;
    }
    break;
    case 27:
    {
    float posicao27_x= 0;                         // Define a posição em x e y para a célula em questão
    float posicao27_y= -3*esp;

    if ((posicao27_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao27_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao27_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao27_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao27_x;                            // Atualiza a posição em x e y
    y = posicao27_y;
    }
    break;
    case 26:
    {
    float posicao26_x= esp;                         // Define a posição em x e y para a célula em questão
    float posicao26_y= -3*esp;

    if ((posicao26_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao26_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao26_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao26_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao26_x;                            // Atualiza a posição em x e y
    y = posicao26_y;
    }
    break;
    case 25:
    {
    float posicao25_x= 2*esp;                         // Define a posição em x e y para a célula em questão
    float posicao25_y= -3*esp;

    if ((posicao25_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao25_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao25_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao25_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao25_x;                            // Atualiza a posição em x e y
    y = posicao25_y;
    }
    break;
    case 24:
    {
    float posicao24_x= 3*esp;                         // Define a posição em x e y para a célula em questão
    float posicao24_y= -3*esp;

    if ((posicao24_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao24_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao24_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao24_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao24_x;                            // Atualiza a posição em x e y
    y = posicao24_y;
    }
    break;
    case 23:
    {
    float posicao23_x= 4*esp;                         // Define a posição em x e y para a célula em questão
    float posicao23_y= -3*esp;

    if ((posicao23_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao23_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao23_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao23_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao23_x;                            // Atualiza a posição em x e y
    y = posicao23_y;
    }
    break;
    case 22:
    {
    float posicao22_x= 5*esp;                         // Define a posição em x e y para a célula em questão
    float posicao22_y= -3*esp;

    if ((posicao22_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao22_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao22_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao22_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao22_x;                            // Atualiza a posição em x e y
    y = posicao22_y;
    }
    break;
    case 21:
    {
    float posicao21_x= -5*esp;                         // Define a posição em x e y para a célula em questão
    float posicao21_y= -4*esp;

    if ((posicao21_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao21_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao21_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao21_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao21_x;                            // Atualiza a posição em x e y
    y = posicao21_y;
    }
    break;
    case 20:
    {
    float posicao20_x= -4*esp;                         // Define a posição em x e y para a célula em questão
    float posicao20_y= -4*esp;

    if ((posicao20_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao20_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao20_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao20_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao20_x;                            // Atualiza a posição em x e y
    y = posicao20_y;
    }
    break;
    case 19:
    {
    float posicao19_x= -3*esp;                         // Define a posição em x e y para a célula em questão
    float posicao19_y= -4*esp;

    if ((posicao19_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao19_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao19_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao19_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao19_x;                            // Atualiza a posição em x e y
    y = posicao19_y;
    }
    break;
    case 18:
    {
    float posicao18_x= -2*esp;                         // Define a posição em x e y para a célula em questão
    float posicao18_y= -4*esp;

    if ((posicao18_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao18_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao18_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao18_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao18_x;                            // Atualiza a posição em x e y
    y = posicao18_y;
    }
    break;
    case 17:
    {
    float posicao17_x= -esp;                         // Define a posição em x e y para a célula em questão
    float posicao17_y= -4*esp;

    if ((posicao17_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao17_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao17_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao17_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao17_x;                            // Atualiza a posição em x e y
    y = posicao17_y;
    }
    break;
    case 16:
    {
    float posicao16_x= 0;                         // Define a posição em x e y para a célula em questão
    float posicao16_y= -4*esp;

    if ((posicao16_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao16_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao16_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao16_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao16_x;                            // Atualiza a posição em x e y
    y = posicao16_y;
    }
    break;
    case 15:
    {
    float posicao15_x= esp;                         // Define a posição em x e y para a célula em questão
    float posicao15_y= -4*esp;

    if ((posicao15_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao15_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao15_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao15_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao15_x;                            // Atualiza a posição em x e y
    y = posicao15_y;
    }
    break;
    case 14:
    {
    float posicao14_x= 2*esp;                         // Define a posição em x e y para a célula em questão
    float posicao14_y= -4*esp;

    if ((posicao14_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao14_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao14_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao14_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao14_x;                            // Atualiza a posição em x e y
    y = posicao14_y;
    }
    break;
    case 13:
    {
    float posicao13_x= 3*esp;                         // Define a posição em x e y para a célula em questão
    float posicao13_y= -4*esp;

    if ((posicao13_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao13_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao13_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao13_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao13_x;                            // Atualiza a posição em x e y
    y = posicao13_y;
    }
    break;
    case 12:
    {
    float posicao12_x= 4*esp;                         // Define a posição em x e y para a célula em questão
    float posicao12_y= -4*esp;

    if ((posicao12_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao12_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao12_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao12_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao12_x;                            // Atualiza a posição em x e y
    y = posicao12_y;
    }
    break;
    case 11:
    {
    float posicao11_x= 5*esp;                         // Define a posição em x e y para a célula em questão
    float posicao11_y= -4*esp;

    if ((posicao11_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao11_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao11_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao11_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao11_x;                            // Atualiza a posição em x e y
    y = posicao11_y;
    }
    break;
    case 10:
    {
    float posicao10_x= -5*esp;                         // Define a posição em x e y para a célula em questão
    float posicao10_y= -5*esp;

    if ((posicao10_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao10_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao10_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao10_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao10_x;                            // Atualiza a posição em x e y
    y = posicao10_y;
    }
    break;
    case 9:
    {
    float posicao9_x= -4*esp;                         // Define a posição em x e y para a célula em questão
    float posicao9_y= -5*esp;

    if ((posicao9_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao9_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao9_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao9_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao9_x;                            // Atualiza a posição em x e y
    y = posicao9_y;
    }
    break;
    case 8:
    {
    float posicao8_x= -3*esp;                         // Define a posição em x e y para a célula em questão
    float posicao8_y= -5*esp;

    if ((posicao8_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao8_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao8_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao8_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao8_x;                            // Atualiza a posição em x e y
    y = posicao8_y;
    }
    break;
    case 7:
    {
    float posicao7_x= -2*esp;                         // Define a posição em x e y para a célula em questão
    float posicao7_y= -5*esp;

    if ((posicao7_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao7_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao7_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao7_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao7_x;                            // Atualiza a posição em x e y
    y = posicao7_y;
    }
    break;
    case 6:
    {
    float posicao6_x= -esp;                         // Define a posição em x e y para a célula em questão
    float posicao6_y= -5*esp;

    if ((posicao6_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao6_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao6_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao6_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao6_x;                            // Atualiza a posição em x e y
    y = posicao6_y;
    }
    break;
    case 5:
    {
    float posicao5_x= 0;                         // Define a posição em x e y para a célula em questão
    float posicao5_y= -5*esp;

    if ((posicao5_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao5_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao5_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao5_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao5_x;                            // Atualiza a posição em x e y
    y = posicao5_y;
    }
    break;
    case 4:
    {
    float posicao4_x= esp;                         // Define a posição em x e y para a célula em questão
    float posicao4_y= -5*esp;

    if ((posicao4_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao4_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao4_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao4_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao4_x;                            // Atualiza a posição em x e y
    y = posicao4_y;
    }
    break;
    case 3:
    {
    float posicao3_x= 2*esp;                         // Define a posição em x e y para a célula em questão
    float posicao3_y= -5*esp;

    if ((posicao3_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao3_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao3_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao3_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao3_x;                            // Atualiza a posição em x e y
    y = posicao3_y;
    }
    break;
    case 2:
    {
    float posicao2_x= 3*esp;                         // Define a posição em x e y para a célula em questão
    float posicao2_y= -5*esp;

    if ((posicao2_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao2_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao2_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao2_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao2_x;                            // Atualiza a posição em x e y
    y = posicao2_y;
    }
    break;
    case 1:
    {
    float posicao1_x= 4*esp;                         // Define a posição em x e y para a célula em questão
    float posicao1_y= -5*esp;

    if ((posicao1_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao1_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao1_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao1_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao1_x;                            // Atualiza a posição em x e y
    y = posicao1_y;
    }
    break;
    case 0:
    {
    float posicao0_x= 5*esp;                         // Define a posição em x e y para a célula em questão
    float posicao0_y= -5*esp;

    if ((posicao0_x - x) < 0){                     // Verifica se o movimento é no sentido +x ou -x (CHECAR NA MONTAGEM EXPERIMENTAL)
      digitalWrite(dirPinX, HIGH);
    }
    else{
      digitalWrite(dirPinX, LOW);
    }
    
    for(int i=0; i < abs(posicao0_x - x)/0.011; i++){    // Move, em x, o necessário para chegar na posição pré definida da célula
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(500);
    }
    
    if ((posicao0_y - y) < 0){                     // Repete o processo para y
      digitalWrite(dirPinY, HIGH);
    }
    else{
      digitalWrite(dirPinY, LOW);
    }
    
    for(int i=0; i < abs(posicao0_y - y)/0.011; i++){
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(500);
    }
    x = posicao0_x;                            // Atualiza a posição em x e y
    y = posicao0_y;
    }
    break;
  }
}
