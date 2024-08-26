#ifndef RA_H
#define RA_H
#include <math.h> 
#include <Arduino.h>

class RA{

public: 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
RA();

#define angMinGraus 35          //angulo minimo que o reflectometro suporta
#define angMaxGraus 70          //angulo maximo que o reflectometro suporta 

#define nPossible 35000      //numero de posições possíveis para fotodetector
#define nTheta 35000      //numero de posições possíveis para fotodetector
#define dL 360       //amplitude máxima de movimento (mm)
#define dtheta 35   //amplitude máxima de rotação (graus)
#define homePinW 2  //pino sensor de fim de curso eixo W 

#define mediaADC 50
#define grauBusca 3
#define passoGrauBusca 50


//eixo W
#define delayPassoW 800 //delay para pulso no pino step do drive W
#define stepPinW  12      //define pino de step para drive W
#define dirPinW   13         //define pino dir para drive W 
#define enableW   33
#define kW        0.00446       //Define valor constante mm/passo motor W

//Eixo X
#define delayPassoX 800 //delay para pulso no pino step do drive X
#define stepPinX  16      //define pino de step para drive X
#define dirPinX   14         //define pino dir para drive X
#define enableX   26
#define kX        0.00525       //Define valor constante mm/passo motor X

//Eixo Y
#define delayPassoY 800 //delay para pulso no pino step do drive y
#define stepPinY  2      //define pino de step para drive Y
#define dirPinY   15         //define pino dir para drive Y 
#define enableY   27
#define kY        0.00446       //Define valor constante mm/passo motor Y

//Eixo Theta
#define delayPassoT 1000 //delay para pulso no pino step do drive THETA
#define stepPinT  5      //define pino de step para drive THETA
#define dirPinT   17         //define pino dir para drive THETA 
#define enableT   25
#define kT        0.000223     //Define valor constante mm/passo motor THETA

#define MS1  21
#define MS2  19
#define MS3  18

#define motorW 0
#define motorT 1
#define motorX 2
#define motorY 3



//float vecW[nPossible][2];  //vetor que armazena valores de distância W em função de ângulo theta

void Begin();
void processFunction(String functionName, String argument1, String argument2, String argument3);
void EsperaMensagem();


//translação

// eixo W
void IrHome(int motor);       //função que faz fotodetector ir para origem do eixo W
void passo(int motor);        //dá apenas 1 passo no motor de passo 
void mover1mm(int motor); 
void mover10mm(int motor);     //move distancia especifica, considerando direçao configurada
void mover100mm(int motor);
void moverDistancia(float d, int motor);  //move motor de passo W a uma distancia d em mm
void ConfigParaCima(int motor); //função que configura motor de passo fazer fotodetector subir
void avisaMovimentoInsuficiente(float movimento, int motor);
void ConfigParabaixo(int motor); //função que configura motor de passo fazer fotodetector descer
void MoverUmPontoW();    //função que faz fotodetector mover um ponto, de acordo com a direção dada, número de pontos e amplitude de movimento
void ConstroiArray();   //percorre todos os n pontos para encontrar o valor de W que obteve maior valor de intensidade do fotodetector para um determinado valor de theta e armazena na flash

//aquisição
void adquirir();
void rotacionar (float theta);
void ir_para(int celula);
void irParaAngulo(float thetaAtual);
float reflectancia();

//armazenamento de dados 
void storeInSPIFFS(const char* path, float angulo, float w);
void readFromSPIFFS(const char* path);
};


#endif 
