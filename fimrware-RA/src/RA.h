#ifndef RA_H
#define RA_H
#include <math.h> 


class RA{

public: 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
RA();

#define angMinGraus 35          //angulo minimo que o reflectometro suporta
#define angMaxGraus 70          //angulo maximo que o reflectometro suporta 

#define nPossible 35000      //numero de posições possíveis para fotodetector
#define nTheta 35000      //numero de posições possíveis para fotodetector
#define dL 400       //amplitude máxima de movimento (mm)
#define dtheta 35   //amplitude máxima de rotação (graus)
#define homePinW 2  //pino sensor de fim de curso eixo W 


//eixo W
#define delayPassoW 800 //delay para pulso no pino step do drive W
#define stepPinW  12      //define pino de step para drive W
#define dirPinW   13         //define pino dir para drive W 
#define enablew   33
#define kW        0.00446       //Define valor constante mm/passo motor W

//Eixo X
#define delayPassoX 800 //delay para pulso no pino step do drive X
#define stepPinX  16      //define pino de step para drive X
#define dirPinX   14         //define pino dir para drive X
#define enablex   26
#define kX        0.00525       //Define valor constante mm/passo motor X

//Eixo Y
#define delayPassoY 800 //delay para pulso no pino step do drive y
#define stepPinY  2      //define pino de step para drive Y
#define dirPinY   15         //define pino dir para drive Y 
#define enabley   27
#define kY        0.00446       //Define valor constante mm/passo motor Y

//Eixo Theta
#define delayPassoT 1000 //delay para pulso no pino step do drive THETA
#define stepPinT  5      //define pino de step para drive THETA
#define dirPinT   17         //define pino dir para drive THETA 
#define enableT   25
#define kT        0.000223     //Define valor constante mm/passo motor THETA

//float vecW[nPossible][2];  //vetor que armazena valores de distância W em função de ângulo theta

void Begin();
void processFunction(String functionName, String argument);
void EsperaMensagem();


//translação

// eixo W
void IrHomeW();       //função que faz fotodetector ir para origem do eixo W
void passoW();        //dá apenas 1 passo no motor de passo 
void mover1mmW(); 
void mover10mmW();     //move distancia especifica, considerando direçao configurada
void mover100mmW();
void moverDistanciaW(float);  //move motor de passo W a uma distancia d em mm
void ConfigParaCimaW(); //função que configura motor de passo fazer fotodetector subir
void avisaMovimentoInsuficienteW(float);
void ConfigParabaixoW(); //função que configura motor de passo fazer fotodetector descer
void MoverUmPontoW();    //função que faz fotodetector mover um ponto, de acordo com a direção dada, número de pontos e amplitude de movimento
int ProcuraPontoW();    //percorre todos os n pontos para encontrar o valor de W que obteve maior valor de intensidade do fotodetector para um determinado valor de theta
void ConstroiArrayW();


// eixo Y
void IrHomeY();       //função que faz fotodetector ir para origem do eixo y
void passoY();        //dá apenas 1 passo no motor de passo 
void mover1mmY(); 
void mover10mmY();     //move distancia especifica, considerando direçao configurada
void mover100mmY();
void moverDistanciaY(float);  //move motor de passo Y a uma distancia d em mm
void ConfigParaCimaY(); //função que configura motor de passo fazer fotodetector subir
void avisaMovimentoInsuficienteY(float);
void ConfigParabaixoY(); //função que configura motor de passo fazer fotodetector descer

//eixo x
void IrHomeX();       //função que faz fotodetector ir para origem do eixo X
void passoX();        //dá apenas 1 passo no motor de passo 
void mover1mmX(); 
void mover10mmX();     //move distancia especifica, considerando direçao configurada
void mover100mmX();
void moverDistanciaX(float);  //move motor de passo X a uma distancia d em mm
void ConfigParaCimaX(); //função que configura motor de passo fazer fotodetector subir
void avisaMovimentoInsuficienteX(float);
void ConfigParabaixoX(); //função que configura motor de passo fazer fotodetector descer


//rotação


// eixo W
void IrHomeT();       //função que faz fotodetector ir para origem do eixo W
void passoT();        //dá apenas 1 passo no motor de passo 
void mover1GT(); 
void mover10GT();     //move distancia especifica, considerando direçao configurada
void mover100GT();
void moverDistanciaT(float);  //move motor de passo W a uma distancia d em mm
void ConfigParaCimaT(); //função que configura motor de passo fazer fotodetector subir
void avisaMovimentoInsuficienteT(float);
void ConfigParabaixoT(); //função que configura motor de passo fazer fotodetector descer


//aquisição
void adquirir();
void rotacionar (float theta);
void ir_para(int celula);
void irParaAngulo(float thetaAtual);
float reflectancia();
};


#endif 
