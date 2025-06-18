#ifndef ASSEMBLER_H
#define ASSEMBLER_H

#include <stdio.h>

// Estrutura do rotulo
typedef struct {
    char nome[50];
    int endereco;
} Rotulo;
//---------------------------------------------------------------------------------------

extern Rotulo tabelaDeRotulos[100];
extern int contadorDeRotulos;
extern int enderecoInstrucaoAtual;

void primeiraPassagem(FILE* arquivoEntrada);
void segundaPassagem(FILE* arquivoEntrada, FILE* arquivoSaida);

#endif
