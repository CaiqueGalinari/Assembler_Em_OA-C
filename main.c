#include <stdio.h>
#include <stdlib.h>
#include "toyota.h"

int main(int argc, char* argv[]) {
    FILE* arquivoEntrada = NULL;
    FILE* arquivoSaida = NULL;

    if (argc == 2) {
        printf("Modo de execução: Lendo de '%s' e imprimindo no terminal.\n\n", argv[1]);
        arquivoSaida = stdout; //Saída vai pro terminal
    
    } else if (argc == 3) {
        printf("Modo de execução: Lendo de '%s' e salvando em '%s'.\n\n", argv[1], argv[2]);
        arquivoSaida = fopen(argv[2], "w");
        if (!arquivoSaida) {
            perror("Erro crítico ao tentar criar o arquivo de saída");
            return 1;
        }

    } else {
        fprintf(stderr, "Erro: Número incorreto de argumentos.\n");
        fprintf(stderr, "Uso 1 (saída no terminal): %s <arquivo_de_entrada.asm>\n", argv[0]);
        fprintf(stderr, "Uso 2 (saída em arquivo):  %s <arquivo_de_entrada.asm> <arquivo_de_saida>\n", argv[0]);
        return 1;
    }

    arquivoEntrada = fopen(argv[1], "r");
    if (!arquivoEntrada) {
        perror("Erro crítico ao tentar ler o arquivo de entrada");
        if (arquivoSaida != stdout) fclose(arquivoSaida);
        return 1;
    }
    
    primeiraPassagem(arquivoEntrada);
    rewind(arquivoEntrada);

    segundaPassagem(arquivoEntrada, arquivoSaida);
    
    fclose(arquivoEntrada);
    if (arquivoSaida != stdout) {
        fclose(arquivoSaida);
        printf("\nMontagem concluída. Saída salva em: %s\n", argv[2]);
    } else {
        printf("\nMontagem concluída.\n");
    }
    return 0;
}