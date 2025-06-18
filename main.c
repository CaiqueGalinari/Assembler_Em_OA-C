#include <stdio.h>
#include <stdlib.h>
#include "toyota.h"


int main(int argc, char* argv[]) {
 
    if (argc != 3) {
        fprintf(stderr, "Uso: %s <arquivo_de_entrada.asm> <arquivo_de_saida.txt>\n", argv[0]);
        return 1;
    }

   
    FILE* arquivoEntrada = fopen(argv[1], "r");
    if (!arquivoEntrada) {
        perror("Erro ao abrir arquivo de entrada");
        return 1;
    }

  
    primeiraPassagem(arquivoEntrada);

  
    rewind(arquivoEntrada);

   
    FILE* arquivoSaida = fopen(argv[2], "w");
    if (!arquivoSaida) {
        perror("Erro ao abrir arquivo de saída");
        fclose(arquivoEntrada);
        return 1;
    }
    

    segundaPassagem(arquivoEntrada, arquivoSaida);


    fclose(arquivoEntrada);
    fclose(arquivoSaida);

    printf("Montagem concluída. Saída binária (em formato de texto) em: %s\n", argv[2]);
    return 0;
}
