#include <stdio.h>
#include <stdlib.h>
#include "assembler.h" // Inclui apenas o cabeçalho, que "anuncia" as funções e variáveis

// *** NÃO COLOQUE NENHUMA OUTRA FUNÇÃO OU VARIÁVEL GLOBAL AQUI ***

int main(int argc, char* argv[]) {
    // Verifica se os argumentos de linha de comando estão corretos
    if (argc != 3) {
        fprintf(stderr, "Uso: %s <arquivo_de_entrada.asm> <arquivo_de_saida.txt>\n", argv[0]);
        return 1;
    }

    // Tenta abrir o arquivo de entrada para leitura
    FILE* arquivoEntrada = fopen(argv[1], "r");
    if (!arquivoEntrada) {
        perror("Erro ao abrir arquivo de entrada");
        return 1;
    }

    // Chama a primeira passagem para mapear os rótulos (a função está definida em assembler.c)
    primeiraPassagem(arquivoEntrada);

    // Usa rewind para voltar ao início do arquivo de entrada sem precisar fechar e reabrir
    rewind(arquivoEntrada);

    // Abre o arquivo de saída para escrita
    FILE* arquivoSaida = fopen(argv[2], "w");
    if (!arquivoSaida) {
        perror("Erro ao abrir arquivo de saída");
        fclose(arquivoEntrada);
        return 1;
    }
    
    // Chama a segunda passagem para traduzir o código (a função está definida em assembler.c)
    segundaPassagem(arquivoEntrada, arquivoSaida);

    // Fecha os arquivos
    fclose(arquivoEntrada);
    fclose(arquivoSaida);

    // Informa que o processo foi concluído com sucesso
    printf("Montagem concluída. Saída binária (em formato de texto) em: %s\n", argv[2]);
    return 0;
}
