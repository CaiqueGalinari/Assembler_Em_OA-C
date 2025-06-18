#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define TAMANHO_MAX_LINHA 256  // Tamanho máximo de uma linha
#define TAMANHO_MAX_ROTULO 50    // Define o tamanho máximo do nome do rotulo
#define MAX_ROTULOS 100        // Define o máximo de rótulos que vão ser usados

//---------------------------------------------------------------------------------------

// Estrutura do rotulo
typedef struct {
    char nome[50];
    int endereco;
} Rotulo;

//---------------------------------------------------------------------------------------

Rotulo tabelaDeRotulos[MAX_ROTULOS]; // Lista de todos os rotulos
int contadorDeRotulos = 0;             // Quantos rotulos tem
int enderecoInstrucaoAtual = 0;      // Tipo um cursor

//---------------------------------------------------------------------------------------
// Tabela de rotulos[x] = novo rotulo
void adicionarRotulo(const char* nome, int endereco) {
    // Se ainda tem espaço...
    if (contadorDeRotulos < MAX_ROTULOS) {
        strcpy(tabelaDeRotulos[contadorDeRotulos].nome, nome);  // Copia o nome do rótulo para a próxima posição livre na tabela.
        tabelaDeRotulos[contadorDeRotulos].endereco = endereco; // Guarda o endereço do rótulo, se não fica só no "vamo combinar" (é uma piada)
        contadorDeRotulos++;
    }
}

//---------------------------------------------------------------------------------------

// procura rotulo pelo nome e retorna o endereco
int obterEnderecoRotulo(const char* nome) {
    //busca sequencial
    for (int i = 0; i < contadorDeRotulos; i++) {

        //compara os nomes, se for igual, retorna o endereco
        if (strcmp(tabelaDeRotulos[i].nome, nome) == 0) {
            return tabelaDeRotulos[i].endereco;
        }
    }
    // Se não achar, da erro
    return -1;
}

//---------------------------------------------------------------------------------------

// xN -> N
int registrador_para_numero(char* str_reg) {
    if (str_reg != NULL && str_reg[0] == 'x') { // se a string nao é nula e começa com x (pq os registradores começam com x)
        return atoi(str_reg + 1);             // atoi faz todo o trabalho pesado :D
    }
    return -1; // Erro
}

//---------------------------------------------------------------------------------------

// int -> binario (string)
void inteiro_para_string_binaria(uint32_t valor, char* saida) {
    saida[32] = '\0'; //bota a string p acabar na posicao 32

    // Loop que vai da direita para a esquerda
    for (int i = 31; i >= 0; i--) {
        //Faz a divis o p converter de decimal p binário
        if ((valor % 2) == 1) {
            saida[i] = '1';
        } else {
            saida[i] = '0';
        }
        // Divide por 2 pro loop não ser infinito (o tanto de 0 que vi na tela por causa disso)
        valor = valor / 2;
    }
}

//---------------------------------------------------------------------------------------

// Lê o arquivo p ir pegango os rótulos primeiro
void primeiraPassagem(FILE* arquivoEntrada) {
    char linha[256]; // Vai armazenar cada linha na memoria temporaria

    // Zera os contadores globais
    enderecoInstrucaoAtual = 0;
    contadorDeRotulos = 0;

    // Loop que lê o arquivo linha por linha até o final.
    while (fgets(linha, sizeof(linha), arquivoEntrada)) {
        char* ptrLinha = linha; //Inicio da linha atual
        char* instrucaoAposRotulo = NULL; // Vai apontar pro inicio de uma instrução (se tiver)
        int temInstrucao = 0; // "Boolean" que diz se tem instrução na linha

        // Procura comentário
        char* comentario = strchr(linha, '#'); //Procura por # na linha
        if (comentario){
            *comentario = '\0'; //troca a # por \0 (parece que alguém será ignorado :)
        } 
        linha[strcspn(linha, "\r\n")] = 0; //Exclui \n ou \r, para não atrapalhar na leitura

        // Procura por ":", porque indica um rotulo
        char* doisPontos = strchr(ptrLinha, ':');
        if (doisPontos != NULL) {
            *doisPontos = '\0'; // Troca ':' por '\0', para isolar o rotulo
            adicionarRotulo(ptrLinha, enderecoInstrucaoAtual); //Adiciona o rotulo que esta na linha e seu endereco
            instrucaoAposRotulo = doisPontos + 1; //Define que a possível instrução começa depois dos ':'
        } else {
            instrucaoAposRotulo = ptrLinha; //Define que a instrução é a linha toda
        }

        // Olha se tem algo além de espaços na instrução
        char temp[10]; // Buffer temporário para ver se tem uma palavra.

        // 'sscanf' tenta ler uma palavra da string. Se conseguir, retorna 1.
        if (instrucaoAposRotulo != NULL && sscanf(instrucaoAposRotulo, " %s", temp) == 1) {
            temInstrucao = 1; // Marca que esta linha tem uma instrução.
        }

        // Se a linha tinha uma instrução
        if (temInstrucao) {
            enderecoInstrucaoAtual += 4; //Incrementa o endereço em 4 bytes
        }
    }
}

//---------------------------------------------------------------------------------------

// Vai ler o arquivo e passar ele para binário
void segundaPassagem(FILE* arquivoEntrada, FILE* arquivoSaida) {
    char linha[256]; // Linha atual
    char str_binaria[33]; // String para armazenar a representação binária da instrução
    enderecoInstrucaoAtual = 0; // Zera o contador do "cursor"

    // Loop que lê o arquivo de entrada linha por linha
    while (fgets(linha, sizeof(linha), arquivoEntrada)) {
        char* comentario = strchr(linha, '#'); //Procura um '#' na linha

        // Se o ponteiro não for null...
        if (comentario){
            *comentario = '\0'; // Troca '#' por '\0', o que faz as funções pularem o comentário
        }
        linha[strcspn(linha, "\r\n")] = 0; //A linha na posição da quebra de linha troca o n(\n) ou o r(\r) por 0(\0)

        char* ptrLinha = linha; //Ponteiro -> inicio da linha
        char* doisPontos = strchr(ptrLinha, ':'); // procura um rotulo
        //Se tem rotulo
        if (doisPontos){
            ptrLinha = doisPontos + 1; // manda o cursor pra depois dele
        }

        char* mnemonico = strtok(ptrLinha, " \t,()"); //Quebra a linha usando \t e () como delimitadores
        if (!mnemonico) {continue;} // Se não tiver instrução, continua para a próxima iteração

        //Variavel do tipo 32bits (é cada coisa que a gente aprende nesse trabalho kkkkkkk)
        uint32_t instrucaoBinaria = 0;

        // Verifica qual a instrução e codifica ela
        if (strcmp(mnemonico, "lb") == 0) { //Se a instrução for lb...
            // Formato esperado: lb rd, imm(rs1)
            char* op_rd = strtok(NULL, " \t,()"); // Pega o rd
            char* op_imm = strtok(NULL, " \t,()"); // Pega o imediato
            char* op_rs1 = strtok(NULL, " \t,()"); // Pega o rs1
            int rd = registrador_para_numero(op_rd); // xN -> N
            int rs1 = registrador_para_numero(op_rs1); // mesma coisa
            int imm = atoi(op_imm); // imm string -> int

            // Monta a instrução binária
            instrucaoBinaria |= 0b0000011;	//Opcode do lb
            instrucaoBinaria |= (rd << 7);
            instrucaoBinaria |= (0b000 << 12);          // Funct3 do lb
            instrucaoBinaria |= (rs1 << 15);
            instrucaoBinaria |= ((imm & 0xFFF) << 20);  // Usa uma máscara p gqrantir que vai pegar só 12 bits do imediato p evitar engraçadinho q vai tentar estourar o limite

        } else if (strcmp(mnemonico, "sb") == 0) { //Se a instrução for sb...
            char* op_rs2 = strtok(NULL, " \t,()");
            char* op_imm = strtok(NULL, " \t,()");
            char* op_rs1 = strtok(NULL, " \t,()");
            int rs1 = registrador_para_numero(op_rs1);
            int rs2 = registrador_para_numero(op_rs2);
            int imm = atoi(op_imm);
            // Divide o imediato em 2 partes
            uint32_t imm_11_5 = (imm >> 5) & 0x7F; // & 0x7F para garantir que vai ter o tamanho correto
            uint32_t imm_4_0 = imm & 0x1F; // & Mesma coisa para o 0x1F

            instrucaoBinaria |= 0b0100011;      // opcode do sb
            instrucaoBinaria |= (imm_4_0 << 7);
            instrucaoBinaria |= (0b000 << 12);  // funct3
            instrucaoBinaria |= (rs1 << 15);
            instrucaoBinaria |= (rs2 << 20);
            instrucaoBinaria |= (imm_11_5 << 25);

        } else if (strcmp(mnemonico, "sub") == 0 || strcmp(mnemonico, "and") == 0 || strcmp(mnemonico, "srl") == 0) {
            char* op_rd = strtok(NULL, " \t,()");
            char* op_rs1 = strtok(NULL, " \t,()");
            char* op_rs2 = strtok(NULL, " \t,()");
            int rd = registrador_para_numero(op_rd);
            int rs1 = registrador_para_numero(op_rs1);
            int rs2 = registrador_para_numero(op_rs2);
            // Define funct7 e funct3 com base na instrução (melhor que criar outros 2 else if)
            uint32_t funct7 = 0, funct3 = 0;
            if (strcmp(mnemonico, "sub") == 0) { funct7 = 0b0100000; funct3 = 0b000; }
            if (strcmp(mnemonico, "and") == 0) { funct7 = 0b0000000; funct3 = 0b111; }
            if (strcmp(mnemonico, "srl") == 0) { funct7 = 0b0000000; funct3 = 0b101; }

            instrucaoBinaria |= 0b0110011; // opcode
            instrucaoBinaria |= (rd << 7);
            instrucaoBinaria |= (funct3 << 12);
            instrucaoBinaria |= (rs1 << 15);
            instrucaoBinaria |= (rs2 << 20);
            instrucaoBinaria |= (funct7 << 25);

        } else if (strcmp(mnemonico, "ori") == 0) {
            char* op_rd = strtok(NULL, " \t,()");
            char* op_rs1 = strtok(NULL, " \t,()");
            char* op_imm = strtok(NULL, " \t,()");
            int rd = registrador_para_numero(op_rd);
            int rs1 = registrador_para_numero(op_rs1);
            int imm = atoi(op_imm);

            instrucaoBinaria |= 0b0010011; // opcode
            instrucaoBinaria |= (rd << 7);
            instrucaoBinaria |= (0b110 << 12); // funct3
            instrucaoBinaria |= (rs1 << 15);
            instrucaoBinaria |= ((imm & 0xFFF) << 20);

        } else if (strcmp(mnemonico, "beq") == 0) {
            char* op_rs1 = strtok(NULL, " \t,()");
            char* op_rs2 = strtok(NULL, " \t,()");
            char* op_label = strtok(NULL, " \t,()");
            int rs1 = registrador_para_numero(op_rs1);
            int rs2 = registrador_para_numero(op_rs2);

            // Procura o endereco do rotulo na tabela
            int enderecoDoRotulo = obterEnderecoRotulo(op_label);
            //Se não der erro...
            if (enderecoDoRotulo != -1) {
                // Calcula o deslocamento.
                int32_t deslocamento = enderecoDoRotulo - enderecoInstrucaoAtual;
                // Quebra o deslocamento nas partes do beq usando máscara p evitar erro (mesmo esquema)
                uint32_t imm12=(deslocamento>>12)&0x1, imm11=(deslocamento>>11)&0x1, imm10_5=(deslocamento>>5)&0x3F, imm4_1=(deslocamento>>1)&0xF;

                instrucaoBinaria |= 0b1100011;         // opcode
                instrucaoBinaria |= (imm11 << 7);
                instrucaoBinaria |= (imm4_1 << 8);
                instrucaoBinaria |= (0b000 << 12);     // funct3
                instrucaoBinaria |= (rs1 << 15);
                instrucaoBinaria |= (rs2 << 20);
                instrucaoBinaria |= (imm10_5 << 25);
                instrucaoBinaria |= (imm12 << 31);
            }
        } else {
            //Se der erro, continua pra próxima linha
            continue;
        }

        // passa de inteiro/32bits para string
        inteiro_para_string_binaria(instrucaoBinaria, str_binaria);
        // Escreve a string no arquivo
        fprintf(arquivoSaida, "%s", str_binaria);
        // Passa para a proxima instrução
        enderecoInstrucaoAtual += 4;
    }
}


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