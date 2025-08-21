.text
.globl main
main:
    # --- 1. Inicialização de Valores nos Registradores ---
    ori x8, x0, 100
    ori x9, x0, 50
    ori x18, x0, 25

    # --- Armazenamento dos Valores na Memória ---
    sw x8, 0(x3)
    sh x9, 4(x3)
    sb x18, 6(x3)

    # --- Carregamento dos Valores da Memória ---
    lw x19, 0(x3)
    lh x20, 4(x3)
    lb x21, 6(x3)

    # --- Operações Aritméticas, Lógicas e de Deslocamento ---
    add x22, x19, x20
    sub x23, x19, x20
    and x8, x20, x21
    or x9, x20, x21
    xor x18, x19, x20
    ori x19, x21, 240
    andi x20, x19, 15
    slli x21, x21, 2
    srli x22, x22, 1

    bne x23, x20, desvio_bne_ok

    add x8, x8, x0      # <-- Esta instrução é PULADA

desvio_bne_ok:
    beq x23, x9, loop_final

    sub x9, x9, x9      # <-- Esta é a última instrução executada

loop_final:
    # Fim do programa