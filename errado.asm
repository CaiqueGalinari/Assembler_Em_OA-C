# Colocando valor base na memória
# O conjunto de instruções não possui addi ou sw.
# Presume-se que o valor 7 já está no endereço de memória 4.

ori x1, x0, 7      # Carrega a palavra (valor 7) do endereço de memória 4 em x1
sub x2, x1, x0    # x2 = x1 - x0 -> x2 = 7 - 0 -> x2 = 7

# (sub é usado no lugar de add, subtraindo de zero para obter o mesmo valor)

sub x1, x1, x2    # x1 = x1 - x2
sub x1, x1, x2    # x1 = x1 - x2
sub x1, x1, x2
sub x1, x1, x2

beq x1, x2, SAIDA # Se x1 for igual a x2, desvia para SAIDA

# Caso o fluxo venha para cá, seu processador está errado
sub x1, x1, x1    # Usado em vez de 'add' para zerar x1 (x1 = x1 - x1)
sb x1, 0(x0)      # Armazena o byte de x1 no endereço de memória 0

SAIDA:
and x1, x1, x2    # Operação 'and' entre x1 e x2, resultado em x1
# A instrução 'or' é substituída por 'ori' (OR imediato)
ori x1, x1, 0     # or x1, x1, x0 -> ori x1, x1, 0 (resultado é o mesmo, já que x0 é 0)
sb x1, 0(x0)      # Armazena o byte de x1 no endereço de memória 0