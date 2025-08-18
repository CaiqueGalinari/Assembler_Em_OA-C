ori x1, x0, 7     
sub x2, x1, x0    
beq x1, x2, SAIDA 

# Caso o fluxo venha para cá, seu processador está errado
sub x1, x1, x1    
sb x1, 0(x0)      

SAIDA:
and x1, x1, x2   
ori x1, x1, 0     
sb x1, 0(x0)      