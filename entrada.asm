ori  x5, x0, 1
ori  x6, x0, 2
beq  x5, x6, FIM
sub  x6, x6, x5
FIM: and  x5, x5, x0