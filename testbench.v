// Salve este código como testbench.v
`timescale 1ns / 1ps

module testbench;

    // Sinais para controlar o processador
    reg clk;
    reg rst;

    // Instancia o seu processador que está no arquivo a.v
    // 'dut' significa "Design Under Test" (o seu processador)
    risc_v_processor dut (
        .clk(clk),
        .rst(rst)
    );

    // Gera o sinal de clock (liga e desliga a cada 5ns)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Tarefa para imprimir o estado final dos registradores de forma organizada
    task print_final_state;
        integer i;
        begin
            $display("\n=======================================================");
            $display("||           ESTADO FINAL DOS REGISTRADORES          ||");
            $display("=======================================================");
            // Itera por todos os 32 registradores e os imprime
            for (i = 0; i < 32; i = i + 1) begin
                // A linha abaixo "espia" dentro do seu processador para ler os registradores.
                // dut.u_regfile.registers[i] acessa o registrador 'i'
                $display("x%02d:\t%d", i, dut.u_regfile.registers[i]);
            end
            $display("=======================================================");
        end
    endtask


    // Bloco principal que controla a simulação
    initial begin
        // 1. Começa com o processador em reset
        rst = 1;
        #20; // Espera 20ns

        // 2. Libera o reset para o programa começar
        rst = 0;
        $display(">>> INICIO DA SIMULACAO <<<");
        $display("Reset liberado. Processador executando instrucoes...");

        // 3. MONITORAMENTO DURANTE A EXECUÇÃO
        // O comando $monitor imprime a linha toda vez que um dos valores mudar.
        // Ele vai mostrar qual instrução está sendo executada e o registrador que foi escrito.
        // dut.u_ctrl.reg_write == 1 -> Condição para escrever
        // dut.instruction[11:7]   -> Registrador de destino (rd)
        // dut.write_back_data     -> Valor que está sendo escrito
        
        // SUBSTITUA A LINHA DO $monitor PELA LINHA ABAIXO
// NO ARQUIVO testbench.v

        $monitor("Registrador:\t[x%0d]\t%d",
                 dut.instruction[11:7], dut.write_back_data);


        // 4. Deixa a simulação rodar por um tempo
        // Ajuste este valor se seu programa precisar de mais tempo. 200ns = 20 ciclos.
        #200;

        // 5. IMPRIME O ESTADO FINAL
        // Desliga o monitor para não poluir a saída final
        $monitoroff;
        print_final_state; // Chama a tarefa que criamos acima

        // 6. Termina a simulação
        $display("\n>>> FIM DA SIMULACAO <<<");
        $finish;
    end

endmodule