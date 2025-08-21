`timescale 1ns / 1ps

module testbench;

    // Sinais para controlar o processador
    reg clk;
    reg rst;

    // Instancia o seu processador
    risc_v_processor dut (
        .clk(clk),
        .rst(rst)
    );

    // Gera o sinal de clock
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Declaração da variável que vai guardar o nome da instrução.
    // O tamanho [8*7-1:0] permite strings de até 7 caracteres (ex: "SRLI").
    reg [8*7-1:0] instruction_mnemonic;

    // Lógica do Decodificador: sempre ativa quando a instrução muda.
    always @(*) begin
        // Valor padrão para instruções não reconhecidas.
        instruction_mnemonic = "???";

        // Decodifica a instrução com base no opcode (bits 6 a 0).
        case (dut.instruction[6:0])

            // Instruções Tipo-R (operações entre registradores)
            7'b0110011:
                case (dut.instruction[14:12]) // Diferencia pelo funct3
                    3'b000: instruction_mnemonic = (dut.instruction[30]) ? "SUB" : "ADD"; // Verifica o funct7 para ADD/SUB
                    3'b101: instruction_mnemonic = (dut.instruction[30]) ? "SRA" : "SRL";
                    3'b001: instruction_mnemonic = "SLL";
                    3'b100: instruction_mnemonic = "XOR";
                    3'b110: instruction_mnemonic = "OR";
                    3'b111: instruction_mnemonic = "AND";
                    default: instruction_mnemonic = "R-Type?";
                endcase

            // Instruções Tipo-I (operações com valor imediato)
            7'b0010011:
                case (dut.instruction[14:12]) // Diferencia pelo funct3
                    3'b000: instruction_mnemonic = "ADDI";
                    3'b001: instruction_mnemonic = "SLLI";
                    3'b101: instruction_mnemonic = (dut.instruction[30]) ? "SRAI" : "SRLI";
                    3'b110: instruction_mnemonic = "ORI";
                    3'b111: instruction_mnemonic = "ANDI";
                    default: instruction_mnemonic = "I-Type?";
                endcase

            // Instruções de Load (carregar da memória)
            7'b0000011:
                case (dut.instruction[14:12]) // Diferencia pelo funct3
                    3'b000: instruction_mnemonic = "LB";   // Load Byte
                    3'b001: instruction_mnemonic = "LH";   // Load Half-word
                    3'b010: instruction_mnemonic = "LW";   // Load Word
                    default: instruction_mnemonic = "LOAD?";
                endcase

            // Instruções de Store (salvar na memória)
            7'b0100011:
                case (dut.instruction[14:12]) // Diferencia pelo funct3
                    3'b000: instruction_mnemonic = "SB";   
                    3'b001: instruction_mnemonic = "SH";   
                    3'b010: instruction_mnemonic = "SW";   
                    default: instruction_mnemonic = "STORE?";
                endcase

            // Instruções de Branch (desvio condicional)
            7'b1100011:
                case (dut.instruction[14:12]) // Diferencia pelo funct3
                    3'b000: instruction_mnemonic = "BEQ";  
                    3'b001: instruction_mnemonic = "BNE";  
                    3'b100: instruction_mnemonic = "BLT";  
                    3'b101: instruction_mnemonic = "BGE";  
                    3'b110: instruction_mnemonic = "BLTU"; 
                    3'b111: instruction_mnemonic = "BGEU"; 
                    default: instruction_mnemonic = "BRANCH?";
                endcase

            // Caso o opcode não corresponda a nenhum dos acima
            default:
                instruction_mnemonic = "???";
        endcase
    end

    // Bloco principal que controla a simulação
    initial begin
        rst = 1;
        #20;
        rst = 0;
        $display("--- INICIO DA SIMULACAO ---");

        // Roda a simulação por 1000ns
        #1000;

        print_final_state;
        $display("--- FIM DA SIMULACAO ---");
        $finish;
    end

    always @(posedge clk) begin
        // Apenas imprime após o reset e enquanto houver uma instrução válida
        if (!rst && dut.instruction != 0) begin
            // Caso 1: A instrução escreve em um registrador (ADD, LW, ORI, etc.)
            if (dut.u_ctrl.reg_write) begin
                $display("PC: %d | %s \t-> Registrador x%0d recebe o valor %d (0x%h)",
                         dut.pc_current, instruction_mnemonic, dut.instruction[11:7], dut.write_back_data, dut.write_back_data);
            // Caso 2: A instrução é um desvio condicional (BEQ, BEQ)
            end else if (dut.u_ctrl.branch) begin
                 $display("PC: %d | %s \t-> (Comparando registradores, resultado para desvio: %b)",
                           dut.pc_current, instruction_mnemonic, dut.branch_control);
            // Caso 3: A instrução escreve na memória (SW, SB, SH)
            end else if (dut.u_ctrl.mem_write) begin
                 $display("PC: %d | %s \t-> (Salvando o valor %d no endereco de memoria %h)",
                           dut.pc_current, instruction_mnemonic, dut.u_regfile.read_data2, dut.alu_result);
            end
        end
    end

    // Tarefa para imprimir o estado final dos registradores
    task print_final_state;
        integer i;
        begin
            $display("\n=======================================================");
            $display("||           ESTADO FINAL DOS REGISTRADORES          ||");
            $display("=======================================================");
            for (i = 0; i < 32; i = i + 1) begin
                $display("x%02d:\t%d", i, dut.u_regfile.registers[i]);
            end
            $display("=======================================================");
        end
    endtask

endmodule