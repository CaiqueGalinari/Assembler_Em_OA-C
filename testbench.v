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

    // =================================================================
    //          Lógica do Decodificador (Corrigida)
    // =================================================================
    reg [8*7-1:0] instruction_mnemonic; // String para guardar o nome da instrução

    always @(*) begin
        // Por padrão, a instrução é desconhecida
        instruction_mnemonic = "???";

        // Decodifica com base no opcode e funct3/funct7
        case (dut.instruction[6:0]) // case no OPCODE
            7'b0110011: // Tipo-R
                case (dut.instruction[14:12]) // case no funct3
                    3'b000: instruction_mnemonic = (dut.instruction[30]) ? "SUB" : "ADD";
                    3'b111: instruction_mnemonic = "AND";
                    default: instruction_mnemonic = "R-Type?";
                endcase
            7'b0010011: // Tipo-I (Imediato)
                case (dut.instruction[14:12]) // case no funct3
                    3'b000: instruction_mnemonic = "ADDI";
                    3'b110: instruction_mnemonic = "ORI";
                    3'b011: instruction_mnemonic = "SLTIU";
                    default: instruction_mnemonic = "I-Type?";
                endcase
            7'b0000011: // LOAD instructions
                case (dut.instruction[14:12]) // case no funct3
                    3'b000: instruction_mnemonic = "LB";  // Load Byte
                    3'b001: instruction_mnemonic = "LH";  // Load Half-word
                    3'b010: instruction_mnemonic = "LW";  // Load Word
                    default: instruction_mnemonic = "LOAD?";
                endcase
            7'b0100011: // STORE instructions
                case (dut.instruction[14:12]) // case no funct3
                    3'b000: instruction_mnemonic = "SB";  // Store Byte
                    3'b001: instruction_mnemonic = "SH";  // Store Half-word
                    3'b010: instruction_mnemonic = "SW";  // Store Word
                    default: instruction_mnemonic = "STORE?";
                endcase
            7'b1100011: instruction_mnemonic = "BEQ";  // Branch if Equal
            default: instruction_mnemonic = "???";
        endcase
    end

    // Bloco principal que controla a simulação
    initial begin
        rst = 1;
        #20;
        rst = 0;
        $display("--- INICIO DA SIMULACAO ---");

        // Roda a simulação por 200ns
        #200;

        print_final_state;
        $display("--- FIM DA SIMULACAO ---");
        $finish;
    end

    // =================================================================
    //          O Monitor de Saída (Seu Formato Preferido)
    // =================================================================
    always @(posedge clk) begin
        // Apenas imprime após o reset e enquanto houver uma instrução válida
        if (!rst && dut.instruction != 0) begin
            // Caso 1: A instrução escreve em um registrador (ADD, LW, ORI, etc.)
            if (dut.u_ctrl.reg_write) begin
                $display("PC: %d | %s \t-> Registrador x%0d recebe o valor %d (0x%h)",
                         dut.pc_current, instruction_mnemonic, dut.instruction[11:7], dut.write_back_data, dut.write_back_data);
            // Caso 2: A instrução é um desvio condicional (BEQ)
            end else if (dut.u_ctrl.branch) begin
                 $display("PC: %d | %s \t-> (Comparando registradores, resultado para desvio: %b)",
                           dut.pc_current, instruction_mnemonic, dut.alu_zero);
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