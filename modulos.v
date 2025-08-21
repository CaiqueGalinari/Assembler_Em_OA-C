module pc (
    input wire clk,
    input wire rst,
    input wire [31:0] next_pc,
    output reg [31:0] current_pc
);

    always @(posedge clk or posedge rst) begin
        if (rst)
            current_pc <= 32'b0;
        else
            current_pc <= next_pc;
    end
endmodule

// Módulo da Memória de Instruções
module instruction_memory #(
    parameter MEM_DEPTH = 2048 // Defina o tamanho da memória aqui
)(
    input wire [31:0] read_address,
    output reg [31:0] instruction
);
     // A memória usa o MEM_DEPTH para definir seu tamanho
    reg [31:0] rom [0:MEM_DEPTH-1];
    reg [1023:0] nome_do_arquivo; // Suporta nomes de arquivo de até 127 caracteres

    initial begin
        
        // $value$plusargs procura por um argumento na linha de comando
        // Se encontrar, ele armazena o valor da string em 'nome_do_arquivo'
        if ($value$plusargs("%s", nome_do_arquivo)) begin
            $display("Carregando instrucoes do arquivo: %s", nome_do_arquivo);
            $readmemb(nome_do_arquivo, rom);
        end else begin
            // Se o argumento +ARQUIVO não for fornecido, imprime uma mensagem de erro e encerra.
            $display("-----------------------------------------------------------------");
            $display("ERRO: Arquivo de instrucoes nao especificado.");
            $display("Uso correto: vvp <executavel> +<caminho_para_o_arquivo>");
            $display("Exemplo: vvp ford +saida.asm");
            $display("-----------------------------------------------------------------");
            $finish;
        end
    end
  

    always @(*) begin
        instruction = rom[read_address[11:2]];
    end

endmodule

// Módulo do Banco de Registradores
module register_file (
    input wire clk,
    input wire reg_write,
    input wire [4:0] read_reg1,
    input wire [4:0] read_reg2,
    input wire [4:0] write_reg,
    input wire [31:0] write_data,
    output wire [31:0] read_data1,
    output wire [31:0] read_data2
);

    reg [31:0] registers [0:31];

    // Lógica de leitura assíncrona
    assign read_data1 = (read_reg1 == 5'b0) ? 32'b0 : registers[read_reg1];
    assign read_data2 = (read_reg2 == 5'b0) ? 32'b0 : registers[read_reg2];

    // Lógica de escrita síncrona
    always @(posedge clk) begin
        if (reg_write && (write_reg != 5'b0)) begin
            registers[write_reg] <= write_data;
        end
    end
    // Inicialização (para simulação)
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            registers[i] = 32'b0;
        end
    end
endmodule

// Módulo de Controle da ALU
module alu_control (
    input wire [1:0] alu_op,
    input wire [2:0] funct3,
    input wire funct7_5,
    output reg [3:0] alu_control_out
);

    always @(*) begin
        case (alu_op)
            // Caso 00: Reservado para o ADD de cálculo de endereço (Load/Store)
            2'b00: begin
                alu_control_out = 4'b0010; // Sempre ADD
            end

            // Caso 01: Reservado para o SUB de comparação (Branch)
            2'b01: begin
                alu_control_out = 4'b0110; // Sempre SUB
            end

            // Caso 10: Para instruções Tipo-R (lê funct3)
            2'b10: begin
                case (funct3)
                    3'b000: alu_control_out = funct7_5 ? 4'b0110 : 4'b0010; // SUB/ADD
                    3'b001: alu_control_out = 4'b0100; // SLL
                    3'b100: alu_control_out = 4'b0011; // XOR
                    3'b101: alu_control_out = 4'b0101; // SRL
                    3'b110: alu_control_out = 4'b0001; // OR
                    3'b111: alu_control_out = 4'b0000; // AND
                    default: alu_control_out = 4'bxxxx;
                endcase
            end

            // Caso 11: Novo caso para instruções Tipo-I (lê funct3)
            2'b11: begin
                case (funct3)
                    3'b000: alu_control_out = 4'b0010; // ADDI
                    3'b001: alu_control_out = 4'b0100; // SLLI
                    3'b101: alu_control_out = 4'b0101; // SRLI
                    3'b110: alu_control_out = 4'b0001; // ORI
                    3'b111: alu_control_out = 4'b0000; // ANDI
                    default: alu_control_out = 4'bxxxx;
                endcase
            end

            default: alu_control_out = 4'bxxxx;
        endcase
    end
endmodule

// Módulo da ALU
module alu (
    input wire [31:0] a,
    input wire [31:0] b,
    input wire [3:0] alu_control,
    output reg [31:0] result,
    output wire zero
);

    always @(*) begin
        case (alu_control)
            4'b0000: result = a & b; // AND
            4'b0001: result = a | b; // OR
            4'b0010: result = a + b; // ADD
            4'b0011: result = a ^ b; // XOR
            4'b0101: result = a >> b[4:0]; //SRL
            4'b0100: result = a << b[4:0]; // SLL
            4'b0110: result = a - b; // SUB
            default: result = 32'b0;
        endcase
    end

    assign zero = (result == 32'b0);

endmodule

// Módulo Gerador de Imediatos
module imm_gen (
    input wire [31:0] instruction,
    output reg [31:0] immediate
);
    // Decodifica o valor imediato com base no tipo da instrução (opcode)
    always @(*) begin
        case (instruction[6:0])
            // Tipo-I (usado por ADDI, ORI, LW, etc.)
            7'b0010011, 7'b0000011:
                immediate = {{20{instruction[31]}}, instruction[31:20]};

            // Tipo-S (usado por SW, SB, etc.)
            7'b0100011:
                immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};

            // Tipo-B (usado por BEQ)
            7'b1100011:
                // Os bits do imediato do Tipo-B são espalhados, aqui nós os juntamos
                immediate = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};

            default:
                immediate = 32'hxxxxxxxx; // Indefinido para outros tipos
        endcase
    end
endmodule

// Módulo da Memória de Dados
module data_memory (
    input wire clk,
    input wire mem_write,
    input wire mem_read,
    input wire [31:0] address,
    input wire [31:0] write_data,
    input wire [2:0] funct3,
    output reg [31:0] read_data
);

    // A memória agora é um array de bytes para facilitar a manipulação
    reg [7:0] ram [0:4095];
    wire [11:0] word_address = address[11:0]; 

    // Lógica de Leitura (combinacional)
    always @(*) begin
        if (mem_read) begin
            case (funct3)
                // LW: Load Word (32 bits)
                3'b010: read_data = {ram[word_address+3], ram[word_address+2], ram[word_address+1], ram[word_address]};
                // LH: Load Half-word (16 bits, com extensão de sinal)
                3'b001: read_data = {{16{ram[word_address+1][7]}}, ram[word_address+1], ram[word_address]};
                // LHU: Load Half-word Unsigned (16 bits, com extensão de zero)
                3'b101: read_data = {{16{1'b0}}, ram[word_address+1], ram[word_address]};
                // LB: Load Byte (8 bits, com extensão de sinal)
                3'b000: read_data = {{24{ram[word_address][7]}}, ram[word_address]};
                // LBU: Load Byte Unsigned (8 bits, com extensão de zero)
                3'b100: read_data = {{24{1'b0}}, ram[word_address]};
                default: read_data = 32'hxxxxxxxx;
            endcase
        end else begin
            read_data = 32'b0;
        end
    end

    // Lógica de Escrita (síncrona)
    always @(posedge clk) begin
        if (mem_write) begin
            case (funct3)
                // SW: Store Word (32 bits)
                3'b010: begin
                    ram[word_address]   <= write_data[7:0];
                    ram[word_address+1] <= write_data[15:8];
                    ram[word_address+2] <= write_data[23:16];
                    ram[word_address+3] <= write_data[31:24];
                end
                // SH: Store Half-word (16 bits)
                3'b001: begin
                    ram[word_address]   <= write_data[7:0];
                    ram[word_address+1] <= write_data[15:8];
                end
                // SB: Store Byte (8 bits)
                3'b000: begin
                    ram[word_address] <= write_data[7:0];
                end
            endcase
        end
    end

endmodule

// Módulo de Controle Principal
module main_control (
    input wire [6:0] opcode,
    output reg branch,
    output reg mem_read,
    output reg mem_to_reg,
    output reg [1:0] alu_op,
    output reg mem_write,
    output reg alu_src,
    output reg reg_write
);

    always @(*) begin
        // Inicializa todos os sinais com um valor padrão "seguro"
        branch      = 1'b0;
        mem_read    = 1'b0;
        mem_to_reg  = 1'b0;
        alu_op      = 2'b00; // Padrão para ADD (usado por lw/sw)
        mem_write   = 1'b0;
        alu_src     = 1'b0; // Padrão para usar registrador na ALU
        reg_write   = 1'b0;

        // Ativa os sinais específicos com base no opcode
        case (opcode)
            // Tipo-R (add, sub, and, or, etc.)
            7'b0110011: begin
                alu_op      = 2'b10;
                reg_write   = 1'b1;
            end
            
            // Tipo-I (addi, ori, slti, etc.)
            7'b0010011: begin
                alu_src     = 1'b1;
                reg_write   = 1'b1;
                alu_op      = 2'b11;
            end
            
            // Tipo-I (lw)
            7'b0000011: begin
                mem_read    = 1'b1;
                mem_to_reg  = 1'b1;
                alu_src     = 1'b1;
                reg_write   = 1'b1;
            end
            
            // Tipo-S (sw, sb, sh)
            7'b0100011: begin
                mem_write   = 1'b1;
                alu_src     = 1'b1;
            end
            
            // Tipo-B (beq)
            7'b1100011: begin
                branch      = 1'b1;
                alu_op      = 2'b01; 
            end
        endcase
    end
endmodule

// Módulo Top-Level do Processador RISC-V
module risc_v_processor (
    input wire clk,
    input wire rst
);

    // Sinais e Fios Internos
    wire [31:0] pc_current, pc_next, pc_plus_4, pc_branch;
    wire [31:0] instruction;
    wire [31:0] read_data1, read_data2;
    wire [31:0] immediate;
    wire [31:0] alu_in_b;
    wire [31:0] alu_result;
    wire [31:0] mem_read_data;
    wire [31:0] write_back_data;

    wire branch, mem_read, mem_to_reg, mem_write, alu_src, reg_write;
    wire [1:0] alu_op;
    wire [3:0] alu_control_out;
    wire beq_cond = (instruction[14:12] == 3'b000) & alu_zero;  
    wire bne_cond = (instruction[14:12] == 3'b001) & ~alu_zero; 
    wire alu_zero;
    wire branch_control;

    // Instanciação dos Módulos

    // --- Estágio de Busca de Instrução (IF) ---
    pc u_pc (
        .clk(clk),
        .rst(rst),
        .next_pc(pc_next),
        .current_pc(pc_current)
    );

    assign pc_plus_4 = pc_current + 4;

    instruction_memory u_imem (
        .read_address(pc_current),
        .instruction(instruction)
    );

    // --- Estágio de Decodificação (ID) ---
    main_control u_ctrl (
        .opcode(instruction[6:0]),
        .branch(branch),
        .mem_read(mem_read),
        .mem_to_reg(mem_to_reg),
        .alu_op(alu_op),
        .mem_write(mem_write),
        .alu_src(alu_src),
        .reg_write(reg_write)
    );

    register_file u_regfile (
        .clk(clk),
        .reg_write(reg_write),
        .read_reg1(instruction[19:15]),
        .read_reg2(instruction[24:20]),
        .write_reg(instruction[11:7]),
        .write_data(write_back_data),
        .read_data1(read_data1),
        .read_data2(read_data2)
    );

    imm_gen u_immgen (
        .instruction(instruction),
        .immediate(immediate)
    );

    // --- Estágio de Execução (EX) ---
    mux2_1 #(32) alu_mux (
        .a(read_data2),
        .b(immediate),
        .sel(alu_src),
        .y(alu_in_b)
    );

    alu_control u_alu_ctrl (
        .alu_op(alu_op),
        .funct3(instruction[14:12]),
        .funct7_5(instruction[30]),
        .alu_control_out(alu_control_out)
    );

    alu u_alu (
        .a(read_data1),
        .b(alu_in_b),
        .alu_control(alu_control_out),
        .result(alu_result),
        .zero(alu_zero)
    );

    // Lógica de cálculo do endereço de branch
    assign pc_branch = pc_current + immediate; // Simplificado
    assign branch_control = branch & (beq_cond | bne_cond);     // O desvio acontece se for uma instrução de branch E uma das condições for atendida

    mux2_1 #(32) pc_mux (
        .a(pc_plus_4),
        .b(pc_branch),
        .sel(branch_control),
        .y(pc_next)
    );

    // --- Estágio de Memória (MEM) ---
    data_memory u_dmem (
        .clk(clk),
        .mem_write(mem_write),
        .mem_read(mem_read),
        .address(alu_result),
        .write_data(read_data2),
        .funct3(instruction[14:12]),
        .read_data(mem_read_data)
    );

    // --- Estágio de Write-Back (WB) ---
    mux2_1 #(32) wb_mux (
        .a(alu_result),
        .b(mem_read_data),
        .sel(mem_to_reg),
        .y(write_back_data)
    );
endmodule 

    // Módulo genérico de MUX 2 para 1
    // (Pode ser colocado em um arquivo separado)
module mux2_1 #(parameter WIDTH = 32) (
    input wire [WIDTH-1:0] a,
    input wire [WIDTH-1:0] b,
    input wire sel,
    output wire [WIDTH-1:0] y
);
    assign y = sel ? b : a;
endmodule