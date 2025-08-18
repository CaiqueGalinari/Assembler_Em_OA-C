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

// Módulo da Memória de Instruções com Parâmetro
module instruction_memory #(
    parameter MEM_DEPTH = 2048 // Defina o tamanho da memória aqui
)(
    input wire [31:0] read_address,
    output reg [31:0] instruction
);
     // A memória usa o parâmetro para definir seu tamanho
    reg [31:0] rom [0:MEM_DEPTH-1];
    reg [1023:0] nome_do_arquivo; // Suporta nomes de arquivo de até 127 caracteres

    initial begin
        // Declara uma variável (string) para armazenar o nome do arquivo
        
        // $value$plusargs procura por um argumento na linha de comando
        // que corresponda ao formato "ARQUIVO=%s".
        // Se encontrar, ele armazena o valor da string em 'nome_do_arquivo'
        // e retorna 1 (verdadeiro).
        if ($value$plusargs("%s", nome_do_arquivo)) begin
            $display("Carregando instrucoes do arquivo: %s", nome_do_arquivo);
            $readmemb(nome_do_arquivo, rom);
        end else begin
            // Se o argumento +ARQUIVO=... não for fornecido, imprime uma mensagem de erro e encerra.
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
    input wire funct7_5, // bit 30 da instrução
    output reg [3:0] alu_control_out
);

    always @(*) begin
        case (alu_op)
            2'b00: alu_control_out = 4'b0010; // add para load/store
            2'b01: alu_control_out = 4'b0110; // sub para branch
            2'b10: begin // Tipo-R
                case (funct3)
                    3'b000: alu_control_out = funct7_5 ? 4'b0110 : 4'b0010; // sub : add
                    3'b111: alu_control_out = 4'b0000; // and
                    3'b110: alu_control_out = 4'b0001; // or
                    default: alu_control_out = 4'bxxxx; // não implementado
                endcase
            end
            default: alu_control_out = 4'bxxxx; // não implementado
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
            4'b0110: result = a - b; // SUB
            default: result = 32'b0;
        endcase
    end

    assign zero = (result == 32'b0);

endmodule

// Módulo Gerador de Imediatos
// =================================================================
//          MÓDULO imm_gen CORRIGIDO E COMPLETO
// =================================================================
module imm_gen (
    input wire [31:0] instruction,
    output reg [31:0] immediate
);
    // Este módulo decodifica o valor imediato com base no tipo da instrução (opcode)
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
    output wire [31:0] read_data
);

    reg [31:0] ram [0:1023];

    assign read_data = mem_read ? ram[address[11:2]] : 32'b0;

    always @(posedge clk) begin
        if (mem_write) begin
            ram[address[11:2]] <= write_data;
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

    // Dentro do module main_control (no seu arquivo modulos.v)
// Substitua o always @(*) inteiro por este:

always @(*) begin
    // Inicializa todos os sinais com valores "seguros" (não fazer nada)
    branch      = 1'b0;
    mem_read    = 1'b0;
    mem_to_reg  = 1'b0;
    alu_op      = 2'b00; // Default para ADD
    mem_write   = 1'b0;
    alu_src     = 1'b0;
    reg_write   = 1'b0;

    // Agora, define os sinais apenas para as instruções que os ativam
    case (opcode)
        7'b0110011: begin // Tipo-R
            alu_op      = 2'b10;
            reg_write   = 1'b1;
        end
        7'b0010011: begin // Tipo-I (addi, ori, etc.)
            alu_src     = 1'b1;
            reg_write   = 1'b1;
        end
        7'b0000011: begin // lw
            mem_read    = 1'b1;
            mem_to_reg  = 1'b1;
            alu_src     = 1'b1;
            reg_write   = 1'b1;
        end
        7'b0100011: begin // sw, sb, etc.
            mem_write   = 1'b1;
            alu_src     = 1'b1;
        end
        7'b1100011: begin // beq
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
    assign branch_control = branch & alu_zero;

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



