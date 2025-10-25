module fetch_flopped (
    input  wire clk,
    input  wire rst_n,
    input  wire i_clu_branch,
    input  wire i_clu_halt,
    input  wire i_alu_o_Zero,
    input  wire [31:0] i_pc_o_rs1_data_mux_pcaddr,
    input  wire [31:0] i_imm_o_immediate,
    output reg  [31:0] o_instr_mem_rd_addr_ff,
    output reg  [31:0] o_pc_plus_4_ff,
    output reg  [31:0] PC_ff
);
    // Wires from combinational fetch
    wire [31:0] o_instr_mem_rd_addr, o_pc_plus_4, PC;

    // Instantiate base fetch (combinational version)
    fetch u_fetch (
        .clk(clk),
        .rst_n(rst_n),
        .i_clu_branch(i_clu_branch),
        .i_clu_halt(i_clu_halt),
        .i_alu_o_Zero(i_alu_o_Zero),
        .i_pc_o_rs1_data_mux_pcaddr(i_pc_o_rs1_data_mux_pcaddr),
        .i_imm_o_immediate(i_imm_o_immediate),
        .o_instr_mem_rd_addr(o_instr_mem_rd_addr_ff),
        .o_pc_plus_4(o_pc_plus_4_ff),
        .PC(PC_ff)
    );

    // Flop outputs here
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_instr_mem_rd_addr_ff <= 0;
            o_pc_plus_4_ff <= 0;
            PC_ff <= 0;
        end else begin
            o_instr_mem_rd_addr_ff <= o_instr_mem_rd_addr;
            o_pc_plus_4_ff <= o_pc_plus_4;
            PC_ff <= PC;
        end
    end
endmodule