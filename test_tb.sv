module test_tb();

logic [ 2:0] i_opsel;
logic        i_sub;
logic        i_unsigned;
logic        i_arith;
logic [31:0] i_op1;
logic [31:0] i_op2;
logic [31:0] o_result;
logic        o_eq;
logic        o_slt;

logic [31:0] i_inst;
logic [ 5:0] i_format;
logic [31:0] o_immediate;

logic clk;
logic rst;
logic [4:0]rs1_addr_i;
logic [4:0]rs2_addr_i;
logic [4:0]rd_waddr_i;
logic rd_wen_i;
logic [31:0] rd_wdata_i;
logic [31:0] rs1_rdata_o;
logic [31:0] rs2_rdata_o;

alu alu_inst(
.i_opsel(i_opsel),
.i_sub(i_sub),
.i_unsigned(i_unsigned),
.i_arith(i_arith),
.i_op1(i_op1),
.i_op2(i_op2),
.o_result(o_result),
.o_eq(o_eq),
.o_slt(o_slt)
);

imm imm_decode_inst(.i_inst(i_inst),.i_format(i_format),.o_immediate(o_immediate));

logic [4:0]rs1_addr_i;
logic [4:0]rs2_addr_i;
logic [4:0]rd_waddr_i;
logic rd_wen_i;
logic [31:0] rd_wdata_i;
logic [31:0] rs1_rdata_o;
logic [31:0] rs2_rdata_o;

rf reg_inst(
    .i_clk(clk),.i_rst(rst),.i_rs1_raddr(rs1_addr_i),.i_rs2_raddr(rs2_addr_i),.i_rd_waddr(rd_waddr_i),
    .i_rd_wen(rd_wen_i),.i_rd_wdata(rd_wdata_i),.o_rs1_rdata(rs1_rdata_o),.o_rs2_rdata(rs2_rdata_o)
);


endmodule