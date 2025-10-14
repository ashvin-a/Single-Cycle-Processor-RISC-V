//Who will control the PC? - What is the signal input fromt he control Unit?
//Shall we assume PC will have 0 as the address location value?
module fetch (
    input  wire clk,
    input  wire rst_n,
    input  wire i_clu_branch,
    input  wire i_alu_o_slt,
    input  wire i_imm_o_immediate,
    output wire [31:0]o_instr_mem_rd_addr // read address is 32 bits and not 5 bits
);
    reg [31:0] PC;
    wire pc_imm_mux_val;
    assign pc_imm_mux_val = (i_clu_branch & i_alu_o_slt)? (PC + ({i_imm_o_immediate[31:1],1'b0})) : (PC + 4) ; 
    assign o_instr_mem_rd_addr = PC;
    //[Q1] : What Value to be initialized for PC?
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            PC <= 0; // Can it be 0?
        end
        else begin
            PC <= pc_imm_mux_val;
        end
    end
endmodule
/////////////////////////////////////////////////
/////////////////Instruction Memory/////////////////////
/////////////////////////////////////////////////
//[Q2] What is the size of the instruction memory? - Assuming 16KB for now
//[Q] Who is initialising the insturction memory? How are the instructions initially written into the memory?
module instruction_memory(
    input  wire clk,
    input  wire rst_n,
    input  wire [31:0]i_instr_mem_rd_addr,
    // output instruction word. This is used to extract the relevant immediate
    // bits and assemble them into the final immediate.
    output wire [31:0] o_inst
);
    reg [31:0] instr_mem [4095:0];//16 KB Memory with 32 bit instruction in each location
    reg [31:0] t_inst_reg; //temporary reg for o_inst
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            t_inst_reg <= 0; // [Q3] Should it be don't care instead of 0?
            for (i = 0;i<4096;i++) begin
                instr_mem[i] <= 32'b0;
            end
        end
        else if (i_instr_mem_rd_addr)
            t_inst_reg <= instr_mem [i_instr_mem_rd_addr];
    end
    assign o_inst = t_inst_reg;
endmodule
/////////////////////////////////////////////////
/////////////////Data Memory/////////////////////
/////////////////////////////////////////////////
//[Q] Shall I assume that that instructions will always be 32 bit? - I guess Yes
//[Q] Will there be a negative case where both MemWrite and MemRead will be high?
//[Q] Who is initialising the data memory? And Should I be initializing to zero??
module data_memory(
    input  wire clk,
    input  wire rst_n,
    input  wire i_clu_MemWrite,
    input  wire i_clu_MemRead,
    input  wire [31:0]i_data_mem_addr, // This can be for both read and write
    input  wire [31:0] i_data_mem_wr_data, //Coming from Reg block o_rd_data2
    output wire [31:0] o_data_mem_rd_data 
);
    reg [31:0] data_mem [4095:0];//16 KB Memory with 32 bit data in each location
    reg [31:0] t_data_mem_rd_data_reg;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for (i = 0;i<4096;i++) begin
                data_mem[i] <= 32'b0;
            end
        end
        if (i_data_mem_addr && i_clu_MemRead && !i_clu_MemWrite)
            t_data_mem_rd_data_reg <= data_mem [i_data_mem_addr];
        if (i_data_mem_addr && !i_clu_MemRead && i_clu_MemWrite) // Wrote a seperate if - independent
                data_mem[i_data_mem_addr] <= i_data_mem_wr_data;
    end
    assign o_data_mem_rd_data = t_data_mem_rd_data_reg;

    //Also add a mux at the end

endmodule

module control_unit(
    input  wire clk,
    input  wire rst_n,
    input  wire [31:0] i_clu_inst,
    output wire o_clu_Branch,
    output wire o_clu_MemRead,
    output wire o_clu_MemtoReg,
    output wire [3:0] o_clu_ALUOp,
    output wire o_clu_MemWrite,
    output wire o_clu_ALUSrc,
    output wire o_clu_RegWrite
);
// Defauting values to 0;
wire o_clu_Branch = 1'b0;
wire o_clu_MemRead = 1'b0;
wire o_clu_MemtoReg = 1'b0;
wire [3:0] o_clu_ALUOp = 3'b000;
wire o_clu_MemWrite = 1'b0;
wire o_clu_ALUSrc = 1'b0;
wire o_clu_RegWrite = 1'b0;

wire [3:0] clu_inst_type; // Decode the type of instruction from the opcode
assign clu_inst_type = (i_clu_inst[6:0] == 4'b011_0011) ? 4'd0 :  // R - Type instruction
                        (i_clu_inst[6:0] == 4'b001_0011) ? 4'd1 : // I - Type instruction
                        (i_clu_inst[6:0] == 4'b011_0111) ? 4'd2 : // LUI instruction
                        (i_clu_inst[6:0] == 4'b001_0111) ? 4'd3 : // AUIPC instruction
                        (i_clu_inst[6:0] == 4'b000_0011) ? 4'd4 : // Load Instruction
                        (i_clu_inst[6:0] == 4'b010_0011) ? 4'd5 : // Store Instruction
                        (i_clu_inst[6:0] == 4'b110_0011) ? 4'd6 : // Branch Instruction
                        (i_clu_inst[6:0] == 4'b110_1111) ? 4'd7 : // JAL Instruction
                        (i_clu_inst[6:0] == 4'b110_0111) ? 4'd8 : // JALR Instruction

assign o_clu_Branch = (clu_inst_type == 4'd6); 

assign o_clu_MemRead = (clu_inst_type == 4'd4);

assign o_clu_MemtoReg = (clu_inst_type == 4'd4);

assign o_clu_MemWrite = (clu_inst_type == 4'd5);

assign o_clu_ALUSrc = (!(clu_inst_type == 4'd0) && !(clu_inst_type == 4'd6));

assign o_clu_RegWrite = (clu_inst_type == 4'd0) || (clu_inst_type == 4'd1) || (clu_inst_type == 4'd2) 
                        || (clu_inst_type == 4'd3) || (clu_inst_type == 4'd4);

assign o_clu_ALUOp = (clu_inst_type == 4'd0) ? 2'b10 : // R-type - Use funct3 and funct4 to determine ALU Operation
                     (clu_inst_type == 4'd1) ? 2'b11 : // I-type - Use funct3 to determine ALU operation
                     (clu_inst_type == 4'd6) ? 2'b01 : // Branch - ALU does subtraction
                     2'b00;                            // Others (load/store/jump) -  ALU does addition

                    
endmodule


module ALU_Control(
    
);


endmodule

module single_processor_unit(
    input logic clk,
    input logic rst_n
);
    ///Fetch Unit Instance/////////

    ///Instruction Memory Instance///

    ///Register file Instance//////

    //////imm gen instance/////////

    ///Control Unit Instance///////

    ///ALU Instance///////////////

    ///ALU Control Instance///////

    ///Data Memory Instance///////

endmodule