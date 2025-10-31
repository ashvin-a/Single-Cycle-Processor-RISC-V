// Retire Valid has to come from PC/fetch - infact i guess all of them
// To connect the RETIRE TRACE OUT
// Check all the Dont care cases I have defined - is it ok to define Dont care


`timescale 1ns/1ps

/////////////////////////// FETCH MODULE ///////////////////////////

module fetch (
    input  wire clk,
    input  wire rst_n,
    //input  wire i_clu_branch,                       // Updates the PC accordingly when there is a branch instruction // Not required anymore as we are ANDING it within the MEM stage itself
    input  wire i_clu_halt,                         // When high the program has completed excecution and the PC stops updating
    //input  wire i_alu_o_Zero,                       // Needed for deciding between JALR+Branch and Other instructions  // Not required anymore as we are ANDING it within the MEM stage itself
    input  wire i_alu_o_Zero_clu_Branch_and,            // NEW ANDed SIGNAL from MEM Stage of the pipeline
    //input  wire [31:0] i_pc_o_rs1_data_mux_pcaddr,  // Take o_rs1 for JALR instruction and PC for others
    input  wire [31:0] i_pc_o_rs1_data_mux_imm_add_data,  // Take o_rs1 for JALR instruction and PC for others
    //input  wire [31:0]i_imm_o_immediate,
    output wire [31:0]o_instr_mem_rd_addr,          // Read address is 32 bits and not 5 bits
    output wire [31:0]o_pc_plus_4,
    output reg  [31:0] PC                           // Program Counter register
);
    wire [31:0]pc_imm_mux_val;
    //assign pc_imm_mux_val = (i_clu_branch & i_alu_o_Zero)? (i_pc_o_rs1_data_mux_pcaddr + i_imm_o_immediate) : (PC + 4) ; // Selecting the next value of PC to be updated after this fetch cycle according if breanch has to happen
    assign pc_imm_mux_val = (i_alu_o_Zero_clu_Branch_and)? (i_pc_o_rs1_data_mux_imm_add_data) : (PC + 4) ; // Selecting the next value of PC to be updated after this fetch cycle according if breanch has to happen
    assign o_instr_mem_rd_addr = PC;

    assign o_pc_plus_4 = PC + 4;

    always @(posedge clk) begin
        if(rst_n) begin
            PC <= 0; // Can it be 0?
        end
        else begin
            if (!i_clu_halt)
                PC <= pc_imm_mux_val;
        end
    end
endmodule

/////////////////////////// MAIN CONTROL UNIT ///////////////////////////
module control_unit(
    input  wire clk,
    input  wire rst_n,
    input  wire [31:0] i_clu_inst,
    output wire o_clu_Branch,
    output wire o_clu_halt, // TODO - To check whether we have to connect it at which stage of the pipeline
    output wire o_clu_MemRead,
    output wire o_clu_MemtoReg,
    output wire [1:0]o_clu_ALUOp,
    output wire o_clu_MemWrite,
    output wire [1:0] o_clu_ALUSrc,
    output wire o_clu_RegWrite,
    output wire [1:0]o_clu_lui_auipc_mux_sel,       // The Mux in between reg and alu for lui and auipc instruction implementation
    output wire [1:0]o_clu_branch_instr_alu_sel,    // Should be invalid by default
    output wire o_clu_pc_o_rs1_data_mux_sel,
    output wire [2:0]o_clu_ld_st_type_sel,
    output wire [2:0]o_sign_or_zero_ext_data_mux    // This signal will go to 5:1 MUX which will choose between ZERO extend , SIGN EXTEND or NO EXTEND on the read data from the datamem - ONLY FOR LOAD
);

assign o_clu_Branch =   ((i_clu_inst[6:0] == 7'b110_0011)|| (i_clu_inst[6:0] == 7'b110_1111) || (i_clu_inst[6:0] == 7'b110_0111)); // Branch enabled for BRANCH , JAL and JALR instruction types

assign o_clu_halt =     (i_clu_inst[6:0] == 7'b111_0011); // Halt

assign o_clu_MemRead =  (i_clu_inst[6:0] == 7'b000_0011); // Load

assign o_clu_MemtoReg = (i_clu_inst[6:0] == 7'b000_0011) ? 1'b1 : // Load
                        (i_clu_inst[6:0] == 7'b110_0011) ? 1'bx : // Branch
                        (i_clu_inst[6:0] == 7'b010_0011) ? 1'b0 : // Store
                        (i_clu_inst[6:0] == 7'b110_1111) ? 1'b0 : // JAL
                        (i_clu_inst[6:0] == 7'b110_0111) ? 1'b0 : // JALR
                        1'b0;

assign o_clu_MemWrite = (i_clu_inst[6:0] == 7'b010_0011);   // Store

assign o_clu_ALUSrc =   ((i_clu_inst[6:0] == 7'b001_0011)|| // I type
                        (i_clu_inst[6:0] == 7'b011_0111) || // LUI
                        (i_clu_inst[6:0] == 7'b001_0111) || // AUIPC
                        (i_clu_inst[6:0] == 7'b000_0011) || // Load
                        (i_clu_inst[6:0] == 7'b010_0011)) ? 2'b01 : // Store
                        ((i_clu_inst[6:0] == 7'b110_1111) || (i_clu_inst[6:0] == 7'b110_0111)) ? 2'b00 :  //JAL and JALR type
                        2'b10;   // Branch and R type

assign o_clu_RegWrite = (i_clu_inst[6:0] == 7'b011_0011) || // R type
                        (i_clu_inst[6:0] == 7'b001_0011) || // I type
                        (i_clu_inst[6:0] == 7'b011_0111) || // LUI
                        (i_clu_inst[6:0] == 7'b001_0111) || // AUIPC
                        (i_clu_inst[6:0] == 7'b000_0011) || // Load
                        (i_clu_inst[6:0] == 7'b110_1111) || // Jal
                        (i_clu_inst[6:0] == 7'b110_0111);   // Jalr

assign o_clu_ALUOp =    ((i_clu_inst[6:0] == 7'b011_0011) || (i_clu_inst[6:0] == 7'b001_0011)) ?  2'b10 : // R and I type 
                        (i_clu_inst[6:0] == 7'b110_0011) ? 2'b01 : //Branch
                        2'b00;

assign o_clu_branch_instr_alu_sel  =    (i_clu_inst[6:0] == 7'b110_0011)? (      //Check for BRANCH Instruction type
                                        (i_clu_inst[14:12] == 3'b000)? 2'b00  : //beq
                                        (i_clu_inst[14:12] == 3'b001)? 2'b01  : //bne
                                        (i_clu_inst[14:12] == 3'b100)? 2'b10  : //blt
                                        (i_clu_inst[14:12] == 3'b101)? 2'b11  : //bge
                                        (i_clu_inst[14:12] == 3'b110)? 2'b10  : //bltu
                                        (i_clu_inst[14:12] == 3'b111)? 2'b11  : 2'bxx) : //bgeu
                                        ((i_clu_inst[6:0] == 7'b110_1111) || (i_clu_inst[6:0] == 7'b110_0111)) ?  2'b01 : // JAl and JALR - forcing to ~o_eq check which is always true for PC+4 vs 32'b0
                                        2'bxx;

assign o_clu_lui_auipc_mux_sel =        (i_clu_inst[6:0] == 7'b011_0111) ? 2'b01 : // LUI
                                        (i_clu_inst[6:0] == 7'b001_0111) ? 2'b10 : // AUICP
                                        ((i_clu_inst[6:0] == 7'b110_1111) || (i_clu_inst[6:0] == 7'b110_0111)) ? 2'b11 : // JAL, JALR
                                        2'b00; // Register File

assign o_clu_pc_o_rs1_data_mux_sel =    (i_clu_inst[6:0] == 7'b110_0111); // JALR - Select o_rs1_data instead of PC

assign o_sign_or_zero_ext_data_mux =    (i_clu_inst[6:0] == 7'b000_0011) ? ( // Check for Load instruction
                                        (i_clu_inst[14:12] == 3'b000) ? 3'b001 : // lb
                                        (i_clu_inst[14:12] == 3'b001) ? 3'b011 : // lh
                                        (i_clu_inst[14:12] == 3'b010) ? 3'b100 : // lw
                                        (i_clu_inst[14:12] == 3'b100) ? 3'b000 : // lbu
                                        (i_clu_inst[14:12] == 3'b101) ? 3'b010 : // lhu
                                        3'b100) :
                                        3'b100; 

// LOAD and STORE INSTRCUTION TYPE SEL FOR MASK GENERATION
assign o_clu_ld_st_type_sel = (i_clu_inst[6:0] == 7'b000_0011) ? (                  // Check for Load instruction
                              (i_clu_inst[14:12] == 3'b000)? 3'b000  :              // Byte - lb 
                              (i_clu_inst[14:12] == 3'b001)? 3'b001  :              // Half - lh 
                              (i_clu_inst[14:12] == 3'b010)? 3'b010  :              // Word - lw 
                              (i_clu_inst[14:12] == 3'b100)? 3'b011  :              // Byte - unsigned - lbu
                              (i_clu_inst[14:12] == 3'b101)? 3'b100  : 3'b010) :    // Half - unsigned - lhu
                              (i_clu_inst[6:0] == 7'b010_0011) ? (                  // Check for STORE instruction
                              (i_clu_inst[14:12] == 3'b000)? 3'b101  :              // Byte - sb 
                              (i_clu_inst[14:12] == 3'b001)? 3'b110  :              // Half - sh 
                              (i_clu_inst[14:12] == 3'b010)? 3'b111  : 3'b111)      // Word - sw
                              : 3'bxxx ;                                            // SHOULD NOT HAPPEN
                              

endmodule

/////////////////////////// ALU MODULE ///////////////////////////

// The arithmetic logic unit (ALU) is responsible for performing the core
// calculations of the processor. It takes two 32-bit operands and outputs
// a 32 bit result based on the selection operation - addition, comparison,
// shift, or logical operation. 

module alu (
    // 3'b000: addition/subtraction if `i_sub` asserted
    // 3'b001: shift left logical
    // 3'b010,
    // 3'b011: set less than/unsigned if `i_unsigned` asserted
    // 3'b100: exclusive or
    // 3'b101: shift right logical/arithmetic if `i_arith` asserted
    // 3'b110: or
    // 3'b111: and
    input  wire [ 2:0] i_opsel,
    // When asserted, addition operations should subtract instead.
    // This is only used for `i_opsel == 3'b000` (addition/subtraction).
    input  wire        i_sub,
    // When asserted, comparison operations should be treated as unsigned.
    // This is only used for branch comparisons and set less than.
    // For branch operations, the ALU result is not used, only the comparison
    // results.
    input  wire        i_unsigned,
    // When asserted, right shifts should be treated as arithmetic instead of
    // logical. This is only used for `i_opsel == 3'b011` (shift right).
    input  wire        i_arith,
    // First 32-bit input operand.
    input  wire [31:0] i_op1,
    // Second 32-bit input operand.
    input  wire [31:0] i_op2,
    // 32-bit output result. Any carry out (from addition) should be ignored.
    output wire [31:0] o_result,
    // Equality result. This is used downstream to determine if a
    // branch should be taken. // BEQ
    output wire        o_eq,
    // Set less than result. This is used downstream to determine if a
    // branch should be taken.
    output wire        o_slt // BLT and BLTU
);

    wire [4:0]temp = i_op2[4:0];
    wire [31:0] sll = i_op1 << temp;
    wire [31:0] srl = i_op1 >> temp;
    wire [31:0] sra = $signed(i_op1) >>> temp;
    wire [31:0] slt_signed = ($signed(i_op1) < $signed(i_op2)) ? 32'b1 : 32'b0;
    wire [31:0] slt_unsigned = (i_op1 < i_op2) ? 32'b1 : 32'b0;

    assign o_eq  = (i_op1 == i_op2);
    assign o_slt =  i_unsigned ? (i_op1 <  i_op2) : ($signed(i_op1) <  $signed(i_op2));
    assign o_result =       (i_opsel == 3'b000)? ((i_sub) ? (i_op1 - i_op2) : (i_op1 + i_op2)) :
                            (i_opsel == 3'b001)? (i_op1 << temp) : 
                            ((i_opsel == 3'b010) || (i_opsel == 3'b011)) ? (i_unsigned ? slt_unsigned : slt_signed) : 
                            (i_opsel == 3'b100)? (i_op1 ^ i_op2) :  
                            (i_opsel == 3'b101)? ((i_arith)? sra : srl) : 
                            (i_opsel == 3'b110)? (i_op1 | i_op2) : 
                            (i_opsel == 3'b111)? (i_op1 & i_op2) : 
                            32'h0;
    
endmodule

/////////////////////////// ALU WRAPPER ///////////////////////////

module alu_wrapper (
    // 4 bit input from ALU Control block
    input  wire [ 3:0] i_alu_ctrl_opsel,
    // First 32-bit input operand.
    input  wire [31:0] i_rf_op1,
    // Second 32-bit input operand.
    input  wire [31:0] i_rf_op2,
    // Signal for Branch instruction selection coming from alu control
    input wire [1:0]i_clu_branch_instr_alu_sel,
    // 32-bit output result. Any carry out (from addition) should be ignored.
    output wire [31:0] o_alu_result,
    // Equality result. This is used downstream to determine if a
    // branch should be taken. (case of BLT/U , BGE/U , BNE , BEQ)
    output wire        o_alu_Zero
);
    wire [ 2:0] i_opsel;
    wire        i_sub;
    wire        i_unsigned;
    wire        i_arith;
    wire        o_eq;
    wire        o_slt;
    alu alu_inst(
        .i_opsel(i_opsel),
        .i_sub(i_sub),
        .i_unsigned(i_unsigned),
        .i_arith(i_arith),
        .i_op1(i_rf_op1),
        .i_op2(i_rf_op2),
        .o_result(o_alu_result),
        .o_eq(o_eq),
        .o_slt(o_slt)
    );

    // INPUTS TO THE INSIDE ALU
    assign i_unsigned = (i_alu_ctrl_opsel == 4'b1001)? 1'b1 : 1'b0; // Whenever it is SLTU , we will enable the unsigned signal high
    assign i_arith    = (i_alu_ctrl_opsel == 4'b0111)? 1'b1 : 1'b0;
    assign i_sub      = (i_alu_ctrl_opsel == 4'b0001)? 1'b1 : 1'b0;
    assign i_opsel    = (i_alu_ctrl_opsel == 4'b0000)? 3'b000 :    //ADD
                        (i_alu_ctrl_opsel == 4'b0001)? 3'b000 :    //SUB   
                        (i_alu_ctrl_opsel == 4'b0010)? 3'b111 :    //AND   
                        (i_alu_ctrl_opsel == 4'b0011)? 3'b110 :    //OR   
                        (i_alu_ctrl_opsel == 4'b0100)? 3'b100 :    //XOR   
                        (i_alu_ctrl_opsel == 4'b0101)? 3'b001 :    //SLL   
                        (i_alu_ctrl_opsel == 4'b0110)? 3'b101 :    //SRL   
                        (i_alu_ctrl_opsel == 4'b0111)? 3'b101 :    //SRA   
                        (i_alu_ctrl_opsel == 4'b1000)? 3'b010 :    //SLT
                        (i_alu_ctrl_opsel == 4'b1001)? 3'b010 :    //SLTU   
                        (i_alu_ctrl_opsel == 4'b1010)? 3'b011 :    //PASS_B - LUI   - Can I use 011 for LUI?
                        3'bXXX; //Don't care               
    
    // OUTPUTS
    assign o_alu_Zero = (i_clu_branch_instr_alu_sel == 2'b00) ?   o_eq :  //BEQ
                        (i_clu_branch_instr_alu_sel == 2'b01) ?  ~o_eq :  //BNE
                        (i_clu_branch_instr_alu_sel == 2'b10) ?  o_slt :  //BLT / BLTU 
                        ~o_slt;                                           //BGE / BGEU
                        
endmodule
/////////////////////////// ALU CONTROL ///////////////////////////

module alu_control( input  wire [1:0]  i_clu_alu_op,
                    // input  wire [31:0] i_instr_mem_inst,
                    input  wire [2:0] funct3,
                    input  wire [6:0] funct7,
                    input  wire opcode_5thbit_add_sub, // TODO - Change the name from add_sub to R_I as this for that purpose
                    output wire [3:0]  o_alu_control_sel
                    );

assign o_alu_control_sel = 
    (i_clu_alu_op == 2'b00) ? 4'b0000 : // Forced Addition (S, U, J) - Here we can have I' and S as well
    (i_clu_alu_op == 2'b01) ?           // Forced Subtraction (B) - BEQ , BNE - This needs to be updated - Add a func3 check here
            ((funct3 == 3'b000) ? 4'b1000 : //beq 
             (funct3 == 3'b001) ? 4'b1000 : //bne
             (funct3 == 3'b100) ? 4'b1000 : //blt 
             (funct3 == 3'b101) ? 4'b1000 : //bge
             (funct3 == 3'b110) ? 4'b1001 : //bltu 
             (funct3 == 3'b111) ? 4'b1001 : //bgeu
             4'bxxxx) :
    (i_clu_alu_op == 2'b10) ? (
            (funct3 == 3'b000) ? (opcode_5thbit_add_sub? ((funct7[5]) ? 4'b0001 : 4'b0000) : 4'b0000) : // sub/add
            (funct3 == 3'b001) ? 4'b0101 : // sll
            (funct3 == 3'b010) ? 4'b1000 : // slt //BLT
            (funct3 == 3'b011) ? 4'b1001 : // sltu
            (funct3 == 3'b100) ? 4'b0100 : // xor
            (funct3 == 3'b101) ? ((funct7[5]) ? 4'b0111 : 4'b0110) : // sra/srl
            (funct3 == 3'b110) ? 4'b0011 : // or
            (funct3 == 3'b111) ? 4'b0010 : // and
            4'bxxxx) :
    (i_clu_alu_op == 2'b11) ? 4'bxxxx : 4'bxxxx;

endmodule

/////////////////////////// HART MODULE ///////////////////////////
module hart #(
    // After reset, the program counter (PC) should be initialized to this
    // address and start executing instructions from there.
    parameter RESET_ADDR = 32'h00000000
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // Instruction fetch goes through a read only instruction memory (imem)
    // port. The port accepts a 32-bit address (e.g. from the program counter)
    // per cycle and combinationally returns a 32-bit instruction word. This
    // is not representative of a realistic memory interface; it has been
    // modeled as more similar to a DFF or SRAM to simplify phase 3. In
    // later phases, you will replace this with a more realistic memory.
    //
    // 32-bit read address for the instruction memory. This is expected to be
    // 4 byte aligned - that is, the two LSBs should be zero.
    output wire [31:0] o_imem_raddr,
    // Instruction word fetched from memory, available on the same cycle.
    input  wire [31:0] i_imem_rdata,
    // Data memory accesses go through a separate read/write data memory (dmem)
    // that is shared between read (load) and write (stored). The port accepts
    // a 32-bit address, read or write enable, and mask (explained below) each
    // cycle. Reads are combinational - values are available immediately after
    // updating the address and asserting read enable. Writes occur on (and
    // are visible at) the next clock edge.
    //
    // Read/write address for the data memory. This should be 32-bit aligned
    // (i.e. the two LSB should be zero). See `o_dmem_mask` for how to perform
    // half-word and byte accesses at unaligned addresses.
    output wire [31:0] o_dmem_addr,
    // When asserted, the memory will perform a read at the aligned address
    // specified by `i_addr` and return the 32-bit word at that address
    // immediately (i.e. combinationally). It is illegal to assert this and
    // `o_dmem_wen` on the same cycle.
    output wire        o_dmem_ren,
    // When asserted, the memory will perform a write to the aligned address
    // `o_dmem_addr`. When asserted, the memory will write the bytes in
    // `o_dmem_wdata` (specified by the mask) to memory at the specified
    // address on the next rising clock edge. It is illegal to assert this and
    // `o_dmem_ren` on the same cycle.
    output wire        o_dmem_wen,
    // The 32-bit word to write to memory when `o_dmem_wen` is asserted. When
    // write enable is asserted, the byte lanes specified by the mask will be
    // written to the memory word at the aligned address at the next rising
    // clock edge. The other byte lanes of the word will be unaffected.
    output wire [31:0] o_dmem_wdata,
    // The dmem interface expects word (32 bit) aligned addresses. However,
    // WISC-25 supports byte and half-word loads and stores at unaligned and
    // 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    //
    // To perform a half-word read at address 0x00001002, align `o_dmem_addr`
    // to 0x00001000, assert `o_dmem_ren`, and set the mask to 0b1100 to
    // indicate that only the upper two bytes should be read. Only the upper
    // two bytes of `i_dmem_rdata` can be assumed to have valid data; to
    // calculate the final value of the `lh[u]` instruction, shift the rdata
    // word right by 16 bits and sign/zero extend as appropriate.
    //
    // To perform a byte write at address 0x00002003, align `o_dmem_addr` to
    // `0x00002000`, assert `o_dmem_wen`, and set the mask to 0b1000 to
    // indicate that only the upper byte should be written. On the next clock
    // cycle, the upper byte of `o_dmem_wdata` will be written to memory, with
    // the other three bytes of the aligned word unaffected. Remember to shift
    // the value of the `sb` instruction left by 24 bits to place it in the
    // appropriate byte lane.
    output wire [ 3:0] o_dmem_mask,
    // The 32-bit word read from data memory. When `o_dmem_ren` is asserted,
    // this will immediately reflect the contents of memory at the specified
    // address, for the bytes enabled by the mask. When read enable is not
    // asserted, or for bytes not set in the mask, the value is undefined.
    input  wire [31:0] i_dmem_rdata,
	// The output `retire` interface is used to signal to the testbench that
    // the CPU has completed and retired an instruction. A single cycle
    // implementation will assert this every cycle; however, a pipelined
    // implementation that needs to stall (due to internal hazards or waiting
    // on memory accesses) will not assert the signal on cycles where the
    // instruction in the writeback stage is not retiring.
    //
    // Asserted when an instruction is being retired this cycle. If this is
    // not asserted, the other retire signals are ignored and may be left invalid.
    output wire        o_retire_valid,
    // The 32 bit instruction word of the instrution being retired. This
    // should be the unmodified instruction word fetched from instruction
    // memory.
    output wire [31:0] o_retire_inst,
    // Asserted if the instruction produced a trap, due to an illegal
    // instruction, unaligned data memory access, or unaligned instruction
    // address on a taken branch or jump.
    output wire        o_retire_trap,
    // Asserted if the instruction is an `ebreak` instruction used to halt the
    // processor. This is used for debugging and testing purposes to end
    // a program.
    output wire        o_retire_halt,
    // The first register address read by the instruction being retired. If
    // the instruction does not read from a register (like `lui`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs1_raddr,
    // The second register address read by the instruction being retired. If
    // the instruction does not read from a second register (like `addi`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs2_raddr,
    // The first source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs1 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs1_rdata,
    // The second source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs2 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs2_rdata,
    // The destination register address written by the instruction being
    // retired. If the instruction does not write to a register (like `sw`),
    // this should be 5'd0.
    output wire [ 4:0] o_retire_rd_waddr,
    // The destination register data written to the register file in the
    // writeback stage by this instruction. If rd is 5'd0, this field is
    // ignored and can be treated as a don't care.
    output wire [31:0] o_retire_rd_wdata,
    // The current program counter of the instruction being retired - i.e.
    // the instruction memory address that the instruction was fetched from.
    output wire [31:0] o_retire_pc,
    // the next program counter after the instruction is retired. For most
    // instructions, this is `o_retire_pc + 4`, but must be the branch or jump
    // target for *taken* branches and jumps.
    output wire [31:0] o_retire_next_pc

`ifdef RISCV_FORMAL
    ,`RVFI_OUTPUTS,
`endif
);

// Wire declarations
wire [5:0] i_imm_format;
wire [31:0] t_rs2_rdata;
wire [31:0] i_dmem_alu_muxout_data;
wire [31:0] o_rs1_rdata;
wire [31:0] rs2_rdata_imm_mux_data;
wire [3:0] o_alu_control_sel;
wire [1:0] t_clu_ALUSrc;
wire t_clu_MemtoReg, t_clu_branch;
wire t_clu_halt;
wire [31:0] PC_current_val;
wire [31:0] t_lui_auipc_mux_data;
wire [1:0]t_clu_lui_auipc_mux_sel;
wire [2:0]t_sign_or_zero_ext_data_mux;
wire [1:0] t_clu_branch_instr_alu_sel;
wire [1:0]t_clu_alu_op;
wire [31:0] i_dmem_rdata_sign_or_zero_ext_mux_data;
wire t_rd_wen;
wire [31:0] t_pc_plus_4; 
wire [31:0] t_pc_o_rs1_data_mux_pcaddr;
wire t_clu_pc_o_rs1_data_mux_sel;
wire target;
wire t_alu_o_Zero;
wire [31:0]t_o_dmem_addr;
wire t_dmem_wen;
wire [3:0]t_dmem_mask;
wire t_dmem_ren;
wire [2:0]t_clu_ld_st_type_sel;
wire [31:0] t_immediate_out_data;

//////////Phase 5 Wires//////////
wire [4:0] t_i_rd_waddr;
wire t_i_rd_wen;
wire [31:0] t_pc_o_rs1_data_mux_imm_add_data; // New wire added for the imm and PC added value which is passed through EX/MEM Pipieline register
wire t_alu_o_Zero_clu_Branch_and; // NEW INPUT ADDED FOR THE ANDing in MEM stage
wire [31:0] t_pc_o_rs1_data_mux_imm_add_EX_stage_data; //TODO - Should go to EX/MEM Pipeline Register
wire [31:0] f_rs2_rdata;
wire [31:0] t_o_alu_result; // Getting the Alu result before pipelining

///TODO - To change all the reture signals to the latest values pipelined
assign o_retire_halt      = t_clu_halt;
assign o_retire_valid     = 1;
assign o_retire_inst      =   i_imem_rdata;         
assign o_retire_trap      =   0; //Temporary assignment - Need to be modified         
assign o_retire_rs1_raddr =   i_imem_rdata[19:15];   // TODO - When to retire - Do we need to get it from the pipeline      
assign o_retire_rs1_rdata =   o_rs1_rdata;           // TODO - When to retire - Do we need to get it from the pipeline 
assign o_retire_rs2_raddr =   i_imem_rdata[24:20];   // TODO - When to retire - Do we need to get it from the pipeline 
assign o_retire_rs2_rdata =   t_rs2_rdata;           // TODO - When to retire - Do we need to get it from the pipeline 
assign o_retire_rd_waddr  =   t_rd_wen?i_imem_rdata[11:7]:5'b0;         
assign o_retire_rd_wdata  =   i_dmem_alu_muxout_data;        
assign o_retire_pc        =   PC_current_val;         
assign o_retire_next_pc   =   PC_current_val + 4;   //Need to be modified based on Branch and Jump instructions   

// DUT Instantiations
// Fetch Section
fetch fetch_inst(
    .clk(i_clk),
    .rst_n(i_rst),
    //.i_clu_branch(t_clu_branch), //TODO - Needs to update with the 3rd pipeline input of the Branch control signal - NOT NEEDED ANY MORE AS WE ARE SENDING OUT AN ANDED SIGNAL
    .i_clu_halt(t_clu_halt),
    //.i_alu_o_Zero(t_alu_o_Zero), // Not required any more as AND is done in MEM Stage
    .i_alu_o_Zero_clu_Branch_and(t_alu_o_Zero_clu_Branch_and), //  MEM_EX Reg ANDED output given to the Fetch
    //.i_imm_o_immediate(t_immediate_out_data), // Removing Imm from fetch and keeping the addition in EX stage
    .i_pc_o_rs1_data_mux_imm_add_data(t_pc_o_rs1_data_mux_imm_add_data), //T Connected it to EX/MEM Pipeline register out of the muxed and added data
    //.i_pc_o_rs1_data_mux_pcaddr(t_pc_o_rs1_data_mux_pcaddr),
    .PC(PC_current_val),
    .o_pc_plus_4(t_pc_plus_4),
    .o_instr_mem_rd_addr(o_imem_raddr)
);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////*IF_ID Pipeline Register Implementation*/////////////////////////////////////////////////////////////////////
/////////////////////////////////////{ t_pc_plus_4[31:0], PC_current_val[31:0], i_imem_rdata[31:0]}///////////////////////////////////////////////////////
/////////////////////////////////    { IF_ID[95:64],      IF_ID[63:32],         IF_ID[31:0]}//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//TODO : Enable Clear and ready
reg [95:0]IF_ID;
wire [95:0]IF_ID_temp;
assign IF_ID_temp = {t_pc_plus_4,PC_current_val,i_imem_rdata};
always @ (posedge i_clk) begin
    if (i_rst)
            IF_ID <= 95'b0;
    else begin
            IF_ID <= IF_ID_temp;
    end
end
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
wire [31:0] t_i_imem_to_rf_instr;
assign t_i_imem_to_rf_instr = IF_ID[31:0]; // The Pipelined instruction in the ID Stage
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////Register File Instance///////////////////////////////////////////////////////
rf #(.BYPASS_EN(0)) rf(
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_rs1_raddr(t_i_imem_to_rf_instr[19:15]), // Replaced it with the pipelined reg out instruction
    .i_rs2_raddr(t_i_imem_to_rf_instr[24:20]), // Replaced it with the pipelined reg out instruction
    .i_rd_waddr(t_i_rd_waddr), //Coming from MEM_WB Pipeline register
    .i_rd_wen(t_i_rd_wen), //Has to come from the MEM_WB pipeline register
    .i_rd_wdata(i_dmem_alu_muxout_data), // Coming from MEM_WB Pipeline register
    .o_rs1_rdata(o_rs1_rdata),
    .o_rs2_rdata(t_rs2_rdata) // This signal is going only to the Pipeline ID_EX
);

/////////////////////////////////////////////////////////Control Unit Instance///////////////////////////////////////////////////////
control_unit control_unit_inst(
    .clk(i_clk),
    .rst_n(i_rst),
    .i_clu_inst(t_i_imem_to_rf_instr), // Replaced it with the pipelined reg out instruction
    .o_clu_Branch(t_clu_branch),
    .o_clu_halt(t_clu_halt),
    .o_clu_MemRead(t_dmem_ren),
    .o_clu_MemtoReg(t_clu_MemtoReg),
    .o_clu_ALUOp(t_clu_alu_op),
    .o_clu_MemWrite(t_dmem_wen),
    .o_clu_ALUSrc(t_clu_ALUSrc),
    .o_clu_RegWrite(t_rd_wen),
    .o_clu_lui_auipc_mux_sel(t_clu_lui_auipc_mux_sel),
    .o_clu_branch_instr_alu_sel(t_clu_branch_instr_alu_sel),
    .o_clu_pc_o_rs1_data_mux_sel(t_clu_pc_o_rs1_data_mux_sel),
    .o_clu_ld_st_type_sel(t_clu_ld_st_type_sel),
    .o_sign_or_zero_ext_data_mux(t_sign_or_zero_ext_data_mux)
);

///////////////////////////////Immediate format decoding -- > Updated to do the Decoding in the ID stage ///////////////////////////////
assign i_imm_format =   
    (t_i_imem_to_rf_instr[6:0] == 7'b0110011)? 6'b000001 : // R
    ((t_i_imem_to_rf_instr[6:0] == 7'b0010011)  || (t_i_imem_to_rf_instr[6:0] == 7'b1100111))? 6'b000010 : // I and Jalr
    (t_i_imem_to_rf_instr[6:0] == 7'b0000011)? 6'b000010 : // I (Load)
    (t_i_imem_to_rf_instr[6:0] == 7'b0100011)? 6'b000100 : // S
    (t_i_imem_to_rf_instr[6:0] == 7'b1100011)? 6'b001000 : // B
    ((t_i_imem_to_rf_instr[6:0] == 7'b0110111) || (t_i_imem_to_rf_instr[6:0] == 7'b0010111))? 6'b010000 : // U
    (t_i_imem_to_rf_instr[6:0] == 7'b1101111)? 6'b100000 : // Jal
    6'bXXXXXX;

//////////////////////////////////////////////////////////Immediate Generator Instance////////////////////////////////////////////////////
imm imm_decode_inst(
    .i_inst(t_i_imem_to_rf_instr), // Replaced it with the pipelined reg out instruction
    .i_format(i_imm_format),
    .o_immediate(t_immediate_out_data)
);

///#################################################################################################################################################################################
////////////////#######################################*ID_EX Pipeline Register Implementation*#######################################//////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////Assigning a 20 bit Wire to group all the Control Signal together for pipelining///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//{ID_EX[195], ID_EX[194],                 ID_EX[193], ID_EX[192], ID_EX[191:189],            ID_EX[188:186],                   ID_EX[185],     ID_EX[184],   ID_EX[183:182],                  ID_EX[181:180],    ID_EX[179:178],    ID_EX[177:176]||||/////////////
//{t_rd_wen,   t_clu_pc_o_rs1_data_mux_sel,t_dmem_wen, t_dmem_ren, t_clu_ld_st_type_sel[2:0], t_sign_or_zero_ext_data_mux[2:0], t_clu_MemtoReg, t_clu_branch, t_clu_branch_instr_alu_sel[1:0], t_clu_alu_op[1:0], t_clu_ALUSrc[1:0], t_clu_lui_auipc_mux_sel[1:0]}//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
wire [19:0] Control_input_ID_EX;
assign Control_input_ID_EX = {t_rd_wen,t_clu_pc_o_rs1_data_mux_sel,t_dmem_wen,t_dmem_ren,t_clu_ld_st_type_sel[2:0],t_sign_or_zero_ext_data_mux[2:0],t_clu_MemtoReg,
                             t_clu_branch,t_clu_branch_instr_alu_sel[1:0],t_clu_alu_op[1:0],t_clu_ALUSrc[1:0],t_clu_lui_auipc_mux_sel[1:0]};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////{ID_EX[195:176],        ID_EX[175:144], ID_EX[143:112], ID_EX[111:80],      ID_EX[79:48],      ID_EX[47:41], ID_EX[40:38], ID_EX[37],    ID_EX[36:5],           ID_EX[4:0]};///////////
///      {Control Signals[20:0], PC+4,           PC,             o_rs1_rdata[31:0],  t_rs2_rdata[31:0], func7,        func3,        opcode5thbit, t_immediate_out_data,  wr_addr[4:0]}//////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
reg [195:0]ID_EX;
wire [195:0]ID_EX_temp;
assign ID_EX_temp = {Control_input_ID_EX[19:0],IF_ID[95:64],IF_ID[63:32],o_rs1_rdata[31:0],t_rs2_rdata[31:0],IF_ID[31:25],IF_ID[14:12],IF_ID[5],t_immediate_out_data[31:0],IF_ID[11:7]};
always @ (posedge i_clk) begin
    if (i_rst)
            ID_EX <= 196'b0;
    else begin
            ID_EX <= ID_EX_temp;
    end
end
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////MUXES in  the EX Stage//////////////////////////////////////////////////////
// TODO : ALL THE MUX INPUTS ARE NOT UPDATED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
assign rs2_rdata_imm_mux_data       = (ID_EX[179:178] == 2'b00) ? 32'b0 : 
                                      (ID_EX[179:178] == 2'b01) ? ID_EX[36:5]  : // t_immediate_out_data - Updated to Reg Data from Pipileine ID_EX
                                      (ID_EX[179:178] == 2'b10) ? ID_EX[79:48] : // t_rs2_rdata          - Updated to Reg Data from Pipileine ID_EX
                                       ID_EX[79:48];                             // t_rs2_rdata          - Updated to Reg Data from Pipileine ID_EX

assign t_lui_auipc_mux_data         = (ID_EX[177:176] == 2'b00)? ID_EX[111:80] :  //Default      //o_rs1_rdata     Updated to Reg Data from Pipileine ID_EX
                                      (ID_EX[177:176] == 2'b01)? 32'b0 :          //LUI     
                                      (ID_EX[177:176] == 2'b10)? ID_EX[143:112] : //AUIPC        //PC_current_val  Updated to Reg Data from Pipileine ID_EX
                                      (ID_EX[177:176] == 2'b11)? ID_EX[175:144] : //Jal, Jalr    //t_pc_plus_4     Updated to Reg Data from Pipileine ID_EX
                                      ID_EX[111:80];                                                      //o_rs1_rdata     Updated to Reg Data from Pipileine ID_EX    

assign t_pc_o_rs1_data_mux_pcaddr   = (ID_EX[194]) ? (ID_EX[111:80]) :                                                        // o_rs1_rdata Updated to Reg Data from Pipeline ID_EX
                                       ID_EX[143:112];                                // Select o_rs1_data for jalr instruction else retain pc //PC_current_val  Updated to Reg Data from Pipeline ID_EX     

assign t_pc_o_rs1_data_mux_imm_add_EX_stage_data = t_pc_o_rs1_data_mux_pcaddr + t_immediate_out_data; //Connected to the MEM_EX Pipeline

///////////////////////////////////////////ALU Control Instance///////////////////////////////////////// 
alu_control alu_control_inst( 
    //.i_clu_alu_op(t_clu_alu_op), //Updated it to the Pipeline input
    .i_clu_alu_op(ID_EX[181:180]), // t_clu_alu_op
    //.i_instr_mem_inst(i_imem_rdata), //Removed as we dont have to sent the full instruction to the ALU control    
    .funct3(ID_EX[40:38]), // funct3 coming from the ID_EX Pipeline register
    .funct7(ID_EX[47:41]), // funct7 coming from the ID_EX Pipeline register
    .opcode_5thbit_add_sub(ID_EX[37]), // opcode5thbit coming from the ID_EX Pipeline register
    .o_alu_control_sel(o_alu_control_sel)
);
////////////////////////////////////////ALU Wrapper Instance/////////////////////////////////////////
alu_wrapper alu_wrapper_inst(
    .i_alu_ctrl_opsel(o_alu_control_sel),
    .i_rf_op1(t_lui_auipc_mux_data), //replaced it with Muxed out data from the 4:1 mux
    .i_rf_op2(rs2_rdata_imm_mux_data),
    //.i_clu_branch_instr_alu_sel(t_clu_branch_instr_alu_sel),
    .i_clu_branch_instr_alu_sel(ID_EX[183:182]), //t_clu_branch_instr_alu_sel
    //.o_alu_result(t_o_dmem_addr), 
    .o_alu_result(t_o_alu_result), //Redefined a new temp wire for connecting it to MEM_EX pipeline below
    .o_alu_Zero(t_alu_o_Zero) // Being sent to the MEM_EX Pipeline
);
//////////////////////////////////Mask Generation in the EX Stage/////////////////////////////////////
assign t_dmem_mask =    ((ID_EX[191:189] == 3'b000) || (ID_EX[191:189] == 3'b011) || (ID_EX[191:189] == 3'b101)) ? ( // For lb, lbu, sb - BYTE
                            (t_o_alu_result[1:0] == 2'b00) ? 4'b0001 : // Eg. 2000
                            (t_o_alu_result[1:0] == 2'b01) ? 4'b0010 : // Eg. 2001
                            (t_o_alu_result[1:0] == 2'b10) ? 4'b0100 : // Eg. 2002
                            (t_o_alu_result[1:0] == 2'b11) ? 4'b1000 : // Eg. 2003
                            4'bxxxx)
                        :
                        ((ID_EX[191:189] == 3'b001) || (ID_EX[191:189] == 3'b100) || (ID_EX[191:189] == 3'b110)) ? ( // For lh, lhu, sh - HALF
                            (t_o_alu_result[1:0] == 2'b00) ? 4'b0011 : // Eg. 2000
                            // (t_o_alu_result[1:0] == 2'b01) Eg. 2001 - Not a valid case for half
                            (t_o_alu_result[1:0] == 2'b10) ? 4'b1100 : // Eg. 2002
                            // (t_o_alu_result[1:0] == 2'b11) Eg. 2003 - Not a valid case for half
                            4'bxxxx)
                        :
                        4'b1111;                      

/////////////#######################################*#####################################################################################################################////////
/////////////#######################################*EX_MEM Pipeline Register Implementation*#######################################//////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////Assigning a 5 bit Wire to group all the Control Signal together for EX_MEM pipeline/////////
//////////{ t_sign_or_zero_ext_data_mux[2:0], t_rd_wen,    t_dmem_wen,  t_dmem_ren,  t_clu_MemtoReg, t_clu_branch}///////////////////
//////////{ EX_MEM[113:111],                  EX_MEM[110], EX_MEM[109], EX_MEM[108], EX_MEM[107],    EX_MEM[106]  }///////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
wire [7:0] Control_input_EX_MEM;
assign Control_input_EX_MEM = {ID_EX[188:186],ID_EX[195],ID_EX[193],ID_EX[192],ID_EX[185],ID_EX[184]}; // Mapped to the Signals as above description
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////|||{EX_MEM[113:106],             EX_MEM[105:74],                                EX_MEM[73],    EX_MEM[72:69],            EX_MEM[68:37],         EX_MEM[36:5],       EX_MEM[4:0]}|||||///
///      |||{Control_input_EX_MEM[7:0], t_pc_o_rs1_data_mux_imm_add_EX_stage_data[31:0], t_alu_o_Zero , t_dmem_mask[3:0]          t_o_alu_result[31:0],  t_rs2_rdata[31:0],  wr_addr[4:0]|||||///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
reg [113:0]EX_MEM;
wire [113:0]EX_MEM_temp;
assign EX_MEM_temp = {Control_input_EX_MEM[7:0],t_pc_o_rs1_data_mux_imm_add_EX_stage_data[31:0],t_alu_o_Zero,t_dmem_mask[3:0],t_o_alu_result[31:0],ID_EX[79:48],ID_EX[4:0]};
always @ (posedge i_clk) begin
    if (i_rst)
            EX_MEM <= 114'b0;
    else begin
            EX_MEM <= EX_MEM_temp;
    end
end
////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////AND Logic implemented in MEM Stage/////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
assign  t_alu_o_Zero_clu_Branch_and = EX_MEM[73] & EX_MEM[106]; // AND of t_clu_branch and t_alu_o_Zero
////////////////////////////////////////////////////////////////////////////////////////////////
assign t_pc_o_rs1_data_mux_imm_add_data =  EX_MEM[105:74]; // The PC + imm value executed in EX is Pipelined to MEM and is now being sent to Fetch (For Branch and Jumps)
////////////////////////////////////////////////////////////////////////////////////////////////
///////////Muxes - Changing the assignments to receive the data from MEM_EX pipeline///////////
assign o_dmem_wen = EX_MEM[109];
assign o_dmem_ren = EX_MEM[108];
assign o_dmem_addr = (EX_MEM[109] || EX_MEM[108]) ? {EX_MEM[68:39],2'b0} : EX_MEM[68:37]; //Only when w_en or ren is set we are to align the addresses //TODO : NEEDS an UPDATE with the MEM_EX  Pipleined out

// For Load instructions only
assign  i_dmem_rdata_sign_or_zero_ext_mux_data  =   (EX_MEM[72:69] == 4'b0001) && (EX_MEM[113:111] == 3'b001) ?  {{24{i_dmem_rdata[7]}},i_dmem_rdata[7:0]}         :   // lb and MASK = 4'b0001
                                                    (EX_MEM[72:69] == 4'b0001) && (EX_MEM[113:111] == 3'b000) ?  {24'b0,i_dmem_rdata[7:0]}                         :   // lbu and MASK = 4'b0001
                                                    (EX_MEM[72:69] == 4'b0010) && (EX_MEM[113:111] == 3'b001) ?  {{16{i_dmem_rdata[15]}},i_dmem_rdata[15:8],8'b0}  :   // lb and MASK = 4'b0010 - sign ext
                                                    (EX_MEM[72:69] == 4'b0010) && (EX_MEM[113:111] == 3'b000) ?  {16'b0,i_dmem_rdata[15:8],8'b0}                   :   // lbu and MASK = 4'b0010
                                                    (EX_MEM[72:69] == 4'b0100) && (EX_MEM[113:111] == 3'b001) ?  {{8{i_dmem_rdata[23]}},i_dmem_rdata[23:16],16'b0} :   // lb and MASK = 4'b0100 - sign ext
                                                    (EX_MEM[72:69] == 4'b0100) && (EX_MEM[113:111] == 3'b000) ?  {8'b0,i_dmem_rdata[23:16],16'b0}                  :   // lbu and MASK = 4'b0100
                                                    (EX_MEM[72:69] == 4'b1000) && (EX_MEM[113:111] == 3'b001) ?  {i_dmem_rdata[31:24],24'b0}                       :   // lb and MASK = 4'b1000
                                                    (EX_MEM[72:69] == 4'b1000) && (EX_MEM[113:111] == 3'b000) ?  {i_dmem_rdata[31:24],24'b0}                       :   // lbu and MASK = 4'b1000
                                                    (EX_MEM[72:69] == 4'b0011) && (EX_MEM[113:111] == 3'b011) ?  {{16{i_dmem_rdata[15]}},i_dmem_rdata[15:0]}       :   // lh and MASK = 4'b0011
                                                    (EX_MEM[72:69] == 4'b0011) && (EX_MEM[113:111] == 3'b010) ?  {16'b0,i_dmem_rdata[15:0]}                        :   // lhu and MASK = 4'b0011
                                                    (EX_MEM[72:69] == 4'b1100) && (EX_MEM[113:111] == 3'b011) ?  {{16{i_dmem_rdata[31]}},i_dmem_rdata[31:16]}      :   // lh and MASK = 4'b1100
                                                    (EX_MEM[72:69] == 4'b1100) && (EX_MEM[113:111] == 3'b010) ?  {16'b0,i_dmem_rdata[31:16]}                       :   // lhu and MASK = 4'b1100
                                                    (EX_MEM[72:69] == 4'b1111) && (EX_MEM[113:111] == 3'b100) ?  i_dmem_rdata                                      :   // lw and MASK = 4'b1111
                                                                                i_dmem_rdata ;
 
assign f_rs2_rdata  = EX_MEM[36:5]; // The Pipelined o_rs2_data coming from REG for STORE instruction
// For store instructions only 
assign o_dmem_wdata =   (EX_MEM[72:69] == 4'b0001) ? {{24{f_rs2_rdata[7]}},f_rs2_rdata[7:0]} :                   // Applicable for sb
                        (EX_MEM[72:69] == 4'b0010) ? {{16{f_rs2_rdata[15]}},f_rs2_rdata[15:8],8'b0} :             // Applicable for sb
                        (EX_MEM[72:69] == 4'b0100) ? {{8{f_rs2_rdata[23]}},f_rs2_rdata[23:16],16'b0} :            // Applicable for sb
                        (EX_MEM[72:69] == 4'b1000) ? {f_rs2_rdata[7:0],24'b0} :                   // Applicable for sb
                        (EX_MEM[72:69] == 4'b0011) ? {{16{f_rs2_rdata[15]}},f_rs2_rdata[15:0]} :  // Applicable for sh - SIGN EXTENSION by default
                        (EX_MEM[72:69] == 4'b1100) ? {f_rs2_rdata[15:0],16'b0} :                  // Applicable for sh
                        (EX_MEM[72:69] == 4'b1111) ? f_rs2_rdata :  
                        32'bxxxx;                                                               // Default case

//MASK Implementation
assign o_dmem_mask = EX_MEM[72:69]; // Changed it to value coming from MEM_EX Pipeline - t_dmem_mask

/////////############################################################################################################################################################////////
////////////#######################################*MEM_WB Pipeline Register Implementation*#######################################//////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////Assigning a 2 bit Wire to group all the Control Signal together for MEM_WB pipeline/////////
//////////{      t_rd_wen,         t_clu_MemtoReg  }/////////////////////////////////////////////
//////////{ (MEM_WB[70]/EX_MEM[110], MEM_WB[69]/EX_MEM[107]) }/////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
wire [1:0] Control_input_MEM_WB;
assign Control_input_MEM_WB = {EX_MEM[110],EX_MEM[107]}; // Mapped to the Signals as in above description
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////|||{MEM_WB[70:69],              MEM_WB[68:37],                                   MEM_WB[36:5],                      ,        MEM_WB[4:0]}|||||//////////////////
///      |||{Control_input_MEM_WB[1:0],  i_dmem_rdata_sign_or_zero_ext_mux_data[31:0]    (EX_MEM[68:37]/t_o_alu_result[31:0]),  (EX_MEM[4:0]/wr_addr[4:0])|||||//////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
reg [70:0]MEM_WB;
wire [70:0]MEM_WB_temp;
assign MEM_WB_temp = {Control_input_MEM_WB[1:0],i_dmem_rdata_sign_or_zero_ext_mux_data[31:0],EX_MEM[68:37],EX_MEM[4:0]};
always @ (posedge i_clk) begin
    if (i_rst)
            MEM_WB <= 71'b0;
    else begin
            MEM_WB <= MEM_WB_temp;
    end
end
/////////////////THIS WILL BE IN THE LAST STAGE OF THE PIPELINE///////////////////////////
assign i_dmem_alu_muxout_data                     =    MEM_WB[69] ? MEM_WB[68:37] : MEM_WB[36:5]; // The Data going to be written to the Register - Either the Alu result or Data Read from Memory in case of Load instruction
assign t_i_rd_waddr                               =    MEM_WB[4:0]; // Pipeline Register Write Address (Source Instruction)
assign t_i_rd_wen                                 =    MEM_WB[70]; // Pipelined Register Write Enable  (Source Control Unit)
////////////////////////////////////////////////////////////////////////////////////////////////

endmodule



































////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

//assign t_pc_o_rs1_data_mux_pcaddr = (t_clu_pc_o_rs1_data_mux_sel) ? (o_rs1_rdata) :  PC_current_val; // Select o_rs1_data for jalr instruction else retain pc        
// assign rs2_rdata_imm_mux_data       = (t_clu_ALUSrc == 2'b00) ? 32'b0 : 
//                                       (t_clu_ALUSrc == 2'b01) ? ID_EX[36:5]  : // t_immediate_out_data - Updated to Reg Data from Pipileine ID_EX
//                                       (t_clu_ALUSrc == 2'b10) ? ID_EX[79:48] : // t_rs2_rdata          - Updated to Reg Data from Pipileine ID_EX
//                                        ID_EX[79:48];                           // t_rs2_rdata          - Updated to Reg Data from Pipileine ID_EX

// assign t_lui_auipc_mux_data         = (t_clu_lui_auipc_mux_sel == 2'b00)? ID_EX[111:80] :  //Default      //o_rs1_rdata     Updated to Reg Data from Pipileine ID_EX
//                                       (t_clu_lui_auipc_mux_sel == 2'b01)? 32'b0 :          //LUI     
//                                       (t_clu_lui_auipc_mux_sel == 2'b10)? ID_EX[143:112] : //AUIPC        //PC_current_val  Updated to Reg Data from Pipileine ID_EX
//                                       (t_clu_lui_auipc_mux_sel == 2'b11)? ID_EX[175:144] : //Jal, Jalr    //t_pc_plus_4     Updated to Reg Data from Pipileine ID_EX
//                                       ID_EX[111:80];                                                      //o_rs1_rdata     Updated to Reg Data from Pipileine ID_EX    

// assign t_pc_o_rs1_data_mux_pcaddr   = (t_clu_pc_o_rs1_data_mux_sel) ? (ID_EX[111:80]) :                                                        // o_rs1_rdata Updated to Reg Data from Pipeline ID_EX
//                                        ID_EX[143:112];                                // Select o_rs1_data for jalr instruction else retain pc //PC_current_val  Updated to Reg Data from Pipeline ID_EX     



// assign t_dmem_mask =    ((t_clu_ld_st_type_sel == 3'b000) || (t_clu_ld_st_type_sel == 3'b011) || (t_clu_ld_st_type_sel == 3'b101)) ? ( // For lb, lbu, sb - BYTE
//                             (t_o_dmem_addr[1:0] == 2'b00) ? 4'b0001 : // Eg. 2000
//                             (t_o_dmem_addr[1:0] == 2'b01) ? 4'b0010 : // Eg. 2001
//                             (t_o_dmem_addr[1:0] == 2'b10) ? 4'b0100 : // Eg. 2002
//                             (t_o_dmem_addr[1:0] == 2'b11) ? 4'b1000 : // Eg. 2003
//                             4'bxxxx)
//                         :
//                         ((t_clu_ld_st_type_sel == 3'b001) || (t_clu_ld_st_type_sel == 3'b100) || (t_clu_ld_st_type_sel == 3'b110)) ? ( // For lh, lhu, sh - HALF
//                             (t_o_dmem_addr[1:0] == 2'b00) ? 4'b0011 : // Eg. 2000
//                             // (t_o_dmem_addr[1:0] == 2'b01) Eg. 2001 - Not a valid case for half
//                             (t_o_dmem_addr[1:0] == 2'b10) ? 4'b1100 : // Eg. 2002
//                             // (t_o_dmem_addr[1:0] == 2'b11) Eg. 2003 - Not a valid case for half
//                             4'bxxxx)
//                         :
//                         4'b1111;    

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

// //  Muxes
// assign o_dmem_wen = t_dmem_wen;
// assign o_dmem_ren = t_dmem_ren;
// assign o_dmem_addr = (t_dmem_wen || t_dmem_ren) ? {t_o_dmem_addr[31:2],2'b0} : t_o_dmem_addr; //Only when w_en or ren is set we are to align the addresses //TODO : NEEDS an UPDATE with the MEM_EX  Pipleined out
// assign i_dmem_alu_muxout_data                   =   t_clu_MemtoReg ? i_dmem_rdata_sign_or_zero_ext_mux_data : o_dmem_addr;

// // For Load instructions only
// assign  i_dmem_rdata_sign_or_zero_ext_mux_data  =   (t_dmem_mask == 4'b0001) && (t_sign_or_zero_ext_data_mux == 3'b001) ?  {{24{i_dmem_rdata[7]}},i_dmem_rdata[7:0]}         :   // lb and MASK = 4'b0001
//                                                     (t_dmem_mask == 4'b0001) && (t_sign_or_zero_ext_data_mux == 3'b000) ?  {24'b0,i_dmem_rdata[7:0]}                         :   // lbu and MASK = 4'b0001
//                                                     (t_dmem_mask == 4'b0010) && (t_sign_or_zero_ext_data_mux == 3'b001) ?  {{16{i_dmem_rdata[15]}},i_dmem_rdata[15:8],8'b0}  :   // lb and MASK = 4'b0010 - sign ext
//                                                     (t_dmem_mask == 4'b0010) && (t_sign_or_zero_ext_data_mux == 3'b000) ?  {16'b0,i_dmem_rdata[15:8],8'b0}                   :   // lbu and MASK = 4'b0010
//                                                     (t_dmem_mask == 4'b0100) && (t_sign_or_zero_ext_data_mux == 3'b001) ?  {{8{i_dmem_rdata[23]}},i_dmem_rdata[23:16],16'b0} :   // lb and MASK = 4'b0100 - sign ext
//                                                     (t_dmem_mask == 4'b0100) && (t_sign_or_zero_ext_data_mux == 3'b000) ?  {8'b0,i_dmem_rdata[23:16],16'b0}                  :   // lbu and MASK = 4'b0100
//                                                     (t_dmem_mask == 4'b1000) && (t_sign_or_zero_ext_data_mux == 3'b001) ?  {i_dmem_rdata[31:24],24'b0}                       :   // lb and MASK = 4'b1000
//                                                     (t_dmem_mask == 4'b1000) && (t_sign_or_zero_ext_data_mux == 3'b000) ?  {i_dmem_rdata[31:24],24'b0}                       :   // lbu and MASK = 4'b1000
//                                                     (t_dmem_mask == 4'b0011) && (t_sign_or_zero_ext_data_mux == 3'b011) ?  {{16{i_dmem_rdata[15]}},i_dmem_rdata[15:0]}       :   // lh and MASK = 4'b0011
//                                                     (t_dmem_mask == 4'b0011) && (t_sign_or_zero_ext_data_mux == 3'b010) ?  {16'b0,i_dmem_rdata[15:0]}                        :   // lhu and MASK = 4'b0011
//                                                     (t_dmem_mask == 4'b1100) && (t_sign_or_zero_ext_data_mux == 3'b011) ?  {{16{i_dmem_rdata[31]}},i_dmem_rdata[31:16]}      :   // lh and MASK = 4'b1100
//                                                     (t_dmem_mask == 4'b1100) && (t_sign_or_zero_ext_data_mux == 3'b010) ?  {16'b0,i_dmem_rdata[31:16]}                       :   // lhu and MASK = 4'b1100
//                                                     (t_dmem_mask == 4'b1111) && (t_sign_or_zero_ext_data_mux == 3'b100) ?  i_dmem_rdata                                      :   // lw and MASK = 4'b1111
//                                                                                 i_dmem_rdata ;
 
// // For store instructions only 
// assign o_dmem_wdata =   (t_dmem_mask == 4'b0001) ? {{24{t_rs2_rdata[7]}},t_rs2_rdata[7:0]} :                   // Applicable for sb
//                         (t_dmem_mask == 4'b0010) ? {{16{t_rs2_rdata[15]}},t_rs2_rdata[15:8],8'b0} :             // Applicable for sb
//                         (t_dmem_mask == 4'b0100) ? {{8{t_rs2_rdata[23]}},t_rs2_rdata[23:16],16'b0} :            // Applicable for sb
//                         (t_dmem_mask == 4'b1000) ? {t_rs2_rdata[7:0],24'b0} :                   // Applicable for sb
//                         (t_dmem_mask == 4'b0011) ? {{16{t_rs2_rdata[15]}},t_rs2_rdata[15:0]} :  // Applicable for sh - SIGN EXTENSION by default
//                         (t_dmem_mask == 4'b1100) ? {t_rs2_rdata[15:0],16'b0} :                  // Applicable for sh
//                         (t_dmem_mask == 4'b1111) ? t_rs2_rdata :  
//                         32'bxxxx;                                                               // Default case




////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

//assign o_dmem_mask = t_dmem_mask; // This should be a coming out of the MEM_EX Pipeline

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

//assign i_dmem_alu_muxout_data                   =   t_clu_MemtoReg ? i_dmem_rdata_sign_or_zero_ext_mux_data : o_dmem_addr;