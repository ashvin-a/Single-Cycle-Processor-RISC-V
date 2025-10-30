// Retire Valid has to come from PC/fetch - infact i guess all of them
// To connect the RETIRE TRACE OUT
// Check all the Dont care cases I have defined - is it ok to define Dont care


`timescale 1ns/1ps

/////////////////////////// FETCH MODULE ///////////////////////////

module fetch (
    input  wire clk,
    input  wire rst_n,
    input  wire i_clu_branch,                       // Updates the PC accordingly when there is a branch instruction
    input  wire i_clu_halt,                         // When high the program has completed excecution and the PC stops updating
    input  wire i_alu_o_Zero,                       // Needed for deciding between JALR+Branch and Other instructions
    input  wire [31:0] i_pc_o_rs1_data_mux_pcaddr,  // Take o_rs1 for JALR instruction and PC for others
    input  wire [31:0]i_imm_o_immediate,
    output wire [31:0]o_instr_mem_rd_addr,          // Read address is 32 bits and not 5 bits
    output wire [31:0]o_pc_plus_4,
    output reg  [31:0] PC                           // Program Counter register
);
    wire [31:0]pc_imm_mux_val;
    assign pc_imm_mux_val = (i_clu_branch & i_alu_o_Zero)? (i_pc_o_rs1_data_mux_pcaddr + i_imm_o_immediate) : (PC + 4) ; // Selecting the next value of PC to be updated after this fetch cycle according if breanch has to happen
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
    output wire o_clu_halt,
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
                    input  wire [31:0] i_instr_mem_inst,
                    output wire [3:0]  o_alu_control_sel
                    );

wire [5:0] op_code;
wire [2:0] funct3;
wire [6:0] funct7;
wire opcode_5thbit_add_sub;

assign funct3  = i_instr_mem_inst[14:12];
assign funct7  = i_instr_mem_inst[31:25];
assign op_code = i_instr_mem_inst[6:0];
assign opcode_5thbit_add_sub = i_instr_mem_inst[5];

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
wire t_clu_MemtoReg, i_clu_branch;
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

assign o_retire_halt      = t_clu_halt;
assign o_retire_valid     = 1;
assign o_retire_inst      =   i_imem_rdata;         
assign o_retire_trap      =   0; //Temporary assignment - Need to be modified         
assign o_retire_rs1_raddr =   i_imem_rdata[19:15];         
assign o_retire_rs1_rdata =   o_rs1_rdata;         
assign o_retire_rs2_raddr =   i_imem_rdata[24:20];         
assign o_retire_rs2_rdata =   t_rs2_rdata;         
assign o_retire_rd_waddr  =   t_rd_wen?i_imem_rdata[11:7]:5'b0;         
assign o_retire_rd_wdata  =   i_dmem_alu_muxout_data;        
assign o_retire_pc        =   PC_current_val;         
assign o_retire_next_pc   =   PC_current_val + 4;   //Need to be modified based on Branch and Jump instructions   

// Immediate format decoding
assign i_imm_format =   
    (i_imem_rdata[6:0] == 7'b0110011)? 6'b000001 : // R
    ((i_imem_rdata[6:0] == 7'b0010011)  || (i_imem_rdata[6:0] == 7'b1100111))? 6'b000010 : // I and Jalr
    (i_imem_rdata[6:0] == 7'b0000011)? 6'b000010 : // I (Load)
    (i_imem_rdata[6:0] == 7'b0100011)? 6'b000100 : // S
    (i_imem_rdata[6:0] == 7'b1100011)? 6'b001000 : // B
    ((i_imem_rdata[6:0] == 7'b0110111) || (i_imem_rdata[6:0] == 7'b0010111))? 6'b010000 : // U
    (i_imem_rdata[6:0] == 7'b1101111)? 6'b100000 : // Jal
    6'bXXXXXX;


// DUT Instantiations

// Register File
rf #(.BYPASS_EN(0)) rf(
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_rs1_raddr(i_imem_rdata[19:15]),
    .i_rs2_raddr(i_imem_rdata[24:20]),
    .i_rd_waddr(i_imem_rdata[11:7]),
    .i_rd_wen(t_rd_wen),
    .i_rd_wdata(i_dmem_alu_muxout_data),
    .o_rs1_rdata(o_rs1_rdata),
    .o_rs2_rdata(t_rs2_rdata)
);
 
// Immediate Generator

imm imm_decode_inst(
    .i_inst(i_imem_rdata),
    .i_format(i_imm_format),
    .o_immediate(t_immediate_out_data)
);

// Fetch Section
fetch fetch_inst(
    .clk(i_clk),
    .rst_n(i_rst),
    .i_clu_branch(i_clu_branch),
    .i_clu_halt(t_clu_halt),
    .i_alu_o_Zero(t_alu_o_Zero),
    .i_imm_o_immediate(t_immediate_out_data),
    .i_pc_o_rs1_data_mux_pcaddr(t_pc_o_rs1_data_mux_pcaddr),
    .PC(PC_current_val),
    .o_pc_plus_4(t_pc_plus_4),
    .o_instr_mem_rd_addr(o_imem_raddr)
);


// Control Unit
control_unit control_unit_inst(
    .clk(i_clk),
    .rst_n(i_rst),
    .i_clu_inst(i_imem_rdata),
    .o_clu_Branch(i_clu_branch),
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

//////////*ID_EX Pipeline Register Implementation////////
//! Im not considering the signals that are'nt passed onto the ID_EX pipeline.
//? 5 + (32 + 32 + 32) + (32 + 32) + (32 + 32 + 32) + (32 + 32) + ctrl_signal_bit{2 + 2 + 2 + 1 + 1 + 1 + 1 + 1}
reg [335:0]ID_EX;
wire [335:0]ID_EX_temp;
assign ID_EX_temp = {i_imem};
//Implementing the synchronous logics
always @ (posedge i_clk) begin
    //Reset logic of register - Synchronous Active High i_rst
    if (i_rst)
            mem[i] <= 336'b0;
    //Synchronous write when the write en is high
    else begin
            ID_EX <= ID_EX_temp;
    end
end

// ALU Control
alu_control alu_control_inst( 
    .i_clu_alu_op(t_clu_alu_op),
    .i_instr_mem_inst(i_imem_rdata),
    .o_alu_control_sel(o_alu_control_sel)
);

// ALU
alu_wrapper alu_wrapper_inst(
    .i_alu_ctrl_opsel(o_alu_control_sel),
    .i_rf_op1(t_lui_auipc_mux_data), //replaced it with Muxed out data from the 4:1 mux
    .i_rf_op2(rs2_rdata_imm_mux_data),
    .i_clu_branch_instr_alu_sel(t_clu_branch_instr_alu_sel),
    .o_alu_result(t_o_dmem_addr),
    .o_alu_Zero(t_alu_o_Zero)
);



//  Muxes
assign o_dmem_wen = t_dmem_wen;
assign o_dmem_ren = t_dmem_ren;
assign o_dmem_addr = (t_dmem_wen || t_dmem_ren) ? {t_o_dmem_addr[31:2],2'b0} : t_o_dmem_addr; //Only when w_en or ren is set we are to align the addresses
assign i_dmem_alu_muxout_data                   =   t_clu_MemtoReg ? i_dmem_rdata_sign_or_zero_ext_mux_data : o_dmem_addr;
assign rs2_rdata_imm_mux_data                   =   (t_clu_ALUSrc == 2'b00) ? 32'b0 : 
                                                    (t_clu_ALUSrc == 2'b01) ? t_immediate_out_data :
                                                    (t_clu_ALUSrc == 2'b10) ? t_rs2_rdata :
                                                    t_rs2_rdata;
assign t_lui_auipc_mux_data                     =   (t_clu_lui_auipc_mux_sel == 2'b00)? o_rs1_rdata :    //Default
                                                    (t_clu_lui_auipc_mux_sel == 2'b01)? 32'b0 :          //LUI
                                                    (t_clu_lui_auipc_mux_sel == 2'b10)? PC_current_val : //AUIPC
                                                    (t_clu_lui_auipc_mux_sel == 2'b11)? t_pc_plus_4 :    //Jal, Jalr
                                                    o_rs1_rdata;
assign t_pc_o_rs1_data_mux_pcaddr                 = (t_clu_pc_o_rs1_data_mux_sel) ? (o_rs1_rdata) :  PC_current_val; // Select o_rs1_data for jalr instruction else retain pc        

//MASK Implementation
assign o_dmem_mask = t_dmem_mask;

assign t_dmem_mask =    ((t_clu_ld_st_type_sel == 3'b000) || (t_clu_ld_st_type_sel == 3'b011) || (t_clu_ld_st_type_sel == 3'b101)) ? ( // For lb, lbu, sb - BYTE
                            (t_o_dmem_addr[1:0] == 2'b00) ? 4'b0001 : // Eg. 2000
                            (t_o_dmem_addr[1:0] == 2'b01) ? 4'b0010 : // Eg. 2001
                            (t_o_dmem_addr[1:0] == 2'b10) ? 4'b0100 : // Eg. 2002
                            (t_o_dmem_addr[1:0] == 2'b11) ? 4'b1000 : // Eg. 2003
                            4'bxxxx)
                        :
                        ((t_clu_ld_st_type_sel == 3'b001) || (t_clu_ld_st_type_sel == 3'b100) || (t_clu_ld_st_type_sel == 3'b110)) ? ( // For lh, lhu, sh - HALF
                            (t_o_dmem_addr[1:0] == 2'b00) ? 4'b0011 : // Eg. 2000
                            // (t_o_dmem_addr[1:0] == 2'b01) Eg. 2001 - Not a valid case for half
                            (t_o_dmem_addr[1:0] == 2'b10) ? 4'b1100 : // Eg. 2002
                            // (t_o_dmem_addr[1:0] == 2'b11) Eg. 2003 - Not a valid case for half
                            4'bxxxx)
                        :
                        4'b1111;                      

// For Load instructions only
assign  i_dmem_rdata_sign_or_zero_ext_mux_data  =   (t_dmem_mask == 4'b0001) && (t_sign_or_zero_ext_data_mux == 3'b001) ?  {{24{i_dmem_rdata[7]}},i_dmem_rdata[7:0]}         :   // lb and MASK = 4'b0001
                                                    (t_dmem_mask == 4'b0001) && (t_sign_or_zero_ext_data_mux == 3'b000) ?  {24'b0,i_dmem_rdata[7:0]}                         :   // lbu and MASK = 4'b0001
                                                    (t_dmem_mask == 4'b0010) && (t_sign_or_zero_ext_data_mux == 3'b001) ?  {{16{i_dmem_rdata[15]}},i_dmem_rdata[15:8],8'b0}  :   // lb and MASK = 4'b0010 - sign ext
                                                    (t_dmem_mask == 4'b0010) && (t_sign_or_zero_ext_data_mux == 3'b000) ?  {16'b0,i_dmem_rdata[15:8],8'b0}                   :   // lbu and MASK = 4'b0010
                                                    (t_dmem_mask == 4'b0100) && (t_sign_or_zero_ext_data_mux == 3'b001) ?  {{8{i_dmem_rdata[23]}},i_dmem_rdata[23:16],16'b0} :   // lb and MASK = 4'b0100 - sign ext
                                                    (t_dmem_mask == 4'b0100) && (t_sign_or_zero_ext_data_mux == 3'b000) ?  {8'b0,i_dmem_rdata[23:16],16'b0}                  :   // lbu and MASK = 4'b0100
                                                    (t_dmem_mask == 4'b1000) && (t_sign_or_zero_ext_data_mux == 3'b001) ?  {i_dmem_rdata[31:24],24'b0}                       :   // lb and MASK = 4'b1000
                                                    (t_dmem_mask == 4'b1000) && (t_sign_or_zero_ext_data_mux == 3'b000) ?  {i_dmem_rdata[31:24],24'b0}                       :   // lbu and MASK = 4'b1000
                                                    (t_dmem_mask == 4'b0011) && (t_sign_or_zero_ext_data_mux == 3'b011) ?  {{16{i_dmem_rdata[15]}},i_dmem_rdata[15:0]}       :   // lh and MASK = 4'b0011
                                                    (t_dmem_mask == 4'b0011) && (t_sign_or_zero_ext_data_mux == 3'b010) ?  {16'b0,i_dmem_rdata[15:0]}                        :   // lhu and MASK = 4'b0011
                                                    (t_dmem_mask == 4'b1100) && (t_sign_or_zero_ext_data_mux == 3'b011) ?  {{16{i_dmem_rdata[31]}},i_dmem_rdata[31:16]}      :   // lh and MASK = 4'b1100
                                                    (t_dmem_mask == 4'b1100) && (t_sign_or_zero_ext_data_mux == 3'b010) ?  {16'b0,i_dmem_rdata[31:16]}                       :   // lhu and MASK = 4'b1100
                                                    (t_dmem_mask == 4'b1111) && (t_sign_or_zero_ext_data_mux == 3'b100) ?  i_dmem_rdata                                      :   // lw and MASK = 4'b1111
                                                                                i_dmem_rdata ;
 
// For store instructions only 
// Do we need any sign extension or zero extension for this data to be written? - NEED TO DO
// Dont we have to maintain the value of the other bytes than the byte that we are writting to in the memory
assign o_dmem_wdata =   (t_dmem_mask == 4'b0001) ? {{24{t_rs2_rdata[7]}},t_rs2_rdata[7:0]} :                   // Applicable for sb
                        (t_dmem_mask == 4'b0010) ? {{16{t_rs2_rdata[15]}},t_rs2_rdata[15:8],8'b0} :             // Applicable for sb
                        (t_dmem_mask == 4'b0100) ? {{8{t_rs2_rdata[23]}},t_rs2_rdata[23:16],16'b0} :            // Applicable for sb
                        (t_dmem_mask == 4'b1000) ? {t_rs2_rdata[7:0],24'b0} :                   // Applicable for sb
                        (t_dmem_mask == 4'b0011) ? {{16{t_rs2_rdata[15]}},t_rs2_rdata[15:0]} :  // Applicable for sh - SIGN EXTENSION by default
                        (t_dmem_mask == 4'b1100) ? {t_rs2_rdata[15:0],16'b0} :                  // Applicable for sh
                        (t_dmem_mask == 4'b1111) ? t_rs2_rdata :  
                        32'bxxxx;                                                               // Default case

endmodule


// To - do's
// Trap signal implementation
// Valid signal implementation
// Review architecture with George
// Review verilog with Eric