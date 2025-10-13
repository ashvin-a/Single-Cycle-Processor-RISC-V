//Who will control the PC? - What is the signal input fromt he control Unit?
//Shall we assume PC will have 0 as the address location value?
module fetch (
    input  wire clk,
    input  wire rst_n,
    input  wire i_clu_branch,
    input  wire i_alu_o_slt,
    input  wire [31:0]i_imm_o_immediate,
    output wire [31:0]o_instr_mem_rd_addr // read address is 32 bits and not 5 bits
);
    reg [31:0] PC;
    reg [31:0]pc_imm_mux_val;
    ///[Q] : Should it be o_slt or o_eq?
    assign pc_imm_mux_val = (i_clu_branch & i_alu_o_slt)? (PC + {{i_imm_o_immediate[31:1],1'b0}}) : (PC + 4) ; 
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

module control_unit(
    input  wire clk,
    input  wire rst_n,
    input  wire [31:0] i_clu_inst,
    output wire o_clu_Branch,
    output wire o_clu_MemRead,
    output wire o_clu_MemtoReg,
    output wire o_clu_ALUOp,
    output wire o_clu_MemWrite,
    output wire o_clu_ALUSrc,
    output wire o_clu_RegWrite
);

endmodule

module alu_control(input  wire [1:0]  i_clu_alu_op,
                   input  wire [31:0] i_instr_mem_inst,
                   output wire [3:0]  o_alu_control);
endmodule

// The arithmetic logic unit (ALU) is responsible for performing the core
// calculations of the processor. It takes two 32-bit operands and outputs
// a 32 bit result based on the selection operation - addition, comparison,
// shift, or logical operation. This ALU is a purely combinational block, so
// you should not attempt to add any registers or pipeline it in phase 3.
module alu (
    // Major operation selection.
    // NOTE: In order to simplify instruction decoding in phase 4, both 3'b010
    // and 3'b011 are used for set less than (they are equivalent).
    // Unsigned comparison is controlled through the `i_unsigned` signal.
    //
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
    output wire        o_slt, // BLT and BLTU
   // Set Not Equal result. This is used downstream to determine if a
    // branch should be taken.
    output wire        o_sne,  // BNE
   // Set Greater than result. This is used downstream to determine if a
    // branch should be taken.
    output wire        o_sge  // BGE and BGEU

);

    wire [4:0]temp = i_op2[4:0];
    wire [31:0] sll = i_op1 << temp;
    wire [31:0] srl = i_op1 >> temp;
    wire [31:0] sra = $signed(i_op1) >>> temp;
    wire [31:0] slt_signed = ($signed(i_op1) < $signed(i_op2)) ? 32'b1 : 32'b0;
    wire [31:0] slt_unsigned = (i_op1 < i_op2) ? 32'b1 : 32'b0;

    assign o_eq  = (i_op1 == i_op2);
    assign o_slt =  i_unsigned ? (i_op1 <  i_op2) : ($signed(i_op1) <  $signed(i_op2));
    assign o_sne =  i_unsigned ? (i_op1 != i_op2) : ($signed(i_op1) != $signed(i_op2));
    assign o_sge =  i_unsigned ? (i_op1 >= i_op2) : ($signed(i_op1) >= $signed(i_op2));
    assign o_result =       (i_opsel == 3'b000)? ((i_sub) ? (i_op1 - i_op2) : (i_op1 + i_op2)) :
                            (i_opsel == 3'b001)? (i_op1 << temp) : 
                            ((i_opsel == 3'b010) || (i_opsel == 3'b011)) ? (i_unsigned ? slt_unsigned : slt_signed) : 
                            (i_opsel == 3'b100)? (i_op1 ^ i_op2) :  
                            (i_opsel == 3'b101)? ((i_arith)? sra : srl) : 
                            (i_opsel == 3'b110)? (i_op1 | i_op2) : 
                            (i_opsel == 3'b111)? (i_op1 & i_op2) : 
                            32'h0;

endmodule

module alu_wrapper (
    // 4 bit input from ALU Control block
    input  wire [ 3:0] i_alu_ctrl_opsel,
    // First 32-bit input operand.
    input  wire [31:0] i_rf_op1,
    // Second 32-bit input operand.
    input  wire [31:0] i_rf_op2,
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
    wire        o_slt;  // BLT and BLTU
    wire        o_sne;  // BNE
    wire        o_sge;  // BGE and BGEU
    alu (
        .i_opsel(i_opsel),
        .i_sub(i_sub),
        .i_unsigned(i_unsigned),
        .i_arith(i_arith),
        .i_op1(i_rf_op1),
        .i_op2(i_rf_op2),
        .o_result(o_alu_result),
        .o_eq(o_eq),
        .o_slt(o_slt),
        .o_sne(o_sne),
        .o_sge(o_sge)
    );
    ////////////////////////////////////////////
    //////////INPUTS TO THE INSIDE ALU//////////
    ////////////////////////////////////////////
    assign i_unsigned = (i_alu_ctrl_opsel == 4'b1001)? 1'b1 : 1'b0;
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
                        (i_alu_ctrl_opsel == 4'b1000)? 3'b010 :    //SLT   - 010 and 011
                        (i_alu_ctrl_opsel == 4'b1001)? 3'b010 :    //SLTU   
                        (i_alu_ctrl_opsel == 4'b1010)? 3'b011 :    //PASS_B - LUI   - Can I use 011 for LUI?
                        3'bXXX; //Don't care                               
    ////////////////////////////////////////////
    /////////////////OUTPUTS////////////////////
    ////////////////////////////////////////////
    assign o_alu_Zero  = o_eq | o_slt | o_sne | o_sge ; 
endmodule

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
    // `0x00002003`, assert `o_dmem_wen`, and set the mask to 0b1000 to
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
    // Fill in your implementation here.
/////////////////////////////////////////////////////////////
///////////////////REG FILE Instance/////////////////////////
/////////////////////////////////////////////////////////////
                //BYPASS Not enabled//
rf #(.BYPASS_EN(0)) reg_inst(
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_rs1_raddr(i_imem_rdata[19:15]),
    .i_rs2_raddr(i_imem_rdata[24:20]),
    .i_rd_waddr(i_imem_rdata[11:7]),
    .i_rd_wen(i_clu_RegWrite), //Input from CLU
    .i_rd_wdata(i_dmem_alu_muxout_data),   //Muxed out data of ALU and Data Mem
    .o_rs1_rdata(o_reg_rd1_rdata), //Input to the ALU
    .o_rs2_rdata(o_reg_rd2_imm_mux_alu_in2_data)  //Input to the reg_imm_alu_in2_mux
);

//IMM instruction to i_format decoder
wire [5:0]i_imm_format;
assign i_imm_format =   (i_imem_rdata[6:0] == 011_0011)? 6'b000_001 : //R
                    (i_imem_rdata[6:0] == 001_0011)? 6'b000_010 ://I Type
                    (i_imem_rdata[6:0] == 000_0011)? 6'b000_010 ://I
                    (i_imem_rdata[6:0] == 010_0011)? 6'b000_100 ://S
                    (i_imem_rdata[6:0] == 110_0011)? 6'b001_000 ://B
                    (i_imem_rdata[6:0] == 011_0111)? 6'b010_000 ://U
                    (i_imem_rdata[6:0] == 110_1111)? 6'b100_000 ://J
                    6'bX;

//IMM Gen Instance
imm imm_decode_inst(
        .i_inst(i_imem_rdata),
        .i_format(i_imm_format),
        .o_immediate(o_immediate)
);

    //Fetch section instance

    //ALU Instance

    //Main control instance

    //ALU Control Instance



endmodule

`default_nettype wire
