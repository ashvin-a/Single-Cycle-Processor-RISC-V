//Who will control the PC? - What is the signal input fromt he control Unit?
//Shall we assume PC will have 0 as the address location value?
module fetch (
    input  wire clk,
    input  wire rst_n,
    input  wire i_clu_branch,
    input  wire i_alu_o_Zero,
    input  wire [31:0]i_imm_o_immediate,
    output wire [31:0]o_instr_mem_rd_addr // read address is 32 bits and not 5 bits
);
    reg [31:0] PC;
    wire [31:0]pc_imm_mux_val;
    ///[Q] : Should it be o_slt or o_eq?
    assign pc_imm_mux_val = (i_clu_branch & i_alu_o_Zero)? (PC + {{i_imm_o_immediate[31:1],1'b0}}) : (PC + 4) ; 
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
    output wire o_clu_RegWrite,
    output wire o_clu_dmem_mask
);

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
    //Aluctrl unsigned chk input.
    input wire i_aluctrl_unsigned,
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
    alu alu_inst(
        .i_opsel(i_opsel),
        .i_sub(i_sub),
        .i_unsigned(i_aluctrl_unsigned),
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
    //assign i_unsigned = (i_alu_ctrl_opsel == 4'b1001)? 1'b1 : 1'b0;

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


module alu_control( input  wire [1:0]  i_clu_alu_op,
                    input  wire [31:0] i_instr_mem_inst,
                    output wire o_unsigned,
                    output wire [3:0]  o_alu_control_sel
                    );

wire [5:0] op_code;
wire [3:0] decode;
wire [2:0] funct3;
wire [6:0] funct7;

assign funct3 = i_instr_mem_inst[14:12];
assign funct7 = i_instr_mem_inst[31:25];
assign op_code = i_instr_mem_inst[6:0];

assign o_unsigned = 
    ((funct3 == 3'b011) && ((op_code == 7'b0110011) || (op_code == 7'b0010011))) ||
    ((op_code == 7'b0000011) && ((funct3 == 3'b100) || (funct3 == 3'b101))) ||
    ((op_code == 7'b1100011) && ((funct3 == 3'b110) || (funct3 == 3'b111)));

assign decode = (op_code == 7'b0110011) ? 4'b0000: // R type
                (op_code == 7'b0010011) ? 4'b0001: // I type
                (op_code == 7'b0110111) ? 4'b0010: // LUI
                (op_code == 7'b0010111) ? 4'b0011: // AUIPC
                (op_code == 7'b0000011) ? 4'b0100: // LOAD
                (op_code == 7'b0100011) ? 4'b0101: // STORE
                (op_code == 7'b1100011) ? 4'b0110: // BRANCH
                (op_code == 7'b1100111) ? 4'b0111: // JALR
                (op_code == 7'b1101111) ? 4'b1000: // JAL
                4'bxxxx;

assign o_alu_control_sel = 
    (i_clu_alu_op == 2'b00) ? 4'b0000 : // Forced Addition (S, U, J)
    (i_clu_alu_op == 2'b01) ? 4'b0001 : // Forced Subtraction (B) - BEQ , BNE
    (i_clu_alu_op == 2'b10) ? (
        (decode == 4'b0000) ? ( // R type
            (funct3 == 3'b000) ? ((funct7[5]) ? 4'b0001 : 4'b0000) : // sub/add
            (funct3 == 3'b001) ? 4'b0101 : // sll
            (funct3 == 3'b010) ? 4'b1000 : // slt //BLT
            (funct3 == 3'b011) ? 4'b1001 : // sltu
            (funct3 == 3'b100) ? 4'b0100 : // xor
            (funct3 == 3'b101) ? ((funct7[5]) ? 4'b0111 : 4'b0110) : // sra/srl
            (funct3 == 3'b110) ? 4'b0011 : // or
            (funct3 == 3'b111) ? 4'b0010 : // and
            4'b0000
        ) : 
        (decode == 4'b0001) ? ( // I type
            (funct3 == 3'b000) ? 4'b0000 : // addi
            (funct3 == 3'b001) ? 4'b0101 : // slli
            (funct3 == 3'b010) ? 4'b1000 : // slti
            (funct3 == 3'b011) ? 4'b1001 : // sltiu
            (funct3 == 3'b100) ? 4'b0100 : // xori
            (funct3 == 3'b101) ? ((funct7[5]) ? 4'b0111 : 4'b0110) : // srai/srli
            (funct3 == 3'b110) ? 4'b0011 : // ori
            (funct3 == 3'b111) ? 4'b0010 : // andi
            4'b0000
        ) :
        (decode == 4'b0100) ? 4'b0000 : // Load (add base + offset)
        4'b0000
    ) :
    (i_clu_alu_op == 2'b11) ? 4'bxxxx : 4'b0000;

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

// Immediate format decoding
wire [5:0] i_imm_format;
assign i_imm_format =   
    (i_imem_rdata[6:0] == 7'b0110011)? 6'b000001 : // R
    (i_imem_rdata[6:0] == 7'b0010011)? 6'b000010 : // I
    (i_imem_rdata[6:0] == 7'b0000011)? 6'b000010 : // I (Load)
    (i_imem_rdata[6:0] == 7'b0100011)? 6'b000100 : // S
    (i_imem_rdata[6:0] == 7'b1100011)? 6'b001000 : // B
    (i_imem_rdata[6:0] == 7'b0110111)? 6'b010000 : // U
    (i_imem_rdata[6:0] == 7'b1101111)? 6'b100000 : // J
    6'bXXXXXX;

//------------------------------------//
// Register File
//------------------------------------//
wire [31:0] t_rs2_rdata;
wire [31:0] i_dmem_alu_muxout_data;
wire [31:0] o_rs1_rdata;

wire t_clu_ALUSrc, t_clu_MemtoReg, i_clu_branch;
wire [1:0] i_clu_alu_op;

rf #(.BYPASS_EN(0)) reg_inst(
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_rs1_raddr(i_imem_rdata[19:15]),
    .i_rs2_raddr(i_imem_rdata[24:20]),
    .i_rd_waddr(i_imem_rdata[11:7]),
    .i_rd_wen(i_rd_wen),
    .i_rd_wdata(i_dmem_alu_muxout_data),
    .o_rs1_rdata(o_rs1_rdata),
    .o_rs2_rdata(t_rs2_rdata)
);

// Immediate Generator
wire [31:0] t_immediate_out_data;
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
    .i_alu_o_Zero(i_alu_o_Zero),
    .i_imm_o_immediate(t_immediate_out_data),
    .o_instr_mem_rd_addr(o_imem_raddr)
);

// ALU Control
wire [3:0] o_alu_control_sel;
wire o_unsigned;
alu_control alu_control_inst( 
    .i_clu_alu_op(i_clu_alu_op),
    .i_instr_mem_inst(i_imem_rdata),
    .o_unsigned(o_unsigned),
    .o_alu_control_sel(o_alu_control_sel)
);

// Data Muxes
assign i_dmem_alu_muxout_data = t_clu_MemtoReg ? i_dmem_rdata : o_dmem_addr;
assign rs2_rdata_imm_mux_data = t_clu_ALUSrc ? t_immediate_out_data : t_rs2_rdata;

// ALU
alu_wrapper alu_wrapper_inst(
    .i_alu_ctrl_opsel(o_alu_control_sel),
    .i_rf_op1(o_rs1_rdata),
    .i_rf_op2(rs2_rdata_imm_mux_data),
    .i_aluctrl_unsigned(o_unsigned),
    .o_alu_result(o_dmem_addr),
    .o_alu_Zero(i_alu_o_Zero)
);

// Control Unit

control_unit control_unit_inst(
    .clk(i_clk),
    .rst_n(i_rst),
    .i_clu_inst(i_imem_rdata),
    .o_clu_Branch(i_clu_branch),
    .o_clu_MemRead(o_dmem_ren),
    .o_clu_MemtoReg(t_clu_MemtoReg),
    .o_clu_ALUOp(i_clu_alu_op),
    .o_clu_MemWrite(o_dmem_wen),
    .o_clu_ALUSrc(t_clu_ALUSrc),
    .o_clu_RegWrite(i_rd_wen),
    .o_clu_dmem_mask(o_dmem_mask)
);

endmodule

`default_nettype wire
