`default_nettype none

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
    // branch should be taken.
    output wire        o_eq,
    // Set less than result. This is used downstream to determine if a
    // branch should be taken.
    output wire        o_slt
);

///////////////////////////////////////////
/////////////////Queries///////////////////    
///////////////////////////////////////////
//For shift left and shift right (arithmetic as well as logical) for a 2 input ALU , which input will be the immediate value??
//Logical shift has no sign awareness where as Arithmetic shift has - more evident in case of right shift where we will have to
// extend by sign bit if it is a signed number.
//What if op1<op2? in case of subtract?
//How do I know whether an operand is signed or unsigned?
//If you use a signed instruction (slt, blt, div), 
//the hardware interprets operands as two’s complement signed integers.
//If you use an unsigned instruction (sltu, bltu, divu), the hardware interprets them as unsigned integers.
//what is the default value for slt x or 0? - I am setting as X as 0 gives a wrong information.
//What is the default operation expected?
//What is the o_result expectation when we check for slt and sltu?

   wire [4:0]temp = i_op2[4:0];
   wire [31:0] sll = i_op1 << temp;
   wire [31:0] srl = i_op1 >> temp;
   wire [31:0] sra = $signed(i_op1) >>> temp;
   wire [31:0] slt_signed = ($signed(i_op1) < $signed(i_op2)) ? 32'b1 : 32'b0;
   wire [31:0] slt_unsigned = (i_op1 < i_op2) ? 32'b1 : 32'b0;


assign o_eq  = (i_op1 == i_op2);
assign o_slt =  i_unsigned ? (i_op1 < i_op2) : ($signed(i_op1) < $signed(i_op2)); 
assign o_result =       (i_opsel == 3'b000)? ((i_sub) ? (i_op1 - i_op2) : (i_op1 + i_op2)) :
                        (i_opsel == 3'b001)? (i_op1 << temp) : 
                        ((i_opsel == 3'b010) || (i_opsel == 3'b011)) ? (i_unsigned ? slt_unsigned : slt_signed) : 
                        (i_opsel == 3'b100)? (i_op1 ^ i_op2) :  
                        (i_opsel == 3'b101)? ((i_arith)? sra : srl) : 
                        (i_opsel == 3'b110)? (i_op1 | i_op2) : 
                        (i_opsel == 3'b111)? (i_op1 & i_op2) : 
                        32'h0;

endmodule

`default_nettype wire


module alu_control( input  wire [1:0]  i_clu_alu_op,
                    input  wire [31:0] i_instr_mem_inst,
                    output wire [3:0]  o_alu_control);

assign op_code = i_instr_mem_inst[6:0]
assign decode = (op_code == 7'b0110011) ? 4'b0000: // R type
              (op_code == 7'b0010011) ? 4'b0001: // I type
              (op_code == 7'b0110111) ? 4'b0010: // LUI
              (op_code == 7'b0010111) ? 4'b0011: // AUIPC Where the hell do we even use this?
              (op_code == 7'b0000011) ? 4'b0100: // LOAD
              (op_code == 7'b0100011) ? 4'b0101: // STORE
              (op_code == 7'b1100011) ? 4'b0110: // BRANCH
              (op_code == 7'b1100111) ? 4'b0111: // JALR
              (op_code == 7'b1101111) ? 4'b1000; // JAL


              

assign o_alu_control =  (i_clu_alu_op == 2'b0 ) ? (4'b0000) : //Forced Addition
                        (i_clu_alu_op == 2'b01 ) ? (4'b0001): //Forced Subtraction
                        (i_clu_alu_op == 2'b10 ) ? // Might be an R type or I type
                        // Check for R type
                        // TODO: Complete this!!
                        (deco == 4'b0000) ? ()
                        // Check for I type
                        (deco == 4'b0000) 
                        (i_clu_alu_op == 2'b11 )

endmodule