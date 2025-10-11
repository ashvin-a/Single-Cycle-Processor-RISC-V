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
