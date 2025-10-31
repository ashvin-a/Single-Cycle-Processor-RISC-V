///////////////////////////////////////////
/////////////////Queries on ALU///////////////////    
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
//For LUI we can have a bypass scenario where whatever data that comes in the second operand input of ALU after the MUX
// Will be bypassed drectly to the ALU result - But do we have an opsel 3bit field free for that?
//What Does ALU control do? If its output is 4 bit then why dont we have a 4bit input to the ALU?
//[Q] DO we have Zero and Overflow as output in ALU?
// [Q] Both in case of o_slt and o_eq is it expected to be connected to Zero output?


////Project Phase 5

1. What should be the values of the register fields which are not being assigned in the next clock?
2. For nop implementation - Do we have to retain the reg value as it is or do we have to propogate something? (To retain the reg value we can just freeze the reg)
3. New OPCODE check for nop has to be implemented in MCU.
4. How about I still keep an enable for a single cycle processor? A register Bypass signal?
