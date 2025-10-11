module alu_tb();

 logic [ 2:0] i_opsel;
 logic        i_sub;
 logic        i_unsigned;
 logic        i_arith;
 logic [31:0] i_op1;
 logic [31:0] i_op2;
 logic [31:0] o_result;
 logic        o_eq;
 logic        o_slt;

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

initial begin

#10
i_unsigned = 1'b1;
i_arith = 1'b0;
i_opsel = 3'b000;
i_sub = 1'b0;
i_op1 = 32'h5;
i_op2 = 32'h6;
#10;
i_sub = 1'b1;
i_op1 = 32'h5;
i_op2 = 32'h6;
#10;
i_opsel = 3'b001;
i_op1 = 32'h5;
i_op2 = 32'h6;
#10;
i_opsel = 3'b010;
i_op1 = 32'h5;
i_op2 = 32'h6;
#10;
i_opsel = 3'b011;
i_op1 = 32'h5;
i_op2 = 32'h6;
#10;
i_opsel = 3'b100;
i_op1 = 32'h5;
i_op2 = 32'h6;
#10;
i_opsel = 3'b101;
i_op1 = 32'h5;
i_op2 = 32'h6;
#10;
i_opsel = 3'b110;
i_op1 = 32'h5;
i_op2 = 32'h6;
#10;
i_opsel = 3'b111;
i_op1 = 32'h5;
i_op2 = 32'h6;
#10;

#10;
i_sub = 1'b1;
i_op1 = 32'h6;
i_op2 = 32'h6;
#10;
i_opsel = 3'b001;
i_op1 = 32'h6;
i_op2 = 32'h6;
#10;
i_opsel = 3'b010;
i_op1 = 32'h6;
i_op2 = 32'h6;
#10;
i_opsel = 3'b011;
i_op1 = 32'h6;
i_op2 = 32'h6;
#10;
i_opsel = 3'b100;
i_op1 = 32'h6;
i_op2 = 32'h6;
#10;
i_opsel = 3'b101;
i_op1 = 32'h6;
i_op2 = 32'h6;
#10;
i_opsel = 3'b110;
i_op1 = 32'h6;
i_op2 = 32'h6;
#10;
i_opsel = 3'b111;
i_op1 = 32'h6;
i_op2 = 32'h6;
#10;


#10;
i_sub = 1'b1;
i_op1 = 32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b001;
i_op1 = 32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b010;
i_op1 = 32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b011;
i_op1 = 32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b100;
i_op1 = 32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b101;
i_op1 = 32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b110;
i_op1 = 32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b111;
i_op1 = 32'h5;
i_op2 = 32'h3;
#10;



#10;
i_sub = 1'b1;
i_op1 = -32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b001;
i_op1 = -32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b010;
i_op1 = -32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b011;
i_op1 = -32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b100;
i_op1 = -32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b101;
i_op1 = -32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b110;
i_op1 = -32'h5;
i_op2 = 32'h3;
#10;
i_opsel = 3'b111;
i_op1 = -32'h5;
i_op2 = 32'h3;
#10;

end

endmodule