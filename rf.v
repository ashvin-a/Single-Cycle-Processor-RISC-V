`default_nettype none

// The register file is effectively a single cycle memory with 32-bit words
// and depth 32. It has two asynchronous read ports, allowing two independent
// registers to be read at the same time combinationally, and one synchronous
// write port, allowing a register to be written to on the next clock edge.
//
// The register `x0` is hardwired to zero.
// NOTE: This can be implemented either by silently discarding writesto
// address 5'd0, or by muxing the output to zero when reading from that
// address.
module rf #(
    // When this parameter is set to 1, "RF bypass" mode is enabled. This
    // allows data at the write port to be observed at the read ports
    // immediately without having to wait for the next clock edge. This is
    // a common forwarding optimization in a pipelined core (phase 5), but will
    // cause a single-cycle processor to behave incorrectly. You are required
    // to implement and test both modes. In phase 4, you will disable this
    // parameter, before enabling it in phase 6.
    parameter BYPASS_EN = 0
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // Both read register ports are asynchronous (zero-cycle). That is, read
    // data is visible combinationally without having to wait for a clock.
    //
    // The read ports are *independent* and can read two different registers
    // (but of course, also the same register if needed).
    //
    // Register `x0` is hardwired to zero, so reading from address 5'd0
    // should always return 32'd0 on either port regardless of a    ny writes.
    //
    // Register read port 1, with input address [0, 31] and output data.
    input  wire [ 4:0] i_rs1_raddr,
    output wire [31:0] o_rs1_rdata,
    // Register read port 2, with input address [0, 31] and output data.
    input  wire [ 4:0] i_rs2_raddr,
    output wire [31:0] o_rs2_rdata,
    // The register write port is synchronous. When write is enabled, the
    // data at the write port will be written to the specified register
    // at the next clock edge. When the writen enable is low, the register
    // file should remain unchanged at the clock edge.
    //
    // Write register enable, address [0, 31] and input data.
    input  wire        i_rd_wen,
    input  wire [ 4:0] i_rd_waddr,
    input  wire [31:0] i_rd_wdata
);
    // 32 individual 32-bit registers
    reg [31:0] reg_x0,  reg_x1,  reg_x2,  reg_x3,  reg_x4,  reg_x5,  reg_x6,  reg_x7;
    reg [31:0] reg_x8,  reg_x9,  reg_x10, reg_x11, reg_x12, reg_x13, reg_x14, reg_x15;
    reg [31:0] reg_x16, reg_x17, reg_x18, reg_x19, reg_x20, reg_x21, reg_x22, reg_x23;
    reg [31:0] reg_x24, reg_x25, reg_x26, reg_x27, reg_x28, reg_x29, reg_x30, reg_x31;

    always @(posedge i_clk) begin
        if (i_rst) begin
            reg_x0  <= 32'd0;  reg_x1  <= 32'd0;  reg_x2  <= 32'd0;  reg_x3  <= 32'd0;
            reg_x4  <= 32'd0;  reg_x5  <= 32'd0;  reg_x6  <= 32'd0;  reg_x7  <= 32'd0;
            reg_x8  <= 32'd0;  reg_x9  <= 32'd0;  reg_x10 <= 32'd0;  reg_x11 <= 32'd0;
            reg_x12 <= 32'd0;  reg_x13 <= 32'd0;  reg_x14 <= 32'd0;  reg_x15 <= 32'd0;
            reg_x16 <= 32'd0;  reg_x17 <= 32'd0;  reg_x18 <= 32'd0;  reg_x19 <= 32'd0;
            reg_x20 <= 32'd0;  reg_x21 <= 32'd0;  reg_x22 <= 32'd0;  reg_x23 <= 32'd0;
            reg_x24 <= 32'd0;  reg_x25 <= 32'd0;  reg_x26 <= 32'd0;  reg_x27 <= 32'd0;
            reg_x28 <= 32'd0;  reg_x29 <= 32'd0;  reg_x30 <= 32'd0;  reg_x31 <= 32'd0;
        end 
        else if (i_rd_wen && (i_rd_waddr != 5'd0)) begin
            case (i_rd_waddr)
                5'd1:  reg_x1  <= i_rd_wdata;
                5'd2:  reg_x2  <= i_rd_wdata;
                5'd3:  reg_x3  <= i_rd_wdata;
                5'd4:  reg_x4  <= i_rd_wdata;
                5'd5:  reg_x5  <= i_rd_wdata;
                5'd6:  reg_x6  <= i_rd_wdata;
                5'd7:  reg_x7  <= i_rd_wdata;
                5'd8:  reg_x8  <= i_rd_wdata;
                5'd9:  reg_x9  <= i_rd_wdata;
                5'd10: reg_x10 <= i_rd_wdata;
                5'd11: reg_x11 <= i_rd_wdata;
                5'd12: reg_x12 <= i_rd_wdata;
                5'd13: reg_x13 <= i_rd_wdata;
                5'd14: reg_x14 <= i_rd_wdata;
                5'd15: reg_x15 <= i_rd_wdata;
                5'd16: reg_x16 <= i_rd_wdata;
                5'd17: reg_x17 <= i_rd_wdata;
                5'd18: reg_x18 <= i_rd_wdata;
                5'd19: reg_x19 <= i_rd_wdata;
                5'd20: reg_x20 <= i_rd_wdata;
                5'd21: reg_x21 <= i_rd_wdata;
                5'd22: reg_x22 <= i_rd_wdata;
                5'd23: reg_x23 <= i_rd_wdata;
                5'd24: reg_x24 <= i_rd_wdata;
                5'd25: reg_x25 <= i_rd_wdata;
                5'd26: reg_x26 <= i_rd_wdata;
                5'd27: reg_x27 <= i_rd_wdata;
                5'd28: reg_x28 <= i_rd_wdata;
                5'd29: reg_x29 <= i_rd_wdata;
                5'd30: reg_x30 <= i_rd_wdata;
                5'd31: reg_x31 <= i_rd_wdata;
            endcase
        end
    end

    wire [31:0] rs1_val;
    wire [31:0] rs2_val;

    assign rs1_val = (i_rs1_raddr == 5'd0) ? 32'd0 :
                     (i_rs1_raddr == 5'd1)  ? reg_x1  :
                     (i_rs1_raddr == 5'd2)  ? reg_x2  :
                     (i_rs1_raddr == 5'd3)  ? reg_x3  :
                     (i_rs1_raddr == 5'd4)  ? reg_x4  :
                     (i_rs1_raddr == 5'd5)  ? reg_x5  :
                     (i_rs1_raddr == 5'd6)  ? reg_x6  :
                     (i_rs1_raddr == 5'd7)  ? reg_x7  :
                     (i_rs1_raddr == 5'd8)  ? reg_x8  :
                     (i_rs1_raddr == 5'd9)  ? reg_x9  :
                     (i_rs1_raddr == 5'd10) ? reg_x10 :
                     (i_rs1_raddr == 5'd11) ? reg_x11 :
                     (i_rs1_raddr == 5'd12) ? reg_x12 :
                     (i_rs1_raddr == 5'd13) ? reg_x13 :
                     (i_rs1_raddr == 5'd14) ? reg_x14 :
                     (i_rs1_raddr == 5'd15) ? reg_x15 :
                     (i_rs1_raddr == 5'd16) ? reg_x16 :
                     (i_rs1_raddr == 5'd17) ? reg_x17 :
                     (i_rs1_raddr == 5'd18) ? reg_x18 :
                     (i_rs1_raddr == 5'd19) ? reg_x19 :
                     (i_rs1_raddr == 5'd20) ? reg_x20 :
                     (i_rs1_raddr == 5'd21) ? reg_x21 :
                     (i_rs1_raddr == 5'd22) ? reg_x22 :
                     (i_rs1_raddr == 5'd23) ? reg_x23 :
                     (i_rs1_raddr == 5'd24) ? reg_x24 :
                     (i_rs1_raddr == 5'd25) ? reg_x25 :
                     (i_rs1_raddr == 5'd26) ? reg_x26 :
                     (i_rs1_raddr == 5'd27) ? reg_x27 :
                     (i_rs1_raddr == 5'd28) ? reg_x28 :
                     (i_rs1_raddr == 5'd29) ? reg_x29 :
                     (i_rs1_raddr == 5'd30) ? reg_x30 : reg_x31;

    assign rs2_val = (i_rs2_raddr == 5'd0) ? 32'd0 :
                     (i_rs2_raddr == 5'd1)  ? reg_x1  :
                     (i_rs2_raddr == 5'd2)  ? reg_x2  :
                     (i_rs2_raddr == 5'd3)  ? reg_x3  :
                     (i_rs2_raddr == 5'd4)  ? reg_x4  :
                     (i_rs2_raddr == 5'd5)  ? reg_x5  :
                     (i_rs2_raddr == 5'd6)  ? reg_x6  :
                     (i_rs2_raddr == 5'd7)  ? reg_x7  :
                     (i_rs2_raddr == 5'd8)  ? reg_x8  :
                     (i_rs2_raddr == 5'd9)  ? reg_x9  :
                     (i_rs2_raddr == 5'd10) ? reg_x10 :
                     (i_rs2_raddr == 5'd11) ? reg_x11 :
                     (i_rs2_raddr == 5'd12) ? reg_x12 :
                     (i_rs2_raddr == 5'd13) ? reg_x13 :
                     (i_rs2_raddr == 5'd14) ? reg_x14 :
                     (i_rs2_raddr == 5'd15) ? reg_x15 :
                     (i_rs2_raddr == 5'd16) ? reg_x16 :
                     (i_rs2_raddr == 5'd17) ? reg_x17 :
                     (i_rs2_raddr == 5'd18) ? reg_x18 :
                     (i_rs2_raddr == 5'd19) ? reg_x19 :
                     (i_rs2_raddr == 5'd20) ? reg_x20 :
                     (i_rs2_raddr == 5'd21) ? reg_x21 :
                     (i_rs2_raddr == 5'd22) ? reg_x22 :
                     (i_rs2_raddr == 5'd23) ? reg_x23 :
                     (i_rs2_raddr == 5'd24) ? reg_x24 :
                     (i_rs2_raddr == 5'd25) ? reg_x25 :
                     (i_rs2_raddr == 5'd26) ? reg_x26 :
                     (i_rs2_raddr == 5'd27) ? reg_x27 :
                     (i_rs2_raddr == 5'd28) ? reg_x28 :
                     (i_rs2_raddr == 5'd29) ? reg_x29 :
                     (i_rs2_raddr == 5'd30) ? reg_x30 : reg_x31;

    assign o_rs1_rdata = (BYPASS_EN && i_rd_wen && (i_rd_waddr == i_rs1_raddr) && (i_rs1_raddr != 5'd0))
                         ? i_rd_wdata : rs1_val;

    assign o_rs2_rdata = (BYPASS_EN && i_rd_wen && (i_rd_waddr == i_rs2_raddr) && (i_rs2_raddr != 5'd0))
                         ? i_rd_wdata : rs2_val;

endmodule

`default_nettype wire



        
