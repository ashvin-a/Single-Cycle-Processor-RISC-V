module hart_tb ();
    // Synchronous active-high reset.
    reg         clk, rst;
    // Instruction memory interface.
    reg  [31:0] imem_rdata, dmem_rdata;
    wire [31:0] imem_raddr, dmem_addr;
    // Data memory interface.
    wire        dmem_ren, dmem_wen;
    wire [31:0] dmem_wdata;
    wire [ 3:0] dmem_mask;

    // Instruction retire interface.
    wire        valid, trap, halt;
    wire [31:0] inst;
    wire [ 4:0] rs1_raddr, rs2_raddr;
    wire [31:0] rs1_rdata, rs2_rdata;
    wire [ 4:0] rd_waddr;
    wire [31:0] rd_wdata;
    wire [31:0] pc, next_pc;

    hart #(
        .RESET_ADDR (32'h0)
    ) dut (
        .i_clk        (clk),
        .i_rst        (rst),
        .o_imem_raddr (imem_raddr),
        .i_imem_rdata (imem_rdata),
        //.i_imem_rdata (32'h00618113),
        .o_dmem_addr  (dmem_addr),
        .o_dmem_ren   (dmem_ren),
        .o_dmem_wen   (dmem_wen),
        .o_dmem_wdata (dmem_wdata),
        .o_dmem_mask  (dmem_mask),
        .i_dmem_rdata (dmem_rdata),
        .o_retire_valid     (valid),
        .o_retire_inst      (inst),
        .o_retire_trap      (trap),
        .o_retire_halt      (halt),
        .o_retire_rs1_raddr (rs1_raddr),
        .o_retire_rs1_rdata (rs1_rdata),
        .o_retire_rs2_raddr (rs2_raddr),
        .o_retire_rs2_rdata (rs2_rdata),
        .o_retire_rd_waddr  (rd_waddr),
        .o_retire_rd_wdata  (rd_wdata),
        .o_retire_pc        (pc),
        .o_retire_next_pc   (next_pc)
    );

    // The tesbench uses separate instruction and data memory banks.
    reg [7:0] imem [0:1023];
    reg [7:0] dmem [0:1023];

    // Instruction memory read.
    always @(*) begin
        imem_rdata = {imem[imem_raddr + 3], imem[imem_raddr + 2], imem[imem_raddr + 1], imem[imem_raddr + 0]};
    end

    // Data memory read. Masks are ignored since it is always safe
    // to access the full bytes in this memory.
    always @(*) begin
        if (dmem_ren)
            dmem_rdata = {dmem[dmem_addr + 3], dmem[dmem_addr + 2], dmem[dmem_addr + 1], dmem[dmem_addr + 0]};
        else
            dmem_rdata = 32'h0;
    end

    // Synchronous data memory write. Masks must be respected.
    // The byte order is little-endian.
    always @(posedge clk) begin
        if (dmem_wen & dmem_mask[0])
            dmem[dmem_addr + 0] <= dmem_wdata[ 7: 0];
        if (dmem_wen & dmem_mask[1])
            dmem[dmem_addr + 1] <= dmem_wdata[15: 8];
        if (dmem_wen & dmem_mask[2])
            dmem[dmem_addr + 2] <= dmem_wdata[23:16];
        if (dmem_wen & dmem_mask[3])
            dmem[dmem_addr + 3] <= dmem_wdata[31:24];
    end

    integer cycles;
    initial begin
        clk = 0;
        // Load the test program into memory at address 0.
        $display("Loading program.");
       // $readmemh("program.mem", imem);
        imem[0] = 8'hb7;
        imem[1] = 8'h30;
        imem[2] = 8'h00;
        imem[3] = 8'h00;

        imem[4] = 8'h37;
        imem[5] = 8'h71;
        imem[6] = 8'h00;
        imem[7] = 8'h00;

        imem[8] = 8'h33;
        imem[9] = 8'h8f;
        imem[10] = 8'h20;
        imem[11] = 8'h00;

        imem[12] = 8'hb7;
        imem[13] = 8'hae;
        imem[14] = 8'h00;
        imem[15] = 8'h00;

      //  imem[16] = 8'hb7;
      //  imem[17] = 8'h41;
      //  imem[18] = 8'h00;
      //  imem[19] = 8'h00;

        imem[16] = 8'h63;
        imem[17] = 8'h12;
        imem[18] = 8'hdf;
        imem[19] = 8'h01;

        imem[20] = 8'hb7;
        imem[21] = 8'h42;
        imem[22] = 8'h00;
        imem[23] = 8'h00;

        imem[24] = 8'h37;
        imem[25] = 8'h53;
        imem[26] = 8'h00;
        imem[27] = 8'h00;

        imem[28] = 8'h33;
        imem[29] = 8'h8e;
        imem[30] = 8'h62;
        imem[31] = 8'h00;

        imem[32] = 8'hb7;
        imem[33] = 8'h9d;
        imem[34] = 8'h00;
        imem[35] = 8'h00;

        imem[36] = 8'h63;
        imem[37] = 8'h02;
        imem[38] = 8'hbe;
        imem[39] = 8'h01;

// 0x000042b7
// 0x00004337
// 0x00628e33
// 0x00009db7
// 0x01be0263

        // Reset the dut.
        $display("Resetting hart.");
        @(negedge clk); rst = 1;
        @(negedge clk); rst = 0;

        $display("Cycle  PC        Inst     rs1            rs2            [rd, load, store]");
        cycles = 0;
        //while (!halt ) begin
        while (cycles != 100) begin
            @(posedge clk);
            cycles = cycles + 1;

            if (valid) begin
                // Base information for all instructions.
                $write("%05d [%08h] %08h r[%d]=%08h r[%d]=%08h at time = %0t", cycles, pc, inst, rs1_raddr, rs1_rdata, rs2_raddr, rs2_rdata,$time);
                // Only display write information for instructions that write.
                if (rd_waddr != 5'd0)
                    $write(" w[%d]=%08h", rd_waddr, rd_wdata);
                // Only display memory information for load/store instructions.
                if (dmem_ren)
                    $write(" l[%08h,%04b]=%08h", dmem_addr, dmem_mask, dmem_rdata);
                if (dmem_wen)
                    $write(" s[%08h,%04b]=%08h", dmem_addr, dmem_mask, dmem_wdata);
                // Display trap information if a trap occurred.
                if (trap)
                    $write(" TRAP");
                $display();
            end
        end

        $display("Program halted after %d cycles.", cycles);
        //$display("r[a0]=%08h (%d)", dut.rf.mem[10], dut.rf.mem[10]);
        $stop;
    end
    always
        #5 clk = ~clk;
endmodule
