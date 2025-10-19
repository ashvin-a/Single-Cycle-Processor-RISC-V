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


    wire [7:0] imem_raddr1;
    wire [7:0] imem_raddr2;
    wire [7:0] imem_raddr3;
    wire [7:0] imem_raddr0;

    assign imem_raddr3 = imem[imem_raddr + 3];
    assign imem_raddr2 = imem[imem_raddr + 2];
    assign imem_raddr1 = imem[imem_raddr + 1];
    assign imem_raddr0 = imem[imem_raddr + 0];
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

// Test 2: add x30, x1, x2 (0 + 0 = 0)
imem[0]   = 8'h93; imem[1]   = 8'h00; imem[2]   = 8'h00; imem[3]   = 8'h00; // addi x1, x0, 0
imem[4]   = 8'h13; imem[5]   = 8'h01; imem[6]   = 8'h00; imem[7]   = 8'h00; // addi x2, x0, 0
imem[8]   = 8'h33; imem[9]   = 8'h8f; imem[10]  = 8'h20; imem[11]  = 8'h00; // add x30, x1, x2
imem[12]  = 8'h93; imem[13]  = 8'h0e; imem[14]  = 8'h00; imem[15]  = 8'h00; // addi x29, x0, 0
imem[16]  = 8'h93; imem[17]  = 8'h01; imem[18]  = 8'h20; imem[19]  = 8'h00; // addi x3, x0, 2
imem[20]  = 8'h63; imem[21]  = 8'h0c; imem[22]  = 8'hdf; imem[23]  = 8'h1f; // bne x30, x29, 500 - Updated bne to beq and also to 504 + 20 position

// Test 3: add x30, x1, x2 (1 + 1 = 2)
imem[24]  = 8'h93; imem[25]  = 8'h00; imem[26]  = 8'h10; imem[27]  = 8'h00; // addi x1, x0, 1
imem[28]  = 8'h13; imem[29]  = 8'h01; imem[30]  = 8'h10; imem[31]  = 8'h00; // addi x2, x0, 1
imem[32]  = 8'h33; imem[33]  = 8'h8f; imem[34]  = 8'h20; imem[35]  = 8'h00; // add x30, x1, x2
imem[36]  = 8'h93; imem[37]  = 8'h0e; imem[38]  = 8'h20; imem[39]  = 8'h00; // addi x29, x0, 2
imem[40]  = 8'h93; imem[41]  = 8'h01; imem[42]  = 8'h30; imem[43]  = 8'h00; // addi x3, x0, 3
imem[44]  = 8'h63; imem[45]  = 8'h1e; imem[46]  = 8'hdf; imem[47]  = 8'h1d; // bne x30, x29, 476

// Test 4: add x30, x1, x2 (3 + 7 = 10)
imem[48]  = 8'h93; imem[49]  = 8'h00; imem[50]  = 8'h30; imem[51]  = 8'h00; // addi x1, x0, 3
imem[52]  = 8'h13; imem[53]  = 8'h01; imem[54]  = 8'h70; imem[55]  = 8'h00; // addi x2, x0, 7
imem[56]  = 8'h33; imem[57]  = 8'h8f; imem[58]  = 8'h20; imem[59]  = 8'h00; // add x30, x1, x2
imem[60]  = 8'h93; imem[61]  = 8'h0e; imem[62]  = 8'ha0; imem[63]  = 8'h00; // addi x29, x0, 10
imem[64]  = 8'h93; imem[65]  = 8'h01; imem[66]  = 8'h40; imem[67]  = 8'h00; // addi x3, x0, 4
imem[68]  = 8'h63; imem[69]  = 8'h12; imem[70]  = 8'hdf; imem[71]  = 8'h1d; // bne x30, x29, 452

// Test 5: add x30, x1, x2 (0 + 0xffff8000 = 0xffff8000)
imem[72]  = 8'h93; imem[73]  = 8'h00; imem[74]  = 8'h00; imem[75]  = 8'h00; // addi x1, x0, 0
imem[76]  = 8'h37; imem[77]  = 8'h81; imem[78]  = 8'hff; imem[79]  = 8'hff; // lui x2, 0xffff8
imem[80]  = 8'h13; imem[81]  = 8'h01; imem[82]  = 8'h01; imem[83]  = 8'h00; // addi x2, x2, 0
imem[84]  = 8'h33; imem[85]  = 8'h8f; imem[86]  = 8'h20; imem[87]  = 8'h00; // add x30, x1, x2
imem[88]  = 8'hb7; imem[89]  = 8'h8e; imem[90]  = 8'hff; imem[91]  = 8'hff; // lui x29, 0xffff8
imem[92]  = 8'h93; imem[93]  = 8'h8e; imem[94]  = 8'h0e; imem[95]  = 8'h00; // addi x29, x29, 0
imem[96]  = 8'h93; imem[97]  = 8'h01; imem[98]  = 8'h50; imem[99]  = 8'h00; // addi x3, x0, 5
imem[100] = 8'h63; imem[101] = 8'h12; imem[102] = 8'hdf; imem[103] = 8'h1b; // bne x30, x29, 420

// Test 6: add x30, x1, x2 (0x80000000 + 0 = 0x80000000)
imem[104] = 8'hb7; imem[105] = 8'h00; imem[106] = 8'h00; imem[107] = 8'h80; // lui x1, 0x80000
imem[108] = 8'h93; imem[109] = 8'h80; imem[110] = 8'h00; imem[111] = 8'h00; // addi x1, x1, 0
imem[112] = 8'h13; imem[113] = 8'h01; imem[114] = 8'h00; imem[115] = 8'h00; // addi x2, x0, 0
imem[116] = 8'h33; imem[117] = 8'h8f; imem[118] = 8'h20; imem[119] = 8'h00; // add x30, x1, x2
imem[120] = 8'hb7; imem[121] = 8'h0e; imem[122] = 8'h00; imem[123] = 8'h80; // lui x29, 0x80000
imem[124] = 8'h93; imem[125] = 8'h8e; imem[126] = 8'h0e; imem[127] = 8'h00; // addi x29, x29, 0
imem[128] = 8'h93; imem[129] = 8'h01; imem[130] = 8'h60; imem[131] = 8'h00; // addi x3, x0, 6
imem[132] = 8'h63; imem[133] = 8'h12; imem[134] = 8'hdf; imem[135] = 8'h19; // bne x30, x29, 388

// Test 7: add x30, x1, x2 (0x80000000 + 0xffff8000 = 0x7fff8000)
imem[136] = 8'hb7; imem[137] = 8'h00; imem[138] = 8'h00; imem[139] = 8'h80; // lui x1, 0x80000
imem[140] = 8'h93; imem[141] = 8'h80; imem[142] = 8'h00; imem[143] = 8'h00; // addi x1, x1, 0
imem[144] = 8'h37; imem[145] = 8'h81; imem[146] = 8'hff; imem[147] = 8'hff; // lui x2, 0xffff8
imem[148] = 8'h13; imem[149] = 8'h01; imem[150] = 8'h01; imem[151] = 8'h00; // addi x2, x2, 0
imem[152] = 8'h33; imem[153] = 8'h8f; imem[154] = 8'h20; imem[155] = 8'h00; // add x30, x1, x2
imem[156] = 8'hb7; imem[157] = 8'h8e; imem[158] = 8'hff; imem[159] = 8'h7f; // lui x29, 0x7fff8
imem[160] = 8'h93; imem[161] = 8'h8e; imem[162] = 8'h0e; imem[163] = 8'h00; // addi x29, x29, 0
imem[164] = 8'h93; imem[165] = 8'h01; imem[166] = 8'h70; imem[167] = 8'h00; // addi x3, x0, 7
imem[168] = 8'h63; imem[169] = 8'h10; imem[170] = 8'hdf; imem[171] = 8'h17; // bne x30, x29, 352

// Test 8: add x30, x1, x2 (0 + 0x7fff = 0x7fff)
imem[172] = 8'h93; imem[173] = 8'h00; imem[174] = 8'h00; imem[175] = 8'h00; // addi x1, x0, 0
imem[176] = 8'h37; imem[177] = 8'h81; imem[178] = 8'h00; imem[179] = 8'h00; // lui x2, 0x8
imem[180] = 8'h13; imem[181] = 8'h01; imem[182] = 8'hf1; imem[183] = 8'hff; // addi x2, x2, -1
imem[184] = 8'h33; imem[185] = 8'h8f; imem[186] = 8'h20; imem[187] = 8'h00; // add x30, x1, x2
imem[188] = 8'hb7; imem[189] = 8'h8e; imem[190] = 8'h00; imem[191] = 8'h00; // lui x29, 0x8
imem[192] = 8'h93; imem[193] = 8'h8e; imem[194] = 8'hfe; imem[195] = 8'hff; // addi x29, x29, -1
imem[196] = 8'h93; imem[197] = 8'h01; imem[198] = 8'h80; imem[199] = 8'h00; // addi x3, x0, 8
imem[200] = 8'h63; imem[201] = 8'h10; imem[202] = 8'hdf; imem[203] = 8'h15; // bne x30, x29, 320

// Test 9: add x30, x1, x2 (0x7fffffff + 0 = 0x7fffffff)
imem[204] = 8'hb7; imem[205] = 8'h00; imem[206] = 8'h00; imem[207] = 8'h80; // lui x1, 0x80000
imem[208] = 8'h93; imem[209] = 8'h80; imem[210] = 8'hf0; imem[211] = 8'hff; // addi x1, x1, -1
imem[212] = 8'h13; imem[213] = 8'h01; imem[214] = 8'h00; imem[215] = 8'h00; // addi x2, x0, 0
imem[216] = 8'h33; imem[217] = 8'h8f; imem[218] = 8'h20; imem[219] = 8'h00; // add x30, x1, x2
imem[220] = 8'hb7; imem[221] = 8'h0e; imem[222] = 8'h00; imem[223] = 8'h80; // lui x29, 0x80000
imem[224] = 8'h93; imem[225] = 8'h8e; imem[226] = 8'hfe; imem[227] = 8'hff; // addi x29, x29, -1
imem[228] = 8'h93; imem[229] = 8'h01; imem[230] = 8'h90; imem[231] = 8'h00; // addi x3, x0, 9
imem[232] = 8'h63; imem[233] = 8'h10; imem[234] = 8'hdf; imem[235] = 8'h13; // bne x30, x29, 288

// Test 10: add x30, x1, x2 (0x7fffffff + 0x7fff = overflow)
imem[236] = 8'hb7; imem[237] = 8'h00; imem[238] = 8'h00; imem[239] = 8'h80; // lui x1, 0x80000
imem[240] = 8'h93; imem[241] = 8'h80; imem[242] = 8'hf0; imem[243] = 8'hff; // addi x1, x1, -1
imem[244] = 8'h37; imem[245] = 8'h81; imem[246] = 8'h00; imem[247] = 8'h00; // lui x2, 0x8
imem[248] = 8'h13; imem[249] = 8'h01; imem[250] = 8'hf1; imem[251] = 8'hff; // addi x2, x2, -1
imem[252] = 8'h33; imem[253] = 8'h8f; imem[254] = 8'h20; imem[255] = 8'h00; // add x30, x1, x2
imem[256] = 8'hb7; imem[257] = 8'h8e; imem[258] = 8'h00; imem[259] = 8'h80; // lui x29, 0x80008
imem[260] = 8'h93; imem[261] = 8'h8e; imem[262] = 8'hee; imem[263] = 8'hff; // addi x29, x29, -2
imem[264] = 8'h93; imem[265] = 8'h01; imem[266] = 8'ha0; imem[267] = 8'h00; // addi x3, x0, 10
imem[268] = 8'h63; imem[269] = 8'h1e; imem[270] = 8'hdf; imem[271] = 8'h0f; // bne x30, x29, 252

// Test 11: add x30, x1, x2 (0x80000000 + 0x7fff = 0x80007fff)
imem[272] = 8'hb7; imem[273] = 8'h00; imem[274] = 8'h00; imem[275] = 8'h80; // lui x1, 0x80000
imem[276] = 8'h93; imem[277] = 8'h80; imem[278] = 8'h00; imem[279] = 8'h00; // addi x1, x1, 0
imem[280] = 8'h37; imem[281] = 8'h81; imem[282] = 8'h00; imem[283] = 8'h00; // lui x2, 0x8
imem[284] = 8'h13; imem[285] = 8'h01; imem[286] = 8'hf1; imem[287] = 8'hff; // addi x2, x2, -1
imem[288] = 8'h33; imem[289] = 8'h8f; imem[290] = 8'h20; imem[291] = 8'h00; // add x30, x1, x2
imem[292] = 8'hb7; imem[293] = 8'h8e; imem[294] = 8'h00; imem[295] = 8'h80; // lui x29, 0x80008
imem[296] = 8'h93; imem[297] = 8'h8e; imem[298] = 8'hfe; imem[299] = 8'hff; // addi x29, x29, -1
imem[300] = 8'h93; imem[301] = 8'h01; imem[302] = 8'hb0; imem[303] = 8'h00; // addi x3, x0, 11
imem[304] = 8'h63; imem[305] = 8'h1c; imem[306] = 8'hdf; imem[307] = 8'h0d; // bne x30, x29, 216

// Test 12: add x30, x1, x2 (0x7fffffff + 0xffff8000 = 0x7fff7fff)
imem[308] = 8'hb7; imem[309] = 8'h00; imem[310] = 8'h00; imem[311] = 8'h80; // lui x1, 0x80000
imem[312] = 8'h93; imem[313] = 8'h80; imem[314] = 8'hf0; imem[315] = 8'hff; // addi x1, x1, -1
imem[316] = 8'h37; imem[317] = 8'h81; imem[318] = 8'hff; imem[319] = 8'hff; // lui x2, 0xffff8
imem[320] = 8'h13; imem[321] = 8'h01; imem[322] = 8'h01; imem[323] = 8'h00; // addi x2, x2, 0
imem[324] = 8'h33; imem[325] = 8'h8f; imem[326] = 8'h20; imem[327] = 8'h00; // add x30, x1, x2
imem[328] = 8'hb7; imem[329] = 8'h8e; imem[330] = 8'hff; imem[331] = 8'h7f; // lui x29, 0x7fff8
imem[332] = 8'h93; imem[333] = 8'h8e; imem[334] = 8'hfe; imem[335] = 8'hff; // addi x29, x29, -1
imem[336] = 8'h93; imem[337] = 8'h01; imem[338] = 8'hc0; imem[339] = 8'h00; // addi x3, x0, 12
imem[340] = 8'h63; imem[341] = 8'h1a; imem[342] = 8'hdf; imem[343] = 8'h0b; // bne x30, x29, 180

// Test 13: add x30, x1, x2 (0 + -1 = -1)
imem[344] = 8'h93; imem[345] = 8'h00; imem[346] = 8'h00; imem[347] = 8'h00; // addi x1, x0, 0
imem[348] = 8'h13; imem[349] = 8'h01; imem[350] = 8'hf0; imem[351] = 8'hff; // addi x2, x0, -1
imem[352] = 8'h33; imem[353] = 8'h8f; imem[354] = 8'h20; imem[355] = 8'h00; // add x30, x1, x2
imem[356] = 8'h93; imem[357] = 8'h0e; imem[358] = 8'hf0; imem[359] = 8'hff; // addi x29, x0, -1
imem[360] = 8'h93; imem[361] = 8'h01; imem[362] = 8'hd0; imem[363] = 8'h00; // addi x3, x0, 13
imem[364] = 8'h63; imem[365] = 8'h1e; imem[366] = 8'hdf; imem[367] = 8'h09; // bne x30, x29, 156

// Test 14: add x30, x1, x2 (-1 + 1 = 0)
imem[368] = 8'h93; imem[369] = 8'h00; imem[370] = 8'hf0; imem[371] = 8'hff; // addi x1, x0, -1
imem[372] = 8'h13; imem[373] = 8'h01; imem[374] = 8'h10; imem[375] = 8'h00; // addi x2, x0, 1
imem[376] = 8'h33; imem[377] = 8'h8f; imem[378] = 8'h20; imem[379] = 8'h00; // add x30, x1, x2
imem[380] = 8'h93; imem[381] = 8'h0e; imem[382] = 8'h00; imem[383] = 8'h00; // addi x29, x0, 0
imem[384] = 8'h93; imem[385] = 8'h01; imem[386] = 8'he0; imem[387] = 8'h00; // addi x3, x0, 14
imem[388] = 8'h63; imem[389] = 8'h12; imem[390] = 8'hdf; imem[391] = 8'h09; // bne x30, x29, 132

// Test 15: add x30, x1, x2 (-1 + -1 = -2)
imem[392] = 8'h93; imem[393] = 8'h00; imem[394] = 8'hf0; imem[395] = 8'hff; // addi x1, x0, -1
imem[396] = 8'h13; imem[397] = 8'h01; imem[398] = 8'hf0; imem[399] = 8'hff; // addi x2, x0, -1
imem[400] = 8'h33; imem[401] = 8'h8f; imem[402] = 8'h20; imem[403] = 8'h00; // add x30, x1, x2
imem[404] = 8'h93; imem[405] = 8'h0e; imem[406] = 8'he0; imem[407] = 8'hff; // addi x29, x0, -2
imem[408] = 8'h93; imem[409] = 8'h01; imem[410] = 8'hf0; imem[411] = 8'h00; // addi x3, x0, 15
imem[412] = 8'h63; imem[413] = 8'h16; imem[414] = 8'hdf; imem[415] = 8'h07; // bne x30, x29, 108

// Test 16: add x30, x1, x2 (1 + 0x7fffffff = overflow to 0x80000000)
imem[416] = 8'h93; imem[417] = 8'h00; imem[418] = 8'h10; imem[419] = 8'h00; // addi x1, x0, 1
imem[420] = 8'h37; imem[421] = 8'h01; imem[422] = 8'h00; imem[423] = 8'h80; // lui x2, 0x80000
imem[424] = 8'h13; imem[425] = 8'h01; imem[426] = 8'hf1; imem[427] = 8'hff; // addi x2, x2, -1
imem[428] = 8'h33; imem[429] = 8'h8f; imem[430] = 8'h20; imem[431] = 8'h00; // add x30, x1, x2
imem[432] = 8'hb7; imem[433] = 8'h0e; imem[434] = 8'h00; imem[435] = 8'h80; // lui x29, 0x80000
imem[436] = 8'h93; imem[437] = 8'h8e; imem[438] = 8'h0e; imem[439] = 8'h00; // addi x29, x29, 0
imem[440] = 8'h93; imem[441] = 8'h01; imem[442] = 8'h00; imem[443] = 8'h01; // addi x3, x0, 16
imem[444] = 8'h63; imem[445] = 8'h16; imem[446] = 8'hdf; imem[447] = 8'h05; // bne x30, x29, 76

// Test 17: add x1, x1, x2 (destination = source1: 13 + 11 = 24)
imem[448] = 8'h93; imem[449] = 8'h00; imem[450] = 8'hd0; imem[451] = 8'h00; // addi x1, x0, 13
imem[452] = 8'h13; imem[453] = 8'h01; imem[454] = 8'hb0; imem[455] = 8'h00; // addi x2, x0, 11
imem[456] = 8'hb3; imem[457] = 8'h80; imem[458] = 8'h20; imem[459] = 8'h00; // add x1, x1, x2
imem[460] = 8'h93; imem[461] = 8'h0e; imem[462] = 8'h80; imem[463] = 8'h01; // addi x29, x0, 24
imem[464] = 8'h93; imem[465] = 8'h01; imem[466] = 8'h10; imem[467] = 8'h01; // addi x3, x0, 17
imem[468] = 8'h63; imem[469] = 8'h9a; imem[470] = 8'hd0; imem[471] = 8'h03; // bne x1, x29, 52

// Test 18: add x2, x1, x2 (destination = source2: 14 + 11 = 25)
imem[472] = 8'h93; imem[473] = 8'h00; imem[474] = 8'he0; imem[475] = 8'h00; // addi x1, x0, 14
imem[476] = 8'h13; imem[477] = 8'h01; imem[478] = 8'hb0; imem[479] = 8'h00; // addi x2, x0, 11
imem[480] = 8'h33; imem[481] = 8'h81; imem[482] = 8'h20; imem[483] = 8'h00; // add x2 x1 x2
imem[484] = 8'h93; imem[485] = 8'h0e; imem[486] = 8'h90; imem[487] = 8'h01; // 
imem[488] = 8'h93; imem[489] = 8'h01; imem[490] = 8'h20; imem[491] = 8'h01; // 
imem[492] = 8'h63; imem[493] = 8'h1e; imem[494] = 8'h1d; imem[495] = 8'h01; // 

// Test 19: 
imem[496] = 8'h93; imem[497] = 8'h00; imem[498] = 8'hd0; imem[499] = 8'h00; // 
imem[500] = 8'hb3; imem[501] = 8'h80; imem[502] = 8'h10; imem[503] = 8'h00; // 
imem[504] = 8'h93; imem[505] = 8'h0e; imem[506] = 8'ha0; imem[507] = 8'h01; // 
imem[508] = 8'h93; imem[509] = 8'h01; imem[510] = 8'h30; imem[511] = 8'h01; // 
imem[512] = 8'h63; imem[513] = 8'h94; imem[514] = 8'hd0; imem[515] = 8'h01; // 

//Pass
imem[516] = 8'h13; imem[517] = 8'h05; imem[518] = 8'h10; imem[519] = 8'h00; // 
imem[520] = 8'h73; imem[521] = 8'h00; imem[522] = 8'h10; imem[523] = 8'h00; // ebreak

//fail
imem[524] = 8'h37; imem[525] = 8'he5; imem[526] = 8'h00; imem[527] = 8'h00; // 
imem[528] = 8'h13; imem[529] = 8'h05; imem[530] = 8'hd5; imem[531] = 8'hea; // 
imem[532] = 8'h73; imem[533] = 8'h00; imem[534] = 8'h10; imem[535] = 8'h00; // ebreak


       // $readmemh("program.mem", imem);
     /* imem[0] = 8'hb7;
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
        imem[13] = 8'hbe;
        imem[14] = 8'h00;
        imem[15] = 8'h00;

      // imem[16] = 8'hb7;
      // imem[17] = 8'h41;
      // imem[18] = 8'h00;
      // imem[19] = 8'h00;

        imem[16] = 8'h63;
        imem[17] = 8'h13;
        imem[18] = 8'hdf;
        imem[19] = 8'h01;
////////////////////////////////////////
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

        imem[36] = 8'he3;
        imem[37] = 8'h1e;
        imem[38] = 8'hbe;
        imem[39] = 8'hff; 
        */

        // Reset the dut.
        $display("Resetting hart.");
        @(negedge clk); rst = 1;
        @(negedge clk); rst = 0;

        $display("Cycle  PC        Inst     rs1            rs2            [rd, load, store]");
        cycles = 0;
        while (!halt ) begin
        //while (cycles != 134) begin
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
        $display("r[a0]=%08h (%d)", dut.rf.mem[10], dut.rf.mem[10]);
        $stop;
    end
    always
        #5 clk = ~clk;
endmodule
