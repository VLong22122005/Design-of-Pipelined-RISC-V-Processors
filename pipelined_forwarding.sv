// //----------------------------------------------------------------------//
// //  Design Note
// //----------------------------------------------------------------------//
// //  1. Instruction Memory Depth (IMEM): At least 8  kiB to run the "isa_1b.hex" or "isa_4b.hex"
// //  2. Data        Memory Depth (DMEM): At least 64 kiB (0x0000_0000 - 0x0000_FFFF)
// //  3. IMEM and DMEM are separate memory blocks.

module pipelined (
    input  logic         i_clk     ,
    input  logic         i_reset   ,
    // Input peripherals
    input  logic [31:0]  i_io_sw   ,
    // Output peripherals
    output logic [31:0]  o_io_lcd  ,
    output logic [31:0]  o_io_ledr ,
    output logic [31:0]  o_io_ledg ,
    output logic [ 6:0]  o_io_hex0 ,
    output logic [ 6:0]  o_io_hex1 ,
    output logic [ 6:0]  o_io_hex2 ,
    output logic [ 6:0]  o_io_hex3 ,
    output logic [ 6:0]  o_io_hex4 ,
    output logic [ 6:0]  o_io_hex5 ,
    output logic [ 6:0]  o_io_hex6 ,
    output logic [ 6:0]  o_io_hex7 ,
    // Debug
    output logic [31:0]  o_pc_debug,
    output logic         o_insn_vld,
    output logic         o_ctrl    ,
    output logic         o_mispred
);

    // IF stage
    logic [31:0] pc_F;
    logic [31:0] pc_next_F;
    logic [31:0] pcplus4_F;
    logic [31:0] instr_F;

    // Global pipeline control
    logic        stall_F, stall_D, stall_E;
    logic        flush_D, flush_E, flush_M;

    // IF/ID stage
    logic [31:0] instr_D;
    logic [31:0] pc_D;
    logic [31:0] pcplus4_D;
    logic        insn_vld_D;

    // Decode stage: instruction fields
    logic [6:0]  opcode_D;
    logic [2:0]  funct3_D;
    logic [6:0]  funct7_D;
    logic [4:0]  rs1_D, rs2_D, rd_D;

    // Control signals from control_unit (D stage)
    logic        rd_wren_D;
    logic        opa_sel_D;
    logic        opb_sel_D;
    logic [3:0]  alu_op_D;
    logic        mem_wren_D;
    logic [1:0]  wb_sel_D;   
    logic        is_load_D;
    logic        is_jump_D;
    logic        is_branch_D;
    logic        insn_vld_ctrl;
    logic        ctrl_D;     // branch/jump flag

    // Register file (D stage)
    logic [31:0] rs1_data_D, rs2_data_D;

    // Immediate (D stage)
    logic [31:0] imm_D;

    // MEM/WB stage register writeback info
    logic [4:0]  rd_WB;
    logic [31:0] wb_data_WB;
    logic        rd_wren_WB;

    // ID/EX pipeline register (E stage inputs)
    logic [31:0] rs1_data_E, rs2_data_E;
    logic [4:0]  rs1_E, rs2_E, rd_E;
    logic [31:0] imm_E;
    logic [31:0] pc_E, pcplus4_E;

    logic        is_load_E;
    logic        rd_wren_E;
    logic        opa_sel_E;
    logic        opb_sel_E;
    logic [3:0]  alu_op_E;
    logic        mem_wren_E;
    logic [1:0]  wb_sel_E;     
    logic        is_jump_E;
    logic        is_branch_E;
    logic        insn_vld_E;
    logic        ctrl_E;

    // funct3 pipelining (for BRC + LSU)
    logic [2:0]  funct3_E, funct3_M, funct3_WB;

    // ALU / forwarding operands (E stage)
    logic [31:0] op_a_E, op_b_E;
    logic [31:0] fwdata1, fwdata2;
    logic [31:0] alu_data_E;

    // Branch comparator
    logic        br_taken_E;
    logic        take_branch_E;

    // EX/MEM pipeline register (M stage inputs)
    logic [31:0] alu_data_M, wr_data_M;
    logic [4:0]  rd_M;
    logic [31:0] pc_M, pcplus4_M;

    logic        is_load_M;
    logic        rd_wren_M;
    logic        mem_wren_M;
    logic [1:0]  wb_sel_M;
    logic        insn_vld_M;
    logic        ctrl_M;
    logic        mispred_M;
    logic        is_jump_M;

    // LSU + Data memory (M stage)
    logic [31:0] ld_data_M;
    logic [31:0] lsu_ledr_M;
    logic [31:0] lsu_ledg_M;
    logic [31:0] lsu_lcd_M;
    logic [6:0]  lsu_hex0_M, lsu_hex1_M, lsu_hex2_M, lsu_hex3_M;
    logic [6:0]  lsu_hex4_M, lsu_hex5_M, lsu_hex6_M, lsu_hex7_M;

    // MEM/WB pipeline register (WB stage)
    logic [31:0] alu_data_WB;
    logic [31:0] pcplus4_WB;
    logic [31:0] lcd_WB;
    logic [31:0] io_sw_M;
    logic [31:0] ld_data_WB;

    logic        is_load_WB;
    logic [1:0]  wb_sel_WB;
    logic        insn_vld_WB;
    logic        mispred_WB;
    logic        ctrl_WB;

    // Forwarding control
    logic [1:0]  fwrs1, fwrs2;

    assign pc_next_F = (take_branch_E) ? {alu_data_E[31:1], 1'b0} : pcplus4_F;

    // ============================================================
    // IF stage
    // ============================================================

    // Program counter
    pc u_pc (
        .i_clk     (i_clk),
        .i_reset   (i_reset),
        .i_stall   (stall_F),
        .i_pc_next (pc_next_F),
        .o_pc      (pc_F)
    );

    // PC + 4
    pc_plus_4 u_pc_plus4 (
        .i_pc      (pc_F),
        .o_pc_four (pcplus4_F)
    );

    // Instruction memory (uses stall input)
    instruction_memory u_imem (
        .i_pc    (pc_F),
        .o_instr (instr_F)
    );

    // ============================================================
    // IF/ID pipeline register
    // ============================================================

    if_id_reg u_if_id (
        .i_clk      (i_clk),
        .i_reset    (i_reset),
        .i_flush    (flush_D),
        .i_stall    (stall_D),
        .i_pc       (pc_F),
        .i_pcplus4  (pcplus4_F),
        .instr      (instr_F),

        .instr_D    (instr_D),
        .pc_D       (pc_D),
        .pcplus4_D  (pcplus4_D),
        .insn_vld_D (insn_vld_D)
    );

    // ============================================================
    // Decode stage (D)
    // ============================================================

    // Instruction fields
    assign opcode_D = instr_D[6:0];
    assign funct3_D = instr_D[14:12];
    assign funct7_D = instr_D[31:25];
    assign rd_D     = instr_D[11:7];
    assign rs1_D    = instr_D[19:15];
    assign rs2_D    = instr_D[24:20];

    control_unit u_ctrl (
        .opcode   (opcode_D),
        .funct3   (funct3_D),
        .funct7   (funct7_D),

        .rd_wren  (rd_wren_D),
        .opa_sel  (opa_sel_D),
        .opb_sel  (opb_sel_D),
        .alu_op   (alu_op_D),
        .mem_wren (mem_wren_D),
        .wb_sel   (wb_sel_D),
        .is_load  (is_load_D),
        .is_jump  (is_jump_D),
        .is_branch(is_branch_D),
        .insn_vld (insn_vld_ctrl),
        .ctrl     (ctrl_D)
    );

    // Immediate generator
    immgen u_immgen (
        .i_instruction (instr_D),
        .o_imm         (imm_D)
    );

    // ============================================================
    // Register file
    // ============================================================

    regfile_2 u_regfile (
        .i_clk     (i_clk),
        .i_reset   (i_reset),
        .i_rs1_addr(rs1_D),
        .i_rs2_addr(rs2_D),
        .o_rs1_data(rs1_data_D),
        .o_rs2_data(rs2_data_D),
        .i_rd_addr (rd_WB),
        .i_rd_data (wb_data_WB),
        .i_rd_wren (rd_wren_WB)
    );

    // ============================================================
    // ID/EX pipeline register (E stage inputs)
    // ============================================================

    id_ex_reg u_id_ex (
        .i_clk       (i_clk),
        .i_reset     (i_reset),
        .i_flush     (flush_E),
        .i_stall     (stall_E),

        .rs1_data    (rs1_data_D),
        .rs2_data    (rs2_data_D),
        .rs1         (rs1_D),
        .rs2         (rs2_D),
        .rd          (rd_D),
        .imm         (imm_D),
        .pc_D        (pc_D),
        .pcplus4_D   (pcplus4_D),

        .rd_wren     (rd_wren_D),
        .opa_sel     (opa_sel_D),
        .opb_sel     (opb_sel_D),
        .alu_op      (alu_op_D),
        .mem_wren    (mem_wren_D),
        .wb_sel      (wb_sel_D),  
        .is_load     (is_load_D),
        .is_jump     (is_jump_D),
        .is_branch   (is_branch_D),
        .ctrl        (ctrl_D),
        .insn_vld_D  (insn_vld_D&insn_vld_ctrl),

        .rs1_data_E  (rs1_data_E),
        .rs2_data_E  (rs2_data_E),
        .rs1_E       (rs1_E),
        .rs2_E       (rs2_E),
        .rd_E        (rd_E),
        .imm_E       (imm_E),
        .pc_E        (pc_E),
        .pcplus4_E   (pcplus4_E),

        .is_load_E   (is_load_E),
        .rd_wren_E   (rd_wren_E),
        .opa_sel_E   (opa_sel_E),
        .opb_sel_E   (opb_sel_E),
        .alu_op_E    (alu_op_E),
        .mem_wren_E  (mem_wren_E),
        .wb_sel_E    (wb_sel_E),
        .is_jump_E   (is_jump_E),
        .is_branch_E (is_branch_E),
        .insn_vld_E  (insn_vld_E),
        .ctrl_E      (ctrl_E)
    );

    // ============================================================
    // EX stage
    // ============================================================

    // Pipeline for funct3 into EX and MEM (for BRC and LSU)
    always_ff @(posedge i_clk) begin
        if (!i_reset || flush_E)
            funct3_E <= 3'b0;
        else if (stall_E) 
            funct3_E <= funct3_E;
        else
            funct3_E <= funct3_D;
    end

    always_ff @(posedge i_clk) begin
        if (!i_reset)
            funct3_M <= 3'b0;
        else
            funct3_M <= funct3_E;
    end

    always_ff @(posedge i_clk) begin
        if (!i_reset)
            funct3_WB <= 3'b0;
        else
            funct3_WB <= funct3_M;
    end

    // ALU operands (forwarding)
    always_comb begin
        case (fwrs1)
            2'b00: fwdata1 = rs1_data_E;
            2'b01: fwdata1 = (is_jump_M)? pcplus4_M : alu_data_M;
            2'b10: fwdata1 = wb_data_WB;
            2'b11: fwdata1 = 32'b0; 
        endcase

        case (fwrs2)
            2'b00: fwdata2 = rs2_data_E;
            2'b01: fwdata2 = (is_jump_M)? pcplus4_M : alu_data_M;
            2'b10: fwdata2 = wb_data_WB;
            2'b11: fwdata2 = 32'b0; 
        endcase
    end

    always_comb begin
        op_a_E = opa_sel_E ? fwdata1 : pc_E;
        op_b_E = opb_sel_E ? imm_E   : fwdata2;
    end

    alu u_alu (
        .i_op_a    (op_a_E),
        .i_op_b    (op_b_E),
        .i_alu_op  (alu_op_E),
        .o_alu_data(alu_data_E)
    );

    // Branch comparator
    brc u_brc (
        .i_rs1_data (fwdata1),
        .i_rs2_data (fwdata2),
        .funct3     (funct3_E),
        .br_taken   (br_taken_E)
    );

    // Actual control transfer taken?
    assign take_branch_E = (is_branch_E & br_taken_E) | is_jump_E;

    ex_mem_reg u_ex_mem (
        .i_clk        (i_clk),
        .i_reset      (i_reset),
        .i_flush      (flush_M),

        .alu_data_E   (alu_data_E),
        .wr_data_E    (fwdata2),
        .rd_E         (rd_E),
        .pc_E         (pc_E),
        .pcplus4_E    (pcplus4_E),

        .is_load_E    (is_load_E),
        .rd_wren_E    (rd_wren_E),
        .mem_wren_E   (mem_wren_E),
        .wb_sel_E     (wb_sel_E),
        .insn_vld_E   (insn_vld_E),
        .ctrl_E       (ctrl_E),
        .take_branch  (take_branch_E),
        .is_jump_E    (is_jump_E),

        .alu_data_M   (alu_data_M),
        .wr_data_M    (wr_data_M),
        .rd_M         (rd_M),
        .pc_M         (pc_M),
        .pcplus4_M    (pcplus4_M),

        .is_load_M    (is_load_M),
        .rd_wren_M    (rd_wren_M),
        .mem_wren_M   (mem_wren_M),
        .wb_sel_M     (wb_sel_M),
        .insn_vld_M   (insn_vld_M),
        .ctrl_M       (ctrl_M),
        .mispred_M    (mispred_M),
        .is_jump_M    (is_jump_M)
    );  

    // ============================================================
    // LSU + Data memory (M stage)
    // ============================================================

    lsu u_lsu (
        .i_clk        (i_clk),
        .i_reset      (i_reset),

        .i_funct3_M   (funct3_M),
        .i_funct3_WB  (funct3_WB),

        .is_load_WB   (is_load_WB),
        .read_addr_M  (alu_data_M),
        .read_addr_WB (alu_data_WB),
        
        .i_st_data    (wr_data_M),
        .i_lsu_wren   (mem_wren_M),

        .o_ld_data    (ld_data_M),

        .o_io_ledr    (lsu_ledr_M),
        .o_io_ledg    (lsu_ledg_M),
        .o_io_lcd     (lsu_lcd_M),

        .o_io_hex0    (lsu_hex0_M),
        .o_io_hex1    (lsu_hex1_M),
        .o_io_hex2    (lsu_hex2_M),
        .o_io_hex3    (lsu_hex3_M),
        .o_io_hex4    (lsu_hex4_M),
        .o_io_hex5    (lsu_hex5_M),
        .o_io_hex6    (lsu_hex6_M),
        .o_io_hex7    (lsu_hex7_M),

        .i_io_sw      (i_io_sw)
    );

    // ============================================================
    // MEM/WB pipeline register (WB stage)
    // ============================================================

    mem_wb_reg u_mem_wb (
        .i_clk        (i_clk),
        .i_reset      (i_reset),

        .alu_data_M   (alu_data_M),
        .rd_M         (rd_M),
        .pc_M         (pc_M),
        .pcplus4_M    (pcplus4_M),

        .is_load_M    (is_load_M),
        .rd_wren_M    (rd_wren_M),
        .wb_sel_M     (wb_sel_M),
        .insn_vld_M   (insn_vld_M),
        .mispred_M    (mispred_M),
        .ctrl_M       (ctrl_M),
        .ld_data_M    (ld_data_M),

        .alu_data_WB  (alu_data_WB),
        .rd_WB        (rd_WB),
        .o_pc_debug   (o_pc_debug),
        .pcplus4_WB   (pcplus4_WB),
        .ld_data_WB   (ld_data_WB),
        
        .is_load_WB   (is_load_WB),
        .rd_wren_WB   (rd_wren_WB),
        .wb_sel_WB    (wb_sel_WB),
        .o_insn_vld   (o_insn_vld),
        .o_mispred    (o_mispred),
        .o_ctrl       (o_ctrl)
    );

    // WB data MUX
    always_comb begin
        case (wb_sel_WB)
            2'b00: wb_data_WB = pcplus4_WB;   // JAL / JALR
            2'b01: wb_data_WB = alu_data_WB;  // ALU / LUI / AUIPC
            2'b10: wb_data_WB = ld_data_M;    // Loads
            default: wb_data_WB = 32'b0;
        endcase
    end

    // ============================================================
    // Hazard Detection
    // ============================================================

    hazard_detection u_hazard (
        .i_lsu_wren   (mem_wren_D),
        .is_load_E    (is_load_E),
        .is_load_D    (is_load_D),

        .rs1_D        (rs1_D),
        .rs2_D        (rs2_D),

        .rd_E         (rd_E),
        .rd_wren_E    (rd_wren_E),

        .rd_WB        (rd_WB),
        .rd_wren_WB   (rd_wren_WB),

        .take_branch_E(take_branch_E),

        .stall_E      (stall_E),
        .stall_F      (stall_F),
        .stall_D      (stall_D),
        .flush_D      (flush_D),
        .flush_E      (flush_E),
        .flush_M      (flush_M)
    );

    // ============================================================
    // Forwarding Unit
    // ============================================================

    forwarding_unit fw1 (
        .rs1_E      (rs1_E),
        .rs2_E      (rs2_E),
        .rd_M       (rd_M),
        .rd_WB      (rd_WB),
        .is_load_M  (is_load_M),
        .is_load_WB (is_load_WB),
        .rd_wren_M  (rd_wren_M),
        .rd_wren_WB (rd_wren_WB),
        .fwrs1      (fwrs1),
        .fwrs2      (fwrs2)
    );

    // ============================================================
    // Top-level I/O mapping
    // ============================================================

    assign o_io_ledr = lsu_ledr_M;
    assign o_io_ledg = lsu_ledg_M;
    assign o_io_lcd  = lsu_lcd_M;

    assign o_io_hex0 = lsu_hex0_M;
    assign o_io_hex1 = lsu_hex1_M;
    assign o_io_hex2 = lsu_hex2_M;
    assign o_io_hex3 = lsu_hex3_M;

    assign o_io_hex4 = lsu_hex4_M;
    assign o_io_hex5 = lsu_hex5_M;
    assign o_io_hex6 = lsu_hex6_M;
    assign o_io_hex7 = lsu_hex7_M;

endmodule : pipelined
