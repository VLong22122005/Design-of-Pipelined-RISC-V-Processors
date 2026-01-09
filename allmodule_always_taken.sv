//===========================module for alu=======================
module add (
    input  logic[31:0] a,
    input  logic[31:0] b,
    output logic[31:0] s
);  
    logic[32:0] c ; 
    
    always_comb begin
    c[0] = 1'b0 ; 
    for(integer i=0 ; i<32; i++) begin
        s[i] = a[i]^b[i]^c[i];
        c[i+1] = a[i]&b[i] | a[i]&c[i] | c[i]&b[i] ;
    end 
    end

endmodule

module sub (
    input  logic[31:0] a,
    input  logic[31:0] b,
    output logic[31:0] s
);  
    logic[32:0] c ; 
    logic[31:0] bnot ; 
    assign bnot = ~b ;
    
    always_comb begin
    c[0] = 1'b1 ;  
    for(integer i=0 ; i<32; i++) begin
        s[i] = a[i]^bnot[i]^c[i];
        c[i+1] = a[i]&bnot[i] | a[i]&c[i] | c[i]&bnot[i] ;
    end 
    end
endmodule

module slt(
    input  logic[31:0] a,
    input  logic[31:0] b,
    output logic[31:0] o
);
    logic[31:0] s ; 
    logic[32:0] c ; 
    logic[31:0] bnot ; 
    logic overflow ; 
    logic neg ; 
    assign bnot = ~b ;
    
    always_comb begin
    c[0] = 1'b1 ;  
    for(integer i=0 ; i<32; i++) begin
        s[i] = a[i]^bnot[i]^c[i];
        c[i+1] = a[i]&bnot[i] | a[i]&c[i] | c[i]&bnot[i] ;
    end 
    end
    assign overflow = c[32]^c[31];
    assign neg = (overflow&(~s[31])) | ((~overflow)&(s[31])) ; 
    assign o = {31'b0 , neg} ; 

endmodule

module sltu(
    input  logic[31:0] a,
    input  logic[31:0] b,
    output logic[31:0] o
);  
    logic [32:0] s ; 
    logic [32:0] a_33bit ; 
    assign a_33bit = {1'b0, a[31:0]} ; 
    logic [32:0] b_33bit ; 
    assign b_33bit = {1'b0, b[31:0]} ;
    logic[32:0] bnot ; 
    assign bnot = ~b_33bit ;
    logic[33:0] c ;
 
    always_comb begin 
        c[0] = 1 ;
        for(integer i=0 ; i<33; i++) begin
        s[i] = a_33bit[i]^bnot[i]^c[i];
        c[i+1] = a_33bit[i]&bnot[i] | a_33bit[i]&c[i] | c[i]&bnot[i] ;
        end 
    end
    assign o = {31'b0, s[32]} ;
 
endmodule

module sll(
    input  logic[31:0] a,
    input  logic[31:0] b,
    output logic[31:0] c
);
    logic[31:0] temp ; 

    always_comb begin
        temp = a ; 
        if (b[0]) temp = {temp[30:0],  1'b0};
        if (b[1]) temp = {temp[29:0],  2'b0};
        if (b[2]) temp = {temp[27:0],  4'b0};
        if (b[3]) temp = {temp[23:0],  8'b0};
        if (b[4]) temp = {temp[15:0], 16'b0};
    end

    assign c = temp ; 

endmodule

module srl(
    input  logic[31:0] a,
    input  logic[31:0] b,
    output logic[31:0] c
); 
    logic[31:0] temp ; 

    always_comb begin
        temp = a ; 
        if (b[0]) temp = {1'b0 , temp[31:1] };
        if (b[1]) temp = {2'b0 , temp[31:2] };
        if (b[2]) temp = {4'b0 , temp[31:4] };
        if (b[3]) temp = {8'b0 , temp[31:8] };
        if (b[4]) temp = {16'b0, temp[31:16]};
    end

    assign c = temp ;
endmodule

module sra(
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] c
); 
 logic[31:0] temp ; 

    always_comb begin
        temp = a ; 
        if (b[0]) temp = {{1{temp[31]}} , temp[31:1] };
        if (b[1]) temp = {{2{temp[31]}} , temp[31:2] };
        if (b[2]) temp = {{4{temp[31]}} , temp[31:4] };
        if (b[3]) temp = {{8{temp[31]}} , temp[31:8] };
        if (b[4]) temp = {{16{temp[31]}}, temp[31:16]};
    end

    assign c = temp ;
endmodule

module alu(
    input  logic[31:0] i_op_a,
    input  logic[31:0] i_op_b,
    input  logic[3:0]  i_alu_op,
    output logic[31:0] o_alu_data
);
    logic[31:0] add_out, sub_out, sll_out, slt_out, sltu_out, srl_out, sra_out; 
    add  add_alu(.a(i_op_a), .b(i_op_b), .s(add_out)) ;
    sub  sub_alu(.a(i_op_a), .b(i_op_b), .s(sub_out)) ;
    sll  sll_alu(.a(i_op_a), .b(i_op_b), .c(sll_out)) ;
    slt  slt_alu(.a(i_op_a), .b(i_op_b), .o(slt_out)) ;
    sltu sltu_alu(.a(i_op_a), .b(i_op_b), .o(sltu_out));
    srl  srl_alu(.a(i_op_a), .b(i_op_b), .c(srl_out)) ;
    sra  sra_alu(.a(i_op_a), .b(i_op_b), .c(sra_out)) ;
    always_comb begin
        case(i_alu_op)
            4'b0000: o_alu_data = add_out ; 
            4'b1000: o_alu_data = sub_out ; 
            4'b0001: o_alu_data = sll_out ; 
            4'b0010: o_alu_data = slt_out ;
            4'b0011: o_alu_data = sltu_out ; 
            4'b0100: o_alu_data = i_op_a ^ i_op_b ; 
            4'b0101: o_alu_data = srl_out ;
            4'b1101: o_alu_data = sra_out ;
            4'b0110: o_alu_data = i_op_a | i_op_b ; 
            4'b0111: o_alu_data = i_op_a & i_op_b ;
            4'b1111: o_alu_data = i_op_b ;    //immediate
            default: o_alu_data = 0 ; 
        endcase
    end

endmodule
//=================================================================

//===============================decoder===========================
module decoder1to2(
    input logic en,
    input logic in,
    output logic[1:0] out
);
    assign out[0] = en & ~in ; 
    assign out[1] = en &  in ; 

 endmodule

module decoder2to4(
    input logic en,
    input logic[1:0] in,
    output logic[3:0] out
);
    logic[1:0] en_wire ;
    decoder1to2 decoder1(.en(en) , .in(in[1]) , .out(en_wire))  ;
    decoder1to2 decoder2(.en(en_wire[0]) , .in(in[0]) , .out(out[1:0]))  ;
    decoder1to2 decoder3(.en(en_wire[1]) , .in(in[0]) , .out(out[3:2]))  ;

endmodule

module decoder3to8(
    input logic en,
    input logic[2:0] in,
    output logic[7:0] out
);
    logic[1:0] en_wire ;
    decoder1to2 decoder1(.en(en) , .in(in[2]) , .out(en_wire))  ;
    decoder2to4 decoder2(.en(en_wire[0]) , .in(in[1:0]) , .out(out[3:0]))  ;
    decoder2to4 decoder3(.en(en_wire[1]) , .in(in[1:0]) , .out(out[7:4]))  ;

endmodule

module decoder4to16(
    input logic en,
    input logic[3:0] in,
    output logic[15:0] out
);
    logic[1:0] en_wire ;
    decoder1to2 decoder1(.en(en) , .in(in[3]) , .out(en_wire))  ;
    decoder3to8 decoder2(.en(en_wire[0]) , .in(in[2:0]) , .out(out[7:0]))  ;
    decoder3to8 decoder3(.en(en_wire[1]) , .in(in[2:0]) , .out(out[15:8]))  ;

endmodule

module decoder5to32(
    input logic en,
    input logic[4:0] in,
    output logic[31:0] out
);
    logic[1:0] en_wire ;
    decoder1to2  decoder1(.en(en) , .in(in[4]) , .out(en_wire))  ;
    decoder4to16 decoder2(.en(en_wire[0]) , .in(in[3:0]) , .out(out[15:0]))  ;
    decoder4to16 decoder3(.en(en_wire[1]) , .in(in[3:0]) , .out(out[31:16]))  ;

endmodule
//===============================================================================

//==================================mux==========================================
module mux2to1(
    input  logic[31:0] a ,  
    input  logic[31:0] b ,
    input  logic sel , 
    output logic[31:0] c
) ; 
    logic[31:0] s ; 
    assign s = {32{sel}} ; 
    assign c = (~s & a) | (s & b); 
endmodule

module mux4to1(
    input  logic[31:0] in[3:0] , 
    input  logic[1:0] sel ,
    output logic[31:0] out
);
    logic[31:0] stage[1:0] ; 
    mux2to1 mux1(.a(in[0]) , .b(in[1]) , .sel(sel[0]) , .c(stage[0])) ; 
    mux2to1 mux2(.a(in[2]) , .b(in[3]) , .sel(sel[0]) , .c(stage[1])) ; 
    mux2to1 mux3(.a(stage[0]) , .b(stage[1]) , .sel(sel[1]) , .c(out)) ;

endmodule

module mux8to1(
    input  logic[31:0] in[7:0],
    input  logic[2:0] sel,
    output logic[31:0] out
);
    logic[31:0] stage[1:0] ;
    mux4to1 mux1(.in(in[3:0]) , .sel(sel[1:0]) , .out(stage[0])) ; 
    mux4to1 mux2(.in(in[7:4]) , .sel(sel[1:0]) , .out(stage[1])) ;
    mux2to1 mux3(.a(stage[0]) , .b(stage[1]) , .sel(sel[2]) , .c(out)) ;

endmodule

module mux16to1(
    input  logic[31:0] in[15:0],
    input  logic[3:0] sel,
    output logic[31:0] out
);
    logic[31:0] stage[1:0] ;
    mux8to1 mux1(.in(in[7:0]) , .sel(sel[2:0]) , .out(stage[0])) ; 
    mux8to1 mux2(.in(in[15:8]) , .sel(sel[2:0]) , .out(stage[1])) ;
    mux2to1 mux3(.a(stage[0]) , .b(stage[1]) , .sel(sel[3]) , .c(out)) ;

endmodule

module mux32to1(
    input  logic[31:0] in[31:0],
    input  logic[4:0] sel,
    output logic[31:0] out
);
    logic[31:0] stage[1:0] ;
    mux16to1 mux1(.in(in[15:0]) , .sel(sel[3:0]) , .out(stage[0])) ; 
    mux16to1 mux2(.in(in[31:16]) , .sel(sel[3:0]) , .out(stage[1])) ;
    mux2to1  mux3(.a(stage[0]) , .b(stage[1]) , .sel(sel[4]) , .c(out)) ;

endmodule
//===========================================================================

module regfile_2(
    input  logic        i_clk,
    input  logic        i_reset,
    input  logic [4:0]  i_rs1_addr,
    input  logic [4:0]  i_rs2_addr,
    output logic [31:0] o_rs1_data,
    output logic [31:0] o_rs2_data,
    input  logic [4:0]  i_rd_addr,
    input  logic [31:0] i_rd_data,
    input  logic        i_rd_wren
);
    logic [31:0] register [31:0] ;
    logic [31:0] wren ; 
    logic [31:0] reset ; 
    assign reset = {32{i_reset}} ;

    decoder5to32 addr(.en(i_rd_wren) , .in(i_rd_addr) , .out(wren)) ; 

    genvar i ; 
    generate
        for(i = 1 ; i<32 ; i++) begin : reg_block
            register_single Reg(.i_clk(i_clk) ,
                                .i_data(i_rd_data) ,
                                .i_reset(reset[i]) ,
                                .i_en(wren[i]) ,
                                .odata(register[i])) ; 
        end
    endgenerate

    register_single reg0(.i_clk(i_clk) ,
                                .i_data(32'b0) ,
                                .i_reset(reset[0]) ,
                                .i_en(wren[0]) ,
                                .odata(register[0])) ; 

    mux32to1 mux1(.in(register) , .sel(i_rs1_addr) , .out(o_rs1_data)) ; 
    mux32to1 mux2(.in(register) , .sel(i_rs2_addr) , .out(o_rs2_data)) ; 


endmodule 


module lsu (
    input  logic        i_clk,
    input  logic        i_reset,
    
    input  logic[2:0]   i_funct3_M,
    input  logic[2:0]   i_funct3_WB,

    input logic is_load_WB,
    input logic [31:0] read_addr_WB,
    input logic [31:0] read_addr_M, 

    input  logic [31:0] i_st_data,    
    input  logic        i_lsu_wren,   

    output logic [31:0] o_ld_data,    

    // I/O mapped peripherals
    output logic [31:0] o_io_ledr,
    output logic [31:0] o_io_ledg,
    output logic [31:0] o_io_lcd,
    output logic [6:0]  o_io_hex0,
    output logic [6:0]  o_io_hex1,
    output logic [6:0]  o_io_hex2,
    output logic [6:0]  o_io_hex3,
    output logic [6:0]  o_io_hex4,
    output logic [6:0]  o_io_hex5,
    output logic [6:0]  o_io_hex6,
    output logic [6:0]  o_io_hex7,

    input  logic [31:0] i_io_sw        
);
    //select addr, funct3
    logic [31:0] i_lsu_addr ; 
    logic [2:0] i_funct3;
    always_comb begin
        if(is_load_WB) begin
             i_lsu_addr = read_addr_WB ;
             i_funct3   = i_funct3_WB ;
        end
        else begin 
            i_lsu_addr = read_addr_M ; 
            i_funct3   = i_funct3_M ;
        end
    end

    logic[31:0] data_mem , io_ledg , io_ledr , io_lcd , io_sevenseg1 ,io_sevenseg2 , io_sw ;
    assign o_io_ledg = io_ledg ; 
    assign o_io_ledr = io_ledr ; 
    assign o_io_lcd  = io_lcd ; 
    assign {o_io_hex3 , o_io_hex2 , o_io_hex1 , o_io_hex0} = io_sevenseg1[27:0] ; 
    assign {o_io_hex7 , o_io_hex6 , o_io_hex5 , o_io_hex4} = io_sevenseg2[27:0] ;

    logic[31:0] dmem_or_not ;
    slt slt_dmem_or_not(.a(i_lsu_addr), .b(32'hFFFF), .o(dmem_or_not)) ;
    logic is_mem ,  is_ledr , is_ledg , is_lcd , is_sevenseg1 , is_sevenseg2, is_sw ; 
    assign is_mem = dmem_or_not[0] ; 
    assign is_ledr = ~(|((i_lsu_addr & 32'hFFFF_F000) ^ 32'h1000_0000)) ; 
    assign is_ledg = ~(|((i_lsu_addr & 32'hFFFF_F000) ^ 32'h1000_1000)) ;
    assign is_lcd = ~(|((i_lsu_addr & 32'hFFFF_F000) ^ 32'h1000_4000)) ;
    assign is_sevenseg1 = ~(|((i_lsu_addr & 32'hFFFF_F000) ^ 32'h1000_2000)) ;
    assign is_sevenseg2 = ~(|((i_lsu_addr & 32'hFFFF_F000) ^ 32'h1000_3000)) ;
    assign is_sw = ~(|((i_lsu_addr & 32'hFFFF_F000) ^ 32'h1001_0000)) ;
 
    always_comb begin
        if(is_mem) 
            o_ld_data = data_mem ;
        else if(is_ledg) o_ld_data = io_ledg ; 
        else if(is_ledr) o_ld_data = io_ledr ; 
        else if(is_lcd)  o_ld_data = io_lcd ;
        else if(is_sevenseg1) o_ld_data = io_sevenseg1 ;
        else if(is_sevenseg2) o_ld_data = io_sevenseg2 ;
        else if(is_sw) o_ld_data = io_sw ;
        else o_ld_data = 32'b0 ; 
    end

    memory dmem(.i_clk(i_clk) , .i_reset(i_reset) , .i_addr(i_lsu_addr) , .i_st_data(i_st_data) , .i_wren(i_lsu_wren&is_mem) , .i_funct3(i_funct3) , .odata(data_mem)) ; 

    register_single ledg(.i_clk(i_clk) , .i_data(i_st_data) , .i_reset(i_reset) , .i_en(i_lsu_wren&is_ledg) , .odata(io_ledg)) ; 
    register_single ledr(.i_clk(i_clk) , .i_data(i_st_data) , .i_reset(i_reset) , .i_en(i_lsu_wren&is_ledr), .odata(io_ledr)) ; 
    register_single lcd(.i_clk(i_clk) , .i_data(i_st_data) , .i_reset(i_reset) , .i_en(i_lsu_wren&is_lcd) , .odata(io_lcd)) ; 
    register_single sevenseg1(.i_clk(i_clk) , .i_data(i_st_data) , .i_reset(i_reset) , .i_en(i_lsu_wren&is_sevenseg1) , .odata(io_sevenseg1)) ; 
    register_single sevenseg2(.i_clk(i_clk) , .i_data(i_st_data) , .i_reset(i_reset) , .i_en(i_lsu_wren&is_sevenseg2) , .odata(io_sevenseg2)) ;  
    register_single sw(.i_clk(i_clk) , .i_data(i_io_sw) , .i_reset(i_reset) , .i_en(is_sw) , .odata(io_sw)) ;

endmodule

module register_single(
    input logic i_clk ,
    input logic[31:0] i_data,
    input logic i_reset , 
    input logic i_en ,
    output logic[31:0] odata 
);

    always_ff @(posedge i_clk) begin
        if(!i_reset) odata <= 32'b0 ; 
        else if(i_en) odata <= i_data ;
    end

endmodule

module memory (
    input  logic        i_clk,
    input  logic        i_reset,       
    input  logic [31:0] i_addr,        
    input  logic [31:0] i_st_data,       
    input  logic        i_wren,  
    input  logic [2:0]  i_funct3,      
    output logic [31:0] odata
);  
    logic[3:0] bmask1 , bmask2; 
    logic b , h , w , bu , hu ; 
    logic[1:0] b_addr ;  // byte address
    assign b_addr = i_addr[1:0] ; 
    assign b  = ~i_funct3[0] & ~i_funct3[1] & ~i_funct3[2] ; 
    assign h  =  i_funct3[0] & ~i_funct3[1] & ~i_funct3[2] ;
    assign w  = ~i_funct3[0] &  i_funct3[1] & ~i_funct3[2] ;
    assign bu = ~i_funct3[0] & ~i_funct3[1] &  i_funct3[2] ;
    assign hu =  i_funct3[0] & ~i_funct3[1] &  i_funct3[2] ;

    logic[31:0] st_data1 , st_data2 ; 

    always_comb begin    // handle store
        bmask1 = 4'b0 ;
        bmask2 = 4'b0 ; 
        st_data1 = 32'b0 ; 
        st_data2 = 32'b0 ;
        if(b) begin
            case(b_addr)
                2'b00: begin bmask1 = 4'b0001 ; bmask2 = 4'b0000; st_data1 = {24'b0 , i_st_data[7:0]} ; st_data2 = 32'b0; end
                2'b01: begin bmask1 = 4'b0010 ; bmask2 = 4'b0000; st_data1 = {16'b0 , i_st_data[7:0] , 8'b0} ; st_data2 = 32'b0; end
                2'b10: begin bmask1 = 4'b0100 ; bmask2 = 4'b0000; st_data1 = {8'b0 , i_st_data[7:0] , 16'b0} ; st_data2 = 32'b0; end
                2'b11: begin bmask1 = 4'b1000 ; bmask2 = 4'b0000; st_data1 = {i_st_data[7:0] , 24'b0} ; st_data2 = 32'b0; end
            endcase
        end
         else if(h) begin
            case(b_addr)
                2'b00: begin bmask1 = 4'b0011 ; bmask2 = 4'b0000; st_data1 = {16'b0 , i_st_data[15:0]} ; st_data2 = 32'b0; end
                2'b01: begin bmask1 = 4'b0110 ; bmask2 = 4'b0000; st_data1 = {8'b0 , i_st_data[15:0] , 8'b0} ; st_data2 = 32'b0; end
                2'b10: begin bmask1 = 4'b1100 ; bmask2 = 4'b0000; st_data1 = {i_st_data[15:0] , 16'b0} ; st_data2 = 32'b0; end
                2'b11: begin bmask1 = 4'b1000 ; bmask2 = 4'b0001; st_data1 = {i_st_data[7:0] , 24'b0} ; st_data2 = {24'b0 , i_st_data[15:8]}; end
            endcase
        end
         else if(w) begin
            case(b_addr)
                2'b00: begin bmask1 = 4'b1111 ; bmask2 = 4'b0000; st_data1 = i_st_data ; st_data2 = 32'b0; end
                2'b01: begin bmask1 = 4'b1110 ; bmask2 = 4'b0001; st_data1 = {i_st_data[23:0] , 8'b0} ; st_data2 = {24'b0 , i_st_data[31:24]}; end
                2'b10: begin bmask1 = 4'b1100 ; bmask2 = 4'b0011; st_data1 = {i_st_data[15:0] , 16'b0} ; st_data2 = {16'b0 , i_st_data[31:16]}; end
                2'b11: begin bmask1 = 4'b1000 ; bmask2 = 4'b0111; st_data1 = {i_st_data[7:0] , 24'b0} ; st_data2 = {8'b0 , i_st_data[31:8]}; end
            endcase
        end
    end

    logic [31:0] even_mem[0:8191] ; 
    logic [31:0] odd_mem [0:8191] ;

    initial begin
        for (int i = 0; i < 8192; i++) begin
            even_mem[i] = 32'b0;
            odd_mem[i]  = 32'b0;
        end
    end

    logic[31:0] mem_addr , mem_addr_inc ; 
    assign mem_addr = {17'b0 , i_addr[16:2]} ; 
    add add_1(.a(mem_addr) , .b(32'd1) , .s(mem_addr_inc)) ;  

    logic [31:0] mem_addr1 , mem_addr_inc1 ; 
    srl srl1(.a(mem_addr) , .b(32'b1) , .c(mem_addr1)) ; 
    srl srl2(.a(mem_addr_inc) , .b(32'b1) , .c(mem_addr_inc1)) ;

    logic parity ; 
    assign parity = mem_addr[0] ; 

    logic [31:0] even_data , odd_data ; 
    logic [ 3:0] even_mask , odd_mask ; 
    logic [31:0] even_addr , odd_addr ;
    logic [31:0] even_o , odd_o ; 

    always_comb begin
        if(parity) begin
            even_data = st_data2 ; 
            even_mask = bmask2 ; 
            even_addr = mem_addr_inc1 ; 
            odd_data = st_data1 ;
            odd_mask = bmask1 ; 
            odd_addr = mem_addr1 ; 
        end
        else begin
            even_data = st_data1 ; 
            even_mask = bmask1 ;
            even_addr = mem_addr1 ;
            odd_data = st_data2 ; 
            odd_mask = bmask2 ;
            odd_addr = mem_addr_inc1 ; 
        end
    end

    //infer memory to bram
    always_ff @(posedge i_clk) begin 
            if (i_wren) begin
            if (even_mask[0]) even_mem[even_addr][ 7: 0] <= even_data[7:0  ];
            if (even_mask[1]) even_mem[even_addr][15: 8] <= even_data[15:8 ];
            if (even_mask[2]) even_mem[even_addr][23:16] <= even_data[23:16];
            if (even_mask[3]) even_mem[even_addr][31:24] <= even_data[31:24];
        end
        even_o <= even_mem[even_addr]; 
    end

    always_ff @(posedge i_clk) begin 
            if (i_wren) begin
            if (odd_mask[0]) odd_mem[odd_addr][7 :0 ] <= odd_data[7:0  ];
            if (odd_mask[1]) odd_mem[odd_addr][15:8 ] <= odd_data[15:8 ];
            if (odd_mask[2]) odd_mem[odd_addr][23:16] <= odd_data[23:16];
            if (odd_mask[3]) odd_mem[odd_addr][31:24] <= odd_data[31:24];
        end
        odd_o <= odd_mem[odd_addr];
    end


    
    logic[31:0] o_data_mem1 , o_data_mem2 ; 
    assign o_data_mem1 = (parity)? odd_o  : even_o ; 
    assign o_data_mem2 = (parity)? even_o : odd_o;

    always_comb begin    // handle load
        odata = 32'b0 ; 
        if(hu) begin
            case(b_addr)
                2'b00: odata = {16'b0 , o_data_mem1[15:8] , o_data_mem1[7:0]} ; 
                2'b01: odata = {16'b0 , o_data_mem1[23:16] , o_data_mem1[15:8]} ;
                2'b10: odata = {16'b0 , o_data_mem1[31:24] , o_data_mem1[23:16]} ;
                2'b11: odata = {16'b0 , o_data_mem2[7:0] , o_data_mem1[31:24]} ;
            endcase
        end

        if(w) begin
            case(b_addr)
                2'b00: odata = {o_data_mem1[31:24] , o_data_mem1[23:16], o_data_mem1[15:8], o_data_mem1[7:0]} ; 
                2'b01: odata = {o_data_mem2[7:0] , o_data_mem1[31:24] , o_data_mem1[23:16], o_data_mem1[15:8]} ;
                2'b10: odata = {o_data_mem2[15:8] , o_data_mem2[7:0] , o_data_mem1[31:24] , o_data_mem1[23:16]} ;
                2'b11: odata = {o_data_mem2[23:16] , o_data_mem2[15:8] , o_data_mem2[7:0] , o_data_mem1[31:24]} ;
            endcase
        end

        if(bu) begin
            case(b_addr)
                2'b00: odata = {24'b0 , o_data_mem1[7:0]} ; 
                2'b01: odata = {24'b0 , o_data_mem1[15:8]} ;
                2'b10: odata = {24'b0 , o_data_mem1[23:16]} ;
                2'b11: odata = {24'b0 , o_data_mem1[31:24]} ;
            endcase
        end

        if(b) begin
            case(b_addr)
                2'b00: odata = {{24{o_data_mem1[7]}} , o_data_mem1[7:0]} ; 
                2'b01: odata = {{24{o_data_mem1[15]}} , o_data_mem1[15:8]} ;
                2'b10: odata = {{24{o_data_mem1[23]}} , o_data_mem1[23:16]} ;
                2'b11: odata = {{24{o_data_mem1[31]}} , o_data_mem1[31:24]} ;
            endcase
        end

        if(h) begin
            case(b_addr)
                2'b00: odata = {{16{o_data_mem1[15]}} , o_data_mem1[15:8] , o_data_mem1[7:0]} ; 
                2'b01: odata = {{16{o_data_mem1[23]}} , o_data_mem1[23:16] , o_data_mem1[15:8]} ;
                2'b10: odata = {{16{o_data_mem1[31]}} , o_data_mem1[31:24] , o_data_mem1[23:16]} ;
                2'b11: odata = {{16{o_data_mem2[7]}} , o_data_mem2[7:0] , o_data_mem1[31:24]} ;
            endcase
        end

    end

    endmodule

module pc_plus_4(
    input  logic[31:0] i_pc,
    output logic[31:0] o_pc_four
) ; 
    logic[31:0] four ; 
    assign four = 32'd4 ; 
    add add_pc_four(.a(i_pc), .b(four), .s(o_pc_four)) ; 
    
endmodule

module instruction_memory (       
    input  logic [31:0] i_pc,     // 2^11 = 2048Byte                         
    output logic [31:0] o_instr       
);

    logic [31:0] imem [0:2047];

    logic [10:0] pc ;
    assign pc = {i_pc[12:2]} ; 

    initial begin
        $readmemh("isa_4b.hex", imem);
        //$readmemh("test.dump", imem);
    end
    
    assign o_instr = imem[pc] ; 

endmodule

module immgen(
    input logic[31:0] i_instruction,
    output logic[31:0] o_imm
);

    always_comb begin
        case(i_instruction[6:0]) 
        7'b0010011: o_imm = ({{20{i_instruction[31]}} , i_instruction[31:20]}) & 
                          ({{27{i_instruction[13] | ~i_instruction[12]}} , 5'b11111}) ;  // I-type
        7'b1100111: o_imm = {{20{i_instruction[31]}} , i_instruction[31:20]} ;  //jalr
        7'b0100011: o_imm = {{20{i_instruction[31]}} , i_instruction[31:25] , i_instruction[11:7]} ; // S-type
        7'b0000011: o_imm = {{20{i_instruction[31]}} , i_instruction[31:20]} ;  // load
        7'b1100011: o_imm = {{19{i_instruction[31]}} , i_instruction[31] , i_instruction[7] , i_instruction[30:25],
                            i_instruction[11:8] ,1'b0} ;  // B-type
        7'b1101111: o_imm = {{12{i_instruction[31]}} , i_instruction[19:12], i_instruction[20],
                         i_instruction[30:21], 1'b0} ;   //jal
        7'b0110111: o_imm = {i_instruction[31:12], 12'b0};  // lui
        7'b0010111: o_imm = {i_instruction[31:12], 12'b0};  //auipc
        default : o_imm = 32'b0 ; 
        endcase
    end
endmodule

module control_unit(
    input  logic [6:0] opcode,
    input  logic [2:0] funct3,
    input  logic [6:0] funct7,

    output logic       rd_wren,
    output logic       opa_sel,
    output logic       opb_sel,
    output logic [3:0] alu_op,
    output logic       mem_wren,
    output logic [1:0] wb_sel,
    output logic       is_load,
    output logic       is_jump,
    output logic       is_branch,
    output logic       insn_vld,
    output logic       ctrl
);

    localparam [6:0]
        OP_R     = 7'b0110011,
        OP_I     = 7'b0010011,
        OP_L     = 7'b0000011,
        OP_S     = 7'b0100011,
        OP_B     = 7'b1100011,
        OP_LUI   = 7'b0110111,
        OP_AUIPC = 7'b0010111,
        OP_JAL   = 7'b1101111,
        OP_JALR  = 7'b1100111;

    logic is_R, is_I, is_L, is_S, is_B, is_LUI, is_AUIPC, is_JAL, is_JALR ; 

    assign is_R     = ~(|(opcode ^ OP_R)) ; 
    assign is_I     = ~(|(opcode ^ OP_I)) ;
    assign is_L     = ~(|(opcode ^ OP_L)) ;
    assign is_S     = ~(|(opcode ^ OP_S)) ;
    assign is_B     = ~(|(opcode ^ OP_B)) ;
    assign is_LUI   = ~(|(opcode ^ OP_LUI)) ;
    assign is_AUIPC = ~(|(opcode ^ OP_AUIPC)) ;
    assign is_JAL   = ~(|(opcode ^ OP_JAL)) ;
    assign is_JALR  = ~(|(opcode ^ OP_JALR)) ;

    assign rd_wren   = ~(is_S | is_B) ; // only S and B instr don't need this
    assign mem_wren  = is_S ;
    assign is_load   = is_L ; 
    assign is_jump   = is_JAL | is_JALR ; 
    assign is_branch = is_B ; 

    always_comb begin
        if(is_R) alu_op = {funct7[5] , funct3} ; 
        else if(is_I) alu_op = {funct7[5] & (funct3[2] & ~funct3[1] & funct3[0]) , funct3} ; 
        else if(is_LUI) alu_op = 4'b1111 ; 
        else alu_op = 4'b0000 ; 
    end

    always_comb begin
        if(is_JAL | is_JALR) wb_sel = 2'b00 ; 
        else if(is_R | is_I | is_LUI | is_AUIPC) wb_sel = 2'b01 ;
        else if(is_L) wb_sel = 2'b10 ; 
        else wb_sel = 2'b11 ; 
    end

    assign opa_sel = is_R | is_I | is_L | is_S | is_JALR ; 
    assign opb_sel = ~is_R ; 

    assign ctrl = is_branch | is_JAL | is_JALR ; 


    insnvld insn_ckeck(.op(opcode) , .funct3(funct3) , .funct7(funct7) , .valid(insn_vld)) ; 

endmodule

module pc(
    input  logic       i_clk,
    input  logic       i_reset,
    input  logic       i_stall,
    input  logic[31:0] i_pc_next,
    output logic[31:0] o_pc
) ; 
    always_ff @(posedge i_clk) begin
        if(!i_reset) o_pc <= 32'b0 ; 
        else if(i_stall) o_pc <= o_pc ; 
        else o_pc <= i_pc_next ; 
    end 

endmodule

module insnvld(
    input logic[6:0] op,
    input logic[2:0] funct3,
    input logic[6:0] funct7,
    output logic valid
);

    localparam [6:0]
    OP_R     = 7'b0110011,
    OP_I     = 7'b0010011,
    OP_L     = 7'b0000011,
    OP_S     = 7'b0100011,
    OP_B     = 7'b1100011,
    OP_LUI   = 7'b0110111,
    OP_AUIPC = 7'b0010111,
    OP_JAL   = 7'b1101111,
    OP_JALR  = 7'b1100111;

    always_comb begin
        if(~(|(op^OP_R))) begin
            case(funct3)
                3'b000: begin
                    if(~(|funct7) | ~(|(funct7 ^ 7'b0100000))) valid = 1'b1 ; 
                    else valid = 1'b0 ; 
                end

                3'b001: begin
                    if(~(|funct7)) valid = 1'b1 ; 
                    else valid = 1'b0 ;
                end

                3'b010: begin
                    if(~(|funct7)) valid = 1'b1 ; 
                    else valid = 1'b0 ;
                end

                3'b011: begin
                    if(~(|funct7)) valid = 1'b1 ; 
                    else valid = 1'b0 ;
                end

                3'b100: begin
                    if(~(|funct7)) valid = 1'b1 ; 
                    else valid = 1'b0 ;
                end

                3'b101: begin
                    if(~(|funct7) | ~(|(funct7 ^ 7'b0100000))) valid = 1'b1 ; 
                    else valid = 1'b0 ; 
                end

                3'b110: begin
                    if(~(|funct7)) valid = 1'b1 ; 
                    else valid = 1'b0 ;
                end

                3'b111: begin
                    if(~(|funct7)) valid = 1'b1 ; 
                    else valid = 1'b0 ;
                end
            endcase
        end
        else if(~(|(op^OP_I))) begin
            case(funct3)
                3'b000: begin
                    valid = 1'b1 ;
                end

                3'b001: begin
                    if(~(|funct7)) valid = 1'b1 ; 
                    else valid = 1'b0 ;
                end

                3'b010: begin
                    valid = 1'b1 ;
                end

                3'b011: begin
                    valid = 1'b1 ;
                end

                3'b100: begin
                    valid = 1'b1 ;
                end

                3'b101: begin
                    if(~(|funct7) | ~(|(funct7 ^ 7'b0100000))) valid = 1'b1 ; 
                    else valid = 1'b0 ; 
                end

                3'b110: begin
                    valid = 1'b1 ;
                end

                3'b111: begin
                    valid = 1'b1 ;
                end
            endcase
        end
        else if(~(|(op^OP_S))) begin
            case(funct3)
                3'b000 , 3'b001 , 3'b010 : valid = 1'b1 ; 
                default: valid = 1'b0 ;
            endcase
        end
        else if(~(|(op^OP_L))) begin
            case(funct3)
                3'b000 , 3'b001 , 3'b010 , 3'b100 , 3'b101 : valid = 1'b1 ; 
                default: valid = 1'b0 ;
            endcase
        end
        else if(~(|(op^OP_B))) begin
            case(funct3)
                3'b010 , 3'b011 : valid = 1'b0 ; 
                default: valid = 1'b1 ;
            endcase
        end
        else if(~(|(op^OP_JAL))) begin
            valid = 1'b1 ; 
        end
        else if(~(|(op^OP_JALR))) begin
            case(funct3)
                3'b000 : valid = 1'b1 ; 
                default: valid = 1'b0 ;
            endcase
        end
        else if(~(|(op^OP_LUI))) begin
            valid = 1'b1 ;
        end
        else if(~(|(op^OP_AUIPC))) begin
            valid = 1'b1 ;
        end
        else valid = 1'b0 ; 

    end 

endmodule

module brc(
    input  logic [31:0] i_rs1_data,
    input  logic [31:0] i_rs2_data,     
    input  logic [2:0]  funct3,
    output logic        br_taken
);
logic br_greater;
logic beq , bne , blt , bge , bltu, bgeu ;

compare_32b compare(.a(i_rs1_data),.b(i_rs2_data),.less(bltu),.equal(beq),.greater(br_greater));
assign bne = ~beq ; 
assign bgeu = br_greater | beq ; 

  logic sign_rs1 , sign_rs2 , differ_sign ;
  assign sign_rs1 = i_rs1_data[31];
  assign sign_rs2 = i_rs2_data[31];
  assign differ_sign= sign_rs1^sign_rs2;

  always_comb begin
    if (differ_sign)
      blt = sign_rs1; 
    else
      blt = bltu;        
  end

  assign bge = ~blt ; 

  always_comb begin
    case(funct3)
        3'b000 : br_taken = beq ; 
        3'b001 : br_taken = bne ; 
        3'b100 : br_taken = blt ; 
        3'b101 : br_taken = bge ; 
        3'b110 : br_taken = bltu ; 
        3'b111 : br_taken = bgeu ; 
        default: br_taken = 1'b0 ; 
    endcase
  end
 
endmodule

module compare_1b
(
  input logic a,b,
  output logic equal,less,greater
);
assign equal=~(a^b);
assign less=~a&b;
assign greater=a&~b;
endmodule

module compare_2b
(
  input logic [1:0] a,b,
  output logic equal,less,greater
);
logic equal_hi,less_hi,greater_hi;
logic equal_low,less_low,greater_low;
compare_1b c1(.a(a[1]),.b(b[1]),.equal(equal_hi),.less(less_hi),.greater(greater_hi));
compare_1b c2(.a(a[0]),.b(b[0]),.equal(equal_low),.less(less_low),.greater(greater_low));
assign equal=equal_hi & equal_low;
assign less=(less_low & equal_hi)| less_hi;
assign greater=(greater_low & equal_hi)|greater_hi;
endmodule 

module compare_4b
(
  input logic [3:0] a,b,
  output logic equal,less,greater
);
logic equal_hi,less_hi,greater_hi;
logic equal_low,less_low,greater_low;
compare_2b c3(.a(a[3:2]),.b(b[3:2]),.equal(equal_hi),.less(less_hi),.greater(greater_hi));
compare_2b c4(.a(a[1:0]),.b(b[1:0]),.equal(equal_low),.less(less_low),.greater(greater_low));
assign equal=equal_hi & equal_low;
assign less=(less_low & equal_hi)| less_hi;
assign greater=(greater_low & equal_hi)|greater_hi;
endmodule 

module compare_8b
(
  input logic [7:0] a,b,
  output logic equal,less,greater
);
logic equal_hi,less_hi,greater_hi;
logic equal_low,less_low,greater_low;
compare_4b c5(.a(a[7:4]),.b(b[7:4]),.equal(equal_hi),.less(less_hi),.greater(greater_hi));
compare_4b c6(.a(a[3:0]),.b(b[3:0]),.equal(equal_low),.less(less_low),.greater(greater_low));
assign equal=equal_hi & equal_low;
assign less=(less_low & equal_hi)| less_hi;
assign greater=(greater_low & equal_hi)|greater_hi;
endmodule

module compare_16b
(
  input logic [15:0] a,b,
  output logic equal,less,greater
);
logic equal_hi,less_hi,greater_hi;
logic equal_low,less_low,greater_low;
compare_8b c7(.a(a[15:8]),.b(b[15:8]),.equal(equal_hi),.less(less_hi),.greater(greater_hi));
compare_8b c8(.a(a[7:0]),.b(b[7:0]),.equal(equal_low),.less(less_low),.greater(greater_low));
assign equal=equal_hi & equal_low;
assign less=(less_low & equal_hi)| less_hi;
assign greater=(greater_low & equal_hi)|greater_hi;
endmodule  

module compare_32b
(
  input logic [31:0] a,b,
  output logic equal,less,greater
);
logic equal_hi,less_hi,greater_hi;
logic equal_low,less_low,greater_low;
compare_16b c9(.a(a[31:16]),.b(b[31:16]),.equal(equal_hi),.less(less_hi),.greater(greater_hi));
compare_16b c10(.a(a[15:0]),.b(b[15:0]),.equal(equal_low),.less(less_low),.greater(greater_low));
assign equal=equal_hi & equal_low;
assign less=(less_low & equal_hi)| less_hi;
assign greater=(greater_low & equal_hi)|greater_hi;
endmodule  


module if_id_reg(
    input logic i_clk , 
    input logic i_reset,
    input logic i_flush , 
    input logic i_stall,
    input logic [31:0] i_pc,
    input logic [31:0] i_pcplus4,
    input logic [31:0] instr,
    input logic hit_F,
    input logic [31:0] target_F,

    output logic [31:0] instr_D,
    output logic [31:0] pc_D,
    output logic [31:0] pcplus4_D,
    output logic insn_vld_D,
    output logic hit_D,
    output logic [31:0] target_D

);

    always_ff @(posedge i_clk) begin
        if(~i_reset) begin
            instr_D <= 32'b0 ; 
            pc_D <= 32'b0 ; 
            pcplus4_D <= 32'b0 ; 
            insn_vld_D <= 1'b0 ; 
            hit_D <= 1'b0;
            target_D <= 32'b0;
        end
        else if(i_flush) begin
            instr_D <= 32'b0 ; 
            pc_D <= 32'b0 ; 
            pcplus4_D <= 32'b0 ; 
            insn_vld_D <= 1'b0 ;
            hit_D <= 1'b0 ;
            target_D <= 32'b0;
        end
        else if(i_stall) begin
            instr_D <= instr_D ; 
            pc_D <= pc_D ;
            pcplus4_D <= pcplus4_D ;
            insn_vld_D <= insn_vld_D ;  
            hit_D <= hit_D ;
            target_D <= target_D;
        end
        else begin
            instr_D <= instr ; 
            pc_D <= i_pc ;
            pcplus4_D <= i_pcplus4 ; 
            insn_vld_D <= 1'b1 ; 
            hit_D <= hit_F ;
            target_D <= target_F; 
        end
    end

endmodule

module id_ex_reg(
    input logic i_clk,
    input logic i_reset,
    input logic i_flush,
    input logic i_stall,

    input logic [31:0] rs1_data,
    input logic [31:0] rs2_data,
    input logic [ 4:0] rs1,
    input logic [ 4:0] rs2,
    input logic [ 4:0] rd,
    input logic [31:0] imm,
    input logic [31:0] pc_D,
    input logic [31:0] pcplus4_D,

    input logic rd_wren,
    input logic opa_sel,
    input logic opb_sel,
    input logic [3:0] alu_op,
    input logic mem_wren,
    input logic [1:0] wb_sel,
    input logic is_load,
    input logic is_jump,
    input logic is_branch,
    input logic ctrl,
    input logic insn_vld_D,
    input logic hit_D,
    input logic [31:0] target_D,

    output logic [31:0] rs1_data_E,
    output logic [31:0] rs2_data_E,
    output logic [ 4:0] rs1_E,
    output logic [ 4:0] rs2_E,
    output logic [ 4:0] rd_E,
    output logic [31:0] imm_E,
    output logic [31:0] pc_E,
    output logic [31:0] pcplus4_E,

    output logic rd_wren_E,
    output logic opa_sel_E,
    output logic opb_sel_E,
    output logic [3:0] alu_op_E,
    output logic mem_wren_E,
    output logic [1:0] wb_sel_E,
    output logic is_load_E,
    output logic is_jump_E,
    output logic is_branch_E,
    output logic insn_vld_E,
    output logic ctrl_E,
    output logic hit_E,
    output logic [31:0] target_E

);

    always_ff @(posedge i_clk) begin
        if(~i_reset | i_flush) begin
            rs1_data_E   <= 32'b0;
            rs2_data_E   <= 32'b0;
            rs1_E        <= 5'b0;
            rs2_E        <= 5'b0;
            rd_E         <= 5'b0;
            imm_E        <= 32'b0;
            pc_E         <= 32'b0;
            pcplus4_E    <= 32'b0;

            rd_wren_E    <= 1'b0;
            opa_sel_E    <= 1'b0;
            opb_sel_E    <= 1'b0;
            alu_op_E     <= 4'b0;
            mem_wren_E   <= 1'b0;
            wb_sel_E     <= 2'b0;
            is_load_E    <= 1'b0;
            is_jump_E    <= 1'b0;
            is_branch_E  <= 1'b0;

            insn_vld_E   <= 1'b0;  
            ctrl_E       <= 1'b0;
            hit_E        <= 1'b0;
            target_E     <= 32'b0;
        end
        else if(i_stall) begin
            rs1_data_E  <= rs1_data_E;
            rs2_data_E  <= rs2_data_E;
            rs1_E       <= rs1_E;
            rs2_E       <= rs2_E;
            rd_E        <= rd_E;
            imm_E       <= imm_E;
            pc_E        <= pc_E;
            pcplus4_E   <= pcplus4_E;

            rd_wren_E   <= rd_wren_E;
            opa_sel_E   <= opa_sel_E;
            opb_sel_E   <= opb_sel_E;
            alu_op_E    <= alu_op_E;
            mem_wren_E  <= mem_wren_E;
            wb_sel_E    <= wb_sel_E;
            is_load_E   <= is_load_E;
            is_jump_E   <= is_jump_E;
            is_branch_E <= is_branch_E;

            insn_vld_E  <= insn_vld_E; 
            ctrl_E      <= ctrl_E;    
            hit_E       <= hit_E;
            target_E    <= target_E;
        end
        else begin
            rs1_data_E   <= rs1_data;
            rs2_data_E   <= rs2_data;
            rs1_E        <= rs1;
            rs2_E        <= rs2;
            rd_E         <= rd;
            imm_E        <= imm;
            pc_E         <= pc_D;
            pcplus4_E    <= pcplus4_D;

            rd_wren_E    <= rd_wren;
            opa_sel_E    <= opa_sel;
            opb_sel_E    <= opb_sel;
            alu_op_E     <= alu_op;
            mem_wren_E   <= mem_wren;
            wb_sel_E     <= wb_sel;
            is_load_E    <= is_load;
            is_jump_E    <= is_jump;
            is_branch_E  <= is_branch;

            insn_vld_E   <= insn_vld_D;
            ctrl_E       <= ctrl;
            hit_E        <= hit_D;
            target_E     <= target_D;
        end
    end

endmodule

module ex_mem_reg(
    input logic i_clk,
    input logic i_reset,
    input logic i_flush,

    input logic [31:0] alu_data_E,
    input logic [31:0] wr_data_E,
    input logic [ 4:0] rd_E,
    input logic [31:0] pc_E,
    input logic [31:0] pcplus4_E,

    input logic is_load_E,
    input logic rd_wren_E,
    input logic mem_wren_E,
    input logic [1:0] wb_sel_E,
    input logic insn_vld_E,
    input logic ctrl_E,
    input logic take_branch,
    input logic mispred_E,
    input logic is_jump_E,
    
    output logic is_load_M,
    output logic [31:0] alu_data_M,
    output logic [31:0] wr_data_M,
    output logic [ 4:0] rd_M,
    output logic [31:0] pc_M,
    output logic [31:0] pcplus4_M,

    output logic rd_wren_M,
    output logic mem_wren_M,
    output logic [1:0] wb_sel_M,
    output logic insn_vld_M,
    output logic ctrl_M,
    output logic mispred_M,
    output logic is_jump_M

);

    always_ff @(posedge i_clk) begin
        if (!i_reset | i_flush) begin
            alu_data_M  <= 32'b0;
            wr_data_M   <= 32'b0;
            rd_M        <= 5'b0;
            pc_M        <= 32'b0;
            pcplus4_M   <= 32'b0;

            is_load_M   <= 1'b0;
            rd_wren_M   <= 1'b0;
            mem_wren_M  <= 1'b0;
            wb_sel_M    <= 2'b0;
            insn_vld_M  <= 1'b0;
            ctrl_M      <= 1'b0;
            mispred_M   <= 1'b0;
            is_jump_M   <= 1'b0;
        end 
        
        else begin
            alu_data_M  <= alu_data_E;
            wr_data_M   <= wr_data_E;
            rd_M        <= rd_E;
            pc_M        <= pc_E;
            pcplus4_M   <= pcplus4_E;

            is_load_M   <= is_load_E;
            rd_wren_M   <= rd_wren_E;
            mem_wren_M  <= mem_wren_E;
            wb_sel_M    <= wb_sel_E;
            insn_vld_M  <= insn_vld_E;
            ctrl_M      <= ctrl_E;
            mispred_M   <= mispred_E;
            is_jump_M   <= is_jump_E;
        end
    end

endmodule

module mem_wb_reg(
    input logic i_clk,
    input logic i_reset,

    input logic [31:0] alu_data_M,
    input logic [ 4:0] rd_M,
    input logic [31:0] pc_M,
    input logic [31:0] pcplus4_M,

    input logic is_load_M,
    input logic rd_wren_M,
    input logic [1:0] wb_sel_M,
    input logic insn_vld_M,
    input logic mispred_M,
    input logic ctrl_M,
    input logic [31:0] ld_data_M,

    output logic is_load_WB,
    output logic [31:0] alu_data_WB,
    output logic [ 4:0] rd_WB,
    output logic [31:0] o_pc_debug,
    output logic [31:0] pcplus4_WB,
    output logic [31:0] ld_data_WB,
 
    output logic rd_wren_WB,
    output logic [1:0] wb_sel_WB,
    output logic o_insn_vld,
    output logic o_mispred,
    output logic o_ctrl
);

    always_ff @(posedge i_clk) begin
       if (~i_reset) begin
            alu_data_WB  <= 32'b0;
            rd_WB        <= 5'b0;
            o_pc_debug   <= 32'b0;
            pcplus4_WB   <= 32'b0;

            rd_wren_WB   <= 1'b0;
            wb_sel_WB    <= 2'b0;
            o_insn_vld   <= 1'b0;
            o_mispred    <= 1'b0;
            o_ctrl       <= 1'b0;
            ld_data_WB   <= 32'b0;
            is_load_WB   <= 1'b0;
        end
        else begin
            alu_data_WB  <= alu_data_M;
            rd_WB        <= rd_M;
            o_pc_debug   <= pc_M;
            pcplus4_WB   <= pcplus4_M;
            ld_data_WB   <= ld_data_M;

            rd_wren_WB   <= rd_wren_M;
            wb_sel_WB    <= wb_sel_M;
            o_insn_vld   <= insn_vld_M;
            o_mispred    <= mispred_M;
            o_ctrl       <= ctrl_M;
            is_load_WB   <= is_load_M;

        end
    end

endmodule

module hazard_detection (
    input  logic i_lsu_wren,
    input  logic is_load_E,
    input  logic mispred_E,
    input  logic is_load_D,

    input logic [4:0] rs1_D,
    input logic [4:0] rs2_D,

    // From EX/MEM (M stage)
    input  logic [4:0] rd_E,
    input  logic       rd_wren_E,

    input logic [4:0] rd_WB,
    input logic       rd_wren_WB,

    input  logic       take_branch_E,  

    // To PC and pipeline regs
    output logic       stall_F,         
    output logic       stall_D,        
    output logic       stall_E,
    output logic       flush_D,         
    output logic       flush_E,          
    output logic       flush_M
);

    logic hazard_load, hazard_WB, hazard_loadstore, hazard_loadload ;

    assign hazard_load  = rd_wren_E && (rd_E != 5'd0) &&
                        ((rd_E == rs1_D) || (rd_E == rs2_D)) && is_load_E;

    assign hazard_WB = rd_wren_WB && (rd_WB != 5'd0) &&
                       ((rd_WB == rs1_D) || (rd_WB == rs2_D));
 
    assign hazard_loadstore = is_load_E & i_lsu_wren ;

    assign hazard_loadload = is_load_E & is_load_D ; 

    logic hazard ;
    assign hazard = hazard_load | hazard_WB | hazard_loadstore | hazard_loadload ; 

    
    always_comb begin
        stall_F = 1'b0;
        stall_D = 1'b0;
        stall_E = 1'b0;
        flush_D = 1'b0;
        flush_E = 1'b0;
        flush_M = 1'b0;

        if (mispred_E) begin
            stall_F = 1'b0;
            stall_D = 1'b0;
            flush_D = 1'b1;  
            flush_E = 1'b1;  
        end
        else if (hazard) 
        begin
            stall_F = 1'b1;  
            stall_D = 1'b1; 
            stall_E = 1'b0;
            flush_D = 1'b0;  
            flush_E = 1'b1;
            flush_M = 1'b0;  
        end
    end

endmodule

module forwarding_unit(
    input logic [4:0] rs1_E,
    input logic [4:0] rs2_E,
    input logic [4:0] rd_M,
    input logic [4:0] rd_WB,
    input logic rd_wren_M , 
    input logic rd_wren_WB,
    input logic is_load_M,
    input logic is_load_WB,

    output logic [1:0] fwrs1,
    output logic [1:0] fwrs2
);
    always_comb begin
        fwrs1 = 2'b00 ; 
        fwrs2 = 2'b00 ; 

         // RS1
        if (rd_wren_M && !is_load_M && rd_M != 0 && rd_M == rs1_E)
            fwrs1 = 2'b01;
        else if (rd_wren_WB && rd_WB != 0 && rd_WB == rs1_E)
            fwrs1 = 2'b10;

        // RS2
        if (rd_wren_M && !is_load_M && rd_M != 0 && rd_M == rs2_E)
            fwrs2 = 2'b01;
        else if (rd_wren_WB && rd_WB != 0 && rd_WB == rs2_E)
            fwrs2 = 2'b10;
    end

endmodule

module btb (
    input  logic        i_clk,
    input  logic        i_reset,

    // Lookup
    input  logic [31:0] i_pc,          // PC_F
    output logic        o_hit,
    output logic [31:0] o_target,
    output logic [7:0]  o_index,

    // Update
    input  logic        i_update_en,   // only when branch resolved
    input  logic [31:0] i_pc_update,   // PC_E
    input  logic [31:0] i_target_update
);

    assign o_index = i_pc[9:2];

    logic [7:0] update_idx;
    assign update_idx = i_pc_update[9:2];

  
    logic [31:0] tag_table    [0:255];
    logic [31:0] target_table [0:255];

    initial begin
        for (int i = 0; i < 256; i++) begin
            tag_table[i] = 32'b0;
            target_table[i]  = 32'b0;
        end
    end

   
    always_comb begin
        o_hit    = (tag_table[o_index] == i_pc);
        o_target = target_table[o_index];
    end

   
    always_ff @(posedge i_clk) begin
        if (!i_reset) begin
            for (int i = 0; i < 256; i++) begin
                tag_table[i]    <= 32'b0;
                target_table[i] <= 32'b0;
            end
        end 
        else if (i_update_en) begin
            tag_table[update_idx]    <= i_pc_update;
            target_table[update_idx] <= i_target_update;
        end
    end

endmodule

