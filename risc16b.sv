`default_nettype none

module risc16b
  #(
    parameter integer BITWIDTH = 16,
    parameter integer FRAME_SIZE = 128
    )
   (
    input wire			clk,
    input wire			rst,
    output logic [BITWIDTH-1:0]	i_addr,
    output logic		i_oe,
    input wire [BITWIDTH-1:0]	i_din,
    output logic [BITWIDTH-1:0]	d_addr,
    output logic		d_oe,
    input wire [BITWIDTH-1:0]	d_din,
    output logic [BITWIDTH-1:0]	d_dout,
    output logic [1:0]		d_we
    );
   
   
   integer		i;
   
   
   //-------------------------IF-------------------------------------   
   logic [BITWIDTH-1:0]	if_pc;// Program Counter
   logic [BITWIDTH-1:0]	if_ir;// Instruction Register
   
   always_ff @(posedge clk) begin
      if (rst)
        if_pc <= 'h8000;
      else begin
	 if(if_pc  != 'hbffe)
           if_pc <= if_pc + 'h0002;
	 else if(d_addr == 'hfffe) 
	   if_pc <= 'h7fff;
	 else
	   if_pc <= if_pc;
      end 
   end
   
   always_ff @(posedge clk) begin
      if (rst)
        if_ir <= 'd0;
      else
        if_ir <= i_din;
   end
   
   assign i_oe = 1'b1;
   assign i_addr = if_pc;
   //-------------------------------------------------------------------

   //-------------------------stencil patch-----------------------------
   localparam integer	KERNEL_SIZE = 3;                //3,5,7
   localparam integer	LINE_SIZE = (FRAME_SIZE/2) - KERNEL_SIZE;
   
   logic [BITWIDTH-1:0] stencil_buf1[KERNEL_SIZE-1:0];
   logic [BITWIDTH-1:0]	stencil_buf2[KERNEL_SIZE-1:0];
   logic [BITWIDTH-1:0]	stencil_buf3[KERNEL_SIZE-1:0];
   
   
   stencil_patch#(
		  .KERNEL_SIZE(KERNEL_SIZE),
		  .LINE_SIZE(LINE_SIZE),
		  .BITWIDTH(BITWIDTH)
		  )
   stancil_inst(
		.clk(clk),
		.rst(rst),
		.din(if_ir),
		.stencil_buf1(stencil_buf1),
		.stencil_buf2(stencil_buf2),
		.stencil_buf3(stencil_buf3)
		);
   //-------------------------------------------------------------------
   
   
   //--------------------------pixel_address----------------------------
   localparam integer	PIXEL_ADDR_BUFFERSIZE = LINE_SIZE + KERNEL_SIZE+2 +3;
   
   reg [BITWIDTH-1:0]	pixel_addr;
   wire [BITWIDTH-1:0]	in_pixel_addr;
   wire [BITWIDTH-1:0]	out_pixel_addr;
   
   wire			addr_e;
   
   assign addr_e = (if_pc>='h8002) ? 1:0;
   
   
   
   always_ff @(posedge clk) begin
      if(rst)
	pixel_addr <= 'd0;
      else begin
	 if(addr_e) begin
	    if(pixel_addr == 'd0)
	      pixel_addr <= 16'hC000;
	    else
	      pixel_addr <= pixel_addr + 2;
	 end
      end
   end // always_ff @ (posedge clk)
   
   
   assign in_pixel_addr = (addr_e)? pixel_addr : 0;   
   
   
   
   delay
     #(
       .LATENCY(PIXEL_ADDR_BUFFERSIZE),
       .BIT_WIDTH(BITWIDTH)
       )
   delay_inst0
     (
      .clk(clk),
      .n_rst(rst),
      .din(in_pixel_addr),
      .dout(out_pixel_addr)
      );
   //------------------------------------------------------------------


   //-------------------------convolution 1----------------------------
   logic signed [BITWIDTH-1:0]	kernel_pixel_0[(KERNEL_SIZE*KERNEL_SIZE)-1:0];
   logic signed [BITWIDTH-1:0]	kernel_pixel_1[(KERNEL_SIZE*KERNEL_SIZE)-1:0];

   always_comb begin
      kernel_pixel_0[0] = stencil_buf3[2][7:0]  * (-1);
      kernel_pixel_0[1] = stencil_buf3[1][15:8] * (-1);
      kernel_pixel_0[2] = stencil_buf3[1][7:0]  * (-1);
      kernel_pixel_0[3] = stencil_buf2[2][7:0]  * (-1);
      kernel_pixel_0[4] = stencil_buf2[1][15:8] *   8 ;
      kernel_pixel_0[5] = stencil_buf2[1][7:0]  * (-1);
      kernel_pixel_0[6] = stencil_buf1[2][7:0]  * (-1);
      kernel_pixel_0[7] = stencil_buf1[1][15:8] * (-1);
      kernel_pixel_0[8] = stencil_buf1[1][7:0]  * (-1);
   end // always_comb

   always_comb begin
      kernel_pixel_1[0] = stencil_buf3[1][15:8] * (-1);
      kernel_pixel_1[1] = stencil_buf3[1][7:0]  * (-1);
      kernel_pixel_1[2] = stencil_buf3[0][15:8] * (-1);
      kernel_pixel_1[3] = stencil_buf2[1][15:8] * (-1);
      kernel_pixel_1[4] = stencil_buf2[1][7:0]  *   8 ;
      kernel_pixel_1[5] = stencil_buf2[0][15:8] * (-1);
      kernel_pixel_1[6] = stencil_buf1[1][15:8] * (-1);
      kernel_pixel_1[7] = stencil_buf1[1][7:0]  * (-1);
      kernel_pixel_1[8] = stencil_buf1[0][15:8] * (-1);
   end // always_comb
    
   
   //convolution reg
   reg signed [BITWIDTH-1:0]  conv_reg1_0[(KERNEL_SIZE*KERNEL_SIZE)-1:0]; 
   reg signed [BITWIDTH-1:0]  conv_reg1_1[(KERNEL_SIZE*KERNEL_SIZE)-1:0];

   
   always_ff @(posedge clk) begin
      if(rst) begin
	 for(i = 0;i < KERNEL_SIZE*KERNEL_SIZE;i = i+1)
	   conv_reg1_0[i] <= 'd0;
      end
      else begin
	 for(i = 0;i < KERNEL_SIZE*KERNEL_SIZE;i = i+1)
	   conv_reg1_0[i] <= kernel_pixel_0[i];
      end
   end

   
   always_ff @(posedge clk) begin
      if(rst) begin
	 for(i = 0;i < KERNEL_SIZE*KERNEL_SIZE;i = i+1)
	   conv_reg1_1[i] <= 'd0;
      end
      else begin
	 for(i = 0;i < KERNEL_SIZE*KERNEL_SIZE;i = i+1)
	   conv_reg1_1[i] <= kernel_pixel_1[i];
      end
   end
   
   //------------------------------------------------------------------

   
   //------------------------convolution 2-----------------------------
   reg signed [BITWIDTH -1:0] conv_reg2_0[KERNEL_SIZE-1:0];
   reg signed [BITWIDTH -1:0] conv_reg2_1[KERNEL_SIZE-1:0];
   
   always_ff @(posedge clk) begin
      if(rst) begin
	for(i = 0;i<KERNEL_SIZE;i = i+1)
	  conv_reg2_0[i] <= 'd0;
      end
      else begin
	 conv_reg2_0[0] <= conv_reg1_0[0] + conv_reg1_0[1] + conv_reg1_0[2];
	 conv_reg2_0[1] <= conv_reg1_0[3] + conv_reg1_0[4] + conv_reg1_0[5];
	 conv_reg2_0[2] <= conv_reg1_0[6] + conv_reg1_0[7] + conv_reg1_0[8];
      end
   end

   always_ff @(posedge clk) begin
      if(rst) begin
	for(i = 0;i<KERNEL_SIZE;i = i+1)
	  conv_reg2_1[i] <= 'd0;
      end
      else begin
	 conv_reg2_1[0] <= conv_reg1_1[0] + conv_reg1_1[1] + conv_reg1_1[2];
	 conv_reg2_1[1] <= conv_reg1_1[3] + conv_reg1_1[4] + conv_reg1_1[5];
	 conv_reg2_1[2] <= conv_reg1_1[6] + conv_reg1_1[7] + conv_reg1_1[8];
      end
   end // always_ff @ (posedge clk)
   
   //------------------------------------------------------------------
     
   //------------------------convlution 3------------------------------
   reg signed [BITWIDTH-1:0] conv_data_0;
   reg signed [BITWIDTH-1:0] conv_data_1;
   

   always_ff @(posedge clk) begin
      if(rst)
	conv_data_0 <= 'd0;
      else
	conv_data_0 <= conv_reg2_0[0] + conv_reg2_0[1] + conv_reg2_0[2];
   end

   always_ff @(posedge clk) begin
      if(rst)
	conv_data_1 <= 'd0;
      else
	conv_data_1 <= conv_reg2_1[0] + conv_reg2_1[1] + conv_reg2_1[2];
   end
   
   //-----------------------------------------------------------------
   
   
   //------------------shift reg (stencil_buf2[1])-------------------- 
   reg [BITWIDTH-1:0]	      pdata_shift_reg[2:0];
   wire [BITWIDTH-1:0]	      pixel_data;
   
   
   always_ff @(posedge clk) begin
      if(rst) begin
	 for(i=0;i<3;i=i+1)
	   pdata_shift_reg[i] <= 'd0;
      end
      else begin
	 for(i=2;i>0;i=i-1)
	   pdata_shift_reg[i] <= pdata_shift_reg[i-1];
	 pdata_shift_reg[0] <= stencil_buf2[1];
      end
   end // always_ff @ (posedge clk)
   //

   assign pixel_data = pdata_shift_reg[2];
   //------------------------------------------------------------------
   
   //-----------------------select filter data-------------------------
   reg[BITWIDTH-1:0] filter_data;
   logic[1:0]	     edge_flag;
   
   
   always_ff @(posedge clk) begin
      if(rst)
	edge_flag <= 2'b00;
      else begin
	 //addr_flag
	 if((out_pixel_addr+4)<=16'hC07F)
	   edge_flag[0] <= 1'b1;
	 else if(((out_pixel_addr[7:0]+4)==8'h80) || ((out_pixel_addr[7:0]+4)=='h100))
	   edge_flag[0] <= 1'b1;
	 else if(((out_pixel_addr[7:0]+4)==8'h7F) || ((out_pixel_addr[7:0]+4)==8'hFF))
	   edge_flag[0] <= 1'b1;
	 else if(out_pixel_addr+4 >= 16'hFF80)
	   edge_flag[0] <= 1'b1;
	 else
	   edge_flag[0] <= 1'b0;
	 
	 //addr+1_frag
	 if(out_pixel_addr+5<=16'hC07F)
	   edge_flag[1] <= 1'b1;
	 else if(((out_pixel_addr[7:0]+5)==8'h80) || ((out_pixel_addr[7:0]+5)=='h100))
	   edge_flag[1] <= 1'b1;
	 else if(((out_pixel_addr[7:0]+5)==8'h7F) || ((out_pixel_addr[7:0]+5)==8'hFF))
	   edge_flag[1] <= 1'b1;
	 else if(out_pixel_addr+5 >= 16'hFF80)
	   edge_flag[1] <= 1'b1;
	 else
	   edge_flag[1] <= 1'b0;
      end
   end

   
   always_ff @(posedge clk) begin
      if(rst)
	filter_data <= 'd0;
      else begin
	 if(edge_flag == 2'b11)
	   filter_data <= pixel_data;
	 
	 else if(edge_flag == 2'b01) begin
	    filter_data[15:8] <= pixel_data[15:8];
	    select_data(0);
	 end
	 
	 else if(edge_flag == 2'b10) begin
	    filter_data[7:0] <= pixel_data[7:0];
	    select_data(1);
	 end
	 
	 else begin
	    select_data(0);
	    select_data(1);
	 end
      end // else: !if(rst)   
   end // always_ff @ (posedge clk)

   

   task select_data(input f);
      if(!f) begin
	 if(conv_data_1[BITWIDTH-1] == 1'b1)
	   filter_data[7:0] <= 8'd0;
	 else if(conv_data_1>='d255)
	   filter_data[7:0] <= 8'd255;
	 else
	   filter_data[7:0] <= conv_data_1[7:0];
      end

      else begin
	 if(conv_data_0[BITWIDTH-1] == 1'b1)
	   filter_data[15:8] <= 8'd0;
	 else if(conv_data_0>='d255)
	   filter_data[15:8] <= 8'd255;
	 else
	   filter_data[15:8] <= conv_data_0[7:0];
      end
   endtask

   //------------------------------------------------------------------

  
   //-------------------------write memory-----------------------------
   wire d_we_flag;
   
			    
   assign d_oe = 1'b0;

   always_ff @(posedge clk) begin
      if(rst)
	d_addr <= 'd0;
      else
	d_addr <= out_pixel_addr;
   end
   
   
   always_ff @(posedge clk) begin
      if(rst)
	d_dout <= 'd0;
      else
	d_dout <= filter_data;
   end   

      
   assign d_we = 2'b11;
   
   //------------------------------------------------------------------
   
endmodule // risc16b




module stencil_patch
  #(
    parameter integer KERNEL_SIZE = -1,
    parameter integer LINE_SIZE = -1,
    parameter integer BITWIDTH = -1
    )
   (
    input wire		      clk,
    input wire		      rst,
    input wire [BITWIDTH-1:0] din,
    output reg [BITWIDTH-1:0] stencil_buf1[KERNEL_SIZE-1:0],
    output reg [BITWIDTH-1:0] stencil_buf2[KERNEL_SIZE-1:0],
    output reg [BITWIDTH-1:0] stencil_buf3[KERNEL_SIZE-1:0]
    );
   
   integer		      i;
   
   
   wire [BITWIDTH-1:0]	      in_line_data1;
   wire [BITWIDTH-1:0]	      out_line_data1;
   wire [BITWIDTH-1:0]	      in_line_data2;
   wire [BITWIDTH-1:0]	      out_line_data2;
   
   
   always_ff @(posedge clk) begin
      if(rst) begin
	 for(i = 0;i<KERNEL_SIZE;i = i+1)begin
	    stencil_buf1[i] <= 'd0;
	 end
      end  
      else begin
	 for(i=KERNEL_SIZE-1;i>0;i = i-1) begin
	    stencil_buf1[i] <= stencil_buf1[i-1];
	 end
	 stencil_buf1[0] <= din;
      end // else: !if(rst)
   end // always_ff @ (posedge clk)
   
   
   assign in_line_data1 = stencil_buf1[KERNEL_SIZE-1];
   
   
   delay
     #(
       .LATENCY(LINE_SIZE),
       .BIT_WIDTH(BITWIDTH)
       )
   delay_inst1
     (
      .clk(clk),
      .n_rst(rst),
      .din(in_line_data1),
      .dout(out_line_data1)
      );

   
   always_ff @(posedge clk) begin
      if(rst) begin
	 for(i = 0;i<KERNEL_SIZE;i = i+1)begin
	    stencil_buf2[i] <= 'd0;	    
	 end
      end
      else begin
	 for(i=KERNEL_SIZE-1;i>0;i = i-1) begin
	    stencil_buf2[i] <= stencil_buf2[i-1];    
	 end
	 stencil_buf2[0] <= out_line_data1;
	 
      end // else: !if(rst)
   end // always_ff @ (posedge clk)

   assign in_line_data2 = stencil_buf2[KERNEL_SIZE-1];
   

   delay#(
	  .LATENCY(LINE_SIZE),
	  .BIT_WIDTH(BITWIDTH)
	  )
   delay_inst2
     (
      .clk(clk),
      .n_rst(rst),
      .din(in_line_data2),
      .dout(out_line_data2)
      );

   always_ff @(posedge clk) begin
      if(rst) begin
	 for(i = 0;i<KERNEL_SIZE;i = i+1)begin
	    stencil_buf3[i] <= 'd0;
	 end
      end
      else begin
	 for(i=KERNEL_SIZE-1;i>0;i = i-1) begin
	    stencil_buf3[i] <= stencil_buf3[i-1];
	 end
	 stencil_buf3[0] <= out_line_data2;
      end // else: !if(rst)
   end // always_ff @ (posedge clk)

//------------------------------------------------------------




endmodule





module delay(clk, n_rst, din, dout);

    parameter LATENCY = 9;
    parameter BIT_WIDTH = 4;

    input wire clk, n_rst;
    input wire  [BIT_WIDTH-1:0] din;
    output wire [BIT_WIDTH-1:0] dout;

   wire				we;
   wire				re;
   reg[$clog2(LATENCY)-1:0]				waddr;
   reg[$clog2(LATENCY)-1:0]				raddr;
   

   dpram #(
	   .FIFO_SIZE(LATENCY),
	   .BIT_WIDTH(BIT_WIDTH)
	   )
   dpram_inst(
	      .clk(clk),
	      .n_rst(n_rst),
	      .we(we),
	      .re(re),
	      .raddr(raddr),
	      .waddr(waddr),
	      .din(din),
	      .dout(dout)
	      );


   assign re = 1;
   assign we = 1;
   
   always @(posedge clk) begin
      if(n_rst)
	waddr <= 'd0;
      else begin
	 if(waddr == LATENCY-1)
	   waddr <= 'd0;
	 else
	   waddr <= waddr + 1;
	 end
   end
	   
   

   always @(posedge clk) begin
      if(n_rst) 
	raddr <= 'd1;
      else begin
	 if(raddr == LATENCY-1) 
	   raddr = 'd0;
	 else
	   raddr <= raddr + 1'b1;
      end   
   end
   



endmodule



module dpram (clk, n_rst, we, re, raddr, waddr, din, dout);
    parameter FIFO_SIZE = 1024;
    parameter BIT_WIDTH = 8;

    input wire                         clk, n_rst, we, re;
    input wire [$clog2(FIFO_SIZE)-1:0] raddr, waddr;
    input wire [BIT_WIDTH-1:0]         din;
    output reg [BIT_WIDTH-1:0]         dout;

    (* ram_style = "block" *) 
    reg [BIT_WIDTH-1:0] mem [FIFO_SIZE-1:0];

    always @(posedge clk) begin
        if (n_rst)  dout <= 1'd0;
        else if (re) dout <= mem[raddr];
    end

    always @(posedge clk) begin
        if (we) mem[waddr] <= din;
    end

   integer i;   
   initial begin
        for(i = 0; i < FIFO_SIZE; i = i+1) mem[i] = 'b0;
    end
endmodule



`default_nettype wire
