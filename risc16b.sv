`default_nettype none

  module risc16b
    (
     input wire 	 clk,
     input wire 	 rst,
     output logic [15:0] i_addr,
     output logic 	 i_oe,
     input wire [15:0] 	 i_din,
     output logic [15:0] d_addr,
     output logic 	 d_oe,
     input wire [15:0] 	 d_din,
     output logic [15:0] d_dout,
     output logic [1:0]  d_we
     );


   integer		 i;
   

//------------------------IF-------------------------------------   
   logic [15:0] 	 if_pc;// Program Counter
   logic [15:0] 	 if_ir;// Instruction Register
   
   always_ff @(posedge clk) begin
      if (rst)
        if_pc <= 16'h8000;
      else begin
	 if(if_pc  != 16'hbffe)
           if_pc <= if_pc + 16'h0002;
	 else if(d_addr == 16'hffff) 
	   if_pc <= 16'h7fff;
	 else
	   if_pc <= if_pc;
      end 
   end

   always_ff @(posedge clk) begin
      if (rst)
        if_ir <= 16'd0;
      else
        if_ir <= i_din;
   end

   assign i_oe = 1'b1;
   assign i_addr = if_pc;
//-----------------------------------------------------

   
   
//-----------------------in_pixel_buffer---------------------
   //bufferサイズを一回りさせないために
   localparam integer BITWIDTH = 16;
   localparam integer FRAME_SIZE = 128;
   localparam integer KERNEL_SIZE = 3;                //3,5,7
   localparam integer FIFO_SIZE = FRAME_SIZE*FRAME_SIZE;
   localparam integer ADDR_BITWIDTH = $clog2(FIFO_SIZE);
   localparam integer LINE_SIZE = FRAME_SIZE - KERNEL_SIZE;
   
   
   (*ram_style = "block"*)
   reg [BITWIDTH-1:0] in_pixel_buffer[FIFO_SIZE:0];
   
   reg [ADDR_BITWIDTH:0] in_pibuf_raddr;
   reg [ADDR_BITWIDTH:0] in_pibuf_waddr;

   wire			 re;
   wire			 we;
   reg [15:0]		 count;

   always_ff @(posedge clk) begin
      if(rst)
	count <='d0;
      else
	count <= count+1;
   end
   
   assign we = (count>=16'h0001)? 1 : 0;
   assign re = (count>=16'h0001)? 1 : 0;
  
   
   
   reg [BITWIDTH-1:0] 	 in_pibuf_rdata;
   wire [BITWIDTH-1:0] 	 buf_pixel;
   

   always_ff @(posedge clk) begin
      if(rst) 
	in_pibuf_raddr <= 'd0;
      else begin
	 if(re) begin
	    if(in_pibuf_raddr == FIFO_SIZE)
	      in_pibuf_raddr <= 'd1;
	    else
	      in_pibuf_raddr <= in_pibuf_raddr + 'd1;
	 end
      end
   end // always_ff @ (posedge clk)
   
   
   always_ff @(posedge clk) begin
      if(rst)
	 in_pibuf_waddr <= 'd1;
      else begin
	 if(we) begin
	    if( in_pibuf_waddr+1 == FIFO_SIZE)
	      in_pibuf_waddr <= 'd1;
	    else
	      in_pibuf_waddr <= in_pibuf_waddr + 'd2;
	 end
      end
   end

   always_ff @(posedge clk) begin
      if(rst)
	in_pixel_buffer[0] <= 'd0;
      else begin
	 in_pixel_buffer[in_pibuf_waddr] <= {8'd0,if_ir[15:8]};
	 in_pixel_buffer[in_pibuf_waddr+1]  <= {8'd0,if_ir[7:0]};
      end
   end
   
   always_ff @(posedge clk) begin
      if(rst)
	in_pibuf_rdata <= 'd0;
      else
	in_pibuf_rdata <= in_pixel_buffer[in_pibuf_raddr];
   end

   
   assign buf_pixel = in_pibuf_rdata;

   //----------------------------------------------------------

   //------------pixel_address------------------------------
   
   reg [BITWIDTH-1:0] pixel_addr;
   wire [BITWIDTH-1:0] in_pixel_addr;
   wire [BITWIDTH-1:0] out_pixel_addr;
   
   wire		       addr_e;
       
   assign addr_e = (in_pibuf_raddr>=1) ? 1:0;
   
   
   
   always_ff @(posedge clk) begin
      if(rst)
	pixel_addr <= 'd0;
      else begin
	 if(addr_e) begin
	    if(pixel_addr == 'd0)
	      pixel_addr <= 16'hC000;
	    else
	      pixel_addr <= pixel_addr + 1;
	 end
      end
   end // always_ff @ (posedge clk)


   assign in_pixel_addr = (addr_e)? pixel_addr : 0;
   
  
   localparam integer PIXEL_ADDR_BUFFERSIZE = LINE_SIZE + KERNEL_SIZE+2 + 1;
   
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

   //-----stencil patch-------------------------

   logic[BITWIDTH-1:0] stencil_buf1[KERNEL_SIZE-1:0];
   logic[BITWIDTH-1:0] stencil_buf2[KERNEL_SIZE-1:0];
   logic[BITWIDTH-1:0] stencil_buf3[KERNEL_SIZE-1:0];

   
   stencil_patch#(
		 .KERNEL_SIZE(KERNEL_SIZE),
		 .LINE_SIZE(LINE_SIZE),
		 .BITWIDTH(BITWIDTH)
		 )
   stancil_inst(
		.clk(clk),
		.rst(rst),
		.din(buf_pixel),
		.stencil_buf1(stencil_buf1),
		.stencil_buf2(stencil_buf2),
		.stencil_buf3(stencil_buf3)
		);
   
      /*
   
   //------------stencil calucration--------------------

   reg[BITWIDTH-1:0] stencil_buf1[KERNEL_SIZE-1:0];
   reg[BITWIDTH-1:0] stencil_buf2[KERNEL_SIZE-1:0];
   reg[BITWIDTH-1:0] stencil_buf3[KERNEL_SIZE-1:0];


   wire [BITWIDTH-1:0] 	in_line_data1;
   wire [BITWIDTH-1:0] 	out_line_data1;
   wire [BITWIDTH-1:0] 	in_line_data2;
   wire [BITWIDTH-1:0] 	out_line_data2;
   
   
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
	 stencil_buf1[0] <= buf_pixel;
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

       */
   
   //-------------------------calc-----------------------------------
   
   
   reg[BITWIDTH-1:0] filter_pdata;
   reg		     edge_flag;
   wire signed [BITWIDTH-1:0] calc_pixel;

   

   always_comb begin
      if(out_pixel_addr<=16'hC07F)
	edge_flag = 1'b1;
      else if(((out_pixel_addr[7:0]+1)==8'h80) || ((out_pixel_addr[7:0]+'b1)=='h100))
	edge_flag = 1'b1;
      else if(((out_pixel_addr[7:0]+1)==8'h7F) || ((out_pixel_addr[7:0]+1)==8'hFF))
	edge_flag = 1'b1;
      else if(out_pixel_addr >= 16'hFF80)
	edge_flag = 1'b1;
      else
	edge_flag = 1'b0;
   end
      
   
   assign calc_pixel = stencil_buf3[2]*(-1) + stencil_buf3[1]*(-1) + stencil_buf3[0]*(-1)+
		       stencil_buf2[2]*(-1) + stencil_buf2[1]*8    + stencil_buf2[0]*(-1)+
		       stencil_buf1[2]*(-1) + stencil_buf1[1]*(-1) + stencil_buf1[0]*(-1);


   
   always_ff @(posedge clk) begin
      if(rst)
	filter_pdata <= 'd0;
      else begin
	 if(edge_flag)
	   filter_pdata <= stencil_buf2[1];
	 else begin
	    if(calc_pixel[BITWIDTH-1] == 1'b1)
	      filter_pdata <= 'd0;
	    else if(calc_pixel>='d255)
	      filter_pdata <= 'd255;
	    else
	      filter_pdata <= calc_pixel[BITWIDTH-1:0];
	 end
      end // else: !if(rst)   
   end // always_ff @ (posedge clk)


//------------------------------------------------------------------

  
   //-------------write memory-------------------------
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
      else begin
	 if(out_pixel_addr[0]==0)
	   d_dout <= {filter_pdata[7:0],8'h00};
	 else
	   d_dout <= filter_pdata;
      end   
   end


  // assign d_we = 2'b11;

//   assign d_we_flag = (out_pixel_addr != 'd0) ? 1:0;
   
   
   
   always_ff @(posedge clk) begin
      if(rst)
	d_we <= 2'd0;
      else begin
//	 if(d_we_flag) begin
	    if(out_pixel_addr[0]==0)
	      d_we <= 2'b01;
	    else
	     d_we <= 2'b10;
//	 end
      end
   end


   
endmodule // risc16b




module stencil_patch#(
		      parameter integer KERNEL_SIZE = -1,
		      parameter integer LINE_SIZE = -1,
		      parameter integer BITWIDTH = -1
		      )
   (
    input wire		      clk,
    input wire		      rst,
    input wire	[BITWIDTH-1:0]	      din,
    output reg [BITWIDTH-1:0] stencil_buf1[KERNEL_SIZE-1:0],
    output reg [BITWIDTH-1:0] stencil_buf2[KERNEL_SIZE-1:0],
    output reg [BITWIDTH-1:0] stencil_buf3[KERNEL_SIZE-1:0]
    );
   
   integer		      i;
   
   
   wire [BITWIDTH-1:0] 	in_line_data1;
   wire [BITWIDTH-1:0] 	out_line_data1;
   wire [BITWIDTH-1:0] 	in_line_data2;
   wire [BITWIDTH-1:0] 	out_line_data2;
   
   
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
