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


//------------------------IF-------------------------------------   
   logic [15:0] 	 if_pc;// Program Counter
   logic [15:0] 	 if_ir;// Instruction Register
   
   always_ff @(posedge clk) begin
      if (rst)
        if_pc <= 16'hfe00;
      else
        if_pc <= if_pc + 16'h0002;      
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

   
   
//-----------------------pixel_fifo---------------------
   
   localparam integer BITWIDTH = 16;
   localparam integer FRAME_SIZE = 128;
   localparam integer KERNEL_SIZE = 3;                //3,5,7
   localparam integer BUFFER_SIZE = FRAME_SIZE*FRAME_SIZE;
   localparam integer ADDR_BITWIDTH = clog2(BUFFER_SIZE);
   localparam integer LINE_SIZE = FRAME_SIZE - KERNEL_WIDTH;
   
   
   (*ram_style = "block"*)
   reg [BUFFER_SIZE-1:0] pixel_buffer[BITWIDTH-1:0];
   
   reg 			 pibuf_re_addr[ADDR_BITWIDTH:0];
   reg 			 pibuf_we_addr[ADDR_BITWIDTH:0];
   
   
   wire [BITWIDTH-1:0] 	 pibuf_re_data;
   wire [BITWODTH-1:0] 	 buf_pixel;
   

   always_ff @(posedge clk) begin
      if(rst) 
	pibuf_re_addr <= 'd0;
      else if(pibuf_re_addr == BUFFER_SIZE-1)
	pibuf_re_addr <= 'd0;
      else
	pibuf_re_addr <= pibuf_re_addr + 'd1;
   end
   
   always_ff @(posedge clk) begin
      if(rst)
	pibuf_we_addr <= 'd1;
      else
	pibuf_we_addr <= pibuf_we_addr + 'd2;
   end

   always_ff @(posedge clk) begin
      pixel_buffer[pibuf_we_addr] <= if_ir[15:8];
      pixel_buffer[pibuf_we_addr+1]  <= if_ir[7:0];
   end
   
   always_ff @(posedge clk) begin
      if(rst)
	pibuf_re_data <= 'd0;
      else
	pibuf_re_data <= pixel_buffer[pibuf_re_addr];
   end

   
   assign buf_pixel = pibuf_re_data;

   //------------pixel_address------------------------------
   reg [BITWIDTH-1:0] pixel_addr;
   wire [BITWIDTH-1:0] in_pixel_addr;
   wire [BITWIDTH-1:0] out_pixel_addr;
   
   
   always_ff @(posedge clk) begin
      if(rst)
	pixel_addr <= 'd0;
      else begin
	 if(pixel_addr == 'd0)
	   pixel_addr <= 16'hff00;
	 else
	   pixel_addr <= pixel_addr + 1;
      end
   end

  
   localparam PIXEL_ADDR_BUFFRSIZE = (FRAME_SIZE*2) + KERNEL_SIZE + 1;
   
   delay
     #(
       .LATENCY(PIXEL_ADDR_BUFFERSIZE),
       .BIT_WIDTH(BITWIDTH)
       )
   delay_inst1
     (
      .clk(clk),
      .n_rst(~rst),
      .din(in_pixel_addr),
      .dout(out_pixel_addr)
      );
   
	   
      
   
   //------------stencil calucration--------------------
   integer i;
   
   reg[KERNEL_SIZE-1:0] stencil_buf1[BITWIDTH-1:0];
   reg[KERNEL_SIZE-1:0] stencil_buf2[BITWIDTH-1:0];
   reg[KERNEL_SIZE-1:0] stencil_buf3[BITWIDTH-1:0];

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
   

   assign in_line_data1 = stencil_buf1[BITWIDTH-1];
   
   
   delay
     #(
       .LATENCY(LINE_SIZE),
       .BIT_WIDTH(BITWIDTH)
       )
   delay_inst1
     (
      .clk(clk),
      .n_rst(~rst),
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

   assign in_line_data2 = stencil_buf2[BITWIDTH-1];
   

   delay
     #(
       .LATENCY(LINE_SIZE),
       .BIT_WIDTH(BITWIDTH)
       )
   delay_inst1
     (
      .clk(clk),
      .n_rst(~rst),
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


   

   reg[BITWIDTH-1:0] filter_pdata;
   wire 	     edge_flag;
   wire[BITWIDTH:0] 	     calc_pixel;

   assign calc_pixel = stencil_buf3[2]*(-1) + stencil_buf3[1]*(-1) + stencil_buf3[0]*(-1)+
		       stencil_buf2[2]*(-1) + stencil_buf2[1]*8    + stencil_buf2[0]*(-1)+
		       stencil_buf1[2]*(-1) + stencil_buf1[1]*(-1) + stencil_buf1[0]*(-1);
   

   
   always_ff @(posedge clk) begin
      if(rst)
	fileter_pdata <= 'd0;
      else begin
	 if(edge_flag)
	   fileter_pdata <= stencil_buf2[1];
	 else begin
	    if(calc_pixel>='d255)
	      filter_pdata <= 16'd255;
	    else if(calc_pixel<='d0)
	      filter_pdata <= 16'd0;
	    else
	      filter_pdata <= calc_pixel[15:0];
	 end
      end // else: !if(rst)   
   end // always_ff @ (posedge clk)
   

  
   //-------------write memory-------------------------
			    
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
	d_dout <= filter_pdata;   
   end
   
   
   always_ff @(posedge clk) begin
      if(rst)
	d_we <= 2'd0;
      else begin
	 if(out_pixel_addr[0]==0)
	   d_we <= 2'b10;
	 else
	   d_we <= 2'b01;
      end
   end
   
   
endmodule
`default_nettype wire
