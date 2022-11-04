`default_nettype none

module risc16b_top 
  (
   // clock and reset
   input wire 	       CLOCK_50_B5B,
   input wire 	       CPU_RESET_n,

   // Push buttons
   input wire [3:0]    KEY, // active-low

   // Slide switches   
   input wire [7:0]    SW, 
     
   // LEDs
   output wire [9:0]   LEDR, // active-high
   output wire [7:0]   LEDG, // active-high
  
   // 7-segment LEDs
   output wire [6:0]   HEX0_D, // active-low
   output wire [6:0]   HEX1_D, // active-low
   output wire [6:0]   HEX2_D, // active-low
   output wire [6:0]   HEX3_D, // active-low

   // SRAM
   output logic [17:0] SRAM_A,
   output logic        SRAM_CE_n,
   inout wire [15:0]   SRAM_D,
   output logic        SRAM_LB_n,
   output logic        SRAM_OE_n,
   output logic        SRAM_UB_n,
   output logic        SRAM_WE_n,

   // UART
   output logic        UART_TX,
   input wire 	       UART_RX   
   );		     

   enum int unsigned {D0, D1, D2, D3, I0, U0, U1, U2} main_state = D0;
   logic 	     processor_clk = 1'b0, processor_rst = 1'b1;
   logic [15:0]      d_din, d_dout, d_addr, i_din, i_addr;
   logic 	     d_oe, i_oe;
   logic [1:0] 	     d_we;
   logic [15:0]      data_from_uart, data_to_uart, addr_from_uart;
   logic [15:0]      ctrl_from_uart, keys_from_uart;
   logic 	     we_from_uart;
   logic 	     in_uart_write_mode = 1'b0;

   // Dummy signals for suprressing warnings
   logic 	      zero;
   assign zero = CPU_RESET_n? 1'b0: 1'bz;

   // System reset input
   logic 	      CPU_RESET_n_sync, CPU_RESET_n_tmp;
   double_ff
     #(
       .WIDTH(1),
       .INIT_VAL(0)
       )
   double_ff_n_rst_sys
     (
      .clk(CLOCK_50_B5B),
      .din(CPU_RESET_n),
      .dout(CPU_RESET_n_tmp)
      );

   logic [63:0]       reset_timers = '0;
   always @(posedge CLOCK_50_B5B) begin
      if (!CPU_RESET_n_tmp)
	reset_timers <= '0;
      else
	reset_timers <= {reset_timers[62:0], 1'b1};
   end
   assign CPU_RESET_n_sync = reset_timers[63];
   
   // Processor lock
   wire [1:0]	     keys_sync;
   double_ff
     #(
       .WIDTH(2),
       .INIT_VAL(0)
       )
   double_ff_key
     (
      .clk(CLOCK_50_B5B),
      .din(~KEY[1:0] | keys_from_uart[1:0]),
      .dout(keys_sync)
      );

   logic [1:0] 	     keys_sync_prev = '0;
   always @(posedge CLOCK_50_B5B) begin
      keys_sync_prev <= keys_sync;
   end

   logic [1:0] keys_push;
   assign keys_push = keys_sync & ~keys_sync_prev;

   enum int unsigned {INIT, RESET, LOCK, FREE, STEP} 
	processor_lock_state = INIT;
   always @(posedge CLOCK_50_B5B) begin
      if (!CPU_RESET_n_sync)
	processor_lock_state <= INIT;
      else begin
	 if (processor_lock_state == INIT && main_state == U2)
	   processor_lock_state <= RESET;
	 else if (processor_lock_state == RESET && main_state == U2)
	   processor_lock_state <= LOCK;
	 else if (processor_lock_state == LOCK) begin
	    if (keys_push[1])
	      processor_lock_state <= FREE;
	    else if (keys_push[0])
	      processor_lock_state <= STEP;
	 end
	 else if (processor_lock_state == STEP && main_state == U2)
	   processor_lock_state <= LOCK;
      end
   end
   
   // state control
   always @(posedge CLOCK_50_B5B) begin
      if (!CPU_RESET_n_sync)
	main_state <= D0;
      else begin
	 case (main_state)
	   D0: main_state <= D1;
	   D1: main_state <= D2;
	   D2: main_state <= D3;
	   D3: main_state <= I0;
	   I0: main_state <= U0;
	   U0: main_state <= U1;
	   U1: main_state <= U2;
	   U2: main_state <= D0;
	 endcase
      end
   end

   
   always @(posedge CLOCK_50_B5B) begin
      if (!CPU_RESET_n_sync)
	processor_clk <= 1'b0;
      else begin
	 if (main_state == U2) begin
	    if (processor_lock_state != RESET && processor_lock_state != LOCK)
	      processor_clk <= 1'b1;
	    else
	      processor_clk <= 1'b0;
	 end
	 else if (main_state == D3)
	   processor_clk <= 1'b0;
      end
   end

   // avoid processor_rst change at rising edege of processor_clk
   always @(posedge CLOCK_50_B5B) begin
      if (!CPU_RESET_n_sync)
	processor_rst <= 1'b1;
      else begin
	 if (processor_lock_state == INIT)
	   processor_rst <= 1'b1;
	 else if (processor_lock_state == RESET && main_state == D3)
	   processor_rst <= 1'b0;
      end
   end

   // memory if
   always_comb begin
      if (main_state == D0 || main_state == D1 || main_state == D2 || 
	  main_state == D3) 
	SRAM_A = {{3{zero}}, d_addr[15:1]};
      else if (main_state == I0)
	SRAM_A = {{3{zero}}, i_addr[15:1]};
      else if (main_state == U0 || main_state == U1 || main_state == U2)
	SRAM_A = {{3{zero}}, addr_from_uart[15:1]};
      else
	SRAM_A = {{3{zero}}, 15'h0};
   end

   always_comb begin
      if (main_state == D0 || main_state == D1 || main_state == D2 || 
	  main_state == D3) begin
	 SRAM_OE_n = !d_oe;
	 SRAM_LB_n = !(d_oe | d_we[1]);
	 SRAM_UB_n = !(d_oe | d_we[0]);
      end
      else if (main_state == I0) begin
	 SRAM_OE_n = !i_oe;
	 SRAM_LB_n = !i_oe;
	 SRAM_UB_n = !i_oe;
      end
      else if (main_state == U0 || main_state == U1 || main_state == U2) begin
	 SRAM_OE_n = in_uart_write_mode;
	 SRAM_LB_n = 1'b0;
	 SRAM_UB_n = 1'b0;	 
      end
      else begin
	 SRAM_OE_n = 1'b1;
	 SRAM_LB_n = 1'b1;
	 SRAM_UB_n = 1'b1;
      end
   end

   always @(posedge CLOCK_50_B5B) begin
      if (!CPU_RESET_n_sync)
	SRAM_WE_n <= 1'b1;
      else if (!processor_rst && main_state == D1 && d_we != 2'b00)
	SRAM_WE_n <= 1'b0;
      else if (main_state == U0 && in_uart_write_mode)
	SRAM_WE_n <= 1'b0;
      else
	SRAM_WE_n <= 1'b1;
   end

   assign SRAM_D 
     = ((main_state == D0 || main_state == D1 || 
	 main_state == D2 || main_state == D3) && d_we != 2'b00)? d_dout: 
       ((main_state == U0 || main_state == U1 || 
	 main_state == U2) && in_uart_write_mode)? data_from_uart: 16'hzzzz;

   assign SRAM_CE_n = !CPU_RESET_n_sync;

   reg [15:0] idreg = '0, ddreg = '0;
   
   always @(posedge CLOCK_50_B5B) begin
      if (!CPU_RESET_n_sync)
	idreg <= '0;
      else if (main_state == I0)
	idreg <= SRAM_D;
   end

   assign i_din = idreg;

   always @(posedge CLOCK_50_B5B) begin
      if (!CPU_RESET_n_sync)
	ddreg <= '0;
      else if (main_state == D3)
	ddreg <= SRAM_D;
   end

   assign d_din = ddreg;

   // Processor instance
   risc16b risc16b_inst
     (
      .clk(processor_clk),
      .rst(processor_rst),
      .d_din(d_din),
      .d_dout(d_dout),
      .d_addr(d_addr),
      .d_oe(d_oe),
      .d_we(d_we),      
      .i_din(i_din),
      .i_addr(i_addr),
      .i_oe(i_oe)
      );
   
   always @(posedge CLOCK_50_B5B) begin
      if (!CPU_RESET_n_sync)
	data_to_uart <= '0;
      else if (main_state == U1)
	data_to_uart <= SRAM_D;
   end

   always @(posedge CLOCK_50_B5B) begin
      if (!CPU_RESET_n_sync)
	in_uart_write_mode <= 1'b0;
      else if (main_state == I0 && we_from_uart)
	in_uart_write_mode <= 1'b1;
      else if (main_state == U2 && in_uart_write_mode)
	in_uart_write_mode <= 1'b0;
   end

   // LED
   reg [15:0] led_reg = '0;
   always @(posedge CLOCK_50_B5B) begin
      if (!CPU_RESET_n_sync)
	led_reg <= '0;
      else if (d_addr == 16'h7f00) begin
	 if (d_we[0])
	   led_reg[15:8] <= d_dout[15:8];
	 if (d_we[1])
	   led_reg[7:0] <= d_dout[7:0];
      end
      //else if (SW[0] == 1'b1)
      //led_reg <= i_addr;
   end

   decode_7seg decode_7seg_0(.dout(HEX0_D), .din(led_reg[3:0]));
   decode_7seg decode_7seg_1(.dout(HEX1_D), .din(led_reg[7:4]));
   decode_7seg decode_7seg_2(.dout(HEX2_D), .din(led_reg[11:8]));
   decode_7seg decode_7seg_3(.dout(HEX3_D), .din(led_reg[15:12]));

   assign LEDR = {{5{zero}},
		  processor_lock_state == STEP,
		  processor_lock_state == FREE,
		  processor_lock_state == LOCK,
		  processor_lock_state == RESET,
		  processor_lock_state == INIT};
   assign LEDG = !CPU_RESET_n_sync? 8'hzz: i_addr[7:0];
   
   // UART
   localparam int CLOCK_FREQ = 50 * 10**6; // 50Hz
   localparam int RS232C_RATE = 115200;
   localparam int WORD_WIDTH = 16;
   localparam int REGFILE_NUM_ENTRIES = 8;
   
   logic [WORD_WIDTH*REGFILE_NUM_ENTRIES-1:0] rdata, wdata;
   logic 				      UART_RX_sync;
   
   double_ff
     #(
       .WIDTH(1),
       .INIT_VAL(1)
       )
   double_ff_uart_rx
     (
      .clk(CLOCK_50_B5B),
      .din(UART_RX),
      .dout(UART_RX_sync)
      );
   
   uart_reg_file
     #(
       .DATA_WIDTH(WORD_WIDTH),
       .NUM_ENTRIES(REGFILE_NUM_ENTRIES),
       .FREQ(CLOCK_FREQ),
       .RS232C_RATE(RS232C_RATE)
      )
   uart_reg_file_0
     (
      .clk(CLOCK_50_B5B),
      .n_rst(CPU_RESET_n_sync),
      .uart_rx(UART_RX_sync),
      .uart_tx(UART_TX),
      //
      .rdata(rdata),
      .wdata(wdata),
      //
      .init_addr(),
      .init_data(),
      .init_en(1'b0),
      .init_we(1'b1)
      );

   always_comb begin
      set_rdata(0, addr_from_uart);
      set_rdata(1, data_from_uart);
      set_rdata(2, data_to_uart);
      set_rdata(3, ctrl_from_uart);
      set_rdata(4, keys_from_uart);
      set_rdata(5, i_addr);
      set_rdata(6, led_reg);
      set_rdata(7, {4'h0, KEY, SW});
   end   

   always_comb begin
      addr_from_uart = get_wdata(0);
      data_from_uart = get_wdata(1);
      ctrl_from_uart = get_wdata(3);
      keys_from_uart = get_wdata(4);
   end

   assign we_from_uart = ctrl_from_uart[0];
   
   function void set_rdata(int i, logic [WORD_WIDTH-1:0] d);
      rdata[i * WORD_WIDTH +: WORD_WIDTH] = d;
   endfunction
   
   function logic[WORD_WIDTH-1:0] get_wdata(int i);
      return (wdata[i * WORD_WIDTH +: WORD_WIDTH]);
   endfunction // get_wdata


endmodule 


module double_ff
  #(
    parameter integer WIDTH = 32,
    parameter [WIDTH-1:0] INIT_VAL = 0
    )
   (
    input wire 		    clk,
    input wire [WIDTH-1:0]  din,
    output wire [WIDTH-1:0] dout
    );

   reg [WIDTH-1:0] 	    tmp_reg = INIT_VAL;
   reg [WIDTH-1:0] 	    sync_reg = INIT_VAL;
   
   always @(posedge clk) begin
      tmp_reg <= din;
      sync_reg <= tmp_reg;
   end
   assign dout = sync_reg;
endmodule 


module decode_7seg 
  (
   input wire [3:0]   din,
   output logic [6:0] dout
   );

   // for active-low LEDs
   always_comb begin
      case (din)
        4'h0: dout = 7'b1000000;
        4'h1: dout = 7'b1111001;
        4'h2: dout = 7'b0100100;
        4'h3: dout = 7'b0110000;
        4'h4: dout = 7'b0011001;
        4'h5: dout = 7'b0010010;
	4'h6: dout = 7'b0000010;
        4'h7: dout = 7'b1111000;
        4'h8: dout = 7'b0000000;
        4'h9: dout = 7'b0010000;
	4'ha: dout = 7'b0001000;
	4'hb: dout = 7'b0000011;
	4'hc: dout = 7'b1000110;
	4'hd: dout = 7'b0100001;
	4'he: dout = 7'b0000110;
	4'hf: dout = 7'b0001110;
        default: dout = 7'b1111111;
      endcase
   end
endmodule


/*
 * Originally designed by <dohi@pca.cis.nagasaki-u.ac.jp>
 */
module rs232c_endpoint
  #(
    parameter real FREQ = 25 * 10**6,
    parameter real RS232C_RATE = 9600,
    parameter bit  RS232C_PARITY_EN = 1'b0,
    parameter bit  RS232C_PARITY_EVEN = 1'b1,
    parameter int  EP_N_ENTRIES = 16,
    parameter int  EP_BITWIDTH = 64
    )
    (
     input wire                            clk,
     input wire                            n_rst,
    
     ////////////////////////////////////////////
     input wire                            rs232c_rxd_in,
     output wire                           rs232c_txd_out,
    
     ////////////////////////////////////////////
     output reg                            ep_w_en = 1'b0,
     output reg [$clog2(EP_N_ENTRIES)-1:0] ep_w_addr = '0,
     output reg [EP_BITWIDTH-1:0]          ep_w_data = '0,
    
     ////////////////////////////////////////////
     output reg [$clog2(EP_N_ENTRIES)-1:0] ep_r_addr = '0,
     input wire [EP_BITWIDTH-1:0]          ep_r_data,
     output reg                            ep_r_request = 1'b0,
     input wire                            ep_r_ack
     );

   ////////////////////////////////////////////
   localparam int ST_IDLE = 0;
   localparam int ST_ADDR_FETCH = 1;
   localparam int ST_READ_WAIT = 2;
   localparam int ST_READ = 3;
   localparam int ST_WRITE = 4;
   localparam int ST_BREAD_WAIT = 5; // for burst read
   localparam int ST_BREAD = 6;
   

   reg [2:0] state = ST_IDLE;

   ////////////////////////////////////////////
   localparam int CMD_READ  = 8'h00;
   localparam int CMD_WRITE = 8'h01;
   localparam int CMD_BREAD = 8'h02; // burst read

   reg [7:0] cmd;
   
   ////////////////////////////////////////////
   localparam int EP_ADDR_BITWIDTH = $clog2(EP_N_ENTRIES);
   localparam int EP_ADDR_BYTE_LENGTH = (EP_ADDR_BITWIDTH + 7) / 8;
   
   reg  [EP_ADDR_BITWIDTH-1:0] addr_buf;
   wire [EP_ADDR_BITWIDTH-1:0] addr_from_rx;
   reg  [EP_ADDR_BITWIDTH-1:0] burst_addr_reg;
   
   ////////////////////////////////////////////
   // byte length of each endpoint data
   localparam int EP_DATA_BYTE_LENGTH = (EP_BITWIDTH + 7) / 8;
   localparam int RS232C_BUF_BITWIDTH = EP_DATA_BYTE_LENGTH * 8;
   
   wire [RS232C_BUF_BITWIDTH-1:0] word_from_rx;
   wire [7:0]                     rx_data;
   wire                           rx_data_en;
   
   ////////////////////////////////////////////

   localparam int MAX_BYTE_LENGTH = (EP_DATA_BYTE_LENGTH > EP_ADDR_BYTE_LENGTH)?
                  EP_DATA_BYTE_LENGTH: EP_ADDR_BYTE_LENGTH;
   localparam int CNT_BITWIDTH = $clog2(MAX_BYTE_LENGTH);

   reg [CNT_BITWIDTH-1:0] data_cnt;
   
   wire                   last_data_cnt;
   assign last_data_cnt 
     = (data_cnt == EP_DATA_BYTE_LENGTH-1)? 1'b1: 1'b0;

   wire                   last_addr_cnt;   
   assign last_addr_cnt 
     = (data_cnt == EP_ADDR_BYTE_LENGTH-1)? 1'b1: 1'b0;

   ////////////////////////////////////////////
   wire [7:0]             tx_data;
   reg                    tx_data_en;
   wire                   tx_data_done;

   ////////////////////////////////////////////

   always @(posedge clk) begin
      if (!n_rst) 
        state <= ST_IDLE;
      else if ((state == ST_IDLE) && rx_data_en)
        state <= ST_ADDR_FETCH;
      else if ((state == ST_ADDR_FETCH) && rx_data_en) begin
         if ((cmd == CMD_READ) && last_addr_cnt)
           state <= ST_READ_WAIT;
         else if ((cmd == CMD_WRITE) && last_addr_cnt)
           state <= ST_WRITE;
         else if ((cmd == CMD_BREAD) && last_addr_cnt)
           state <= ST_BREAD_WAIT;
      end
      else if ((state == ST_WRITE) && last_data_cnt && rx_data_en)
        state <= ST_IDLE;
      else if ((state == ST_READ_WAIT) && ep_r_ack)
        state <= ST_READ;
      else if ((state == ST_READ) && last_data_cnt && tx_data_done)
        state <= ST_IDLE;
      else if ((state == ST_BREAD_WAIT) && ep_r_ack)
        state <= ST_BREAD;
      else if ((state == ST_BREAD) && last_data_cnt && tx_data_done) begin
         if (ep_r_addr == burst_addr_reg)
           state <= ST_IDLE;
         else
           state <= ST_BREAD_WAIT;
      end
   end

   always @(posedge clk) begin
      if ((state == ST_IDLE) && rx_data_en)
        cmd <= rx_data;
   end
   
   always @(posedge clk) begin
      if (!n_rst)
        data_cnt <= '0;
      else if ((state == ST_IDLE) && rx_data_en)
        data_cnt <= '0;
      else if ((state == ST_ADDR_FETCH) && last_addr_cnt && rx_data_en) 
        data_cnt <= '0;
      else if ((state == ST_WRITE) && last_data_cnt && rx_data_en) 
        data_cnt <= '0;
      else if ((state == ST_READ) && last_data_cnt && tx_data_done)
        data_cnt <= '0;
      else if ((state == ST_BREAD) && last_data_cnt && tx_data_done)
        data_cnt <= '0;
      else if ((state == ST_ADDR_FETCH) && rx_data_en)
        data_cnt <= data_cnt + 1'b1;
      else if ((state == ST_WRITE) && rx_data_en)
        data_cnt <= data_cnt + 1'b1;
      else if ((state == ST_READ) && tx_data_done)
        data_cnt <= data_cnt + 1'b1;
      else if ((state == ST_BREAD) && tx_data_done)
        data_cnt <= data_cnt + 1'b1;      
   end
   
   ////////////////////////////////////////////

   rs232c_rx 
     #(
       .FREQ(FREQ),
       .RATE(RS232C_RATE),
       .PARITY_EN(RS232C_PARITY_EN),
       .PARITY_EVEN(RS232C_PARITY_EVEN)
       )
       rs232c_rx_0
         (
          .clk(clk),
          .n_rst(n_rst),
          .rxd(rs232c_rxd_in),
          .data(rx_data),
          .data_en(rx_data_en)
          );
   
   ////////////////////////////////////////////
   
   generate
      if (EP_ADDR_BYTE_LENGTH == 1)
         assign addr_from_rx = rx_data[EP_ADDR_BITWIDTH-1:0];
      else if (EP_ADDR_BYTE_LENGTH > 1) 
        assign addr_from_rx = {rx_data, addr_buf[EP_ADDR_BITWIDTH-1:8]};
   endgenerate

   always @(posedge clk) begin
      if ((state == ST_ADDR_FETCH) && rx_data_en)
        addr_buf <= addr_from_rx;
   end

   ////////////////////////////////////////////
   
   generate
      if (EP_DATA_BYTE_LENGTH == 1)
        assign word_from_rx = rx_data;
      else if (EP_DATA_BYTE_LENGTH > 1) begin
         reg [RS232C_BUF_BITWIDTH-1-8:0] word_buf;
         
         assign word_from_rx = {rx_data, word_buf};

         always @(posedge clk) begin
            if (rx_data_en)
              word_buf <= word_from_rx[RS232C_BUF_BITWIDTH-1:8];
         end
      end
   endgenerate

           
   ////////////////////////////////////////////

   always @(posedge clk) begin
      if (!n_rst) 
        ep_w_en <= 1'b0;
      else if (ep_w_en == 1'b1)
        ep_w_en <= 1'b0;
      else if ((state == ST_WRITE) && last_data_cnt && rx_data_en)
        ep_w_en <= 1'b1;
   end 

   always @(posedge clk) begin
      if ((state == ST_WRITE) && last_data_cnt && rx_data_en) begin
         ep_w_addr <= addr_buf;
         ep_w_data <= word_from_rx;
      end
   end
   
   ////////////////////////////////////////////

   generate
      if (EP_DATA_BYTE_LENGTH == 1)
        assign tx_data = ep_r_data;
      else if (EP_DATA_BYTE_LENGTH > 1) begin
         reg [RS232C_BUF_BITWIDTH-1:0] tx_buf;
         
         assign tx_data = tx_buf[7:0];
         
         always @(posedge clk) begin
            if ((state == ST_READ_WAIT) && ep_r_ack)
              tx_buf <= ep_r_data;
            else if ((state == ST_BREAD_WAIT) && ep_r_ack)
              tx_buf <= ep_r_data;
            else if ((state == ST_READ) && tx_data_done)
              tx_buf <= {8'h00, tx_buf[RS232C_BUF_BITWIDTH-1:8]};
            else if ((state == ST_BREAD) && tx_data_done)
              tx_buf <= {8'h00, tx_buf[RS232C_BUF_BITWIDTH-1:8]};
         end
      end 
   endgenerate
   
   always @(posedge clk) begin
      if (!n_rst)
        tx_data_en <= 1'b0;
      else if ((state == ST_READ_WAIT) && ep_r_ack)
        tx_data_en <= 1'b1;
      else if ((state == ST_BREAD_WAIT) && ep_r_ack)
        tx_data_en <= 1'b1;      
      else if ((state == ST_READ) && (!last_data_cnt) && tx_data_done)
        tx_data_en <= 1'b1;
      else if ((state == ST_BREAD) && (!last_data_cnt) && tx_data_done)
        tx_data_en <= 1'b1;
      else
        tx_data_en <= 1'b0;
   end
   
   rs232c_tx 
     #(
       .FREQ(FREQ),
       .RATE(RS232C_RATE),
       .PARITY_EN(RS232C_PARITY_EN),
       .PARITY_EVEN(RS232C_PARITY_EVEN)
       )
       rs232c_tx_0
         (
          .clk(clk),
          .n_rst(n_rst),
          .txd(rs232c_txd_out),
          .data(tx_data),
          .en(tx_data_en),
          .done(tx_data_done),
          .busy()
          );

   ////////////////////////////////////////////

   always @(posedge clk) begin
      if (state == ST_ADDR_FETCH) begin
         if ((cmd == CMD_READ) && last_addr_cnt && rx_data_en)
           ep_r_addr <= addr_from_rx;         
         else if ((cmd == CMD_BREAD) && last_addr_cnt && rx_data_en)
           ep_r_addr <= '0;
      end
      else if (state == ST_BREAD) begin
         if (last_data_cnt && tx_data_done)
           ep_r_addr <= ep_r_addr + 1'b1;
      end
   end 

   always @(posedge clk) begin
      if ((state == ST_ADDR_FETCH) && (cmd == CMD_BREAD) && 
          last_addr_cnt && rx_data_en) begin
         burst_addr_reg <= addr_from_rx;
      end
   end
   
   always @(posedge clk) begin
      if (!n_rst)
        ep_r_request <= 1'b0;
      else if (state == ST_ADDR_FETCH) begin
         if ((cmd == CMD_READ) || (cmd == CMD_BREAD)) begin
            if (last_addr_cnt && rx_data_en)
              ep_r_request <= 1'b1;
         end
      end
      else if (state == ST_BREAD) begin
         if (last_data_cnt && tx_data_done && (ep_r_addr != burst_addr_reg)) 
           ep_r_request <= 1'b1;
      end
      else if ((state == ST_READ_WAIT) && ep_r_ack)
        ep_r_request <= 1'b0;
      else if ((state == ST_BREAD_WAIT) && ep_r_ack)
        ep_r_request <= 1'b0;
   end
endmodule


/*
 * Originally designed by <dohi@pca.cis.nagasaki-u.ac.jp>
 */
module rs232c_rx
  #(
    parameter real FREQ        = 25 * 10**6,
    parameter real RATE        = 9600,
    parameter bit  PARITY_EN   = 1'b0,
    parameter bit  PARITY_EVEN = 1'b1
    )
    (
     input wire       clk, 
     input wire       n_rst,
     input wire       rxd,
     output reg [7:0] data,
     output reg       data_en
     );
   
   localparam int CNT_MAX   = FREQ / RATE;
   localparam int CNT_FETCH = FREQ / RATE / 2.0;
   localparam int CNT_WIDTH = $clog2(CNT_MAX);

   typedef enum int unsigned {
      STATE_IDLE,
      STATE_START,
      STATE_BIT0,
      STATE_BIT1,
      STATE_BIT2,      
      STATE_BIT3,
      STATE_BIT4,
      STATE_BIT5,
      STATE_BIT6,            
      STATE_LAST_BIT,
      STATE_PARITY,
      STATE_STOP_BIT
   } state_t;
      
   reg [8:0]    temp;
   
   state_t state = STATE_IDLE;
   reg [CNT_WIDTH-1:0]   cnt = '0;

   reg                   parity_check_ok;
   
   // activated when fetching a bit value 
   wire                  bit_fetch = (cnt == CNT_FETCH-1)? 1'b1: 1'b0;
   
   // activated when moving to a next bit
   wire                  bit_next = (cnt == CNT_MAX-1)? 1'b1: 1'b0;
   
   always @(posedge clk) begin
      if (!n_rst) 
        state <= STATE_IDLE; 
      else if (state == STATE_IDLE && rxd == 1'b0) // start the transfer
        state <= STATE_START;
      else if (state == STATE_LAST_BIT && bit_next && PARITY_EN == 1'b0) 
        state <= STATE_STOP_BIT;
      else if (state == STATE_LAST_BIT && bit_next && PARITY_EN == 1'b1)
        state <= STATE_PARITY;
      else if (state == STATE_PARITY && bit_next)
        state <= STATE_STOP_BIT;
      else if (state == STATE_STOP_BIT && bit_next)
        state <= STATE_IDLE;
      else if (bit_next) begin
	 case (state)
	   STATE_START: state <= STATE_BIT0;
	   STATE_BIT0: state <= STATE_BIT1;
	   STATE_BIT1: state <= STATE_BIT2;
	   STATE_BIT2: state <= STATE_BIT3;
	   STATE_BIT3: state <= STATE_BIT4;
	   STATE_BIT4: state <= STATE_BIT5;
	   STATE_BIT5: state <= STATE_BIT6;
	   STATE_BIT6: state <= STATE_LAST_BIT;
	 endcase
      end
   end

   always @(posedge clk) begin
      if (!n_rst) 
        cnt <= '0;
      else if ((state != STATE_IDLE) && (cnt != (CNT_MAX-1)))
        cnt <= cnt + 1'b1;
      else 
        cnt <= '0;
   end
   
   always @(posedge clk) begin
      if (!n_rst) 
        data <= '0;
      else if (state == STATE_LAST_BIT && bit_next)
        data <= temp[7:0];
   end

   always @(posedge clk) begin
     if (!n_rst)
       parity_check_ok <= 1'b0;
     else if (PARITY_EN == 1'b0) 
       parity_check_ok <= 1'b1;
     else if (state == STATE_PARITY && bit_next && 
              check_parity(temp[7:0], temp[8], PARITY_EVEN))
       parity_check_ok <= 1'b1;
     else if (state == STATE_IDLE) 
       parity_check_ok <= 1'b0;
   end 
   
   always @(posedge clk) begin
      if (!n_rst)
        data_en <= 1'b0;
      else if (state == STATE_STOP_BIT && bit_fetch && 
               rxd == 1'b1 && parity_check_ok) 
        data_en <= 1'b1;
      else 
        data_en <= 1'b0;
   end
   
   always @(posedge clk) begin
      if (!n_rst)
        temp <= '0;
      else if (bit_fetch && (PARITY_EN == 1'b0))
        temp <= {1'b0, rxd, temp[7:1]};
      else if (bit_fetch && (PARITY_EN == 1'b1))
        temp <= {rxd, temp[8:1]};
   end 

   function bit check_parity
     (
      input [7:0] data,
      input       parity_bit,
      input       parity_even
      );
      
      if (parity_even == 1'b1 && ((~^data) == parity_bit))
        return (1'b1);
      else if (parity_even == 1'b0 && ((^data) == parity_bit))
        return (1'b1);
      else 
        return (1'b0);
   endfunction 
endmodule 


/*
 * Originally designed by <dohi@pca.cis.nagasaki-u.ac.jp>
 */
module rs232c_tx
  #(
    parameter real FREQ        = 25 * 10**6,
    parameter real RATE        = 9600,
    parameter      PARITY_EN   = 1'b0,
    parameter      PARITY_EVEN = 1'b1
    )
    (
     input wire       clk, 
     input wire       n_rst,
     output wire      txd,
     input wire [7:0] data,
     input wire       en,
     output reg       done,
     output wire      busy
     );
   
   localparam int CNT_MAX   = FREQ / RATE;
   localparam int CNT_FETCH = FREQ / RATE / 2.0;
   localparam int CNT_WIDTH = $clog2(CNT_MAX);

   typedef enum int unsigned {
      STATE_IDLE,
      STATE_START,
      STATE_BIT0,
      STATE_BIT1,
      STATE_BIT2,      
      STATE_BIT3,
      STATE_BIT4,
      STATE_BIT5,
      STATE_BIT6,            
      STATE_LAST_BIT,
      STATE_PARITY,
      STATE_STOP_BIT      
   } state_t;

   localparam bit START_BIT = 1'b0;
   localparam bit STOP_BIT = 1'b1;
   
   reg [10:0] temp;
   
   state_t state = STATE_IDLE;
   reg [CNT_WIDTH-1:0]   cnt = '0;
   
   // activated when moving to a next bit
   wire                  bit_next = (cnt == CNT_MAX-1)? 1'b1: 1'b0;

   assign                txd = (state == STATE_IDLE)? 1'b1: temp[0];
   assign                busy = (state == STATE_IDLE)? 1'b0: 1'b1;
   
   always @(posedge clk) begin
      if(!n_rst) 
        state <= STATE_IDLE; 
      else if (state == STATE_IDLE && en) // start the transfer
        state <= STATE_START;
      else if (state == STATE_LAST_BIT && bit_next && PARITY_EN == 1'b0)
        state <= STATE_STOP_BIT;
      else if (state == STATE_LAST_BIT && bit_next && PARITY_EN == 1'b1)
        state <= STATE_PARITY;
      else if (state == STATE_PARITY && bit_next)
        state <= STATE_STOP_BIT;
      else if (state == STATE_STOP_BIT && bit_next)
        state <= STATE_IDLE;
      else if (bit_next) begin
	 case (state)
	   STATE_START: state <= STATE_BIT0;
	   STATE_BIT0: state <= STATE_BIT1;
	   STATE_BIT1: state <= STATE_BIT2;
	   STATE_BIT2: state <= STATE_BIT3;
	   STATE_BIT3: state <= STATE_BIT4;
	   STATE_BIT4: state <= STATE_BIT5;
	   STATE_BIT5: state <= STATE_BIT6;
	   STATE_BIT6: state <= STATE_LAST_BIT;
	 endcase
      end
   end 

   always @(posedge clk) begin
      if (!n_rst)
        done <= 1'b0;
      else if (done)
        done <= 1'b0;
      else if (state == STATE_STOP_BIT && bit_next) 
        done <= 1'b1;
   end
   
   always @(posedge clk) begin
      if (!n_rst) 
        cnt <= '0;
      else if ((state != STATE_IDLE) && (cnt != (CNT_MAX-1)))
        cnt <= cnt + 1'b1;
      else 
        cnt <= '0;
   end
   
   always @(posedge clk) begin
      if (!n_rst) 
        temp <= '0;
      else if (state == STATE_IDLE && en && PARITY_EN == 1'b0)
        temp <= {1'b1, STOP_BIT, data, START_BIT};
      else if (state == STATE_IDLE && en && PARITY_EN == 1'b1)
        temp <= {STOP_BIT, make_parity(data, PARITY_EVEN), data, START_BIT};
      else if (bit_next)
        temp <= {1'b1, temp[10:1]};
   end

   function bit make_parity
     (
      input [7:0] data,
      input       parity_even
      );
      
      if (parity_even == 1'b1)
        return (~^data);
      else 
        return (^data);
   endfunction 
endmodule


module uart_reg_file
  #(
    parameter int DATA_WIDTH = 32,
    parameter int NUM_ENTRIES = 256,
    parameter real FREQ = 125000000,
    parameter real RS232C_RATE = 115200
    )
   (
    input wire 				      clk,
    input wire 				      n_rst,
    input wire 				      uart_rx,
    output wire 			      uart_tx,
    //
    input wire [NUM_ENTRIES*DATA_WIDTH-1:0]   rdata,
    output logic [NUM_ENTRIES*DATA_WIDTH-1:0] wdata = '0,
    //
    input wire [$clog2(NUM_ENTRIES)-1:0]      init_addr,
    input wire [DATA_WIDTH-1:0] 	      init_data,
    input wire 				      init_en,
    input wire 				      init_we
    );

   wire 				      ep_w_en;
   wire [$clog2(NUM_ENTRIES)-1:0] 	      ep_w_addr;
   wire [DATA_WIDTH-1:0] 		      ep_w_data;
   
   wire 				      ep_r_req;
   reg 					      ep_r_ack = 0;
   wire [$clog2(NUM_ENTRIES)-1:0] 	      ep_r_addr;
   reg [DATA_WIDTH-1:0] 		      ep_r_data = 0;
   
   reg 					      ep_r_req_reg = 1'b0;

   reg [DATA_WIDTH-1:0] 		      rdata_tmp[NUM_ENTRIES];

   rs232c_endpoint
     #(
       .FREQ(FREQ),
       .RS232C_RATE(RS232C_RATE),
       .RS232C_PARITY_EN(1'b0),
       .RS232C_PARITY_EVEN(1'b0),

       .EP_N_ENTRIES(NUM_ENTRIES),
       .EP_BITWIDTH(DATA_WIDTH)
       )
   rs232c_endpoint_0
     (
      .clk(clk),
      .n_rst(n_rst),

      .rs232c_rxd_in(uart_rx),
      .rs232c_txd_out(uart_tx),

      .ep_w_en(ep_w_en),
      .ep_w_addr(ep_w_addr),
      .ep_w_data(ep_w_data),

      .ep_r_addr(ep_r_addr),
      .ep_r_data(ep_r_data),
      .ep_r_request(ep_r_req),
      .ep_r_ack(ep_r_ack)
      );

   always @(posedge clk) begin
      if (!n_rst) 
        ep_r_ack <= 1'b0;
      else if (ep_r_req_reg)
        ep_r_ack <= 1'b1;
      else 
        ep_r_ack <= 1'b0;
   end

   // delay ep_r_req 1 clock
   always @(posedge clk) begin
      if (!n_rst)
        ep_r_req_reg <= 1'b0;
      else if (ep_r_req)
        ep_r_req_reg <= 1'b1;
      else
        ep_r_req_reg <= 1'b0;
   end

   // maintain values
   always @(posedge clk) begin
      if (!n_rst) begin
	 rdata_tmp <= '{default: '0};
      end begin 
	 for (int i = 0; i < NUM_ENTRIES; i++)
	   rdata_tmp[i] <= rdata[i * DATA_WIDTH +: DATA_WIDTH];
      end 
   end
         
   always @(posedge clk) begin
      if (ep_r_req_reg) begin
	 for (int i = 0; i < NUM_ENTRIES; i++) begin
            if (ep_r_addr == i)  begin
               ep_r_data <= rdata_tmp[i];
	       break;
	   end
	 end
      end
   end

   always @(posedge clk) begin
      if (!n_rst)
	wdata <= '0;
      else if (init_en) begin
	 if (init_we)
	   wdata[init_addr * DATA_WIDTH +: DATA_WIDTH] <= init_data;
      end
      else if (ep_w_en) begin
	 for (int i = 0; i < NUM_ENTRIES; i++) begin
	    if (ep_w_addr == i) begin
	       wdata[i * DATA_WIDTH +: DATA_WIDTH] <= ep_w_data;
	       break;
	    end
	 end
      end
   end 
endmodule 
`default_nettype wire  
