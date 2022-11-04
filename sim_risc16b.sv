`timescale 1ns/1ps
`default_nettype none

module sim_risc16b;
   localparam int  MAX_SIMULATION_CYCLES = 5000000;
   localparam real CLOCK_FREQ_HZ   = 50e6;
   localparam real CLOCK_PERIOD_NS = 1e9 / CLOCK_FREQ_HZ;
   logic           clk, rst;
   logic [15:0]    i_din, i_addr, d_din, d_dout, d_addr, led;
   logic           i_oe, d_oe;
   logic [1:0]     d_we;
   logic [7:0]     mem[2**16];

   initial $readmemb("sim_risc16b.mem", mem);
   initial $readmemh("imfiles/set_image.mem", mem);

   risc16b risc16b_inst(.clk(clk), .rst(rst), .i_addr(i_addr), .i_oe(i_oe), 
                      .i_din(i_din), .d_addr(d_addr), .d_oe(d_oe),
                      .d_din(d_din), .d_dout(d_dout), .d_we(d_we));
   
   initial begin
      clk = 1'b0;
    #(CLOCK_PERIOD_NS / 2)      
      repeat (MAX_SIMULATION_CYCLES) begin
         clk = 1'b1;
       #(CLOCK_PERIOD_NS / 2)
         clk = 1'b0;
       #(CLOCK_PERIOD_NS / 2)    
         print();
         if (risc16b_inst.if_pc == 16'h0018) begin
//            print();
	   dump_and_finish();
	 end
      end
      dump_and_finish();
   end
   
   always_comb begin
      if (i_addr[15:8] != 8'h7f)
        i_din = i_oe? {mem[i_addr & 16'hfffe], mem[i_addr | 16'h1]}: 16'hz;
      else
        i_din = 16'hx;
   end

   always_comb begin
      if (d_addr[15:8] != 8'h7f)
        d_din = d_oe? {mem[d_addr & 16'hfffe], mem[d_addr | 16'h1]}: 16'hz;
      else
        d_din = 16'hx;
   end

   always_ff @(posedge clk) begin
      if (d_addr[15:8] != 8'h7f) begin
         if (d_we[0])
           mem[d_addr & 16'hfffe] <= d_dout[15:8];
         if (d_we[1])
           mem[d_addr | 16'h1] <= d_dout[7:0];
      end
   end

   always_ff @(posedge clk) begin
      if (rst)
        led <= 16'h0000;
      else if (d_addr[15:8] == 8'h7f) begin
         if (d_addr[7:1] == 7'h00) begin // led
            if (d_we[0])
              led[15:8] <= d_dout[15:8];
            if (d_we[1])
              led[7:0] <= d_dout[7:0];
         end
      end
   end
   
   initial begin
      rst = 1'b1;
    #(CLOCK_PERIOD_NS)
      rst = 1'b0;      
   end

   task print(); 
      $write("==== clock: %1d ====\n", $rtoi($time / CLOCK_PERIOD_NS) - 1);
      $write(" if_pc= %x  if_ir= %b (%x)\n",
             risc16b_inst.if_pc, risc16b_inst.if_ir, risc16b_inst.if_ir);
      $write(" i_addr= %x  i_din= %x  i_oe= %b\n", 
             i_addr, i_din, i_oe);
      $write(" id_operand_reg1= %x  id_operand_reg2= %x  id_ir= %b (%x)\n",
             risc16b_inst.id_operand_reg1, risc16b_inst.id_operand_reg2, 
             risc16b_inst.id_ir, risc16b_inst.id_ir);
      $write(" id_imm_reg= %x  id_pc= %x\n",
             risc16b_inst.id_imm_reg, risc16b_inst.id_pc);
      $write(" reg_file_rnum1= %x, reg_file_rnum2= %x\n",
             risc16b_inst.reg_file_rnum1, risc16b_inst.reg_file_rnum2);
      $write(" ex_result_reg= %x  ex_ir= %b (%x)\n", 
             risc16b_inst.ex_result_reg, 
             risc16b_inst.ex_ir, risc16b_inst.ex_ir);
      $write(" alu_ain= %x  alu_bin= %x  alu_op= %04b  alu_dout= %x\n", 
             risc16b_inst.alu_ain, risc16b_inst.alu_bin, risc16b_inst.alu_op, 
             risc16b_inst.alu_dout);
      $write(" d_addr= %x d_din= %x  d_dout= %x  d_oe= %b  d_we= %b\n",
             risc16b_inst.d_addr, risc16b_inst.d_din, risc16b_inst.d_dout,
             risc16b_inst.d_oe, risc16b_inst.d_we);
      $write(" reg_file_wnum= %x  reg_file_din= %x  reg_file_we= %b\n",
             risc16b_inst.reg_file_wnum, risc16b_inst.reg_file_din, 
             risc16b_inst.reg_file_we);
      $write(" if_pc_bta= %x  if_pc_we= %b ", 
             risc16b_inst.if_pc_bta, risc16b_inst.if_pc_we);
      $write(" led= %x\n\n", led);


      $write(" regs:");
      for (int i = 0; i < 8; i++) begin
         $write(" %x", risc16b_inst.reg_file_inst.registers[i]);
      end
      $write("\n\n");

   endtask 

   task dump_and_finish();
      int fp;
      fp= $fopen("sim_risc16b.dump");
      for (int i = 16'hc000; i <= 16'hffff; i+=8) begin
         $fwrite(fp, "%X %X %X %X ", mem[i],   mem[i+1], mem[i+2], mem[i+3]);
	 $fwrite(fp, "%X %X %X %X\n", mem[i+4], mem[i+5], mem[i+6], mem[i+7]);
      end
      $finish;
   endtask
endmodule
`default_nettype wire
