#
# General setting
#
set_time_format -unit ns -decimal_places 3
#
# Clock constraints
#
create_clock -name {sys_clk}  -period 5.0 -waveform { 0.0 2.5 } \
      [get_ports {CLOCK_50_B5B}]

create_clock -name {vclk} -period 5.0 -waveform { 0.0 2.5 }

create_generated_clock \
 -source [get_ports {CLOCK_50_B5B}] \
 -name {clk_div16} \
 -divide_by 16 \
 -master_clock {sys_clk} \
  {processor_clk}

create_generated_clock \
 -source [get_ports {CLOCK_50_B5B}] \
 -add \
 -name {processor_clk} \
 -divide_by 1 \
 -master_clock {sys_clk} \
  {processor_clk}

set_clock_groups -asynchronous \
 -group {clk_div16} -group {processor_clk} -group {sys_clk}
#
# Input constraints
#
set_false_path -from [get_ports {CLOCK_50_B5B}]
set_false_path -from [get_ports {CPU_RESET_n}]
set_false_path -from [get_ports {KEY*}]
set_false_path -from [get_ports {SRAM*}]
set_false_path -from [get_ports {UART*}]
set_false_path -from [get_ports {SW*}]
#
# Output constraints
#
set_false_path -to [get_ports {LED*}]
set_false_path -to [get_ports {HEX*}]
set_false_path -to [get_ports {SRAM*}]
set_output_delay            -clock vclk -max 0 [get_ports {UART_TX}]
set_output_delay -add_delay -clock vclk -min 0 [get_ports {UART_TX}]
