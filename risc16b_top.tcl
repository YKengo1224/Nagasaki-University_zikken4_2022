set DESIGN_NAME $argv
set PROJECT_NAME $DESIGN_NAME\_top
#
project_new $PROJECT_NAME -overwrite
#
set_global_assignment -name FAMILY "Cyclone V"
set_global_assignment -name DEVICE 5CGXFC5C6F27C7
set_global_assignment -name TOP_LEVEL_ENTITY $PROJECT_NAME
set_global_assignment -name CYCLONE_CONFIGURATION_SCHEME "PASSIVE SERIAL"
set_global_assignment -name RESERVE_ALL_UNUSED_PINS "AS INPUT TRI-STATED"
set_global_assignment -name USE_TIMEQUEST_TIMING_ANALYZER ON
set_global_assignment -name NUM_PARALLEL_PROCESSORS 2
#
set_global_assignment -name SYSTEMVERILOG_FILE $PROJECT_NAME\.sv
set_global_assignment -name SYSTEMVERILOG_FILE $DESIGN_NAME\.sv
set_global_assignment -name SDC_FILE $PROJECT_NAME\.sdc
#
# Push-buttons (active low)
#
set_location_assignment PIN_P11  -to KEY[0]
set_location_assignment PIN_P12  -to KEY[1]
set_location_assignment PIN_Y15  -to KEY[2]
set_location_assignment PIN_Y16  -to KEY[3]
set_instance_assignment -name IO_STANDARD "1.2 V" -to KEY[0]
set_instance_assignment -name IO_STANDARD "1.2 V" -to KEY[1]
set_instance_assignment -name IO_STANDARD "1.2 V" -to KEY[2]
set_instance_assignment -name IO_STANDARD "1.2 V" -to KEY[3]
#
# Reset button
#
set_location_assignment PIN_AB24 -to CPU_RESET_n
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to CPU_RESET_n
#
# Green LEDs (active high)
#
set_location_assignment PIN_L7   -to LEDG[0]
set_location_assignment PIN_K6   -to LEDG[1]
set_location_assignment PIN_D8   -to LEDG[2]
set_location_assignment PIN_E9   -to LEDG[3]
set_location_assignment PIN_A5   -to LEDG[4]
set_location_assignment PIN_B6   -to LEDG[5]
set_location_assignment PIN_H8   -to LEDG[6]
set_location_assignment PIN_H9   -to LEDG[7]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDG[0]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDG[1]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDG[2]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDG[3]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDG[4]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDG[5]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDG[6]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDG[7]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDG[0]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDG[1]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDG[2]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDG[3]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDG[4]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDG[5]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDG[6]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDG[7]
set_instance_assignment -name SLEW_RATE 1 -to LEDG[0]
set_instance_assignment -name SLEW_RATE 1 -to LEDG[1]
set_instance_assignment -name SLEW_RATE 1 -to LEDG[2]
set_instance_assignment -name SLEW_RATE 1 -to LEDG[3]
set_instance_assignment -name SLEW_RATE 1 -to LEDG[4]
set_instance_assignment -name SLEW_RATE 1 -to LEDG[5]
set_instance_assignment -name SLEW_RATE 1 -to LEDG[6]
set_instance_assignment -name SLEW_RATE 1 -to LEDG[7]
#
# Red LEDs (active high)
#
set_location_assignment PIN_F7   -to LEDR[0]
set_location_assignment PIN_F6   -to LEDR[1]
set_location_assignment PIN_G6   -to LEDR[2]
set_location_assignment PIN_G7   -to LEDR[3]
set_location_assignment PIN_J8   -to LEDR[4]
set_location_assignment PIN_J7   -to LEDR[5]
set_location_assignment PIN_K10  -to LEDR[6]
set_location_assignment PIN_K8   -to LEDR[7]
set_location_assignment PIN_H7   -to LEDR[8]
set_location_assignment PIN_J10  -to LEDR[9]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDR[0]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDR[1]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDR[2]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDR[3]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDR[4]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDR[5]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDR[6]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDR[7]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDR[8]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDR[9]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDR[0]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDR[1]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDR[2]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDR[3]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDR[4]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDR[5]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDR[6]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDR[7]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDR[8]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to LEDR[9]
set_instance_assignment -name SLEW_RATE 1 -to LEDR[0]
set_instance_assignment -name SLEW_RATE 1 -to LEDR[1]
set_instance_assignment -name SLEW_RATE 1 -to LEDR[2]
set_instance_assignment -name SLEW_RATE 1 -to LEDR[3]
set_instance_assignment -name SLEW_RATE 1 -to LEDR[4]
set_instance_assignment -name SLEW_RATE 1 -to LEDR[5]
set_instance_assignment -name SLEW_RATE 1 -to LEDR[6]
set_instance_assignment -name SLEW_RATE 1 -to LEDR[7]
set_instance_assignment -name SLEW_RATE 1 -to LEDR[8]
set_instance_assignment -name SLEW_RATE 1 -to LEDR[9]
#
# 7-Segment Displays (active low)
#
set_location_assignment PIN_V19  -to HEX0_D[0]
set_location_assignment PIN_V18  -to HEX0_D[1]
set_location_assignment PIN_V17  -to HEX0_D[2]
set_location_assignment PIN_W18  -to HEX0_D[3]
set_location_assignment PIN_Y20  -to HEX0_D[4]
set_location_assignment PIN_Y19  -to HEX0_D[5]
set_location_assignment PIN_Y18  -to HEX0_D[6]
set_location_assignment PIN_AA18 -to HEX1_D[0]
set_location_assignment PIN_AD26 -to HEX1_D[1]
set_location_assignment PIN_AB19 -to HEX1_D[2]
set_location_assignment PIN_AE26 -to HEX1_D[3]
set_location_assignment PIN_AE25 -to HEX1_D[4]
set_location_assignment PIN_AC19 -to HEX1_D[5]
set_location_assignment PIN_AF24 -to HEX1_D[6]
set_location_assignment PIN_AD7  -to HEX2_D[0]
set_location_assignment PIN_AD6  -to HEX2_D[1]
set_location_assignment PIN_U20  -to HEX2_D[2]
set_location_assignment PIN_V22  -to HEX2_D[3]
set_location_assignment PIN_V20  -to HEX2_D[4]
set_location_assignment PIN_W21  -to HEX2_D[5]
set_location_assignment PIN_W20  -to HEX2_D[6]
set_location_assignment PIN_Y24  -to HEX3_D[0]
set_location_assignment PIN_Y23  -to HEX3_D[1]
set_location_assignment PIN_AA23 -to HEX3_D[2]
set_location_assignment PIN_AA22 -to HEX3_D[3]
set_location_assignment PIN_AC24 -to HEX3_D[4]
set_location_assignment PIN_AC23 -to HEX3_D[5]
set_location_assignment PIN_AC22 -to HEX3_D[6]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX0_D[0]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX0_D[1]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX0_D[2]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX0_D[3]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX0_D[4]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX0_D[5]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX0_D[6]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX1_D[0]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX1_D[1]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX1_D[2]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX1_D[3]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX1_D[4]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX1_D[5]
set_instance_assignment -name IO_STANDARD "1.2 V" -to HEX1_D[6]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX2_D[0]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX2_D[1]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX2_D[2]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX2_D[3]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX2_D[4]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX2_D[5]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX2_D[6]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX3_D[0]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX3_D[1]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX3_D[2]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX3_D[3]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX3_D[4]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX3_D[5]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to HEX3_D[6]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX0_D[0]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX0_D[1]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX0_D[2]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX0_D[3]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX0_D[4]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX0_D[5]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX0_D[6]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX1_D[0]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX1_D[1]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX1_D[2]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX1_D[3]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX1_D[4]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX1_D[5]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX1_D[6]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX2_D[0]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX2_D[1]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX2_D[2]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX2_D[3]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX2_D[4]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX2_D[5]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX2_D[6]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX3_D[0]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX3_D[1]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX3_D[2]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX3_D[3]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX3_D[4]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX3_D[5]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" -to HEX3_D[6]
set_instance_assignment -name SLEW_RATE 1 -to HEX0_D[0]
set_instance_assignment -name SLEW_RATE 1 -to HEX0_D[1]
set_instance_assignment -name SLEW_RATE 1 -to HEX0_D[2]
set_instance_assignment -name SLEW_RATE 1 -to HEX0_D[3]
set_instance_assignment -name SLEW_RATE 1 -to HEX0_D[4]
set_instance_assignment -name SLEW_RATE 1 -to HEX0_D[5]
set_instance_assignment -name SLEW_RATE 1 -to HEX0_D[6]
set_instance_assignment -name SLEW_RATE 1 -to HEX1_D[0]
set_instance_assignment -name SLEW_RATE 1 -to HEX1_D[1]
set_instance_assignment -name SLEW_RATE 1 -to HEX1_D[2]
set_instance_assignment -name SLEW_RATE 1 -to HEX1_D[3]
set_instance_assignment -name SLEW_RATE 1 -to HEX1_D[4]
set_instance_assignment -name SLEW_RATE 1 -to HEX1_D[5]
set_instance_assignment -name SLEW_RATE 1 -to HEX1_D[6]
set_instance_assignment -name SLEW_RATE 1 -to HEX2_D[0]
set_instance_assignment -name SLEW_RATE 1 -to HEX2_D[1]
set_instance_assignment -name SLEW_RATE 1 -to HEX2_D[2]
set_instance_assignment -name SLEW_RATE 1 -to HEX2_D[3]
set_instance_assignment -name SLEW_RATE 1 -to HEX2_D[4]
set_instance_assignment -name SLEW_RATE 1 -to HEX2_D[5]
set_instance_assignment -name SLEW_RATE 1 -to HEX2_D[6]
set_instance_assignment -name SLEW_RATE 1 -to HEX3_D[0]
set_instance_assignment -name SLEW_RATE 1 -to HEX3_D[1]
set_instance_assignment -name SLEW_RATE 1 -to HEX3_D[2]
set_instance_assignment -name SLEW_RATE 1 -to HEX3_D[3]
set_instance_assignment -name SLEW_RATE 1 -to HEX3_D[4]
set_instance_assignment -name SLEW_RATE 1 -to HEX3_D[5]
set_instance_assignment -name SLEW_RATE 1 -to HEX3_D[6]
#
# SRAM
#
set_location_assignment PIN_B25 -to SRAM_A[0]
set_location_assignment PIN_B26 -to SRAM_A[1]
set_location_assignment PIN_H19 -to SRAM_A[2]
set_location_assignment PIN_H20 -to SRAM_A[3]
set_location_assignment PIN_D25 -to SRAM_A[4]
set_location_assignment PIN_C25 -to SRAM_A[5]
set_location_assignment PIN_J20 -to SRAM_A[6]
set_location_assignment PIN_J21 -to SRAM_A[7]
set_location_assignment PIN_D22 -to SRAM_A[8]
set_location_assignment PIN_E23 -to SRAM_A[9]
set_location_assignment PIN_G20 -to SRAM_A[10]
set_location_assignment PIN_F21 -to SRAM_A[11]
set_location_assignment PIN_E21 -to SRAM_A[12]
set_location_assignment PIN_F22 -to SRAM_A[13]
set_location_assignment PIN_J25 -to SRAM_A[14]
set_location_assignment PIN_J26 -to SRAM_A[15]
set_location_assignment PIN_N24 -to SRAM_A[16]
set_location_assignment PIN_M24 -to SRAM_A[17]
set_location_assignment PIN_N23 -to SRAM_CE_n
set_location_assignment PIN_E24 -to SRAM_D[0]
set_location_assignment PIN_E25 -to SRAM_D[1]
set_location_assignment PIN_K24 -to SRAM_D[2]
set_location_assignment PIN_K23 -to SRAM_D[3]
set_location_assignment PIN_F24 -to SRAM_D[4]
set_location_assignment PIN_G24 -to SRAM_D[5]
set_location_assignment PIN_L23 -to SRAM_D[6]
set_location_assignment PIN_L24 -to SRAM_D[7]
set_location_assignment PIN_H23 -to SRAM_D[8]
set_location_assignment PIN_H24 -to SRAM_D[9]
set_location_assignment PIN_H22 -to SRAM_D[10]
set_location_assignment PIN_J23 -to SRAM_D[11]
set_location_assignment PIN_F23 -to SRAM_D[12]
set_location_assignment PIN_G22 -to SRAM_D[13]
set_location_assignment PIN_L22 -to SRAM_D[14]
set_location_assignment PIN_K21 -to SRAM_D[15]
set_location_assignment PIN_H25 -to SRAM_LB_n
set_location_assignment PIN_M22 -to SRAM_OE_n
set_location_assignment PIN_M25 -to SRAM_UB_n
set_location_assignment PIN_G25 -to SRAM_WE_n
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[0]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[1]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[2]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[3]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[4]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[5]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[6]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[7]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[8]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[9]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[10]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[11]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[12]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[13]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[14]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[15]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[16]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_A[17]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_CE_n
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[0]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[1]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[2]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[3]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[4]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[5]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[6]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[7]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[8]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[9]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[10]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[11]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[12]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[13]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[14]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_D[15]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_LB_n
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_OE_n
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_UB_n
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to SRAM_WE_n
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[0]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[1]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[2]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[3]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[4]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[5]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[6]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[7]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[8]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[9]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[10]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[11]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[12]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[13]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[14]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[15]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[16]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_A[17]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_CE_n
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_LB_n
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_OE_n
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_UB_n
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_WE_n
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to UART_TX
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[0]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[1]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[2]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[3]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[4]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[5]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[6]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[7]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[8]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[9]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[10]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[11]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[12]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[13]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[14]
set_instance_assignment -name CURRENT_STRENGTH_NEW "MAXIMUM CURRENT" \
    -to SRAM_D[15]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[0]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[1]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[2]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[3]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[4]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[5]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[6]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[7]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[8]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[9]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[10]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[11]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[12]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[13]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[14]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[15]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[16]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_A[17]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_CE_n
set_instance_assignment -name SLEW_RATE 1 -to SRAM_LB_n
set_instance_assignment -name SLEW_RATE 1 -to SRAM_OE_n
set_instance_assignment -name SLEW_RATE 1 -to SRAM_UB_n
set_instance_assignment -name SLEW_RATE 1 -to SRAM_WE_n
set_instance_assignment -name SLEW_RATE 1 -to UART_TX
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[0]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[1]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[2]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[3]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[4]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[5]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[6]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[7]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[8]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[9]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[10]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[11]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[12]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[13]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[14]
set_instance_assignment -name SLEW_RATE 1 -to SRAM_D[15]
#
# UART
# 
set_location_assignment PIN_L9 -to UART_TX
set_location_assignment PIN_M9 -to UART_RX
set_instance_assignment -name IO_STANDARD "2.5 V" -to UART_TX
set_instance_assignment -name IO_STANDARD "2.5 V" -to UART_RX
#
# Slide Switches
#
set_location_assignment PIN_AC9 -to SW[0]
set_location_assignment PIN_AE10 -to SW[1]
set_location_assignment PIN_AD13 -to SW[2]
set_location_assignment PIN_AC8 -to SW[3]
set_location_assignment PIN_W11 -to SW[4]
set_location_assignment PIN_AB10 -to SW[5]
set_location_assignment PIN_V10 -to SW[6]
set_location_assignment PIN_AC10 -to SW[7]
set_instance_assignment -name IO_STANDARD "1.2 V" -to SW[0]
set_instance_assignment -name IO_STANDARD "1.2 V" -to SW[1]
set_instance_assignment -name IO_STANDARD "1.2 V" -to SW[2]
set_instance_assignment -name IO_STANDARD "1.2 V" -to SW[3]
set_instance_assignment -name IO_STANDARD "1.2 V" -to SW[4]
set_instance_assignment -name IO_STANDARD "1.2 V" -to SW[5]
set_instance_assignment -name IO_STANDARD "1.2 V" -to SW[6]
set_instance_assignment -name IO_STANDARD "1.2 V" -to SW[7]
#
# Clock sources
#
set_location_assignment PIN_R20 -to CLOCK_50_B5B
#set_location_assignment PIN_N20 -to CLOCK_50_B6A
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to CLOCK_50_B5B
#set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to CLOCK_50_B6A
#
project_close
