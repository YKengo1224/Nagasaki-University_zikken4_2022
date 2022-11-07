###################################################################
# Project Configuration: 
# 
# Specify the name of the design (project), the Quartus II Settings
# File (.qsf), and the list of source files used.
###################################################################

DESIGN = risc16b
PROJECT = $(DESIGN)_top
SOURCE_FILES = $(PROJECT).sv $(DESIGN).sv $(PROJECT).sdc
ASSIGNMENT_FILES = $(PROJECT).qpf $(PROJECT).qsf
TCL_FILE = $(PROJECT).tcl
SIM_TOP  = sim_$(DESIGN)
SIM_FILE = $(SIM_TOP).sv
SIM_WORK = work
SIM_DO   = "add wave -r /$(SIM_TOP)/$(DESIGN)_inst/*; run -all"
CHECKER  = check_$(SIM_TOP)
PGM_FILE = $(SIM_TOP).pgm
IMG_DUMP = $(SIM_TOP).dump
IMG_DIR  = imfiles
REF_DUMP = answer.dump



###################################################################
# Main Targets
#
# all: build everything
# clean: remove output files and database
###################################################################

all: smart.log $(PROJECT).asm.rpt $(PROJECT).sta.rpt $(PROJECT).resource.html

clean:
	rm -rf *.rbf *.rpt *.chg smart.log *.htm *.eqn *.pin *.sof *.jdi \
	*.pof db INCA_libs ncverilog.log incremental_db *~ \
v	*.sld *_pin_model_dump.txt *.done *.html *.dump \
	$(ASSIGNMENT_FILES) *.summary *.smsg *.dpf *.qws \
	*.shm *.wlf *.log $(SIM_WORK) transcript

config: 
	quartus_pgm -m JTAG -o "p;$(PROJECT).sof@1"

map: smart.log $(PROJECT).map.rpt
fit: smart.log $(PROJECT).fit.rpt
asm: smart.log $(PROJECT).asm.rpt
sta: smart.log $(PROJECT).sta.rpt
rep: smart.log $(PROJECT).report.html
smart: smart.log

###################################################################
# Executable Configuration
###################################################################

MAP_ARGS = --family="Cyclone V"
FIT_ARGS = --part=5CGXFC5C6F27C7
ASM_ARGS =
STA_ARGS = --do_report_timing --model=slow --temperature=85 --voltage=1100

###################################################################
# Target implementations
###################################################################

STAMP = echo done >

$(PROJECT).map.rpt: map.chg $(SOURCE_FILES) 
	quartus_map $(MAP_ARGS) $(PROJECT)
	$(STAMP) fit.chg

$(PROJECT).fit.rpt: fit.chg $(PROJECT).map.rpt
	quartus_fit $(FIT_ARGS) $(PROJECT)
	$(STAMP) asm.chg
	$(STAMP) sta.chg
	$(STAMP) htm.chg

$(PROJECT).asm.rpt: asm.chg $(PROJECT).fit.rpt
	quartus_asm $(ASM_ARGS) $(PROJECT)

$(PROJECT).sta.rpt: sta.chg $(PROJECT).fit.rpt 
	quartus_sta $(STA_ARGS) $(PROJECT)

$(PROJECT).resource.html: htm.chg $(PROJECT).fit.rpt
	quartus_sh -t $(PROJECT).report.tcl $(PROJECT) 

smart.log: $(ASSIGNMENT_FILES)
	quartus_sh --determine_smart_action $(PROJECT) > smart.log

###################################################################
# Project initialization
###################################################################

$(ASSIGNMENT_FILES): $(TCL_FILE)
	quartus_sh -t $(TCL_FILE) $(DESIGN)

map.chg:
	$(STAMP) map.chg
fit.chg:
	$(STAMP) fit.chg
sta.chg:
	$(STAMP) sta.chg
asm.chg:
	$(STAMP) asm.chg
htm.chg:
	$(STAMP) htm.chg

###################################################################
# Simulation
###################################################################
$(SIM_WORK):
	vlib work

sim: $(SIM_WORK)
	vlog $(SIM_FILE) $(DESIGN).sv
	vsim -c $(SIM_TOP) -l $(SIM_TOP).log -wlf $(SIM_TOP).wlf -do $(SIM_DO)


check: $(CHECKER)
	./$(CHECKER) $(IMG_DUMP) $(PGM_FILE)
	diff $(IMG_DUMP) $(IMG_DIR)/$(REF_DUMP)


ASSEMBLER = assemble.py
assemble:$(ASSEMBLER)
	python3.8 $(ASSEMBLER)
