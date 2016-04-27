set sysname [lindex $argv 0]
set tmpstr [exec pwd]
open_project $tmpstr/$sysname/$sysname.xpr
set proname [current_project]
set vivado_archgen_path $tmpstr

set current_path $tmpstr/[current_project]
set src_path $current_path/$proname.srcs
# set_property ip_repo_paths $tmpstr/HLS_ACC [current_project]
# update_ip_catalog
add_files -norecurse -scan_for_includes $tmpstr/fifo_128_32.xci
import_files -norecurse                 $tmpstr/fifo_128_32.xci
add_files -norecurse -scan_for_includes $tmpstr/AXIS_fifo_32x16.xci
import_files -norecurse                 $tmpstr/AXIS_fifo_32x16.xci
add_files -norecurse -scan_for_includes $tmpstr/jit_fifo.xci
import_files -norecurse                 $tmpstr/jit_fifo.xci
add_files -norecurse -scan_for_includes $tmpstr/jit_clk.xci
import_files -norecurse                 $tmpstr/jit_clk.xci
add_files -norecurse -scan_for_includes $tmpstr/jit_reset.xci
import_files -norecurse                 $tmpstr/jit_reset.xci
# add_files -norecurse -scan_for_includes $tmpstr/jit_blackbox.xci
# import_files -norecurse                 $tmpstr/jit_blackbox.xci
update_compile_order -fileset sources_1
report_ip_status -name ip_status
upgrade_ip [get_ips *]

set tmpstr [exec pwd]
set ModuleName jit_blackbox
set proname [current_project]
set current_path $tmpstr/[current_project]
set src_path $current_path/$proname.srcs
set run_path $current_path/$proname.runs
set new_path $src_path/$ModuleName/new
set xdc_name $ModuleName\_ooc.xdc

file mkdir $new_path
create_fileset -blockset -define_from $ModuleName $ModuleName
close [ open $new_path/$xdc_name w ]
add_files -fileset $ModuleName $new_path/$xdc_name
set filename $new_path/$xdc_name
set fileId [open $filename "w"]
puts -nonewline $fileId "#Sen Ma Created"
close $fileId
set_property USED_IN {out_of_context synthesis implementation}  [get_files $new_path/$xdc_name]
set synth_name $ModuleName\_synth_1

# set_property generate_synth_checkpoint true [get_files $src_path/sources_1/ip/$ModuleName/$ModuleName.xci]
# generate_target all [get_files $src_path/sources_1/ip/$ModuleName/$ModuleName.xci]
# create_ip_run [get_files -of_objects [get_fileset sources_1] $src_path/sources_1/ip/$ModuleName/$ModuleName.xci]
# launch_runs $ModuleName\_synth_1 -jobs 12

cd $vivado_archgen_path/$sysname
exec mkdir $sysname.bits
cd ./$sysname.bits/

reset_run synth_1
launch_runs synth_1 -jobs 12
wait_on_run  synth_1
open_run synth_1 -name synth_1

write_checkpoint -force [expr {$sysname}]_original.dcp
close_project

open_checkpoint [expr {$sysname}]_original.dcp
# set PR_NAME_LIST {1 2 3 4 5 6 7 8}
set PR_NAME_LIST {1 2 3 4}

set path UserWrapper/UserModule
set LIST_LEN [llength $PR_NAME_LIST]
for {set ii 0} {$ii < $LIST_LEN} {incr ii} \
{
  set tmppr ACC_PR[lindex $PR_NAME_LIST $ii]
  set prpath $path/$tmppr
  set_property HD.RECONFIGURABLE 1 [get_cells $prpath]
}

# set offset {{0 0 0 0} {50 20 20 10} {200 80 80 40} {250 100 100 50} {300 120 120 60}}
set offsetX {{50 2 2 2} {50  2  2  2} { 50  2  2  2} { 50   2   2  2} { 50   2   2  2} {80 3 3 3}  {80  3  3  3}  { 80  3  3  3}}
set offsetY {{ 0 0 0 0} {50 20 20 10} {200 80 80 40} {250 100 100 50} {300 120 120 60} { 0 0 0 0}  {50 20 20 10}  {100 40 40 20}}
set sW 27
set sH 49
set DW  0
set DH 19
set B18W  0
set B18H 19
set B36W  0
set B36H  9
set path UserWrapper/UserModule
for {set ii 0} {$ii < $LIST_LEN} {incr ii} \
{
  set tmppr ACC_PR[lindex $PR_NAME_LIST $ii]
  set prpath $path/$tmppr
  set Stmpx   [expr 0 + [lindex $offsetX $ii 0]]
  set Stmpy   [expr 0 + [lindex $offsetY $ii 0]]
  set Dtmpx   [expr 0 + [lindex $offsetX $ii 1]]
  set Dtmpy   [expr 0 + [lindex $offsetY $ii 1]]
  set B18tmpx [expr 0 + [lindex $offsetX $ii 2]]
  set B18tmpy [expr 0 + [lindex $offsetY $ii 2]]
  set B36tmpx [expr 0 + [lindex $offsetX $ii 3]]
  set B36tmpy [expr 0 + [lindex $offsetY $ii 3]]

  set Stmpxx [expr $Stmpx + $sW]
  set Stmpyy [expr $Stmpy + $sH]
  set Stmpsxy SLICE_X$Stmpx\Y$Stmpy
  set Stmpexy SLICE_X$Stmpxx\Y$Stmpyy
  set Stmploc $Stmpsxy\:$Stmpexy

  set Dtmpxx [expr $Dtmpx + $DW]
  set Dtmpyy [expr $Dtmpy + $DH]
  set Dtmpsxy DSP48_X$Dtmpx\Y$Dtmpy
  set Dtmpexy DSP48_X$Dtmpxx\Y$Dtmpyy
  set Dtmploc $Dtmpsxy\:$Dtmpexy

  set B18tmpxx [expr $B18tmpx + $B18W]
  set B18tmpyy [expr $B18tmpy + $B18H]
  set B18tmpsxy RAMB18_X$B18tmpx\Y$B18tmpy
  set B18tmpexy RAMB18_X$B18tmpxx\Y$B18tmpyy
  set B18tmploc $B18tmpsxy\:$B18tmpexy

  set B36tmpxx [expr $B36tmpx + $B36W]
  set B36tmpyy [expr $B36tmpy + $B36H]
  set B36tmpsxy RAMB36_X$B36tmpx\Y$B36tmpy
  set B36tmpexy RAMB36_X$B36tmpxx\Y$B36tmpyy
  set B36tmploc $B36tmpsxy\:$B36tmpexy

  set tmploc $Stmploc\ $Dtmploc\ $B18tmploc\ $B36tmploc
  set tmpname pblock_$tmppr
  startgroup
  create_pblock $tmpname
  resize_pblock $tmpname -add $tmploc
  add_cells_to_pblock $tmpname [get_cells [list $prpath]] -clear_locs
  set_property RESET_AFTER_RECONFIG 1 [get_pblocks $tmpname]
  set_property SNAPPING_MODE ON [get_pblocks $tmpname]
  endgroup
}

write_checkpoint -force [expr {$sysname}]_BeforeSyn.dcp

opt_design
place_design
route_design

set LIST_LEN [llength $PR_NAME_LIST]
for {set ii 0} {$ii < $LIST_LEN} {incr ii} \
{
  set tmppr ACC_PR[lindex $PR_NAME_LIST $ii]
  set prpath $path/$tmppr
  update_design -cell $prpath -black_box
}

lock_design -level routing

write_checkpoint -force [expr {$sysname}]_static.dcp
set_property SEVERITY {Warning} [get_drc_checks LUTLP-1]
write_bitstream -force -file [expr {$sysname}]_static.bit
close_project
