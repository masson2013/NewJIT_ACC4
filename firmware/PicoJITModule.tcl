set sysname [lindex $argv 0]
set ModuleFile [lindex $argv 1]

if {[string match *.dcp $ModuleFile] == 1} {
  set tmpstr [string trim $ModuleFile dcp]
  set ModuleName [string trim $tmpstr .]
}

set tmpstr [exec pwd]
set bits_path $tmpstr/$sysname/$sysname.bits
cd $bits_path
# exec ln -sf ../../../software/bit_h_gen.py bit_h_gen.py

set PR_NAME_LIST {1 2 3 4 5 6 7 8}
############# PR Module ######################
set ACC_NAME $ModuleName
exec cp -u ../../$ACC_NAME.dcp .
open_checkpoint [expr {$sysname}]_static.dcp
set path UserWrapper/UserModule
set LIST_LEN [llength $PR_NAME_LIST]
for {set ii 0} {$ii < $LIST_LEN} {incr ii} \
{
  set tmppr ACC_PR[lindex $PR_NAME_LIST $ii]
  set prpath $path/$tmppr
  read_checkpoint -cell $prpath $ACC_NAME.dcp
}

opt_design
place_design
route_design

write_checkpoint -force [expr {$sysname}]_$ACC_NAME.dcp
write_bitstream -force -file $ACC_NAME.bit
write_checkpoint -force [expr {$sysname}]_$ACC_NAME\_BitDone.dcp

exec cp -u ../../../firmware/pr.py .
exec cp -u ../../../firmware/pr.c .
set LIST_LEN [llength $PR_NAME_LIST]
for {set ii 0} {$ii < $LIST_LEN} {incr ii} \
{
  set tmppr ACC_PR[lindex $PR_NAME_LIST $ii]
  set bitname     [expr {$ACC_NAME}]_pblock_[expr {$tmppr}]_partial
  set newname     [expr {$ACC_NAME}]_PR[lindex $PR_NAME_LIST $ii]
  set newname_bit [expr {$ACC_NAME}]_PR[lindex $PR_NAME_LIST $ii]\_bit
  exec mv [expr {$bitname}].bit [expr {$newname}].bit
  exec xxd -i -c 4 [expr {$newname}].bit >> [expr {$newname}].h
  exec rm -f [expr {$newname_bit}].h
  exec python ./pr.py [expr {$newname}].h
  exec ln -f -s [exec pwd]/[expr {$newname_bit}].h ../../../software/[expr {$newname_bit}].h
}
exec ./bit_h_gen.py
exec cp -u ./jit_bit.h ../../../software/
close_project
