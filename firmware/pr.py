#!/usr/bin/python
import sys, os, re, commands, pprint

def smart_replace_file(tmpname, nline, line):
  tmpw = open('tmpw', 'w')
  file = open(tmpname,'r')
  tmplines = file.readlines()
  index = 0
  for tmpline in tmplines:
    if index == nline:
      tmpw.write(line)
    else:
      tmpw.write(tmpline)
    index = index + 1
  file.close()
  tmpw.close()
  commands.getstatusoutput('rm ' + tmpname)
  commands.getstatusoutput('mv ' + 'tmpw ' + tmpname)

def main():
  # --- Read number of arguments --- #
  num_args = len(sys.argv)
  if num_args >= 2:
    # --- Save the source file name --- #
    SRCFILE = sys.argv[1]
  else:
    # --- Insufficient number of args, exit with error --- #
    print "Incorrect argument usage!! Aborting..."
    print "Correct usage :\n    ./pr.py acc_vadd_pr.h\n"
    sys.exit(1)

  # Insert the name into the c file
  ACC_INCLUDE = "#include \"" + SRCFILE + "\"\r\n";
  NAME = SRCFILE.split(".")[0]
  ACC_NAME = "#define STR_ACC_NAME \"" + NAME + "\"\r\n"
  ACC_VAR  = "#define STR_VAR " + NAME + "_bit\r\n"
  ACC_FILE = "#define STR_FILE_NAME \"" + NAME + "_bit.h\"\r\n"

  STR = ACC_INCLUDE + ACC_NAME + ACC_VAR + ACC_FILE;
  # print STR
  # execute the c file
  commands.getstatusoutput('cp pr.c pr_tmp.c')
  smart_replace_file("pr_tmp.c", 4, STR)
  commands.getstatusoutput('gcc pr_tmp.c')
  commands.getstatusoutput('./a.out')
  commands.getstatusoutput('rm -f a.out')
  commands.getstatusoutput('rm -f pr_tmp.c')
  cmd = 'rm -f ' + SRCFILE
  commands.getstatusoutput(cmd)

if __name__ == "__main__":
  main()
