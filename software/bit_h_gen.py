#!/usr/bin/python
import sys, os, commands

def main():
  PATH = os.getcwd()
  LIST = os.listdir(PATH)
  LIST.sort()

  file = open("jit_bit.h", "w")
  for i in range(len(LIST)):
    if ("_bit.h" in LIST[i]):
      if (LIST[i] == "jit_bit.h"):
        continue
      print LIST[i]
      file.write("#include \"" + LIST[i] + "\"\r\n")
  file.close()

if __name__ == "__main__":
  main()
