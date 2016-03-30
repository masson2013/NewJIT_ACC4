#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <picodrv.h>
#include <pico_errors.h>
#include "jit_bit.h"

#define SIZE 32

int main(int argc, char* argv[])
{
  int         err, i, j, stream, stream100;
  int         room;
  int         *tmp = NULL;
  int         stream11, stream12, stream13;
  int         stream21, stream22, stream23;
  int         stream31, stream32, stream33;
  int         stream41, stream42, stream43;
  int         stream51, stream52, stream53;
  int         stream61, stream62, stream63;
  int         stream71, stream72, stream73;
  int         stream81, stream82, stream83;

  PicoDrv     *pico;

  uint32_t    buf[1024], u32, addr;
  char        ibuf[1024];
  const char* bitFileName;
  printf("Start\r\n");
  // specify the .bit file name on the command line
    switch(argc) {

      case 1: {
        if ((err = FindPico(0x505, &pico)) < 0) {
            printf("FindPico Error\r\n");
            return err;
        }
      }break;

      case 2: {
        bitFileName = argv[1];
        printf("Loading FPGA with '%s' ...\n", bitFileName);
        err = RunBitFile(bitFileName, &pico);
        if (err < 0) {
            fprintf(stderr, "RunBitFile error: %s\n", PicoErrors_FullError(err, ibuf, sizeof(ibuf)));
            exit(1);
        }
      }break;
    }

  printf("Opening stream50 (CMD)\r\n");
  stream = pico->CreateStream(50);
  if (stream < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream100 (CMD)\r\n");
  stream100 = pico->CreateStream(100);
  if (stream < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream, ibuf, sizeof(ibuf)));
      exit(1);
  }
//==============================================================================
  int *A = new int[SIZE];
  int *B = new int[SIZE];
  int *C = new int[SIZE];
  int *D = new int[SIZE];

  for (i = 0; i < SIZE; i++) {
    A[i] = i + 1;
    B[i] = i + 1;
    C[i] = 0;
    D[i] = 0;
  }
  struct timeval start, end;
  gettimeofday(&start, NULL);
  for (i = 0; i < SIZE; i++) {
    C[i] = A[i] + B[i];
  }
  gettimeofday(&end, NULL);

  int timeuse = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
  printf("CPU %4d threads :\t%9d us\r\n", 1, timeuse);

  int k;
  int node;
  uint32_t  cmd[4]; // {0xC1100000, 0xC1200000, 0xC1300001, 0xB1000000};
//==================================================================================================
//       _    ____ ____       _
//      / \  / ___/ ___|  _  / |
//     / _ \| |  | |     (_) | |
//    / ___ \ |__| |___   _  | |
//   /_/   \_\____\____| (_) |_|
//==================================================================================================
  printf("Opening stream11 (CMD)\r\n");
  stream11 = pico->CreateStream(11);
  if (stream11 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream11, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream12 (CMD)\r\n");
  stream12 = pico->CreateStream(12);
  if (stream12 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream12, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream13 (CMD)\r\n");
  stream13 = pico->CreateStream(13);
  if (stream13 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream13, ibuf, sizeof(ibuf)));
      exit(1);
  }

  node = 1;
  cmd[3] = 0xBABEFACE;
  cmd[2] = 0xDEADBEEF;
  cmd[1] = 0xDEADBEEF;
  cmd[0] = 0xD000BEEF | (node << 24); // PR Start CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  printf("Doing PR: %d words, %dbytes\r\n", acc_vadd_PR1_bit_len, acc_vadd_PR1_bit_len * 4);
  pico->WriteStream(stream100, acc_vadd_PR1_bit, acc_vadd_PR1_bit_len * 4);
  printf("PR Done\r\n");

  cmd[0] = 0xD000DEAD | (node << 24); // PR End CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  k = 0;
  do {
    printf("%i B available to read from 50. %5d\r\n", i=pico->GetBytesAvailable(stream, true), k);
    if (i < 0){
        fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
        exit(1);
    }
    room = i / 4;
    tmp = new int[room];
    pico->ReadStream(stream, tmp, i);
    printf("%d\r\n", room);
    // for (i = 0; i < room; i++) {
    //   printf("[%d] 0x%08x\r\n", i, tmp[i]);
    // }
    delete[] tmp;
    k++;
  } while (room != 0 && k < 10000);

  cmd[0] = 0xC1100000;
  cmd[1] = 0xC1200000 | SIZE;
  cmd[2] = 0xC1300001;
  cmd[3] = 0xB1000000;
  pico->WriteStream(stream, cmd, 16);

  printf("Lanching ACC\r\n");
  cmd[0] = 0xA1000000;
  cmd[1] = 0x00000000;
  cmd[2] = 0x00000000;
  cmd[3] = 0x00000000;
  pico->WriteStream(stream, cmd, 16);

  pico->WriteStream(stream11, A, SIZE * 4);
  pico->WriteStream(stream12, B, SIZE * 4);
  printf("%i B available to read from 13.\n", i=pico->GetBytesAvailable(stream13, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  pico->ReadStream(stream13, tmp, i);
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }
  delete[] tmp;

  printf("%i B available to read from 50.\n", i=pico->GetBytesAvailable(stream, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  pico->ReadStream(stream, tmp, i);
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }
  delete[] tmp;
//==================================================================================================
//       _    ____ ____       ____
//      / \  / ___/ ___|  _  |___ \
//     / _ \| |  | |     (_)   __) |
//    / ___ \ |__| |___   _   / __/
//   /_/   \_\____\____| (_) |_____|
//==================================================================================================
  printf("Opening stream21 (CMD)\r\n");
  stream21 = pico->CreateStream(21);
  if (stream21 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream21, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream22 (CMD)\r\n");
  stream22 = pico->CreateStream(22);
  if (stream22 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream22, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream23 (CMD)\r\n");
  stream23 = pico->CreateStream(23);
  if (stream23 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream23, ibuf, sizeof(ibuf)));
      exit(1);
  }

  node = 2;
  cmd[3] = 0xBABEFACE;
  cmd[2] = 0xDEADBEEF;
  cmd[1] = 0xDEADBEEF;
  cmd[0] = 0xD000BEEF | (node << 24); // PR Start CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  printf("Doing PR: %d words, %dbytes\r\n", acc_vadd_PR2_bit_len, acc_vadd_PR2_bit_len * 4);
  pico->WriteStream(stream100, acc_vadd_PR2_bit, acc_vadd_PR2_bit_len * 4);
  printf("PR Done\r\n");

  cmd[0] = 0xD000DEAD | (node << 24); // PR End CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  k = 0;
  do {
    printf("%i B available to read from 50. %5d\r\n", i=pico->GetBytesAvailable(stream, true), k);
    if (i < 0){
        fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
        exit(1);
    }
    room = i / 4;
    tmp = new int[room];
    pico->ReadStream(stream, tmp, i);
    printf("%d\r\n", room);
    // for (i = 0; i < room; i++) {
    //   printf("[%d] 0x%08x\r\n", i, tmp[i]);
    // }
    delete[] tmp;
    k++;
  } while (room != 0 && k < 10000);

  cmd[0] = 0xC2100000;
  cmd[1] = 0xC2200000 | SIZE;
  cmd[2] = 0xC2300001;
  cmd[3] = 0xB2000000;
  pico->WriteStream(stream, cmd, 16);

  printf("Lanching ACC\r\n");
  cmd[0] = 0xA2000000;
  cmd[1] = 0x00000000;
  cmd[2] = 0x00000000;
  cmd[3] = 0x00000000;
  pico->WriteStream(stream, cmd, 16);

  pico->WriteStream(stream21, A, SIZE * 4);
  pico->WriteStream(stream22, B, SIZE * 4);
  printf("%i B available to read from 23.\n", i=pico->GetBytesAvailable(stream23, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  pico->ReadStream(stream23, tmp, i);
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }
  delete[] tmp;

  printf("%i B available to read from 50.\n", i=pico->GetBytesAvailable(stream, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  pico->ReadStream(stream, tmp, i);
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }
  delete[] tmp;
//==============================================================================
//       _    ____ ____       _____
//      / \  / ___/ ___|  _  |___ /
//     / _ \| |  | |     (_)   |_ \
//    / ___ \ |__| |___   _   ___) |
//   /_/   \_\____\____| (_) |____/
//==============================================================================
  printf("Opening stream31 (CMD)\r\n");
  stream31 = pico->CreateStream(31);
  if (stream31 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream31, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream32 (CMD)\r\n");
  stream32 = pico->CreateStream(32);
  if (stream32 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream32, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream33 (CMD)\r\n");
  stream33 = pico->CreateStream(33);
  if (stream33 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream33, ibuf, sizeof(ibuf)));
      exit(1);
  }

  node = 3;
  cmd[3] = 0xBABEFACE;
  cmd[2] = 0xDEADBEEF;
  cmd[1] = 0xDEADBEEF;
  cmd[0] = 0xD000BEEF | (node << 24); // PR Start CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  printf("Doing PR: %d words, %dbytes\r\n", acc_vadd_PR3_bit_len, acc_vadd_PR3_bit_len * 4);
  pico->WriteStream(stream100, acc_vadd_PR3_bit, acc_vadd_PR3_bit_len * 4);
  printf("PR Done\r\n");

  cmd[0] = 0xD000DEAD | (node << 24); // PR End CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  k = 0;
  do {
    printf("%i B available to read from 50. %5d\r\n", i=pico->GetBytesAvailable(stream, true), k);
    if (i < 0){
        fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
        exit(1);
    }
    room = i / 4;
    tmp = new int[room];
    pico->ReadStream(stream, tmp, i);
    printf("%d\r\n", room);
    // for (i = 0; i < room; i++) {
    //   printf("[%d] 0x%08x\r\n", i, tmp[i]);
    // }
    delete[] tmp;
    k++;
  } while (room != 0 && k < 10000);

  cmd[0] = 0xC3100000;
  cmd[1] = 0xC3200000 | SIZE;
  cmd[2] = 0xC3300001;
  cmd[3] = 0xB3000000;
  pico->WriteStream(stream, cmd, 16);

  printf("Lanching ACC\r\n");
  cmd[0] = 0xA3000000;
  cmd[1] = 0x00000000;
  cmd[2] = 0x00000000;
  cmd[3] = 0x00000000;
  pico->WriteStream(stream, cmd, 16);

  pico->WriteStream(stream31, A, SIZE * 4);
  pico->WriteStream(stream32, B, SIZE * 4);
  printf("%i B available to read from 33.\n", i=pico->GetBytesAvailable(stream33, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  pico->ReadStream(stream33, tmp, i);
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }
  delete[] tmp;

  printf("%i B available to read from 50.\n", i=pico->GetBytesAvailable(stream, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  pico->ReadStream(stream, tmp, i);
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }
  delete[] tmp;
//==============================================================================
//       _    ____ ____       _  _
//      / \  / ___/ ___|  _  | || |
//     / _ \| |  | |     (_) | || |_
//    / ___ \ |__| |___   _  |__   _|
//   /_/   \_\____\____| (_)    |_|
//==============================================================================
  printf("Opening stream41 (CMD)\r\n");
  stream41 = pico->CreateStream(41);
  if (stream41 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream41, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream42 (CMD)\r\n");
  stream42 = pico->CreateStream(42);
  if (stream42 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream42, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream43 (CMD)\r\n");
  stream43 = pico->CreateStream(43);
  if (stream43 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream43, ibuf, sizeof(ibuf)));
      exit(1);
  }

  node = 4;
  cmd[3] = 0xBABEFACE;
  cmd[2] = 0xDEADBEEF;
  cmd[1] = 0xDEADBEEF;
  cmd[0] = 0xD000BEEF | (node << 24); // PR Start CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  printf("Doing PR: %d words, %dbytes\r\n", acc_vadd_PR4_bit_len, acc_vadd_PR4_bit_len * 4);
  pico->WriteStream(stream100, acc_vadd_PR4_bit, acc_vadd_PR4_bit_len * 4);
  printf("PR Done\r\n");

  cmd[0] = 0xD000DEAD | (node << 24); // PR End CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  k = 0;
  do {
    printf("%i B available to read from 50. %5d\r\n", i=pico->GetBytesAvailable(stream, true), k);
    if (i < 0){
        fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
        exit(1);
    }
    room = i / 4;
    tmp = new int[room];
    pico->ReadStream(stream, tmp, i);
    printf("%d\r\n", room);
    // for (i = 0; i < room; i++) {
    //   printf("[%d] 0x%08x\r\n", i, tmp[i]);
    // }
    delete[] tmp;
    k++;
  } while (room != 0 && k < 10000);

  cmd[0] = 0xC4100000;
  cmd[1] = 0xC4200000 | SIZE;
  cmd[2] = 0xC4300001;
  cmd[3] = 0xB4000000;
  pico->WriteStream(stream, cmd, 16);

  printf("Lanching ACC\r\n");
  cmd[0] = 0xA4000000;
  cmd[1] = 0x00000000;
  cmd[2] = 0x00000000;
  cmd[3] = 0x00000000;
  pico->WriteStream(stream, cmd, 16);

  err = pico->WriteStream(stream41, A, SIZE * 4);
  err = pico->WriteStream(stream42, B, SIZE * 4);
  printf("%i B available to read from stream43.\n", i=pico->GetBytesAvailable(stream43, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  err = pico->ReadStream(stream43, tmp, i);
  if (err < 0) {
      fprintf(stderr, "ReadStream stream43 error: %s\n", PicoErrors_FullError(err, ibuf, sizeof(ibuf)));
      exit(1);
  }
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }

  printf("%i B available to read from stream.\n", i=pico->GetBytesAvailable(stream, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  pico->ReadStream(stream, tmp, i);
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }
  delete[] tmp;

//==============================================================================
//       _    ____ ____       ____
//      / \  / ___/ ___|  _  | ___|
//     / _ \| |  | |     (_) |___ \
//    / ___ \ |__| |___   _   ___) |
//   /_/   \_\____\____| (_) |____/
//==============================================================================
  printf("Opening stream51 (CMD)\r\n");
  stream51 = pico->CreateStream(51);
  if (stream51 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream51, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream52 (CMD)\r\n");
  stream52 = pico->CreateStream(52);
  if (stream52 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream52, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream53 (CMD)\r\n");
  stream53 = pico->CreateStream(53);
  if (stream53 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream53, ibuf, sizeof(ibuf)));
      exit(1);
  }

  node = 5;
  cmd[3] = 0xBABEFACE;
  cmd[2] = 0xDEADBEEF;
  cmd[1] = 0xDEADBEEF;
  cmd[0] = 0xD000BEEF | (node << 24); // PR Start CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  printf("Doing PR: %d words, %dbytes\r\n", acc_vadd_PR5_bit_len, acc_vadd_PR5_bit_len * 4);
  pico->WriteStream(stream100, acc_vadd_PR5_bit, acc_vadd_PR5_bit_len * 4);
  printf("PR Done\r\n");

  cmd[0] = 0xD000DEAD | (node << 24); // PR End CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  k = 0;
  do {
    printf("%i B available to read from 50. %5d\r\n", i=pico->GetBytesAvailable(stream, true), k);
    if (i < 0){
        fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
        exit(1);
    }
    room = i / 4;
    tmp = new int[room];
    pico->ReadStream(stream, tmp, i);
    printf("%d\r\n", room);
    // for (i = 0; i < room; i++) {
    //   printf("[%d] 0x%08x\r\n", i, tmp[i]);
    // }
    delete[] tmp;
    k++;
  } while (room != 0 && k < 10000);

  cmd[0] = 0xC5100000;
  cmd[1] = 0xC5200000 | SIZE;
  cmd[2] = 0xC5300001;
  cmd[3] = 0xB5000000;
  pico->WriteStream(stream, cmd, 16);

  printf("Lanching ACC\r\n");
  cmd[0] = 0xA5000000;
  cmd[1] = 0x00000000;
  cmd[2] = 0x00000000;
  cmd[3] = 0x00000000;
  pico->WriteStream(stream, cmd, 16);

  err = pico->WriteStream(stream51, A, SIZE * 4);
  err = pico->WriteStream(stream52, B, SIZE * 4);
  printf("%i B available to read from stream53.\n", i=pico->GetBytesAvailable(stream53, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  err = pico->ReadStream(stream53, tmp, i);
  if (err < 0) {
      fprintf(stderr, "ReadStream stream53 error: %s\n", PicoErrors_FullError(err, ibuf, sizeof(ibuf)));
      exit(1);
  }
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }

  printf("%i B available to read from stream.\n", i=pico->GetBytesAvailable(stream, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  pico->ReadStream(stream, tmp, i);
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }
  delete[] tmp;
//==============================================================================
//       _    ____ ____        __
//      / \  / ___/ ___|  _   / /_
//     / _ \| |  | |     (_) | '_ \
//    / ___ \ |__| |___   _  | (_) |
//   /_/   \_\____\____| (_)  \___/
//==============================================================================
  printf("Opening stream61 (CMD)\r\n");
  stream61 = pico->CreateStream(61);
  if (stream61 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream61, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream62 (CMD)\r\n");
  stream62 = pico->CreateStream(62);
  if (stream62 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream62, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream63 (CMD)\r\n");
  stream63 = pico->CreateStream(63);
  if (stream63 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream63, ibuf, sizeof(ibuf)));
      exit(1);
  }

  node = 6;
  cmd[3] = 0xBABEFACE;
  cmd[2] = 0xDEADBEEF;
  cmd[1] = 0xDEADBEEF;
  cmd[0] = 0xD000BEEF | (node << 24); // PR Start CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  printf("Doing PR: %d words, %dbytes\r\n", acc_vadd_PR6_bit_len, acc_vadd_PR6_bit_len * 4);
  pico->WriteStream(stream100, acc_vadd_PR6_bit, acc_vadd_PR6_bit_len * 4);
  printf("PR Done\r\n");

  cmd[0] = 0xD000DEAD | (node << 24); // PR End CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  k = 0;
  do {
    printf("%i B available to read from 50. %5d\r\n", i=pico->GetBytesAvailable(stream, true), k);
    if (i < 0){
        fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
        exit(1);
    }
    room = i / 4;
    tmp = new int[room];
    pico->ReadStream(stream, tmp, i);
    printf("%d\r\n", room);
    // for (i = 0; i < room; i++) {
    //   printf("[%d] 0x%08x\r\n", i, tmp[i]);
    // }
    delete[] tmp;
    k++;
  } while (room != 0 && k < 10000);

  cmd[0] = 0xC6100000;
  cmd[1] = 0xC6200000 | SIZE;
  cmd[2] = 0xC6300001;
  cmd[3] = 0xB6000000;
  pico->WriteStream(stream, cmd, 16);

  printf("Lanching ACC\r\n");
  cmd[0] = 0xA6000000;
  cmd[1] = 0x00000000;
  cmd[2] = 0x00000000;
  cmd[3] = 0x00000000;
  pico->WriteStream(stream, cmd, 16);

  err = pico->WriteStream(stream61, A, SIZE * 4);
  err = pico->WriteStream(stream62, B, SIZE * 4);
  printf("%i B available to read from stream63.\n", i=pico->GetBytesAvailable(stream63, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  err = pico->ReadStream(stream63, tmp, i);
  if (err < 0) {
      fprintf(stderr, "ReadStream stream63 error: %s\n", PicoErrors_FullError(err, ibuf, sizeof(ibuf)));
      exit(1);
  }
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }

  printf("%i B available to read from stream.\n", i=pico->GetBytesAvailable(stream, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  pico->ReadStream(stream, tmp, i);
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }
  delete[] tmp;

//==============================================================================
//       _    ____ ____       _____
//      / \  / ___/ ___|  _  |___  |
//     / _ \| |  | |     (_)    / /
//    / ___ \ |__| |___   _    / /
//   /_/   \_\____\____| (_)  /_/
//==============================================================================
  printf("Opening stream71 (CMD)\r\n");
  stream71 = pico->CreateStream(71);
  if (stream71 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream71, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream72 (CMD)\r\n");
  stream72 = pico->CreateStream(72);
  if (stream72 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream72, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream73 (CMD)\r\n");
  stream73 = pico->CreateStream(73);
  if (stream73 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream73, ibuf, sizeof(ibuf)));
      exit(1);
  }

  node = 7;
  cmd[3] = 0xBABEFACE;
  cmd[2] = 0xDEADBEEF;
  cmd[1] = 0xDEADBEEF;
  cmd[0] = 0xD000BEEF | (node << 24); // PR Start CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  printf("Doing PR: %d words, %dbytes\r\n", acc_vadd_PR7_bit_len, acc_vadd_PR7_bit_len * 4);
  pico->WriteStream(stream100, acc_vadd_PR7_bit, acc_vadd_PR7_bit_len * 4);
  printf("PR Done\r\n");

  cmd[0] = 0xD000DEAD | (node << 24); // PR End CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  k = 0;
  do {
    printf("%i B available to read from 50. %5d\r\n", i=pico->GetBytesAvailable(stream, true), k);
    if (i < 0){
        fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
        exit(1);
    }
    room = i / 4;
    tmp = new int[room];
    pico->ReadStream(stream, tmp, i);
    printf("%d\r\n", room);
    // for (i = 0; i < room; i++) {
    //   printf("[%d] 0x%08x\r\n", i, tmp[i]);
    // }
    delete[] tmp;
    k++;
  } while (room != 0 && k < 10000);

  cmd[0] = 0xC7100000;
  cmd[1] = 0xC7200000 | SIZE;
  cmd[2] = 0xC7300001;
  cmd[3] = 0xB7000000;
  pico->WriteStream(stream, cmd, 16);

  printf("Lanching ACC\r\n");
  cmd[0] = 0xA7000000;
  cmd[1] = 0x00000000;
  cmd[2] = 0x00000000;
  cmd[3] = 0x00000000;
  pico->WriteStream(stream, cmd, 16);

  err = pico->WriteStream(stream71, A, SIZE * 4);
  err = pico->WriteStream(stream72, B, SIZE * 4);
  printf("%i B available to read from stream73.\n", i=pico->GetBytesAvailable(stream73, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  err = pico->ReadStream(stream73, tmp, i);
  if (err < 0) {
      fprintf(stderr, "ReadStream stream73 error: %s\n", PicoErrors_FullError(err, ibuf, sizeof(ibuf)));
      exit(1);
  }
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }

  printf("%i B available to read from stream.\n", i=pico->GetBytesAvailable(stream, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  pico->ReadStream(stream, tmp, i);
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }
  delete[] tmp;
//==============================================================================
//       _    ____ ____        ___
//      / \  / ___/ ___|  _   ( _ )
//     / _ \| |  | |     (_)  / _ \
//    / ___ \ |__| |___   _  | (_) |
//   /_/   \_\____\____| (_)  \___/
//==============================================================================
  printf("Opening stream81 (CMD)\r\n");
  stream81 = pico->CreateStream(81);
  if (stream81 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream81, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream82 (CMD)\r\n");
  stream82 = pico->CreateStream(82);
  if (stream82 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream82, ibuf, sizeof(ibuf)));
      exit(1);
  }

  printf("Opening stream83 (CMD)\r\n");
  stream83 = pico->CreateStream(83);
  if (stream83 < 0) {
      fprintf(stderr, "CreateStream error: %s\n", PicoErrors_FullError(stream83, ibuf, sizeof(ibuf)));
      exit(1);
  }

  node = 8;
  cmd[3] = 0xBABEFACE;
  cmd[2] = 0xDEADBEEF;
  cmd[1] = 0xDEADBEEF;
  cmd[0] = 0xD000BEEF | (node << 24); // PR Start CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  printf("Doing PR: %d words, %dbytes\r\n", acc_vadd_PR8_bit_len, acc_vadd_PR8_bit_len * 4);
  pico->WriteStream(stream100, acc_vadd_PR8_bit, acc_vadd_PR8_bit_len * 4);
  printf("PR Done\r\n");

  cmd[0] = 0xD000DEAD | (node << 24); // PR End CMD
  printf("0x%08x\r\n", cmd[0]);
  pico->WriteStream(stream, cmd, 16);

  k = 0;
  do {
    printf("%i B available to read from 50. %5d\r\n", i=pico->GetBytesAvailable(stream, true), k);
    if (i < 0){
        fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
        exit(1);
    }
    room = i / 4;
    tmp = new int[room];
    pico->ReadStream(stream, tmp, i);
    printf("%d\r\n", room);
    // for (i = 0; i < room; i++) {
    //   printf("[%d] 0x%08x\r\n", i, tmp[i]);
    // }
    delete[] tmp;
    k++;
  } while (room != 0 && k < 10000);

  cmd[0] = 0xC8100000;
  cmd[1] = 0xC8200000 | SIZE;
  cmd[2] = 0xC8300001;
  cmd[3] = 0xB8000000;
  pico->WriteStream(stream, cmd, 16);

  printf("Lanching ACC\r\n");
  cmd[0] = 0xA8000000;
  cmd[1] = 0x00000000;
  cmd[2] = 0x00000000;
  cmd[3] = 0x00000000;
  pico->WriteStream(stream, cmd, 16);

  err = pico->WriteStream(stream81, A, SIZE * 4);
  err = pico->WriteStream(stream82, B, SIZE * 4);
  printf("%i B available to read from stream83.\n", i=pico->GetBytesAvailable(stream83, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  err = pico->ReadStream(stream83, tmp, i);
  if (err < 0) {
      fprintf(stderr, "ReadStream stream83 error: %s\n", PicoErrors_FullError(err, ibuf, sizeof(ibuf)));
      exit(1);
  }
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }

  printf("%i B available to read from stream.\n", i=pico->GetBytesAvailable(stream, true));
  if (i < 0){
      fprintf(stderr, "GetBytesAvailable error: %s\n", PicoErrors_FullError(i, ibuf, sizeof(ibuf)));
      exit(1);
  }
  room = i / 4;
  tmp = new int[room];
  pico->ReadStream(stream, tmp, i);
  printf("%d\r\n", room);
  for (i = 0; i < room; i++) {
    printf("[%d] 0x%08x\r\n", i, tmp[i]);
  }
  delete[] tmp;
//==============================================================================
  printf("Closing streams (CMD)\r\n");
  pico->CloseStream(stream);
  pico->CloseStream(stream100);
  pico->CloseStream(stream11);
  pico->CloseStream(stream12);
  pico->CloseStream(stream13);
  pico->CloseStream(stream21);
  pico->CloseStream(stream22);
  pico->CloseStream(stream23);
  printf("Done\r\n");
  delete[] A;
  delete[] B;
  delete[] C;
  delete[] D;
  return 0;
}
