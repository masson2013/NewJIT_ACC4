#include <iostream>
#include <vector>
// #include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <picodrv.h>
#include <pico_errors.h>
#include <sys/time.h>
#include <pthread.h>
#include <cmath>
#include <locale.h>

using namespace std;

 // #define VERBOSE
 // #define VERBOSE_THREAD
// #define P_TEST
#include "jit_isa.h"
// 1024 * 10 * 1024
// 524880000 = 1.9 GBytes
// 402653184 = 1.5 GBytes
// 314572800 = 1.2 GBytes
// 78643200  = 300 MBytes
// 52428800  = 200 MBytes
// 26214400  = 100 MBytes
// FCCM Test:

// #define THREADS 16
#define SIZE 524880000
// #define SIZE 0x10000
// #define SIZE    (MAX_SIZE + 32) * THREADS
// #define SIZE    65528 * THREADS

#define SWSIZE  SIZE
#define STEPS   1
#define MAX_SIZE   65528


// #define ITEMS   SIZE

// #define DEBUG
#define NODEBUG

typedef struct {
  int key;
  int len;
  int *A;
  int *B;
  int *C;
}pe_t;

typedef struct {
  int          task_id;
  vam_vm_t     *VM;
  vector<int>  *nPR;
  int len;
  int     *In1;
  int     SizeIn1;
  int     *In2;
  int     SizeIn2;
  int     *Out;
  int     SizeOut;
}task_pk_t;

void * threads_call(void* ptr)
{
  pe_t *p = (pe_t *)ptr;
  // printf("Key:%4d\tLen:%4d\tA:%p\tB:%p\tC:%p\r\n", p->key, p->len, p->A, p->B, p->C);
  int i;
  for (i = 0; i < p->len; i++) {
    p->C[i] = p->A[i] - p->B[i];
  }
  return (void*) 0;
}

void * VMUL_Threads_Call(void *pk)
{
#ifdef VERBOSE_THREAD
  printf("\r\n");
#endif
  task_pk_t *p      = (task_pk_t*) pk;
#ifdef VERBOSE_THREAD
  printf("[DEBUG->task_thread:%2d] task thread start\r\n", p->task_id);
#endif
  vam_vm_t    *VM      = p->VM;
  vector<int> *nPR     = p->nPR;
  int         items    = p->len;
  int         *In1     = p->In1;
  int         sizeIn1  = p->SizeIn1;
  int         *In2     = p->In2;
  int         sizeIn2  = p->SizeIn2;
  int         *Out     = p->Out;
  int         sizeOut  = p->SizeOut;
  int         err;

  int         iter     = items / MAX_SIZE;
  int         reminder = items - iter * MAX_SIZE;
#ifdef VERBOSE_THREAD
  printf("[DEBUG->task_thread:%2d] items:%d\r\n", p->task_id, items);
  printf("[DEBUG->task_thread:%2d] sizeIn1:%d, 0x%x\r\n", p->task_id, sizeIn1, sizeIn1);
  printf("[DEBUG->task_thread:%2d] sizeIn1:%d, 0x%x\r\n", p->task_id, sizeIn2, sizeIn2);
  printf("[DEBUG->task_thread:%2d] sizeIn1:%d, 0x%x\r\n", p->task_id, sizeOut, sizeOut);
  // printf("[DEBUG->task_thread:%2d] iter:%d\r\n", p->task_id, iter);
  // printf("[DEBUG->task_thread:%2d] reminder:%d\r\n", p->task_id, reminder);
#endif

  err =   vnew(VM, nPR);                                                        errCheck(err, FUN_VNEW);
  err =   vlpr(VM, nPR->at(0),  VMUL);                                          errCheck(err, FUN_VLPR);
  err = vtieio(VM, nPR->at(0),  In1,  In2,  Out, items);                        errCheck(err, FUN_VTIEIO);
  err = vstart(VM, nPR, sizeIn1, sizeIn2, sizeOut);                             errCheck(err, FUN_VSTART);
  err =   vdel(VM, nPR);                                                        errCheck(err, FUN_VDEL);

  // if (items >= MAX_SIZE) {
  //   sizeIn1  = MAX_SIZE;
  //   sizeIn2  = MAX_SIZE;
  //   sizeOut  = MAX_SIZE;
  // }
  // else {
  //   sizeIn1  = p->SizeIn1;
  //   sizeIn2  = p->SizeIn2;
  //   sizeOut  = p->SizeOut;
  // }

  // err =   vnew(VM, nPR);                                                        errCheck(err, FUN_VNEW);
  // err =   vlpr(VM, nPR->at(0),  VMUL);                                          errCheck(err, FUN_VLPR);
  // err = vtieio(VM, nPR->at(0),  In1,  In2,  Out, sizeIn1);                      errCheck(err, FUN_VTIEIO);

  // for (int i = 0; i < iter; i++) {
  //   err = vstart(VM, nPR, MAX_SIZE, MAX_SIZE, MAX_SIZE);                        errCheck(err, FUN_VSTART);
  // }

  // if (reminder != 0) {
  //   err = vtieio(VM, nPR->at(0),  In1,  In2,  Out, reminder);                   errCheck(err, FUN_VTIEIO);
  //   err = vstart(VM, nPR, reminder, reminder, reminder);                        errCheck(err, FUN_VSTART);
  // }
  // err =   vdel(VM, nPR);                                                        errCheck(err, FUN_VDEL);


#ifdef VERBOSE_THREAD
  printf("[DEBUG->task_thread] task thread done\r\n");
#endif
  return NULL;
}

int main(int argc, char* argv[])
{
  printf("Begin...\r\n");
  setlocale(LC_NUMERIC, ""); // for thounds seperator
  printf("%'d\r\n", SIZE);

  int *A = new int[SWSIZE];
  int *B = new int[SWSIZE];
  int *C = new int[SWSIZE];
  int *D = new int[SWSIZE];

  int i;
  int j;
  int sum;

  for (i = 0; i < SWSIZE; i++) {
    A[i] = i + 1;
    B[i] = i + 1;
    C[i] = 0;
    D[i] = 0;
  }
  //////////////////////////////////////////////////////////////////////////////
  struct timeval start, end;
  gettimeofday(&start, NULL);
  for (i = 0; i < SWSIZE; i++) {
    C[i] = A[i] * B[i];
  }
  gettimeofday(&end, NULL);

  int timeuse = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
  printf("CPU %'4d threads :\t%'9d us\r\n", 1, timeuse);

  // pe_t package[THREADS];
  // for (i = 0; i < THREADS; i++) {
  //   package[i].key = i;
  //   package[i].len = SIZE / THREADS;
  //   // int alignment_buf
  //   package[i].A = &A[i * package[i].len];
  //   package[i].B = &B[i * package[i].len];
  //   package[i].C = &D[i * package[i].len];
  // }
  //////////////////////////////////////////////////////////////////////////////
  int err;
  vam_vm_t VM;
  VM.VAM_TABLE = NULL;
  VM.BITSTREAM_TABLE = NULL;
  VAM_VM_INIT(&VM, argc, argv);
  // VAM_TABLE_SHOW(VM);

  {
    int THREADS = 16;
    pthread_t thread[THREADS];
    task_pk_t task_pkg[THREADS];

    vector<vector<int> > nPR(THREADS);
    for (i = 0; i < THREADS; i++) {
      nPR[i].resize(STEPS);
    }

    for (i = 0; i < THREADS; i++) {
      task_pkg[i].task_id = i;
      task_pkg[i].VM      = &VM;
      task_pkg[i].nPR     = &nPR[i];
      task_pkg[i].len     = SIZE / THREADS;
      task_pkg[i].In1     = &A[i * task_pkg[i].len];
      task_pkg[i].SizeIn1 = task_pkg[i].len;
      task_pkg[i].In2     = &B[i * task_pkg[i].len];
      task_pkg[i].SizeIn2 = task_pkg[i].len;
      task_pkg[i].Out     = &D[i * task_pkg[i].len];
      task_pkg[i].SizeOut = task_pkg[i].len;
    }

    gettimeofday(&start, NULL);
    for (i = 0; i < THREADS; i++) {
      pthread_create(&thread[i], NULL, VMUL_Threads_Call, (void *)&task_pkg[i]);
    }
    for (i = 0; i < THREADS; i++) {
      pthread_join(thread[i], NULL);
    }
    gettimeofday(&end, NULL);
    timeuse = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
    printf("JIT %'4d threads :\t%'9d us\t", THREADS, timeuse);

    for (i = 0; i < SWSIZE; i++) {
      // printf("%d:\tA:%d\tB:%d\tC:%d\tD:%d\r\n", i, A[i], B[i], C[i], D[i]);
      if (C[i] != D[i]) {
        printf("Error at %d:\tA:%d\tB:%d\tC:%d\tD:%d\r\nFailed!\r\n", i, A[i], B[i], C[i], D[i]);
        VAM_VM_CLEAN(&VM);
        delete[] A;
        delete[] B;
        delete[] C;
        delete[] D;
        exit(1);
      }
    }
    printf("Passed!\r\n");
  }

  {
    int THREADS = 8;
    pthread_t thread[THREADS];
    task_pk_t task_pkg[THREADS];

    vector<vector<int> > nPR(THREADS);
    for (i = 0; i < THREADS; i++) {
      nPR[i].resize(STEPS);
    }

    for (i = 0; i < THREADS; i++) {
      task_pkg[i].task_id = i;
      task_pkg[i].VM      = &VM;
      task_pkg[i].nPR     = &nPR[i];
      task_pkg[i].len     = SIZE / THREADS;
      task_pkg[i].In1     = &A[i * task_pkg[i].len];
      task_pkg[i].SizeIn1 = task_pkg[i].len;
      task_pkg[i].In2     = &B[i * task_pkg[i].len];
      task_pkg[i].SizeIn2 = task_pkg[i].len;
      task_pkg[i].Out     = &D[i * task_pkg[i].len];
      task_pkg[i].SizeOut = task_pkg[i].len;
    }

    gettimeofday(&start, NULL);
    for (i = 0; i < THREADS; i++) {
      pthread_create(&thread[i], NULL, VMUL_Threads_Call, (void *)&task_pkg[i]);
    }
    for (i = 0; i < THREADS; i++) {
      pthread_join(thread[i], NULL);
    }
    gettimeofday(&end, NULL);
    timeuse = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
    printf("JIT %'4d threads :\t%'9d us\t", THREADS, timeuse);

    for (i = 0; i < SWSIZE; i++) {
      // printf("%d:\tA:%d\tB:%d\tC:%d\tD:%d\r\n", i, A[i], B[i], C[i], D[i]);
      if (C[i] != D[i]) {
        printf("Error at %d:\tA:%d\tB:%d\tC:%d\tD:%d\r\nFailed!\r\n", i, A[i], B[i], C[i], D[i]);
        VAM_VM_CLEAN(&VM);
        delete[] A;
        delete[] B;
        delete[] C;
        delete[] D;
        exit(1);
      }
    }
    printf("Passed!\r\n");
  }

  {
    int THREADS = 4;
    pthread_t thread[THREADS];
    task_pk_t task_pkg[THREADS];

    vector<vector<int> > nPR(THREADS);
    for (i = 0; i < THREADS; i++) {
      nPR[i].resize(STEPS);
    }

    for (i = 0; i < THREADS; i++) {
      task_pkg[i].task_id = i;
      task_pkg[i].VM      = &VM;
      task_pkg[i].nPR     = &nPR[i];
      task_pkg[i].len     = SIZE / THREADS;
      task_pkg[i].In1     = &A[i * task_pkg[i].len];
      task_pkg[i].SizeIn1 = task_pkg[i].len;
      task_pkg[i].In2     = &B[i * task_pkg[i].len];
      task_pkg[i].SizeIn2 = task_pkg[i].len;
      task_pkg[i].Out     = &D[i * task_pkg[i].len];
      task_pkg[i].SizeOut = task_pkg[i].len;
    }

    gettimeofday(&start, NULL);
    for (i = 0; i < THREADS; i++) {
      pthread_create(&thread[i], NULL, VMUL_Threads_Call, (void *)&task_pkg[i]);
    }
    for (i = 0; i < THREADS; i++) {
      pthread_join(thread[i], NULL);
    }
    gettimeofday(&end, NULL);
    timeuse = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
    printf("JIT %'4d threads :\t%'9d us\t", THREADS, timeuse);

    for (i = 0; i < SWSIZE; i++) {
      // printf("%d:\tA:%d\tB:%d\tC:%d\tD:%d\r\n", i, A[i], B[i], C[i], D[i]);
      if (C[i] != D[i]) {
        printf("Error at %d:\tA:%d\tB:%d\tC:%d\tD:%d\r\nFailed!\r\n", i, A[i], B[i], C[i], D[i]);
        VAM_VM_CLEAN(&VM);
        delete[] A;
        delete[] B;
        delete[] C;
        delete[] D;
        exit(1);
      }
    }
    printf("Passed!\r\n");
  }

  {
    int THREADS = 2;
    pthread_t thread[THREADS];
    task_pk_t task_pkg[THREADS];

    vector<vector<int> > nPR(THREADS);
    for (i = 0; i < THREADS; i++) {
      nPR[i].resize(STEPS);
    }

    for (i = 0; i < THREADS; i++) {
      task_pkg[i].task_id = i;
      task_pkg[i].VM      = &VM;
      task_pkg[i].nPR     = &nPR[i];
      task_pkg[i].len     = SIZE / THREADS;
      task_pkg[i].In1     = &A[i * task_pkg[i].len];
      task_pkg[i].SizeIn1 = task_pkg[i].len;
      task_pkg[i].In2     = &B[i * task_pkg[i].len];
      task_pkg[i].SizeIn2 = task_pkg[i].len;
      task_pkg[i].Out     = &D[i * task_pkg[i].len];
      task_pkg[i].SizeOut = task_pkg[i].len;
    }

    gettimeofday(&start, NULL);
    for (i = 0; i < THREADS; i++) {
      pthread_create(&thread[i], NULL, VMUL_Threads_Call, (void *)&task_pkg[i]);
    }
    for (i = 0; i < THREADS; i++) {
      pthread_join(thread[i], NULL);
    }
    gettimeofday(&end, NULL);
    timeuse = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
    printf("JIT %'4d threads :\t%'9d us\t", THREADS, timeuse);

    for (i = 0; i < SWSIZE; i++) {
      // printf("%d:\tA:%d\tB:%d\tC:%d\tD:%d\r\n", i, A[i], B[i], C[i], D[i]);
      if (C[i] != D[i]) {
        printf("Error at %d:\tA:%d\tB:%d\tC:%d\tD:%d\r\nFailed!\r\n", i, A[i], B[i], C[i], D[i]);
        VAM_VM_CLEAN(&VM);
        delete[] A;
        delete[] B;
        delete[] C;
        delete[] D;
        exit(1);
      }
    }
    printf("Passed!\r\n");
  }

  {
    int THREADS = 1;
    pthread_t thread[THREADS];
    task_pk_t task_pkg[THREADS];

    vector<vector<int> > nPR(THREADS);
    for (i = 0; i < THREADS; i++) {
      nPR[i].resize(STEPS);
    }

    for (i = 0; i < THREADS; i++) {
      task_pkg[i].task_id = i;
      task_pkg[i].VM      = &VM;
      task_pkg[i].nPR     = &nPR[i];
      task_pkg[i].len     = SIZE / THREADS;
      task_pkg[i].In1     = &A[i * task_pkg[i].len];
      task_pkg[i].SizeIn1 = task_pkg[i].len;
      task_pkg[i].In2     = &B[i * task_pkg[i].len];
      task_pkg[i].SizeIn2 = task_pkg[i].len;
      task_pkg[i].Out     = &D[i * task_pkg[i].len];
      task_pkg[i].SizeOut = task_pkg[i].len;
    }

    gettimeofday(&start, NULL);
    for (i = 0; i < THREADS; i++) {
      pthread_create(&thread[i], NULL, VMUL_Threads_Call, (void *)&task_pkg[i]);
    }
    for (i = 0; i < THREADS; i++) {
      pthread_join(thread[i], NULL);
    }
    gettimeofday(&end, NULL);
    timeuse = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
    printf("JIT %'4d threads :\t%'9d us\t", THREADS, timeuse);

    for (i = 0; i < SWSIZE; i++) {
      // printf("%d:\tA:%d\tB:%d\tC:%d\tD:%d\r\n", i, A[i], B[i], C[i], D[i]);
      if (C[i] != D[i]) {
        printf("Error at %d:\tA:%d\tB:%d\tC:%d\tD:%d\r\nFailed!\r\n", i, A[i], B[i], C[i], D[i]);
        VAM_VM_CLEAN(&VM);
        delete[] A;
        delete[] B;
        delete[] C;
        delete[] D;
        exit(1);
      }
    }
    printf("Passed!\r\n");
  }

  VAM_VM_CLEAN(&VM);
  delete[] A;
  delete[] B;
  delete[] C;
  delete[] D;
  return 0;
}

/*
11/13/15, VADD, SIZE: 524880000 = 1.9 GBytes
================================================================================
Begin...
CPU    1 threads :      417,612 us
JIT    1 threads :    2,928,620 us (with PR)
JIT    1 threads :    2,927,495 us
JIT    2 threads :    1,448,748 us
JIT    4 threads :    1,398,468 us <-
JIT    6 threads :    1,413,245 us
JIT    8 threads :    1,413,486 us
JIT   10 threads :    1,407,700 us
JIT   12 threads :    1,405,887 us
Passed!
*/

/*
11/13/15, VMUL, SIZE: 524880000 = 1.9 GBytes
================================================================================
Begin...
CPU    1 threads :      431,807 us
JIT    1 threads :    6,582,495 us (with PR)
JIT    1 threads :    6,579,973 us
JIT    2 threads :    3,216,971 us
JIT    4 threads :    1,646,160 us
JIT    6 threads :    1,471,663 us <-
JIT    8 threads :    1,555,613 us
JIT   10 threads :    1,471,380 us
JIT   12 threads :    1,506,018 us
Passed!
*/

/*
11/13/15, A^2+b^2, SIZE: 524880000 = 1.9 GBytes
================================================================================
Begin...
CPU    1 threads :      417,908 us
JIT    1 threads :    8,658,581 us (with PR)
JIT    1 threads :    8,646,399 us
JIT    2 threads :    4,272,384 us
JIT    4 threads :    2,157,717 us
JIT    6 threads :    1,466,381 us
JIT    8 threads :    1,517,133 us
JIT   10 threads :    1,464,127 us
JIT   12 threads :    1,441,734 us
Passed!
*/

/*
03/31/16, VMUL, SIZE: 524880000 = 1.9 GBytes
================================================================================
Begin...
CPU    1 threads :      419,407 us
JIT    8 threads :   49,452,013 us
0:  A:1 B:1 C:1 D:1
1:  A:2 B:2 C:4 D:4
2:  A:3 B:3 C:9 D:9
3:  A:4 B:4 C:16  D:16
4:  A:5 B:5 C:25  D:25
5:  A:6 B:6 C:36  D:36
6:  A:7 B:7 C:49  D:49
7:  A:8 B:8 C:64  D:64
8:  A:9 B:9 C:81  D:81
9:  A:10  B:10  C:100 D:100
Passed!
*/

/*
04/01/16, VADD, SIZE: 26214400  = 100 MBytes
================================================================================
CPU    1 threads :      24208 us
JIT    1 threads :     296062 us
JIT    2 threads :     167116 us
JIT    4 threads :     127317 us
JIT    8 threads :     153507 us
*/

/*
04/01/16, VMUL, SIZE: 524880000 = 1.9 GBytes
================================================================================
Begin...
524,880,000
CPU    1 threads :    419,488 us
JIT   16 threads :  1,483,885 us  Passed!
JIT    8 threads :  1,551,444 us  Passed!
JIT    4 threads :  1,493,106 us  Passed!
JIT    2 threads :  2,736,046 us  Passed!
JIT    1 threads :  5,603,247 us  Passed!
*/
