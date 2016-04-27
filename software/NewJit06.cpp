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

#define VERBOSE
#define VERBOSE_THREAD
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
#define SIZE 32
// #define SIZE 0x10000
// #define SIZE    (MAX_SIZE + 32) * THREADS
// #define SIZE    65528 * THREADS

#define SWSIZE  SIZE
// #define STEPS   3
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

  int         len;

  int        *PR0_In1;
  int     SizePR0_In1;
  int        *PR0_In2;
  int     SizePR0_In2;
  int        *PR0_Out;
  int     SizePR0_Out;

  int        *PR1_In1;
  int     SizePR1_In1;
  int        *PR1_In2;
  int     SizePR1_In2;
  int        *PR1_Out;
  int     SizePR1_Out;

  int        *PR2_In1;
  int     SizePR2_In1;
  int        *PR2_In2;
  int     SizePR2_In2;
  int        *PR2_Out;
  int     SizePR2_Out;

  int        *PR3_In1;
  int     SizePR3_In1;
  int        *PR3_In2;
  int     SizePR3_In2;
  int        *PR3_Out;
  int     SizePR3_Out;

  int        *PR4_In1;
  int     SizePR4_In1;
  int        *PR4_In2;
  int     SizePR4_In2;
  int        *PR4_Out;
  int     SizePR4_Out;

  int        *PR5_In1;
  int     SizePR5_In1;
  int        *PR5_In2;
  int     SizePR5_In2;
  int        *PR5_Out;
  int     SizePR5_Out;

  int        *PR6_In1;
  int     SizePR6_In1;
  int        *PR6_In2;
  int     SizePR6_In2;
  int        *PR6_Out;
  int     SizePR6_Out;
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
  // vector<int> *nPR0     = p->nPR;
  // vector<int> *nPR1     = p->nPR;
  // vector<int> *nPR2     = p->nPR;

  int         items    = p->len;
  int         err;

  // VADD BBB^
  // err =   vnew(VM, nPR);                                                                                                errCheck(err, FUN_VNEW);
  // err =   vlpr(VM, nPR->at(0), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err = vtieio(VM, nPR->at(0), p->PR0_In1, p->SizePR0_In1, p->PR0_In2, p->SizePR0_In2, p->PR0_Out, p->SizePR0_Out);     errCheck(err, FUN_VTIEIO);
  // err = vstart(VM, nPR);                                                                                                errCheck(err, FUN_VSTART);
  // err =   vdel(VM, nPR);                                                                                                errCheck(err, FUN_VDEL);

  // VADD BBR^ + BBR + RRB^
  // err =   vnew(VM, nPR);                                                                                                errCheck(err, FUN_VNEW);
  // err =   vlpr(VM, nPR->at(0), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err =   vlpr(VM, nPR->at(1), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err =   vlpr(VM, nPR->at(2), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err = vtieio(VM, nPR->at(0), p->PR0_In1, p->SizePR0_In1, p->PR0_In2, p->SizePR0_In2, nPR->at(2), p->SizePR0_Out);     errCheck(err, FUN_VTIEIO);
  // err = vtieio(VM, nPR->at(1), p->PR1_In1, p->SizePR1_In1, p->PR1_In2, p->SizePR1_In2, nPR->at(2), p->SizePR1_Out);     errCheck(err, FUN_VTIEIO);
  // err = vtieio(VM, nPR->at(2), nPR->at(0), p->SizePR2_In1, nPR->at(1), p->SizePR2_In2, p->PR2_Out, p->SizePR2_Out);     errCheck(err, FUN_VTIEIO);
  // err = vstart(VM, nPR);                                                                                                errCheck(err, FUN_VSTART);
  // err =   vdel(VM, nPR);                                                                                                errCheck(err, FUN_VDEL);

  // VADD BBR + RBB^
  // err =   vnew(VM, nPR);                                                                                                errCheck(err, FUN_VNEW);
  // err =   vlpr(VM, nPR->at(0), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err =   vlpr(VM, nPR->at(1), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err = vtieio(VM, nPR->at(0), p->PR0_In1, p->SizePR0_In1, p->PR0_In2, p->SizePR0_In2, nPR->at(1), p->SizePR0_Out);     errCheck(err, FUN_VTIEIO);
  // err = vtieio(VM, nPR->at(1), nPR->at(0), p->SizePR1_In1, p->PR1_In2, p->SizePR1_In2, p->PR1_Out, p->SizePR1_Out);     errCheck(err, FUN_VTIEIO);
  // err = vstart(VM, nPR);                                                                                                errCheck(err, FUN_VSTART);
  // err =   vdel(VM, nPR);                                                                                                errCheck(err, FUN_VDEL);

  // VADD BBR + BRB^
  // err =   vnew(VM, nPR);                                                                                                errCheck(err, FUN_VNEW);
  // err =   vlpr(VM, nPR->at(0), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err =   vlpr(VM, nPR->at(1), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err = vtieio(VM, nPR->at(0), p->PR0_In1, p->SizePR0_In1, p->PR0_In2, p->SizePR0_In2, nPR->at(1), p->SizePR0_Out);     errCheck(err, FUN_VTIEIO);
  // err = vtieio(VM, nPR->at(1), p->PR1_In1, p->SizePR1_In1, nPR->at(0), p->SizePR1_In2, p->PR1_Out, p->SizePR1_Out);     errCheck(err, FUN_VTIEIO);
  // err = vstart(VM, nPR);                                                                                                errCheck(err, FUN_VSTART);
  // err =   vdel(VM, nPR);                                                                                                errCheck(err, FUN_VDEL);

  // VADD BBR + RBR^ + BRB
  // err =   vnew(VM, nPR);                                                                                                errCheck(err, FUN_VNEW);
  // err =   vlpr(VM, nPR->at(0), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err =   vlpr(VM, nPR->at(1), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err =   vlpr(VM, nPR->at(2), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err = vtieio(VM, nPR->at(0), p->PR0_In1, p->SizePR0_In1, p->PR0_In2, p->SizePR0_In2, nPR->at(1), p->SizePR0_Out);     errCheck(err, FUN_VTIEIO);
  // err = vtieio(VM, nPR->at(1), nPR->at(0), p->SizePR1_In1, p->PR1_In2, p->SizePR1_In2, nPR->at(2), p->SizePR1_Out);     errCheck(err, FUN_VTIEIO);
  // err = vtieio(VM, nPR->at(2), p->PR2_In1, p->SizePR2_In1, nPR->at(1), p->SizePR2_In2, p->PR2_Out, p->SizePR2_Out);     errCheck(err, FUN_VTIEIO);
  // err = vstart(VM, nPR);                                                                                                errCheck(err, FUN_VSTART);
  // err =   vdel(VM, nPR);                                                                                                errCheck(err, FUN_VDEL);

  // VADD BBR + BRR^ + BRB
  // err =   vnew(VM, nPR);                                                                                                errCheck(err, FUN_VNEW);
  // err =   vlpr(VM, nPR->at(0), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err =   vlpr(VM, nPR->at(1), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err =   vlpr(VM, nPR->at(2), VADD);                                                                                   errCheck(err, FUN_VLPR);
  // err = vtieio(VM, nPR->at(0), p->PR0_In1, p->SizePR0_In1, p->PR0_In2, p->SizePR0_In2, nPR->at(1), p->SizePR0_Out);     errCheck(err, FUN_VTIEIO);
  // err = vtieio(VM, nPR->at(1), p->PR1_In1, p->SizePR1_In1, nPR->at(0), p->SizePR1_In2, nPR->at(2), p->SizePR1_Out);     errCheck(err, FUN_VTIEIO);
  // err = vtieio(VM, nPR->at(2), p->PR2_In1, p->SizePR2_In1, nPR->at(1), p->SizePR2_In2, p->PR2_Out, p->SizePR2_Out);     errCheck(err, FUN_VTIEIO);
  // err = vstart(VM, nPR);                                                                                                errCheck(err, FUN_VSTART);
  // err =   vdel(VM, nPR);                                                                                                errCheck(err, FUN_VDEL);

  // VADD BBR + BBR + RRR^ + RBB
  err =   vnew(VM, nPR);                                                                                                errCheck(err, FUN_VNEW);
  err =   vlpr(VM, nPR->at(0), VADD);                                                                                   errCheck(err, FUN_VLPR);
  err =   vlpr(VM, nPR->at(1), VADD);                                                                                   errCheck(err, FUN_VLPR);
  err =   vlpr(VM, nPR->at(2), VADD);                                                                                   errCheck(err, FUN_VLPR);
  err =   vlpr(VM, nPR->at(3), VADD);                                                                                   errCheck(err, FUN_VLPR);
  err = vtieio(VM, nPR->at(0), p->PR0_In1, p->SizePR0_In1, p->PR0_In2, p->SizePR0_In2, nPR->at(2), p->SizePR0_Out);     errCheck(err, FUN_VTIEIO);
  err = vtieio(VM, nPR->at(1), p->PR1_In1, p->SizePR1_In1, p->PR1_In2, p->SizePR1_In2, nPR->at(2), p->SizePR1_Out);     errCheck(err, FUN_VTIEIO);
  err = vtieio(VM, nPR->at(2), nPR->at(0), p->SizePR2_In1, nPR->at(1), p->SizePR2_In2, nPR->at(3), p->SizePR2_Out);     errCheck(err, FUN_VTIEIO);
  err = vtieio(VM, nPR->at(3), nPR->at(2), p->SizePR3_In1, p->PR3_In2, p->SizePR3_In2, p->PR3_Out, p->SizePR3_Out);     errCheck(err, FUN_VTIEIO);
  err = vstart(VM, nPR);                                                                                                errCheck(err, FUN_VSTART);
  err =   vdel(VM, nPR);                                                                                                errCheck(err, FUN_VDEL);

#ifdef VERBOSE_THREAD
  printf("[DEBUG->task_thread] task thread done\r\n");
#endif
  return NULL;
}

void InsertionSort(int array[], int len)
{
  // L1:
  int i, j, index;
  for (i = 1; i < len; i++)
  {
    // L2:
    index = array[i];
    j = i;
    while ( (j > 0) && (array[j-1] > index) ) {
      // L3:
      array[j] = array[j - 1];
      j--;
    }
    array[j] = index;
  }
}

// Function to Merge Arrays L and R into A.
// lefCount = number of elements in L
// rightCount = number of elements in R.
void Merge(int *A,int *L,int leftCount,int *R,int rightCount) {
  int i,j,k;

  // i - to mark the index of left aubarray (L)
  // j - to mark the index of right sub-raay (R)
  // k - to mark the index of merged subarray (A)
  i = 0; j = 0; k =0;

  while(i<leftCount && j< rightCount) {
    if(L[i]  < R[j]) A[k++] = L[i++];
    else A[k++] = R[j++];
  }
  while(i < leftCount) A[k++] = L[i++];
  while(j < rightCount) A[k++] = R[j++];
}

// Recursive function to sort an array of integers.
void MergeSort(int *A, int n) {
  int mid,i, *L, *R;
  if(n < 2) return; // base condition. If the array has less than two element, do nothing.

  mid = n/2;  // find the mid index.

  // create left and right subarrays
  // mid elements (from index 0 till mid-1) should be part of left sub-array
  // and (n-mid) elements (from mid to n-1) will be part of right sub-array
  L = (int*)malloc(mid*sizeof(int));
  R = (int*)malloc((n- mid)*sizeof(int));

  for(i = 0;i<mid;i++) L[i] = A[i]; // creating left subarray
  for(i = mid;i<n;i++) R[i-mid] = A[i]; // creating right subarray

  MergeSort(L,mid);  // sorting the left subarray
  MergeSort(R,n-mid);  // sorting the right subarray
  Merge(A,L,mid,R,n-mid);  // Merging L and R into A as sorted list.
  free(L);
  free(R);
}

int main(int argc, char* argv[])
{
  printf("Begin...\r\n");
  setlocale(LC_NUMERIC, ""); // for thounds seperator
  printf("%'d\r\n", SIZE);

  struct timeval start, end;
  int timeuse;

  int *A = new int[SWSIZE * 2 * 2 * 2];
  int *B = new int[SWSIZE * 2 * 2 * 2];
  int *C = new int[SWSIZE * 2 * 2 * 2];
  int *D = new int[SWSIZE * 2 * 2 * 2];
  int *E = new int[SWSIZE * 2 * 2 * 2];
  int *F = new int[SWSIZE * 2 * 2 * 2];

  int i;
  int j;
  int sum;

  for (i = 0; i < SWSIZE; i++) {
    A[i] = -2;
    B[i] = -2;
    C[i] = -2;
    D[i] = -2;
    E[i] = -2;
    F[i] = -2;
  }

  for (i = 0; i < SWSIZE; i++) {
    A[i] = i + 0;
    B[i] = i + 1;
    C[i] = i + 2;
    D[i] = i + 3;
  }


  //////////////////////////////////////////////////////////////////////////////
  gettimeofday(&start, NULL);
  InsertionSort(F, SWSIZE * 4);
  for (i = 0; i < SWSIZE; i++) {
    // F[i] = A[i] + B[i];
    // F[i] = (A[i] + B[i]) + (C[i] + D[i]);
    // F[i] = A[i] + B[i] + C[i];
    // F[i] = A[i] + B[i] + C[i] + D[i];
    F[i] = (A[i] + A[i]) + (B[i] + B[i]) + C[i];
  }
  gettimeofday(&end, NULL);
  timeuse = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
  printf("CPU %'4d threads :\t%'9d us\r\n", 1, timeuse);
  //////////////////////////////////////////////////////////////////////////////
  int err;
  vam_vm_t VM;
  VM.VAM_TABLE = NULL;
  VM.BITSTREAM_TABLE = NULL;
  VAM_VM_INIT(&VM, argc, argv);
  // VAM_TABLE_SHOW(VM);



  {
    int THREADS = 1;
    int STEPS   = 4;
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

      task_pkg[i].PR0_In1      = A;
      task_pkg[i].SizePR0_In1  = task_pkg[i].len;
      task_pkg[i].PR0_In2      = A;
      task_pkg[i].SizePR0_In2  = task_pkg[i].len;
      task_pkg[i].PR0_Out      = NULL;
      task_pkg[i].SizePR0_Out  = 0;

      task_pkg[i].PR1_In1      = B;
      task_pkg[i].SizePR1_In1  = task_pkg[i].len;
      task_pkg[i].PR1_In2      = B;
      task_pkg[i].SizePR1_In2  = task_pkg[i].len;
      task_pkg[i].PR1_Out      = NULL;
      task_pkg[i].SizePR1_Out  = 0;

      task_pkg[i].PR2_In1      = NULL;
      task_pkg[i].SizePR2_In1  = 0;
      task_pkg[i].PR2_In2      = NULL;
      task_pkg[i].SizePR2_In2  = 0;
      task_pkg[i].PR2_Out      = NULL;
      task_pkg[i].SizePR2_Out  = 0;

      task_pkg[i].PR3_In1      = NULL;
      task_pkg[i].SizePR3_In1  = 0;
      task_pkg[i].PR3_In2      = C;
      task_pkg[i].SizePR3_In2  = task_pkg[i].len;
      task_pkg[i].PR3_Out      = E;
      task_pkg[i].SizePR3_Out  = task_pkg[i].len;
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

    printf("\r\n");
    for (i = 0; i < SWSIZE; i++) {
      printf("%3d:\tA:%10d\tB:%10d\tC:%10d\tD:%10d\tE:%10d\tF:%10d\r\n", i, A[i], B[i], C[i], D[i], E[i], F[i]);
      if (E[i] != F[i]) {
        printf("Error at %d:\tA:%d\tB:%d\tC:%d\tD:%d\r\nFailed!\r\n", i, A[i], B[i], C[i], D[i]);
        VAM_VM_CLEAN(&VM);
        delete[] A;
        delete[] B;
        delete[] C;
        delete[] D;
        delete[] E;
        delete[] F;
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
  delete[] E;
  delete[] F;
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




      // task_pkg[i].In1     =
      // task_pkg[i].SizeIn1 =
      // task_pkg[i].In2     = &B[i * task_pkg[i].len];
      // task_pkg[i].SizeIn2 = task_pkg[i].len;
      // // task_pkg[i].In2     = NULL; //&B[i * task_pkg[i].len];
      // // task_pkg[i].SizeIn2 = 0;    //task_pkg[i].len;
      // task_pkg[i].Out     = &D[i * task_pkg[i].len];
      // task_pkg[i].SizeOut = task_pkg[i].len * 2;

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



  // pe_t package[THREADS];
  // for (i = 0; i < THREADS; i++) {
  //   package[i].key = i;
  //   package[i].len = SIZE / THREADS;
  //   // int alignment_buf
  //   package[i].A = &A[i * package[i].len];
  //   package[i].B = &B[i * package[i].len];
  //   package[i].C = &D[i * package[i].len];
  // }

  // {
  //   int THREADS = 16;
  //   pthread_t thread[THREADS];
  //   task_pk_t task_pkg[THREADS];

  //   vector<vector<int> > nPR(THREADS);
  //   for (i = 0; i < THREADS; i++) {
  //     nPR[i].resize(STEPS);
  //   }

  //   for (i = 0; i < THREADS; i++) {
  //     task_pkg[i].task_id = i;
  //     task_pkg[i].VM      = &VM;
  //     task_pkg[i].nPR     = &nPR[i];
  //     task_pkg[i].len     = SIZE / THREADS;
  //     task_pkg[i].In1     = &A[i * task_pkg[i].len];
  //     task_pkg[i].SizeIn1 = task_pkg[i].len;
  //     task_pkg[i].In2     = &B[i * task_pkg[i].len];
  //     task_pkg[i].SizeIn2 = task_pkg[i].len;
  //     task_pkg[i].Out     = &D[i * task_pkg[i].len];
  //     task_pkg[i].SizeOut = task_pkg[i].len;
  //   }

  //   gettimeofday(&start, NULL);
  //   for (i = 0; i < THREADS; i++) {
  //     pthread_create(&thread[i], NULL, VMUL_Threads_Call, (void *)&task_pkg[i]);
  //   }
  //   for (i = 0; i < THREADS; i++) {
  //     pthread_join(thread[i], NULL);
  //   }
  //   gettimeofday(&end, NULL);
  //   timeuse = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
  //   printf("JIT %'4d threads :\t%'9d us\t", THREADS, timeuse);

  //   for (i = 0; i < SWSIZE; i++) {
  //     // printf("%d:\tA:%d\tB:%d\tC:%d\tD:%d\r\n", i, A[i], B[i], C[i], D[i]);
  //     if (C[i] != D[i]) {
  //       printf("Error at %d:\tA:%d\tB:%d\tC:%d\tD:%d\r\nFailed!\r\n", i, A[i], B[i], C[i], D[i]);
  //       VAM_VM_CLEAN(&VM);
  //       delete[] A;
  //       delete[] B;
  //       delete[] C;
  //       delete[] D;
  //       exit(1);
  //     }
  //   }
  //   printf("Passed!\r\n");
  // }

  // {
  //   int THREADS = 8;
  //   pthread_t thread[THREADS];
  //   task_pk_t task_pkg[THREADS];

  //   vector<vector<int> > nPR(THREADS);
  //   for (i = 0; i < THREADS; i++) {
  //     nPR[i].resize(STEPS);
  //   }

  //   for (i = 0; i < THREADS; i++) {
  //     task_pkg[i].task_id = i;
  //     task_pkg[i].VM      = &VM;
  //     task_pkg[i].nPR     = &nPR[i];
  //     task_pkg[i].len     = SIZE / THREADS;
  //     task_pkg[i].In1     = &A[i * task_pkg[i].len];
  //     task_pkg[i].SizeIn1 = task_pkg[i].len;
  //     task_pkg[i].In2     = &B[i * task_pkg[i].len];
  //     task_pkg[i].SizeIn2 = task_pkg[i].len;
  //     task_pkg[i].Out     = &D[i * task_pkg[i].len];
  //     task_pkg[i].SizeOut = task_pkg[i].len;
  //   }

  //   gettimeofday(&start, NULL);
  //   for (i = 0; i < THREADS; i++) {
  //     pthread_create(&thread[i], NULL, VMUL_Threads_Call, (void *)&task_pkg[i]);
  //   }
  //   for (i = 0; i < THREADS; i++) {
  //     pthread_join(thread[i], NULL);
  //   }
  //   gettimeofday(&end, NULL);
  //   timeuse = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
  //   printf("JIT %'4d threads :\t%'9d us\t", THREADS, timeuse);

  //   for (i = 0; i < SWSIZE; i++) {
  //     // printf("%d:\tA:%d\tB:%d\tC:%d\tD:%d\r\n", i, A[i], B[i], C[i], D[i]);
  //     if (C[i] != D[i]) {
  //       printf("Error at %d:\tA:%d\tB:%d\tC:%d\tD:%d\r\nFailed!\r\n", i, A[i], B[i], C[i], D[i]);
  //       VAM_VM_CLEAN(&VM);
  //       delete[] A;
  //       delete[] B;
  //       delete[] C;
  //       delete[] D;
  //       exit(1);
  //     }
  //   }
  //   printf("Passed!\r\n");
  // }

  // {
  //   int THREADS = 4;
  //   pthread_t thread[THREADS];
  //   task_pk_t task_pkg[THREADS];

  //   vector<vector<int> > nPR(THREADS);
  //   for (i = 0; i < THREADS; i++) {
  //     nPR[i].resize(STEPS);
  //   }

  //   for (i = 0; i < THREADS; i++) {
  //     task_pkg[i].task_id = i;
  //     task_pkg[i].VM      = &VM;
  //     task_pkg[i].nPR     = &nPR[i];
  //     task_pkg[i].len     = SIZE / THREADS;
  //     task_pkg[i].In1     = &A[i * task_pkg[i].len];
  //     task_pkg[i].SizeIn1 = task_pkg[i].len;
  //     task_pkg[i].In2     = &B[i * task_pkg[i].len];
  //     task_pkg[i].SizeIn2 = task_pkg[i].len;
  //     task_pkg[i].Out     = &D[i * task_pkg[i].len];
  //     task_pkg[i].SizeOut = task_pkg[i].len;
  //   }

  //   gettimeofday(&start, NULL);
  //   for (i = 0; i < THREADS; i++) {
  //     pthread_create(&thread[i], NULL, VMUL_Threads_Call, (void *)&task_pkg[i]);
  //   }
  //   for (i = 0; i < THREADS; i++) {
  //     pthread_join(thread[i], NULL);
  //   }
  //   gettimeofday(&end, NULL);
  //   timeuse = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
  //   printf("JIT %'4d threads :\t%'9d us\t", THREADS, timeuse);

  //   for (i = 0; i < SWSIZE; i++) {
  //     // printf("%d:\tA:%d\tB:%d\tC:%d\tD:%d\r\n", i, A[i], B[i], C[i], D[i]);
  //     if (C[i] != D[i]) {
  //       printf("Error at %d:\tA:%d\tB:%d\tC:%d\tD:%d\r\nFailed!\r\n", i, A[i], B[i], C[i], D[i]);
  //       VAM_VM_CLEAN(&VM);
  //       delete[] A;
  //       delete[] B;
  //       delete[] C;
  //       delete[] D;
  //       exit(1);
  //     }
  //   }
  //   printf("Passed!\r\n");
  // }

  // {
  //   int THREADS = 2;
  //   pthread_t thread[THREADS];
  //   task_pk_t task_pkg[THREADS];

  //   vector<vector<int> > nPR(THREADS);
  //   for (i = 0; i < THREADS; i++) {
  //     nPR[i].resize(STEPS);
  //   }

  //   for (i = 0; i < THREADS; i++) {
  //     task_pkg[i].task_id = i;
  //     task_pkg[i].VM      = &VM;
  //     task_pkg[i].nPR     = &nPR[i];
  //     task_pkg[i].len     = SIZE / THREADS;
  //     task_pkg[i].In1     = &A[i * task_pkg[i].len];
  //     task_pkg[i].SizeIn1 = task_pkg[i].len;
  //     task_pkg[i].In2     = &B[i * task_pkg[i].len];
  //     task_pkg[i].SizeIn2 = task_pkg[i].len;
  //     task_pkg[i].Out     = &D[i * task_pkg[i].len];
  //     task_pkg[i].SizeOut = task_pkg[i].len;
  //   }

  //   gettimeofday(&start, NULL);
  //   for (i = 0; i < THREADS; i++) {
  //     pthread_create(&thread[i], NULL, VMUL_Threads_Call, (void *)&task_pkg[i]);
  //   }
  //   for (i = 0; i < THREADS; i++) {
  //     pthread_join(thread[i], NULL);
  //   }
  //   gettimeofday(&end, NULL);
  //   timeuse = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
  //   printf("JIT %'4d threads :\t%'9d us\t", THREADS, timeuse);

  //   for (i = 0; i < SWSIZE; i++) {
  //     // printf("%d:\tA:%d\tB:%d\tC:%d\tD:%d\r\n", i, A[i], B[i], C[i], D[i]);
  //     if (C[i] != D[i]) {
  //       printf("Error at %d:\tA:%d\tB:%d\tC:%d\tD:%d\r\nFailed!\r\n", i, A[i], B[i], C[i], D[i]);
  //       VAM_VM_CLEAN(&VM);
  //       delete[] A;
  //       delete[] B;
  //       delete[] C;
  //       delete[] D;
  //       exit(1);
  //     }
  //   }
  //   printf("Passed!\r\n");
  // }
