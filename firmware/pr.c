#include <stdio.h>
#include <stdlib.h>
#include <string.h>



typedef unsigned int u32;
typedef unsigned char u8;

typedef struct
{
    u32  HeaderLength;     /* Length of header in 32 bit words */
    u32 BitstreamLength;  /* Length of bitstream to read in bytes*/
    u8 *DesignName;       /* Design name read from bitstream header */
    u8 *PartName;         /* Part name read from bitstream header */
    u8 *Date;             /* Date read from bitstream header */
    u8 *Time;             /* Bitstream creation time read from header */
    u32 MagicLength;      /* Length of the magic numbers in header */
} XHwIcap_Bit_Header;

#define XHI_EVEN_MAGIC_BYTE     0x0f
#define XHI_ODD_MAGIC_BYTE      0xf0
#define XHI_BIT_HEADER_FAILURE -1
#define XHwIcap_mGetType(Header) ((Header >> XHI_TYPE_SHIFT) & XHI_TYPE_MASK)
#define XHwIcap_mGetOp(Header) ((Header >> XHI_OP_SHIFT) & XHI_OP_MASK)
#define XHwIcap_mGetRegister(Header) ((Header >> XHI_REGISTER_SHIFT) & XHI_REGISTER_MASK)
#define XHwIcap_mGetWordCountType1(Header) (Header & XHI_WORD_COUNT_MASK_TYPE_1)
#define XHwIcap_mGetWordCountType2(Header) (Header & XHI_WORD_COUNT_MASK_TYPE_2)

#define Little2Big32(A) ((((u32)(A) & 0xff000000) >> 24) | (((u32)(A) & 0x00ff0000) >> 8) | \
             (((u32)(A) & 0x0000ff00) << 8) | (((u32)(A) & 0x000000ff) << 24))

XHwIcap_Bit_Header XHwIcap_ReadHeader(u8 *Data, u32 Size);

#define xil_printf printf

int main()
{
  char str_tmp[256] = "const unsigned int ";
  strcat(str_tmp, STR_ACC_NAME);
  strcat(str_tmp, "_bit[] = {");
  // printf("%s\r\n", str_tmp);

  char len[256] = "const unsigned int ";
  strcat(len, STR_ACC_NAME);
  strcat(len, "_bit_len = ");
  // printf("%s\r\n", len);

  int i = 1;
  char *p = (char *) &i;
  if (*p == 1) printf("Little Endia\r\n");
  else printf("Big Endia\r\n");

  XHwIcap_Bit_Header bit_header_sub;
  bit_header_sub = XHwIcap_ReadHeader((u8*)STR_VAR, 0);
  printf("HeaderLength:     %d\r\n", bit_header_sub.HeaderLength);
  printf("BitstreamLength:  %d\r\n", bit_header_sub.BitstreamLength);
  printf("DesignName:       %s\r\n", bit_header_sub.DesignName);
  printf("PartName:         %s\r\n", bit_header_sub.PartName);
  printf("Date:             %s\r\n", bit_header_sub.Date);
  printf("Time:             %s\r\n", bit_header_sub.Time);
  printf("MagicLength:      %d\r\n", bit_header_sub.MagicLength);
  printf("==========================================\r\n");
  // u32 *Buffer = (u32 *)&STR_VAR[bit_header_sub.HeaderLength];
  FILE *file;
  file = fopen(STR_FILE_NAME, "w");
  u32 words = bit_header_sub.BitstreamLength / 4;

  // printf("Bitstream Length : %d bytes, %d words, %x words\r\n", bit_header.BitstreamLength, words, words);
  u8 last = words & 0xf;
  u8 dis = last == 0 ? 0 : 0x10 - last;
  int new_len = words + dis;
  // printf("last : %x dis : %x\r\n", last, dis);
  // printf("Bitstream Length : %d bytes, %d words, %x words\r\n", new_len * 4, new_len, new_len);
  u32 *pr_buffer = (u32 *)malloc(sizeof(u32) * new_len);
  u32 *pr_tmp = (u32 *)&STR_VAR[bit_header_sub.HeaderLength];

  for (i = 0; i < words; i++) {
    if (pr_tmp[i] == 0x00000000) {
      pr_buffer[i] = 0x00000000;
      continue;
    }
    if (pr_tmp[i] == 0xFFFFFFFF) {
      pr_buffer[i] = 0xffffffff;
      continue;
    }
    pr_buffer[i] = Little2Big32(pr_tmp[i]);
  }
  for (i = 0; i < dis; i++) {
    pr_buffer[words + i] = 0xffffffff;
  }

  // s100i change to 32 bits. DOES NON need this revers
  // for (i = 0; i < new_len; i += 4) {
  //   int tmp;
  //   tmp           = pr_buffer[i];
  //   pr_buffer[i]   = pr_buffer[i+3];
  //   pr_buffer[i+3] = tmp;

  //   tmp           = pr_buffer[i+1];
  //   pr_buffer[i+1] = pr_buffer[i+2];
  //   pr_buffer[i+2] = tmp;
  // }

  fprintf(file, "%s %d;\r\n", len, new_len);
  fprintf(file, "%s\r\n", str_tmp);

  for (i = 0; i < new_len; i++) {
    fprintf(file, "  0x%08x", pr_buffer[i]);
    if (i != new_len - 1) {
      fprintf(file, ",\r\n");
    }
    else {
      fprintf(file, "\r\n");
    }
    // printf("0x%08x : 0x%08x\r\n", Buffer[i], Little2Big32(Buffer[i]));
  }
  fprintf(file, "};");
  fclose(file);

  printf("Done!\r\n");
  return 0;
}

XHwIcap_Bit_Header XHwIcap_ReadHeader(u8 *Data, u32 Size)
{
    u32 I;
    u32 Len;
    u32 Tmp;
    XHwIcap_Bit_Header Header;
    u32 Index;

    /* Start Index at start of bitstream */
    Index=0;

    /* Initialize HeaderLength.  If header returned early inidicates
     * failure.
     */
    Header.HeaderLength = XHI_BIT_HEADER_FAILURE;

    /* Get "Magic" length */
    Header.MagicLength = Data[Index++];
    Header.MagicLength = (Header.MagicLength << 8) | Data[Index++];

    /* Read in "magic" */
    for (I=0; I<Header.MagicLength-1; I++) {
        Tmp = Data[Index++];
        if (I%2==0 && Tmp != XHI_EVEN_MAGIC_BYTE)
        {
          return Header;   /* INVALID_FILE_HEADER_ERROR */
        }
        if (I%2==1 && Tmp != XHI_ODD_MAGIC_BYTE)
        {
            return Header;   /* INVALID_FILE_HEADER_ERROR */
        }
    }

    /* Read null end of magic data. */
    Tmp = Data[Index++];

    /* Read 0x01 (short) */
    Tmp = Data[Index++];
    Tmp = (Tmp << 8) | Data[Index++];

    /* Check the "0x01" half word */
    if (Tmp != 0x01)
    {
       return Header;   /* INVALID_FILE_HEADER_ERROR */
    }


    /* Read 'a' */
    Tmp = Data[Index++];
    if (Tmp != 'a')
    {
        return Header;    /* INVALID_FILE_HEADER_ERROR  */
    }

    /* Get Design Name length */
    Len = Data[Index++];
    Len = (Len << 8) | Data[Index++];

    /* allocate space for design name and final null character. */
    Header.DesignName = (u8 *)malloc(Len);

    /* Read in Design Name */
    for (I=0; I<Len; I++)
    {
        Header.DesignName[I] = Data[Index++];
    }

    /* Read 'b' */
    Tmp = Data[Index++];
    if (Tmp != 'b')
    {
        return Header;  /* INVALID_FILE_HEADER_ERROR */
    }

    /* Get Part Name length */
    Len = Data[Index++];
    Len = (Len << 8) | Data[Index++];

    /* allocate space for part name and final null character. */
    Header.PartName = (u8 *)malloc(Len);

    /* Read in part name */
    for (I=0; I<Len; I++)
    {
        Header.PartName[I] = Data[Index++];
    }

    /* Read 'c' */
    Tmp = Data[Index++];
    if (Tmp != 'c')
    {
        return Header;  /* INVALID_FILE_HEADER_ERROR */
    }

    /* Get date length */
    Len = Data[Index++];
    Len = (Len << 8) | Data[Index++];

    /* allocate space for date and final null character. */
    Header.Date = (u8 *)malloc(Len);

    /* Read in date name */
    for (I=0; I<Len; I++)
    {
        Header.Date[I] = Data[Index++];
    }

    /* Read 'd' */
    Tmp = Data[Index++];
    if (Tmp != 'd')
    {
        return Header;  /* INVALID_FILE_HEADER_ERROR  */
    }

    /* Get time length */
    Len = Data[Index++];
    Len = (Len << 8) | Data[Index++];

    /* allocate space for time and final null character. */
    Header.Time = (u8 *)malloc(Len);

    /* Read in time name */
    for (I=0; I<Len; I++)
    {
        Header.Time[I] = Data[Index++];
    }

    /* Read 'e' */
    Tmp = Data[Index++];
    if (Tmp != 'e')
    {
        return Header;  /* INVALID_FILE_HEADER_ERROR */
    }

    /* Get byte length of bitstream */
    Header.BitstreamLength = Data[Index++];
    Header.BitstreamLength = (Header.BitstreamLength << 8) | Data[Index++];
    Header.BitstreamLength = (Header.BitstreamLength << 8) | Data[Index++];
    Header.BitstreamLength = (Header.BitstreamLength << 8) | Data[Index++];

    Header.HeaderLength = Index;

    return Header;
}
