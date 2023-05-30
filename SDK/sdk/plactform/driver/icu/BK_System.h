/*************************************************************
 * @file        BK_System.h
 * @brief       Header file of BK_System.c
 * @author      GuWenFu
 * @version     V1.0
 * @date        2016-09-29
 * @par         
 * @attention   
 *
 * @history     2016-09-29 gwf    create this file
 */

#ifndef  __BK_SYSTEM_H__

#define  __BK_SYSTEM_H__


#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#include "config.h"


#ifndef NULL
#define NULL 0
#endif


#define MAX(a,b)    (((a) > (b)) ? (a) : (b))
#define MIN(a,b)    (((a) < (b)) ? (a) : (b))


#define NUMBER_ROUND_UP(a,b)        ((a) / (b) + (((a) % (b)) ? 1 : 0))
#define NUMBER_ROUND_DOWN(a,b)      ((a) / (b))


//#define RAM_CODE       __attribute__((section("ram_code")))
#define RAM_CODE



#if 1

typedef unsigned char       BYTE;
typedef signed   char       int8;       // �з���8λ���ͱ���
typedef signed   short      int16;      // �з���16λ���ͱ���
typedef signed   long       int32;      // �з���32λ���ͱ���
typedef signed   long long  int64;      // �з���64λ���ͱ���
typedef unsigned char       uint32; 
typedef signed   long long  int64;      // �з���64λ���ͱ���
typedef unsigned char       uint8;      // �޷���8λ���ͱ���
typedef unsigned short      uint16; 

     // �޷���32λ���ͱ���
typedef unsigned long long  uint64;     // �޷���64λ���ͱ���
typedef float               fp32;       // �����ȸ�����(32λ����)
typedef double              fp64;       // ˫���ȸ�����(64λ����)

typedef signed   char       int8_t;     // �з���8λ���ͱ���
typedef signed   short      int16_t;    // �з���16λ���ͱ���
typedef signed   long       int32_t;    // �з���32λ���ͱ���
typedef signed   long long  int64_t;    // �з���64λ���ͱ���
typedef unsigned char       uint8_t;    // �޷���8λ���ͱ���
typedef unsigned short      uint16_t;   // �޷���16λ���ͱ���
typedef unsigned long       uint32_t;   // �޷���32λ���ͱ���
typedef unsigned long long  uint64_t;   // �޷���64λ���ͱ���
typedef float               fp32_t;     // �����ȸ�����(32λ����)
typedef double              fp64_t;     // ˫���ȸ�����(64λ����)

typedef signed   char       s8;
typedef signed   short      s16;
typedef signed   long       s32;
typedef unsigned char       u8;
typedef unsigned short      u16;
typedef unsigned long       u32;
typedef unsigned long       u_int32;

typedef unsigned char  		UINT8;      /* Unsigned  8 bit quantity        */
typedef signed   char  		INT8;       /* Signed    8 bit quantity        */
typedef unsigned short 		UINT16;     /* Unsigned 16 bit quantity        */
typedef signed   short 		INT16;      /* Signed   16 bit quantity        */
typedef unsigned int   		UINT32;     /* Unsigned 32 bit quantity        */
typedef signed   int   		INT32;      /* Signed   32 bit quantity        */
typedef unsigned long long  UINT64;		/* Unsigned 32 bit quantity        */
typedef signed   long long  INT64;		/* Signed   32 bit quantity        */
typedef float         		FP32;		/* Single precision floating point */
typedef double         		FP64;		/* Double precision floating point */
typedef unsigned int        size_t;
typedef unsigned char       BOOLEAN;

typedef int				    INT;
typedef unsigned int	    UINT;
typedef unsigned char	    BYTE;
typedef short			    SHORT;
typedef unsigned short	    WORD;
typedef unsigned short	    WCHAR;
typedef long			    LONG;
typedef unsigned long	    DWORD;
typedef unsigned long long  QWORD;


#if 0
typedef enum
{
    FALSE=0,
    TRUE=!FALSE
}bool, BOOL;
#endif
/*
typedef enum        // by gwf
{
    OK = 0,
    ERROR = -1
} STATUS;

typedef enum        // by gwf
{
    NO = 0,
    YES = 1
} ASK;
*/

#endif
extern void DelayNops(volatile unsigned long nops);

extern void DelayUS(volatile unsigned long timesUS);
extern void DelayMS(volatile unsigned long timesMS);


#ifdef __cplusplus
}
#endif  /* __cplusplus */


#endif      /* __BK_SYSTEM_H__ */
