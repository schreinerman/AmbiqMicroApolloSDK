/*******************************************************************************
* Copyright (C) 2013-2016, Fujitsu Electronics Europe GmbH or a                *
* subsidiary of Fujitsu Electronics Europe GmbH.  All rights reserved.         *
*                                                                              *
* This software, including source code, documentation and related              *
* materials ("Software"), is owned by Fujitsu Electronics Europe GmbH or       *
* one of its subsidiaries ("Fujitsu").
*                                                                              *
* If no EULA applies, Fujitsu hereby grants you a personal, non-exclusive,     *
* non-transferable license to copy, modify, and compile the                    *
* Software source code solely for use in connection with Fujitsu's             *
* integrated circuit products.  Any reproduction, modification, translation,   *
* compilation, or representation of this Software except as specified          *
* above is prohibited without the express written permission of Fujitsu.       *
*                                                                              *
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO                         *
* WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,                         *
* BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED                                 *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                              *
* PARTICULAR PURPOSE. Fujitsu reserves the right to make                       *
* changes to the Software without notice. Fujitsu does not assume any          *
* liability arising out of the application or use of the Software or any       *
* product or circuit described in the Software. Fujitsu does not               *
* authorize its products for use in any products where a malfunction or        *
* failure of the Fujitsu product may reasonably be expected to result in       *
* significant property damage, injury or death ("High Risk Product"). By       *
* including Fujitsu's product in a High Risk Product, the manufacturer         *
* of such system or application assumes all risk of such use and in doing      *
* so agrees to indemnify Fujitsu against all liability.                        *
*******************************************************************************/
/******************************************************************************/
/** \file ItmSemihost.c
 **
 ** A detailed description is available at 
 ** @link ItmSemihostGroup ITM Semihosting description @endlink
 **
 ** History:
 **   - 2016-15-09  V1.0  Manuel Schreiner   First Version
 **   - 2017-04-04  V1.1  Manuel Schreiner   ITMSEMIHOST_ENABLED added (to be defined in RTE_Device.h)
 **   - 2018-07-06  V1.2  Manuel Schreiner   Updated documentation, 
 **                                          now part of the FEEU ClickBeetle(TM) SW Framework
 **
 *****************************************************************************/
#define __ITMSEMIHOST_C__
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "itmsemihost.h"
#include "mcu.h"
#if ITMSEMIHOST_ENABLED == 1
/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

#if  __CC_ARM
#include <time.h>
#include <rt_misc.h>
// If Keil uVision is used with standard library files like stdio.h and strings.h,
// a special feature by the name of "semihosting" must be considered.
// See http://infocenter.arm.com/help/topic/com.arm.doc.dui0349c/Ciheeaja.html for more information.
// This example is based on C:\Keil\ARM\Startup\Retarget.c
#pragma import(__use_no_semihosting_swi)
struct __FILE { int handle;} ;
FILE __stdout = {1};
FILE __stdin = {0};
FILE __stderr = {2};
#endif

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

static void SendChar(char_t c);
static void SendBuffer(char_t* pcBuffer, uint32_t u32Len);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/**
 ******************************************************************************
 ** \brief Initialization Routine
 **
 ******************************************************************************/
void ItmSemihost_Init(void)
{
    GPIO->PADKEY = 0x00000073;            //unlock pin selection
    GPIO->PADREGK_b.PAD41FNCSEL = 2;      //set SWO on pin 41
    GPIO->PADREGK_b.PAD41INPEN = 0;       //enable input on pin 41
    GPIO->CFGF_b.GPIO41OUTCFG = 1;        //output is push-pull
    GPIO->PADKEY = 0;                     //lock pin selection
    
    CoreDebug->DEMCR |= (1 << CoreDebug_DEMCR_TRCENA_Pos);       //set the TRCENA to use ITM
    ITM->LAR = 0xC5ACCE55;                                       //unlock ITM->TCR

    //set bits in trace enable registers
    ITM->TPR = 0x0000000F;                
    ITM->TER = 0xFFFFFFFF;
    
    // Write to the ITM control and status register (don't enable yet). 
    // ID = 0x15, Prescaler = 1, SWOENA = 1, ITMENA = 1
    ITM->TCR = (0x15 << ITM_TCR_TraceBusID_Pos) | \
               (1 << ITM_TCR_TSPrescale_Pos) | \
               (1 << ITM_TCR_SWOENA_Pos) | \
               (1 << ITM_TCR_ITMENA_Pos); 
    
    // Clear Formatter and Flush Control Register
    TPI->FFCR = 0; 
    
    // Now turn on the TPIU in UART mode at 1.0 MHz based on the HFRC.
    TPI->CSPSR = 1;
    
    //set prescale divider to 2
    TPI->ACPR = 2;     
    
    //UART mode
    TPI->SPPR = 2;     
    
    //Normal mode
    TPI->ITCTRL = 0;   
    
    MCUCTRL->TPIUCTRL = 0x00000200 | 0x00000001; //CLKSEL 3MHz, Enable
}

/**
 ******************************************************************************
 ** \brief Deinitialization Routine
 **
 ******************************************************************************/
void ItmSemihost_Deinit(void)
{

}

/**
 ******************************************************************************
 ** \brief Needed send char routine used later on in the semihosting procedures
 **
 ** \param c   char to send
 **
 ******************************************************************************/   
static void SendChar(char_t c)
{
    ITM_SendChar(c);
}


/**
 ******************************************************************************
 ** \brief Needed send buffer routine used later on in the semihosting procedures
 **
 ** \param pcBuffer  buffer to send
 **
 ** \param u32Len    buffer size
 **
 ******************************************************************************/   
static void SendBuffer(char_t* pcBuffer, uint32_t u32Len)
{
    volatile uint32_t i;
    for(i = 0;i < u32Len;i++)
    {
        ITM_SendChar(pcBuffer[i]);
    }
    
}
      



/**
 ******************************************************************************
 ******************************************************************************
 **
 ** Start of semihosting
 **
 ******************************************************************************
 ******************************************************************************/   



#if  defined(__CC_ARM)
/**
 ******************************************************************************
 ** \brief Low Level for stdio for ARM / Keil µVision
 **
 ******************************************************************************/   

int ferror(FILE *f) {
  /* Your implementation of ferror */
  return 0;
}

void _ttywrch(int c) {
    SendChar((char_t)c);
}

void _sys_exit(int return_code) {
    while(1);  /* endless loop */
}

int fputc(int c, FILE *f) {
    SendChar((char_t)c);
    return 0;
}


int fgetc(FILE *f) {
    char_t c;
    c = 0;
    return (c);
}

#elif defined(__ICCARM__)
/**
 ******************************************************************************
 ** \brief Low Level for stdio for IAR EWARM
 **
 ******************************************************************************/

int putchar(int ch)
{
   SendChar((char_t)ch);
   return ch;
}

int __close(int fileno)
{
    return 0;
}
int __write(int fileno, char *buf, unsigned int size)
{
    SendBuffer(buf,size);
    return 0;
}
int __read(int fileno, char *buf, unsigned int size)
{
    return 0;
}

#elif defined(__GNUC__)

/**
 ******************************************************************************
 ** \brief Low Level for stdio for GNU GCC
 **
 ******************************************************************************/

int _read_r (struct _reent *r, int file, char * ptr, int len)
{
  r = r;
  file = file;
  ptr = ptr;
  len = len;
  
  return 0;
}


int _lseek_r (struct _reent *r, int file, int ptr, int dir)
{
  r = r;
  file = file;
  ptr = ptr;
  dir = dir;
  return 0;
}


int _write (int file, char * ptr, int len)
{  
  file = file;
  ptr = ptr;

  SendBuffer((char*)ptr,(uint32_t)len);
  
  return 0;

}

int _close_r (struct _reent *r, int file)
{
  return 0;
}

/* Register name faking - works in collusion with the linker.  */
register char * stack_ptr asm ("sp");

caddr_t _sbrk_r (struct _reent *r, int incr)
{
  extern char   end asm ("end"); /* Defined by the linker.  */
  static char * heap_end;
  char *        prev_heap_end;

  if (heap_end == NULL)
    heap_end = & end;
  
  prev_heap_end = heap_end;
  
  if (heap_end + incr > stack_ptr)
  {
      /* Some of the libstdc++-v3 tests rely upon detecting
        out of memory errors, so do not abort here.  */
#if 0
      extern void abort (void);

      _write (1, "_sbrk: Heap and stack collision\n", 32);
      
      abort ();
#else
      return (caddr_t) -1;
#endif
  }
  
  heap_end += incr;

  return (caddr_t) prev_heap_end;
}

#endif

#if    HEAP_SIZE
extern  char   *sbrk(int size)
{
   if (brk_siz + size > _heap_size || brk_siz + size < 0)
        return((char*)-1);
   brk_siz += size;
   return( (char *)_heap + brk_siz - size);
}
#endif


#if    HEAP_SIZE
    static   long        brk_siz = 0;
    typedef  int         _heap_t;
    #define ROUNDUP(s)   (((s)+sizeof(_heap_t)-1)&~(sizeof(_heap_t)-1))
    static   _heap_t     _heap[ROUNDUP(HEAP_SIZE)/sizeof(_heap_t)];
    #define              _heap_size       ROUNDUP(HEAP_SIZE)
#else
    extern  char         *_heap;
    extern  long         _heap_size;
#endif
#else
#warning Low-Level-Driver for Apollo 1/2 ITM Semihosting is disabled and could be removed from the project
#endif /* ITMSEMIHOST_ENABLED == 1 */
/**
 ******************************************************************************
 ******************************************************************************
 **
 ** End of semihosting
 **
 ******************************************************************************
 ******************************************************************************/   
    
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
