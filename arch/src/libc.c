/****************************************************************************
 * libc/string/lib_memset.c
 *
 *   Copyright (C) 2007, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void _exit(int status);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

void
__attribute__((noreturn))
__assert_func (
    const char __attribute__((unused)) *file,
    int __attribute__((unused))  line,
    const char __attribute__((unused))  *func,
    const char __attribute__((unused))  *failedexpr)
{
  _exit(0);
  /* NOTREACHED */
}

void abort(void)
{
  _exit(0);
}

void *memcpy(void *dest, const void *src, size_t n)
{
  unsigned char *pout = (unsigned char*)dest;
  unsigned char *pin  = (unsigned char*)src;
  while (n-- > 0) *pout++ = *pin++;
  return dest;
}

void *memset(void *s, int c, size_t n)
{
#ifdef CONFIG_MEMSET_OPTSPEED
  /* This version is optimized for speed (you could do better
   * still by exploiting processor caching or memory burst
   * knowledge.)
   */

  uintptr_t addr  = (uintptr_t)s;
  uint16_t  val16 = ((uint16_t)c << 8) | (uint16_t)c;
  uint32_t  val32 = ((uint32_t)val16 << 16) | (uint32_t)val16;

  /* Make sure that there is something to be cleared */

  if (n > 0)
    {
      /* Align to a 16-bit boundary */

      if ((addr & 1) != 0)
        {
          *(uint8_t*)addr = (uint8_t)c;
          addr += 1;
          n    -= 1;
        }

      /* Check if there are at least 16-bits left to be written */

      if (n >= 2)
        {
          /* Align to a 32-bit boundary (we know that the destination
           * address is already aligned to at least a 16-bit boundary).
           */

          if ((addr & 3) != 0)
            {
              *(uint16_t*)addr = val16;
              addr += 2;
              n    -= 2;
            }

          /* Loop while there are at least 32-bits left to be written */

          while (n >= 4)
            {
              *(uint32_t*)addr = val32;
              addr += 4;
              n    -= 4;
            }
        }

      /* We may get here under the following conditions:
       *
       *   n = 0, addr may or may not be aligned
       *   n = 1, addr is aligned to at least a 16-bit boundary
       *   n = 2, addr is aligned to a 32-bit boundary
       *   n = 3, addr is aligned to a 32-bit boundary
       */

      if (n >= 2)
        {
          *(uint16_t*)addr = val16;
          addr += 2;
          n    -= 2;
        }

      if (n >= 1)
        {
          *(uint8_t*)addr = (uint8_t)c;
        }
    }
#else
  /* This version is optimized for size */

  unsigned char *p = (unsigned char*)s;
  while (n-- > 0) *p++ = c;
#endif
  return s;
}

int strncmp(const char *cs, const char *ct, size_t nb)
{
  int result = 0;
  for (; nb > 0; nb--)
    {
      if ((result = (int)*cs - (int)*ct++) != 0 || !*cs++)
        {
          break;
        }
    }

  return result;
}
