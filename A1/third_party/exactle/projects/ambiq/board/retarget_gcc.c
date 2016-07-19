/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Re-target system call stubs to satisfy linking to hosted system calls.
 *
 *  $Date: 2014-05-13 22:49:06 -0700 (Tue, 13 May 2014) $
 *  $Revision: 1450 $
 *
 *  Copyright (c) 2013 Wicentric, Inc., all rights reserved.
 *  Wicentric confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact Wicentric, Inc. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include <sys/stat.h>

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Open file.
 *
 *  \param      name        Filename.
 *  \param      flags       Flags.
 *  \param      mode        Open mode.
 *
 *  \return     File handle.
 */
/*************************************************************************************************/
int _open(const char *name, int flags, int mode)
{
  /* No implementation. */
  return -1;
}

/*************************************************************************************************/
/*!
 *  \brief      Close file.
 *
 *  \param      file        File handle.
 *
 *  \return     Result.
 */
/*************************************************************************************************/
int _close(int file)
{
  /* No implementation. */
  return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      File status.
 *
 *  \param      file        File handle.
 *  \param      *st         File status response.
 *
 *  \return     Result.
 */
/*************************************************************************************************/
int _fstat(int file, struct stat *st)
{
  /* No implementation. */
  return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      Get terminal type.
 *
 *  \param      file        File handle.
 *
 *  \return     Result.
 */
/*************************************************************************************************/
int _isatty(int file)
{
  /* No implementation. */
  return 1;
}

/*************************************************************************************************/
/*!
 *  \brief      File seek.
 *
 *  \param      file        File handle.
 *  \param      ptr         Offset.
 *  \param      dir         Direction of offset.
 *
 *  \return     Result.
 */
/*************************************************************************************************/
int _lseek(int file, int ptr, int dir)
{
  /* No implementation. */
  return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      Read from file.
 *
 *  \param      file        File handle.
 *  \param      ptr         Destination buffer.
 *  \param      len         Length to read.
 *
 *  \return     Result.
 */
/*************************************************************************************************/
int _read(int file, char *ptr, int len)
{
  /* No implementation. */
  return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      Write to file.
 *
 *  \param      file        File handle.
 *  \param      ptr         Source buffer.
 *  \param      len         Length to write.
 *
 *  \return     Result.
 */
/*************************************************************************************************/
int _write(int file, char *ptr, int len)
{
  /* No implementation. */
  return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      Change the program's break limit.
 *
 *  \param      incr        Size.
 *
 *  \return     Old break value.
 */
/*************************************************************************************************/
caddr_t _sbrk(int incr)
{
  /* No implementation. */
  return 0;
}
