//*****************************************************************************
//
//! @file am_reg_macros.h
//!
//! @brief Helper macros for using hardware registers.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2016, Ambiq Micro
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 1.1.0 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef REG_MACROS_H
#define REG_MACROS_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// High-level Helper Macros.
//
// Usage:
//
// For direct 32-bit access to a register, use AM_REGVAL:
//      AM_REGVAL(REG_VCOMP_BASEADDR + AM_VCOMP_VCMPCFG_O) |= 0xDEADBEEF;
//
// The AM_REG macro can also be used as a shorthand version of AM_REGVAL:
//      AM_REG(VCOMP, VCMPCFG) |= 0xDEADBEEF;
//
// The AM_REGn macro is used for accessing registers of peripherals with
// multiple instances, such as IOMSTR.
//      AM_REGn(IOMSTR, 1, CLKCFG) |= 0xDEADBEEF;
//
// To write to a specific bitfield within a register, use AM_BFW or AM_BFWn:
//      AM_BFW(CTIMER, 0, CTCTRL0, TMRB0FN, 0x3);
//
// To read a field, use AM_BFR or AM_BFRn:
//      ui32Timer0Fn = AM_BFR((CTIMER, 0, CTCTRL0, TMRB0FN);
//
// Note:
//
// AM_REGn, AM_BFW and AM_BFR are concatenation-based, which means that
// standalone macro definitions should not be used for the 'module', 'reg', and
// 'field' arguments.All macro names in the various peripheral header files are
// written in one of the following forms:
//      - AM_REG_##module_reg_O
//      - AM_REG_##module_reg_field_S
//      - AM_REG_##module_reg_field_M
//
// The "module", "reg" and "field" fragments may be used as valid arguments to
// the AM_REGn, AM_BFW, and AM_BFR macros, all of which are able to perform the
// necessary concatenation operations to reconstruct the full macros and look
// up the appropriate base address for the instance number given. For
// peripherals with only one instance, use instance number 0.
//
// The AM_REGVAL macro does not perform any concatenation operations, so the
// complete macro name (including any suffix) must be specified.
//
//*****************************************************************************
#define AM_REGVAL(x)               (*((volatile uint32_t *)(x)))
#define AM_REGVAL_FLOAT(x)         (*((volatile float *)(x)))

//*****************************************************************************
//
// Register access macros for single-instance modules
//
//*****************************************************************************
#define AM_REG(module, reg)                                                   \
    AM_REGn(module, 0, reg)

#define AM_BFW(module, reg, field, value)                                     \
    AM_BFWn(module, 0, reg, field, value)

#define AM_BFWe(module, reg, field, enumval)                                  \
    AM_BFWen(module, 0, reg, field, enumval)

#define AM_BFR(module, reg, field)                                            \
    AM_BFRn(module, 0, reg, field)

#define AM_BFM(module, reg, field)                                            \
    AM_BFMn(module, 0, reg, field)

#define AM_BFV(module, reg, field, value)                                     \
    (((uint32_t)(value) << AM_REG_##module##_##reg##_##field##_S) &           \
     AM_REG_##module##_##reg##_##field##_M)

#define AM_BFX(module, reg, field, value)                                     \
    (((uint32_t)(value) & AM_REG_##module##_##reg##_##field##_M) >>           \
     AM_REG_##module##_##reg##_##field##_S)


//*****************************************************************************
//
// Register access macros for multi-instance modules
//
//*****************************************************************************
#define AM_REGn(module, instance, reg)                                        \
    AM_REGVAL(AM_REG_##module##n(instance) + AM_REG_##module##_##reg##_O)

#define AM_BFWn(module, instance, reg, field, value)                          \
    AM_REGn(module, instance, reg) =                                          \
        (AM_BFV(module, reg, field, value) |                                  \
         (AM_REGn(module, instance, reg) &                                    \
          (~AM_REG_##module##_##reg##_##field##_M)))

#define AM_BFWen(module, instance, reg, field, value)                         \
    AM_BFWn(module, instance, reg, AM_REG_##module##_##reg##_##field##_##enumval)

#define AM_BFRn(module, instance, reg, field)                                 \
    AM_BFX(module, reg, field, AM_REGn(module, instance, reg))

#define AM_BFMn(module, instance, reg, field)                                 \
    (AM_REGn(module, instance, reg) & AM_REG_##module##_##reg##_##field##_M)

//*****************************************************************************
//
// Other helper Macros.
//
// Note: The AM_WRITE_SM and AM_READ_SM macros make use of macro concatenation,
// so the '_S' or '_M' suffix on a register bitfield macro should not be
// supplied by the user. The macro will apply each suffix as needed.
//
//*****************************************************************************

//
// AM_WRITE_SM performs a shift/mask operation to prepare the value 'x' to be
// written to the register field 'field'.
//
// For example:
// AM_REGVAL(ui32Base + AM_VCOMP_VCMP_CFG_O) |=
//     AM_WRITE_SM(AM_VCOMP_VCMP_CFG_LVLSEL, ui32Value);
//
#define AM_WRITE_SM(field, x)      (((x) << field##_S) & field##_M)

//
// AM_READ_SM performs a shift/mask operation to make it easier to interpret
// the value of a given bitfield. This is essentially the reverse of the
// AM_WRITE_SM operation. In most cases, you will want to use the shorter
// AM_BFR macro instead of this one.
//
// For example:
// ui32Value = AM_READ_SM(AM_VCOMP_VCMP_CFG_NSEL,
//                        AM_REGVAL(ui32Base + AM_VCOMP_VCMP_CFG_O));
//
#define AM_READ_SM(field, x)       (((x) & field##_M) >> field##_S)

#ifdef __cplusplus
}
#endif

#endif // REG_MACROS_H

