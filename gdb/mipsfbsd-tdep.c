/* Target-dependent code for FreeBSD/mips.

   Copyright (C) 2016 Free Software Foundation, Inc.

   This software was developed by SRI International and the University
   of Cambridge Computer Laboratory under DARPA/AFRL contract
   FA8750-10-C-0237 ("CTSRD"), as part of the DARPA CRASH research
   programme.

   This file is part of GDB.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

#include "defs.h"
#include "osabi.h"
#include "regset.h"
#include "trad-frame.h"
#include "tramp-frame.h"

#include "fbsd-tdep.h"
#include "mips-tdep.h"
#include "mipsfbsd-tdep.h"

#include "solib-svr4.h"

/* Shorthand for some register numbers used below.  */
#define MIPS_PC_REGNUM  MIPS_EMBED_PC_REGNUM
#define MIPS_FP0_REGNUM MIPS_EMBED_FP0_REGNUM
#define MIPS_FSR_REGNUM MIPS_EMBED_FP0_REGNUM + 32

/* Core file support. */

/* Number of registers in `struct reg' from <machine/reg.h>.  The
   first 38 follow the standard MIPS layout.  The 39th holds
   IC_INT_REG on RM7K and RM9K processors.  The 40th is a dummy for
   padding.  */
#define MIPSFBSD_NUM_GREGS	40

/* Number of registers in `struct fpreg' from <machine/reg.h>.  The
   first 32 hold floating point registers.  33 holds the FSR.  The
   34th is a dummy for padding.  */
#define MIPSFBSD_NUM_FPREGS	34

/* Supply a single register.  If the source register size matches the
   size the regcache expects, this can use regcache_raw_supply().  If
   they are different, this copies the source register into a buffer
   that can be passed to regcache_raw_supply().  */

static void
mipsfbsd_supply_reg (struct regcache *regcache, int regnum, const void *addr,
		     size_t len)
{
  struct gdbarch *gdbarch = get_regcache_arch (regcache);

  if (register_size (gdbarch, regnum) == len)
    {
      regcache_raw_supply (regcache, regnum, addr);
    }
  else
    {
      enum bfd_endian byte_order = gdbarch_byte_order (gdbarch);
      gdb_byte buf[MAX_REGISTER_SIZE];
      LONGEST val;

      val = extract_signed_integer ((const gdb_byte *) addr, len, byte_order);
      store_signed_integer (buf, register_size (gdbarch, regnum), byte_order,
			    val);
      regcache_raw_supply (regcache, regnum, buf);
    }
}

/* Collect a single register.  If the destination register size
   matches the size the regcache expects, this can use
   regcache_raw_supply().  If they are different, this fetches the
   register via regcache_raw_supply() into a buffer and then copies it
   into the final destination.  */

static void
mipsfbsd_collect_reg (const struct regcache *regcache, int regnum, void *addr,
		      size_t len)
{
  struct gdbarch *gdbarch = get_regcache_arch (regcache);

  if (register_size (gdbarch, regnum) == len)
    {
      regcache_raw_collect (regcache, regnum, addr);
    }
  else
    {
      enum bfd_endian byte_order = gdbarch_byte_order (gdbarch);
      gdb_byte buf[MAX_REGISTER_SIZE];
      LONGEST val;

      regcache_raw_collect (regcache, regnum, buf);
      val = extract_signed_integer (buf, register_size (gdbarch, regnum),
				    byte_order);
      store_signed_integer ((gdb_byte *) addr, len, byte_order, val);
    }
}

/* Supply the floating-point registers stored in FPREGS to REGCACHE.
   Each floating-point register in FPREGS is REGSIZE bytes in
   length.  */

void
mipsfbsd_supply_fpregs (struct regcache *regcache, int regnum,
			const void *fpregs, size_t regsize)
{
  const char *regs = (const char *) fpregs;
  int i;

  for (i = MIPS_FP0_REGNUM; i <= MIPS_FSR_REGNUM; i++)
    {
      if (regnum == i || regnum == -1)
	mipsfbsd_supply_reg (regcache, i,
			     regs + (i - MIPS_FP0_REGNUM) * regsize, regsize);
    }
}

/* Supply the general-purpose registers stored in GREGS to REGCACHE.
   Each general-purpose register in GREGS is REGSIZE bytes in
   length.  */

void
mipsfbsd_supply_gregs (struct regcache *regcache, int regnum,
		       const void *gregs, size_t regsize)
{
  const char *regs = (const char *) gregs;
  int i;

  for (i = 0; i <= MIPS_PC_REGNUM; i++)
    {
      if (regnum == i || regnum == -1)
	mipsfbsd_supply_reg (regcache, i, regs + i * regsize, regsize);
    }
}

/* Collect the floating-point registers from REGCACHE and store them
   in FPREGS.  Each floating-point register in FPREGS is REGSIZE bytes
   in length.  */

void
mipsfbsd_collect_fpregs (const struct regcache *regcache, int regnum,
			 void *fpregs, size_t regsize)
{
  char *regs = (char *) fpregs;
  int i;

  for (i = MIPS_FP0_REGNUM; i <= MIPS_FSR_REGNUM; i++)
    {
      if (regnum == i || regnum == -1)
	mipsfbsd_collect_reg (regcache, i,
			     regs + (i - MIPS_FP0_REGNUM) * regsize, regsize);
    }
}

/* Collect the general-purpose registers from REGCACHE and store them
   in GREGS.  Each general-purpose register in GREGS is REGSIZE bytes
   in length.  */

void
mipsfbsd_collect_gregs (const struct regcache *regcache, int regnum,
			void *gregs, size_t regsize)
{
  char *regs = (char *) gregs;
  int i;

  for (i = 0; i <= MIPS_PC_REGNUM; i++)
    {
      if (regnum == i || regnum == -1)
	mipsfbsd_collect_reg (regcache, i, regs + i * regsize, regsize);
    }
}

/* Supply register REGNUM from the buffer specified by FPREGS and LEN
   in the floating-point register set REGSET to register cache
   REGCACHE.  If REGNUM is -1, do this for all registers in REGSET.  */

static void
mipsfbsd_supply_fpregset (const struct regset *regset,
			  struct regcache *regcache,
			  int regnum, const void *fpregs, size_t len)
{
  size_t regsize = mips_abi_regsize (get_regcache_arch (regcache));

  gdb_assert (len >= MIPSFBSD_NUM_FPREGS * regsize);

  mipsfbsd_supply_fpregs (regcache, regnum, fpregs, regsize);
}

/* Collect register REGNUM from the register cache REGCACHE and store
   it in the buffer specified by FPREGS and LEN in the floating-point
   register set REGSET.  If REGNUM is -1, do this for all registers in
   REGSET.  */

static void
mipsfbsd_collect_fpregset (const struct regset *regset,
			   const struct regcache *regcache,
			   int regnum, void *fpregs, size_t len)
{
  size_t regsize = mips_abi_regsize (get_regcache_arch (regcache));

  gdb_assert (len >= MIPSFBSD_NUM_FPREGS * regsize);

  mipsfbsd_collect_fpregs (regcache, regnum, fpregs, regsize);
}

/* Supply register REGNUM from the buffer specified by GREGS and LEN
   in the general-purpose register set REGSET to register cache
   REGCACHE.  If REGNUM is -1, do this for all registers in REGSET.  */

static void
mipsfbsd_supply_gregset (const struct regset *regset,
			 struct regcache *regcache, int regnum,
			 const void *gregs, size_t len)
{
  size_t regsize = mips_abi_regsize (get_regcache_arch (regcache));

  gdb_assert (len >= MIPSFBSD_NUM_GREGS * regsize);

  mipsfbsd_supply_gregs (regcache, regnum, gregs, regsize);
}

/* Collect register REGNUM from the register cache REGCACHE and store
   it in the buffer specified by GREGS and LEN in the general-purpose
   register set REGSET.  If REGNUM is -1, do this for all registers in
   REGSET.  */

static void
mipsfbsd_collect_gregset (const struct regset *regset,
			  const struct regcache *regcache,
			  int regnum, void *gregs, size_t len)
{
  size_t regsize = mips_abi_regsize (get_regcache_arch (regcache));

  gdb_assert (len >= MIPSFBSD_NUM_GREGS * regsize);

  mipsfbsd_collect_gregs (regcache, regnum, gregs, regsize);
}

/* FreeBSD/mips register sets.  */

static const struct regset mipsfbsd_gregset =
{
  NULL,
  mipsfbsd_supply_gregset,
  mipsfbsd_collect_gregset,
};

static const struct regset mipsfbsd_fpregset =
{
  NULL,
  mipsfbsd_supply_fpregset,
  mipsfbsd_collect_fpregset,
};

/* Iterate over core file register note sections.  */

static void
mipsfbsd_iterate_over_regset_sections (struct gdbarch *gdbarch,
				       iterate_over_regset_sections_cb *cb,
				       void *cb_data,
				       const struct regcache *regcache)
{
  size_t regsize = mips_abi_regsize (gdbarch);

  cb (".reg", MIPSFBSD_NUM_GREGS * regsize, &mipsfbsd_gregset,
      NULL, cb_data);
  cb (".reg2", MIPSFBSD_NUM_FPREGS * regsize, &mipsfbsd_fpregset,
      NULL, cb_data);
}

/* Signal trampoline support.  */

static void
mipsfbsd_sigframe_init (const struct tramp_frame *self,
			  struct frame_info *this_frame,
			  struct trad_frame_cache *cache,
			  CORE_ADDR func)
{
  struct gdbarch *gdbarch = get_frame_arch (this_frame);
  enum bfd_endian byte_order = gdbarch_byte_order (gdbarch);
  CORE_ADDR sp, ucontext_addr, addr;
  int regnum;
  gdb_byte buf[4];

  /* We find the appropriate instance of `ucontext_t' at a
     fixed offset in the signal frame.  */
  sp = get_frame_register_signed (this_frame,
				  MIPS_SP_REGNUM + gdbarch_num_regs (gdbarch));
  ucontext_addr = sp + 16;

  /* PC.  */
  regnum = mips_regnum (gdbarch)->pc;
  trad_frame_set_reg_addr (cache,
			   regnum + gdbarch_num_regs (gdbarch),
			    ucontext_addr + 20);

  /* GPRs.  */
  for (regnum = MIPS_AT_REGNUM, addr = ucontext_addr + 28;
       regnum <= MIPS_RA_REGNUM; regnum++, addr += 4)
    trad_frame_set_reg_addr (cache,
			     regnum + gdbarch_num_regs (gdbarch),
			     addr);

  regnum = MIPS_PS_REGNUM;
  trad_frame_set_reg_addr (cache,
			   regnum + gdbarch_num_regs (gdbarch),
			   ucontext_addr + 152);

  /* HI and LO.  */
  regnum = mips_regnum (gdbarch)->lo;
  trad_frame_set_reg_addr (cache,
			   regnum + gdbarch_num_regs (gdbarch),
			   ucontext_addr + 156);
  regnum = mips_regnum (gdbarch)->hi;
  trad_frame_set_reg_addr (cache,
			   regnum + gdbarch_num_regs (gdbarch),
			   ucontext_addr + 160);

  if (target_read_memory (ucontext_addr + 164, buf, 4) == 0 &&
      extract_unsigned_integer (buf, 4, byte_order) != 0)
    {
      for (regnum = 0, addr = ucontext_addr + 168;
	   regnum < 32; regnum++, addr += 8)
	trad_frame_set_reg_addr (cache,
				 regnum + gdbarch_fp0_regnum (gdbarch),
				 addr);
      trad_frame_set_reg_addr (cache, mips_regnum (gdbarch)->fp_control_status,
			       addr);
    }

  trad_frame_set_id (cache, frame_id_build (sp, func));
}

static const struct tramp_frame mipsfbsd_sigframe =
{
  SIGTRAMP_FRAME,
  MIPS_INSN32_SIZE,
  {
    { 0x27a40010, -1 },		/* addiu   a0, sp, SIGF_UC */
    { 0x240201a1, -1 },		/* li      v0, SYS_sigreturn */
    { 0x0000000c, -1 },		/* syscall */
    { 0x0000000d, -1 },		/* break */
    { TRAMP_SENTINEL_INSN, -1 }
  },
  mipsfbsd_sigframe_init
};

static void
mips64fbsd_sigframe_init (const struct tramp_frame *self,
			  struct frame_info *this_frame,
			  struct trad_frame_cache *cache,
			  CORE_ADDR func)
{
  struct gdbarch *gdbarch = get_frame_arch (this_frame);
  enum bfd_endian byte_order = gdbarch_byte_order (gdbarch);
  CORE_ADDR sp, ucontext_addr, addr;
  int regnum;
  gdb_byte buf[4];

  /* We find the appropriate instance of `ucontext_t' at a
     fixed offset in the signal frame.  */
  sp = get_frame_register_signed (this_frame,
				  MIPS_SP_REGNUM + gdbarch_num_regs (gdbarch));
  ucontext_addr = sp + 32;

  /* PC.  */
  regnum = mips_regnum (gdbarch)->pc;
  trad_frame_set_reg_addr (cache,
			   regnum + gdbarch_num_regs (gdbarch),
			    ucontext_addr + 24);

  /* GPRs.  */
  for (regnum = MIPS_AT_REGNUM, addr = ucontext_addr + 40;
       regnum <= MIPS_RA_REGNUM; regnum++, addr += 8)
    trad_frame_set_reg_addr (cache,
			     regnum + gdbarch_num_regs (gdbarch),
			     addr);

  regnum = MIPS_PS_REGNUM;
  trad_frame_set_reg_addr (cache,
			   regnum + gdbarch_num_regs (gdbarch),
			   ucontext_addr + 288);

  /* HI and LO.  */
  regnum = mips_regnum (gdbarch)->lo;
  trad_frame_set_reg_addr (cache,
			   regnum + gdbarch_num_regs (gdbarch),
			   ucontext_addr + 296);
  regnum = mips_regnum (gdbarch)->hi;
  trad_frame_set_reg_addr (cache,
			   regnum + gdbarch_num_regs (gdbarch),
			   ucontext_addr + 304);

  if (target_read_memory (ucontext_addr + 312, buf, 4) == 0 &&
      extract_unsigned_integer (buf, 4, byte_order) != 0)
    {
      for (regnum = 0, addr = ucontext_addr + 320;
	   regnum < 32; regnum++, addr += 8)
	trad_frame_set_reg_addr (cache,
				 regnum + gdbarch_fp0_regnum (gdbarch),
				 addr);
      trad_frame_set_reg_addr (cache, mips_regnum (gdbarch)->fp_control_status,
			       addr);
    }

  trad_frame_set_id (cache, frame_id_build (sp, func));
}

static const struct tramp_frame mips64fbsd_sigframe =
{
  SIGTRAMP_FRAME,
  MIPS_INSN32_SIZE,
  {
    { 0x67a40020, -1 },		/* daddiu  a0, sp, SIGF_UC */
    { 0x240201a1, -1 },		/* li      v0, SYS_sigreturn */
    { 0x0000000c, -1 },		/* syscall */
    { 0x0000000d, -1 },		/* break */
    { TRAMP_SENTINEL_INSN, -1 }
  },
  mips64fbsd_sigframe_init
};

/* Shared library support.  */

/* FreeBSD/mips uses a slightly different `struct link_map' than the
   other FreeBSD platforms as it includes an additional `l_off'
   member.  */

static struct link_map_offsets *
mipsfbsd_ilp32_fetch_link_map_offsets (void)
{
  static struct link_map_offsets lmo;
  static struct link_map_offsets *lmp = NULL;

  if (lmp == NULL) 
    {
      lmp = &lmo;

      lmo.r_version_offset = 0;
      lmo.r_version_size = 4;
      lmo.r_map_offset = 4;
      lmo.r_brk_offset = 8;
      lmo.r_ldsomap_offset = -1;

      lmo.link_map_size = 24;
      lmo.l_addr_offset = 0;
      lmo.l_name_offset = 8;
      lmo.l_ld_offset = 12;
      lmo.l_next_offset = 16;
      lmo.l_prev_offset = 20;
    }

  return lmp;
}

static struct link_map_offsets *
mipsfbsd_lp64_fetch_link_map_offsets (void)
{
  static struct link_map_offsets lmo;
  static struct link_map_offsets *lmp = NULL;

  if (lmp == NULL)
    {
      lmp = &lmo;

      lmo.r_version_offset = 0;
      lmo.r_version_size = 4;
      lmo.r_map_offset = 8;
      lmo.r_brk_offset = 16;
      lmo.r_ldsomap_offset = -1;

      lmo.link_map_size = 48;
      lmo.l_addr_offset = 0;
      lmo.l_name_offset = 16; 
      lmo.l_ld_offset = 24;
      lmo.l_next_offset = 32;
      lmo.l_prev_offset = 40;
    }

  return lmp;
}

static void
mipsfbsd_init_abi (struct gdbarch_info info, struct gdbarch *gdbarch)
{
  enum mips_abi abi = mips_abi (gdbarch);

  /* Generic FreeBSD support. */
  fbsd_init_abi (info, gdbarch);

  set_gdbarch_software_single_step (gdbarch, mips_software_single_step);

  switch (abi)
    {
      case MIPS_ABI_O32:
	tramp_frame_prepend_unwinder (gdbarch, &mipsfbsd_sigframe);
	break;
      case MIPS_ABI_N32:
	set_gdbarch_long_double_bit (gdbarch, 128);
	/* These floatformats should probably be renamed.  MIPS uses
	   the same 128-bit IEEE floating point format that IA-64 uses,
	   except that the quiet/signalling NaN bit is reversed (GDB
	   does not distinguish between quiet and signalling NaNs).  */
	set_gdbarch_long_double_format (gdbarch, floatformats_ia64_quad);
	break;
      case MIPS_ABI_N64:
	set_gdbarch_long_double_bit (gdbarch, 128);
	/* These floatformats should probably be renamed.  MIPS uses
	   the same 128-bit IEEE floating point format that IA-64 uses,
	   except that the quiet/signalling NaN bit is reversed (GDB
	   does not distinguish between quiet and signalling NaNs).  */
	set_gdbarch_long_double_format (gdbarch, floatformats_ia64_quad);
	tramp_frame_prepend_unwinder (gdbarch, &mips64fbsd_sigframe);
	break;
    }

  /* TODO: set_gdbarch_longjmp_target */

  set_gdbarch_iterate_over_regset_sections
    (gdbarch, mipsfbsd_iterate_over_regset_sections);

  /* FreeBSD/mips has SVR4-style shared libraries.  */
  set_solib_svr4_fetch_link_map_offsets
    (gdbarch, (gdbarch_ptr_bit (gdbarch) == 32 ?
	       mipsfbsd_ilp32_fetch_link_map_offsets :
	       mipsfbsd_lp64_fetch_link_map_offsets));
}


/* Provide a prototype to silence -Wmissing-prototypes.  */
void _initialize_mipsfbsd_tdep (void);

void
_initialize_mipsfbsd_tdep (void)
{
  gdbarch_register_osabi (bfd_arch_mips, 0, GDB_OSABI_FREEBSD_ELF,
			  mipsfbsd_init_abi);
}
