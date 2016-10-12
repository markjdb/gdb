/* Target-dependent code for FreeBSD/mips.

   Copyright (C) 2016 Free Software Foundation, Inc.

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

/* Supply register REGNUM from the buffer specified by FPREGS and LEN
   in the floating-point register set REGSET to register cache
   REGCACHE.  If REGNUM is -1, do this for all registers in REGSET.  */

static void
mipsfbsd_supply_fpregset (const struct regset *regset,
			  struct regcache *regcache,
			  int regnum, const void *fpregs, size_t len)
{
  size_t regsize = mips_isa_regsize (get_regcache_arch (regcache));
  const char *regs = (const char *) fpregs;
  int i;

  gdb_assert (len >= MIPSFBSD_NUM_FPREGS * regsize);

  for (i = MIPS_FP0_REGNUM; i <= MIPS_FSR_REGNUM; i++)
    {
      if (regnum == i || regnum == -1)
	regcache_raw_supply (regcache, i,
			     regs + (i - MIPS_FP0_REGNUM) * regsize);
    }
}

/* Supply register REGNUM from the buffer specified by GREGS and LEN
   in the general-purpose register set REGSET to register cache
   REGCACHE.  If REGNUM is -1, do this for all registers in REGSET.  */

static void
mipsfbsd_supply_gregset (const struct regset *regset,
			 struct regcache *regcache, int regnum,
			 const void *gregs, size_t len)
{
  size_t regsize = mips_isa_regsize (get_regcache_arch (regcache));
  const char *regs = (const char *) gregs;
  int i;

  gdb_assert (len >= MIPSFBSD_NUM_GREGS * regsize);

  for (i = 0; i <= MIPS_PC_REGNUM; i++)
    {
      if (regnum == i || regnum == -1)
	regcache_raw_supply (regcache, i, regs + i * regsize);
    }
}

/* FreeBSD/mips register sets.  */

static const struct regset mipsfbsd_gregset =
{
  NULL,
  mipsfbsd_supply_gregset,
};

static const struct regset mipsfbsd_fpregset =
{
  NULL,
  mipsfbsd_supply_fpregset,
};

/* Iterate over core file register note sections.  */

static void
mipsfbsd_iterate_over_regset_sections (struct gdbarch *gdbarch,
				       iterate_over_regset_sections_cb *cb,
				       void *cb_data,
				       const struct regcache *regcache)
{
  size_t regsize = mips_isa_regsize (gdbarch);

  cb (".reg", MIPSFBSD_NUM_GREGS * regsize, &mipsfbsd_gregset,
      NULL, cb_data);
  cb (".reg2", MIPSFBSD_NUM_FPREGS * regsize, &mipsfbsd_fpregset,
      NULL, cb_data);
}

/* Signal trampoline support.  */

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

  regnum = mips_regnum (gdbarch)->cause;
  trad_frame_set_reg_addr (cache,
			   regnum + gdbarch_num_regs (gdbarch),
			   ucontext_addr + 600);

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
	break;
      case MIPS_ABI_N32:
	/* Float formats similar to Linux? */
	break;
      case MIPS_ABI_N64:
	/* Float formats similar to Linux? */
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
