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
#include "inferior.h"
#include "regcache.h"
#include "target.h"

#include <sys/types.h>
#include <sys/ptrace.h>
#include <machine/reg.h>

#include "fbsd-nat.h"
#include "mips-tdep.h"
#include "mips-fbsd-tdep.h"
#include "inf-ptrace.h"

/* Determine if PT_GETREGS fetches this register.  */

static int
getregs_supplies (struct gdbarch *gdbarch, int regnum)
{
  return ((regnum) >= MIPS_ZERO_REGNUM
	  && (regnum) <= gdbarch_pc_regnum (gdbarch));
}

/* Fetch register REGNUM from the inferior.  If REGNUM is -1, do this
   for all registers.  */

static void
mipsfbsd_fetch_inferior_registers (struct target_ops *ops,
				   struct regcache *regcache, int regnum)
{
  struct gdbarch *gdbarch = get_regcache_arch (regcache);
  if (regnum == -1 || getregs_supplies (gdbarch, regnum))
    {
      struct reg regs;

      if (ptrace (PT_GETREGS, get_ptrace_pid (inferior_ptid),
		  (PTRACE_TYPE_ARG3) &regs, 0) == -1)
	perror_with_name (_("Couldn't get registers"));
      
      mipsfbsd_supply_gregs (regcache, regnum, &regs, sizeof (register_t));
      if (regnum != -1)
	return;
    }

  if (regnum == -1
      || regnum >= gdbarch_fp0_regnum (get_regcache_arch (regcache)))
    {
      struct fpreg fpregs;

      if (ptrace (PT_GETFPREGS, get_ptrace_pid (inferior_ptid),
		  (PTRACE_TYPE_ARG3) &fpregs, 0) == -1)
	perror_with_name (_("Couldn't get floating point status"));

      mipsfbsd_supply_fpregs (regcache, regnum, &fpregs,
			      sizeof (f_register_t));
    }
}

/* Store register REGNUM back into the inferior.  If REGNUM is -1, do
   this for all registers.  */

static void
mipsfbsd_store_inferior_registers (struct target_ops *ops,
				   struct regcache *regcache, int regnum)
{
  struct gdbarch *gdbarch = get_regcache_arch (regcache);
  if (regnum == -1 || getregs_supplies (gdbarch, regnum))
    {
      struct reg regs;

      if (ptrace (PT_GETREGS, get_ptrace_pid (inferior_ptid),
		  (PTRACE_TYPE_ARG3) &regs, 0) == -1)
	perror_with_name (_("Couldn't get registers"));

      mipsfbsd_collect_gregs (regcache, regnum, (char *) &regs,
			      sizeof (register_t));

      if (ptrace (PT_SETREGS, get_ptrace_pid (inferior_ptid), 
		  (PTRACE_TYPE_ARG3) &regs, 0) == -1)
	perror_with_name (_("Couldn't write registers"));

      if (regnum != -1)
	return;
    }

  if (regnum == -1
      || regnum >= gdbarch_fp0_regnum (get_regcache_arch (regcache)))
    {
      struct fpreg fpregs; 

      if (ptrace (PT_GETFPREGS, get_ptrace_pid (inferior_ptid),
		  (PTRACE_TYPE_ARG3) &fpregs, 0) == -1)
	perror_with_name (_("Couldn't get floating point status"));

      mipsfbsd_collect_fpregs (regcache, regnum, (char *) &fpregs,
			       sizeof (f_register_t));

      if (ptrace (PT_SETFPREGS, get_ptrace_pid (inferior_ptid),
		  (PTRACE_TYPE_ARG3) &fpregs, 0) == -1)
	perror_with_name (_("Couldn't write floating point status"));
    }
}


/* Provide a prototype to silence -Wmissing-prototypes.  */
void _initialize_mipsfbsd_nat (void);

void
_initialize_mipsfbsd_nat (void)
{
  struct target_ops *t;

  t = inf_ptrace_target ();
  t->to_fetch_registers = mipsfbsd_fetch_inferior_registers;
  t->to_store_registers = mipsfbsd_store_inferior_registers;
  fbsd_nat_add_target (t);
}
