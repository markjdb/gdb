/* Target-dependent code for FreeBSD, architecture-independent.

   Copyright (C) 2002-2015 Free Software Foundation, Inc.

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
#include "gdbcore.h"
#include "inferior.h"
#include "regcache.h"
#include "regset.h"
#include "gdbthread.h"

#include "elf-bfd.h"
#include "fbsd-tdep.h"


/* This is how we want PTIDs from core files to be printed.  */

static char *
fbsd_core_pid_to_str (struct gdbarch *gdbarch, ptid_t ptid)
{
  static char buf[80], name[64];
  struct bfd_section *section;
  bfd_size_type size;
  char sectionstr[32];

  if (ptid_get_lwp (ptid) != 0)
    {
      snprintf (sectionstr, sizeof sectionstr, ".thrmisc/%ld",
		ptid_get_lwp (ptid));
      section = bfd_get_section_by_name (core_bfd, sectionstr);
      if (section != NULL)
	{
	  char *name;

	  size = bfd_section_size (core_bfd, section);
	  name = alloca (size + 1);
	  if (bfd_get_section_contents (core_bfd, section, name, (file_ptr) 0,
					size) && name[0] != '\0')
	    {
	      name[size] = '\0';
	      if (strcmp(name, elf_tdata (core_bfd)->core->program) != 0)
		{
		  snprintf (buf, sizeof buf, "LWP %ld \"%s\"",
			    ptid_get_lwp (ptid), name);
		  return buf;
		}
	    }
	}
      snprintf (buf, sizeof buf, "LWP %ld", ptid_get_lwp (ptid));
      return buf;
    }

  return normal_pid_to_str (ptid);
}

static int
find_signalled_thread (struct thread_info *info, void *data)
{
  if (info->suspend.stop_signal != GDB_SIGNAL_0
      && ptid_get_pid (info->ptid) == ptid_get_pid (inferior_ptid))
    return 1;

  return 0;
}

static enum gdb_signal
find_stop_signal (void)
{
  struct thread_info *info =
    iterate_over_threads (find_signalled_thread, NULL);

  if (info)
    return info->suspend.stop_signal;
  else
    return GDB_SIGNAL_0;
}

/* Structure for passing information from
   fbsd_collect_thread_registers via an iterator to
   fbsd_collect_regset_section_cb. */

struct fbsd_collect_regset_section_cb_data
{
  struct gdbarch *gdbarch;
  const struct regcache *regcache;
  bfd *obfd;
  char *note_data;
  int *note_size;
  unsigned long lwp;
  enum gdb_signal stop_signal;
  int abort_iteration;
};

static void
fbsd_collect_regset_section_cb (const char *sect_name, int size,
				const struct regset *regset,
				const char *human_name, void *cb_data)
{
  char *buf;
  struct fbsd_collect_regset_section_cb_data *data = cb_data;

  if (data->abort_iteration)
    return;

  gdb_assert (regset->collect_regset);

  buf = xmalloc (size);
  regset->collect_regset (regset, data->regcache, -1, buf, size);

  /* PRSTATUS still needs to be treated specially.  */
  if (strcmp (sect_name, ".reg") == 0)
    data->note_data = (char *) elfcore_write_prstatus
      (data->obfd, data->note_data, data->note_size, data->lwp,
       gdb_signal_to_host (data->stop_signal), buf);
  else
    data->note_data = (char *) elfcore_write_register_note
      (data->obfd, data->note_data, data->note_size,
       sect_name, buf, size);
  xfree (buf);

  if (data->note_data == NULL)
    data->abort_iteration = 1;
}

/* Records the thread's register state for the corefile note
   section.  */

static char *
fbsd_collect_thread_registers (const struct regcache *regcache,
			       ptid_t ptid, bfd *obfd,
			       char *note_data, int *note_size,
			       enum gdb_signal stop_signal)
{
  struct gdbarch *gdbarch = get_regcache_arch (regcache);
  struct fbsd_collect_regset_section_cb_data data;

  data.gdbarch = gdbarch;
  data.regcache = regcache;
  data.obfd = obfd;
  data.note_data = note_data;
  data.note_size = note_size;
  data.stop_signal = stop_signal;
  data.abort_iteration = 0;
  data.lwp = ptid_get_lwp (ptid);

  gdbarch_iterate_over_regset_sections (gdbarch,
					fbsd_collect_regset_section_cb,
					&data, regcache);
  return data.note_data;
}

struct fbsd_corefile_thread_data
{
  struct gdbarch *gdbarch;
  int pid;
  bfd *obfd;
  char *note_data;
  int *note_size;
  enum gdb_signal stop_signal;
};

/* Called by gdbthread.c once per thread.  Records the thread's
   register state for the corefile note section.  */

static int
fbsd_corefile_thread_callback (struct thread_info *info, void *data)
{
  struct fbsd_corefile_thread_data *args = data;

  /* It can be current thread
     which cannot be removed by update_thread_list.  */
  if (info->state == THREAD_EXITED)
    return 0;

  if (ptid_get_pid (info->ptid) == args->pid)
    {
      struct cleanup *old_chain;
      struct regcache *regcache;

      regcache = get_thread_arch_regcache (info->ptid, args->gdbarch);

      old_chain = save_inferior_ptid ();
      inferior_ptid = info->ptid;
      target_fetch_registers (regcache, -1);
      do_cleanups (old_chain);

      args->note_data = fbsd_collect_thread_registers
	(regcache, info->ptid, args->obfd, args->note_data,
	 args->note_size, args->stop_signal);
    }

  return !args->note_data;
}

/* Create appropriate note sections for a corefile, returning them in
   allocated memory.  */

static char *
fbsd_make_corefile_notes (struct gdbarch *gdbarch, bfd *obfd, int *note_size)
{
  struct fbsd_corefile_thread_data thread_args;
  char *note_data = NULL;
  Elf_Internal_Ehdr *i_ehdrp;

  /* Put a "FreeBSD" label in the ELF header.  */
  i_ehdrp = elf_elfheader (obfd);
  i_ehdrp->e_ident[EI_OSABI] = ELFOSABI_FREEBSD;

  gdb_assert (gdbarch_iterate_over_regset_sections_p (gdbarch));

  if (get_exec_file (0))
    {
      const char *fname = lbasename (get_exec_file (0));
      char *psargs = xstrdup (fname);

      if (get_inferior_args ())
	psargs = reconcat (psargs, psargs, " ", get_inferior_args (),
			   (char *) NULL);

      note_data = elfcore_write_prpsinfo (obfd, note_data, note_size,
					  fname, psargs);
    }

  /* Thread register information.  */
  TRY
    {
      update_thread_list ();
    }
  CATCH (e, RETURN_MASK_ERROR)
    {
      exception_print (gdb_stderr, e);
    }
  END_CATCH

  thread_args.gdbarch = gdbarch;
  thread_args.pid = ptid_get_pid (inferior_ptid);
  thread_args.obfd = obfd;
  thread_args.note_data = note_data;
  thread_args.note_size = note_size;
  thread_args.stop_signal = find_stop_signal ();
  iterate_over_threads (fbsd_corefile_thread_callback, &thread_args);
  note_data = thread_args.note_data;

  return note_data;
}

/* To be called from GDB_OSABI_FREEBSD_ELF handlers. */

void
fbsd_init_abi (struct gdbarch_info info, struct gdbarch *gdbarch)
{
  set_gdbarch_core_pid_to_str (gdbarch, fbsd_core_pid_to_str);
  set_gdbarch_make_corefile_notes (gdbarch, fbsd_make_corefile_notes);
}
