/* Tcl/Tk interface routines.
   Copyright 1994, 1995 Free Software Foundation, Inc.

   Written by Stu Grossman <grossman@cygnus.com> of Cygnus Support.

This file is part of GDB.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.  */

#include "defs.h"
#include "symtab.h"
#include "inferior.h"
#include "command.h"
#include "bfd.h"
#include "symfile.h"
#include "objfiles.h"
#include "target.h"
#include <tcl.h>
#include <tk.h>
#include <varargs.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <setjmp.h>
#include "top.h"
#include <sys/ioctl.h>
#include <string.h>
#include "dis-asm.h"

#ifndef FIOASYNC
#include <sys/stropts.h>
#endif

/* Non-zero means that we're doing the gdbtk interface. */
int gdbtk = 0;

/* Non-zero means we are reloading breakpoints, etc from the
   Gdbtk kernel, and we should suppress various messages */
static int gdbtk_reloading = 0;

/* Handle for TCL interpreter */
static Tcl_Interp *interp = NULL;

/* Handle for TK main window */
static Tk_Window mainWindow = NULL;

static int x_fd;		/* X network socket */

/* This variable determines where memory used for disassembly is read from.

   If > 0, then disassembly comes from the exec file rather than the target
   (which might be at the other end of a slow serial link).  If == 0 then
   disassembly comes from target.  If < 0 disassembly is automatically switched
   to the target if it's an inferior process, otherwise the exec file is
   used.
 */

static int disassemble_from_exec = -1;

static void
null_routine(arg)
     int arg;
{
}

/* The following routines deal with stdout/stderr data, which is created by
   {f}printf_{un}filtered and friends.  gdbtk_fputs and gdbtk_flush are the
   lowest level of these routines and capture all output from the rest of GDB.
   Normally they present their data to tcl via callbacks to the following tcl
   routines:  gdbtk_tcl_fputs, gdbtk_tcl_fputs_error, and gdbtk_flush.  These
   in turn call tk routines to update the display.

   Under some circumstances, you may want to collect the output so that it can
   be returned as the value of a tcl procedure.  This can be done by
   surrounding the output routines with calls to start_saving_output and
   finish_saving_output.  The saved data can then be retrieved with
   get_saved_output (but this must be done before the call to
   finish_saving_output).  */

/* Dynamic string header for stdout. */

static Tcl_DString stdout_buffer;

/* Use this to collect stdout output that will be returned as the result of a
   tcl command.  */

static int saving_output = 0;

static void
start_saving_output ()
{
  saving_output = 1;
}

#define get_saved_output() (Tcl_DStringValue (&stdout_buffer))

static void
finish_saving_output ()
{
  if (!saving_output)
    return;

  saving_output = 0;

  Tcl_DStringFree (&stdout_buffer);
}

/* This routine redirects the output of fputs_unfiltered so that
   the user can see what's going on in his debugger window. */

static void
flush_holdbuf ()
{
  char *s, *argv[1];

  /* We use Tcl_Merge to quote braces and funny characters as necessary. */

  argv[0] = Tcl_DStringValue (&stdout_buffer);
  s = Tcl_Merge (1, argv);

  Tcl_DStringFree (&stdout_buffer);

  Tcl_VarEval (interp, "gdbtk_tcl_fputs ", s, NULL);

  free (s);
}

static void
gdbtk_flush (stream)
     FILE *stream;
{
  if (stream != gdb_stdout || saving_output)
    return;

  /* Flush output from C to tcl land.  */

  flush_holdbuf ();

  /* Force immediate screen update */

  Tcl_VarEval (interp, "gdbtk_tcl_flush", NULL);
}

static void
gdbtk_fputs (ptr, stream)
     const char *ptr;
     FILE *stream;
{
  int len;

  if (stream != gdb_stdout)
    {
      Tcl_VarEval (interp, "gdbtk_tcl_fputs_error ", "{", ptr, "}", NULL);
      return;
    }

  Tcl_DStringAppend (&stdout_buffer, ptr, -1);

  if (saving_output)
    return;

  if (Tcl_DStringLength (&stdout_buffer) > 1000)
    flush_holdbuf ();
}

static int
gdbtk_query (args)
     va_list args;
{
  char *query;
  char buf[200];
  long val;

  query = va_arg (args, char *);

  vsprintf(buf, query, args);
  Tcl_VarEval (interp, "gdbtk_tcl_query ", "{", buf, "}", NULL);

  val = atol (interp->result);
  return val;
}

static void
breakpoint_notify(b, action)
     struct breakpoint *b;
     const char *action;
{
  struct symbol *sym;
  char bpnum[50], line[50], pc[50];
  struct symtab_and_line sal;
  char *filename;
  int v;

  if (b->type != bp_breakpoint)
    return;

  sal = find_pc_line (b->address, 0);

  filename = symtab_to_filename (sal.symtab);

  sprintf (bpnum, "%d", b->number);
  sprintf (line, "%d", sal.line);
  sprintf (pc, "0x%lx", b->address);
 
  v = Tcl_VarEval (interp,
		   "gdbtk_tcl_breakpoint ",
		   action,
		   " ", bpnum,
		   " ", filename ? filename : "{}",
		   " ", line,
		   " ", pc,
		   NULL);

  if (v != TCL_OK)
    {
      gdbtk_fputs (interp->result, gdb_stdout);
      gdbtk_fputs ("\n", gdb_stdout);
    }
}

static void
gdbtk_create_breakpoint(b)
     struct breakpoint *b;
{
  breakpoint_notify(b, "create");
}

static void
gdbtk_delete_breakpoint(b)
     struct breakpoint *b;
{
  breakpoint_notify(b, "delete");
}

static void
gdbtk_enable_breakpoint(b)
     struct breakpoint *b;
{
  breakpoint_notify(b, "enable");
}

static void
gdbtk_disable_breakpoint(b)
     struct breakpoint *b;
{
  breakpoint_notify(b, "disable");
}

/* This implements the TCL command `gdb_loc', which returns a list consisting
   of the source and line number associated with the current pc. */

static int
gdb_loc (clientData, interp, argc, argv)
     ClientData clientData;
     Tcl_Interp *interp;
     int argc;
     char *argv[];
{
  char *filename;
  char buf[100];
  struct symtab_and_line sal;
  char *funcname;
  CORE_ADDR pc;

  if (argc == 1)
    {
      pc = selected_frame ? selected_frame->pc : stop_pc;
      sal = find_pc_line (pc, 0);
    }
  else if (argc == 2)
    {
      struct symtabs_and_lines sals;
      int nelts;

      sals = decode_line_spec (argv[1], 1);

      nelts = sals.nelts;
      sal = sals.sals[0];
      free (sals.sals);

      if (sals.nelts != 1)
	{
	  Tcl_SetResult (interp, "Ambiguous line spec", TCL_STATIC);
	  return TCL_ERROR;
	}

      pc = sal.pc;
    }
  else
    {
      Tcl_SetResult (interp, "wrong # args", TCL_STATIC);
      return TCL_ERROR;
    }

  if (sal.symtab)
    Tcl_AppendElement (interp, sal.symtab->filename);
  else
    Tcl_AppendElement (interp, "");

  find_pc_partial_function (pc, &funcname, NULL, NULL);
  Tcl_AppendElement (interp, funcname);

  filename = symtab_to_filename (sal.symtab);
  Tcl_AppendElement (interp, filename);

  sprintf (buf, "%d", sal.line);
  Tcl_AppendElement (interp, buf); /* line number */

  sprintf (buf, "0x%lx", pc);
  Tcl_AppendElement (interp, buf); /* PC */

  return TCL_OK;
}

/* This implements the TCL command `gdb_eval'. */

static int
gdb_eval (clientData, interp, argc, argv)
     ClientData clientData;
     Tcl_Interp *interp;
     int argc;
     char *argv[];
{
  struct expression *expr;
  struct cleanup *old_chain;
  value_ptr val;

  if (argc != 2)
    {
      Tcl_SetResult (interp, "wrong # args", TCL_STATIC);
      return TCL_ERROR;
    }

  expr = parse_expression (argv[1]);

  old_chain = make_cleanup (free_current_contents, &expr);

  val = evaluate_expression (expr);

  start_saving_output ();	/* Start collecting stdout */

  val_print (VALUE_TYPE (val), VALUE_CONTENTS (val), VALUE_ADDRESS (val),
	     gdb_stdout, 0, 0, 0, 0);
#if 0
  value_print (val, gdb_stdout, 0, 0);
#endif

  Tcl_AppendElement (interp, get_saved_output ());

  finish_saving_output ();	/* Set stdout back to normal */

  do_cleanups (old_chain);

  return TCL_OK;
}

/* This implements the TCL command `gdb_sourcelines', which returns a list of
   all of the lines containing executable code for the specified source file
   (ie: lines where you can put breakpoints). */

static int
gdb_sourcelines (clientData, interp, argc, argv)
     ClientData clientData;
     Tcl_Interp *interp;
     int argc;
     char *argv[];
{
  struct symtab *symtab;
  struct linetable_entry *le;
  int nlines;
  char buf[100];

  if (argc != 2)
    {
      Tcl_SetResult (interp, "wrong # args", TCL_STATIC);
      return TCL_ERROR;
    }

  symtab = lookup_symtab (argv[1]);

  if (!symtab)
    {
      Tcl_SetResult (interp, "No such file", TCL_STATIC);
      return TCL_ERROR;
    }

  /* If there's no linetable, or no entries, then we are done. */

  if (!symtab->linetable
      || symtab->linetable->nitems == 0)
    {
      Tcl_AppendElement (interp, "");
      return TCL_OK;
    }

  le = symtab->linetable->item;
  nlines = symtab->linetable->nitems;

  for (;nlines > 0; nlines--, le++)
    {
      /* If the pc of this line is the same as the pc of the next line, then
	 just skip it.  */
      if (nlines > 1
	  && le->pc == (le + 1)->pc)
	continue;

      sprintf (buf, "%d", le->line);
      Tcl_AppendElement (interp, buf);
    }

  return TCL_OK;
}

static int
map_arg_registers (argc, argv, func, argp)
     int argc;
     char *argv[];
     int (*func) PARAMS ((int regnum, void *argp));
     void *argp;
{
  int regnum;

  /* Note that the test for a valid register must include checking the
     reg_names array because NUM_REGS may be allocated for the union of the
     register sets within a family of related processors.  In this case, the
     trailing entries of reg_names will change depending upon the particular
     processor being debugged.  */

  if (argc == 0)		/* No args, just do all the regs */
    {
      for (regnum = 0;
	   regnum < NUM_REGS
	   && reg_names[regnum] != NULL
	   && *reg_names[regnum] != '\000';
	   regnum++)
	func (regnum, argp);

      return TCL_OK;
    }

  /* Else, list of register #s, just do listed regs */
  for (; argc > 0; argc--, argv++)
    {
      regnum = atoi (*argv);

      if (regnum >= 0
	  && regnum < NUM_REGS
	  && reg_names[regnum] != NULL
	  && *reg_names[regnum] != '\000')
	func (regnum, argp);
      else
	{
	  Tcl_SetResult (interp, "bad register number", TCL_STATIC);

	  return TCL_ERROR;
	}
    }

  return TCL_OK;
}

static int
get_register_name (regnum, argp)
     int regnum;
     void *argp;		/* Ignored */
{
  Tcl_AppendElement (interp, reg_names[regnum]);
}

/* This implements the TCL command `gdb_regnames', which returns a list of
   all of the register names. */

static int
gdb_regnames (clientData, interp, argc, argv)
     ClientData clientData;
     Tcl_Interp *interp;
     int argc;
     char *argv[];
{
  argc--;
  argv++;

  return map_arg_registers (argc, argv, get_register_name, 0);
}

#ifndef REGISTER_CONVERTIBLE
#define REGISTER_CONVERTIBLE(x) (0 != 0)
#endif

#ifndef REGISTER_CONVERT_TO_VIRTUAL
#define REGISTER_CONVERT_TO_VIRTUAL(x, y, z, a)
#endif

#ifndef INVALID_FLOAT
#define INVALID_FLOAT(x, y) (0 != 0)
#endif

static int
get_register (regnum, fp)
     void *fp;
{
  char raw_buffer[MAX_REGISTER_RAW_SIZE];
  char virtual_buffer[MAX_REGISTER_VIRTUAL_SIZE];
  int format = (int)fp;

  if (read_relative_register_raw_bytes (regnum, raw_buffer))
    {
      Tcl_AppendElement (interp, "Optimized out");
      return;
    }

  start_saving_output ();	/* Start collecting stdout */

  /* Convert raw data to virtual format if necessary.  */

  if (REGISTER_CONVERTIBLE (regnum))
    {
      REGISTER_CONVERT_TO_VIRTUAL (regnum, REGISTER_VIRTUAL_TYPE (regnum),
				   raw_buffer, virtual_buffer);
    }
  else
    memcpy (virtual_buffer, raw_buffer, REGISTER_VIRTUAL_SIZE (regnum));

  val_print (REGISTER_VIRTUAL_TYPE (regnum), virtual_buffer, 0,
	     gdb_stdout, format, 1, 0, Val_pretty_default);

  Tcl_AppendElement (interp, get_saved_output ());

  finish_saving_output ();	/* Set stdout back to normal */
}

static int
gdb_fetch_registers (clientData, interp, argc, argv)
     ClientData clientData;
     Tcl_Interp *interp;
     int argc;
     char *argv[];
{
  int format;

  if (argc < 2)
    {
      Tcl_SetResult (interp, "wrong # args", TCL_STATIC);
      return TCL_ERROR;
    }

  argc--;
  argv++;

  argc--;
  format = **argv++;

  return map_arg_registers (argc, argv, get_register, format);
}

/* This contains the previous values of the registers, since the last call to
   gdb_changed_register_list.  */

static char old_regs[REGISTER_BYTES];

static int
register_changed_p (regnum, argp)
     void *argp;		/* Ignored */
{
  char raw_buffer[MAX_REGISTER_RAW_SIZE];
  char buf[100];

  if (read_relative_register_raw_bytes (regnum, raw_buffer))
    return;

  if (memcmp (&old_regs[REGISTER_BYTE (regnum)], raw_buffer,
	      REGISTER_RAW_SIZE (regnum)) == 0)
    return;

  /* Found a changed register.  Save new value and return it's number. */

  memcpy (&old_regs[REGISTER_BYTE (regnum)], raw_buffer,
	  REGISTER_RAW_SIZE (regnum));

  sprintf (buf, "%d", regnum);
  Tcl_AppendElement (interp, buf);
}

static int
gdb_changed_register_list (clientData, interp, argc, argv)
     ClientData clientData;
     Tcl_Interp *interp;
     int argc;
     char *argv[];
{
  int format;

  argc--;
  argv++;

  return map_arg_registers (argc, argv, register_changed_p, NULL);
}

/* This implements the TCL command `gdb_cmd', which sends it's argument into
   the GDB command scanner.  */

static int
gdb_cmd (clientData, interp, argc, argv)
     ClientData clientData;
     Tcl_Interp *interp;
     int argc;
     char *argv[];
{
  if (argc != 2)
    {
      Tcl_SetResult (interp, "wrong # args", TCL_STATIC);
      return TCL_ERROR;
    }

  execute_command (argv[1], 1);

  bpstat_do_actions (&stop_bpstat);

  /* Drain all buffered command output */

  gdb_flush (gdb_stdout);

  return TCL_OK;
}

/* This routine acts as a top-level for all GDB code called by tcl/Tk.  It
   handles cleanups, and calls to return_to_top_level (usually via error).
   This is necessary in order to prevent a longjmp out of the bowels of Tk,
   possibly leaving things in a bad state.  Since this routine can be called
   recursively, it needs to save and restore the contents of the jmp_buf as
   necessary.  */

static int
call_wrapper (clientData, interp, argc, argv)
     ClientData clientData;
     Tcl_Interp *interp;
     int argc;
     char *argv[];
{
  int val;
  struct cleanup *saved_cleanup_chain;
  Tcl_CmdProc *func;
  jmp_buf saved_error_return;

  func = (Tcl_CmdProc *)clientData;
  memcpy (saved_error_return, error_return, sizeof (jmp_buf));

  saved_cleanup_chain = save_cleanups ();

  if (!setjmp (error_return))
    val = func (clientData, interp, argc, argv);
  else
    {
      val = TCL_ERROR;		/* Flag an error for TCL */

      finish_saving_output ();	/* Restore stdout to normal */

      gdb_flush (gdb_stderr);	/* Flush error output */

      gdb_flush (gdb_stdout);	/* Sometimes error output comes here as well */

/* In case of an error, we may need to force the GUI into idle mode because
   gdbtk_call_command may have bombed out while in the command routine.  */

      Tcl_VarEval (interp, "gdbtk_tcl_idle", NULL);
    }

  do_cleanups (ALL_CLEANUPS);

  restore_cleanups (saved_cleanup_chain);

  memcpy (error_return, saved_error_return, sizeof (jmp_buf));

  return val;
}

static int
gdb_listfiles (clientData, interp, argc, argv)
     ClientData clientData;
     Tcl_Interp *interp;
     int argc;
     char *argv[];
{
  int val;
  struct objfile *objfile;
  struct partial_symtab *psymtab;
  struct symtab *symtab;

  ALL_PSYMTABS (objfile, psymtab)
    Tcl_AppendElement (interp, psymtab->filename);

  ALL_SYMTABS (objfile, symtab)
    Tcl_AppendElement (interp, symtab->filename);

  return TCL_OK;
}

static int
gdb_stop (clientData, interp, argc, argv)
     ClientData clientData;
     Tcl_Interp *interp;
     int argc;
     char *argv[];
{
  target_stop ();

  return TCL_OK;
}

/* This implements the TCL command `gdb_disassemble'.  */

static int
gdbtk_dis_asm_read_memory (memaddr, myaddr, len, info)
     bfd_vma memaddr;
     bfd_byte *myaddr;
     int len;
     disassemble_info *info;
{
  extern struct target_ops exec_ops;
  int res;

  errno = 0;
  res = xfer_memory (memaddr, myaddr, len, 0, &exec_ops);

  if (res == len)
    return 0;
  else
    if (errno == 0)
      return EIO;
    else
      return errno;
}

/* We need a different sort of line table from the normal one cuz we can't
   depend upon implicit line-end pc's for lines.  This is because of the
   reordering we are about to do.  */

struct my_line_entry {
  int line;
  CORE_ADDR start_pc;
  CORE_ADDR end_pc;
};

static int
compare_lines (mle1p, mle2p)
     const PTR mle1p;
     const PTR mle2p;
{
  struct my_line_entry *mle1, *mle2;
  int val;

  mle1 = (struct my_line_entry *) mle1p;
  mle2 = (struct my_line_entry *) mle2p;

  val =  mle1->line - mle2->line;

  if (val != 0)
    return val;

  return mle1->start_pc - mle2->start_pc;
}

static int
gdb_disassemble (clientData, interp, argc, argv)
     ClientData clientData;
     Tcl_Interp *interp;
     int argc;
     char *argv[];
{
  CORE_ADDR pc, low, high;
  int mixed_source_and_assembly;
  static disassemble_info di = {
    (fprintf_ftype) fprintf_filtered, /* fprintf_func */
    gdb_stdout,			/* stream */
    NULL,			/* application_data */
    0,				/* flags */
    NULL,			/* private_data */
    NULL,			/* read_memory_func */
    dis_asm_memory_error,	/* memory_error_func */
    dis_asm_print_address	/* print_address_func */
    };

  if (argc != 3 && argc != 4)
    {
      Tcl_SetResult (interp, "wrong # args", TCL_STATIC);
      return TCL_ERROR;
    }

  if (strcmp (argv[1], "source") == 0)
    mixed_source_and_assembly = 1;
  else if (strcmp (argv[1], "nosource") == 0)
    mixed_source_and_assembly = 0;
  else
    {
      Tcl_SetResult (interp, "First arg must be 'source' or 'nosource'",
		     TCL_STATIC);
      return TCL_ERROR;
    }

  low = parse_and_eval_address (argv[2]);

  if (argc == 3)
    {
      if (find_pc_partial_function (low, NULL, &low, &high) == 0)
	{
	  Tcl_SetResult (interp, "No function contains specified address",
			 TCL_STATIC);
	  return TCL_ERROR;
	}
    }
  else
    high = parse_and_eval_address (argv[3]);

  /* If disassemble_from_exec == -1, then we use the following heuristic to
     determine whether or not to do disassembly from target memory or from the
     exec file:

     If we're debugging a local process, read target memory, instead of the
     exec file.  This makes disassembly of functions in shared libs work
     correctly.

     Else, we're debugging a remote process, and should disassemble from the
     exec file for speed.  However, this is no good if the target modifies it's
     code (for relocation, or whatever).
   */

  if (disassemble_from_exec == -1)
    if (strcmp (target_shortname, "child") == 0
	|| strcmp (target_shortname, "procfs") == 0)
      disassemble_from_exec = 0; /* It's a child process, read inferior mem */
    else
      disassemble_from_exec = 1; /* It's remote, read the exec file */

  if (disassemble_from_exec)
    di.read_memory_func = gdbtk_dis_asm_read_memory;
  else
    di.read_memory_func = dis_asm_read_memory;

  /* If just doing straight assembly, all we need to do is disassemble
     everything between low and high.  If doing mixed source/assembly, we've
     got a totally different path to follow.  */

  if (mixed_source_and_assembly)
    {				/* Come here for mixed source/assembly */
      /* The idea here is to present a source-O-centric view of a function to
	 the user.  This means that things are presented in source order, with
	 (possibly) out of order assembly immediately following.  */
      struct symtab *symtab;
      struct linetable_entry *le;
      int nlines;
      int newlines;
      struct my_line_entry *mle;
      struct symtab_and_line sal;
      int i;
      int out_of_order;
      int next_line;

      symtab = find_pc_symtab (low); /* Assume symtab is valid for whole PC range */

      if (!symtab)
	goto assembly_only;

/* First, convert the linetable to a bunch of my_line_entry's.  */

      le = symtab->linetable->item;
      nlines = symtab->linetable->nitems;

      if (nlines <= 0)
	goto assembly_only;

      mle = (struct my_line_entry *) alloca (nlines * sizeof (struct my_line_entry));

      out_of_order = 0;

/* Copy linetable entries for this function into our data structure, creating
   end_pc's and setting out_of_order as appropriate.  */

/* First, skip all the preceding functions.  */

      for (i = 0; i < nlines - 1 && le[i].pc < low; i++) ;

/* Now, copy all entries before the end of this function.  */

      newlines = 0;
      for (; i < nlines - 1 && le[i].pc < high; i++)
	{
	  if (le[i].line == le[i + 1].line
	      && le[i].pc == le[i + 1].pc)
	    continue;		/* Ignore duplicates */

	  mle[newlines].line = le[i].line;
	  if (le[i].line > le[i + 1].line)
	    out_of_order = 1;
	  mle[newlines].start_pc = le[i].pc;
	  mle[newlines].end_pc = le[i + 1].pc;
	  newlines++;
	}

/* If we're on the last line, and it's part of the function, then we need to
   get the end pc in a special way.  */

      if (i == nlines - 1
	  && le[i].pc < high)
	{
	  mle[newlines].line = le[i].line;
	  mle[newlines].start_pc = le[i].pc;
	  sal = find_pc_line (le[i].pc, 0);
	  mle[newlines].end_pc = sal.end;
	  newlines++;
	}

/* Now, sort mle by line #s (and, then by addresses within lines). */

      if (out_of_order)
	qsort (mle, newlines, sizeof (struct my_line_entry), compare_lines);

/* Now, for each line entry, emit the specified lines (unless they have been
   emitted before), followed by the assembly code for that line.  */

      next_line = 0;		/* Force out first line */
      for (i = 0; i < newlines; i++)
	{
/* Print out everything from next_line to the current line.  */

	  if (mle[i].line >= next_line)
	    {
	      if (next_line != 0)
		print_source_lines (symtab, next_line, mle[i].line + 1, 0);
	      else
		print_source_lines (symtab, mle[i].line, mle[i].line + 1, 0);

	      next_line = mle[i].line + 1;
	    }

	  for (pc = mle[i].start_pc; pc < mle[i].end_pc; )
	    {
	      QUIT;
	      fputs_unfiltered ("    ", gdb_stdout);
	      print_address (pc, gdb_stdout);
	      fputs_unfiltered (":\t    ", gdb_stdout);
	      pc += (*tm_print_insn) (pc, &di);
	      fputs_unfiltered ("\n", gdb_stdout);
	    }
	}
    }
  else
    {
assembly_only:
      for (pc = low; pc < high; )
	{
	  QUIT;
	  fputs_unfiltered ("    ", gdb_stdout);
	  print_address (pc, gdb_stdout);
	  fputs_unfiltered (":\t    ", gdb_stdout);
	  pc += (*tm_print_insn) (pc, &di);
	  fputs_unfiltered ("\n", gdb_stdout);
	}
    }

  gdb_flush (gdb_stdout);

  return TCL_OK;
}

static void
tk_command (cmd, from_tty)
     char *cmd;
     int from_tty;
{
  int retval;
  char *result;
  struct cleanup *old_chain;

  retval = Tcl_Eval (interp, cmd);

  result = strdup (interp->result);

  old_chain = make_cleanup (free, result);

  if (retval != TCL_OK)
    error (result);

  printf_unfiltered ("%s\n", result);

  do_cleanups (old_chain);
}

static void
cleanup_init (ignored)
     int ignored;
{
  if (mainWindow != NULL)
    Tk_DestroyWindow (mainWindow);
  mainWindow = NULL;

  if (interp != NULL)
    Tcl_DeleteInterp (interp);
  interp = NULL;
}

/* Come here during long calculations to check for GUI events.  Usually invoked
   via the QUIT macro.  */

static void
gdbtk_interactive ()
{
  /* Tk_DoOneEvent (TK_DONT_WAIT|TK_IDLE_EVENTS); */
}

/* Come here when there is activity on the X file descriptor. */

static void
x_event (signo)
     int signo;
{
  /* Process pending events */

  while (Tk_DoOneEvent (TK_DONT_WAIT|TK_ALL_EVENTS) != 0);
}

static int
gdbtk_wait (pid, ourstatus)
     int pid;
     struct target_waitstatus *ourstatus;
{
  struct sigaction action;
  static sigset_t nullsigmask = {0};

#ifndef SA_RESTART
  /* Needed for SunOS 4.1.x */
#define SA_RESTART 0
#endif

  action.sa_handler = x_event;
  action.sa_mask = nullsigmask;
  action.sa_flags = SA_RESTART;
  sigaction(SIGIO, &action, NULL);

  pid = target_wait (pid, ourstatus);

  action.sa_handler = SIG_IGN;
  sigaction(SIGIO, &action, NULL);

  return pid;
}

/* This is called from execute_command, and provides a wrapper around
   various command routines in a place where both protocol messages and
   user input both flow through.  Mostly this is used for indicating whether
   the target process is running or not.
*/

static void
gdbtk_call_command (cmdblk, arg, from_tty)
     struct cmd_list_element *cmdblk;
     char *arg;
     int from_tty;
{
  if (cmdblk->class == class_run)
    {
      Tcl_VarEval (interp, "gdbtk_tcl_busy", NULL);
      (*cmdblk->function.cfunc)(arg, from_tty);
      Tcl_VarEval (interp, "gdbtk_tcl_idle", NULL);
    }
  else
    (*cmdblk->function.cfunc)(arg, from_tty);
}

static void
gdbtk_init ()
{
  struct cleanup *old_chain;
  char *gdbtk_filename;
  int i;
  struct sigaction action;
  static sigset_t nullsigmask = {0};
  extern struct cmd_list_element *setlist;
  extern struct cmd_list_element *showlist;

  old_chain = make_cleanup (cleanup_init, 0);

  /* First init tcl and tk. */

  interp = Tcl_CreateInterp ();

  if (!interp)
    error ("Tcl_CreateInterp failed");

  Tcl_DStringInit (&stdout_buffer); /* Setup stdout buffer */

  mainWindow = Tk_CreateMainWindow (interp, NULL, "gdb", "Gdb");

  if (!mainWindow)
    return;			/* DISPLAY probably not set */

  if (Tcl_Init(interp) != TCL_OK)
    error ("Tcl_Init failed: %s", interp->result);

  if (Tk_Init(interp) != TCL_OK)
    error ("Tk_Init failed: %s", interp->result);

  Tcl_CreateCommand (interp, "gdb_cmd", call_wrapper, gdb_cmd, NULL);
  Tcl_CreateCommand (interp, "gdb_loc", call_wrapper, gdb_loc, NULL);
  Tcl_CreateCommand (interp, "gdb_sourcelines", call_wrapper, gdb_sourcelines,
		     NULL);
  Tcl_CreateCommand (interp, "gdb_listfiles", call_wrapper, gdb_listfiles,
		     NULL);
  Tcl_CreateCommand (interp, "gdb_stop", call_wrapper, gdb_stop, NULL);
  Tcl_CreateCommand (interp, "gdb_regnames", call_wrapper, gdb_regnames, NULL);
  Tcl_CreateCommand (interp, "gdb_fetch_registers", call_wrapper,
		     gdb_fetch_registers, NULL);
  Tcl_CreateCommand (interp, "gdb_changed_register_list", call_wrapper,
		     gdb_changed_register_list, NULL);
  Tcl_CreateCommand (interp, "gdb_disassemble", call_wrapper,
		     gdb_disassemble, NULL);
  Tcl_CreateCommand (interp, "gdb_eval", call_wrapper, gdb_eval, NULL);

  command_loop_hook = Tk_MainLoop;
  print_frame_info_listing_hook = null_routine;
  query_hook = gdbtk_query;
  flush_hook = gdbtk_flush;
  create_breakpoint_hook = gdbtk_create_breakpoint;
  delete_breakpoint_hook = gdbtk_delete_breakpoint;
  enable_breakpoint_hook = gdbtk_enable_breakpoint;
  disable_breakpoint_hook = gdbtk_disable_breakpoint;
  interactive_hook = gdbtk_interactive;
  target_wait_hook = gdbtk_wait;
  call_command_hook = gdbtk_call_command;

  /* Get the file descriptor for the X server */

  x_fd = ConnectionNumber (Tk_Display (mainWindow));

  /* Setup for I/O interrupts */

  action.sa_mask = nullsigmask;
  action.sa_flags = 0;
  action.sa_handler = SIG_IGN;
  sigaction(SIGIO, &action, NULL);

#ifdef FIOASYNC
  i = 1;
  if (ioctl (x_fd, FIOASYNC, &i))
    perror_with_name ("gdbtk_init: ioctl FIOASYNC failed");

  i = getpid();
  if (ioctl (x_fd, SIOCSPGRP, &i))
    perror_with_name ("gdbtk_init: ioctl SIOCSPGRP failed");
#else
  if (ioctl (x_fd,  I_SETSIG, S_INPUT|S_RDNORM) < 0)
    perror_with_name ("gdbtk_init: ioctl I_SETSIG failed");
#endif /* ifndef FIOASYNC */

  add_com ("tk", class_obscure, tk_command,
	   "Send a command directly into tk.");

#if 0
  add_show_from_set (add_set_cmd ("disassemble-from-exec", class_support,
				  var_boolean, (char *)&disassemble_from_exec,
				  "Set ", &setlist),
		     &showlist);
#endif

  Tcl_LinkVar (interp, "disassemble-from-exec", (char *)&disassemble_from_exec,
	       TCL_LINK_INT);

  /* Load up gdbtk.tcl after all the environment stuff has been setup.  */

  gdbtk_filename = getenv ("GDBTK_FILENAME");
  if (!gdbtk_filename)
    if (access ("gdbtk.tcl", R_OK) == 0)
      gdbtk_filename = "gdbtk.tcl";
    else
      gdbtk_filename = GDBTK_FILENAME;

/* Defer setup of fputs_unfiltered_hook to near the end so that error messages
   prior to this point go to stdout/stderr.  */

  fputs_unfiltered_hook = gdbtk_fputs;

  if (Tcl_EvalFile (interp, gdbtk_filename) != TCL_OK)
    {
      char *err;

      fputs_unfiltered_hook = NULL; /* Force errors to stdout/stderr */

      fprintf_unfiltered (stderr, "%s:%d: %s\n", gdbtk_filename,
			  interp->errorLine, interp->result);

      fputs_unfiltered ("Stack trace:\n", gdb_stderr);
      fputs_unfiltered (Tcl_GetVar (interp, "errorInfo", 0), gdb_stderr);
      error ("");
    }

  discard_cleanups (old_chain);
}

/* Come here during initialze_all_files () */

void
_initialize_gdbtk ()
{
  if (use_windows)
    {
      /* Tell the rest of the world that Gdbtk is now set up. */

      init_ui_hook = gdbtk_init;
    }
}
