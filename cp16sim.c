// Western Digital MCP1600 microcode-level simulator
// Copyright 2016 Eric Smith <spacewar@gmail.com>

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sysexits.h>

// The WD9000 chipset is rated for operation between 1.0 and 3.3 MHz.
// The WD900 and PDQ-3 boards run at 2.5 MHz.
#define CPU_CLOCK_FREQ_HZ   (2.5e6)
#define CPU_CLOCK_PERIOD_S  (1.0/CPU_CLOCK_FREQ_HZ)
#define CPU_CLOCK_PERIOD_NS (CPU_CLOCK_PERIOD_S * 1.0e9)

#define IO_TICK_FREQ_HZ     (10.0e3)
#define IO_TICK_PERIOD_S    (1.0/IO_TICK_FREQ_HZ)
#define IO_TICK_PERIOD_NS   (IO_TICK_PERIOD_S * 1.0e9)

#define UINST_LRR 0x010000
#define UINST_RNI 0x020000

uint64_t time_ns;
uint64_t io_time_ns;

typedef struct translation_t
{
  struct translation_t *next;

  // inputs
  uint8_t translation_state_match;
  uint8_t translation_state_mask;
  bool interrupt; // true if data, mask match interrupt register
                  // false if data, mask match translation register
  uint8_t data;
  uint8_t mask;
  // Q input not simulated, not used in WD9000

  // outputs
  bool ltsr;  // load translation state register
  bool lta;   // load translation address into PC
  bool lra;   // load return address into PC
  uint8_t translation_state;
  uint16_t addr;
} translation_t;
  

uint32_t ucode[0x800];
uint8_t tr_num_from_addr[0x800];
uint8_t rni_tr_num;

translation_t *translation_from_num[0x80];



typedef struct proc_t
{
  bool two_cycle;

  uint16_t pc;
  uint16_t return_stack;  // single level return stack
  
  uint16_t prev_pc;       // for error messages

  uint16_t modify_uinst;  // value or'd into next microinstruction
  bool rni;               // true if uinst RNI bit set on previous instruction

  uint32_t uinst;         // current microinstruction

  int g;

  bool branch;
  uint16_t branch_target;

  int translation_state;  // three bits
  uint16_t translation_register;

  uint8_t reg[28];  // 0x00 to 0x01: not used (indirect)
                    // 0x02 to 0x0f: direct
                    // 0x10 to 0x1b: indirect only
                    // 0x1c to 0x1f: remapped to 0x0c to 0x0f

  uint16_t addr;  // external address

  uint8_t interrupt;  // 7 bits - 0..3 from external hardware
                      //          4..6 register controled by microcode

  // status bits
  bool nb;
  bool zb;
  bool c4;
  bool c8;

  // condition codes
  bool cc_n;
  bool cc_z;
  bool cc_v;
  bool cc_c;

} proc_t;


typedef struct device_t device_t;

typedef void reset_fn_t(proc_t *proc,  // intended for debug use only
			device_t *device);

typedef void tick_fn_t(proc_t *proc, // intended for debug use only
		       device_t *device);

typedef uint16_t read_fn_t(proc_t *proc,  // intended for debug use only
			   device_t *device,
			   uint16_t addr);

typedef void write_fn_t(proc_t *proc,  // intended for debug use only
			device_t *device,
			uint16_t addr,
			uint16_t data);


typedef struct device_t
{
  struct device_t *next;
  reset_fn_t *reset_fn;
  tick_fn_t  *tick_fn;
  read_fn_t  *read_fn;
  write_fn_t *write_fn;
  void *device;
} device_t;

int32_t    mem[0x10000];  // -1 if uninitialized
read_fn_t  *read_fn[0x10000];
write_fn_t *write_fn[0x10000];
device_t   *mm_device[0x10000];

device_t *device_head;

void mem_install(uint16_t addr, uint32_t size, read_fn_t *r, write_fn_t *w)
{
  while (size--)
    {
      read_fn[addr] = r;
      write_fn[addr] = w;
      mm_device[addr] = NULL;
      addr++;
    }
}

void device_install(uint16_t addr, uint32_t size, device_t *device)
{
  mem_install(addr, size, device->read_fn, device->write_fn);
  while (size--)
    mm_device[addr++] = device;

  device->next = device_head;
  device_head = device;
}


uint16_t bad_read(proc_t *proc, device_t *device, uint16_t addr)
{
  fprintf(stderr, "stray read, addr=%04x, uPC=%03x\n", addr, proc->prev_pc);
  exit(EX_SOFTWARE);
}

void bad_write(proc_t *proc, device_t *device, uint16_t addr, uint16_t data)
{
  fprintf(stderr, "stray write, addr=%04x, data=%04x, uPC=%03x\n", addr, data, proc->prev_pc);
  exit(EX_SOFTWARE);
}

uint16_t ram_read(proc_t *proc, device_t *device, uint16_t addr)
{
  int32_t data = mem[addr];
  if (data < 0)
    {
      fprintf(stderr, "@@@ read of uninitialzed data, addr=%04x, uPC=%03x\n", addr, proc->prev_pc);
      //exit(EX_SOFTWARE);
      data = 0;
    }
  printf("read mem %04x, data %04x\n", addr, data);
  return data;
}

void ram_write(proc_t *proc, device_t *device, uint16_t addr, uint16_t data)
{
  mem[addr] = data;
}


// start an external cycle
void ext_cycle_start(proc_t   *proc,
		     uint16_t addr,
		     bool     acknowledge,
		     bool     write)
{
  proc->addr = addr;
}

// external input, finish an external read cycle or partially finish RMW cycle
uint16_t ext_input(proc_t *proc,
		   bool wait_for_reply,
		   bool word,
		   bool status,
		   bool rmw)
{
  uint16_t addr = proc->addr;
  if (status)
    {
      fprintf(stderr, "external status input undefined, uPC %03x\n", proc->prev_pc);
      exit(EX_SOFTWARE);
      
    }
  return read_fn[addr](proc, mm_device[addr], addr);
}

void ext_output(proc_t   *proc,
		bool     wait_for_reply,
		bool     word,
		bool     status,
		uint16_t data)
{
  uint16_t addr = proc->addr;
  if (status)
    {
      fprintf(stderr, "external status output undefined, uPC %03x\n", proc->prev_pc);
      exit(EX_SOFTWARE);
      
    }
  write_fn[addr](proc, mm_device[addr], addr, data);
}

#define UPD_NB 0x80
#define UPD_ZB 0x40
#define UPD_C4 0x20
#define UPD_C8 0x10
#define UPD_N  0x08
#define UPD_Z  0x04
#define UPD_V  0x02
#define UPD_C  0x01

uint8_t add_8(uint8_t op1,
	      uint8_t op2,
	      bool carry_in,
	      proc_t *proc,
	      int flags)
{
  uint16_t sum;
  uint8_t result;
  bool n, z;
  bool c4, c8;
  bool c7, v;

  sum = ((uint16_t) (op1 & 0x0f)) + ((uint16_t) (op2 & 0x0f)) + ((uint16_t) carry_in);
  c4 = sum >> 4;

  sum = ((uint16_t) (op1 & 0x7f)) + ((uint16_t) (op2 & 0x7f)) + ((uint16_t) carry_in);
  c7 = sum >> 7;

  sum = ((uint16_t) op1) + ((uint16_t) op2) + ((uint16_t) carry_in);
  result = sum & 0xff;
  n = result >> 7;
  z = result == 0;
  c8 = sum >> 8;
  v = c7 ^ c8;

  if (proc)
    {
      if (flags & UPD_NB)
	proc->nb = n;
      if (flags & UPD_ZB)
	proc->zb = z;
      if (flags & UPD_C8)
	proc->c8 = c8;
      if (flags & UPD_C4)
	proc->c4 = c4;
      if (flags & UPD_N)
	proc->cc_n = n;
      if (flags & UPD_Z)
	proc->cc_z = z;
      if (flags & UPD_V)
	proc->cc_v = v;
      if (flags & UPD_C)
	proc->cc_c = c8;
    }
  return result;
}

uint8_t sub_8(uint8_t op1,
	      uint8_t op2,
	      bool borrow_in,
	      proc_t *proc,
	      int flags)
{
  bool carry_in;
  uint16_t sum;
  uint8_t result;
  bool n, z;
  bool c4, c8;
  bool c7, v;

  carry_in = ! borrow_in;
  op2 ^= 0xff;
  
  sum = ((uint16_t) (op1 & 0x0f)) + ((uint16_t) (op2 & 0x0f)) + ((uint16_t) carry_in);
  c4 = sum >> 4;

  sum = ((uint16_t) (op1 & 0x7f)) + ((uint16_t) (op2 & 0x7f)) + ((uint16_t) carry_in);
  c7 = sum >> 7;

  sum = ((uint16_t) op1) + ((uint16_t) op2) + ((uint16_t) carry_in);
  result = sum & 0xff;
  n = result >> 7;
  z = result == 0;
  c8 = sum >> 8;
  v = c7 ^ c8;

  if (proc)
    {
      if (flags & UPD_NB)
	proc->nb = n;
      if (flags & UPD_ZB)
	proc->zb = z;
      if (flags & UPD_C8)
	proc->c8 = ! c8;
      if (flags & UPD_C4)
	proc->c4 = ! c4;
      if (flags & UPD_N)
	proc->cc_n = n;
      if (flags & UPD_Z)
	proc->cc_z = z;
      if (flags & UPD_V)
	proc->cc_v = v;
      if (flags & UPD_C)
	proc->cc_c = ! c8;
    }
  return result;
}

uint16_t add_16(uint16_t op1,
		uint16_t op2,
		bool carry_in,
		proc_t *proc,
		int flags)
{
  uint32_t sum;
  uint16_t result;
  bool n, z;
  bool c4, c8;
  bool c7, v;

  // XXX not sure whether C4 flag for 16-bit arithmetic should be carry
  // into bit 4 or bit 12 (bit 4 of high byte)
  sum = ((uint32_t) (op1 & 0x0fff)) + ((uint32_t) (op2 & 0x0fff)) + ((uint32_t) carry_in);
  c4 = sum >> 12;

  sum = ((uint32_t) (op1 & 0x7fff)) + ((uint32_t) (op2 & 0x7fff)) + ((uint32_t) carry_in);
  c7 = sum >> 15;

  sum = ((uint32_t) op1) + ((uint32_t) op2) + ((uint32_t) carry_in);
  result = sum & 0xffff;
  n = result >> 15;
  z = result == 0;
  c8 = sum >> 16;
  v = c7 ^ c8;

  if (proc)
    {
      if (flags & UPD_NB)
	proc->nb = n;
      if (flags & UPD_ZB)
	proc->zb = z;
      if (flags & UPD_C8)
	proc->c8 = c8;
      if (flags & UPD_C4)
	proc->c4 = c4;
      if (flags & UPD_N)
	proc->cc_n = n;
      if (flags & UPD_Z)
	proc->cc_z = z;
      if (flags & UPD_V)
	proc->cc_v = v;
      if (flags & UPD_C)
	proc->cc_c = c8;
    }
  return result;
}

uint16_t sub_16(uint16_t op1,
		uint16_t op2,
		bool borrow_in,
		proc_t *proc,
		int flags)
{
  bool carry_in;
  uint32_t sum;
  uint16_t result;
  bool n, z;
  bool c4, c8;
  bool c7, v;

  carry_in = ! borrow_in;
  op2 ^= 0xffff;
  
  // XXX not sure whether C4 flag for 16-bit arithmetic should be carry
  // into bit 4 or bit 12 (bit 4 of high byte)
  sum = ((uint32_t) (op1 & 0x0fff)) + ((uint32_t) (op2 & 0x0fff)) + ((uint32_t) carry_in);
  c4 = sum >> 12;

  sum = ((uint32_t) (op1 & 0x7fff)) + ((uint32_t) (op2 & 0x7fff)) + ((uint32_t) carry_in);
  c7 = sum >> 15;

  sum = ((uint32_t) op1) + ((uint32_t) op2) + ((uint32_t) carry_in);
  result = sum & 0xffff;
  n = result >> 15;
  z = result == 0;
  c8 = sum >> 16;
  v = c7 ^ c8;

  if (proc)
    {
      if (flags & UPD_NB)
	proc->nb = n;
      if (flags & UPD_ZB)
	proc->zb = z;
      if (flags & UPD_C8)
	proc->c8 = ! c8;
      if (flags & UPD_C4)
	proc->c4 = ! c4;
      if (flags & UPD_N)
	proc->cc_n = n;
      if (flags & UPD_Z)
	proc->cc_z = z;
      if (flags & UPD_V)
	proc->cc_v = v;
      if (flags & UPD_C)
	proc->cc_c = ! c8;
    }
  return result;
}

bool icc(proc_t *proc)
{
  fprintf(stderr, "indirect condition code unimplemented\n");
  exit(EX_SOFTWARE);
}


uint16_t sign_extend(uint16_t d)
{
  d &= 0xff;
  if (d & 0x80)
    return 0xff00 | d;
  else
    return d;
}

static inline int reg_addr(proc_t *proc, int reg_num)
{
  if (reg_num <= 0x01)
    {
      reg_num = 0x10 | (proc->g << 1) | (reg_num & 1);
      if (reg_num >= 0x1c)
	reg_num -= 0x10;
    }
  return reg_num;
}

void write_reg_8(proc_t *proc, int reg_num, uint8_t value)
{
  proc->reg[reg_addr(proc, reg_num)] = value;
}

uint8_t read_reg_8(proc_t *proc, int reg_num)
{
  return proc->reg[reg_addr(proc, reg_num)];
}

uint16_t read_reg_16_a(proc_t *proc, int reg_num)
{
  return (read_reg_8(proc, reg_num ^ 1) << 8) | read_reg_8(proc, reg_num);
}

uint16_t read_reg_16_b(proc_t *proc, int reg_num)
{
  if (reg_num & 1)
    return sign_extend(read_reg_8(proc, reg_num));
  else
    return (read_reg_8(proc, reg_num + 1) << 8) | read_reg_8(proc, reg_num);
}

uint16_t read_reg_16_ba(proc_t *proc, int b, int a)
{
  return (read_reg_8(proc, b) << 8) | read_reg_8(proc, a);
}

void write_reg_16_a(proc_t *proc, int reg_num, uint16_t d)
{
  write_reg_8(proc, reg_num, d & 0xff);
  write_reg_8(proc, reg_num ^ 1, d >> 8);
}

void write_reg_16_ba(proc_t *proc, int b, int a, uint16_t d)
{
  write_reg_8(proc, a, d & 0xff);
  write_reg_8(proc, b, d >> 8);
}


void op_illegal(proc_t *proc)
{
  fprintf(stderr, "illegal opcode %04x at addr %03x\n", proc->uinst, proc->prev_pc);
  exit(EX_SOFTWARE);
}

// 0000..07ff JMP: unconditional JuMP
void op_jmp(proc_t *proc)
{
  proc->branch_target = proc->uinst & 0x07ff;
  proc->branch = true;
}

// 0800..0fff RFS: Return From Subroutine
void op_rfs(proc_t *proc)
{
  proc->branch_target = proc->return_stack;
  proc->branch = true;
}

// 10xx JZBF: Jump if Zero Bit False
void op_jzbf(proc_t *proc)
{
  if (! proc->zb)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 11xx JZBT: Jump if Zero Bit True
void op_jzbt(proc_t *proc)
{
  if (proc->zb)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 12xx JC8F: Jump if Carry 8 False
void op_jc8f(proc_t *proc)
{
  if (! proc->c8)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 13xx JC8T: Jump if Carry 8 True
void op_jc8t(proc_t *proc)
{
  if (proc->c8)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 14xx JIF: Jump if Indirect condition code False
void op_jif(proc_t *proc)
{
  if (! icc(proc))
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 15xx JIT: Jump if Indirect condition code True
void op_jit(proc_t *proc)
{
  if (icc(proc))
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 16xx JNBF: Jump if Negative Bit False
void op_jnbf(proc_t *proc)
{
  if (! proc->nb)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 17xx JNBT: Jump if Negative Bit True
void op_jnbt(proc_t *proc)
{
  if (proc->nb)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 18xx JZF: Jump if Zero condition code False
void op_jzf(proc_t *proc)
{
  if (! proc->cc_z)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 19xx JZT: Jump if Zero condition code True
void op_jzt(proc_t *proc)
{
  if (proc->cc_z)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 1axx JCF: Jump if Carry condition code False
void op_jcf(proc_t *proc)
{
  if (! proc->cc_c)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 1bxx JCT: Jump if Carry condition code True
void op_jct(proc_t *proc)
{
  if (proc->cc_c)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 1cxx JVF: Jump if oVerflow condition code False
void op_jvf(proc_t *proc)
{
  if (! proc->cc_v)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 1dxx JVT: Jump if oVerflow condition code True
void op_jvt(proc_t *proc)
{
  if (proc->cc_v)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 1exx JNF: Jump if Negative condition code False
void op_jnf(proc_t *proc)
{
  if (! proc->cc_n)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 1fxx JNT: Jump if Negative condition code True
void op_jnt(proc_t *proc)
{
  if (proc->cc_n)
    {
      proc->branch_target = (proc->pc & 0x0700) | (proc->uinst & 0x00ff);
      proc->branch = true;
    }
}

// 2xxx AL: Add Literal
void op_al(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  uint8_t da = read_reg_8(proc, a);
  uint8_t dl = (proc->uinst >> 4) & 0xff;
  uint8_t result;

  result = add_8(da, dl, 0,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4);
  // condition codes unaffected
  write_reg_8(proc, a, result);
}

// 3xxx CL: Compare Literal
void op_cl(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  uint8_t da = read_reg_8(proc, a);
  uint8_t dl = (proc->uinst >> 4) & 0xff;
  uint8_t result;

  result = sub_8(da, dl, 0,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4);
  // condition codes unaffected
  // no result written to register
}

// 4xxx NL: aNd Literal
void op_nl(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  uint8_t d = read_reg_8(proc, a);
  d &= ((proc->uinst >> 4) & 0xff);
  write_reg_8(proc, a, d);
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // C4, C8 flags unaffected
  // condition codes unaffected
}

// 5xxx TL: Test Literal
void op_tl(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  uint8_t d = read_reg_8(proc, a);
  d &= ((proc->uinst >> 4) & 0xff);
  // no result written to register
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // C4, C8 flags unaffected
  // condition codes unaffected
}

// 6xxx LL: Load Literal
void op_ll(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  uint8_t d = ((proc->uinst >> 4) & 0xff);
  write_reg_8(proc, a, d);
  // NB, ZB C4, C8 flags unaffected
  // condition codes unaffected
}

// 70xx RI: Reset Interrupts
void op_ri(proc_t *proc)
{
  int b = (proc->uinst >> 4) & 0x0f;
  if (b & 4)
    proc->interrupt &= ~(1 << 6);
  if (b & 2)
    proc->interrupt &= ~(1 << 5);
  if (b & 1)
    proc->interrupt &= ~(1 << 4);
  // no flags affected
}

// 71xx SI: Set Interrupts
void op_si(proc_t *proc)
{
  int b = (proc->uinst >> 4) & 0x0f;
  if (b & 4)
    proc->interrupt |= (1 << 6);
  if (b & 2)
    proc->interrupt |= (1 << 5);
  if (b & 1)
    proc->interrupt |= (1 << 4);
  // no flags affected
}

// 72xx CCF: Copy Condition Flags
void op_ccf(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  uint8_t d = ((proc->nb << 7) |
	       (proc->zb << 6) |
	       (proc->c4 << 5) |
	       (proc->c8 << 4) |
	       (proc->cc_n << 3) |
	       (proc->cc_z << 2) |
	       (proc->cc_v << 1) |
	       (proc->cc_c << 0));
  write_reg_8(proc, a, d);
}

// 73xx LCF: Load Condition Flags
void op_lcf(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  uint8_t d = read_reg_8(proc, a);
  proc->nb = (d >> 7) & 1;
  proc->zb = (d >> 6) & 1;
  proc->c4 = (d >> 5) & 1;
  proc->c8 = (d >> 4) & 1;
  if (proc->uinst & 0x0080)
    proc->cc_n = (d >> 3) & 1;
  if (proc->uinst & 0x0040)
    proc->cc_z = (d >> 2) & 1;
  if (proc->uinst & 0x0020)
    proc->cc_v = (d >> 1) & 1;
  if (proc->uinst & 0x0010)
    proc->cc_c = (d >> 0) & 1;
}

// 74xx RTSR: Reset Translation State Register
void op_rtsr(proc_t *proc)
{
  proc->translation_state = 0;
}

// 75xx LGL: Load G Low
void op_lgl(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  proc->g = read_reg_8(proc, a) & 0x7;
}

// 76xx CIB: Conditionally Increment Byte
void op_cib(proc_t *proc)
{
  if (! proc->c8)
    return;
  int a = proc->uinst & 0x0f;
  uint8_t da = read_reg_8(proc, a);
  uint8_t result;
  
  result = add_8(da, 1, 0,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4);
  // condition codes unaffected
  write_reg_8(proc, a, result);
}

// 77xx CDB: Conditionally Decrement Byte
void op_cdb(proc_t *proc)
{
  if (! proc->c8)
    return;
  int a = proc->uinst & 0x0f;
  uint8_t da = read_reg_8(proc, a);
  uint8_t result;

  result = sub_8(da, 1, 0,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4);
  // condition codes unaffected
  write_reg_8(proc, a, result);
}

// 80xx MB: Move Byte
// 81xx MBF: Move Byte, update condition code Flags
void op_mb(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t d = read_reg_8(proc, b);
  write_reg_8(proc, a, d);
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // c4, c8 not affected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // cc_c not affected
    }
}

// 82xx MW: Move Word
// 83xx MWF: Move Word, update condition code Flags
void op_mw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_b(proc, b);
  write_reg_16_a(proc, a, d);
  proc->nb = d >> 15;
  proc->zb = d == 0;
  // c4, c8 not affected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // cc_c not affected
    }
}

// 84xx CMB: Conditionally Move Byte
// 85xx CMBF: Conditionally Move Byte, update condition code Flags
void op_cmb(proc_t *proc)
{
  if (! proc->cc_c)
    return;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t d = read_reg_8(proc, b);
  write_reg_8(proc, a, d);
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // c4, c8 not affected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // cc_c not affected
    }
}

// 86xx CMW: Conditionally Move Word
// 87xx CMWF: Conditionally Move Word, update condition code Flags
void op_cmw(proc_t *proc)
{
  proc->two_cycle = true;
  if (! proc->cc_c)
    return;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_b(proc, b);
  write_reg_16_a(proc, a, d);
  proc->nb = d >> 15;
  proc->zb = d == 0;
  // c4, c8 not affected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // cc_c not affected
    }
}

// 88xx SLBC: Shift Left Byte with Carry
// 89xx SLBCF: Shift Left Byte with Carry, update condition code Flags
void op_slbc(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t d = read_reg_8(proc, b);
  proc->c8 = d >> 7;
  d <<= 1;
  d |= proc->cc_c;
  write_reg_8(proc, a, d);
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // XXX c4 = ?
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      // XXX proc->cc_v = ???
      proc->cc_c = proc->c8;
    }
}

// 8axx SLWC: Shift Left Word with Carry
// 8bxx SLWCF: Shift Left Word with Carry, update condition code Flags
void op_slwc(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_b(proc, b);
  proc->c8 = d >>15;
  d <<= 1;
  d |= proc->cc_c;
  write_reg_16_a(proc, a, d);
  proc->nb = d >> 15;
  proc->zb = d == 0;
  // XXX c4 = ?
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      // XXX proc->cc_v = ???
      proc->cc_c = proc->c8;
    }
}

// 8cxx SLB: Shift Left Byte
// 8dxx SLBF: Shift Left Byte, update condition code Flags
void op_slb(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t d = read_reg_8(proc, b);
  proc->c8 = d >> 7;
  d <<= 1;
  write_reg_8(proc, a, d);
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // XXX c4 = ?
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      // XXX proc->cc_v = ???
      proc->cc_c = proc->c8;
    }
}

// 8exx SLW: Shift Left Word
// 8fxx SLWF: Shift Left Word, update condition code Flags
void op_slw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_b(proc, b);
  proc->c8 = d >>15;
  d <<= 1;
  write_reg_16_a(proc, a, d);
  proc->nb = d >> 15;
  proc->zb = d == 0;
  // XXX c4 = ?
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      // XXX proc->cc_v = ???
      proc->cc_c = proc->c8;
    }
}

// 90xx ICB1: InCrement Byte by 1
// 91xx ICB1F: InCrement Byte by 1, update condition code Flags
void op_icb1(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t db = read_reg_8(proc, b);
  uint8_t result;

  result = add_8(db, 1, 0,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		 ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_8(proc, a, result);
}

// 92xx ICW1: InCrement Word by 1
// 93xx ICW1F: InCrement Word by 1, update condition code Flags
void op_icw1(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t db = read_reg_16_b(proc, b);
  uint16_t result;

  result = add_16(db, 1, 0,
		  proc,
		  UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		  ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_16_a(proc, a, result);
}

// 94xx ICB2: InCrement Byte by 2
// 95xx ICB2F: InCrement Byte by 2, update condition code Flags
void op_icb2(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t db = read_reg_8(proc, b);
  uint8_t result;

  result = add_8(db, 2, 0,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		 ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_8(proc, a, result);
}

// 96xx ICW2: InCrement Word by 2
// 97xx ICW2F: InCrement Word by 2, update condition code Flags
void op_icw2(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t db = read_reg_16_b(proc, b);
  uint16_t result;

  result = add_16(db, 2, 0,
		  proc,
		  UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		  ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_16_a(proc, a, result);
}

// 98xx TCB: Two's Complement Byte
// 99xx TCBF: Two's Complement Byte, update condition code Flags
void op_tcb(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t da = read_reg_8(proc, a);
  uint8_t db = read_reg_8(proc, b);
  uint8_t result;

  result = sub_8(0, db, 0,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		 ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_8(proc, a, result);
}

// 9axx TCW: Two's Complement Word
// 9bxx TCWF: Two's Complement Word, update condition code Flags
void op_tcw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t da = read_reg_16_b(proc, a);
  uint16_t db = read_reg_16_b(proc, b);
  uint16_t result;

  result = sub_16(0, db, 0,
		  proc,
		  UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		  ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_16_a(proc, a, result);
}

// 9cxx OCB: One's Complement Byte
// 9dxx OCBF: One's Complement Byte, update condition code Flags
void op_ocb(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t d = read_reg_8(proc, b);
  d ^= 0xff;
  write_reg_8(proc, a, d);
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // C4, C8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      // proc->cc_v unaffected
      proc->cc_c = proc->c8;
    }
}

// 9exx OCW: One's Complement Word
// 9fxx OCWF: One's Complement Word, update condition code Flags
void op_ocw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_b(proc, b);
  d ^= 0xffff;
  write_reg_16_a(proc, a, d);
  proc->nb = d >> 15;
  proc->zb = d == 0;
  // C4, C8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      // proc->cc_v unaffected
      proc->cc_c = proc->c8;
    }
}

// a0xx AB: Add Byte
// a1xx ABF: Add Byte, update condition code Flags
void op_ab(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t da = read_reg_8(proc, a);
  uint8_t db = read_reg_8(proc, b);
  uint8_t result;

  result = add_8(da, db, 0,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		 ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_8(proc, a, result);
}

// a2xx AW: Add Word
// a3xx AWF: Add Word, update condition code Flags
void op_aw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t da = read_reg_16_b(proc, a);
  uint16_t db = read_reg_16_b(proc, b);
  uint16_t result;

  result = add_16(da, db, 0,
		  proc,
		  UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		  ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_16_a(proc, a, result);
}

// a4xx CAB: Conditionally Add Byte
// a5xx CABF: Conditionally Add Byte, update condition code Flags
void op_cab(proc_t *proc)
{
  if (! proc->cc_c)
    return;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t da = read_reg_8(proc, a);
  uint8_t db = read_reg_8(proc, b);
  uint8_t result;

  result = add_8(da, db, 0,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		 ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_8(proc, a, result);
}

// a6xx CAW: Conditionally Add Word
// a7xx CAWF: Conditionally Add Word, update condition code Flags
void op_caw(proc_t *proc)
{
  proc->two_cycle = true;
  if (! proc->cc_c)
    return;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t da = read_reg_16_b(proc, a);
  uint16_t db = read_reg_16_b(proc, b);
  uint16_t result;

  result = add_16(da, db, 0,
		  proc,
		  UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		  ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_16_a(proc, a, result);
}

// a8xx ABC: Add Byte with Carry
// a9xx ABCF: Add Byte with Carry, update condition code Flags
void op_abc(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t da = read_reg_8(proc, a);
  uint8_t db = read_reg_8(proc, b);
  uint8_t result;

  result = add_8(da, db, proc->cc_c,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		 ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_8(proc, a, result);
}

// aaxx AWC: Add Word with Carry
// abxx AWCF: Add Word with Carry, update condition code Flags
void op_awc(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t da = read_reg_16_b(proc, a);
  uint16_t db = read_reg_16_b(proc, b);
  uint16_t result;

  result = add_16(da, db, proc->cc_c,
		  proc,
		  UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		  ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_16_a(proc, a, result);
}

// acxx CAD: Conditionally Add Digits
void op_cad(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t db = read_reg_8(proc, b);
  uint8_t da = read_reg_8(proc, a);

  if (proc->c4)
    {
      uint8_t ld = (da & 0x0f) + (db & 0x0f);
      proc->c4 = ld >> 4;
      da = (da & 0xf0) | (ld & 0x0f);
    }
  if (proc->c8)
    {
      uint8_t ld = (da >> 4) + (db >> 4);
      proc->c8 = ld >> 4;
      da = (ld & 0x0f) | (da & 0x0f);
    }
  write_reg_8(proc, a, da);
  proc->nb = da >> 7;
  proc->zb = da == 0;
  proc->cc_n = proc->nb;
  proc->cc_z = proc->zb;
  // XXX proc->cc_v = ?
  proc->cc_c = proc->c8;
}

// aexx CAWI: Conditionally Add Word on ICC
// afxx CAWIF: Conditionally Add Word on ICC, update condition code Flags
void op_cawi(proc_t *proc)
{
  proc->two_cycle = true;
  if (! icc(proc))
    return;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t da = read_reg_16_b(proc, a);
  uint16_t db = read_reg_16_b(proc, b);
  uint16_t result;

  result = add_16(da, db, 0,
		  proc,
		  UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		  ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_16_a(proc, a, result);
}

// b0xx SB: Subtract Byte
// b1xx SBF: Subtract Byte, update condition code Flags
void op_sb(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t da = read_reg_8(proc, a);
  uint8_t db = read_reg_8(proc, b);
  uint8_t result;

  result = sub_8(da, db, 0,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		 ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_8(proc, a, result);
}

// b2xx SW: Subtract Word
// b3xx SWF: Subtract Word, update condition code Flags
void op_sw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t da = read_reg_16_b(proc, a);
  uint16_t db = read_reg_16_b(proc, b);
  uint16_t result;

  result = sub_16(da, db, 0,
		  proc,
		  UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		  ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_16_a(proc, a, result);
}

// b4xx CB: Compare Byte
// b5xx CBF: Compare Byte, update condition code Flags
void op_cb(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t da = read_reg_8(proc, a);
  uint8_t db = read_reg_8(proc, b);
  uint8_t result;

  result = sub_8(da, db, 0,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		 ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  // no result written to register
}

// b6xx CW: Compare Word
// b7xx CWF: Compare Word, update condition code Flags
void op_cw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t da = read_reg_16_b(proc, a);
  uint16_t db = read_reg_16_b(proc, b);
  uint16_t result;

  result = sub_16(da, db, 0,
		  proc,
		  UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		  ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  // no result written to register
}

// b8xx SBC: Subtract Byte with Carry
// b9xx SBCF: Subtract Byte with Carry, update condition code Flags
void op_sbc(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t da = read_reg_8(proc, a);
  uint8_t db = read_reg_8(proc, b);
  uint8_t result;

  result = sub_8(da, db, proc->cc_c,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		 ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_8(proc, a, result);
}

// baxx SWC: Subtract Word with Carry
// bbxx SWCF: Subtract Word with Carry, update condition code Flags
void op_swc(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t da = read_reg_16_b(proc, a);
  uint16_t db = read_reg_16_b(proc, b);
  uint16_t result;

  result = sub_16(da, db, proc->cc_c,
		  proc,
		  UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		  ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_16_a(proc, a, result);
}

// bcxx DB1: Decrement Byte by 1
// bdxx DB1F: Decrement Byte by 1, update condition code Flags
void op_db1(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t db = read_reg_8(proc, b);
  uint8_t result;

  result = sub_8(db, 1, 0,
		 proc,
		 UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		 ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_8(proc, a, result);
}

// bexx DW1: Decrement Word by 1
// bfxx DW1F: Decrement Word by 1, update condition code Flags
void op_dw1(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t db = read_reg_16_b(proc, b);
  uint16_t result;

  result = sub_16(db, 1, 0,
		  proc,
		  UPD_NB | UPD_ZB | UPD_C8 | UPD_C4 |
		  ((proc->uinst & 0x0100) ? UPD_N | UPD_Z | UPD_V | UPD_C : 0));
  write_reg_16_a(proc, a, result);
}

// c0xx NB: aNd Byte
// c1xx NBF: aNd Byte, update condition code Flags
void op_nb(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t d = read_reg_8(proc, b);
  d &= read_reg_8(proc, a);
  write_reg_8(proc, a, d);
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// c2xx NW: aNd Word
// c3xx NWF: aNd Word, update condition code Flags
void op_nw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_b(proc, b);
  d &= read_reg_16_a(proc, a);
  write_reg_16_a(proc, a, d);
  proc->nb = d >> 15;
  proc->zb = d == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// c4xx TB: Test Byte
// c5xx TBF: Test Byte, update condition code Flags
void op_tb(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t d = read_reg_8(proc, b);
  d &= read_reg_8(proc, a);
  // no result written to register
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// c6xx TW: Test Word
// c7xx TWF: Test Word, update condition code Flags
void op_tw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_b(proc, b);
  d &= read_reg_16_a(proc, a);
  // no result written to register
  proc->nb = d >> 15;
  proc->zb = d == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// c8xx ORB: OR Byte
// c9xx ORBF: OR Byte, update condition code Flags
void op_orb(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t d = read_reg_8(proc, b);
  d |= read_reg_8(proc, a);
  write_reg_8(proc, a, d);
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// caxx ORW: OR Word
// cbxx ORWF: OR Word, update condition code Flags
void op_orw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_b(proc, b);
  d |= read_reg_16_a(proc, a);
  write_reg_16_a(proc, a, d);
  proc->nb = d >> 15;
  proc->zb = d == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// ccxx XB: eXclusive or Byte
// cdxx XBF: eXclusive or Byte, update condition code Flags
void op_xb(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t d = read_reg_8(proc, b);
  d ^= read_reg_8(proc, a);
  write_reg_8(proc, a, d);
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// cexx XW: eXclusive or Word
// cfxx XWF: eXclusive or Word, update condition code Flags
void op_xw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_b(proc, b);
  d ^= read_reg_16_a(proc, a);
  write_reg_16_a(proc, a, d);
  proc->nb = d >> 15;
  proc->zb = d == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// d0xx NCB: aNd Complement of Byte
// d1xx NCBF: aNd Complement of Byte, update condition code Flags
void op_ncb(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t d = read_reg_8(proc, a);
  d = d & ~ read_reg_8(proc, b);
  write_reg_8(proc, a, d);
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// d2xx NCW: aNd Complement of Word
// d3xx NCWF: aNd Complement of Word, update condition code Flags
void op_ncw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_a(proc, a);
  d &= ~ read_reg_16_b(proc, b);
  write_reg_16_a(proc, a, d);
  proc->nb = d >> 15;
  proc->zb = d == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// d8xx SRBC: Shift Right Byte with Carry
// d9xx SRBCF: Shift Right Byte with Carry, update condition code Flags
void op_srbc(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t d = read_reg_8(proc, b);
  proc->c8 = d & 1;
  d >>= 1;
  d |= (proc->cc_c << 7);
  write_reg_8(proc, a, d);
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // c4 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      // proc->cc_v unaffected
      proc->cc_c = proc->c8;
    }
}

// daxx SRWC: Shift Right Word with Carry
// dbxx SRWCF: Shift Right Word with Carry, update condition code Flags
void op_srwc(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  if (((a & 1) != 1) || ((b & 1) != 1))
    {
      fprintf(stderr, "srwc[f] doesn't have both A and B odd\n");
    }
  a &= 0xe;
  b &= 0xe;
  uint16_t d = read_reg_16_b(proc, b);
  proc->c8 = d & 1;
  d >>= 1;
  d |= (proc->cc_c << 15);
  write_reg_16_a(proc, a, d);
  proc->nb = d >> 15;
  proc->zb = d == 0;
  // c4 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      // proc->cc_v unaffected
      proc->cc_c = proc->c8;
    }
}

// dcxx SRB: Shift Right Byte
// ddxx SRBF: Shift Right Byte, update condition code Flags
void op_srb(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t d = read_reg_8(proc, b);
  proc->c8 = d & 1;
  d >>= 1;
  write_reg_8(proc, a, d);
  proc->nb = d >> 7;
  proc->zb = d == 0;
  // c4 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      // proc->cc_v unaffected
      proc->cc_c = proc->c8;
    }
}

// dexx SRW: Shift Right Word
// dfxx SRWF: Shift Right Word, update condition code Flags
void op_srw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  if (((a & 1) != 1) || ((b & 1) != 1))
    {
      fprintf(stderr, "srwc[f] doesn't have both A and B odd\n");
    }
  a &= 0xe;
  b &= 0xe;
  uint16_t d = read_reg_16_b(proc, b);
  proc->c8 = d & 1;
  d >>= 1;
  write_reg_16_a(proc, a, d);
  proc->nb = d >> 15;
  proc->zb = d == 0;
  // c4 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      // proc->cc_v unaffected
      proc->cc_c = proc->c8;
    }
}

// e0xx IB: Input Byte
// e1xx IBF: Input Byte, update condition code Flags
void op_ib(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  bool rmw = b & 0x4;
  uint8_t d16 = ext_input(proc,
			  true,  // wait for reply
			  false, // word
			  false, // status
			  rmw);
  // which byte is read depends on (b & 0x3), and possibly (addr & 0x1)
  uint8_t d8;
  switch (b & 3)
    {
    case 0: d8 = d16 >> 8;   break;
    case 1: d8 = d16 & 0xff; break;
    case 2: if (proc->addr & 1)
	d8 = d16 >> 8;
      else
	d8 = d16 & 0xff;
      break;
    case 3: if (proc->addr & 1)
	d8 = d16 & 0xff;
      else
	d8 = d16 >> 8;
      break;
    }
  write_reg_8(proc, a, d8);
  proc->nb = d8 >> 7;
  proc->zb = d8 == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// e2xx IW: Input Word
// e3xx IWF: Input Word, update condition code Flags
void op_iw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  bool rmw = b & 0x4;
  uint16_t d16 = ext_input(proc,
			   true,  // wait for reply
			   true,  // word
			   false, // status
			   rmw);
  write_reg_16_a(proc, a, d16);
  if (b & 0x3)
    {
      proc->translation_register = d16;
      // XXX set ICC flag
    }
  if ((b & 0x3) == 1)
    proc->g = (d16 >> 4) & 7;
  else if ((b & 0x3) == 2)
    proc->g = (d16 >> 6) & 7;
  proc->nb = d16 >> 15;
  proc->zb = d16 == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// e4xx ISB: Input Status Byte
// e5xx ISBF: Input Status Byte, update condition code Flags
void op_isb(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint8_t d16 = ext_input(proc,
			  false, // wait for reply
			  false, // word
			  true, // status
			  false);
  // which byte is read depends on (b & 0x3), and possibly (addr & 0x1)
  uint8_t d8;
  switch (b & 3)
    {
    case 0: d8 = d16 >> 8;   break;
    case 1: d8 = d16 & 0xff; break;
    case 2: if (proc->addr & 1)
	d8 = d16 >> 8;
      else
	d8 = d16 & 0xff;
      break;
    case 3: if (proc->addr & 1)
	d8 = d16 & 0xff;
      else
	d8 = d16 >> 8;
      break;
    }
  write_reg_8(proc, a, d8);
  proc->nb = d8 >> 7;
  proc->zb = d8 == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// e6xx ISW: Input Status Word
// e7xx ISWF: Input Status Word, update condition code Flags
void op_isw(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  bool rmw = b & 0x4;
  uint8_t d16 = ext_input(proc,
			  false,  // wait for reply
			  true,  // word
			  true, // status
			  rmw);

  write_reg_16_a(proc, a, d16);
  proc->nb = d16 >> 15;
  proc->zb = d16 == 0;
  // c4, c8 unaffected
  if (proc->uinst & 0x0100)
    {
      proc->cc_n = proc->nb;
      proc->cc_z = proc->zb;
      proc->cc_v = 0;
      // proc->cc_c unaffected
    }
}

// ecxx MI: Modify microInstruction
void op_mi(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_ba(proc, b, a);
  proc->modify_uinst = d;
}

// eexx LTR: Load Translation Register
void op_ltr(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_ba(proc, b, a);
  proc->translation_register = d;
}

// f0xx RIB1: Read and Increment Byte by One
void op_rib1(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t addr = read_reg_16_ba(proc, b, a);
  ext_cycle_start(proc,
		  addr,
		  false,  // acknowledge
		  false); // write
  uint8_t d8 = (addr & 0xff) + 1;
  write_reg_8(proc, a, d8);
  proc->nb = d8 >> 7;
  proc->zb = d8 == 0;
  // XXX set c4
  // XXX set c8
  // condition codes unaffected
  // should be followed by IB or IW
}

// f1xx WIB1: Write and Increment Byte by One
void op_wib1(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t addr = read_reg_16_ba(proc, b, a);
  ext_cycle_start(proc,
		  addr,
		  false, // acknowledge
		  true); // write
  uint8_t d8 = (addr & 0xff) + 1;
  write_reg_8(proc, a, d8);
  proc->nb = d8 >> 7;
  proc->zb = d8 == 0;
  // XXX set c4
  // XXX set c8
  // condition codes unaffected
  // should be followed by OB or OW
}

// f2xx RIW1: Read and Increment Word by One
void op_riw1(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t addr = read_reg_16_ba(proc, b, a);
  ext_cycle_start(proc,
		  addr,
		  false,  // acknowledge
		  false); // write
  addr += 1;
  write_reg_16_ba(proc, b, a, addr);
  proc->nb = addr >> 15;
  proc->zb = addr == 0;
  // XXX set c4
  // XXX set c8
  // condition codes unaffected
  // should be followed by IB or IW
}

// f3xx WIW1: Write and Increment Word by One
void op_wiw1(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t addr = read_reg_16_ba(proc, b, a);
  ext_cycle_start(proc,
		  addr,
		  false, // acknowledge
		  true); // write
  addr += 1;
  write_reg_16_ba(proc, b, a, addr);
  proc->nb = addr >> 15;
  proc->zb = addr == 0;
  // XXX set c4
  // XXX set c8
  // condition codes unaffected
  // should be followed by OB or OW
}

// f4xx RIB2: Read and Increment Byte by Two
void op_rib2(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t addr = read_reg_16_ba(proc, b, a);
  ext_cycle_start(proc,
		  addr,
		  false,  // acknowledge
		  false); // write
  uint8_t d8 = (addr & 0xff) + 2;
  write_reg_8(proc, a, d8);
  proc->nb = d8 >> 7;
  proc->zb = d8 == 0;
  // XXX set c4
  // XXX set c8
  // condition codes unaffected
  // should be followed by IB or IW
}

// f5xx WIB2: Write and Increment Byte by Two
void op_wib2(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t addr = read_reg_16_ba(proc, b, a);
  ext_cycle_start(proc,
		  addr,
		  false, // acknowledge
		  true); // write
  uint8_t d8 = (addr & 0xff) + 2;
  write_reg_8(proc, a, d8);
  proc->nb = d8 >> 7;
  proc->zb = d8 == 0;
  // XXX set c4
  // XXX set c8
  // condition codes unaffected
  // should be followed by OB or OW
}

// f6xx RIW2: Read and Increment Word by Two
void op_riw2(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t addr = read_reg_16_ba(proc, b, a);
  ext_cycle_start(proc,
		  addr,
		  false,  // acknowledge
		  false); // write
  addr += 2;
  write_reg_16_ba(proc, b, a, addr);
  proc->nb = addr >> 15;
  proc->zb = addr == 0;
  // XXX set c4
  // XXX set c8
  // condition codes unaffected
  // should be followed by IB or IW
}

// f7xx WIW2: Write and Increment Word by Two
void op_wiw2(proc_t *proc)
{
  proc->two_cycle = true;
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t addr = read_reg_16_ba(proc, b, a);
  ext_cycle_start(proc,
		  addr,
		  false, // acknowledge
		  true); // write
  addr += 2;
  write_reg_16_ba(proc, b, a, addr);
  proc->nb = addr >> 15;
  proc->zb = addr == 0;
  // XXX set c4
  // XXX set c8
  // condition codes unaffected
  // should be followed by OB or OW
}

// f8xx R: Read
void op_r(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t addr = read_reg_16_ba(proc, b, a);
  ext_cycle_start(proc,
		  addr,
		  false,  // acknowledge
		  false); // write
  // status bits unaffected
  // condition codes unaffected
  // should be followed by IB or IW
}

// f9xx W: Write
void op_w(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t addr = read_reg_16_ba(proc, b, a);
  ext_cycle_start(proc,
		  addr,
		  false, // acknowledge
		  true); // write
  // status bits unaffected
  // condition codes unaffected
  // should be followed by OB or OW
}

// faxx RA: Read Acknowledge
void op_ra(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t addr = read_reg_16_ba(proc, b, a);
  ext_cycle_start(proc,
		  addr,
		  true,   // acknowledge
		  false); // write
  // status bits unaffected
  // condition codes unaffected
  // should be followed by IB or IW
}

// fbxx WA: Write Acknowledge
void op_wa(proc_t *proc)
{
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t addr = read_reg_16_ba(proc, b, a);
  ext_cycle_start(proc,
		  addr,
		  true,  // acknowledge
		  true); // write
  // status bits unaffected
  // condition codes unaffected
  // should be followed by OB or OW
}

// fcxx OB: Output Byte
void op_ob(proc_t *proc)
{
  // XXX only one cycle?
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_ba(proc, b, a);
  ext_output(proc,
	     true,  // wait for reply
	     false, // word
	     false, // status
	     d);
}
      
// fdxx OW: Output Word
void op_ow(proc_t *proc)
{
  // XXX only one cycle?
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_ba(proc, b, a);
  ext_output(proc,
	     true,  // wait for reply
	     true,  // word
	     false, // status
	     d);
}
      
// fexx OS: Output Status
// Note: normally used without a preceding Write instruction.
void op_os(proc_t *proc)
{
  // XXX only one cycle?
  int a = proc->uinst & 0x0f;
  int b = (proc->uinst >> 4) & 0x0f;
  uint16_t d = read_reg_16_ba(proc, b, a);
  ext_output(proc,
	     false, // wait for reply
	     true,  // word
	     true,  // status
	     d);
}

// ffxx NOP: No OPeration
void op_nop(proc_t *proc)
{
}


void (*op[256])(proc_t *proc) =
{
  [0x00] = op_jmp,
  [0x01] = op_jmp,
  [0x02] = op_jmp,
  [0x03] = op_jmp,
  [0x04] = op_jmp,
  [0x05] = op_jmp,
  [0x06] = op_jmp,
  [0x07] = op_jmp,
  [0x08] = op_rfs,
  [0x09] = op_rfs,
  [0x0a] = op_rfs,
  [0x0b] = op_rfs,
  [0x0c] = op_rfs,
  [0x0d] = op_rfs,
  [0x0e] = op_rfs,
  [0x0f] = op_rfs,

  [0x10] = op_jzbf,
  [0x11] = op_jzbt,
  [0x12] = op_jc8f,
  [0x13] = op_jc8t,
  [0x14] = op_jif,
  [0x15] = op_jit,
  [0x16] = op_jnbf,
  [0x17] = op_jnbt,
  [0x18] = op_jzf,
  [0x19] = op_jzt,
  [0x1a] = op_jcf,
  [0x1b] = op_jct,
  [0x1c] = op_jvf,
  [0x1d] = op_jvt,
  [0x1e] = op_jnf,
  [0x1f] = op_jnt,

  [0x20] = op_al,
  [0x21] = op_al,
  [0x22] = op_al,
  [0x23] = op_al,
  [0x24] = op_al,
  [0x25] = op_al,
  [0x26] = op_al,
  [0x27] = op_al,
  [0x28] = op_al,
  [0x29] = op_al,
  [0x2a] = op_al,
  [0x2b] = op_al,
  [0x2c] = op_al,
  [0x2d] = op_al,
  [0x2e] = op_al,
  [0x2f] = op_al,

  [0x30] = op_cl,
  [0x31] = op_cl,
  [0x32] = op_cl,
  [0x33] = op_cl,
  [0x34] = op_cl,
  [0x35] = op_cl,
  [0x36] = op_cl,
  [0x37] = op_cl,
  [0x38] = op_cl,
  [0x39] = op_cl,
  [0x3a] = op_cl,
  [0x3b] = op_cl,
  [0x3c] = op_cl,
  [0x3d] = op_cl,
  [0x3e] = op_cl,
  [0x3f] = op_cl,

  [0x40] = op_nl,
  [0x41] = op_nl,
  [0x42] = op_nl,
  [0x43] = op_nl,
  [0x44] = op_nl,
  [0x45] = op_nl,
  [0x46] = op_nl,
  [0x47] = op_nl,
  [0x48] = op_nl,
  [0x49] = op_nl,
  [0x4a] = op_nl,
  [0x4b] = op_nl,
  [0x4c] = op_nl,
  [0x4d] = op_nl,
  [0x4e] = op_nl,
  [0x4f] = op_nl,

  [0x50] = op_tl,
  [0x51] = op_tl,
  [0x52] = op_tl,
  [0x53] = op_tl,
  [0x54] = op_tl,
  [0x55] = op_tl,
  [0x56] = op_tl,
  [0x57] = op_tl,
  [0x58] = op_tl,
  [0x59] = op_tl,
  [0x5a] = op_tl,
  [0x5b] = op_tl,
  [0x5c] = op_tl,
  [0x5d] = op_tl,
  [0x5e] = op_tl,
  [0x5f] = op_tl,

  [0x60] = op_ll,
  [0x61] = op_ll,
  [0x62] = op_ll,
  [0x63] = op_ll,
  [0x64] = op_ll,
  [0x65] = op_ll,
  [0x66] = op_ll,
  [0x67] = op_ll,
  [0x68] = op_ll,
  [0x69] = op_ll,
  [0x6a] = op_ll,
  [0x6b] = op_ll,
  [0x6c] = op_ll,
  [0x6d] = op_ll,
  [0x6e] = op_ll,
  [0x6f] = op_ll,

  [0x70] = op_ri,
  [0x71] = op_si,
  [0x72] = op_ccf,
  [0x73] = op_lcf,
  [0x74] = op_rtsr,
  [0x75] = op_lgl,
  [0x76] = op_cib,
  [0x77] = op_cdb,
  [0x78] = op_illegal,
  [0x79] = op_illegal,
  [0x7a] = op_illegal,
  [0x7b] = op_illegal,
  [0x7c] = op_illegal,
  [0x7d] = op_illegal,
  [0x7e] = op_illegal,
  [0x7f] = op_illegal,
  
  [0x80] = op_mb,
  [0x81] = op_mb, // MBF
  [0x82] = op_mw,
  [0x83] = op_mw, // MWF
  [0x84] = op_cmb,
  [0x85] = op_cmb, // CMBF
  [0x86] = op_cmw,
  [0x87] = op_cmw, // CMWF
  [0x88] = op_slbc,
  [0x89] = op_slbc, // SLBCF
  [0x8a] = op_slwc,
  [0x8b] = op_slwc, // SLWCF
  [0x8c] = op_slb,
  [0x8d] = op_slb, // SLBF
  [0x8e] = op_slw,
  [0x8f] = op_slw, // SLWF

  [0x90] = op_icb1,
  [0x91] = op_icb1, // ICB1F
  [0x92] = op_icw1,
  [0x93] = op_icw1, // ICW1F
  [0x94] = op_icb2,
  [0x95] = op_icb2, // ICB2F
  [0x96] = op_icw2,
  [0x97] = op_icw2, // ICW2F
  [0x98] = op_tcb,
  [0x99] = op_tcb, // TCBF
  [0x9a] = op_tcw,
  [0x9b] = op_tcw, // TCWF
  [0x9c] = op_ocb,
  [0x9d] = op_ocb, // OCBF
  [0x9e] = op_ocw,
  [0x9f] = op_ocw, // OCWF

  [0xa0] = op_ab,
  [0xa1] = op_ab, // ABF
  [0xa2] = op_aw,
  [0xa3] = op_aw, // AWF
  [0xa4] = op_cab,
  [0xa5] = op_cab, // CABF
  [0xa6] = op_caw,
  [0xa7] = op_caw, // CAWF
  [0xa8] = op_abc,
  [0xa9] = op_abc, // ABCF
  [0xaa] = op_awc,
  [0xab] = op_awc, // AWCF
  [0xac] = op_cad,
  [0xad] = op_illegal,
  [0xae] = op_cawi,
  [0xaf] = op_cawi, // CAWIF

  [0xb0] = op_sb,
  [0xb1] = op_sb, // SBF
  [0xb2] = op_sw,
  [0xb3] = op_sw, // SWF
  [0xb4] = op_cb,
  [0xb5] = op_cb, // CBF
  [0xb6] = op_cw,
  [0xb7] = op_cw, // CWF
  [0xb8] = op_sbc,
  [0xb9] = op_sbc, // SBCF
  [0xba] = op_swc,
  [0xbb] = op_swc, // SWCF
  [0xbc] = op_db1,
  [0xbd] = op_db1, // DB1F
  [0xbe] = op_dw1,
  [0xbf] = op_dw1, // DW1F

  [0xc0] = op_nb,
  [0xc1] = op_nb, // NBF
  [0xc2] = op_nw,
  [0xc3] = op_nw, // NWF
  [0xc4] = op_tb,
  [0xc5] = op_tb, // TBF
  [0xc6] = op_tw,
  [0xc7] = op_tw, // TWF
  [0xc8] = op_orb,
  [0xc9] = op_orb, // ORBF
  [0xca] = op_orw,
  [0xcb] = op_orw, // ORWF
  [0xcc] = op_xb,
  [0xcd] = op_xb, // XBF
  [0xce] = op_xw,
  [0xcf] = op_xw, // XWF

  [0xd0] = op_ncb,
  [0xd1] = op_ncb, // NCBF
  [0xd2] = op_ncw,
  [0xd3] = op_ncw, // NCWF
  [0xd4] = op_illegal,
  [0xd5] = op_illegal,
  [0xd6] = op_illegal,
  [0xd7] = op_illegal,
  [0xd8] = op_srbc,
  [0xd9] = op_srbc, // SRBCF
  [0xda] = op_srwc,
  [0xdb] = op_srwc, // SRWCF
  [0xdc] = op_srb,
  [0xdd] = op_srb, // SRBF
  [0xde] = op_srw,
  [0xdf] = op_srw, // SRWF

  [0xe0] = op_ib,
  [0xe1] = op_ib, // IBF
  [0xe2] = op_iw,
  [0xe3] = op_iw, // IWF
  [0xe4] = op_isb,
  [0xe5] = op_isb, // ISBF
  [0xe6] = op_isw,
  [0xe7] = op_isw, // ISWF
  [0xe8] = op_illegal,
  [0xe9] = op_illegal,
  [0xea] = op_illegal,
  [0xeb] = op_illegal,
  [0xec] = op_mi,
  [0xed] = op_illegal,
  [0xee] = op_ltr,
  [0xef] = op_illegal,

  [0xf0] = op_rib1,
  [0xf1] = op_wib1,
  [0xf2] = op_riw1,
  [0xf3] = op_wiw1,
  [0xf4] = op_rib2,
  [0xf5] = op_wib2,
  [0xf6] = op_riw2,
  [0xf7] = op_wiw2,
  [0xf8] = op_r,
  [0xf9] = op_w,
  [0xfa] = op_ra,
  [0xfb] = op_wa,
  [0xfc] = op_ob,
  [0xfd] = op_ow,
  [0xfe] = op_os,
  [0xff] = op_nop,
};

typedef enum
{
  FMT_NO_ARG,
  FMT_JMP,
  FMT_COND_JUMP,
  FMT_LITERAL,
  FMT_INTERRUPT,
  FMT_A_REG,
  FMT_B_REG_A_REG,
  FMT_B_LIT_A_REG,
  FMT_BA_REG,
} op_fmt_t;

typedef struct
{
  char *mnemonic;
  int format;
} op_info_t;

op_info_t op_info[256] =
{
  [0x00] = { "jmp", FMT_JMP },
  [0x01] = { "jmp", FMT_JMP },
  [0x02] = { "jmp", FMT_JMP },
  [0x03] = { "jmp", FMT_JMP },
  [0x04] = { "jmp", FMT_JMP },
  [0x05] = { "jmp", FMT_JMP },
  [0x06] = { "jmp", FMT_JMP },
  [0x07] = { "jmp", FMT_JMP },
  [0x08] = { "rfs", FMT_NO_ARG },
  [0x09] = { "rfs", FMT_NO_ARG },
  [0x0a] = { "rfs", FMT_NO_ARG },
  [0x0b] = { "rfs", FMT_NO_ARG },
  [0x0c] = { "rfs", FMT_NO_ARG },
  [0x0d] = { "rfs", FMT_NO_ARG },
  [0x0e] = { "rfs", FMT_NO_ARG },
  [0x0f] = { "rfs", FMT_NO_ARG },

  [0x10] = { "jzbf", FMT_COND_JUMP },
  [0x11] = { "jzbt", FMT_COND_JUMP },
  [0x12] = { "jc8f", FMT_COND_JUMP },
  [0x13] = { "jc8t", FMT_COND_JUMP },
  [0x14] = { "jif",  FMT_COND_JUMP },
  [0x15] = { "jit",  FMT_COND_JUMP },
  [0x16] = { "jnbf", FMT_COND_JUMP },
  [0x17] = { "jnbt", FMT_COND_JUMP },
  [0x18] = { "jzf",  FMT_COND_JUMP },
  [0x19] = { "jzt",  FMT_COND_JUMP },
  [0x1a] = { "jcf",  FMT_COND_JUMP },
  [0x1b] = { "jct",  FMT_COND_JUMP },
  [0x1c] = { "jvf",  FMT_COND_JUMP },
  [0x1d] = { "jvt",  FMT_COND_JUMP },
  [0x1e] = { "jnf",  FMT_COND_JUMP },
  [0x1f] = { "jnt",  FMT_COND_JUMP },

  [0x20] = { "al", FMT_LITERAL },
  [0x21] = { "al", FMT_LITERAL },
  [0x22] = { "al", FMT_LITERAL },
  [0x23] = { "al", FMT_LITERAL },
  [0x24] = { "al", FMT_LITERAL },
  [0x25] = { "al", FMT_LITERAL },
  [0x26] = { "al", FMT_LITERAL },
  [0x27] = { "al", FMT_LITERAL },
  [0x28] = { "al", FMT_LITERAL },
  [0x29] = { "al", FMT_LITERAL },
  [0x2a] = { "al", FMT_LITERAL },
  [0x2b] = { "al", FMT_LITERAL },
  [0x2c] = { "al", FMT_LITERAL },
  [0x2d] = { "al", FMT_LITERAL },
  [0x2e] = { "al", FMT_LITERAL },
  [0x2f] = { "al", FMT_LITERAL },

  [0x30] = { "cl", FMT_LITERAL },
  [0x31] = { "cl", FMT_LITERAL },
  [0x32] = { "cl", FMT_LITERAL },
  [0x33] = { "cl", FMT_LITERAL },
  [0x34] = { "cl", FMT_LITERAL },
  [0x35] = { "cl", FMT_LITERAL },
  [0x36] = { "cl", FMT_LITERAL },
  [0x37] = { "cl", FMT_LITERAL },
  [0x38] = { "cl", FMT_LITERAL },
  [0x39] = { "cl", FMT_LITERAL },
  [0x3a] = { "cl", FMT_LITERAL },
  [0x3b] = { "cl", FMT_LITERAL },
  [0x3c] = { "cl", FMT_LITERAL },
  [0x3d] = { "cl", FMT_LITERAL },
  [0x3e] = { "cl", FMT_LITERAL },
  [0x3f] = { "cl", FMT_LITERAL },

  [0x40] = { "nl", FMT_LITERAL },
  [0x41] = { "nl", FMT_LITERAL },
  [0x42] = { "nl", FMT_LITERAL },
  [0x43] = { "nl", FMT_LITERAL },
  [0x44] = { "nl", FMT_LITERAL },
  [0x45] = { "nl", FMT_LITERAL },
  [0x46] = { "nl", FMT_LITERAL },
  [0x47] = { "nl", FMT_LITERAL },
  [0x48] = { "nl", FMT_LITERAL },
  [0x49] = { "nl", FMT_LITERAL },
  [0x4a] = { "nl", FMT_LITERAL },
  [0x4b] = { "nl", FMT_LITERAL },
  [0x4c] = { "nl", FMT_LITERAL },
  [0x4d] = { "nl", FMT_LITERAL },
  [0x4e] = { "nl", FMT_LITERAL },
  [0x4f] = { "nl", FMT_LITERAL },

  [0x50] = { "tl", FMT_LITERAL },
  [0x51] = { "tl", FMT_LITERAL },
  [0x52] = { "tl", FMT_LITERAL },
  [0x53] = { "tl", FMT_LITERAL },
  [0x54] = { "tl", FMT_LITERAL },
  [0x55] = { "tl", FMT_LITERAL },
  [0x56] = { "tl", FMT_LITERAL },
  [0x57] = { "tl", FMT_LITERAL },
  [0x58] = { "tl", FMT_LITERAL },
  [0x59] = { "tl", FMT_LITERAL },
  [0x5a] = { "tl", FMT_LITERAL },
  [0x5b] = { "tl", FMT_LITERAL },
  [0x5c] = { "tl", FMT_LITERAL },
  [0x5d] = { "tl", FMT_LITERAL },
  [0x5e] = { "tl", FMT_LITERAL },
  [0x5f] = { "tl", FMT_LITERAL },

  [0x60] = { "ll", FMT_LITERAL },
  [0x61] = { "ll", FMT_LITERAL },
  [0x62] = { "ll", FMT_LITERAL },
  [0x63] = { "ll", FMT_LITERAL },
  [0x64] = { "ll", FMT_LITERAL },
  [0x65] = { "ll", FMT_LITERAL },
  [0x66] = { "ll", FMT_LITERAL },
  [0x67] = { "ll", FMT_LITERAL },
  [0x68] = { "ll", FMT_LITERAL },
  [0x69] = { "ll", FMT_LITERAL },
  [0x6a] = { "ll", FMT_LITERAL },
  [0x6b] = { "ll", FMT_LITERAL },
  [0x6c] = { "ll", FMT_LITERAL },
  [0x6d] = { "ll", FMT_LITERAL },
  [0x6e] = { "ll", FMT_LITERAL },
  [0x6f] = { "ll", FMT_LITERAL },

  [0x70] = { "ri",   FMT_INTERRUPT },
  [0x71] = { "si",   FMT_INTERRUPT },
  [0x72] = { "ccf",  FMT_A_REG },
  [0x73] = { "lcf",  FMT_A_REG },
  [0x74] = { "rtsr", FMT_NO_ARG },
  [0x75] = { "lgl",  FMT_A_REG },
  [0x76] = { "cib",  FMT_A_REG },
  [0x77] = { "cdb",  FMT_A_REG },
  [0x78] = { "illegal", FMT_NO_ARG },
  [0x79] = { "illegal", FMT_NO_ARG },
  [0x7a] = { "illegal", FMT_NO_ARG },
  [0x7b] = { "illegal", FMT_NO_ARG },
  [0x7c] = { "illegal", FMT_NO_ARG },
  [0x7d] = { "illegal", FMT_NO_ARG },
  [0x7e] = { "illegal", FMT_NO_ARG },
  [0x7f] = { "illegal", FMT_NO_ARG },
  
  [0x80] = { "mb",    FMT_B_REG_A_REG },
  [0x81] = { "mbf",   FMT_B_REG_A_REG },
  [0x82] = { "mw",    FMT_B_REG_A_REG },
  [0x83] = { "mwf",   FMT_B_REG_A_REG },
  [0x84] = { "cmb",   FMT_B_REG_A_REG },
  [0x85] = { "cmbf",  FMT_B_REG_A_REG },
  [0x86] = { "cmw",   FMT_B_REG_A_REG },
  [0x87] = { "cmwf",  FMT_B_REG_A_REG },
  [0x88] = { "slbc",  FMT_B_REG_A_REG },
  [0x89] = { "slbcf", FMT_B_REG_A_REG },
  [0x8a] = { "slwc",  FMT_B_REG_A_REG },
  [0x8b] = { "slwcf", FMT_B_REG_A_REG },
  [0x8c] = { "slb",   FMT_B_REG_A_REG },
  [0x8d] = { "slbf",  FMT_B_REG_A_REG },
  [0x8e] = { "slw",   FMT_B_REG_A_REG },
  [0x8f] = { "slwf",  FMT_B_REG_A_REG },

  [0x90] = { "icb1",  FMT_B_REG_A_REG },
  [0x91] = { "icb1f", FMT_B_REG_A_REG },
  [0x92] = { "icw1",  FMT_B_REG_A_REG },
  [0x93] = { "icw1f", FMT_B_REG_A_REG },
  [0x94] = { "icb2",  FMT_B_REG_A_REG },
  [0x95] = { "icb2f", FMT_B_REG_A_REG },
  [0x96] = { "icw2",  FMT_B_REG_A_REG },
  [0x97] = { "icw2f", FMT_B_REG_A_REG },
  [0x98] = { "tcb",   FMT_B_REG_A_REG },
  [0x99] = { "tcbf",  FMT_B_REG_A_REG },
  [0x9a] = { "tcw",   FMT_B_REG_A_REG },
  [0x9b] = { "tcwf",  FMT_B_REG_A_REG },
  [0x9c] = { "ocb",   FMT_B_REG_A_REG },
  [0x9d] = { "ocbf",  FMT_B_REG_A_REG },
  [0x9e] = { "ocw",   FMT_B_REG_A_REG },
  [0x9f] = { "ocwf",  FMT_B_REG_A_REG },

  [0xa0] = { "ab",    FMT_B_REG_A_REG },
  [0xa1] = { "abf",   FMT_B_REG_A_REG },
  [0xa2] = { "aw",    FMT_B_REG_A_REG },
  [0xa3] = { "awf",   FMT_B_REG_A_REG },
  [0xa4] = { "cab",   FMT_B_REG_A_REG },
  [0xa5] = { "cabf",  FMT_B_REG_A_REG },
  [0xa6] = { "caw",   FMT_B_REG_A_REG },
  [0xa7] = { "cawf",  FMT_B_REG_A_REG },
  [0xa8] = { "abc",   FMT_B_REG_A_REG },
  [0xa9] = { "abcf",  FMT_B_REG_A_REG },
  [0xaa] = { "awc",   FMT_B_REG_A_REG },
  [0xab] = { "awcf",  FMT_B_REG_A_REG },
  [0xac] = { "cad",   FMT_B_REG_A_REG },
  [0xad] = { "illegal", FMT_NO_ARG },
  [0xae] = { "cawi",  FMT_B_REG_A_REG },
  [0xaf] = { "cawif", FMT_B_REG_A_REG },

  [0xb0] = { "sb",    FMT_B_REG_A_REG },
  [0xb1] = { "sbf",   FMT_B_REG_A_REG },
  [0xb2] = { "sw",    FMT_B_REG_A_REG },
  [0xb3] = { "swf",   FMT_B_REG_A_REG },
  [0xb4] = { "cb",    FMT_B_REG_A_REG },
  [0xb5] = { "cbf",   FMT_B_REG_A_REG },
  [0xb6] = { "cw",    FMT_B_REG_A_REG },
  [0xb7] = { "cwf",   FMT_B_REG_A_REG },
  [0xb8] = { "sbc",   FMT_B_REG_A_REG },
  [0xb9] = { "sbcf",  FMT_B_REG_A_REG },
  [0xba] = { "swc",   FMT_B_REG_A_REG },
  [0xbb] = { "swcf",  FMT_B_REG_A_REG },
  [0xbc] = { "db1",   FMT_B_REG_A_REG },
  [0xbd] = { "db1f",  FMT_B_REG_A_REG },
  [0xbe] = { "dw1",   FMT_B_REG_A_REG },
  [0xbf] = { "dw1f",  FMT_B_REG_A_REG },

  [0xc0] = { "nb",    FMT_B_REG_A_REG },
  [0xc1] = { "nbf",   FMT_B_REG_A_REG },
  [0xc2] = { "nw",    FMT_B_REG_A_REG },
  [0xc3] = { "nwf",   FMT_B_REG_A_REG },
  [0xc4] = { "tb",    FMT_B_REG_A_REG },
  [0xc5] = { "tbf",   FMT_B_REG_A_REG },
  [0xc6] = { "tw",    FMT_B_REG_A_REG },
  [0xc7] = { "twf",   FMT_B_REG_A_REG },
  [0xc8] = { "orb",   FMT_B_REG_A_REG },
  [0xc9] = { "orbf",  FMT_B_REG_A_REG },
  [0xca] = { "orw",   FMT_B_REG_A_REG },
  [0xcb] = { "orwf",  FMT_B_REG_A_REG },
  [0xcc] = { "xb",    FMT_B_REG_A_REG },
  [0xcd] = { "xbf",   FMT_B_REG_A_REG },
  [0xce] = { "xw",    FMT_B_REG_A_REG },
  [0xcf] = { "xwf",   FMT_B_REG_A_REG },

  [0xd0] = { "ncb",   FMT_B_REG_A_REG },
  [0xd1] = { "ncbf",  FMT_B_REG_A_REG },
  [0xd2] = { "ncw",   FMT_B_REG_A_REG },
  [0xd3] = { "ncwf",  FMT_B_REG_A_REG },
  [0xd4] = { "illegal", FMT_NO_ARG },
  [0xd5] = { "illegal", FMT_NO_ARG },
  [0xd6] = { "illegal", FMT_NO_ARG },
  [0xd7] = { "illegal", FMT_NO_ARG },
  [0xd8] = { "srbc",  FMT_B_REG_A_REG },
  [0xd9] = { "srbcf", FMT_B_REG_A_REG },
  [0xda] = { "srwc",  FMT_B_REG_A_REG },
  [0xdb] = { "srwcf", FMT_B_REG_A_REG },
  [0xdc] = { "srb",   FMT_B_REG_A_REG },
  [0xdd] = { "srbf",  FMT_B_REG_A_REG },
  [0xde] = { "srw",   FMT_B_REG_A_REG },
  [0xdf] = { "srwf",  FMT_B_REG_A_REG },

  [0xe0] = { "ib",    FMT_B_LIT_A_REG },
  [0xe1] = { "ibf",   FMT_B_LIT_A_REG },
  [0xe2] = { "iw",    FMT_B_LIT_A_REG },
  [0xe3] = { "iwf",   FMT_B_LIT_A_REG },
  [0xe4] = { "isb",   FMT_B_LIT_A_REG },
  [0xe5] = { "isbf",  FMT_B_LIT_A_REG },
  [0xe6] = { "isw",   FMT_A_REG },
  [0xe7] = { "iswf",  FMT_A_REG },
  [0xe8] = { "illegal", FMT_NO_ARG },
  [0xe9] = { "illegal", FMT_NO_ARG },
  [0xea] = { "illegal", FMT_NO_ARG },
  [0xeb] = { "illegal", FMT_NO_ARG },
  [0xec] = { "mi",      FMT_BA_REG },
  [0xed] = { "illegal", FMT_NO_ARG },
  [0xee] = { "ltr",     FMT_BA_REG },
  [0xef] = { "illegal", FMT_NO_ARG },

  [0xf0] = { "rib1", FMT_BA_REG },
  [0xf1] = { "wib1", FMT_BA_REG },
  [0xf2] = { "riw1", FMT_BA_REG },
  [0xf3] = { "wiw1", FMT_BA_REG },
  [0xf4] = { "rib2", FMT_BA_REG },
  [0xf5] = { "wib2", FMT_BA_REG },
  [0xf6] = { "riw2", FMT_BA_REG },
  [0xf7] = { "wiw2", FMT_BA_REG },
  [0xf8] = { "r",    FMT_BA_REG },
  [0xf9] = { "w",    FMT_BA_REG },
  [0xfa] = { "ra",   FMT_BA_REG },
  [0xfb] = { "wa",   FMT_BA_REG },
  [0xfc] = { "ob",   FMT_BA_REG },
  [0xfd] = { "ow",   FMT_BA_REG },
  [0xfe] = { "os",   FMT_BA_REG },
  [0xff] = { "nop",  FMT_NO_ARG },
};

void dump_reg(proc_t *proc)
{
  int i;
  int gm;

  printf("tsr: %x tr: %02x %02x\n",
	 proc->translation_state,
	 proc->translation_register >> 8,
	 proc->translation_register & 0xff);
  if (proc->g < 6)
    gm = proc->g * 2 + 0x10;
  else
    gm = proc->g * 2;
  for (i = 2; i < 16; i += 2)
    printf("%c %x:%x: %02x %02x\n", " *" [i == gm], i+1, i, proc->reg[i+1], proc->reg[i]);
  for (i = 0; i < 12; i += 2)
    printf("%cg%x:%x: %02x %02x\n", " *" [(i + 0x10) == gm], i+1, i, proc->reg[i+0x11], proc->reg[i+0x10]);
}

void dump_ram(proc_t *proc, uint16_t low, uint16_t high)
{
  uint16_t addr;
  int32_t prev_addr = -1;
  for (addr = low; addr <= high; addr++)
    {
      int r;
      if (mem[addr] < 0)
	continue;
      if (prev_addr < 0)
	printf("ram:\n");
      if ((prev_addr >= 0) && (addr != (prev_addr + 1)))
	printf("...\n");
      printf("  ");
      for (r = 2; r < 28; r += 2)
	{
	  uint16_t regpair = (proc->reg[r+1] << 8) + proc->reg[r];
	  if (addr == regpair)
	    {
	      if (r <= 0x0f)
		printf("r%x:%x->", r+1, r);
	      else
		printf("g%x:%x->", r-15, r-16);
	    }
	}
      printf("%04x: %04x\n", addr, mem[addr]);
      prev_addr = addr;
    }
}

void disassemble(uint16_t addr, uint32_t uinst)
{
  uint8_t opcode = (uinst >> 8) & 0xff;
  int a = uinst & 0x0f;
  int b = (uinst >> 4) & 0x0f;
  printf("%04x: %06x %s", addr, uinst, op_info[opcode].mnemonic);
  switch (op_info[opcode].format)
    {
    case FMT_NO_ARG:
      break;
    case FMT_JMP:
      printf(" 0x%03x", uinst & 0x7ff);
      break;
    case FMT_COND_JUMP:
      printf(" 0x%03x", ((addr + 1) & 0x700) | (uinst & 0xff));
      break;
    case FMT_LITERAL:
      printf(" 0x%02x,r%x", (uinst >> 4) & 0xff, a);
      break;
    case FMT_INTERRUPT:
      printf(" ");
      if (b == 0)
	{
	  printf("0");
	  break;
	}
      if (b & 4)
	{
	  printf("i6");
	  if (b & 3)
	    printf(",");
	}
      if (b & 2)
	{
	  printf("i5");
	  if (b & 1)
	    printf(",");
	}
      if (b & 1)
	printf("i4");
      break;
    case FMT_A_REG:
      printf(" r%x", a);
      break;
    case FMT_B_REG_A_REG:
      printf(" r%x,r%x", b, a);
      break;
    case FMT_B_LIT_A_REG:
      printf(" 0x%x,r%x", b, a);
      break;
    case FMT_BA_REG:
      printf(" r%x:%x", b, a);
      break;
    }
  if (uinst & UINST_LRR)
    printf(",lrr");
  if (uinst & UINST_RNI)
    printf(",rsvc");
  printf("\n");
}


void sim_inst(proc_t *proc)
{
  int tr_num;
  translation_t *translation;

  proc->two_cycle = false;
  proc->branch = false;
  proc->uinst = ucode[proc->pc] | proc->modify_uinst;
  proc->modify_uinst = 0;
  tr_num = tr_num_from_addr[proc->pc];

  dump_reg(proc);
  dump_ram(proc, 0x0000, 0x7fff);
  printf("time: %" PRIu64 " ns\n", time_ns);
  disassemble(proc->pc, proc->uinst);

  proc->prev_pc = proc->pc;
  proc->pc = (proc->pc + 1) & 0x7ff;

  if (proc->uinst & 0x010000) // LRR bit
    proc->return_stack = proc->pc;

  op[(proc->uinst >> 8) & 0xff](proc);

  if (proc->rni)
    {
      // previous instruction had RNI bit set
      proc->rni = false;
      tr_num = rni_tr_num;
    }
  if (tr_num == 0x7f)
    translation = NULL;
  else
    {
      printf("*** addr %03x translation number %02x, tsr=%d\n", proc->prev_pc, tr_num, proc->translation_state);
      for (translation = translation_from_num[tr_num]; translation; translation = translation->next)
	{
	  uint8_t data;
	  if ((proc->translation_state & translation->translation_state_mask) != translation->translation_state_match)
	    continue;
	  if (translation->interrupt)
	    {
	      data = proc->interrupt;
	      printf("using interrupt register: %02x\n", data);
	    }
	  else if (proc->translation_state & 4)
	    {
	      data = proc->translation_register >> 8;
	      printf("using high byte of translation register: %02x\n", data);
	    }
	  else
	    {
	      data = proc->translation_register & 0xff;
	      printf("using low byte of translation register: %02x\n", data);
	    }
	  if ((data & translation->mask) != translation->data)
	    continue;
	  // found a match - assume only one, though hardware can OR multiple
	  // translations together
	  printf("translation match found\n");
	  break;
	}
    }

  if (proc->branch)
    proc->pc = proc->branch_target;
  else if (translation && translation->lta)
    {
      printf("translation lta, loading addr %03x\n", translation->addr);
      proc->pc = translation->addr;
    }
  else if (translation && translation->lra)
    {
      printf("translation lra, returning to addr %03x\n", proc->return_stack);
      proc->pc = proc->return_stack;
    }

  if (translation && translation->ltsr)
    {
      printf("translation ltsr %d\n", translation->translation_state);
      proc->translation_state = translation->translation_state;;
    }

  if (proc->uinst & UINST_RNI)
    proc->rni = true;
}


void reset(proc_t *proc)
{
  proc->modify_uinst = 0;
  proc->rni = false;
  proc->pc = 0x001;
}

void init_address_space(void)
{
  uint32_t addr;

  mem_install(0, 0x10000, bad_read, bad_write);

  for (addr = 0; addr < 0x10000; addr++)
    mem[addr] = -1;
}

void init_ram(uint16_t base, uint16_t size)
{
  mem_install(base, size, ram_read, ram_write);
}

void init_rom(uint16_t base, uint16_t size, char *fn)
{
  FILE *f;
  uint32_t addr;

  f = fopen(fn, "rb");
  if (! f)
    {
      fprintf(stderr, "can't open ROM file\n");
      exit(EX_NOINPUT);
    }

  mem_install(base, size, ram_read, bad_write); // can't write to ROM

  for (addr = base; addr < base + size; addr++)
    {
      int lb, hb;
      lb = getc(f);
      if (lb < 0)
	{
	  fprintf(stderr, "error reading ROM file\n");
	  exit(EX_IOERR);
	}
      hb = getc(f);
      if (hb < 0)
	{
	  fprintf(stderr, "error reading ROM file\n");
	  exit(EX_IOERR);
	}
      mem[addr] = (hb << 8) | lb;
    }

  mem_install(0xfc68, 1, ram_read, bad_write);
  mem[0xfc68] = mem[base];

  fclose(f);
}

uint16_t ssr_read(proc_t *proc, device_t *device, uint16_t addr)
{
  // bit 15..8: undefined
  // bit 7: init
  // bit 6: inten
  // bit 5: prnt - always reads 0
  // bit 4: pwrf
  // bit 3: 0
  // bit 2: intvl
  // bit 1: tick
  // bit 0: berr
  return 0x00;
}

void ssr_write(proc_t *proc, device_t *device, uint16_t addr, uint16_t data)
{
  // bit 15..8: undefined
  // bit 7: init  - write 1 causes bus reset (but doesn't reset 8253)
  // bit 6: inten - interrupt enable
  // bit 5: prnt  - 1 sends console output to printer
  // bit 4: pwrf  - write 1 to clear
  // bit 3: must be 0
  // bit 2: intvl - write 1 to clear
  // bit 1: tick  - write 1 to clear
  // bit 0: berr  - write 1 to clear
}

typedef struct
{
  uint8_t control_1;
  uint8_t control_2;
} usart_t;

void usart_reset(proc_t *proc, device_t *device)
{
  usart_t *usart = device->device;
}

uint16_t usart_read(proc_t *proc, device_t *device, uint16_t addr)
{
  usart_t *usart = device->device;
  int reg = addr & 3;

  switch (reg)
    {
    case 0:
      // control reg 1
      return usart->control_1;
    case 1:
      // control reg 2
      return usart->control_2;
      break;
    case 2:
      // status reg
      break;
    case 3:
      // receiver holding reg
      break;
    }
}

void usart_write(proc_t *proc, device_t *device, uint16_t addr, uint16_t data)
{
  usart_t *usart = device->device;
  int reg = addr & 3;

  switch (reg)
    {
    case 0:
      // control reg 1
      usart->control_1 = data;
      break;
    case 1:
      // control reg 2
      usart->control_2 = data;
      break;
    case 2:
      fprintf(stderr, "write to USART status register\n");
      break;
    case 3:
      // transmitter holding reg
      break;
    }
}


typedef struct
{
  uint16_t timer[3];
  uint8_t  mode[3];
  bool     byte_select[3];
} interval_timer_t;

void interval_timer_reset(proc_t *proc, device_t *device)
{
  interval_timer_t *timer = device->device;
}

uint16_t interval_timer_read(proc_t *proc, device_t *device, uint16_t addr)
{
  interval_timer_t *timer = device->device;
  int reg = addr & 3;
}

void interval_timer_write(proc_t *proc, device_t *device, uint16_t addr, uint16_t data)
{
  interval_timer_t *timer = device->device;
  int reg = addr & 3;

  if (reg == 3)
    {
      // write mode register
      int channel = (data >> 6) & 3;
      if (channel == 3)
	{
	  fprintf(stderr, "wrote illegal channel number to interval timer mode register\n");
	  exit(EX_SOFTWARE);
	}
      timer->mode[channel] = data;
    }
  else
    {
      // write count
    }
}


#define FDC_STATUS_BUSY             0x01
#define FDC_STATUS_INDEX            0x02  /* type I commands */
#define FDC_STATUS_DATA_REQUEST     0x02  /* type II, III commands */
#define FDC_STATUS_TRACK_00         0x04  /* type I commands */
#define FDC_STATUS_LOST_DATA        0x04  /* type II, III commands */
#define FDC_STATUS_CRC_ERROR        0x08
#define FDC_STATUS_SEEK_ERROR       0x10  /* type I commands */
#define FDC_STATUS_RECORD_NOT_FOUND 0x10  /* type II, III commands */
#define FDC_STATUS_HEAD_LOADED      0x20  /* type I commands */
#define FDC_STATUS_RECORD_TYPE      0x20  /* read command */
#define FDC_STATUS_WRITE_FAULT      0x20  /* write, write track commands */
#define FDC_STATUS_WRITE_PROTECT    0x40
#define FDC_STATUS_NOT_READY        0x80

#define FDC_BUFFER_SIZE 1024

typedef struct
{
  uint8_t drive_select;

  uint8_t drive_track;

  bool direction_out;  // true for out (toward track 76)

  uint8_t command;
  uint8_t status;
  uint8_t track;
  uint8_t sector;

  uint8_t buffer[FDC_BUFFER_SIZE];
  int max_count;
  int buffer_index;

  uint8_t step_rate;

  // class I commands: restore, seek, step
  bool verify;
  bool head_load;
  bool update_track;

  // class II commands: read, write
  // class III commands: read address, read track, write track
  bool write_deleted;   // write command only
  bool side_compare;    // read, write commands only
  bool settle_delay;
  bool side;            // read, write commands only
  bool multiple_record; // read, write commands only

  // class IV command: force interrupt
  uint8_t force_int_cond;

  uint32_t timer;
  void (*completion)(proc_t *proc, device_t *device);
} fdc_t;


void fdc_reset(proc_t *proc, device_t *device)
{
  fdc_t *fdc = device->device;
  fdc->buffer_index = 0;
  fdc->max_count = 1;
}

void fdc_tick(proc_t *proc, device_t *device)
{
  fdc_t *fdc = device->device;

  if (! fdc->timer)
    return;

  if (--fdc->timer)
    return;

  fdc->completion(proc, device);
}

uint16_t fdc_read(proc_t *proc, device_t *device, uint16_t addr)
{
  fdc_t *fdc = device->device;
  int reg = addr & 3;
  uint8_t data;

  switch (reg)
    {
    case 0:
      // status register
      return fdc->status;
    case 1:
      // track register
      return fdc->track;
    case 2:
      // sector register
      return fdc->sector;
    case 3:
      // data register XXX side effect, pull byte from buffer
      data = fdc->buffer[fdc->buffer_index++];
      if (fdc->buffer_index >= fdc->max_count)
	{
	  fdc->buffer_index = 0;
	  // XXX side effect - finish transfer
	}
      return data;
    }
}

void fdc_write(proc_t *proc, device_t *device, uint16_t addr, uint16_t data)
{
  fdc_t *fdc = device->device;
  bool write_drive_select = (addr >> 2) & 1;
  int reg = addr & 3;

  if (write_drive_select)
    fdc->drive_select = data >> 8;

  data &= 0xff;
  switch (reg)
    {
    case 0:
      // command register XXX side effect
      fdc->command = data;
      switch (fdc->command >> 4)
	{
	case 0:
	  // restore
	  fdc->step_rate       = fdc->command & 0x03;
	  fdc->verify          = (fdc->command >> 2) & 1;
	  fdc->head_load       = (fdc->command >> 3) & 1;

	  fdc->direction_out = false;

	  // XXX should set timer for completion

	  fdc->track = 0;
	  fdc->drive_track = 0;
	  break;
	  
	case 1:
	  // seek
	  fdc->step_rate       = fdc->command & 0x03;
	  fdc->verify          = (fdc->command >> 2) & 1;
	  fdc->head_load       = (fdc->command >> 3) & 1;

	  fdc->direction_out = fdc->drive_track > fdc->track;
	  // XXX should set timer for completion
	  break;

	case 2:
	case 3:
	  // step
	  fdc->step_rate       = fdc->command & 0x03;
	  fdc->verify          = (fdc->command >> 2) & 1;
	  fdc->head_load       = (fdc->command >> 3) & 1;
	  fdc->update_track    = (fdc->command >> 4) & 1;

	  // XXX should set timer for completion
	  break;

	case 4:
	case 5:
	  // step in
	  fdc->step_rate       = fdc->command & 0x03;
	  fdc->verify          = (fdc->command >> 2) & 1;
	  fdc->head_load       = (fdc->command >> 3) & 1;
	  fdc->update_track    = (fdc->command >> 4) & 1;

	  fdc->direction_out = false;
	  // XXX should set timer for completion
	  break;

	case 6:
	case 7:
	  // step out
	  fdc->step_rate       = fdc->command & 0x03;
	  fdc->verify          = (fdc->command >> 2) & 1;
	  fdc->head_load       = (fdc->command >> 3) & 1;
	  fdc->update_track    = (fdc->command >> 4) & 1;

	  fdc->direction_out = true;
	  // XXX should set timer for completion
	  break;

	case 8:
	case 9:
	  // read sector
	  // bit 0 must be 0
	  fdc->side_compare    = (fdc->command >> 1) & 1;
	  fdc->settle_delay    = (fdc->command >> 2) & 1;
	  fdc->side            = (fdc->command >> 3) & 1;
	  fdc->multiple_record = (fdc->command >> 4) & 1;

	  // XXX should set timer for completion
	  break;

	case 0xa:
	case 0xb:
	  // write sector
	  fdc->write_deleted   = (fdc->command >> 0) & 1;
	  fdc->side_compare    = (fdc->command >> 1) & 1;
	  fdc->settle_delay    = (fdc->command >> 2) & 1;
	  fdc->side            = (fdc->command >> 3) & 1;
	  fdc->multiple_record = (fdc->command >> 4) & 1;

	  // XXX should set timer for completion
	  break;

	case 0xc:
	  // read address
	  // bits 0, 1, 3 must be zero
	  fdc->settle_delay    = (fdc->command >> 2) & 1;

	  fprintf(stderr, "FDC read address command not implemented\n");
	  // XXX should set timer for completion
	  break;

	case 0xd:
	  // force interrupt
	  fdc->force_int_cond  = fdc->command & 0x0f;
	  if (fdc->force_int_cond == 0)
	    {
	      // force interrupt now
	      fdc->status &= ~ FDC_STATUS_BUSY;
	    }

	  break;

	case 0xe:
	  // read track
	  // bits 0, 1, 3 must be zero
	  fdc->settle_delay    = (fdc->command >> 2) & 1;

	  fprintf(stderr, "FDC read address command not implemented\n");
	  // XXX should set timer for completion
	  break;

	case 0xf:
	  // write track
	  // bits 0, 1, 3 must be zero
	  fdc->settle_delay    = (fdc->command >> 2) & 1;

	  fprintf(stderr, "FDC write track command not implemented\n");
	  // XXX should set timer for completion
	  break;
	}
      break;
    case 1:
      // track register
      fdc->track = data;
      break;
    case 2:
      // sector register
      fdc->sector = data;
      break;
    case 3:
      // data register
      fdc->buffer[fdc->buffer_index++] = data;
      if (fdc->buffer_index >= fdc->max_count)
	{
	  fdc->buffer_index = 0;
	  // XXX side effect - finish transfer
	}
      break;
    }
}


typedef struct
{
  uint8_t control;

  bool status_bow;
  bool status_dint;
  bool status_toi;
  bool status_tczi;

  uint16_t transfer_count;
  uint32_t address;  // 18-bit -- byte address, not word address

  uint8_t id_code;
} dmac_t;

void dmac_reset(proc_t *proc, device_t *device)
{
  dmac_t *dmac = device->device;

  dmac->control = 0x70;

  dmac->status_bow  = 1;
  dmac->status_dint = 0;
  dmac->status_toi  = 0;
  dmac->status_tczi = 0;

  dmac->transfer_count = 1;
  dmac->address = 0;
  dmac->id_code = 0;
}

uint16_t dmac_read(proc_t *proc, device_t *device, uint16_t addr)
{
  dmac_t *dmac = device->device;
  int reg = addr & 3;

  switch (reg)
    {
    case 0:
      // control
      return dmac->control;
    case 1:
      // status
      return (((dmac->control & 1) << 7) |   // busy
	      (dmac->control & 0x70) |       // aece, hbus, iom
	      (dmac->status_tczi   << 3) |
	      (dmac->status_toi    << 2) |
	      (dmac->status_dint   << 1) |
	      (dmac->status_bow    << 0));
    case 2:
      // transfer count low
      return dmac->transfer_count & 0xff;
    case 3:
      // transfer count high
      return dmac->transfer_count >> 8;
    case 4:
      // address low
      return dmac->address & 0xff;
    case 5:
      // address high
      return (dmac->address >> 8) & 0xff;
    case 6:
      // address extension
      return dmac->address >> 16;
    case 7:
      // ID code
      return dmac->id_code;
    }
}

void dmac_write(proc_t *proc, device_t *device, uint16_t addr, uint16_t data)
{
  dmac_t *dmac = device->device;
  int reg = addr & 3;

  switch (reg)
    {
    case 0:
      // control
      dmac->control = data;
      break;
    case 1:
      // status - only bits 1 and 2 are read/write
      dmac->status_dint = (data >> 1) & 1;
      dmac->status_toi  = (data >> 2) & 1;
      break;
    case 2:
      // transfer count low
      dmac->transfer_count = (dmac->transfer_count & 0xff00) | (data & 0xff);
      break;
    case 3:
      // transfer count high
      dmac->transfer_count = ((data & 0xff) << 8) | (dmac->transfer_count & 0x00ff);
      break;
    case 4:
      // address low
      dmac->address = (dmac->address & 0x3ff00) | (data & 0xff);
      break;
    case 5:
      // address high
      dmac->address = (dmac->address & 0x300ff) | ((data & 0xff) << 8);
      break;
    case 6:
      // address extension
      dmac->address = (dmac->address & 0x0ffff) | ((data & 0x03) << 16);
      break;
    case 7:
      // ID code register
      dmac->id_code = data;
      break;
    }
}

uint16_t ses_read(proc_t *proc, device_t *device, uint16_t addr)
{
  return 0x98;  // bit 7 = 1, boot into HDT
                // bits 4..3 = 11: printer speed = 9600
                // bits 2..0 = 000: console speed = 19200
}


usart_t usart;

device_t device_usart =
{
  .next = NULL,  // next
  .reset_fn = & usart_reset,
  .read_fn  = & usart_read,
  .write_fn = & usart_write,
  .device   = & usart
};

interval_timer_t interval_timer;

device_t device_interval_timer =
{
  .next = NULL,  // next
  .reset_fn = & interval_timer_reset,
  .read_fn  = & interval_timer_read,
  .write_fn = & interval_timer_write,
  .device   = & interval_timer
};

fdc_t fdc;

device_t device_fdc =
{
  .next = NULL,  // next
  .reset_fn = & fdc_reset,
  .tick_fn  = & fdc_tick,
  .read_fn  = & fdc_read,
  .write_fn = & fdc_write,
  .device   = & fdc
};

dmac_t dmac;

device_t device_dmac =
{
  .next = NULL,  // next
  .reset_fn = & dmac_reset,
  .read_fn  = & dmac_read,
  .write_fn = & dmac_write,
  .device   = & dmac
};

void tick_io(proc_t *proc)
{
  device_t *device;

  for (device = device_head; device; device = device->next)
    if (device->tick_fn)
      device->tick_fn(proc, device);
}

void reset_io(proc_t *proc)
{
  device_t *device;
  
  for (device = device_head; device; device = device->next)
    if (device->reset_fn)
      device->reset_fn(proc, device);
}

void init_io(proc_t *proc)
{
  // I/O definitions here are for PDQ-3 CPU board, not the same
  // as WD900.
  
  // FC10..FC13: TR1931 console USART
  device_install(0xfc10, 4, & device_usart);

  // FC18: system environment switch
  mem_install(0xfc18, 1, ses_read, bad_write);
  
  // FC20..FC23: 8253 interval timer
  device_install(0xfc20, 4, & device_interval_timer);

  // FC24: system status register
  mem_install(0xfc24, 1, ssr_read, ssr_write);

  // FC30..FC37: FD1793 floppy controller
  device_install(0xfc30, 8, & device_fdc);

  // FC38..FC3E: DM1883 DMA controller
  device_install(0xfc38, 7, & device_dmac);

  reset_io(proc);
}

void load_microcode(char *fn)
{
  FILE *f;
  int line_number = 0;
  int count = 0;
  int translation_count = 0;
  uint16_t addr;
  uint32_t data;
  uint8_t tr_num;
  char buf[40];

  for (addr = 0; addr < 0x800; addr++)
    ucode[addr] = 0;

  f = fopen(fn, "r");
  if (! f)
    {
      fprintf(stderr, "can't open microcode file\n");
      exit(EX_NOINPUT);
    }

  while (fgets(buf, sizeof(buf), f))
    {
      line_number++;
      if (buf[0] == 't')
	{
	  uint8_t tr_num;
	  int interrupt;
	  uint8_t lsr;
	  int ltsr;
	  int lta;
	  int lra;

	  translation_t *tp = calloc(1, sizeof(translation_t));
	  if (! tp)
	    {
	      fprintf(stderr, "memory allocation failure\n");
	      exit(EX_OSERR);
	    }

	  int r = sscanf(buf,
			 "t%" SCNx8  // tr_num
			 ": %" SCNx8 // ts_match
			 " %" SCNx8  // ts_mask
			 " %d"       // interrupt
			 " %" SCNx8  // data_val
			 " %" SCNx8  // data_mask
			 " %" SCNx8  // tsr
			 " %d"       // ltsr
			 " %" SCNx16 // addr
			 " %d"       // lta
			 " %d",      // lra
			 & tr_num,
			 & tp->translation_state_match,
			 & tp->translation_state_mask,
			 & interrupt,
			 & tp->data,
			 & tp->mask,
			 & tp->translation_state,
			 & ltsr,
			 & tp->addr,
			 & lta,
			 & lra);
	  if (r != 11)
	    {
	      fprintf(stderr, "malformed microcode file, line %d\n", line_number);
	      fprintf(stderr, "line: '%s'\n", buf);
	      exit(EX_NOINPUT);
	    }
	  tp->interrupt = interrupt;
	  tp->ltsr = ltsr;
	  tp->lta = lta;
	  tp->lra = lra;

	  tp->next = translation_from_num[tr_num];
	  translation_from_num[tr_num] = tp;

	  translation_count++;
	}
      else
	{
	  int r = sscanf(buf, "%" SCNx16 ": %" SCNx32 " %" SCNx8, & addr, & data, & tr_num);
	  if (r == EOF)
	    break;
	  if (r != 3)
	    {
	      fprintf(stderr, "malformed microcode file, line %d\n", line_number);
	      exit(EX_NOINPUT);
	    }
	  if (addr == 0x800)
	    rni_tr_num = tr_num;
	  else
	    {
	      ucode[addr] = data;
	      tr_num_from_addr[addr] = tr_num;
	      count++;
	    }
	}
    }

  printf("%d words of microcode loaded\n", count);
  printf("%d translations loaded\n", translation_count);
  
  fclose(f);
}

int main(int argc, char *argv[])
{
  proc_t proc;

  load_microcode("wd9000-141518-ucode.txt");
 
  init_address_space();
  init_ram(0x0000, 0xf000);
  init_rom(0xf400, 0x0200, "bootrom.bin");

  reset(& proc);

  init_io(& proc);

  while (true)
    {
      uint64_t time_step_ns;

      sim_inst(& proc);
      time_step_ns = (proc.two_cycle + 1) * CPU_CLOCK_PERIOD_NS;

      time_ns += time_step_ns;

      io_time_ns += time_step_ns;
      while (io_time_ns >= IO_TICK_PERIOD_NS)
	{
	  io_time_ns -= IO_TICK_PERIOD_NS;
	  tick_io(& proc);
	}
    }

  exit(EX_OK);
}
