#!/usr/bin/python3

import argparse
import re
import sys

def parse_defs(fname):
    defs = {}
    for line in open(fname):
        line = line.strip()
        m = re.match(r'^#define\s+(\w+)\s+(\w+)', line)
        if m:
            key = m.group(1)
            val = m.group(2)
            defs[key] = val
    return defs

def expand_macros(defs, s):
    for v in defs:
        s = s.replace(v, defs[v])
    return s

def parse_irqmap(fname):
    d = {}
    defs = {}
    for line in open(fname):
        line = line.strip()
        if len(line) == 0 or line.startswith('//'):
                continue
        m = re.match(r'^#include ["<]([^">]+)[>"]', line)
        if m:
                incfile = m.group(1)
                defs.update(parse_defs(incfile))
                continue

        line = expand_macros(defs, line)
        p = line
        if ':' in p: # explicitly named C ISR
            kv = [s.strip() for s in p.split(':')]
            irq = eval(kv[0])
            d[irq] = kv[1]
        else: # create an ISR stub
            if '-' in p:
                r = map(int, p.split('-'))
                irq_nums = range(r[0], r[1])
            else:
                irq_nums = [int(p)]
            for n in irq_nums:
                d[n] = None
    return d

parser = argparse.ArgumentParser(
    description="Generate assembly source for vector table")
parser.add_argument('--internal-irqs', type=int, default=16,
    help='Number internal IRQs')
parser.add_argument('--external-irqs', type=int, default=240,
   help='Number external IRQs')
parser.add_argument('--irqmap',
    help='IRQ to ISR handler map file')
parser.add_argument('--verbose', '-v', action='store_true',
    help='Print IRQ map')
parser.add_argument('out_asm',
    help='Output file with generated C source')
parser.add_argument('out_c',
    help='Output file with generated assembly source')
args = parser.parse_args()

irqmap = parse_irqmap(args.irqmap)
if args.verbose:
    for irq in irqmap:
        print("%4u: %s" % (irq, irqmap[irq]))

if irqmap is None:
        irqmap = range(0, 240)

def external(irq):
	return irq - args.internal_irqs

def is_internal(irq):
        return irq < 16;

NVIC_BASE = 0xe000e000
NVIC_ICPR = 0x280

# ISR handlers for each vector number
# The rest of the vectors (not in this dict) get default handler
DEFAULT_ISR = "hang"
isr = {
 0: None,
 1: "reset",
11: "svc",
}

f = open(args.out_asm, "w")

f.write(
"""/* This file was automatically generated by genisr.py. */

.cpu cortex-m4
.thumb

.global __entry
.word __stacktop
"""
)

for i in range(0, args.internal_irqs + args.external_irqs):
    handler = None
    if i in isr:
        if isr[i] is not None:
            handler = isr[i]
    elif external(i) in irqmap:
        handler = "isr%u" % external(i)
    elif is_internal(i):
        handler = "exc%u" % i
    else:
        handler = DEFAULT_ISR

    if handler is not None:
        f.write(".word %s\n" % handler)

f.write("\n")

f.write(
"""
__entry: /* same as 'reset', but must not be marked with .thumb_func */

.thumb_func
reset:
    bl notmain
    b hang
    b hang

.thumb_func
svc:
    mov r0, #0
    sub r0, #7 // 0xfffffff9: priveledged Thread mode with main stack
    bx r0

.thumb_func
hang:   b .
    b hang
"""
+ "\n");

for irq in range(args.internal_irqs):
    if not irq in isr:
      f.write(("""
.thumb_func
exc%u:
    b exc%u
""") % (irq, irq))

for irq in irqmap:
    nvic_icpr_addr = NVIC_BASE + NVIC_ICPR + (irq / 32) * 4
    nvic_icpr_shift = irq % 32
    if irqmap[irq] is not None:
        isr = irqmap[irq]
    else:
        isr = "c_isr%u" % irq

    f.write(("""
.thumb_func
isr%u:
    push {r0, r1, lr}

    mov r1, #%u
    ldrh r0, isr%u_fmt_str_addr
    bl printf

    bl %s

    /* Clear Pending flag */
    ldrh r0, isr%u_icpr_addr
    mov r1, #1
    lsl r1, #%u
    strh r1, [r0]

    pop {r0, r1, pc}

    .align 2
isr%u_icpr_addr:
    .word 0x%08x
isr%u_fmt_str_addr:
    .word isr_fmt_str
""") % (irq, irq, irq, isr, irq, nvic_icpr_shift, irq, nvic_icpr_addr, irq))

if len(irqmap) > 0:
    f.write("""
isr_fmt_str:
    .string "IRQ #%u\\r\\n"
""")

# Generate C source for stub IRQ handlers (ISRs)

f = open(args.out_c, "w")

f.write(
"""
/* This file was automatically generated by genisr.py.
 *
 * The following define stub functions that are called by the IRQ handlers
 * defined in assembly in vectors.s (generated by genvec.py).
 */
""")

f.write(
"""
#include "printf.h"
""")

# Create stub ISRs for IRQs for which no ISR func was named
for irq in irqmap:
    if irqmap[irq] is None:
        f.write(
"""
int c_isr%u (void) {
    static unsigned num_invoc = 0;
    void *p = 0x0;
    asm ("mov %%0, lr\\n" : "=r" (p));
    printf("IRQ %u (%%lu): LR %%p\\r\\n", num_invoc, p);
    num_invoc++;
    return(0);
}
""" % (irq, irq))
