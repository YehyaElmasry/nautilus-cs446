/*
 * This file is part of the Nautilus AeroKernel developed
 * by the Hobbes and V3VEE Projects with funding from the
 * United States National  Science Foundation and the Department of Energy.
 *
 * The V3VEE Project is a joint project between Northwestern University
 * and the University of New Mexico.  The Hobbes Project is a collaboration
 * led by Sandia National Laboratories that includes several national
 * laboratories and universities. You can find out more at:
 * http://www.v3vee.org  and
 * http://xstack.sandia.gov/hobbes
 *
 * Copyright (c) 2019, XXX
 * All rights reserved.
 *
 * Authors: XXX

 * This is free software.  You are permitted to use,
 * redistribute, and modify it as specified in the file "LICENSE.txt".
 */

#include <nautilus/nautilus.h>
#include <nautilus/dev.h>   // eventually snddev.h
#include <nautilus/irq.h>
#include <dev/pci.h>

#include <dev/hda_pci.h>

#ifndef NAUT_CONFIG_DEBUG_HDA_PCI
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

#define INFO(fmt, args...) INFO_PRINT("hda_pci: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("hda_pci: " fmt, ##args)
#define ERROR(fmt, args...) ERROR_PRINT("hda_pci: " fmt, ##args)

// show output for all reg read/writes
#define DO_DEBUG_REGS 0
#if DO_DEBUG_REGS
#define DEBUG_REGS(fmt, args...) DEBUG(fmt, ##args)
#else
#define DEBUG_REGS(fmt, args...)
#endif

#define GLOBAL_LOCK_CONF uint8_t _global_lock_flags
#define GLOBAL_LOCK() _global_lock_flags = spin_lock_irq_save(&global_lock)
#define GLOBAL_UNLOCK() spin_unlock_irq_restore(&global_lock, _global_lock_flags)

#define STATE_LOCK_CONF uint8_t _state_lock_flags
#define STATE_LOCK(state) _state_lock_flags = spin_lock_irq_save(&(state->lock))
#define STATE_UNLOCK(state) spin_unlock_irq_restore(&(state->lock), _state_lock_flags)

#define assert(cond) if (!cond) ERROR("Assert failed in ")
#define assert(cond) \
    if (cond) {} \
    else { \
    ERROR("Assert failed in %s: %d", __FILE__, __LINE__);\
    }\
//
// On QEMU using -soundhw hda we see the following:
//
//0:4.0 : 8086:2668 0103c 0010s MSI(off,MSI64,nn=1,bv=0,nv=0,tc=0) legacy(l=11,p=1)

// for protection of global state in the driver
static spinlock_t global_lock;

// number of devices (used to assign names)
static int num_devs = 0;

// list of hda devices
static struct list_head dev_list;

#define GCAP     0x0
#define GCAP_LEN 0x2
typedef union gcap   // all read only
{
    uint16_t val;
    struct
    {
        uint8_t ok64: 1;
        uint8_t nsdo: 2;
        uint8_t bss: 5;
        uint8_t iss: 4;
        uint8_t oss: 4;
#define NUM_SDO(x) (((x).nsdo)==0 ? 1 : ((x).nsdo)==1 ? 2 : ((x).nsdo)==2 ? 4 : 0)
    };
} __attribute__((packed)) gcap_t;

#define VMIN     0x2
#define VMIN_LEN 0x1
typedef uint8_t vmin_t;  // readonly

#define VMAJ     0x3
#define VMAJ_LEN 0x1
typedef uint8_t vmaj_t;  // read only

#define OUTPAY     0x4
#define OUTPAY_LEN 0x2
typedef uint16_t outpay_t;  // read only, maximum payload size

#define INPAY     0x6
#define INPAY_LEN 0x2
typedef uint16_t inpay_t;  // read only, maximum payload size

#define GCTL      0x8
#define GCTL_LEN  0x4
typedef union
{
    uint32_t val;
    struct
    {
        uint8_t crst: 1; // read write sticky => write 0 to assert reset, 1 to deassert rest, read of 1=> ready
        // must have CORB/RIRB RUN, and stream RUN bits off before vbefore reset
        uint8_t fcntrl: 1; // read write write 1 to initiate flush, cycle ends with Flush Status
        uint8_t res2: 6; // reserved, preserve
        uint8_t unsol: 1; // read write 1=> unsolicited responses from codecs are forwarded to RIRB
        uint32_t res1: 23; // reserved, preserve
    };
} __attribute__((packed)) gctl_t;

#define WAKEEN     0xc
#define WAKEEN_LEN 0x2
typedef uint16_t wakeen_t;

#define STATESTS     0xe
#define STATESTS_LEN 0x2
typedef union
{
    uint16_t val;
    struct
    {
        uint16_t  sdiwake: 15; // read only, write 1 to clear, sticky, flag
#define SDIMAX 15
#define SDIWAKE(s,i) ((((s).sdiwake)>>(i)) & 0x1)
#define SDIWAKE_RESET_MASK 0x7fff
        uint8_t   res: 1;   // reserved zero
    };
} __attribute__((packed)) statests_t;


// GSTS
// OUTSTRMPAY
// INSTRMPAY

#define INTCTL       0x20
#define INTCTL_LEN   0x4
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t  sie: 30;     // stream interrupt enable bits (interrupts from streams)
        // low->high, input streams, then output streams, finally bidirs
        uint8_t   cie: 1;      // controller interrupt enable (interrupts from controller)
        uint8_t   gie: 1;      // global interrupt enable (interrupts at all)
    };
} __attribute__((packed)) intctl_t;

#define INTSTS       0x24
#define INTSTS_LEN   0x4
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t  sis: 30;     // high if stream raised interrupt, same format as sie, above
        uint8_t   cis: 1;      // controller interrupt status
        uint8_t   gis: 1;      // global interrupt status
    };
} __attribute__((packed)) intsts_t;


#define CORBLBASE      0x40
#define CORBLBASE_LEN  0x4
typedef uint32_t corblbase_t;   // 128 byte alignment!

#define CORBUBASE      0x44
#define CORBUBASE_LEN  0x4
typedef uint32_t corbubase_t;

#define CORBWP         0x48
#define CORBWP_LEN     0x2
typedef union
{
    uint16_t val;
    struct
    {
        uint8_t corbwp;  // in units of corb commands (4 bytes each)
        uint8_t res;    // reserved/preserve
    };
} __attribute__((packed)) corbwp_t;

#define CORBRP         0x4a
#define CORBRP_LEN     0x2
typedef union
{
    uint16_t val;
    struct
    {
        uint8_t corbrp;   // in units of corb commands
        uint8_t res: 7;   // reserved/preserve
        uint8_t corbrprst: 1; // rw 1=> flush+reset, then wait to transition to 1, then write 0, then wait for transition to 1
    };
} __attribute__((packed)) corbrp_t;

#define CORBCTL         0x4c
#define CORBCTL_LEN     0x1
typedef union
{
    uint8_t val;
    struct
    {
        uint8_t cmeie: 1;   // generate interrupt if memory error
        uint8_t corbrun: 1; // write 1 => start CORB DMA, read value back to confirm started
    };
} __attribute__((packed)) corbctl_t;

#define CORBSTS         0x4d
#define CORBSTS_LEN     0x1
typedef union
{
    uint8_t val;
    struct
    {
        uint8_t cmei: 1;   // memory error detected (DMA is borked), write 1 to clear
        uint8_t res: 7;   // preserve
    };
} __attribute__((packed)) corbsts_t;

#define CORBSIZE         0x4e
#define CORBSIZE_LEN     0x1
typedef union
{
    uint8_t val;
    struct
    {
        uint8_t corbsize: 2;
#define CORBSIZE_DECODE(x) ((x->corbsize==0 ? 2 : x->corbsize==1 ? 16 : x->corbsize==2 : 256 : 0))
        uint8_t res: 2;   // preserve
        uint8_t corbszcap: 4; // read onlye
#define CORBSIZECAP_HAS_2(x) (!!(x.corbszcap & 0x1))
#define CORBSIZECAP_HAS_16(x) (!!(x.corbszcap & 0x2))
#define CORBSIZECAP_HAS_256(x) (!!(x.corbszcap & 0x4))
    };
} __attribute__((packed)) corbsize_t;


#define RIRBLBASE      0x50
#define RIRBLBASE_LEN  0x4
typedef uint32_t rirblbase_t;   // 128 byte alignment!

#define RIRBUBASE      0x54
#define RIRBUBASE_LEN  0x4
typedef uint32_t rirbubase_t;

#define RIRBWP         0x58
#define RIRBWP_LEN     0x2
typedef union
{
    uint16_t val;
    struct
    {
        uint8_t rirbwp;      // in units of rirb responses (8 bytes long)
        uint8_t res: 7;      // reserved/preserve
        uint8_t rirbwprst: 1; // write 1 to reset, must stop DMA engine first
    };
} __attribute__((packed)) rirbwp_t;

#define RINTCNT         0x5a
#define RINTCNT_LEN     0x2
typedef union
{
    uint16_t val;
    struct
    {
        uint8_t rintcnt;  // 1=1, 2=2, but 0=256
        uint8_t res;    // reserved/preserve
    };
} __attribute__((packed)) rintcnt_t;

#define RIRBCTL         0x5c
#define RIRBCTL_LEN     0x1
typedef union
{
    uint8_t val;
    struct
    {
        uint8_t rintctl: 1;   // write 1 to generate interrupt after n responses or empry response slot on input
        uint8_t rirbdmaen: 1; // write 1 to make DMA engine spin
        uint8_t rirboic: 1;   // write 1 => generate interrupt on response overrun interrupt status bit
    };
} __attribute__((packed)) rirbctl_t;

#define RIRBSTS         0x5d
#define RIRBSTS_LEN     0x1
typedef union
{
    uint8_t val;
    struct
    {
        uint8_t rintfl: 1;  // reads as 1 when interrupt generated after n responses or empty slot, clear by writing 1
        uint8_t res1: 1;    // must be zero
        uint8_t rirbois: 1; // reads as 1 when rirb is overrun, clear by writing 1
        uint8_t res2: 5;    // must be zero
    };
} __attribute__((packed)) rirbsts_t;

#define RIRBSIZE         0x5e
#define RIRBSIZE_LEN     0x1
typedef union
{
    uint8_t val;
    struct
    {
        uint8_t rirbsize: 2;
#define RIRBSIZE_DECODE(x) ((x->rirbsize==0 ? 2 : x->rirsize==1 ? 16 : x->rirbsize==2 : 256 : 0))
        uint8_t res: 2;   // preserve
        uint8_t rirbszcap: 4; // read onlye
#define RIRBSIZECAP_HAS_2(x) (!!(x.rirbszcap & 0x1))
#define RIRBSIZECAP_HAS_16(x) (!!(x.rirbszcap & 0x2))
#define RIRBSIZECAP_HAS_256(x) (!!(x.rirbszcap & 0x4))
    };
} __attribute__((packed)) rirbsize_t;





typedef struct
{
    int valid;
    // codec state goes here
} codec_state_t;

#define MAX_CORB_ENTRIES 256
typedef union
{
    uint32_t val;
    struct
    {
        uint32_t verb: 20;  // actual command
        uint8_t nid: 7;     // node id, node 0=>root
        uint8_t indirect: 1; // indirect node ref
        uint8_t CAd: 4;     // codec id (dest)
    } __attribute__((packed)) ;
} __attribute__((packed)) corb_entry_t;
typedef corb_entry_t codec_req_t;

typedef struct
{
    corb_entry_t buf[MAX_CORB_ENTRIES];
    int size; // actual number of entries used
    int cur_write; // our write pointer for comparison with its read pointer
} __attribute__((aligned(128))) corb_state_t;

#define MAX_RIRB_ENTRIES 256
typedef struct
{
    uint32_t resp;
    union
    {
        uint32_t val;
        struct
        {
            uint8_t  codec: 4;
            uint8_t  unsol: 1; // unsolicited response
            uint32_t res: 27;
        } __attribute__((packed)) ;
    } __attribute__((packed)) resp_ex ;
} __attribute__((packed)) rirb_entry_t;
typedef rirb_entry_t codec_resp_t;

typedef struct
{
    rirb_entry_t buf[MAX_RIRB_ENTRIES];
    int size; // actual number of entries used
    int cur_read; // current read pointer (where driver is)
} __attribute__((aligned(128))) rirb_state_t;


struct hda_pci_dev
{
    // for protection of per-device state
    spinlock_t      lock;

    // we are a (generic) nk dev so far
    struct nk_dev  *nk_dev;

    // we are a PCI device
    struct pci_dev *pci_dev;

    // we will be put on a list of all hda devices
    struct list_head hda_node;


    // the following is for legacy interrupts
    // we will try to use MSI first
    uint8_t   pci_intr;  // number on bus
    uint8_t   intr_vec;  // number we will see

    // The following hide the details of the PCI BARs, since
    // we only have one block of registers
    enum { NONE, IO, MEMORY}  method;

    // Where registers are mapped into the I/O address space
    // if at all
    uint16_t  ioport_start;
    uint16_t  ioport_end;

    // Where registers are mapped into the physical memory address space
    // if at all
    uint64_t  mem_start;
    uint64_t  mem_end;

    // copies of state handy to keep, should do similar for each codec
    gcap_t   gcap;
    vmin_t   vmin;
    vmaj_t   vmaj;
    outpay_t outpay;
    inpay_t  inpay;

    // per codec state
    codec_state_t codecs[SDIMAX];

    corb_state_t corb;  // command output ring buffer
    rirb_state_t rirb;  // response input ring buffer

};

// Codec requests

#define MAKE_VERB_8(id,payload)  ((((uint32_t)(id))<<8 ) | (((uint32_t)(payload))&0xff))
#define MAKE_VERB_16(id,payload) ((((uint32_t)(id))<<16) | (((uint32_t)(payload))&0xffff))

// 12 bit identifiers (8 bits payload)
#define GET_PARAM   0xf00
#define GET_CONSEL  0xf01
#define SET_CONSEL  0x701
#define GET_CONLIST 0xf02
#define GET_PROCSTATE 0xf03
#define SET_PROCSTATE 0x703
// ignoring S/PDIF stuff here
// ignoring power state stuff here
#define GET_CONVCTL   0xf06
#define SET_CONVCTL   0x706
#define GET_SDISEL    0xf04
#define SET_SDISEL    0x704
#define GET_PINWGTCTL 0xf07
#define SET_PINWGTCTL 0x707
#define GET_CONSELCTL 0xf08
#define SET_CONSELCTL 0x708
#define GET_PINSENSE  0xf09
#define EXE_PINSENSE  0x709
#define GET_EAPDBTLEN 0xf0c
#define SET_EAPDBTLEN 0xf0c
//
// skipping... a lot of GPIO stuff, Beep Generation, Volume Knob
//
#define GET_IMPLID  0xf20
// skipping... more

// 4 bit identifiers (16 bits payload)
#define GET_COEFFIDX  0xd
#define SET_COEFFIDX  0x5
#define GET_COEFF     0xc
#define SET_COEFF     0x4
#define GET_GAINMUTE  0xb
#define SET_GAINMUTE  0x3
#define GET_CONVFMT   0xa
#define SET_CONVFMT   0x2


// parameters
#define VENDOR      0x0 // also includes device id
#define REVISION    0x2
#define SUBORD_NODE_COUNT 0x4
#define FUNC_GROUP_TYPE   0x5
#define AUDIO_FUNC_GROUP_CAPS        0x8
#define AUDIO_WIDGET_CAPS            0x9
#define PCM_SIZES_AND_RATES          0xa // needed for playback/recording
#define STREAM_FORMATS               0xb // ""
#define PIN_CAPS                     0xc
#define AMP_CAPS                     0xd
#define CON_LIST_LEN                 0xe
#define POWER_STATES                 0xf
#define PROC_CAPS                    0x10
#define GPIO_COUNT                   0x11
#define VOL_KNOB_CAPS                0x13


// accessor functions for device registers

static inline uint32_t hda_pci_read_regl(struct hda_pci_dev *dev, uint32_t offset)
{
    uint32_t result;
    if (dev->method == MEMORY)
    {
        uint64_t addr = dev->mem_start + offset;
        __asm__ __volatile__ ("movl (%1), %0" : "=r"(result) : "r"(addr) : "memory");
    }
    else
    {
        result = inl(dev->ioport_start + offset);
    }
    DEBUG_REGS("readl %08x returns %08x\n", offset, result);
    return result;
}

static inline uint16_t hda_pci_read_regw(struct hda_pci_dev *dev, uint32_t offset)
{
    uint16_t result;
    if (dev->method == MEMORY)
    {
        uint64_t addr = dev->mem_start + offset;
        __asm__ __volatile__ ("movw (%1), %0" : "=r"(result) : "r"(addr) : "memory");
    }
    else
    {
        result = inw(dev->ioport_start + offset);
    }
    DEBUG_REGS("readw %08x returns %04x\n", offset, result);
    return result;
}

static inline uint8_t hda_pci_read_regb(struct hda_pci_dev *dev, uint32_t offset)
{
    uint8_t result;
    if (dev->method == MEMORY)
    {
        uint64_t addr = dev->mem_start + offset;
        __asm__ __volatile__ ("movb (%1), %0" : "=r"(result) : "r"(addr) : "memory");
    }
    else
    {
        result = inb(dev->ioport_start + offset);
    }
    DEBUG_REGS("readb %08x returns %02x\n", offset, result);
    return result;
}

static inline void hda_pci_write_regl(struct hda_pci_dev *dev, uint32_t offset, uint32_t data)
{
    DEBUG_REGS("writel %08x with %08x\n", offset, data);
    if (dev->method == MEMORY)
    {
        uint64_t addr = dev->mem_start + offset;
        __asm__ __volatile__ ("movl %1, (%0)" : : "r"(addr), "r"(data) : "memory");
    }
    else
    {
        outl(data, dev->ioport_start + offset);
    }
}

static inline void hda_pci_write_regw(struct hda_pci_dev *dev, uint32_t offset, uint16_t data)
{
    DEBUG_REGS("writew %08x with %04x\n", offset, data);
    if (dev->method == MEMORY)
    {
        uint64_t addr = dev->mem_start + offset;
        __asm__ __volatile__ ("movw %1, (%0)" : : "r"(addr), "r"(data) : "memory");
    }
    else
    {
        outw(data, dev->ioport_start + offset);
    }
}

static inline void hda_pci_write_regb(struct hda_pci_dev *dev, uint32_t offset, uint8_t data)
{
    DEBUG_REGS("writeb %08x with %02x\n", offset, data);
    if (dev->method == MEMORY)
    {
        uint64_t addr = dev->mem_start + offset;
        __asm__ __volatile__ ("movb %1, (%0)" : : "r"(addr), "r"(data) : "memory");
    }
    else
    {
        outb(data, dev->ioport_start + offset);
    }
}


//
// This dance will eventually get abstracted into the PCI
// subsystem so that we don't repeat it over and over...
static int discover_devices(struct pci_info *pci)
{
    struct list_head *curbus, *curdev;

    INIT_LIST_HEAD(&dev_list);

    if (!pci)
    {
        ERROR("No PCI info\n");
        return -1;
    }

    DEBUG("Finding Intel High Definition Audio (HDA) devices\n");

    list_for_each(curbus, &(pci->bus_list))
    {
        struct pci_bus *bus = list_entry(curbus, struct pci_bus, bus_node);

        DEBUG("Searching PCI bus %u for HDA devices\n", bus->num);

        list_for_each(curdev, &(bus->dev_list))
        {
            struct pci_dev *pdev = list_entry(curdev, struct pci_dev, dev_node);
            struct pci_cfg_space *cfg = &pdev->cfg;

            DEBUG("Device %u is a %x:%x\n", pdev->num, cfg->vendor_id, cfg->device_id);

            // only detect specific chip at the moment
            if (cfg->vendor_id == 0x8086 && cfg->device_id == 0x2668)
            {
                DEBUG("Compatible HDA Device Found\n");
                struct hda_pci_dev *hdev;

                hdev = malloc(sizeof(struct hda_pci_dev));
                if (!hdev)
                {
                    ERROR("Cannot allocate device\n");
                    return -1;
                }

                memset(hdev, 0, sizeof(*hdev));

                spinlock_init(&hdev->lock);

                // BAR handling will eventually be done by common code in PCI

                // we expect one bar exists, just memory-mapped registers
                // and this will be bar 0
                // check to see if there are no others
                int foundmem = 0;
                int foundio = 0;
                for (int i = 0; i < 6; i++)
                {
                    uint32_t bar = pci_cfg_readl(bus->num, pdev->num, 0, 0x10 + i * 4);
                    uint32_t size;
                    DEBUG("bar %d: 0x%0x\n", i, bar);
                    if (i >= 1 && bar != 0)
                    {
                        DEBUG("Not expecting this to be a non-empty bar...\n");
                    }
                    if (!(bar & 0x1))
                    {
                        uint8_t mem_bar_type = (bar & 0x6) >> 1;
                        if (mem_bar_type != 0)
                        {
                            ERROR("Cannot handle memory bar type 0x%x\n", mem_bar_type);
                            return -1;
                        }
                    }

                    // determine size
                    // write all 1s, get back the size mask
                    pci_cfg_writel(bus->num, pdev->num, 0, 0x10 + i * 4, 0xffffffff);
                    // size mask comes back + info bits
                    size = pci_cfg_readl(bus->num, pdev->num, 0, 0x10 + i * 4);

                    // mask all but size mask
                    if (bar & 0x1)
                    {
                        // I/O
                        size &= 0xfffffffc;
                    }
                    else
                    {
                        // memory
                        size &= 0xfffffff0;
                    }
                    size = ~size;
                    size++;

                    // now we have to put back the original bar
                    pci_cfg_writel(bus->num, pdev->num, 0, 0x10 + i * 4, bar);

                    if (!size)
                    {
                        // non-existent bar, skip to next one
                        continue;
                    }

                    if (size > 0 && i >= 1)
                    {
                        ERROR("unexpected hda pci bar with size>0!\n");
                        return -1;
                    }

                    if (bar & 0x1)
                    {
                        hdev->ioport_start = bar & 0xffffffc0;
                        hdev->ioport_end = hdev->ioport_start + size;
                        foundio = 1;
                    }
                    else
                    {
                        hdev->mem_start = bar & 0xfffffff0;
                        hdev->mem_end = hdev->mem_start + size;
                        foundmem = 1;
                    }

                }

                // for now, privilege the memory interface
                if (foundmem)
                {
                    hdev->method = MEMORY;
                }
                else if (foundio)
                {
                    hdev->method = IO;
                }
                else
                {
                    hdev->method = NONE;
                    ERROR("Device has no register access method... Impossible...\n");
                    panic("Device has no register access method... Impossible...\n");
                    return -1;
                }

                hdev->pci_dev = pdev;

                INFO("Found HDA device: bus=%u dev=%u func=%u: pci_intr=%u intr_vec=%u ioport_start=%p ioport_end=%p mem_start=%p mem_end=%p access_method=%s\n",
                     bus->num, pdev->num, 0,
                     hdev->pci_intr, hdev->intr_vec,
                     hdev->ioport_start, hdev->ioport_end,
                     hdev->mem_start, hdev->mem_end,
                     hdev->method == IO ? "IO" : hdev->method == MEMORY ? "MEMORY" : "NONE");


                list_add(&hdev->hda_node, &dev_list);
            }
        }
    }
    return 0;
}

static void hda_reset(struct hda_pci_dev *dev)
{
    uint32_t glob_ctrl_val = hda_pci_read_regl(dev, GLOB_CTRL);
    DEBUG("GLOB CTRL: %x\n", glob_ctrl_val);

    hda_pci_write_regl(dev, GLOB_CTRL, glob_ctrl_val | CRST_MASK);
    DEBUG("Wrote 1 to CRST bit of GLOB CTRL\n");

    DEBUG("Waiting for device to come out of reset...\n");

    // Wait until the CRST bit is flipped to one (device is done resetting)
    while(((glob_ctrl_val = hda_pci_read_regl(dev, GLOB_CTRL)) & CRST_MASK) == 0)
    {
        DEBUG("GLOB CTRL is %x!\n", glob_ctrl_val);
    }
    DEBUG("Device is out of reset\n");
}
static void hda_test(struct hda_pci_dev *dev)
{
    uint8_t major, minor;
    uint16_t cap;
    minor = hda_pci_read_regb(dev, MINOR);
    major = hda_pci_read_regb(dev, MAJOR);
    cap = hda_pci_read_regw(dev, GLOB_CAP);
    DEBUG("Major: %x, Minor: %x, Glob Cap: %x\n", major, minor, cap);
}

static void hda_discover_codecs(struct hda_pci_dev *dev)
{
    // Discover codecs
    uint16_t statests = 0;

    statests = hda_pci_read_regw(dev, STATESTS);
    DEBUG("Statests Value before delay: %x\n", statests);

    udelay(CODECS_DELAY * 100); // Wait for codecs to be initialized

    statests = hda_pci_read_regw(dev, STATESTS);
    DEBUG("Statests Value after delay: %x\n", statests);
}

static void get_caps(struct hda_pci_dev *d)
{
    d->gcap.val = hda_pci_read_regw(d, GCAP);
    d->vmin = hda_pci_read_regb(d, VMIN);
    d->vmaj = hda_pci_read_regb(d, VMAJ);
    d->outpay = hda_pci_read_regw(d, OUTPAY);
    d->inpay = hda_pci_read_regw(d, INPAY);

    DEBUG("device is version 0x%x.0x%x %s with %d input words/frame, %d output words/frame\n",
          d->vmaj, d->vmin, d->gcap.ok64 ? "64ok" : "32only", d->inpay, d->outpay);
    DEBUG("  %d output streams, %d input streams, %d bidir streams, and %d sdos\n",
          d->gcap.oss, d->gcap.iss, d->gcap.bss, NUM_SDO(d->gcap));
}

static void reset(struct hda_pci_dev *d)
{
    gctl_t gctl;

    // verify that corb/rirb run are off (0) and stream runs are off (0)

    gctl.val = hda_pci_read_regl(d, GCTL);

    DEBUG("Initial gctl = %08x\n", gctl.val);

    gctl.crst = 0;

    hda_pci_write_regl(d, GCTL, gctl.val); // assert reset

    DEBUG("reset asserted\n");

    udelay(1000); // wait a beat - probably need non-magic number here

    gctl.crst = 1;

    hda_pci_write_regl(d, GCTL, gctl.val); // deassert reset

    DEBUG("reset deasserted\n");

    // now wait for it to finish
    do
    {
        gctl.val = hda_pci_read_regl(d, GCTL);
    }
    while (gctl.crst != 1);

    DEBUG("reset completed, gctl = %08x\n", gctl.val);


}

// Assumption: reset has occured just before this
// in which case the codecs must have checked in by now
static void discover_codecs(struct hda_pci_dev *d)
{
    statests_t s;
    int i;

    s.val = hda_pci_read_regw(d, STATESTS);

    DEBUG("statests = %x\n", s.val);

    for (i = 0; i < SDIMAX; i++)
    {
        if (SDIWAKE(s, i))
        {
            DEBUG("codec %d exists\n", i);
            d->codecs[i].valid = 1;
        }
    }
}

static void setup_corb(struct hda_pci_dev *d)
{
    corbctl_t cc;
    corbsize_t cs;

    cc.val = hda_pci_read_regb(d, CORBCTL);
    cc.corbrun = 0; // turn off dma
    hda_pci_write_regb(d, CORBCTL, cc.val);
    DEBUG("CORB stopped\n");

    cs.val = hda_pci_read_regb(d, CORBSIZE);
    if (CORBSIZECAP_HAS_256(cs))
    {
        d->corb.size = 256;
        cs.corbsize = 2;
    }
    else if (CORBSIZECAP_HAS_16(cs))
    {
        d->corb.size = 16;
        cs.corbsize = 1;
    }
    else if (CORBSIZECAP_HAS_2(cs))
    {
        d->corb.size = 2;
        cs.corbsize = 0;
    }
    else
    {
        // uh?
        d->corb.size = 256;
        cs.corbsize = 2;
    }
    hda_pci_write_regb(d, CORBSIZE, cs.val);
    DEBUG("CORB size set to %d\n", d->corb.size);

    corbubase_t cu = (uint32_t)(((uint64_t)d->corb.buf) >> 32);
    corblbase_t cl = (uint32_t)(((uint64_t)d->corb.buf));

    hda_pci_write_regl(d, CORBUBASE, cu);
    hda_pci_write_regl(d, CORBLBASE, cl);

    DEBUG("CORB DMA address set to %x:%x (%p)\n", cu, cl, d->corb.buf);

    d->corb.cur_write = 0; // we will advance this to 1 on first queue

    corbrp_t rp;

    rp.val = hda_pci_read_regw(d, CORBRP);
    rp.corbrprst = 1;
    rp.corbrp = 0;
    hda_pci_write_regw(d, CORBRP, rp.val);
    // now wait for reset to "take"
    do
    {
        rp.val = hda_pci_read_regw(d, CORBRP);
    }
    while (!rp.corbrprst);
    // now write it again, with reset off
    rp.corbrprst = 0;
    rp.corbrp = 0;
    hda_pci_write_regw(d, CORBRP, rp.val);
    // now wait for reset-off to "take"
    do
    {
        rp.val = hda_pci_read_regw(d, CORBRP);
    }
    while (rp.corbrprst);

    DEBUG("CORB DMA RP is configured\n");


    // now reset the write pointer (apparently necessary)

    corbwp_t cwp;

    cwp.val = hda_pci_read_regw(d, CORBWP);
    cwp.corbwp = 0;
    hda_pci_write_regw(d, CORBWP, cwp.val);

    DEBUG("CORB DMA WP is reset to zero\n");

    cc.val = hda_pci_read_regb(d, CORBCTL);
    cc.corbrun = 1; // turn on dma
    hda_pci_write_regb(d, CORBCTL, cc.val);
    DEBUG("CORB started\n");

}

static void setup_rirb(struct hda_pci_dev *d)
{
    rirbctl_t rc;
    rirbsize_t rs;

    rc.val = hda_pci_read_regb(d, RIRBCTL);
    rc.rirbdmaen = 0; // turn off dma
    hda_pci_write_regb(d, RIRBCTL, rc.val);
    DEBUG("RIRB stopped\n");

    rs.val = hda_pci_read_regb(d, RIRBSIZE);
    if (RIRBSIZECAP_HAS_256(rs))
    {
        d->rirb.size = 256;
        rs.rirbsize = 2;
    }
    else if (RIRBSIZECAP_HAS_16(rs))
    {
        d->rirb.size = 16;
        rs.rirbsize = 1;
    }
    else if (RIRBSIZECAP_HAS_2(rs))
    {
        d->rirb.size = 2;
        rs.rirbsize = 0;
    }
    else
    {
        // uh?
        d->rirb.size = 256;
        rs.rirbsize = 2;
    }
    hda_pci_write_regb(d, RIRBSIZE, rs.val);
    DEBUG("RIRB size set to %d\n", d->rirb.size);

    rirbubase_t ru = (uint32_t)(((uint64_t)d->rirb.buf) >> 32);
    rirblbase_t rl = (uint32_t)(((uint64_t)d->rirb.buf));

    hda_pci_write_regl(d, RIRBUBASE, ru);
    hda_pci_write_regl(d, RIRBLBASE, rl);

    DEBUG("RIRB DMA address set to %x:%x (%p)\n", ru, rl, d->rirb.buf);

    d->rirb.cur_read = 0; // we will wait on slot 1 when we begin

    rirbwp_t wp;

    wp.val = hda_pci_read_regw(d, RIRBWP);
    wp.rirbwprst = 1;
    wp.rirbwp = 0;
    hda_pci_write_regw(d, RIRBWP, wp.val);

    // the RIRB does not need us to wait on the reset, or toggle it,
    // apparently, so this is then done...

    DEBUG("RIRB DMA WP is configured\n");

    // setup the rirb interrupt counter
    // note that qemu seems to have this wrong - manual
    // indicates that 0=>256, but qemu code has 0=>0
    rintcnt_t ri;

    ri.val = hda_pci_read_regw(d, RINTCNT);
    ri.rintcnt = d->rirb.size - 1;  // 256 => 255, all ones
    hda_pci_write_regw(d, RINTCNT, ri.val);

    DEBUG("RIRB interrupt count set to %d\n", d->rirb.size - 1);

    rc.val = hda_pci_read_regb(d, RIRBCTL);
    rc.rirbdmaen = 1; // turn on dma
    rc.rintctl = 1; // turn on interrupts on empty
    rc.rirboic = 1; // turn on interrupts on overrun
    hda_pci_write_regb(d, RIRBCTL, rc.val);
    DEBUG("RIRB started\n");

}

static void corb_show(struct hda_pci_dev *d, int count)
{
    int i;
    for (i = 0; i < count; i++)
    {
        DEBUG("corb[%d] = %08x\n", i, d->corb.buf[i].val);
    }
}


static void corb_queue_request(struct hda_pci_dev *d, codec_req_t *r)
{
    corbwp_t wp;
    corbrp_t rp;

    //DEBUG("corb queue 0x%08x\n",r->val);

    d->corb.cur_write = (d->corb.cur_write + 1) % d->corb.size;

    // wait for a slot to open
    // probably bogus, but shouldn't matter until we wrap around
    do
    {
        rp.val = hda_pci_read_regw(d, CORBRP);
    }
    while (rp.corbrp == d->corb.cur_write);

    //DEBUG("corb rp=%d cur_write=%d\n",rp.corbrp,d->corb.cur_write);

    d->corb.buf[d->corb.cur_write].val = r->val;

    // make sure this write becomes visible to other cpus
    // this should also make it visible to anything else that is
    // coherent.
    __asm__ __volatile__ ("mfence" : : : "memory");

    // maybe, but should not be needed...
    clflush(&d->corb.buf[d->corb.cur_write].val);

    wp.val = hda_pci_read_regw(d, CORBWP);
    wp.corbwp = (wp.corbwp + 1) % d->corb.size;
    hda_pci_write_regw(d, CORBWP, wp.val);

    //DEBUG("corb request queued - wp=%d cur_write=%d\n", wp.corbwp, d->corb.cur_write);
}

static void rirb_show(struct hda_pci_dev *d, int count)
{
    int i;

    for (i = 0; i < count; i++)
    {
        DEBUG("rirb[%d] = %08x %08x\n", i, d->rirb.buf[i].resp, d->rirb.buf[i].resp_ex.val);
    }
}

static void rirb_dequeue_response(struct hda_pci_dev *d, codec_resp_t *r)
{
    rirbwp_t wp;
    corbrp_t rp;
    corbwp_t cwp;
    corbsts_t cst;
    corbctl_t cct;

    //DEBUG("rirb dequeue response\n");

    do
    {
        //corb_show(d,2);
        //rirb_show(d,2);
        wp.val = hda_pci_read_regw(d, RIRBWP);
        //rp.val = hda_pci_read_regw(d,CORBRP);
        //cwp.val = hda_pci_read_regw(d,CORBWP);
        //cst.val = hda_pci_read_regb(d,CORBSTS);
        //cct.val = hda_pci_read_regb(d,CORBCTL);
        //DEBUG("wp=%d rp=%d\n",wp.rirbwp, rp.corbrp);
    }
    while (wp.rirbwp == d->rirb.cur_read);


    //DEBUG("ready to dequeue - rirb wp=%d rirb cur_read=%d\n",wp.rirbwp, d->rirb.cur_read);

    //corb_show(d,32);
    //rirb_show(d,32);

    *r = d->rirb.buf[wp.rirbwp];

    d->rirb.cur_read = (d->rirb.cur_read + 1) % d->rirb.size;

    //DEBUG("dequeue done - response is %08x:%08x\n",r->resp,r->resp_ex.val);

}


static void setup_interrupts(struct hda_pci_dev *d)
{
    intctl_t c;

    c.val = 0;
    c.cie = 1;
    c.gie = 1;

    hda_pci_write_regl(d, INTCTL, c.val);

    DEBUG("interrupts enabled:  global and controller\n");
}


static void transact(struct hda_pci_dev *d, int codec, int nid, int indirect, uint32_t verb, codec_resp_t *rp)
{
    codec_req_t rq;

    rq.val = 0;
    rq.CAd = codec;
    rq.nid = nid;
    rq.indirect = indirect;
    rq.verb = verb;
    corb_queue_request(d, &rq);
    rirb_dequeue_response(d, rp);
}


static void setup_codec(struct hda_pci_dev *d, int codec)
{
    codec_resp_t rp;
    int node = 0;
    int root_node = 0;
    int start_node = 0;
    int n_nodes = 0;

    transact(d, codec, root_node, 0, MAKE_VERB_8(GET_PARAM, VENDOR), &rp);
    DEBUG("Interrogating root node. Codec vendor %04x device %04x\n", rp.resp >> 16 & 0xffff, rp.resp & 0xffff);
    transact(d, codec, root_node, 0, MAKE_VERB_8(GET_PARAM, SUBORD_NODE_COUNT), &rp);
    DEBUG("Getting subordinate nodes information from root node. Starting node: %d total nodes: %d\n", rp.resp >> 16 & 0xff, rp.resp & 0xff);

    start_node = rp.resp >> 16 & 0xff;
    n_nodes = rp.resp & 0xff;
    
    for(node = start_node; node < start_node + n_nodes; node++)
    {
        DEBUG("============================NODE %d===========================\n", node);
        transact(d, codec, node, 0, MAKE_VERB_8(GET_PARAM, VENDOR), &rp);
        DEBUG("codec vendor %04x device %04x\n", rp.resp >> 16 & 0xffff, rp.resp & 0xffff);

        transact(d, codec, node, 0, MAKE_VERB_8(GET_PARAM, REVISION), &rp);
        DEBUG("major %d minor %d revid %d stepping %d\n",
              rp.resp >> 20 & 0xf,
              rp.resp >> 16 & 0xf,
              rp.resp >> 8  & 0xff,
              rp.resp >> 0  & 0xff);


        transact(d, codec, node, 0, MAKE_VERB_8(GET_PARAM, SUBORD_NODE_COUNT), &rp);
        DEBUG("starting node %d total nodes %d\n", rp.resp >> 16 & 0xff, rp.resp & 0xff);

        transact(d, codec, node, 0, MAKE_VERB_8(GET_PARAM, FUNC_GROUP_TYPE), &rp);
        DEBUG("func group type %x unsol %d\n", rp.resp & 0xff, rp.resp >> 8 & 0x1);

        transact(d, codec, node, 0, MAKE_VERB_8(GET_PARAM, AUDIO_FUNC_GROUP_CAPS), &rp);
        DEBUG("audio caps beep %d input delay %d output delay %d\n", rp.resp & 0x10000, rp.resp >> 8 & 0xf, rp.resp & 0xf);

        transact(d, codec, node, 0, MAKE_VERB_8(GET_PARAM, AUDIO_WIDGET_CAPS), &rp);
        DEBUG("audio widget caps %08x type %x channels %d\n", rp.resp, rp.resp >> 20 & 0xf, 1 + (((rp.resp >> 12) & 0xe) | (rp.resp & 0x1)));
        transact(d, codec, node, 0, MAKE_VERB_8(GET_PARAM, PCM_SIZES_AND_RATES), &rp);
        DEBUG("pcm sizes and rates %08x\n", rp.resp);
        DEBUG("  %s %s %s %s %s\n",
              rp.resp & 0x100000 ? "32bit" : "",
              rp.resp & 0x080000 ? "24bit" : "",
              rp.resp & 0x040000 ? "20bit" : "",
              rp.resp & 0x020000 ? "16bit" : "",
              rp.resp & 0x010000 ? "8bit" : "");

        transact(d, codec, node, 0, MAKE_VERB_8(GET_PARAM, STREAM_FORMATS), &rp);
        DEBUG("stream formats %08x\n", rp.resp);
    }
    
    
}

static void setup_codecs(struct hda_pci_dev *d)
{
    int i;
    for (i = 0; i < SDIMAX; i++)
    {
        if (d->codecs[i].valid)
        {
            setup_codec(d, i);
        }
    }
}

static int handler(excp_entry_t *e, excp_vec_t v, void *priv_data)
{
    struct hda_pci_dev *d = (struct hda_pci_dev *) priv_data;

    //DEBUG("We got interrupt %d for device at %p\n", v, d);

    intsts_t is;

    is.val = hda_pci_read_regl(d, INTSTS);

    //DEBUG("Interrupt status %08x\n", is.val);

    // handle
    

    IRQ_HANDLER_END();

    return 0;
}


// 4.2.1 - PCI config
//     To Do: make sure all enabled, interrupt enabled, dma enabled
//            MSI configured
// 4.2.2 - Reset - set CRST (+8,0), read-wait for it to flip to 1
// 4.3   - Codecs 0 read STATESTS bits after reset to see which have
//         flipped to 1 - these are the codecs that exist
// 4.4   - Codec control - establish ring buffers for requests (CORB)
///        and responses (RIRB).   Read CORBSZCAP to determine ring
//         buf size (typically 256 entries, 4 bytes each), 128 byte aligned, coherent
//             CORB  - WP => driver push  - RP => device pull
//             RIRB  - WP => device push  - RP => driver pull
//                 push => write to WP+(1,2,3,...)*4, then update WP to last entry (notify)
//         CORB run bit => device observes CORB via DMA
//         requests are 4 byte "verbs" that are device (codec) specific
//         responses are 8 byte quanitities (codec number, solicited/unsolcited, 4 byte response)
//         responses can produce interrupts (after n response pusehs, for example)
//         request/response can be done via polling as well
// 4.5   - Streams and channels

static int bringup_device(struct hda_pci_dev *dev)
{
    DEBUG("Bringing up device %u:%u.%u. Starting Address is: %x\n",
          dev->pci_dev->bus->num, dev->pci_dev->num, dev->pci_dev->fun, dev->mem_start);

    // configure interrupts
    if (dev->pci_dev->msi.type == PCI_MSI_32 || dev->pci_dev->msi.type == PCI_MSI_64)
    {
        int num_vecs = dev->pci_dev->msi.num_vectors_needed;
        int base_vec;
        int i;

        if (idt_find_and_reserve_range(num_vecs, 1, (ulong_t*)&base_vec))
        {
            ERROR("Unable to reserve %d interrupt table slots\n", num_vecs);
            return -1;
        }
        if (pci_dev_enable_msi(dev->pci_dev, base_vec, num_vecs, 0))
        {
            ERROR("Failed to enable MSI on device...\n");
            return -1;
        }
        for (i = base_vec; i < base_vec + num_vecs; i++)
        {
            if (register_int_handler(i, handler, dev))
            {
                ERROR("Failed to register interrupt %d\n", i);
                return -1;
            }
        }

        for (i = base_vec; i < base_vec + num_vecs; i++)
        {
            if (pci_dev_unmask_msi(dev->pci_dev, i))
            {
                ERROR("Failed to unmask MSI interrupt %d\n", i);
                return -1;
            }
        }

        DEBUG("Enabled MSI for vectors [%d,%d)\n", base_vec, base_vec + num_vecs);
    }
    else
    {
        ERROR("Device does not support MSI...\n");
        return -1;
    }

    // now make sure pci config space command register is acceptable
    uint16_t cmd = pci_dev_cfg_readw(dev->pci_dev, 0x4);
    cmd &= ~0x0400; // turn off interrupt disable
    cmd |= 0x7; // make sure bus master, memory and io space are enabled
    DEBUG("writing PCI command register to 0x%x\n", cmd);
    pci_dev_cfg_writew(dev->pci_dev, 0x4, cmd);

    uint16_t status = pci_dev_cfg_readw(dev->pci_dev, 0x6);
    DEBUG("reading PCI status register as 0x%x\n", status);

    // Initialize device here...

    get_caps(dev);

    reset(dev);

    discover_codecs(dev);

    setup_corb(dev);

    setup_rirb(dev);

    setup_interrupts(dev);

    setup_codecs(dev);

    // Make sure can read from device. For debugging
    //hda_test(dev);

    //hda_reset(dev);



    // Clear STATESTS register
    //hda_pci_write_regw(dev, STATESTS, STATESTS_INT_MASK);

    // Reset device
    //hda_reset(dev);

    // Discover Codecs
    //hda_discover_codecs(dev);

    return 0;
}

// for a future sound device abstraction
// this would eventually go into snddev.h
struct nk_snd_dev_int
{
    struct nk_dev_int  dev_int;
    // nothing specific to sound cards so far
};

static struct nk_snd_dev_int ops;


static int bringup_devices()
{
    struct list_head *curdev, tx_node;
    int rc;

    rc = 0;

    DEBUG("Bringing up HDA devices\n");
    int num = 0;

    list_for_each(curdev, &(dev_list))
    {
        struct hda_pci_dev *dev = list_entry(curdev, struct hda_pci_dev, hda_node);
        int ret = bringup_device(dev);
        if (ret)
        {
            ERROR("Bringup of HDA device failed\n");
            rc = -1;
        }
        char buf[80];
        snprintf(buf, 80, "hda%d", num_devs++);
        dev->nk_dev = nk_dev_register(buf, NK_DEV_GENERIC, 0, (struct nk_dev_int *)&ops, dev);
        if (!dev->nk_dev)
        {
            ERROR("Unable to register device %s\n", buf);
            return -1;
        }
    }

    return rc;

}

int hda_pci_init(struct naut_info * naut)
{
    spinlock_init(&global_lock);

    if (discover_devices(naut->sys.pci))
    {
        ERROR("Discovery failed\n");
        return -1;
    }

    return bringup_devices();
}


int hda_pci_deinit()
{
    // should really scan list of devices and tear down...
    INFO("deinited\n");
    return 0;
}


#define SDNCTL 0x80
// Stream Descriptor n Control, 3 bytes
typedef union {
    uint32_t val;
    struct {
        uint8_t srst:1;     // stream reset
        uint8_t run:1;      // stream run
        uint8_t ioce:1;     // interrupt on completion enable
        uint8_t feie:1;     // FIFO error interrupt enable
        uint8_t deie:1;     // descriptor error interrupt enable
        uint16_t resv:11;    // 15:5 reserved
        uint8_t stripe:2;   // stripe control
        uint8_t tp:1;       // traffic priority
        uint8_t dir:1;      // bidirectional direction control
        uint8_t strm:4;     // stream number
        // rest is reserved / padding
    }__attribute__((packed));
} __attribute__((packed)) sdncbl_t;


// Audio data
struct audio_data {
    void *buffer;
    uint64_t size;
    char *format;
};

void start_stream(struct hda_pci_dev *dev, struct audio_data data) {
    /* enable SIE */

    /* set stripe control */

    /* set DMA start (run) */ 
    
    /* set interrupt mask */
}


uint64_t get_chunk_size(uint64_t current_offset, uint64_t total_size) {
    uint64_t len = 1280000;
    /* if it's the first chunk and size is too small, split to have at least two entries */
    if (current_offset == 0 && len >= total_size) {
        return len / 2;
    } 
    /* if the remainder is less than len, the remainder will be an entry */
    if (len >= total_size - current_offset) {
        return total_size - current_offset;
    }
    /* If our default len is too small, use a bigger one */
    if (total_size > len * 256) {
        return total_size / 256;
    }
    return len;
}

#define MAX_BDL_ENTIRES 256
#define LAST_VALID_INDEX 0x8C
#define BDL_LOWER 0x98
#define BDL_UPPER 0x9C

// Buffer Descriptor List Entry
typedef struct {
    uint64_t address;
    uint32_t length;
    uint32_t ioc:1;
    uint32_t reserved:31;
} __attribute__((packed, aligned(128))) bdle_t;

// Buffer Descriptor List
struct hda_bdl {
    bdle_t buf[MAX_BDL_ENTIRES];
} __attribute__((aligned(128)));

void initialize_bdl(struct hda_pci_dev *dev, struct audio_data data) {
    assert((data.size & 0x3F) == 0);
    /* split the data into BDL entries */
    struct hda_bdl *bdl;
	bdl = malloc(sizeof(struct hda_bdl));
    uint16_t index = 0;
    uint64_t curr_offset = 0;
    while (data.size >= curr_offset) {
        uint64_t chunksize = get_chunk_size(curr_offset, data.size) & 0x80;
        curr_offset += chunksize;
        bdl->buf[index].address = ((uint64_t)data.buffer) + curr_offset;
        bdl->buf[index].length = chunksize;
        /* we don't want interupts */
        bdl->buf[index].ioc = 0;
        index++;
    }
    /* program the stream LVI (last valid index) of the BDL */
    uint16_t LVI = hda_pci_read_regw(dev, LAST_VALID_INDEX);
    LVI &= 0xFF00;
    LVI |= index - 1;
    hda_pci_write_regw(dev, LAST_VALID_INDEX, LVI);

    /* program the BDL address */
    uint32_t bdl_u = (uint32_t)(((uint64_t)bdl)>>32);
    uint32_t bdl_l = ((uint32_t)(((uint64_t)bdl))) & 0xFFC0 ;

    hda_pci_write_regl(dev, BDL_LOWER, bdl_l);
    hda_pci_write_regl(dev, BDL_UPPER, bdl_u);    
}

void setup_stream(struct hda_pci_dev *dev, struct audio_data data) {
    /* make sure the run bit is zero for SD */  

    /* program the stream_tag */

    /* program the length of samples in cyclic buffer */

    /* program the stream format */

    /* initialize BDL */
    initialize_bdl(dev, data);

    /* enable the position buffer */

    /* set the interrupt enable bits in the descriptor control register */

}

/* function to call externally to play audio */
void audio_from_buffer(struct hda_pci_dev *dev, void *buffer, uint64_t size, char *format) {
    /* package up a struct */
    struct audio_data data;
    data.buffer = buffer;
    data.size = size;
    data.format = format;
    
    /* setup stream */
    setup_stream(dev, data);

    /* start stream */
    start_stream(dev, data);
}
