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
 *
 * This is free software.  You are permitted to use,
 * redistribute, and modify it as specified in the file "LICENSE.txt".
 */

#ifndef __HDA_PCI_INTERNAL
#define __HDA_PCI_INTERNAL

#include <test/piano.h>
#include <test/champions.h>

struct hda_pci_dev;

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

typedef union
{
    uint8_t val;
    struct
    {
        uint8_t vrefen: 2;          // VRefEn[1:0]
        uint8_t vrefen2: 1;         // VrefEn[2]
        uint8_t resv: 2;            // Rsvd
        uint8_t in_enable: 1;       // In Enable
        uint8_t out_enable: 1;      // Out Enable
        uint8_t h_phn_enable: 1;    // H-Phn Enable
    } __attribute__((packed));
} __attribute__((packed)) pinwgctl_t;

typedef union
{
    uint8_t val;
    struct
    {
        uint8_t channel: 4;         // Stream
        uint8_t stream: 4;          // Channel
    } __attribute__((packed));
} __attribute__((packed)) outwgctl_t;


#define SDNCTL 0x80 + (OUTPUT_STREAM_NUM * 0x20)
#define SD0STS 0x83 + (OUTPUT_STREAM_NUM * 0x20)
// Stream Descriptor n Control, 3 bytes
typedef union
{
    struct
    {
        uint8_t byte_1;
        uint8_t byte_2;
        uint8_t byte_3;
    } __attribute__((packed));
    struct
    {
        uint8_t srst: 1;    // stream reset
        uint8_t run: 1;     // stream run
        uint8_t ioce: 1;    // interrupt on completion enable
        uint8_t feie: 1;    // FIFO error interrupt enable
        uint8_t deie: 1;    // descriptor error interrupt enable
        uint16_t resv: 11;  // 15:5 reserved
        uint8_t stripe: 2;  // stripe control
        uint8_t tp: 1;      // traffic priority
        uint8_t dir: 1;     // bidirectional direction control
        uint8_t strm_num: 4; // stream number
    } __attribute__((packed));
} __attribute__((packed)) sdnctl_t;

// Cyclic buffer length
#define SDNCBL 0x88  + (OUTPUT_STREAM_NUM * 0x20)
typedef uint32_t sdcbl_t;

// Audio data
struct audio_data
{
    void *buffer;
    uint64_t size;
    char *format;
} __attribute__((packed));

#define INTCTL 0x20

#define MAX_BDL_ENTIRES 256
#define LPIB 0x84 + (OUTPUT_STREAM_NUM * 0x20)
#define LAST_VALID_INDEX 0x8C + (OUTPUT_STREAM_NUM * 0x20)
#define BDL_LOWER 0x98 + (OUTPUT_STREAM_NUM * 0x20)
#define BDL_UPPER 0x9C + (OUTPUT_STREAM_NUM * 0x20)
#define STRM_DESC 0x92 + (OUTPUT_STREAM_NUM * 0x20)

// Buffer Descriptor List Entry
typedef struct
{
    uint64_t address: 64;
    uint32_t length: 32;
    uint8_t ioc: 1;
    uint32_t reserved: 31;
} __attribute__((packed, aligned(16))) bdle_t;

// Buffer Descriptor List
typedef struct
{
    bdle_t buf[MAX_BDL_ENTIRES];
} __attribute__((aligned(128))) hda_bdl;

#define DPLBASE 0x70
typedef struct
{

} __attribute__((packed, aligned(128))) dplbase_t;

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
#define GET_POWSTATE  0xf05
#define SET_POWSTATE  0x705
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
#define VENDOR                      0x0 // also includes device id
#define REVISION                    0x2
#define SUBORD_NODE_COUNT           0x4
#define FUNC_GROUP_TYPE             0x5
#define AUDIO_FUNC_GROUP_CAPS       0x8
#define AUDIO_WIDGET_CAPS           0x9
#define PCM_SIZES_AND_RATES         0xa // needed for playback/recording
#define STREAM_FORMATS              0xb // ""
#define PIN_CAPS                    0xc
#define AMP_CAPS                    0xd
#define CON_LIST_LEN                0xe
#define POWER_STATES                0xf
#define PROC_CAPS                   0x10
#define GPIO_COUNT                  0x11
#define VOL_KNOB_CAPS               0x13


#define OUTPUT_STREAM_NUM   4 // First output stream in QEMU. See https://github.com/qemu/qemu/blob/ad88e4252f09c2956b99c90de39e95bab2e8e7af/hw/audio/intel-hda.c#L891
#define STREAM_NUM          5 // Arbitrarily chosen
#define PIANO_SIZE          1210848 // For testing. Change later
#define CHAMPIONS_SIZE      3336776 // For testing. Change later
#define ONE_KHZ_SIZE        1920000 // For testing. Change later

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

#endif
