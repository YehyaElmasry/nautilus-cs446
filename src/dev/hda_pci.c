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
#include <math.h>
#include <nautilus/shell.h>

#include <test/piano.h>
//#include <test/one_khz.h>

struct hda_pci_dev *hda_dev; // TODO: Used for play handler. Remove and pass a pointer to dev to handle_play instead

static void read_sd_control(struct hda_pci_dev *dev, sdnctl_t *sd_control);
static void write_sd_control(struct hda_pci_dev *dev, sdnctl_t *sd_control);

static int
handle_play (char * buf, void * priv)
{
    uint64_t frequency, duration;

    sscanf(buf, "play %d %d", &frequency, &duration);

    play_tone(hda_dev, frequency, 48000, duration);
    
    nk_vc_printf("Frequency %d duration %d\n", frequency, duration);

    return 0;
}

static struct shell_cmd_impl play_impl = {
    .cmd      = "play",
    .help_str = "play frequency duration",
    .handler  = handle_play,
};
nk_register_shell_cmd(play_impl);

static int
handle_play_piano (char * buf, void * priv)
{
    char *play_buf = malloc(PIANO_SIZE);
    memcpy(play_buf, piano, PIANO_SIZE);
    audio_from_buffer(hda_dev, play_buf, PIANO_SIZE);

    return 0;
}

static struct shell_cmd_impl play_piano_impl = {
    .cmd      = "play_piano",
    .help_str = "play_piano",
    .handler  = handle_play_piano,
};
nk_register_shell_cmd(play_piano_impl);

// On QEMU using -soundhw hda we see the following:
//
//0:4.0 : 8086:2668 0103c 0010s MSI(off,MSI64,nn=1,bv=0,nv=0,tc=0) legacy(l=11,p=1)

// for protection of global state in the driver
static spinlock_t global_lock;

// number of devices (used to assign names)
static int num_devs = 0;

// list of hda devices
static struct list_head dev_list;

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
    c.sie = -1;

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
    int subnodes = 0;
    int n_subnodes = 0;

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
        subnodes = rp.resp >> 16 & 0xff;
        n_subnodes = rp.resp & 0xff;

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

        transact(d, codec, node, 0, MAKE_VERB_8(GET_PARAM, PIN_CAPS), &rp);
        DEBUG("pin caps %08x\n", rp.resp);
    }

    for(node = subnodes; node < subnodes + n_subnodes; node++)
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

        transact(d, codec, node, 0, MAKE_VERB_8(GET_PARAM, PIN_CAPS), &rp);
        DEBUG("pin caps %08x\n", rp.resp);
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

int done = 0;

static int handler(excp_entry_t *e, excp_vec_t v, void *priv_data)
{
    struct hda_pci_dev *d = (struct hda_pci_dev *) priv_data;

    intsts_t is;

    is.val = hda_pci_read_regl(d, INTSTS);

    //DEBUG("Interrupt %d status %08x\n", v,  is.val);
    //DEBUG("We are at byte %d of %d\n", hda_pci_read_regl(d, LPIB),hda_pci_read_regl(d, SDNCBL));

    if (is.val == 0xc0000010)
    {
        uint32_t bdl_l = hda_pci_read_regl(d, BDL_LOWER);
        uint32_t bdl_u = hda_pci_read_regl(d, BDL_UPPER);

        uint64_t bdl_addr = (((uint64_t) bdl_u) << 32) | bdl_l;

        DEBUG("BDL address: 0x%016lx\n", bdl_addr);

        for (int i = 0; i < 10; i++)
        {
            DEBUG("Index: %d value: 0x%016lx\n", i, ((uint64_t*)bdl_addr)[i]);
        }
        sdnctl_t sd_control;
        read_sd_control(d, &sd_control);

        DEBUG("Interrupt handler: stream desciptor run: %d\n", sd_control.run);
        DEBUG("Interrupt handler: stream desciptor status: 0x%02x\n", hda_pci_read_regb(d, SD0STS));
    }

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
        DEBUG("CONFIGURING INTERUPTS\n");
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
        DEBUG("Trying to register interupts for %d through %d\n", base_vec, base_vec + num_vecs - 1);
        for (i = base_vec; i < base_vec + num_vecs; i++)
        {
            if (register_int_handler(i, handler, dev))
            {
                ERROR("Failed to register interrupt %d\n", i);
                return -1;
            }
            else
            {
                DEBUG("Registered int handler for %d\n", i);
            }
        }

        for (i = base_vec; i < base_vec + num_vecs; i++)
        {
            if (pci_dev_unmask_msi(dev->pci_dev, i))
            {
                ERROR("Failed to unmask MSI interrupt %d\n", i);
                return -1;
            } 
            else 
            {
                DEBUG("Unmasked msi for %d\n", i);
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

    hda_dev = dev;  //TODO: Dont' use global variable. Pass dev pointer to shell handlers instead

    //play_tone(dev, 440, 48000, 4);

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

static void write_sd_control(struct hda_pci_dev *dev, sdnctl_t *sd_control)
{
    DEBUG("Write SD Control: byte1: %02x byte2: %02x byte3: %02x\n",
          sd_control->byte_1, sd_control->byte_2, sd_control->byte_3);

    hda_pci_write_regb(dev, SDNCTL, sd_control->byte_1);
    hda_pci_write_regb(dev, SDNCTL + 1, sd_control->byte_2);
    hda_pci_write_regb(dev, SDNCTL + 2, sd_control->byte_3);
}

static void read_sd_control(struct hda_pci_dev *dev, sdnctl_t *sd_control)
{
    sd_control->byte_1  = hda_pci_read_regb(dev, SDNCTL);
    sd_control->byte_2 = hda_pci_read_regb(dev, SDNCTL + 1);
    sd_control->byte_3 = hda_pci_read_regb(dev, SDNCTL + 2);
    DEBUG("Read SD Control: byte1: %02x byte2: %02x byte3: %02x\n",
          sd_control->byte_1, sd_control->byte_2, sd_control->byte_3);
}

static void start_stream(struct hda_pci_dev *dev)
{
    sdnctl_t sd_control;

    /* enable interupts */
    //hda_pci_write_regl(dev, INTCTL, 0x);

    /* set stripe control */
    read_sd_control(dev, &sd_control);
    sd_control.stripe = 0;
    write_sd_control(dev, &sd_control);

    /* set DMA start and interrupt mask */
    read_sd_control(dev, &sd_control);
    sd_control.ioce = 1;
    sd_control.feie = 1;
    sd_control.deie = 1;
    write_sd_control(dev, &sd_control);

    /* set run bit to 1 */
    read_sd_control(dev, &sd_control);
    sd_control.run = 1;
    write_sd_control(dev, &sd_control);
}

static uint64_t get_chunk_size(uint64_t current_offset, uint64_t total_size)
{
    const uint64_t max_length = 0xFFFFFFFF; // Length is a 32-bit quantity
    if (current_offset == 0)
    {
        return (total_size > max_length) ? max_length : total_size / 2; // Need at least two entries in the BDL
    }
    else
    {
        uint64_t remaining_size = total_size - current_offset;
        return (remaining_size > max_length) ? max_length : remaining_size;
    }
    
}

static void initialize_bdl(struct hda_pci_dev *dev, struct audio_data data)
{
    assert((data.size & 0x3F) == 0); // Ensure 128-byte aligned

    /* split the data into BDL entries */
    hda_bdl *bdl = (hda_bdl *) malloc(sizeof(hda_bdl));
    DEBUG("Allocated %d bytes for bdl\n", sizeof(hda_bdl));
    DEBUG("BDL malloc address: 0x%016lx\n", bdl);
    uint16_t index = 0;
    uint64_t curr_offset = 0;
    uint64_t chunksize = 0;
    while (curr_offset < data.size)
    {
        DEBUG("Address of BDLE is 0x%016lx\n", &(bdl->buf[index]));
        chunksize = get_chunk_size(curr_offset, data.size);
        DEBUG("Initialize BDL: index: %d chuncksize: %d data.size: %d data.buffer: 0x%016lx\n", index, chunksize, data.size, data.buffer);
        bdl->buf[index].reserved = 0; // Reserved must be 0. See page 56 of Intel HDA Manual
        bdl->buf[index].address = (((uint64_t)data.buffer) + curr_offset) & (~0x7FUL);
        DEBUG("BDL address: 0x%016lx\n", bdl->buf[index].address);
        bdl->buf[index].length = chunksize;
        bdl->buf[index].ioc = 0;
        index++;
        curr_offset += chunksize;
    }

    bdl->buf[index-1].ioc = 1;

    DEBUG("Setup %d buffers with %d/%d bytes\n", index, curr_offset, data.size);

    /* program the stream LVI (last valid index) of the BDL */
    uint16_t LVI = hda_pci_read_regw(dev, LAST_VALID_INDEX);
    DEBUG("LVI Read: 0x%04x\n", LVI);
    LVI &= 0xFF00; // Preserve bits [15:8]
    LVI |= index - 1;
    hda_pci_write_regw(dev, LAST_VALID_INDEX, LVI);

    LVI = hda_pci_read_regw(dev, LAST_VALID_INDEX);
    DEBUG("LVI Read: 0x%04x\n", LVI);

    /* program the BDL address */
    uint32_t bdl_u = (uint32_t)(((uint64_t)bdl) >> 32);
    uint32_t bdl_l = ((uint32_t)(((uint64_t)bdl))) & 0xFFFFC0;

    hda_pci_write_regl(dev, BDL_LOWER, bdl_l);
    hda_pci_write_regl(dev, BDL_UPPER, bdl_u);

    uint64_t bdl_addr = (((uint64_t) bdl_u) << 32) | bdl_l;

    DEBUG("BDL address: 0x%016lx\n", bdl_addr);

    for (int i = 0; i < 10; i++)
    {
        DEBUG("Index: %d value: 0x%016lx\n", i, ((uint64_t*)bdl)[i]);
    }

    DEBUG("BDL Write: 0x%08x:%08x\n", bdl_u, bdl_l);

    DEBUG("BDL Read: 0x%08x:%08x\n", hda_pci_read_regl(dev, BDL_UPPER), hda_pci_read_regl(dev, BDL_LOWER));
}

static void setup_stream(struct hda_pci_dev *dev, struct audio_data data)
{
    /* reset stream */
    sdnctl_t sd_control;

    read_sd_control(dev, &sd_control);
    sd_control.srst = 1;
    write_sd_control(dev, &sd_control);

    do
    {
        read_sd_control(dev, &sd_control);
    } while (sd_control.srst == 1); // Wait till stream is done resetting
    

    /* make sure the run bit and reset bits are zero for SD */
    read_sd_control(dev, &sd_control);
    sd_control.run = 0;
    sd_control.srst = 0;
    write_sd_control(dev, &sd_control);

    /* program the stream_tag */
    read_sd_control(dev, &sd_control);
    sd_control.strm_num = STREAM_NUM;
    write_sd_control(dev, &sd_control);
    read_sd_control(dev, &sd_control);

    /* program the length of samples in cyclic buffer */
    sdcbl_t sd_cbl = data.size;
    hda_pci_write_regl(dev, SDNCBL, sd_cbl);

    /* program the stream format */
    // Not doing anything now. Stick with defaults
    DEBUG("Stream Descriptor: 0x%04x\n", hda_pci_read_regw(dev, STRM_DESC));

    /* initialize BDL */
    initialize_bdl(dev, data);

    /* enable the position buffer */
    // TODO: Need to enable? No

    /* set the interrupt enable bits in the descriptor control register */
    read_sd_control(dev, &sd_control);
    sd_control.deie = 0;
    sd_control.feie = 0;
    sd_control.ioce = 0;
    write_sd_control(dev, &sd_control);
}

static void setup_output_widget(struct hda_pci_dev *dev, int codec, int output_widget_node)
{
    DEBUG("Setting up audio output widget (node %d)\n", output_widget_node);

    codec_resp_t rp;

    // Configure the DAC to use the stream number (step 5 is OSDEV)
    outwgctl_t output_wg_ctl;
    transact(dev, codec, output_widget_node, 0, MAKE_VERB_8(GET_CONVCTL, 0), &rp);
    output_wg_ctl.val = (uint8_t) rp.resp;
    DEBUG("Node %d converter stream channel: %08x. Stream: %02x, Channel: %02x\n", \
          output_widget_node, rp.resp, output_wg_ctl.stream, output_wg_ctl.channel);

    output_wg_ctl.stream = STREAM_NUM;
    output_wg_ctl.channel = 0;
    transact(dev, codec, output_widget_node, 0, MAKE_VERB_8(SET_CONVCTL, output_wg_ctl.val), &rp);

    transact(dev, codec, output_widget_node, 0, MAKE_VERB_8(GET_CONVCTL, 0), &rp);
    DEBUG("Node %d converter stream channel: %08x. Stream: %02x, Channel: %02x\n", \
          output_widget_node, rp.resp, output_wg_ctl.stream, output_wg_ctl.channel);

    // Make sure that the DAC is fully powered (step 6 in OSDEV)
    transact(dev, codec, output_widget_node, 0, MAKE_VERB_8(GET_POWSTATE, 0), &rp);
    DEBUG("Node %d power state: %08x\n", output_widget_node, rp.resp);

    // Make sure output is unmuted and volume is suitable (step 7 in OSDEV)
    transact(dev, codec, output_widget_node, 0, MAKE_VERB_16(GET_GAINMUTE, 0xA000), &rp);
    DEBUG("Node %d gain mute: %08x\n", output_widget_node, rp.resp);
}

static void setup_pin_widget(struct hda_pci_dev *dev, int codec, int pin_widget_node)
{
    DEBUG("Enabling speaker (node %d)\n", pin_widget_node);

    codec_resp_t rp;
    pinwgctl_t pin_wg_cntl;

    transact(dev, codec, pin_widget_node, 0, MAKE_VERB_8(GET_PINWGTCTL, 0), &rp);
    DEBUG("Get node %d pin widget control: %08x\n", pin_widget_node, rp.resp);

    pin_wg_cntl.val = (uint8_t) rp.resp;
    DEBUG("Get node %d pin widget control: %08x\n", pin_widget_node, pin_wg_cntl.val);

    pin_wg_cntl.out_enable = 1;
    transact(dev, codec, pin_widget_node, 0, MAKE_VERB_8(SET_PINWGTCTL, pin_wg_cntl.val), &rp);

    transact(dev, codec, pin_widget_node, 0, MAKE_VERB_8(GET_PINWGTCTL, 0), &rp);
    DEBUG("Get node %d pin widget control: %08x\n", pin_widget_node, rp.resp);
    pin_wg_cntl.val = (uint8_t) rp.resp;

    DEBUG("Node %d pin widget control output enable: %d\n", pin_widget_node, pin_wg_cntl.out_enable);
}

// Assumes 8-bit audio with 2 channels
void play_tone(struct hda_pci_dev *dev, uint64_t tone_frequency, uint64_t sampling_frequency, uint32_t duration)
{
    uint64_t buf_len = sampling_frequency * duration * 4;
    //uint64_t buf_len = ONE_KHZ_SIZE / 10;
    uint8_t *buf = (uint8_t *) malloc(buf_len);
    create_sine_wave(buf, buf_len, tone_frequency, sampling_frequency);
    audio_from_buffer(dev, buf, buf_len);
}

// Assumes 8-bit audio with 2 channels
void create_sine_wave(uint8_t *buffer, uint64_t buffer_len, uint64_t tone_frequency, uint64_t sampling_frequency)
{
    for (int i = 0, j = 0; i < buffer_len; i+=4, j++)
    {
        double x = (double) j * 2.0 * M_PI * (double) tone_frequency / (double) sampling_frequency;
        double sin_val = sin(x);
        
        buffer[i] = 0;
        buffer[i + 1] = (uint8_t) (sin_val * 127.0);
        buffer[i + 2] = 0;
        buffer[i + 3] = (uint8_t) (sin_val * 127.0);
    }
}

/* function to call externally to play audio */
void audio_from_buffer(struct hda_pci_dev *dev, void *buffer, uint64_t size)
{
    /* package up a struct */
    struct audio_data data;
    data.buffer = buffer;
    data.size = size;
    //data.format = format;

    /* setup stream */
    setup_stream(dev, data);

    /* enable audio output widget */
    setup_output_widget(dev, 0, 2);     // TODO: Get output widget and codec automatically

    /* enable speaker */
    setup_pin_widget(dev, 0, 3);        // TODO: Get pin widget number and codec automatically

    /* start stream */
    start_stream(dev);

    codec_resp_t rp;
    transact(dev, 0, 0, 0, MAKE_VERB_8(GET_CONLIST, 0), &rp);
    DEBUG("Get node 0 connection list: %08x\n", rp.resp);

    transact(dev, 0, 1, 0, MAKE_VERB_8(GET_CONLIST, 0), &rp);
    DEBUG("Get node 1 connection list: %08x\n", rp.resp);

    transact(dev, 0, 2, 0, MAKE_VERB_8(GET_CONLIST, 0), &rp);
    DEBUG("Get node 2 connection list: %08x\n", rp.resp);

    transact(dev, 0, 3, 0, MAKE_VERB_8(GET_CONLIST, 0), &rp);
    DEBUG("Get node 3 connection list: %08x\n", rp.resp);
}
