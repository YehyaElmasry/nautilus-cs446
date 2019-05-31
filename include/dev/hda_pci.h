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

#ifndef __HDA_PCI
#define __HDA_PCI

#include <nautilus/nautilus.h>


int hda_pci_init(struct naut_info * naut);
int hda_pci_deinit();

// there is currently no abstract interface for sound devices
// so some simple test functions would go here
//
// A sound device interface would go into nautilus/snddev.h
//

// Register offsets from the device starting address
#define GLOB_CAP    0x00
#define MINOR       0x02
#define MAJOR       0x03
#define GLOB_CTRL   0x08
//#define STATESTS    0x0E

// Bits masks within registers
#define CRST_MASK           0x1
#define STATESTS_INT_MASK   0xFF
// Timing
#define CODECS_DELAY 521 * 1000 // 521 us = 25 frames



typedef union {
    uint8_t val;
    struct {
        uint8_t vrefen:2;           // VRefEn[1:0]
        uint8_t vrefen2:1;          // VrefEn[2]
        uint8_t resv:2;             // Rsvd
        uint8_t in_enable:1;        // In Enable
        uint8_t out_enable:1;       // Out Enable
        uint8_t h_phn_enable:1;     // H-Phn Enable
    }__attribute__((packed));
} __attribute__((packed)) pinwgctl_t;

#endif
