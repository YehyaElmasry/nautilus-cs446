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
#define assert(cond) \
    if (cond) {} \
    else { \
    ERROR("Assert failed in %s: %d\n", __FILE__, __LINE__);\
    }\
// there is currently no abstract interface for sound devices
// so some simple test functions would go here
//
// A sound device interface would go into nautilus/snddev.h
//

struct hda_pci_dev;

// Assumes 8-bit audio with 2 channels
void play_tone(struct hda_pci_dev *dev, uint64_t tone_frequency, uint64_t sampling_frequency, uint32_t duration);

/* function to call externally to play audio */
void audio_from_buffer(struct hda_pci_dev *dev, void *buffer, uint64_t size);
#endif
