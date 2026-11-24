/*
 * insn_count.c - QEMU TCG plugin: count guest instructions executed.
 *
 * Each translated block gets an inline "add N" on a counter, where N is the
 * number of instructions in the block. At exit, prints "insns=<count>".
 *
 * This is an INSTRUCTION count, not a cycle count. QEMU does not model the
 * Cortex-M4 pipeline, wait states or flash accelerator, so cycles (and so
 * time and energy) on a real STM32F446RE must be measured on the board.
 *
 * Build: make -C tools/qemu_plugin   (fetches qemu-plugin.h for QEMU 8.2.2;
 * the header is GPL-2.0-or-later and is not stored in this repo).
 */
#include <inttypes.h>
#include <stdio.h>
#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

static uint64_t insn_count;

static void tb_trans(qemu_plugin_id_t id, struct qemu_plugin_tb *tb)
{
    (void)id;
    qemu_plugin_register_vcpu_tb_exec_inline(tb, QEMU_PLUGIN_INLINE_ADD_U64,
                                             &insn_count, qemu_plugin_tb_n_insns(tb));
}

static void at_exit(qemu_plugin_id_t id, void *p)
{
    (void)id; (void)p;
    char buf[64];
    snprintf(buf, sizeof(buf), "insns=%" PRIu64 "\n", insn_count);
    qemu_plugin_outs(buf);
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id, const qemu_info_t *info,
                                           int argc, char **argv)
{
    (void)info; (void)argc; (void)argv;
    qemu_plugin_register_vcpu_tb_trans_cb(id, tb_trans);
    qemu_plugin_register_atexit_cb(id, at_exit, NULL);
    return 0;
}
