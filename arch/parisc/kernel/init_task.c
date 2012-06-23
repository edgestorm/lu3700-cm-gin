
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/init_task.h>
#include <linux/mqueue.h>

#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>

static struct signal_struct init_signals = INIT_SIGNALS(init_signals);
static struct sighand_struct init_sighand = INIT_SIGHAND(init_sighand);
/*
 * Initial task structure.
 *
 * We need to make sure that this is 16384-byte aligned due to the
 * way process stacks are handled. This is done by having a special
 * "init_task" linker map entry..
 */
union thread_union init_thread_union __init_task_data
	__attribute__((aligned(128))) =
		{ INIT_THREAD_INFO(init_task) };

#if PT_NLEVELS == 3
/* NOTE: This layout exactly conforms to the hybrid L2/L3 page table layout
 * with the first pmd adjacent to the pgd and below it. gcc doesn't actually
 * guarantee that global objects will be laid out in memory in the same order 
 * as the order of declaration, so put these in different sections and use
 * the linker script to order them. */
pmd_t pmd0[PTRS_PER_PMD] __attribute__ ((__section__ (".data..vm0.pmd"), aligned(PAGE_SIZE)));
#endif

pgd_t swapper_pg_dir[PTRS_PER_PGD] __attribute__ ((__section__ (".data..vm0.pgd"), aligned(PAGE_SIZE)));
pte_t pg0[PT_INITIAL * PTRS_PER_PTE] __attribute__ ((__section__ (".data..vm0.pte"), aligned(PAGE_SIZE)));

/*
 * Initial task structure.
 *
 * All other task structs will be allocated on slabs in fork.c
 */
EXPORT_SYMBOL(init_task);

__asm__(".data");
struct task_struct init_task = INIT_TASK(init_task);
