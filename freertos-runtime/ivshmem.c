#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pci.h"
#include "ivshmem.h"

typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;

#define JAILHOUSE_INMATE_MEM_PAGE_DIR_LEN	4
#define PAGE_SIZE	(4 * 1024ULL)

typedef enum { true = 1, false = 0 } bool;
enum map_type { MAP_CACHED, MAP_UNCACHED };

#define VENDORID			0x110a
#define DEVICEID			0x4106

#define BAR_BASE			0xff000000
#define PCI_CFG_COMMAND		0x004
#define IVSHMEM_CFG_STATE_TAB_SZ	0x04
#define IVSHMEM_CFG_RW_SECTION_SZ	0x08
#define IVSHMEM_CFG_OUT_SECTION_SZ	0x10
# define PCI_CMD_MEM		(1 << 1)
# define PCI_CMD_MASTER		(1 << 2)
#define IVSHMEM_CFG_ADDRESS		0x18
#define DEFAULT_IRQ_BASE	(comm_region->vpci_irq_base + 32)
#define PCI_CFG_BAR		0x010
#define JAILHOUSE_SHMEM_PROTO_UNDEFINED	0x0000


#define HUGE_PAGE_SIZE		(2 * 1024 * 1024ULL)
#define HUGE_PAGE_MASK		(~(HUGE_PAGE_SIZE - 1))

/* Common definitions for page table structure in long descriptor format */
#define LONG_DESC_BLOCK 0x1
#define LONG_DESC_TABLE 0x3

#define LATTR_CONT		(1 << 12)
#define LATTR_AF		(1 << 10)
#define LATTR_INNER_SHAREABLE	(3 << 8)
#define LATTR_MAIR(n)		(((n) & 0x3) << 2)

#define LATTR_AP(n)		(((n) & 0x3) << 6)
#define LATTR_AP_RW_EL1		LATTR_AP(0x0)

#define PGD_INDEX(addr)		((addr) >> 30)
#define PMD_INDEX(addr)		(((addr) >> 21) & 0x1ff)

#define dsb(domain)	asm volatile("dsb " #domain ::: "memory")

extern const char stack_top[];

static inline void enable_irqs(void)
{
	asm volatile("cpsie if"); /* enable IRQs and FIQs */
}

static inline void disable_irqs(void)
{
	asm volatile("cpsid if"); /* disable IRQs and FIQs */
}

static inline void __attribute__((noreturn)) halt(void)
{
	while (1)
		asm volatile("wfi" : : : "memory");
}

static inline void __attribute__((noreturn)) stop(void)
{
	disable_irqs();
	halt();
}



static inline u8 mmio_read8(void *address)
{
	return *(volatile u8 *)address;
}

static inline void mmio_write8(void *address, u8 value)
{
	*(volatile u8 *)address = value;
}

static inline u16 mmio_read16(void *address)
{
	return *(volatile u16 *)address;
}

static inline void mmio_write16(void *address, u16 value)
{
	*(volatile u16 *)address = value;
}

static inline u32 mmio_read32(void *address)
{
	return *(volatile u32 *)address;
}

static inline void mmio_write32(void *address, u32 value)
{
	*(volatile u32 *)address = value;
}

static inline u64 mmio_read64(void *address)
{
	return *(volatile u64 *)address;
}


//PCI read and write functions
static void *pci_get_device_mmcfg_base(uint16_t bdf)
{
	void *mmcfg = (void *)(unsigned long)comm_region->pci_mmconfig_base;

	return mmcfg + ((unsigned long)bdf << 12);
}

uint32_t pci_read_config(uint16_t bdf, unsigned int addr, unsigned int size)
{
	void *cfgaddr = pci_get_device_mmcfg_base(bdf) + addr;

	switch (size) {
	case 1:
		return mmio_read8(cfgaddr);
	case 2:
		return mmio_read16(cfgaddr);
	case 4:
		return mmio_read32(cfgaddr);
	default:
		return -1;
	}
}

void pci_write_config(uint16_t bdf, unsigned int addr, uint32_t value, unsigned int size)
{
	void *cfgaddr = pci_get_device_mmcfg_base(bdf) + addr;

	switch (size) {
	case 1:
		mmio_write8(cfgaddr, value);
		break;
	case 2:
		mmio_write16(cfgaddr, value);
		break;
	case 4:
		mmio_write32(cfgaddr, value);
		break;
	}
}


int pci_find_device(uint16_t vendor, uint16_t device, uint16_t start_bdf)
{
	unsigned int bdf;
	uint16_t id;

	for (bdf = start_bdf; bdf < 0x10000; bdf++) {
		id = pci_read_config(bdf, PCI_CFG_VENDOR_ID, 2);
		if (id == PCI_ID_ANY || (vendor != PCI_ID_ANY && vendor != id))
			continue;
		if (device == PCI_ID_ANY ||
		    pci_read_config(bdf, PCI_CFG_DEVICE_ID, 2) == device)
			return bdf;
	}
	return -1;
}

int pci_find_cap(uint16_t bdf, uint16_t cap)
{
	uint8_t pos = PCI_CFG_CAP_PTR - 1;

	if (!(pci_read_config(bdf, PCI_CFG_STATUS, 2) & PCI_STS_CAPS))
		return -1;

	while (1) {
		pos = pci_read_config(bdf, pos + 1, 1);
		if (pos == 0)
			return -1;
		if (pci_read_config(bdf, pos, 1) == cap)
			return pos;
	}
}
//

static inline void synchronization_barrier(void)
{
	dsb(ish);
}

void *memset_1(void *s, int c, unsigned long n)
{
	u8 *p = s;

	while (n-- > 0)
		*p++ = c;
	return s;
}

unsigned long heap_pos = (unsigned long)stack_top;

void *alloc(unsigned long size, unsigned long align)
{
	unsigned long base = (heap_pos + align - 1) & ~(align - 1);

	heap_pos = base + size;
	return (void *)base;
}


void *zalloc(unsigned long size, unsigned long align)
{
	void *base = alloc(size, align);
	memset_1(base, 0, size);

	return base;
}

static u64 __attribute__((aligned(4096)))
	page_directory[JAILHOUSE_INMATE_MEM_PAGE_DIR_LEN];


void map_range(void *start, unsigned long size, enum map_type map_type)
{
	u64 vaddr, pmd_entry;
	unsigned pgd_index;
	u64 *pmd;

	vaddr = (unsigned long)start;

	size += (vaddr & ~HUGE_PAGE_MASK) + HUGE_PAGE_SIZE - 1;
	size &= HUGE_PAGE_MASK;

	while (size) {
		pgd_index = PGD_INDEX(vaddr);
		if (!(page_directory[pgd_index] & LONG_DESC_TABLE)) {
			pmd = zalloc(PAGE_SIZE, PAGE_SIZE);
			/* ensure the page table walker will see the zeroes */
			synchronization_barrier();

			page_directory[pgd_index] =
				(unsigned long)pmd | LONG_DESC_TABLE;
		} else {
			pmd = (u64*)(unsigned long)
				(page_directory[pgd_index] & ~LONG_DESC_TABLE);
		}

		pmd_entry = vaddr & HUGE_PAGE_MASK;
		pmd_entry |= LATTR_AF | LATTR_INNER_SHAREABLE | \
			     LATTR_AP_RW_EL1 | LONG_DESC_BLOCK;
		if (map_type == MAP_CACHED)
			pmd_entry |= LATTR_MAIR(0);
		else
			pmd_entry |= LATTR_MAIR(1);

		pmd[PMD_INDEX(vaddr)] = pmd_entry;

		size -= HUGE_PAGE_SIZE;
		vaddr += HUGE_PAGE_SIZE;
	}

	/*
	 * As long es we only add entries and do not modify entries, a
	 * synchronization barrier is enough to propagate changes. Otherwise we
	 * need to flush the TLB.
	 */
	synchronization_barrier();
}


//Ivshmem functions

uint64_t pci_cfg_read64(uint16_t bdf, unsigned int addr)
{
	return pci_read_config(bdf, addr, 4) |
		((uint64_t)pci_read_config(bdf, addr + 4, 4) << 32);
}

 void print_shmem(struct ivshmem_dev_data *d)
{
	printf("state[0] = %d\n", d->state_table[0]);
	printf("state[1] = %d\n", d->state_table[1]);
	printf("state[2] = %d\n", d->state_table[2]);
	printf("rw[0] = %d\n", d->rw_section[0]);
	printf("rw[1] = %d\n", d->rw_section[1]);
	printf("rw[2] = %d\n", d->rw_section[2]);
	printf("in@0x0000 = %d\n", d->in_sections[0/4]);
	printf("in@0x2000 = %d\n", d->in_sections[0x2000/4]);
	printf("in@0x4000 = %d\n", d->in_sections[0x4000/4]);
}

/* HERE read status or something like that*/
uint32_t ivshmem_irq(struct ivshmem_dev_data *dev)
{
	return 0;
}

void ivshmem_init_device(struct ivshmem_dev_data *d)
{
	unsigned long baseaddr, addr, size;
	int vndr_cap;
	uint32_t max_peers;
	int bdf;
	unsigned int class_rev;
    
	bdf=pci_find_device(VENDORID, DEVICEID, 0);
	
	if (bdf == -1) {
		printf("IVSHMEM: No PCI devices found .. nothing to do.\n");
		
	}

	printf("IVSHMEM: Found device at %02x:%02x.%x\n",
	       bdf >> 8, (bdf >> 3) & 0x1f, bdf & 0x3);
	class_rev = pci_read_config(bdf, 0x8, 4);

	if (class_rev != (PCI_DEV_CLASS_OTHER << 24 |
	    JAILHOUSE_SHMEM_PROTO_UNDEFINED << 8)) {
		printf("IVSHMEM: class/revision %08x, not supported\n",
		       class_rev);
		stop();
	}
    
	d->bdf=bdf;
	vndr_cap = pci_find_cap(d->bdf, PCI_CAP_VENDOR);
	if (vndr_cap < 0) {
		printf("IVSHMEM ERROR: missing vendor capability\n");
		stop();
	}

	d->registers = (struct ivshm_regs *)BAR_BASE;
	pci_write_config(d->bdf, PCI_CFG_BAR, (unsigned long)d->registers, 4);
	printf("IVSHMEM: bar0 is at %p\n", d->registers);

	d->msix_table = (uint32_t*)(BAR_BASE + PAGE_SIZE);
	pci_write_config(d->bdf, PCI_CFG_BAR + 4,
			 (unsigned long)d->msix_table, 4);
	printf("IVSHMEM: bar1 is at %p\n", d->msix_table);

	pci_write_config(d->bdf, PCI_CFG_COMMAND,
			 (PCI_CMD_MEM | PCI_CMD_MASTER), 2);

	

	map_range((void *)BAR_BASE, 2 * PAGE_SIZE, MAP_UNCACHED);

	d->id = mmio_read32(&d->registers->id);
	printf("IVSHMEM: ID is %d\n", d->id);

	max_peers = mmio_read32(&d->registers->max_peers);
	printf("IVSHMEM: max. peers is %d\n", max_peers);

	d->state_table_sz =
		pci_read_config(d->bdf, vndr_cap + IVSHMEM_CFG_STATE_TAB_SZ, 4);
	d->rw_section_sz =
		pci_cfg_read64(d->bdf, vndr_cap + IVSHMEM_CFG_RW_SECTION_SZ);
	d->out_section_sz =
		pci_cfg_read64(d->bdf, vndr_cap + IVSHMEM_CFG_OUT_SECTION_SZ);
	baseaddr = pci_cfg_read64(d->bdf, vndr_cap + IVSHMEM_CFG_ADDRESS);

	addr = baseaddr;
	d->state_table = (uint32_t *)addr;

	addr += d->state_table_sz;
	d->rw_section = (uint32_t *)addr;

	addr += d->rw_section_sz;
	d->in_sections = (uint32_t *)addr;

	addr += d->id * d->out_section_sz;
	d->out_section = (uint32_t *)addr;

	printf("IVSHMEM: state table is at %p\n", d->state_table);
	printf("IVSHMEM: R/W section is at %p\n", d->rw_section);
	printf("IVSHMEM: input sections start at %p\n", d->in_sections);
	printf("IVSHMEM: output section is at %p\n", d->out_section);

	/*size = d->state_table_sz + d->rw_section_sz +
		max_peers * d->out_section_sz;
	map_range((void *)baseaddr, size, MAP_CACHED);
	*/
	d->msix_cap = -1;
	
}

/*send interrupt to other cell*/
void ivshmem_send_irq(struct ivshmem_dev_data *d, enum pci_cell pci_cell)    
{
	disable_irqs();
	printf("\nIVSHMEM: sending IRQ to PCI_ID %d\n",pci_cell);
	enable_irqs();
	mmio_write32(&d->registers->doorbell,  (pci_cell << 16));
}

/* write the passed status into the status table (possible status notfification can be adjust in the enum stats and must be include into the IRQ handler )*/
void ivshmem_write_state(struct ivshmem_dev_data *d, enum ivshmem_stats stats ){
 mmio_write32(&(d->registers->state), stats ); 
}

/* Activate the IRQ for the passed PCI ivshmem device and set his stat us to activate*/
void ivshmem_irq_enable(struct ivshmem_dev_data *dev){
 mmio_write32(&(dev->registers->int_control), 1);  
 ivshmem_write_state(&dev,ACTIVATED);
}
