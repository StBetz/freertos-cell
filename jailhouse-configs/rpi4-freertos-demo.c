#include <jailhouse/types.h>
#include <jailhouse/cell-config.h>


#define MBYTE(m) ((m)<<20)

#define GUEST_MEM_START 0x20000000
#define GUEST_MEM_SIZE MBYTE(16)

struct {
	struct jailhouse_cell_desc cell;
	__u64 cpus[1];
	struct jailhouse_memory mem_regions[7];
	struct jailhouse_irqchip irqchips[2];
	struct jailhouse_pci_device pci_devices[1];
} __attribute__((packed)) config = {
	.cell = {
		.signature = JAILHOUSE_CELL_DESC_SIGNATURE,
		.revision = JAILHOUSE_CONFIG_REVISION,
		.name = "FreeRTOS",
		.flags = JAILHOUSE_CELL_PASSIVE_COMMREG | JAILHOUSE_CELL_AARCH32 ,

		.cpu_set_size = sizeof(config.cpus),
		.num_memory_regions = ARRAY_SIZE(config.mem_regions),
		.num_irqchips = ARRAY_SIZE(config.irqchips),
		.num_pci_devices = ARRAY_SIZE(config.pci_devices),

		.vpci_irq_base = 186-32,

		.console = {
			.address = 0xfe215040,
			.type = JAILHOUSE_CON_TYPE_8250,
			.flags = JAILHOUSE_CON_ACCESS_MMIO |
				 JAILHOUSE_CON_REGDIST_4,
		},
	},

	.cpus = {
		0b0100,
	},

	.mem_regions = {
		/* IVSHMEM shared memory regions (demo) */
		{
			.phys_start = 0x1faf0000,
			.virt_start = 0x1faf0000,
			.size = 0x1000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_ROOTSHARED,
		},
		{
			.phys_start = 0x1faf1000,
			.virt_start = 0x1faf1000,
			.size = 0x9000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_ROOTSHARED,
		},
		{
			.phys_start = 0x1fafa000,
			.virt_start = 0x1fafa000,
			.size = 0x2000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_ROOTSHARED,
		},
		{
			.phys_start = 0x1fafc000,
			.virt_start = 0x1fafc000,
			.size = 0x2000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_ROOTSHARED,
		},
		{
			.phys_start = 0x1fafe000,
			.virt_start = 0x1fafe000,
			.size = 0x2000,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_ROOTSHARED,
		},
		/* RAM */ {
			.phys_start = GUEST_MEM_START,
			.virt_start = 0,
			.size = GUEST_MEM_SIZE,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_EXECUTE | JAILHOUSE_MEM_LOADABLE,
		},
		/* UART 1 */ {
			.phys_start = 0xfe215040,
			.virt_start = 0xfe215040,
			.size = 0x40,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_8 | JAILHOUSE_MEM_IO_32 |
				 JAILHOUSE_MEM_ROOTSHARED,
		},
		/* CCU (Hack)  {
			.phys_start = 0x01c2006c,
			.virt_start = 0x01c2006c,
			.size = 0x4,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE | 
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32 | JAILHOUSE_MEM_ROOTSHARED,
		},
#define PIO_P7_REG (0x01c20800 + 7*0x24 + 0x0c)
		 PIO port 7: blinking LED  {
			.phys_start = PIO_P7_REG,
			.virt_start = PIO_P7_REG,
			.size = 0x8,
			.flags = JAILHOUSE_MEM_READ | JAILHOUSE_MEM_WRITE |
				JAILHOUSE_MEM_IO | JAILHOUSE_MEM_IO_32,
		},*/
	},
	.irqchips = {
		/* GIC */ {
			.address = 0xff841000,
			.pin_base = 32,
			/* AUX Interrupt for mini UART*/
			.pin_bitmap = {
				0,
				0,
				1 << (125 - 96),
				0,
			},
		},
		/* GIC */ {
			.address = 0xff841000,
			.pin_base = 160,
			/* Interrupt for Shared Memory */
			.pin_bitmap = {
				 1<<(186-160),
				 0,
				 0,
				 0
			},
		},
	},
	.pci_devices = {
		{
			.type = JAILHOUSE_PCI_TYPE_IVSHMEM,
			.domain= 1,
			.bdf = 0 << 3,
			.bar_mask = JAILHOUSE_IVSHMEM_BAR_MASK_INTX,
			.shmem_regions_start = 0,
			.shmem_dev_id = 3,
			.shmem_peers = 3,
			.shmem_protocol = JAILHOUSE_SHMEM_PROTO_UNDEFINED,
		},
	},
};
