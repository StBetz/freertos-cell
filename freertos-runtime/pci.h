#define COMM_REGION_GENERIC_HEADER					\
	/** Communication region magic JHCOMM */			\
	char signature[6];						\
	/** Communication region ABI revision */			\
	uint16_t revision;							\
	/** Cell state, initialized by hypervisor, updated by cell. */	\
	volatile uint32_t cell_state;					\
	/** Message code sent from hypervisor to cell. */		\
	volatile uint32_t msg_to_cell;					\
	/** Reply code sent from cell to hypervisor. */			\
	volatile uint32_t reply_from_cell;					\
	/** Holds static flags, see JAILHOUSE_COMM_FLAG_*. */		\
	uint32_t flags;							\
	/** Debug console that may be accessed by the inmate. */	\
	struct jailhouse_console console;				\
	/** Base address of PCI memory mapped config. */		\
	uint64_t pci_mmconfig_base;

#define COMM_REGION_BASE	0x80000000
#define comm_region	((struct jailhouse_comm_region *)COMM_REGION_BASE)

#define PCI_CFG_VENDOR_ID	0x000
#define PCI_CFG_DEVICE_ID	0x002
#define PCI_CFG_CAP_PTR		0x034
#define PCI_ID_ANY		0xffff
#define PCI_CFG_STATUS		0x006
# define PCI_STS_CAPS		(1 << 4)
#define PCI_CAP_VENDOR		0x09
#define PCI_CAP_MSIX		0x11
#define PCI_DEV_CLASS_OTHER	0xff
#define PCI_INT_PINT 		0x3e

struct jailhouse_console {
		uint64_t address;
		uint32_t size;
		uint16_t type;
		uint16_t flags;
		uint32_t divider;
		uint32_t gate_nr;
		uint64_t clock_reg;
} __attribute__((packed));


struct jailhouse_comm_region {
	COMM_REGION_GENERIC_HEADER;
	uint8_t gic_version;
	uint8_t padding[7];
	uint64_t gicd_base;
	uint64_t gicc_base;
	uint64_t gicr_base;
	uint32_t vpci_irq_base;
} __attribute__((packed));