

struct ivshm_regs {
	uint32_t id;
	uint32_t max_peers;
	uint32_t int_control;
	uint32_t doorbell;
	uint32_t state;
};

//sz =size
struct ivshmem_dev_data {
	uint16_t bdf;
	struct ivshm_regs *registers;
	uint32_t *state_table;
	uint32_t state_table_sz;
	uint32_t *rw_section;
	uint64_t rw_section_sz;
	uint32_t *in_sections;
	uint32_t *out_section;
	uint64_t out_section_sz;
	uint32_t *msix_table;
	uint32_t id;
	int msix_cap;
};

enum pci_cell { PCI_LINUX_ROOT, PCI_FREERTOS, PCI };
enum ivshmem_stats { DISABLED , ACTIVATED ,UPDATED};

void print_shmem(struct ivshmem_dev_data *d);
uint32_t ivshmem_irq(struct ivshmem_dev_data *dev);
void ivshmem_init_device(struct ivshmem_dev_data *d);
void ivshmem_send_irq(struct ivshmem_dev_data *d,enum pci_cell pci_cell);
void ivshmem_write_state(struct ivshmem_dev_data *d, enum ivshmem_stats stats );
void ivshmem_irq_enable(struct ivshmem_dev_data *dev);

//methoden:
//write/read to memory