// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2010 Loongson Inc. & Lemote Inc. &
 *                    Institute of Computing Technology
 * Author:  Xiang Gao, gaoxiang@ict.ac.cn
 *          Huacai Chen, chenhc@lemote.com
 *          Xiaofu Meng, Shuangshuang Zhang
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/export.h>
#include <linux/nodemask.h>
#include <linux/swap.h>
#include <linux/memblock.h>
#include <linux/bootmem.h>
#include <linux/pfn.h>
#include <linux/acpi.h>
#include <linux/highmem.h>
#include <linux/pci.h>
#include <asm/page.h>
#include <asm/pgalloc.h>
#include <asm/sections.h>
#include <linux/irq.h>
#include <linux/efi.h>
#include <asm/bootinfo.h>
#include <asm/time.h>
#include <boot_param.h>
#include <asm/numa.h>

unsigned char __node_distances[MAX_NUMNODES][MAX_NUMNODES];
EXPORT_SYMBOL(__node_distances);
struct pglist_data *node_data[MAX_NUMNODES];
EXPORT_SYMBOL(node_data);
int numa_off;
static struct numa_meminfo numa_meminfo;
cpumask_t cpus_on_node[MAX_NUMNODES];
static cpumask_t phys_cpus_on_node[MAX_NUMNODES];
EXPORT_SYMBOL(cpus_on_node);

/*
 * apicid, cpu, node mappings
 */
s16 __cpuid_to_node[CONFIG_NR_CPUS] = {
	[0 ... CONFIG_NR_CPUS - 1] = NUMA_NO_NODE
};
EXPORT_SYMBOL(__cpuid_to_node);

nodemask_t numa_nodes_parsed __initdata;

#ifdef CONFIG_HAVE_SETUP_PER_CPU_AREA
unsigned long __per_cpu_offset[NR_CPUS] __read_mostly;
EXPORT_SYMBOL(__per_cpu_offset);

static int __init pcpu_cpu_distance(unsigned int from, unsigned int to)
{
	if (early_cpu_to_node(from) == early_cpu_to_node(to))
		return LOCAL_DISTANCE;
	else
		return REMOTE_DISTANCE;
}

static void * __init pcpu_alloc_bootmem(unsigned int cpu, size_t size,
				       size_t align)
{
	const unsigned long goal = __pa(MAX_DMA_ADDRESS);
#ifdef CONFIG_NEED_MULTIPLE_NODES
	int node = early_cpu_to_node(cpu);
	void *ptr;

	if (!node_online(node) || !NODE_DATA(node)) {
		ptr = __alloc_bootmem(size, align, goal);
		pr_info("cpu %d has no node %d or node-local memory\n",
			cpu, node);
		pr_debug("per cpu data for cpu%d %lu bytes at %016lx\n",
			 cpu, size, __pa(ptr));
	} else {
		ptr = __alloc_bootmem_node(NODE_DATA(node),
						   size, align, goal);
		pr_debug("per cpu data for cpu%d %lu bytes on node%d at %016lx\n",
			 cpu, size, node, __pa(ptr));
	}
	return ptr;
#else
	return __alloc_bootmem(size, align, goal);
#endif
}

static void __init pcpu_free_bootmem(void *ptr, size_t size)
{
	free_bootmem(__pa(ptr), size);
}

static void __init pcpu_populate_pte(unsigned long addr)
{
	pgd_t *pgd = pgd_offset_k(addr);
	pud_t *pud;
	pmd_t *pmd;

	if (pgd_none(*pgd)) {
		pud_t *new;

		new = __alloc_bootmem(PAGE_SIZE, PAGE_SIZE, PAGE_SIZE);
		pgd_populate(&init_mm, pgd, new);
#ifndef __PAGETABLE_PUD_FOLDED
		pud_init((unsigned long)new, (unsigned long)invalid_pmd_table);
#endif
	}

	pud = pud_offset(pgd, addr);
	if (pud_none(*pud)) {
		pmd_t *new;

		new = __alloc_bootmem(PAGE_SIZE, PAGE_SIZE, PAGE_SIZE);
		pud_populate(&init_mm, pud, new);
#ifndef __PAGETABLE_PMD_FOLDED
		pmd_init((void *)new);
#endif
	}

	pmd = pmd_offset(pud, addr);
	if (!pmd_present(*pmd)) {
		pte_t *new;

		new = __alloc_bootmem(PAGE_SIZE, PAGE_SIZE, PAGE_SIZE);
		pmd_populate_kernel(&init_mm, pmd, new);
	}
}

void __init setup_per_cpu_areas(void)
{
	unsigned long delta;
	unsigned int cpu;
	int rc = -EINVAL;

	if (nr_node_ids >= 8) {
		pcpu_chosen_fc = PCPU_FC_PAGE;
	} else {
		pcpu_chosen_fc = PCPU_FC_EMBED;
	}
	/*
	 * Always reserve area for module percpu variables.  That's
	 * what the legacy allocator did.
	 */
	if (pcpu_chosen_fc != PCPU_FC_PAGE) {
		rc = pcpu_embed_first_chunk(PERCPU_MODULE_RESERVE,
					    PERCPU_DYNAMIC_RESERVE, PMD_SIZE,
					    pcpu_cpu_distance,
					    pcpu_alloc_bootmem,
					    pcpu_free_bootmem);
		if (rc)
			pr_warning("PERCPU: %s allocator failed (%d), "
				   "falling back to page size\n",
				   pcpu_fc_names[pcpu_chosen_fc], rc);
	}
	if (rc < 0)
		rc = pcpu_page_first_chunk(PERCPU_MODULE_RESERVE,
					   pcpu_alloc_bootmem,
					   pcpu_free_bootmem,
					   pcpu_populate_pte);
	if (rc < 0)
		panic("cannot initialize percpu area (err=%d)", rc);

	delta = (unsigned long)pcpu_base_addr - (unsigned long)__per_cpu_start;
	for_each_possible_cpu(cpu)
		__per_cpu_offset[cpu] = delta + pcpu_unit_offsets[cpu];
}
#endif

/*
 * Get nodeid by logical cpu number.
 * __cpuid_to_node maps phyical cpu id to node, so we
 * should use cpu_logical_map(cpu) to index it.
 *
 * This routine is only used in early phase during
 * booting, after setup_per_cpu_areas calling and numa_node
 * initialization, cpu_to_node will be used instead.
 * */
int early_cpu_to_node(int cpu)
{
	int apicid = cpu_logical_map(cpu);

	if (apicid != -1)
		return __cpuid_to_node[apicid];
	return NUMA_NO_NODE;
}

void __init early_numa_add_cpu(int cpuid, s16 node)
{
	int cpu = __cpu_number_map[cpuid];

	if (cpu != -1)
		cpumask_set_cpu(cpu, &cpus_on_node[node]);
	cpumask_set_cpu(cpuid, &phys_cpus_on_node[node]);
}

void numa_add_cpu(unsigned int cpu)
{
	int nid = cpu_to_node(cpu);
	cpumask_set_cpu(cpu, &cpus_on_node[nid]);
}

void numa_remove_cpu(unsigned int cpu)
{
	int nid = cpu_to_node(cpu);
	cpumask_clear_cpu(cpu, &cpus_on_node[nid]);
}

static int __init numa_add_memblk_to(int nid, u64 start, u64 end,
				     struct numa_meminfo *mi)
{
	/* ignore zero length blks */
	if (start == end)
		return 0;

	/* whine about and ignore invalid blks */
	if (start > end || nid < 0 || nid >= MAX_NUMNODES) {
		pr_warning("NUMA: Warning: invalid memblk node %d [mem %#010Lx-%#010Lx]\n",
			   nid, start, end - 1);
		return 0;
	}

	if (mi->nr_blks >= NR_NODE_MEMBLKS) {
		pr_err("NUMA: too many memblk ranges\n");
		return -EINVAL;
	}

	mi->blk[mi->nr_blks].start = PFN_ALIGN(start);
	mi->blk[mi->nr_blks].end = PFN_ALIGN(end - PAGE_SIZE + 1);
	mi->blk[mi->nr_blks].nid = nid;
	mi->nr_blks++;
	return 0;
}

/**
 * numa_add_memblk - Add one numa_memblk to numa_meminfo
 * @nid: NUMA node ID of the new memblk
 * @start: Start address of the new memblk
 * @end: End address of the new memblk
 *
 * Add a new memblk to the default numa_meminfo.
 *
 * RETURNS:
 * 0 on success, -errno on failure.
 */
int __init numa_add_memblk(int nid, u64 start, u64 end)
{
	return numa_add_memblk_to(nid, start, end, &numa_meminfo);
}

static void __init alloc_node_data(int nid)
{
	void *nd;
	unsigned long nd_pa;
	size_t nd_sz = roundup(sizeof(pg_data_t), PAGE_SIZE);

	nd_pa = memblock_alloc_nid(nd_sz, SMP_CACHE_BYTES, nid);
	if (!nd_pa) {
		nd_pa = __memblock_alloc_base(nd_sz, SMP_CACHE_BYTES,
				MEMBLOCK_ALLOC_ACCESSIBLE);
		if (!nd_pa) {
			pr_err("Cannot find %zu Byte for node_data %d \n", nd_sz, nid);
			return;
		}
	}

	nd = __va(nd_pa);

	node_data[nid] = nd;
	memset(nd, 0, sizeof(pg_data_t));
}

static void __init node_mem_init(unsigned int node)
{
	unsigned long node_addrspace_offset;
	unsigned long start_pfn, end_pfn;

	node_addrspace_offset = nid_to_addrbase(node);
	pr_info("Node%d's addrspace_offset is 0x%lx\n",
			node, node_addrspace_offset);

	get_pfn_range_for_nid(node, &start_pfn, &end_pfn);
	pr_info("Node%d: start_pfn=0x%lx, end_pfn=0x%lx\n",
		node, start_pfn, end_pfn);

	alloc_node_data(node);

	NODE_DATA(node)->node_start_pfn = start_pfn;
	NODE_DATA(node)->node_spanned_pages = end_pfn - start_pfn;

	if (node == 0) {
		/* kernel end address */
		unsigned long kernel_end_pfn = PFN_UP(__pa_symbol(&_end));

		/* used by finalize_initrd() */
		max_low_pfn = end_pfn;

		/* Reserve the kernel text/data/bss */
		memblock_reserve(start_pfn << PAGE_SHIFT,
				 ((kernel_end_pfn - start_pfn) << PAGE_SHIFT));

		if (node_end_pfn(0) >= (0xffffffff >> PAGE_SHIFT))
			memblock_reserve((node_addrspace_offset | 0xfe000000),
					 32 << 20);
	}
}

#ifdef CONFIG_ACPI_NUMA

/*
 * Sanity check to catch more bad NUMA configurations (they are amazingly
 * common).  Make sure the nodes cover all memory.
 */
static bool __init numa_meminfo_cover_memory(const struct numa_meminfo *mi)
{
	u64 numaram, biosram;
	int i;

	numaram = 0;
	for (i = 0; i < mi->nr_blks; i++) {
		u64 s = mi->blk[i].start >> PAGE_SHIFT;
		u64 e = mi->blk[i].end >> PAGE_SHIFT;
		numaram += e - s;
		numaram -= __absent_pages_in_range(mi->blk[i].nid, s, e);
		if ((s64)numaram < 0)
			numaram = 0;
	}
	max_pfn = max_low_pfn;
	biosram = max_pfn - absent_pages_in_range(0, max_pfn);

	BUG_ON((s64)(biosram - numaram) >= (1 << (20 - PAGE_SHIFT)));
	return true;
}

static void add_node_intersection(u32 node, u64 start, u64 size)
{
	static unsigned long num_physpages;

	num_physpages += (size >> PAGE_SHIFT);
	pr_debug("Debug: node_id:%d, mem_start:0x%llx, mem_size:0x%llx Bytes\n",
		node, start, size);
	pr_debug("       start_pfn:0x%llx, end_pfn:0x%llx, num_physpages:0x%lx\n",
		start >> PAGE_SHIFT, (start + size) >> PAGE_SHIFT, num_physpages);
	memblock_set_node(start, size, &memblock.memory, node);

}

/*
 * add_numamem_region
 *
 * Add a uasable memory region described by BIOS. The
 * routine gets each intersection between BIOS's region
 * and node's region, and adds them into node's memblock
 * pool.
 *
 */
static void __init add_numamem_region(u64 start, u64 end)
{
	u32 i;
	u64 ofs = start;

	if (start >= end) {
		pr_debug("Invalid region: %016llx-%016llx\n", start, end);
		return;
	}

	for (i = 0; i < numa_meminfo.nr_blks; i++) {
		struct numa_memblk *mb = &numa_meminfo.blk[i];

		if (ofs > mb->end)
			continue;

		if (end > mb->end) {
			add_node_intersection(mb->nid, ofs, mb->end - ofs);
			ofs = mb->end;
		} else {
			add_node_intersection(mb->nid, ofs, end - ofs);
			break;
		}
	}
}

static void __init init_node_memblock(void)
{
	u32 i, mem_type;
	u64 mem_end, mem_start, mem_size;
	efi_memory_desc_t *md;

	if (!g_mmap) {
		/* Parse memory information and activate */
		for_each_efi_memory_desc(md) {
			mem_type = md->type;
			mem_start = md->phys_addr;
			mem_size = md->num_pages << EFI_PAGE_SHIFT;
			mem_end = mem_start + mem_size;

			switch (mem_type) {
			case EFI_LOADER_CODE:
			case EFI_LOADER_DATA:
			case EFI_BOOT_SERVICES_CODE:
			case EFI_BOOT_SERVICES_DATA:
			case EFI_PERSISTENT_MEMORY:
			case EFI_CONVENTIONAL_MEMORY:
				add_numamem_region(mem_start, mem_end);
				break;
			case EFI_PAL_CODE:
			case EFI_UNUSABLE_MEMORY:
			case EFI_ACPI_RECLAIM_MEMORY:
			case EFI_RUNTIME_SERVICES_CODE:
			case EFI_RUNTIME_SERVICES_DATA:
				add_numamem_region(mem_start, mem_end);
			case EFI_RESERVED_TYPE:
			case EFI_MEMORY_MAPPED_IO:
			case EFI_MEMORY_MAPPED_IO_PORT_SPACE:
				pr_debug("Resvd: mem_type:%d, mem_start:0x%llx, mem_size:0x%llx Bytes\n",
						mem_type, mem_start, mem_size);
				break;
			}
		}
		return;
	}
	for (i = 0; i < g_mmap->map_count; i++) {
		mem_type = g_mmap->map[i].mem_type;
		mem_start = g_mmap->map[i].mem_start;
		mem_size = g_mmap->map[i].mem_size;
		mem_end = g_mmap->map[i].mem_start + mem_size;

		switch (mem_type) {
		case ADDRESS_TYPE_SYSRAM:
			mem_start = PFN_ALIGN(mem_start);
			mem_end = PFN_ALIGN(mem_end - PAGE_SIZE + 1);
			if (mem_start >= mem_end)
				break;
			add_numamem_region(mem_start, mem_end);
			break;

		case ADDRESS_TYPE_ACPI:
			printk("Debug: mem_type:%d, mem_start:0x%llx, mem_size:0x%llx Bytes\n",
					mem_type, mem_start, mem_size);
			mem_start = PFN_ALIGN(mem_start - PAGE_SIZE + 1);
			mem_end = PFN_ALIGN(mem_end);
			mem_size = mem_end - mem_start;
			memblock_add(mem_start, mem_size);
			memblock_mark_nomap(mem_start, mem_size);
			memblock_set_node(mem_start, mem_size,
					&memblock.memory, 0);
			memblock_reserve(mem_start, mem_size);
			break;

		case ADDRESS_TYPE_RESERVED:
			printk("Debug: mem_type:%d, mem_start:0x%llx, mem_size:0x%llx Bytes\n",
					mem_type, mem_start, mem_size);
			memblock_reserve(mem_start, mem_size);
			break;
		}
	}

}

static void __init numa_default_distance(void)
{
	int row, col;

	for (row = 0; row < MAX_NUMNODES; row++)
		for (col = 0; col < MAX_NUMNODES; col++) {
			if (col == row)
				__node_distances[row][col] = LOCAL_DISTANCE;
			else
				/* We assume that one node per package here!
				 *
				 * A SLIT should be used for multiple nodes per
				 * package to override default setting.
				 */
				__node_distances[row][col] = REMOTE_DISTANCE;
	}
}
static int __init numa_mem_init(int (*init_func)(void))
{
	int i;
	int ret;
	int node;

	for (i = 0; i < CONFIG_NR_CPUS; i++)
		set_cpuid_to_node(i, NUMA_NO_NODE);

	numa_default_distance();
	nodes_clear(numa_nodes_parsed);
	nodes_clear(node_possible_map);
	nodes_clear(node_online_map);
	memset(&numa_meminfo, 0, sizeof(numa_meminfo));
	/* Parse SRAT and SLIT if provided by firmware. */
	ret = init_func();
	if (ret < 0)
		return ret;
	node_possible_map = numa_nodes_parsed;
	if (WARN_ON(nodes_empty(node_possible_map)))
		return -EINVAL;
	init_node_memblock();
	if (numa_meminfo_cover_memory(&numa_meminfo) == false)
		return -EINVAL;

	for_each_node_mask(node, node_possible_map) {
		node_mem_init(node);
		node_set_online(node);
	}
	max_low_pfn = PHYS_PFN(memblock_end_of_DRAM());
	return 0;
}
#endif
void __init paging_init(void)
{
	unsigned node;
	unsigned long zones_size[MAX_NR_ZONES] = {0, };

	for_each_online_node(node) {
		unsigned long start_pfn, end_pfn;

		get_pfn_range_for_nid(node, &start_pfn, &end_pfn);

		if (end_pfn > max_low_pfn)
			max_low_pfn = end_pfn;
	}
#ifdef CONFIG_ZONE_DMA32
	zones_size[ZONE_DMA32] = MAX_DMA32_PFN;
#endif
	zones_size[ZONE_NORMAL] = max_low_pfn;
	free_area_init_nodes(zones_size);
}

void __init mem_init(void)
{
	high_memory = (void *) __va(get_num_physpages() << PAGE_SHIFT);
	free_all_bootmem();
	setup_zero_pages();	/* This comes from node 0 */
	mem_init_print_info(NULL);
}

int pcibus_to_node(struct pci_bus *bus)
{
	return dev_to_node(&bus->dev);
}
EXPORT_SYMBOL(pcibus_to_node);

static void cpu_node_probe(void)
{
	int i;

	nodes_clear(node_possible_map);
	nodes_clear(node_online_map);
	for (i = 0; i < loongson_sysconf.nr_nodes; i++) {
		node_set_state(num_online_nodes(), N_POSSIBLE);
		node_set_online(num_online_nodes());
	}

	pr_info("NUMA: Discovered %d cpus on %d nodes\n",
		loongson_sysconf.nr_cpus, num_online_nodes());
}

static void __init szmem(unsigned int node)
{
	u32 i, mem_type;
	u64 mem_end, mem_start, mem_size;

	if (!g_mmap)
		return;
	/* Parse memory information and activate */
	for (i = 0; i < g_mmap->map_count; i++) {
		mem_type = g_mmap->map[i].mem_type;
		mem_start = g_mmap->map[i].mem_start;
		mem_size = g_mmap->map[i].mem_size;
		mem_end = g_mmap->map[i].mem_start + mem_size;

		switch (mem_type) {
		case ADDRESS_TYPE_SYSRAM:
			pr_info("mem_type:%d, mem_start:0x%llx, mem_size:0x%llx B\n",
				mem_type, mem_start, mem_size);
			memblock_set_node(mem_start, mem_size, &memblock.memory, 0);
			break;
		case ADDRESS_TYPE_ACPI:
		case ADDRESS_TYPE_RESERVED:
			pr_info("mem_type:%d, mem_start:0x%llx, mem_size:0x%llx B\n",
				mem_type, mem_start, mem_size);
			memblock_reserve(mem_start, mem_size);
			break;
		}
	}
}

static __init void prom_meminit(void)
{
	unsigned int node, cpu, active_cpu = 0;

	cpu_node_probe();
	numa_default_distance();

	for (node = 0; node < loongson_sysconf.nr_nodes; node++) {
		if (node_online(node)) {
			szmem(node);
			node_mem_init(node);
			cpumask_clear(&cpus_on_node[node]);
		}
	}
	for (cpu = 0; cpu < loongson_sysconf.nr_cpus; cpu++) {
		node = cpu / loongson_sysconf.cores_per_node;
		if (node >= num_online_nodes())
			node = 0;

		set_cpuid_to_node(cpu, node);

		if (loongson_sysconf.reserved_cpus_mask & (1<<cpu))
			continue;

		cpumask_set_cpu(active_cpu, &cpus_on_node[node]);
		pr_info("NUMA: set cpumask cpu %d on node %d\n", active_cpu, node);

		active_cpu++;
	}
}

void __init fw_init_numa_memory(void)
{
#ifndef CONFIG_ACPI_NUMA
	prom_meminit();
#else
	if (acpi_disabled) {
		prom_meminit();
	} else {
		numa_mem_init(acpi_numa_init);
		setup_nr_node_ids();
		loongson_sysconf.nr_nodes = nr_node_ids;
		loongson_sysconf.cores_per_node = cpumask_weight(&phys_cpus_on_node[0]);
	}
#endif
}
EXPORT_SYMBOL(fw_init_numa_memory);
