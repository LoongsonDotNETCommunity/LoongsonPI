#ifndef __ASM_OFFSETS_H__
#define __ASM_OFFSETS_H__
/*
 * DO NOT MODIFY.
 *
 * This file was generated by Kbuild
 */

/* LoongArch pt_regs offsets. */
#define PT_R0 0 /* offsetof(struct pt_regs, regs[0]) */
#define PT_R1 8 /* offsetof(struct pt_regs, regs[1]) */
#define PT_R2 16 /* offsetof(struct pt_regs, regs[2]) */
#define PT_R3 24 /* offsetof(struct pt_regs, regs[3]) */
#define PT_R4 32 /* offsetof(struct pt_regs, regs[4]) */
#define PT_R5 40 /* offsetof(struct pt_regs, regs[5]) */
#define PT_R6 48 /* offsetof(struct pt_regs, regs[6]) */
#define PT_R7 56 /* offsetof(struct pt_regs, regs[7]) */
#define PT_R8 64 /* offsetof(struct pt_regs, regs[8]) */
#define PT_R9 72 /* offsetof(struct pt_regs, regs[9]) */
#define PT_R10 80 /* offsetof(struct pt_regs, regs[10]) */
#define PT_R11 88 /* offsetof(struct pt_regs, regs[11]) */
#define PT_R12 96 /* offsetof(struct pt_regs, regs[12]) */
#define PT_R13 104 /* offsetof(struct pt_regs, regs[13]) */
#define PT_R14 112 /* offsetof(struct pt_regs, regs[14]) */
#define PT_R15 120 /* offsetof(struct pt_regs, regs[15]) */
#define PT_R16 128 /* offsetof(struct pt_regs, regs[16]) */
#define PT_R17 136 /* offsetof(struct pt_regs, regs[17]) */
#define PT_R18 144 /* offsetof(struct pt_regs, regs[18]) */
#define PT_R19 152 /* offsetof(struct pt_regs, regs[19]) */
#define PT_R20 160 /* offsetof(struct pt_regs, regs[20]) */
#define PT_R21 168 /* offsetof(struct pt_regs, regs[21]) */
#define PT_R22 176 /* offsetof(struct pt_regs, regs[22]) */
#define PT_R23 184 /* offsetof(struct pt_regs, regs[23]) */
#define PT_R24 192 /* offsetof(struct pt_regs, regs[24]) */
#define PT_R25 200 /* offsetof(struct pt_regs, regs[25]) */
#define PT_R26 208 /* offsetof(struct pt_regs, regs[26]) */
#define PT_R27 216 /* offsetof(struct pt_regs, regs[27]) */
#define PT_R28 224 /* offsetof(struct pt_regs, regs[28]) */
#define PT_R29 232 /* offsetof(struct pt_regs, regs[29]) */
#define PT_R30 240 /* offsetof(struct pt_regs, regs[30]) */
#define PT_R31 248 /* offsetof(struct pt_regs, regs[31]) */
#define PT_CRMD 272 /* offsetof(struct pt_regs, csr_crmd) */
#define PT_PRMD 280 /* offsetof(struct pt_regs, csr_prmd) */
#define PT_EUEN 288 /* offsetof(struct pt_regs, csr_euen) */
#define PT_ECFG 296 /* offsetof(struct pt_regs, csr_ecfg) */
#define PT_ESTAT 304 /* offsetof(struct pt_regs, csr_estat) */
#define PT_ERA 256 /* offsetof(struct pt_regs, csr_era) */
#define PT_BVADDR 264 /* offsetof(struct pt_regs, csr_badv) */
#define PT_ORIG_A0 312 /* offsetof(struct pt_regs, orig_a0) */
#define PT_SIZE 320 /* sizeof(struct pt_regs) */

/* LoongArch task_struct offsets. */
#define TASK_STATE 0 /* offsetof(struct task_struct, state) */
#define TASK_THREAD_INFO 8 /* offsetof(struct task_struct, stack) */
#define TASK_FLAGS 20 /* offsetof(struct task_struct, flags) */
#define TASK_MM 680 /* offsetof(struct task_struct, mm) */
#define TASK_PID 832 /* offsetof(struct task_struct, pid) */
#define TASK_STRUCT_SIZE 3584 /* sizeof(struct task_struct) */

/* LoongArch thread_info offsets. */
#define TI_TASK 0 /* offsetof(struct thread_info, task) */
#define TI_FLAGS 8 /* offsetof(struct thread_info, flags) */
#define TI_TP_VALUE 16 /* offsetof(struct thread_info, tp_value) */
#define TI_CPU 24 /* offsetof(struct thread_info, cpu) */
#define TI_PRE_COUNT 28 /* offsetof(struct thread_info, preempt_count) */
#define TI_ADDR_LIMIT 32 /* offsetof(struct thread_info, addr_limit) */
#define TI_REGS 40 /* offsetof(struct thread_info, regs) */
#define _THREAD_SIZE 16384 /* THREAD_SIZE */
#define _THREAD_MASK 16383 /* THREAD_MASK */
#define _IRQ_STACK_SIZE 16384 /* IRQ_STACK_SIZE */
#define _IRQ_STACK_START 16368 /* IRQ_STACK_START */

/* LoongArch specific thread_struct offsets. */
#define THREAD_REG01 1952 /* offsetof(struct task_struct, thread.reg01) */
#define THREAD_REG03 1960 /* offsetof(struct task_struct, thread.reg03) */
#define THREAD_REG22 1968 /* offsetof(struct task_struct, thread.reg22) */
#define THREAD_REG23 1976 /* offsetof(struct task_struct, thread.reg23) */
#define THREAD_REG24 1984 /* offsetof(struct task_struct, thread.reg24) */
#define THREAD_REG25 1992 /* offsetof(struct task_struct, thread.reg25) */
#define THREAD_REG26 2000 /* offsetof(struct task_struct, thread.reg26) */
#define THREAD_REG27 2008 /* offsetof(struct task_struct, thread.reg27) */
#define THREAD_REG28 2016 /* offsetof(struct task_struct, thread.reg28) */
#define THREAD_REG29 2024 /* offsetof(struct task_struct, thread.reg29) */
#define THREAD_REG30 2032 /* offsetof(struct task_struct, thread.reg30) */
#define THREAD_REG31 2040 /* offsetof(struct task_struct, thread.reg31) */
#define THREAD_SCHED_RA 2048 /* offsetof(struct task_struct, thread.sched_ra) */
#define THREAD_SCHED_CFA 2056 /* offsetof(struct task_struct, thread.sched_cfa) */
#define THREAD_CSRCRMD 2072 /* offsetof(struct task_struct, thread.csr_crmd) */
#define THREAD_CSRPRMD 2064 /* offsetof(struct task_struct, thread.csr_prmd) */
#define THREAD_CSREUEN 2080 /* offsetof(struct task_struct, thread.csr_euen) */
#define THREAD_CSRECFG 2088 /* offsetof(struct task_struct, thread.csr_ecfg) */
#define THREAD_FPU 2496 /* offsetof(struct task_struct, thread.fpu) */
#define THREAD_BVADDR 2096 /* offsetof(struct task_struct, thread.csr_badv) */
#define THREAD_ECODE 2456 /* offsetof(struct task_struct, thread.error_code) */
#define THREAD_TRAPNO 2464 /* offsetof(struct task_struct, thread.trap_nr) */

#define THREAD_SCR0 0 /* offsetof(struct loongarch_lbt, scr0) */
#define THREAD_SCR1 8 /* offsetof(struct loongarch_lbt, scr1) */
#define THREAD_SCR2 16 /* offsetof(struct loongarch_lbt, scr2) */
#define THREAD_SCR3 24 /* offsetof(struct loongarch_lbt, scr3) */
#define THREAD_EFLAG 32 /* offsetof(struct loongarch_lbt, eflag) */

#define THREAD_FPR0 24 /* offsetof(struct loongarch_fpu, fpr[0]) */
#define THREAD_FPR1 56 /* offsetof(struct loongarch_fpu, fpr[1]) */
#define THREAD_FPR2 88 /* offsetof(struct loongarch_fpu, fpr[2]) */
#define THREAD_FPR3 120 /* offsetof(struct loongarch_fpu, fpr[3]) */
#define THREAD_FPR4 152 /* offsetof(struct loongarch_fpu, fpr[4]) */
#define THREAD_FPR5 184 /* offsetof(struct loongarch_fpu, fpr[5]) */
#define THREAD_FPR6 216 /* offsetof(struct loongarch_fpu, fpr[6]) */
#define THREAD_FPR7 248 /* offsetof(struct loongarch_fpu, fpr[7]) */
#define THREAD_FPR8 280 /* offsetof(struct loongarch_fpu, fpr[8]) */
#define THREAD_FPR9 312 /* offsetof(struct loongarch_fpu, fpr[9]) */
#define THREAD_FPR10 344 /* offsetof(struct loongarch_fpu, fpr[10]) */
#define THREAD_FPR11 376 /* offsetof(struct loongarch_fpu, fpr[11]) */
#define THREAD_FPR12 408 /* offsetof(struct loongarch_fpu, fpr[12]) */
#define THREAD_FPR13 440 /* offsetof(struct loongarch_fpu, fpr[13]) */
#define THREAD_FPR14 472 /* offsetof(struct loongarch_fpu, fpr[14]) */
#define THREAD_FPR15 504 /* offsetof(struct loongarch_fpu, fpr[15]) */
#define THREAD_FPR16 536 /* offsetof(struct loongarch_fpu, fpr[16]) */
#define THREAD_FPR17 568 /* offsetof(struct loongarch_fpu, fpr[17]) */
#define THREAD_FPR18 600 /* offsetof(struct loongarch_fpu, fpr[18]) */
#define THREAD_FPR19 632 /* offsetof(struct loongarch_fpu, fpr[19]) */
#define THREAD_FPR20 664 /* offsetof(struct loongarch_fpu, fpr[20]) */
#define THREAD_FPR21 696 /* offsetof(struct loongarch_fpu, fpr[21]) */
#define THREAD_FPR22 728 /* offsetof(struct loongarch_fpu, fpr[22]) */
#define THREAD_FPR23 760 /* offsetof(struct loongarch_fpu, fpr[23]) */
#define THREAD_FPR24 792 /* offsetof(struct loongarch_fpu, fpr[24]) */
#define THREAD_FPR25 824 /* offsetof(struct loongarch_fpu, fpr[25]) */
#define THREAD_FPR26 856 /* offsetof(struct loongarch_fpu, fpr[26]) */
#define THREAD_FPR27 888 /* offsetof(struct loongarch_fpu, fpr[27]) */
#define THREAD_FPR28 920 /* offsetof(struct loongarch_fpu, fpr[28]) */
#define THREAD_FPR29 952 /* offsetof(struct loongarch_fpu, fpr[29]) */
#define THREAD_FPR30 984 /* offsetof(struct loongarch_fpu, fpr[30]) */
#define THREAD_FPR31 1016 /* offsetof(struct loongarch_fpu, fpr[31]) */
#define THREAD_FCSR 0 /* offsetof(struct loongarch_fpu, fcsr) */
#define THREAD_FCC 8 /* offsetof(struct loongarch_fpu, fcc) */
#define THREAD_FTOP 16 /* offsetof(struct loongarch_fpu, ftop) */

/* Size of struct page */
#define STRUCT_PAGE_SIZE 56 /* sizeof(struct page) */

/* Linux mm_struct offsets. */
#define MM_USERS 72 /* offsetof(struct mm_struct, mm_users) */
#define MM_PGD 64 /* offsetof(struct mm_struct, pgd) */
#define MM_CONTEXT 712 /* offsetof(struct mm_struct, context) */

#define _PGD_T_SIZE 8 /* sizeof(pgd_t) */
#define _PMD_T_SIZE 8 /* sizeof(pmd_t) */
#define _PTE_T_SIZE 8 /* sizeof(pte_t) */

#define _PGD_T_LOG2 3 /* PGD_T_LOG2 */
#define _PMD_T_LOG2 3 /* PMD_T_LOG2 */
#define _PTE_T_LOG2 3 /* PTE_T_LOG2 */

#define _PGD_ORDER 0 /* PGD_ORDER */
#define _PMD_ORDER 0 /* PMD_ORDER */
#define _PTE_ORDER 0 /* PTE_ORDER */

#define _PMD_SHIFT 25 /* PMD_SHIFT */
#define _PGDIR_SHIFT 36 /* PGDIR_SHIFT */

#define _PTRS_PER_PGD 2048 /* PTRS_PER_PGD */
#define _PTRS_PER_PMD 2048 /* PTRS_PER_PMD */
#define _PTRS_PER_PTE 2048 /* PTRS_PER_PTE */

#define _PAGE_SHIFT 14 /* PAGE_SHIFT */
#define _PAGE_SIZE 16384 /* PAGE_SIZE */

/* Linux sigcontext offsets. */
#define SC_REGS 8 /* offsetof(struct sigcontext, sc_regs) */
#define SC_PC 0 /* offsetof(struct sigcontext, sc_pc) */
#define SC_FPC_CSR 268 /* offsetof(struct sigcontext, sc_fcsr) */

/* Linux signal numbers. */
#define _SIGHUP 1 /* SIGHUP */
#define _SIGINT 2 /* SIGINT */
#define _SIGQUIT 3 /* SIGQUIT */
#define _SIGILL 4 /* SIGILL */
#define _SIGTRAP 5 /* SIGTRAP */
#define _SIGIOT 6 /* SIGIOT */
#define _SIGABRT 6 /* SIGABRT */
#define _SIGFPE 8 /* SIGFPE */
#define _SIGKILL 9 /* SIGKILL */
#define _SIGBUS 7 /* SIGBUS */
#define _SIGSEGV 11 /* SIGSEGV */
#define _SIGSYS 31 /* SIGSYS */
#define _SIGPIPE 13 /* SIGPIPE */
#define _SIGALRM 14 /* SIGALRM */
#define _SIGTERM 15 /* SIGTERM */
#define _SIGUSR1 10 /* SIGUSR1 */
#define _SIGUSR2 12 /* SIGUSR2 */
#define _SIGCHLD 17 /* SIGCHLD */
#define _SIGPWR 30 /* SIGPWR */
#define _SIGWINCH 28 /* SIGWINCH */
#define _SIGURG 23 /* SIGURG */
#define _SIGIO 29 /* SIGIO */
#define _SIGSTOP 19 /* SIGSTOP */
#define _SIGTSTP 20 /* SIGTSTP */
#define _SIGCONT 18 /* SIGCONT */
#define _SIGTTIN 21 /* SIGTTIN */
#define _SIGTTOU 22 /* SIGTTOU */
#define _SIGVTALRM 26 /* SIGVTALRM */
#define _SIGPROF 27 /* SIGPROF */
#define _SIGXCPU 24 /* SIGXCPU */
#define _SIGXFSZ 25 /* SIGXFSZ */

/* Linux smp cpu boot offsets. */
#define CPU_BOOT_STACK 0 /* offsetof(struct secondary_data, stack) */
#define CPU_BOOT_TINFO 8 /* offsetof(struct secondary_data, thread_info) */

/*  Linux struct pbe offsets.  */
#define PBE_ADDRESS 0 /* offsetof(struct pbe, address) */
#define PBE_ORIG_ADDRESS 8 /* offsetof(struct pbe, orig_address) */
#define PBE_NEXT 16 /* offsetof(struct pbe, next) */
#define PBE_SIZE 24 /* sizeof(struct pbe) */

/*  PM offsets.  */
#define SSS_SP 0 /* offsetof(struct loongarch_static_suspend_state, sp) */

/*  KVM/LOONGISA Specific offsets.  */
#define VCPU_FCSR0 384 /* offsetof(struct kvm_vcpu_arch, fpu.fcsr) */
#define VCPU_FCC 392 /* offsetof(struct kvm_vcpu_arch, fpu.fcc) */

#define KVM_VCPU_ARCH 896 /* offsetof(struct kvm_vcpu, arch) */
#define KVM_VCPU_KVM 0 /* offsetof(struct kvm_vcpu, kvm) */
#define KVM_VCPU_RUN 96 /* offsetof(struct kvm_vcpu, run) */

#define KVM_ARCH_HSTACK 32 /* offsetof(struct kvm_vcpu_arch, host_stack) */
#define KVM_ARCH_HGP 40 /* offsetof(struct kvm_vcpu_arch, host_gp) */
#define KVM_ARCH_HANDLE_EXIT 24 /* offsetof(struct kvm_vcpu_arch, handle_exit) */
#define KVM_ARCH_HPGD 48 /* offsetof(struct kvm_vcpu_arch, host_pgd) */
#define KVM_ARCH_GEENTRY 0 /* offsetof(struct kvm_vcpu_arch, guest_eentry) */
#define KVM_ARCH_GPC 376 /* offsetof(struct kvm_vcpu_arch, pc) */
#define KVM_ARCH_GGPR 120 /* offsetof(struct kvm_vcpu_arch, gprs) */
#define KVM_ARCH_HESTAT 80 /* offsetof(struct kvm_vcpu_arch, host_estat) */
#define KVM_ARCH_HBADV 72 /* offsetof(struct kvm_vcpu_arch, badv) */
#define KVM_ARCH_HBADI 88 /* offsetof(struct kvm_vcpu_arch, badi) */
#define KVM_ARCH_ISHYPCALL 112 /* offsetof(struct kvm_vcpu_arch, is_hypcall) */
#define KVM_ARCH_HECFG 96 /* offsetof(struct kvm_vcpu_arch, host_ecfg) */
#define KVM_ARCH_HEENTRY 8 /* offsetof(struct kvm_vcpu_arch, host_eentry) */
#define KVM_ARCH_HPERCPU 104 /* offsetof(struct kvm_vcpu_arch, host_percpu) */
#define KVM_GPGD 2376 /* offsetof(struct kvm, arch.gpa_mm.pgd) */


#endif
