#include <linux/init.h>
#include <linux/module.h>

#include "../../../arch/arm/mach-msm/proc_comm.h"
#include <oem/boot_mode.h>

MODULE_LICENSE("Dual BSD/GPL");

static int reset_init(void)
{
    uint32_t restart_reason;
    extern void backup_restart_reason(int);

    restart_reason = KERNEL_REBOOT_FOR_CPO;
    backup_restart_reason(KERNEL_REBOOT_FOR_CPO);
    msm_proc_comm(PCOM_POWER_DOWN, &restart_reason, 0);
    for(;;);
    printk("reset chip via PCOM_RESET_CHIP, restart_reason=0x%x\n", restart_reason);
    return 0;
}

static void reset_exit(void)
{
    printk(KERN_INFO "Goodbye, reset chip\n");
}

module_init(reset_init);
module_exit(reset_exit);
