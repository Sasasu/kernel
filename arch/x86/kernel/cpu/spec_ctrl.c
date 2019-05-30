/*
 * Speculation control stuff
 *
 */

#include <asm/cpufeature.h>
#include <asm/msr.h>
#include <asm/processor.h>
#include <asm/spec_ctrl.h>
#include <asm/cpufeature.h>

/*
 * Keep it open for more flags in case needed.
 *
 * -1 means "not touched by nospec() earlyparam"
 *
 * If IBRS is set, IBPB is always set. IBPB can be set independently
 * on IBRS state (SKL).
 */
int ibrs_state = -1;
int _ibpb_state = -1;

unsigned int notrace x86_ibrs_enabled(void)
{
	if (ibrs_state != 1)
		return 0;
	return 1;
}
EXPORT_SYMBOL_GPL(x86_ibrs_enabled);

unsigned int notrace x86_ibpb_enabled(void)
{
	if (_ibpb_state != 1)
		return 0;
	return 1;
}
EXPORT_SYMBOL_GPL(x86_ibpb_enabled);

void x86_disable_ibrs(void)
{
	if (x86_ibrs_enabled())
		native_wrmsrl(MSR_IA32_SPEC_CTRL, 0);
}
EXPORT_SYMBOL_GPL(x86_disable_ibrs);

void x86_enable_ibrs(void)
{
	if (x86_ibrs_enabled())
		native_wrmsrl(MSR_IA32_SPEC_CTRL, SPEC_CTRL_IBRS);
}
EXPORT_SYMBOL_GPL(x86_enable_ibrs);

/*
 * Called after upgrading microcode, check CPUID directly.
 */
void x86_spec_check(void)
{

	if (_ibpb_state == 0) {
		printk_once(KERN_INFO "IBRS/IBPB: disabled\n");
		return;
	}

	if (cpuid_edx(7) & BIT(26)) {
		if (ibrs_state == -1) {
			/* noone force-disabled IBRS */
			ibrs_state = 1;
			printk_once(KERN_INFO "IBRS: initialized\n");
		}
		printk_once(KERN_INFO "IBPB: initialized\n");
		_ibpb_state = 1;

		setup_force_cpu_cap(X86_FEATURE_SPEC_CTRL);
		setup_force_cpu_cap(X86_FEATURE_MSR_SPEC_CTRL);
	}

	if (boot_cpu_data.x86_vendor == X86_VENDOR_AMD) {
		if (cpuid_ebx(0x80000008) & BIT(12)) {
			_ibpb_state = 1;
			printk_once(KERN_INFO "IBPB: Initialized\n");
		} else {
			_ibpb_state = 0;
		}
	}
}
EXPORT_SYMBOL_GPL(x86_spec_check);

int nospec(char *str)
{
	/*
	 * Due to way how apply_forced_caps() works, we have to
	 * explicitly clear the flag here from cas_set, otherwise it'll be
	 * kept being put into the global mask.
	 */
	setup_clear_cpu_cap(X86_FEATURE_SPEC_CTRL);
	clear_bit(X86_FEATURE_SPEC_CTRL, (unsigned long *)cpu_caps_set);
	ibrs_state = 0;
	_ibpb_state = 0;

	return 0;
}
early_param("nospec", nospec);

static int __init nospectre_v2(char *str)
{
	return nospec(str);
}
early_param("nospectre_v2", nospectre_v2);

static int __init spectre_v2(char *str)
{
	if (str && !strcmp(str, "off"))
		return nospec(str);

	return 0;
}
early_param("spectre_v2", spectre_v2);

