#include <sys/cdefs.h>
#include <sys/systm.h>
#include <sys/thread.h>
#include <sys/spinlock.h>

#include <lib/cyw43-driver/src/cyw43.h>
#include <lib/cyw43-driver/src/cyw43_country.h>

#include <arm/raspberrypi/rp2040.h>

#include <app/callout_test/callout_test.h>

static struct spinlock l;

static void
test_thr(void *arg)
{
	int n;

	n = (int)arg;

	while (1) {
		critical_enter();
		sl_lock(&l);
		printf("cpu%d: %s%d\n", PCPU_GET(cpuid), __func__, n);
		sl_unlock(&l);
		critical_exit();
		mdx_usleep(10000 + 100000 * n);
	}
}

static uint32_t
cyw43_arch_get_country_code(void)
{

	return (CYW43_COUNTRY_WORLDWIDE);
}

int
main(void)
{
	struct thread *td;

	sl_init(&l);

#if 0
	callout_test();
#endif

	printf("%s: initializing cyw32\n", __func__);
	cyw43_init(&cyw43_state);
	printf("%s: cyw43_init returned\n", __func__);

	cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true,
	    cyw43_arch_get_country_code());

	printf("%s: cyw43_wifi_set_up returned\n", __func__);

	while (1)
		mdx_usleep(50000);

	td = mdx_thread_create("test", 1, 1000, 4096, test_thr, (void *)0);
	if (td)
		mdx_sched_add(td);
	td = mdx_thread_create("test1", 1, 2000, 4096, test_thr, (void *)1);
	if (td)
		mdx_sched_add(td);
	td = mdx_thread_create("test1", 1, 3000, 4096, test_thr, (void *)2);
	if (td)
		mdx_sched_add(td);

	while (1) {
		critical_enter();
		sl_lock(&l);
		printf("cpu%d: Hello world\n", PCPU_GET(cpuid));
		sl_unlock(&l);
		critical_exit();
		mdx_usleep(50000);
	}

	return (0);
}
