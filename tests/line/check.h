/* Shared check macro for the tests/line suites. */
#ifndef TESTS_LINE_CHECK_H
#define TESTS_LINE_CHECK_H

#include <stdio.h>
#include <stdlib.h>

static unsigned check_count;
static unsigned check_failures;

#define CHECK(cond) do { \
	check_count++; \
	if (!(cond)) { \
		check_failures++; \
		(void)fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
	} \
} while (0)

static int check_report(const char *suite)
{
	(void)printf("%s: %u checks, %u failures\n", suite, check_count,
		     check_failures);
	return (check_failures == 0u) ? EXIT_SUCCESS : EXIT_FAILURE;
}

#endif
