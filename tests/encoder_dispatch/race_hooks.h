/*
 *	race_hooks.h
 *	Race build only (forced in with -include): DI/EI that the race test
 *	can see, so a preemption cannot happen while interrupts are masked.
 */

#ifndef RACE_HOOKS_H
#define RACE_HOOKS_H

void race_di(void);
void race_ei(void);

#define DI(s)	((s) = 0u, race_di())
#define EI(s)	((void)(s), race_ei())

#endif /* RACE_HOOKS_H */
