#ifndef YETANOTHERCONTROLLERENABLER_PAD_ASSIGN_H
#define YETANOTHERCONTROLLERENABLER_PAD_ASSIGN_H

/* Ask the system shell to create the virtual controller and assign it.
 * Returns a write handle above 0, or a negative error. */
int pad_assign_from_shell(void);
int pad_assign_device_handle(void);
int pad_assign_release(void);
const char *pad_assign_status(void);

#endif
