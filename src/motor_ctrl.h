#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

void mcSetup(void);
void mcSetDutyCycle(unsigned char channel, float duty_cycle);

extern int pidInput;
#endif // __MOTOR_CTRL_H
