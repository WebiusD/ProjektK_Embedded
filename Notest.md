Für auto-tuning procedure:
- set irun in the range of 8-31 (e.g. 16)
- enable driver
- if standstill reduction enabled, issue a single step
- wait for min. 130 ms
- move at least 400 steps (corresponds to a max. change of pwm_grad_auto of 50).
- store to PWM_GRAD_AUTO
- restore irun and ihold values.


- TPWMTHRS is the velocity threshold for stealth-chop, if TPWMTHRS is exceeded the operating mode changes to
  spread-cycle. Read out TSTEP when moving at the desired velocity and program the resulting value to TPWMTHRS. When the transition velocity is too high, this will result in a yerk (choose TPWMTHRS < 30 RPM)
  TPWMTHRS is reciproc proportional to motor velocity, since it corresponds to the time between two steps. So stealth chop is disabled if TSTEP (the time between two steps is faster than what is set by TPWMTHRS) falls below TPWMTHRS.