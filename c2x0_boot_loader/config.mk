# Debug print works only if UART is enabled.
# In particular, the board's SW8[8] must be in the OFF position to activate
# UART and other components. Normally, PKC runs with SW8[8]=ON and debug print
# must be disabled
PRINT_DEBUG=n
PRINT_ERROR=n

#Specifies type of EP
C293_EP=y

#Specifies whether firmware is running high performance mode
HIGH_PERF_MODE=y
