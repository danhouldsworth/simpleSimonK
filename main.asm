
.cseg
.org 	0x0000
start:
        sbi     0x14, 3
        cbi     0x15, 3

        sbi     0x14, 2
        cbi     0x15, 2

        rjmp 	start