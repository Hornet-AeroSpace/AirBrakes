.section .rodata
DRYAIRCONSTANT: .float 287.058
AIRMOL: .float 0.0289652
UNIGAS: .float 8.31447
LAPSERATE: .float 0.0065
SEALEVEL: .float 101325
GRAV: .float -9.80665
EARTHRAD: .float 6371008
PI: .float 3.141593
TIMESTEP: .float 0.16
ONE: .float 1.0


.section .bss
vel: .zero 64 #0-15 start, 16-31 v1, 32-47 v2, 48-63 v3
height: .zero 64 #0-15 start, 16-31 h1, 32-47 h2, 48-63 h3
acc: .zero 16
airres: .zero 16


.section .text
.global simulate_apogee_vector
.align 2

simulate_apogee_vector:
    csrwi 0x7F2, 0x3

    la t1, vel #storing velocity data in the vel array
    flw ft0, 0(a0)
    fsw ft0, 0(t1)
    flw ft0, 4(a0)
    fsw ft0, 4(t1)
    flw ft0, 8(a0)
    fsw ft0, 8(t1)
    flw ft0, 12(a0)
    fsw ft0, 12(t1)

    la t1, height #storing height data into the height array
    flw ft0, 0(a1)
    fsw ft0, 0(t1)
    flw ft0, 4(a1)
    fsw ft0, 4(t1)
    flw ft0, 8(a1)
    fsw ft0, 8(t1)
    flw ft0, 12(a1)
    fsw ft0, 12(t1)

    esp.xorq q2, q2, q2 #make 128 bit register of zeroes
    esp.vst.128.ip q2, a3, 0 #store that in timestep so they all start at 0

loop:    
    la t3, vel
    esp.vld.128.ip q0, t3, 0
    la t3, height
    esp.vld.128.ip q4, t3, 0 #load height into vector

    



end:
    la t1, ONE
    flw fa0, 0(t1)
    ret