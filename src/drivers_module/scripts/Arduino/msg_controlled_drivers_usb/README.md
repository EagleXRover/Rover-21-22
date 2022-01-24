# **Value control equivalents**
byte = bit[ base_1, base_0, upper_1, upper_0, 
            fore_1, fore_0, wrist_1, wrist_0 ];

- base [2]
- uper [2]
- fore [2]
- wris [2]

values:
- 00 - do nothing
- 01 - do nothing
- 10 - reverse
- 11 - forward

arm_topic/

slave_ip

- #define arm_linear          0x80 
- #define arm_rotational      0x81
- #define wrist_rotational    0x82
- #define zero_speed          0x40
- #define linear_speed        0x20
- #define rot_speed           0x10

&nbsp;

- 0x80 M1 = base brazo ext
- 0x80 M2 = frente brazo ext
- 0x81 M2 = base rot
- 0x82 M1 = muñeca rot

