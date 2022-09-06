//// 100 ms in dw ticks
//#define DELAY_TX 0x17CDC00

//// 5 ms in dw ticks
//#define DELAY_TX 0x130B00

//// 10 ms in dw ticks
//#define DELAY_TX 0x261600

//// 20 ms in dw ticks
//#define DELAY_TX 0x4C2C00

// X ms in dw ticks
#define DELAY_TX_MS(X) (X*0x3CF00)

// X (100 us) in dw ticks
#define DELAY_TX_100US(X) (X*0x6180)

// X (10 us) in dw ticks
#define DELAY_TX_10US(X) (X*0x9C0)

// X (5 us) in dw ticks
#define DELAY_TX_5US(X) (X*0x4E0)

// X us in dw ticks with greatest lower bound estimate (0.9975961538461538 us) -> -2.4038461538461145 ns error
#define DELAY_TX_1US_LOWER(X) (X*0xF9)

// X us in dw ticks with lowest upper bound estimate (1.001602564102564 us) -> 1.602564102564147 ns error
#define DELAY_TX_1US_UPPER(X) (X*0xFA)

// X us in dw ticks with decawave's estimate (1.0256410256410257 us) -> 25.641025641025716 ns error
#define DELAY_TX_1US_DW(X) (X*0x100)

//#define DELAY_TX DELAY_TX_100US(25)
#define DELAY_TX DELAY_TX_100US(27)
#define DELAY_TX_IPI DELAY_TX_100US(3)
