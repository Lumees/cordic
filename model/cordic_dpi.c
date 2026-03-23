/*
 * CORDIC DPI-C Golden Model
 * =========================
 * Fixed-point CORDIC implementation in C.
 * Called from SystemVerilog UVM scoreboard via DPI-C.
 *
 * Matches RTL: DATA_WIDTH=16, STAGES=16, Q2.14 (SCALE=16384)
 * Angle format: π ↔ 16384
 */

#include <stdint.h>
#include <math.h>
#include <stdlib.h>

#define DATA_WIDTH  16
#define STAGES      16
#define FRAC_BITS   14
#define SCALE       (1 << FRAC_BITS)   /* 16384 */
#define INT16_MASK  0xFFFF
#define INT16_SIGN  0x8000

/* Coordinate and mode enums (match SV package) */
#define COORD_CIRC  0
#define COORD_LINR  1
#define COORD_HYPR  2
#define MODE_ROT    0
#define MODE_VEC    1

/* ────────────────────────────────────────────
 * Fixed-point helpers
 * ──────────────────────────────────────────── */
static inline int32_t to_signed16(int32_t v) {
    v &= INT16_MASK;
    if (v & INT16_SIGN) v |= 0xFFFF0000;
    return v;
}

static inline int32_t arith_shr(int32_t v, int n) {
    return (n >= 32) ? (v < 0 ? -1 : 0) : (v >> n);
}

/* ────────────────────────────────────────────
 * LUT tables (computed once at startup)
 * ──────────────────────────────────────────── */
static int32_t atan_lut[STAGES];
static int32_t atanh_lut[STAGES];
static int32_t linr_lut[STAGES];
static int      hypr_sched[STAGES] = {1,2,3,4,4,5,6,7,8,9,10,11,12,13,13,14};
static int      luts_initialized = 0;

static void init_luts(void) {
    if (luts_initialized) return;
    for (int i = 0; i < STAGES; i++) {
        double ang_circ  = atan(pow(2.0, -i));
        double ang_hypr  = (hypr_sched[i] <= 14) ? atanh(pow(2.0, -(double)hypr_sched[i])) : 0.0;
        double ang_linr  = pow(2.0, -i);
        atan_lut[i]  = (int32_t)round(ang_circ / M_PI * SCALE);
        atanh_lut[i] = (int32_t)round(ang_hypr / M_PI * SCALE);
        linr_lut[i]  = (int32_t)round(ang_linr * SCALE);
    }
    luts_initialized = 1;
}

/* ────────────────────────────────────────────
 * CORDIC core iterations
 * ──────────────────────────────────────────── */
static void cordic_iterate(
    int32_t *px, int32_t *py, int32_t *pz,
    int coord, int mode
) {
    int32_t x = *px, y = *py, z = *pz;

    for (int s = 0; s < STAGES; s++) {
        int     shift;
        int32_t alpha;

        if (coord == COORD_CIRC) {
            shift = s;
            alpha = atan_lut[s];
        } else if (coord == COORD_LINR) {
            shift = s;
            alpha = linr_lut[s];
        } else { /* HYPR */
            shift = hypr_sched[s];
            alpha = atanh_lut[s];
        }

        /* Direction */
        int d;
        if (mode == MODE_ROT) {
            d = (z >= 0) ? 1 : -1;
        } else {
            if (coord == COORD_HYPR)
                d = (y >= 0) ? 1 : -1;
            else
                d = (y >= 0) ? -1 : 1;
        }

        int32_t xs = arith_shr(x, shift);
        int32_t ys = arith_shr(y, shift);

        int32_t xn, yn;
        if (coord == COORD_CIRC) {
            xn = to_signed16(x - d * ys);
            yn = to_signed16(y + d * xs);
        } else if (coord == COORD_LINR) {
            xn = x;
            yn = to_signed16(y + d * xs);
        } else { /* HYPR */
            xn = to_signed16(x + d * ys);
            yn = to_signed16(y + d * xs);
        }
        int32_t zn = to_signed16(z - d * alpha);

        x = xn; y = yn; z = zn;
    }

    *px = x; *py = y; *pz = z;
}

/* ────────────────────────────────────────────
 * Pre-processing
 * ──────────────────────────────────────────── */
static int32_t pre_proc(
    int32_t *px, int32_t *py, int32_t *pz,
    int coord, int mode
) {
    int32_t quad_offset = 0;
    if (coord == COORD_CIRC) {
        int32_t pi_half = SCALE / 2;   /* 8192 */
        int32_t pi_val  = SCALE;       /* 16384 */
        int32_t x = *px, y = *py, z = *pz;

        if (mode == MODE_ROT) {
            if (z > pi_half) {
                *px = -x; *py = -y; *pz = to_signed16(z - pi_val);
            } else if (z < -pi_half) {
                *px = -x; *py = -y; *pz = to_signed16(z + pi_val);
            }
        } else { /* VECTORING */
            if (x < 0) {
                *px = -x; *py = -y;
                quad_offset = (y >= 0) ? pi_val : -pi_val;
            }
        }
    }
    return quad_offset;
}

/* ────────────────────────────────────────────
 * DPI-C exported function
 * ──────────────────────────────────────────── */
void cordic_golden(
    int  x_in,
    int  y_in,
    int  z_in,
    int  coord,
    int  mode,
    int *x_out,
    int *y_out,
    int *z_out
) {
    init_luts();

    int32_t x = to_signed16((int32_t)x_in);
    int32_t y = to_signed16((int32_t)y_in);
    int32_t z = to_signed16((int32_t)z_in);

    int32_t quad_offset = pre_proc(&x, &y, &z, coord, mode);
    cordic_iterate(&x, &y, &z, coord, mode);

    /* Gain correction (apply K_inv) */
    if (coord != COORD_LINR) {
        /* K_inv in Q2.14 */
        int32_t kinv = (coord == COORD_CIRC) ? 9949 : 19784;
        x = to_signed16((int32_t)(((int64_t)x * kinv) >> FRAC_BITS));
        y = to_signed16((int32_t)(((int64_t)y * kinv) >> FRAC_BITS));
    }

    /* Quadrant offset for circular vectoring */
    if (coord == COORD_CIRC && mode == MODE_VEC)
        z = to_signed16(z + quad_offset);

    *x_out = (int)x;
    *y_out = (int)y;
    *z_out = (int)z;
}

/* ────────────────────────────────────────────
 * Standalone test (build without DPI)
 * ──────────────────────────────────────────── */
#ifdef STANDALONE_TEST
#include <stdio.h>

static int all_pass = 1;
static void check(const char *label, double got, double expected, double tol) {
    int pass = fabs(got - expected) <= tol;
    if (!pass) all_pass = 0;
    printf("  [%s] %s: got=%.4f expected=%.4f\n", pass?"PASS":"FAIL", label, got, expected);
}

int main(void) {
    init_luts();
    int xo, yo, zo;

    /*
     * For CIRC ROTATION (APPLY_GAIN=1, post-processing):
     *   input x=SCALE (=1.0), y=0, z=angle
     *   output x = cos(angle), y = sin(angle)  (after gain correction)
     *   Safe: K * SCALE = 1.6468 * 16384 = 26985 < 32767 - no overflow
     *
     * For CIRC VECTORING with safe inputs:
     *   x=SCALE/2, y=SCALE/2  → no overflow in first CORDIC stage (x+y = SCALE < 32767)
     *   output z = atan2(y,x)/π * SCALE, x = sqrt(x²+y²)
     */

    printf("CORDIC C Model Self-Test\n");
    printf("========================\n");

    /* --- Circular Rotation --- */
    cordic_golden(SCALE, 0, 0,     COORD_CIRC, MODE_ROT, &xo, &yo, &zo);
    check("cos(0°)=1.0",    xo/(double)SCALE, 1.0,    0.01);
    check("sin(0°)=0.0",    yo/(double)SCALE, 0.0,    0.01);

    cordic_golden(SCALE, 0, 8192,  COORD_CIRC, MODE_ROT, &xo, &yo, &zo);
    check("cos(90°)=0.0",   xo/(double)SCALE, 0.0,    0.01);
    check("sin(90°)=1.0",   yo/(double)SCALE, 1.0,    0.01);

    cordic_golden(SCALE, 0, 4096,  COORD_CIRC, MODE_ROT, &xo, &yo, &zo);
    check("cos(45°)=0.707", xo/(double)SCALE, 0.7071, 0.01);
    check("sin(45°)=0.707", yo/(double)SCALE, 0.7071, 0.01);

    /* cos(180°): use z = -SCALE (=-π) via signed input */
    cordic_golden(SCALE, 0, -SCALE, COORD_CIRC, MODE_ROT, &xo, &yo, &zo);
    check("cos(180°)=-1.0", xo/(double)SCALE, -1.0,   0.01);

    /* --- Circular Vectoring --- */
    /* atan2(0, 1) = 0°: (x=SAFE, y=0) */
    int SAFE = SCALE/2;
    cordic_golden(SAFE, 0, 0, COORD_CIRC, MODE_VEC, &xo, &yo, &zo);
    check("atan2(0,1)=0.0",   zo/(double)SCALE, 0.0,   0.01);
    check("mag(1,0)=0.5",     xo/(double)SCALE, 0.5,   0.01);

    /* atan2(1, 1) = π/4: (x=SAFE, y=SAFE) */
    cordic_golden(SAFE, SAFE, 0, COORD_CIRC, MODE_VEC, &xo, &yo, &zo);
    check("atan2(1,1)=0.25π", zo/(double)SCALE, 0.25,  0.02);
    check("mag(1,1)=0.707",   xo/(double)SCALE, 0.5*1.41421, 0.02);

    /* --- Linear Rotation: y_out = y_in + x_in * z_in (Q2.14) --- */
    /* x=SAFE (0.5), y=0, z=SAFE (0.5) → y_out = 0 + 0.5*0.5 = 0.25 */
    cordic_golden(SAFE, 0, SAFE, COORD_LINR, MODE_ROT, &xo, &yo, &zo);
    check("linear 0.5*0.5=0.25", yo/(double)SCALE, 0.25, 0.02);

    printf("\nResult: %s\n", all_pass ? "ALL PASS" : "SOME FAIL");
    return all_pass ? 0 : 1;
}
#endif
