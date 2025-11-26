# TANHFP32 - Hardware Hyperbolic Tangent Function Implementation

A high-performance hardware implementation of the hyperbolic tangent function (tanh) for single-precision floating-point numbers (FP32), designed using Chisel HDL.

## Overview

This project implements **TANHFP32**, which computes `tanh(x)` for FP32 inputs using a pipelined architecture that combines lookup tables (LUT) and piecewise linear approximation to achieve efficient hardware implementation with controlled accuracy trade-offs.

This project reuses the **XiangShan Fudian** floating-point unit library for basic FP32 arithmetic operations (multiplication, addition, fused multiply-add).

## Algorithm

The computation is based on piecewise linear approximation with range-based optimizations:

$$
\begin{equation}
\begin{aligned}
f             &= \tanh(x) \\
\\
\tanh(-x)     &= -\tanh(x) \\
\\
y             &= \tanh(|x|) \\
\\
\text{result} &= \text{sign}(x)\cdot y \\
\\
\text{For } |x|\in[2^{-5},8): \\
\\
\tanh(x)      &= \text{base}[i] + r \cdot \text{slope}[i] \\
\\
i             &= (e_{\text{unbias}} + 5)\ll 5 \;\;+\;\; \text{mantissa}[22:18] \\
\\
r             &= |x| - x_{\mathrm{lo}}(i) \\
\\
\text{For } |x|\ge 8: \\
\\
\tanh(x)      &=\text{sign}(x)\cdot 1.0 \\
\\
\text{For } |x| < 2^{-5}: \\
\\
\tanh(x)      &= x
\end{aligned}
\end{equation}
$$

### Key Components

- **Sign extraction**: Compute absolute value $|x|$ and store sign bit for later restoration
- **Exponent and mantissa decomposition**: Decompose $|x|$ into IEEE 754 fields
- **Segment indexing**: Construct segment index directly from $(e_\text{unbias}, \text{mantissa}[22:18])$
- **Base point reconstruction**: Rebuild segment left endpoint $x_{\mathrm{lo}}$ from index
- **Offset calculation**: Compute in-segment offset $r=|x| - x_{\mathrm{lo}}$
- **LUT access**: Retrieve $\mathrm{base}[i], \mathrm{slope}[i]$ from lookup tables (256 entries each)
- **Linear interpolation**: Compute $y=\mathrm{base}[i] + r \cdot \mathrm{slope}[i]$
- **Sign restoration**: Apply original sign to get final result

## Hardware Design

### Input Filtering

Special value handling and input validation:

- **Special values**: NaN (return NaN), ±Inf (return ±1)
- **Zero**: +0 or -0 → return 0
- **Subnormal numbers**: Return input directly (approximation for small values)
- **Range bypass**: $|x| < 2^{-5}$ returns input, $|x| \ge 8$ returns ±1

### Pipeline Structure

```
S0: Input Filtering and Special Value Handling
    - Special values: is_nan, is_inf, is_zero, is_subnorm
    - Decomposition: sign, exp_field, frac, xAbs
    - Range checks: small_bypass (e_unbias < -5), large_bypass (e_unbias >= 3)
    - Generate bypass signal and bypass_val

S1: Segment Index Construction + x_lo Reconstruction + LUT Access
    - e_off = e_unbias + 5
    - m_hi5 = frac[22:18]
    - region = (e_off[2:0] << 5) | m_hi5
    - base = LUT_base[region], slope = LUT_slope[region]
    - x_lo = {1'b0, exp_field, m_hi5, 18'b0}

S2: Offset Calculation + FMA Interpolation (2 FADD stages)
    - r = xAbs - x_lo
    - Subtraction using FADD pipeline from Fudian library

S3-S7: Linear Interpolation (5 FCMA stages)
    - y = base + slope × r
    - Uses FCMA (Fused Multiply-Add) pipeline from Fudian library

S8: Sign Restoration + Bypass Multiplexing
    - y_signed = sign ? -y : y
    - result = bypass ? bypass_val : y_signed
```

## Performance Results

### Accuracy Results

Verification on 1,000,000 random test cases:

#### CPU Reference (cmath tanhf)

```
Total:  1,000,000 test cases
Pass:     440,975 (44.10%)
Fail:     559,025 (55.90%)

Average Error: 6.122838e-06
Maximum Error: 7.688999e-05

Average ULP: 109.84
Maximum ULP: 5456

Total Cycles: 1,000,073
Throughput:   1 result/cycle
```

#### GPU Reference (NVIDIA RTX 5060 with -use_fast_math)

```
Total:  1,000,000 test cases
Pass:     454,959 (45.50%)
Fail:     545,041 (54.50%)

Average Error: 6.167674e-06
Maximum Error: 7.730722e-05

Average ULP: 111.08
Maximum ULP: 5540

Total Cycles: 1,000,073
Throughput:   1 result/cycle
```

## Dependencies

### Required

- **Chisel 6.6.0**: Hardware description language
- **Scala 2.13.15**: Programming language for Chisel
- **Mill**: Build tool for Scala/Chisel projects
- **Verilator**: For simulation and verification
- **XiangShan Fudian**: Floating-point arithmetic library (included as git submodule)

### Optional

- **CUDA/NVCC**: For GPU-accelerated reference implementation (NVIDIA GPU required)
- **Synopsys Design Compiler**: For ASIC synthesis (if targeting specific process technology)

## Building

### Initialize Dependencies

```bash
make init
```

This will initialize the XiangShan Fudian submodule.

### Generate SystemVerilog

```bash
# Generate TANHFP32 RTL
./mill --no-server TANHFP32.run
```

The generated SystemVerilog will be placed in `rtl/TANHFP32.sv`.

### Build and Run Simulation

#### CPU Reference (no GPU required)

```bash
make USE_GPU_REF=0 run
```

Uses standard C library `tanhf()` function as the reference.

#### GPU Reference (requires NVIDIA GPU + CUDA)

```bash
make USE_GPU_REF=1 run
```

Uses NVIDIA CUDA math library with `-use_fast_math` flag for hardware-accurate reference.

### Clean Build Artifacts

```bash
make clean
```

## Testing and Verification

### Simulation

Verilator-based testbench with:

- Comprehensive test vector generation (1M test cases)
- Random input generation across full FP32 range
- Special value testing (NaN, Inf, zero, negative numbers, subnormals)
- ULP (Unit in Last Place) error measurement
- Waveform generation (FST format) for debugging

### Reference Models

- **CPU Reference**: Standard C library (`tanhf`)
- **GPU Reference**: NVIDIA CUDA math library with `-use_fast_math` (recommended for hardware comparison)

### Accuracy Metrics

- **ULP Error**: Measures floating-point accuracy in terms of "units in the last place"
- **Absolute Error**: Direct numerical difference between result and reference
- **Pass/Fail**: Bit-exact comparison against reference implementation

## Future Improvements

- [ ] Optimize base/slope table values using numerical optimization techniques

## Credits

- **XiangShan Fudian FPU Library**: Provides high-quality floating-point arithmetic components
  - Repository: <https://github.com/OpenXiangShan/fudian>
  - Used for: FMUL, FADD, FCMA (Fused Multiply-Add), RawFloat utilities

## References

- IEEE Standard for Floating-Point Arithmetic (IEEE 754-2008)
- XiangShan Fudian FPU: <https://github.com/OpenXiangShan/fudian>
- Chisel/FIRRTL Documentation: <https://www.chisel-lang.org/>
- Handbook of Floating-Point Arithmetic (Muller et al.)
- "Elementary Functions: Algorithms and Implementation" (Muller, 2006)

## License

This project reuses the XiangShan Fudian library. Please refer to the respective license files in the `dependencies/fudian` directory for licensing terms.
