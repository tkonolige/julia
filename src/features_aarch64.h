// This file is a part of Julia. License is MIT: https://julialang.org/license

// AArch64 features definition
// hwcap
AArch64_FEATURE_DEF(crypto, 3, 0)
AArch64_FEATURE_DEF(crc, 7, 0)
AArch64_FEATURE_DEF(lse, 8, 0)
AArch64_FEATURE_DEF(fullfp16, 9, 0)
AArch64_FEATURE_DEF(rdm, 12, 50000)
AArch64_FEATURE_DEF(jscvt, 13, UINT32_MAX)
AArch64_FEATURE_DEF(fcma, 14, UINT32_MAX)
AArch64_FEATURE_DEF(lrcpc, 15, UINT32_MAX)
// AArch64_FEATURE_DEF(ras, ???, 0)
// AArch64_FEATURE_DEF(sve, ???, 0)

// hwcap2
// AArch64_FEATURE_DEF(?, 32 + ?, 0)

// custom bits to match llvm model
AArch64_FEATURE_DEF(v8_1a, 32 * 2 + 0, 0)
AArch64_FEATURE_DEF(v8_2a, 32 * 2 + 1, 0)
// AArch64_FEATURE_DEF(v8_3a, 32 * 2 + 2, ?)
