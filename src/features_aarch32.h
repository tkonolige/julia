// This file is a part of Julia. License is MIT: https://julialang.org/license

// AArch32 features definition
// hwcap
AArch32_FEATURE_DEF(neon, 12, 0)
AArch32_FEATURE_DEF(vfp3, 13, 0)
// AArch32_FEATURE_DEF(vfpv3d16, 14, 0) // d16
AArch32_FEATURE_DEF(vfp4, 16, 0)
AArch32_FEATURE_DEF(hwdiv_arm, 17, 0)
AArch32_FEATURE_DEF(hwdiv, 18, 0)
AArch32_FEATURE_DEF(d32, 19, 0) // -d16

// hwcap2
AArch32_FEATURE_DEF(crypto, 32 + 0, 0)
AArch32_FEATURE_DEF(crc, 32 + 4, 0)
// AArch32_FEATURE_DEF(ras, 32 + ???, 0)
// AArch32_FEATURE_DEF(fullfp16, 32 + ???, 0)

// custom bits to match llvm model
AArch32_FEATURE_DEF(aclass, 32 * 2 + 0, 0)
AArch32_FEATURE_DEF(rclass, 32 * 2 + 1, 0)
AArch32_FEATURE_DEF(mclass, 32 * 2 + 2, 0)
AArch32_FEATURE_DEF(v7, 32 * 2 + 3, 0)
AArch32_FEATURE_DEF(v8, 32 * 2 + 4, 0)
AArch32_FEATURE_DEF(v8_1a, 32 * 2 + 5, 0)
AArch32_FEATURE_DEF(v8_2a, 32 * 2 + 6, 0)
// AArch32_FEATURE_DEF(v8_3a, 32 * 2 + 7, ?)
AArch32_FEATURE_DEF(v8_m_main, 32 * 2 + 8, 0)
