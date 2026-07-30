// PS5 ("Prospero") platform support for Jolt Physics.
//@ BASTIEN ADD
#pragma once

// PS5 toolchain is Clang-based (ARM64-style intrinsics not applicable, x86_64 target)
#define JPH_BREAKPOINT __builtin_trap()

// JPH_PLATFORM_PROSPERO_GET_TICKS / _MUTEX* / _RWLOCK* / _SEMAPHORE* are
// intentionally NOT defined here: leaving them undefined makes
// TickCounter.h / Mutex.h / Semaphore.h fall back to their portable
// generic-x86 / std::mutex / std::condition_variable implementations,
// which are correct (if not maximally fast) on PS5. Define these later
// using PS5 SDK primitives (sceKernel* mutex/rwlock/semaphore APIs and
// the PS5 tick-counter API) as a follow-up perf pass.
//@ BASTIEN END
