# monster-truck-baremetal
Monster truck self driving  rc car

## Strict Clang diagnostics

Clang and AppleClang builds use strict warnings by default, including conversion,
shadowing, format, and language-extension diagnostics. Warnings are treated as
errors. The project also disables compiler-specific C++ extensions while
requiring the configured C++23 standard.

Disable strict warnings temporarily with `-DENABLE_STRICT_WARNINGS=OFF` when
working through existing diagnostics.

## Sanitizers and compiler hardening

Compiler hardening is enabled by default. GNU and Clang builds use stack
protection, and Release/RelWithDebInfo builds also enable `_FORTIFY_SOURCE=2`.
MSVC builds use `/GS` and `/sdl`.

Enable AddressSanitizer in its own build directory:

```sh
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DENABLE_ASAN=ON
cmake --build build-asan
./build-asan/bin/Robot
```

Enable ThreadSanitizer in a separate build directory:

```sh
cmake -S . -B build-tsan -DCMAKE_BUILD_TYPE=Debug -DENABLE_TSAN=ON
cmake --build build-tsan
./build-tsan/bin/Robot
```

ASan and TSan cannot be enabled in the same build. They are supported with GCC
and Clang-family compilers; MSVC supports the ASan option only. Sanitizers find
some memory errors and data races when the affected code runs, but they do not
make a program memory-safe or replace tests and code review. Hardware access,
operating-system support, and the selected compiler/runtime may also limit
which sanitizer builds can run.
