#pragma once

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <string>
#include <sstream>
#include <iostream>
#include <mutex>

// Assert stubs
#undef ASSERT
#undef FAIL
#undef VERIFY

#ifndef NDEBUG
#define ASSERT(c) assert(c)
#define VERIFY(c) assert(c)
#else
#define ASSERT(c) static_cast<void>(0)
#define VERIFY(c) static_cast<void>(c)
#endif

#define FAIL(text) \
  do \
  { \
    std::stringstream _bh_ss; \
    _bh_ss << text; \
    fprintf(stderr, "FATAL: %s (%s:%d)\n", _bh_ss.str().c_str(), __FILE__, __LINE__); \
    std::abort(); \
  } \
  while(false)

#define DECLARE_SYNC std::recursive_mutex _mutex
#define SYNC std::lock_guard<std::recursive_mutex> _lock(_mutex)

#define BH_TRACE static_cast<void>(0)

// SystemCall stub
namespace SystemCall
{
  inline unsigned getCurrentSystemTime()
  {
    using namespace std::chrono;
    static const auto start = steady_clock::now();
    return static_cast<unsigned>(duration_cast<milliseconds>(steady_clock::now() - start).count());
  }

  inline void playSound(const char*) {}
  inline void say(const char*, bool = false) {}
  inline void say(const std::string&, bool = false) {}
}
