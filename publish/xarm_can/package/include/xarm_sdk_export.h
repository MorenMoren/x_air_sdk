#pragma once

#if defined(_WIN32) || defined(_WIN64)
#if defined(XARM_SDK_BUILD)
#define XARM_SDK_API __declspec(dllexport)
#else
#define XARM_SDK_API __declspec(dllimport)
#endif
#else
#define XARM_SDK_API __attribute__((visibility("default")))
#endif
