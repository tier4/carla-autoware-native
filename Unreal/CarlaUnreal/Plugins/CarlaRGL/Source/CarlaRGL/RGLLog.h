// RGLLog.h
#pragma once
#include <iostream>

namespace RGLLog
{
#if UE_BUILD_SHIPPING
    template <typename... Args>
    inline void Info(Args&&...) {}
#else
    template <typename... Args>
    inline void Info(Args&&... args)
    {
        std::cout << "INFO_RGL:";
        ((std::cout << ' ' << std::forward<Args>(args)), ...);
        std::cout << std::endl;
    }
#endif
}
