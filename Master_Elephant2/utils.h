#ifndef __UTILS_H__
#define __UTILS_H__

#ifdef DEBUG && !defined(NOPRINT) // DEBUG defined
    // Does printf with format and arguments, then flushes stdout
    #define PRINTF(format, ...) \
        do { \
            printf(format, ##__VA_ARGS__); \
            fflush(stdout); \
        } while(0)
#else // DEBUG undefined
    // Does nothing
    #define PRINTF(format, ...)
#endif

#endif // __UTILS_H__
