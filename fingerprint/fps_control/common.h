#ifndef __common_h__
#define __common_h__


///////////////////////////////////////////////////////////////////////////////
//
// Environment
//

#if (defined(_WINDOWS) || defined(WINDOWS))
    #define __WINDOWS__
#endif

#if (defined(__linux) || defined(__linux__))
    #define __LINUX__
#endif

#if (defined(_UNICODE) || defined(UNICODE))
    #define __UNICODE__
#endif

#if (defined(_DEBUG) || defined(DEBUG))
    #define __DEBUG__
#endif


////////////////////////////////////////////////////////////////////////////////
//
// Include Dependencies
//

#if defined(__cplusplus)
    #if defined(__UNICODE__)
        #include <cwchar>
    #else
        #include <cstdio>
        #include <cstring>
    #endif
    #include <string>
#else
    #if defined(__UNICODE__)
        #include <wchar.h>
    #else
        #include <stdio.h>
        #include <string.h>
    #endif
#endif

#if defined(__WINDOWS__)
    #include <windows.h>
#endif


///////////////////////////////////////////////////////////////////////////////
//
// Data Types
//

typedef char               i08_t;
typedef short int          i16_t;
typedef int                i32_t;
typedef unsigned char      u08_t;
typedef unsigned short int u16_t;
typedef unsigned int       u32_t;
typedef signed char        s08_t;
typedef signed short int   s16_t;
typedef signed int         s32_t;
typedef float              f32_t;
typedef double             f64_t;
typedef int                bit_t;

#if defined(__UNICODE__)
typedef wchar_t            char_t;
#else
typedef char               char_t;
#endif

#if defined(__cplusplus)
typedef bool               bool_t;
#else
typedef int                bool_t;
#endif

#if defined(__cplusplus)
#if defined(__UNICODE__)
typedef std::wstring       str_t;
#else
typedef std::string        str_t;
#endif
#endif


////////////////////////////////////////////////////////////////////////////////
//
// Constants
//

// Null pointer
#if !defined(NULL)
    #if defined(__cplusplus)
        #define NULL (0)
    #else
        #define NULL ((void *) 0)
    #endif
#endif

// Maximum and minimum values for strings
#define MAX_FNAME_LENGTH  (4096)
#define MAX_STRING_LENGTH (256)


////////////////////////////////////////////////////////////////////////////////
//
// Basic Conditions
//

// True or False
#if !defined(TRUE)
    #define TRUE (1 == 1)
#endif
#if !defined(FALSE)
    #define FALSE (!TRUE)
#endif

// Pass or Fail
#if !defined(PASS)
    #define PASS (TRUE)
#endif
#if !defined(FAIL)
    #define FAIL (!PASS)
#endif

// High or Low
#if !defined(HIGH)
    #define HIGH (TRUE)
#endif
#if !defined(LOW)
    #define LOW (!HIGH)
#endif

// String match or mismatch
#define STRING_MATCHED    (0)
#define STRING_MISMATCHED (!STRING_MATCHED)


////////////////////////////////////////////////////////////////////////////////
//
// Function Argument Attributes
//

#if !defined(__INPUT)
    #define __INPUT const
#endif

#if !defined(__OUTPUT)
    #define __OUTPUT
#endif

#if !defined(__INOUT)
    #define __INOUT
#endif


////////////////////////////////////////////////////////////////////////////////
//
// Basic Utilities
//

#if !defined(DUMMY_VAR)
    #if defined(__WINDOWS__)
        #define DUMMY_VAR(_x_) UNREFERENCED_PARAMETER(_x_)
    #else
        #define DUMMY_VAR(_x_) (_x_)
    #endif
#endif

#if !defined(ARRAY_SIZE)
    #define ARRAY_SIZE(_x_) (sizeof(_x_) / sizeof((_x_)[0]))
#endif

#if !defined(BIT)
    #define BIT(_x_) (1 << (_x_))
#endif

#if !defined(MAX)
    #define MAX(_a_, _b_) (((_a_) > (_b_)) ? (_a_) : (_b_))
#endif

#if !defined(MIN)
    #define MIN(_a_, _b_) (((_a_) < (_b_)) ? (_a_) : (_b_))
#endif

#if !defined(CONSTRAINT)
    #define CONSTRAINT(_u_, _x_, _l_) \
        (((_x_) > (_u_)) ? (_u_) : ((_x_) < (_l_)) ? (_l_) : (_x_))
#endif

#if !defined(SQUARE)
    #define SQUARE(_x_) ((_x_) * (_x_))
#endif

#if (!defined(__WINDOWS__) && !defined(TEXT))
    #define TEXT(_x_) _TEXT_(_x_)
    #if defined(__UNICODE__)
        #define _TEXT_(_x_) L##_x_
    #else
        #define _TEXT_(_x_) _x_
    #endif
#endif

#if !defined(__FUNC__)
    #if (defined(__WINDOWS__) && (_MSC_VER == 1200)) // For Visual Studio 6
        #define __FUNC__ TEXT(" ")
    #else
        #define __FUNC__ __func__
    #endif
#endif

#if !defined(SWAP)
    #if (defined(__WINDOWS__) && (_MSC_VER == 1200)) // For Visual Studio 6
        #define SWAP(_t_, _a_, _b_) \
            do { _t_ _x_; _x_ = (_a_); (_a_) = (_b_); (_b_) = _x_; } while (0) \
    #else
        #define SWAP(_t_, _a_, _b_) \
            do { _t_ _x_; _x_ = (_a_); (_a_) = (_b_); (_b_) = _x_; } while (0)
    #endif
#endif

#if defined(__UNICODE__)
    #if (defined(__WINDOWS__) && (_MSC_VER == 1200)) // For Visual Studio 6
        #define PRINTF                                wprintf
        #define SNPRINTF                              _snwprintf
        #define VSNPRINTF(_str_, _len_, _fmt_, _arg_) _vsnwprintf((_str_), (_len_), (_fmt_), (_arg_))
        #define FPRINTF                               fwprintf
    #else
        #define PRINTF(...)                           wprintf(__VA_ARGS__)
        #define SNPRINTF(_str_, _len_, ...)           swprintf((_str_), (_len_), __VA_ARGS__)
        #define VSNPRINTF(_str_, _len_, _fmt_, _arg_) vswprintf((_str_), (_len_), (_fmt_), (_arg_))
        #define FPRINTF(_stream_, ...)                fwprintf((_stream_), __VA_ARGS__)
    #endif

    #define STRLEN(_str_)                             wcslen(_str_)
    #define STRNCMP(_str1_, _str2_, _len_)            wcsncmp((_str1_), (_str2_), (_len_))
    #define STRTOL(_str_, _type_, _base_)             wcstol((_str_), (_type_), (_base_))
    #define STRTOD(_str_, _type_)                     wcstod((_str_), (_type_))
    #define STRNCPY(_dst_, _src_, _len_)              wcsncpy((_dst_), (_src_), (_len_))
    #define STRCPY(_dst_, _src_)                      wcscpy((_dst_), (_src_))
    #define FGETS(_str_, _len_, _stream_)             fgetws((_str_), (_len_), (_stream_))
    #define MEMSET(_str_, _char_, _len_)              wmemset((_str_), (_char_), (_len_))
#else
    #if (defined(__WINDOWS__) && (_MSC_VER == 1200)) // For Visual Studio 6
        #define PRINTF                                printf
        #define SNPRINTF                              _snprintf
        #define VSNPRINTF(_str_, _len_, _fmt_, _arg_) _vsnprintf((_str_), (_len_), (_fmt_), (_arg_))
        #define FPRINTF                               fprintf
    #else
        #define PRINTF(...)                           printf(__VA_ARGS__)
        #define SNPRINTF(_str_, _len_, ...)           snprintf((_str_), (_len_), __VA_ARGS__)
        #define VSNPRINTF(_str_, _len_, _fmt_, _arg_) vsnprintf((_str_), (_len_), (_fmt_), (_arg_))
        #define FPRINTF(_stream_, ...)                fprintf((_stream_), __VA_ARGS__)
    #endif

    #define STRLEN(_str_)                             strlen(_str_)
    #define STRNCMP(_str1_, _str2_, _len_)            strncmp((_str1_), (_str2_), (_len_))
    #define STRTOL(_str_, _type_, _base_)             strtol((_str_), (_type_), (_base_))
    #define STRTOD(_str_, _type_)                     strtod((_str_), (_type_))
    #define STRNCPY(_dst_, _src_, _len_)              strncpy((_dst_), (_src_), (_len_))
    #define STRCPY(_dst_, _src_)                      strcpy((_dst_), (_src_))
    #define FGETS(_str_, _num_, _stream_)             fgets((_str_), (_num_), (_stream_))
    #define MEMSET(_str_, _char_, _len_)              memset((_str_), (_char_), (_len_))
#endif


#endif // __common_h__
