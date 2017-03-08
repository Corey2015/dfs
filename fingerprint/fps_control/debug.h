#ifndef __debug_h__
#define __debug_h__


#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
//
// Debug Level
//

#define LEVEL_ERROR  (0)
#define LEVEL_WARN   (1)
#define LEVEL_INFO   (2)
#define LEVEL_DEBUG  (3)
#define LEVEL_DETAIL (4)


////////////////////////////////////////////////////////////////////////////////
//
// Debug Messages
//

#if (defined(__WINDOWS__) && (_MSC_VER == 1200))
    #define ERROR  print_error
    #define WARN   print_warn
    #define INFO   print_info
    #define DEBUG  print_debug
    #define DETAIL print_detail
#else
    #define ERROR(...)  do { (void) print_error (TEXT(__FILE__), __LINE__, __VA_ARGS__); } while(0)
    #define WARN(...)   do { (void) print_warn  (TEXT(__FILE__), __LINE__, __VA_ARGS__); } while(0)
    #define INFO(...)   do { (void) print_info  (TEXT(__FILE__), __LINE__, __VA_ARGS__); } while(0)
    #define DEBUG(...)  do { (void) print_debug (TEXT(__FILE__), __LINE__, __VA_ARGS__); } while(0)
    #define DETAIL(...) do { (void) print_detail(TEXT(__FILE__), __LINE__, __VA_ARGS__); } while(0)
#endif


////////////////////////////////////////////////////////////////////////////////
//
// Prototypes
//

int print_error(__INPUT char_t *file,
                __INPUT int     line,
                __INPUT char_t *format, ...);

int print_warn(__INPUT char_t *file,
               __INPUT int     line,
               __INPUT char_t *format, ...);

int print_info(__INPUT char_t *file,
               __INPUT int     line,
               __INPUT char_t *format, ...);

int print_debug(__INPUT char_t *file,
                __INPUT int     line,
                __INPUT char_t *format, ...);

int print_detail(__INPUT char_t *file,
                 __INPUT int     line,
                 __INPUT char_t *format, ...);

int set_debug_level(__INPUT int level);
int get_debug_level(__OUTPUT int *level);


#ifdef __cplusplus
}
#endif


#endif // __debug_h__
