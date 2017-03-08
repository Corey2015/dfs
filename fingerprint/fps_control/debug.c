#include <stdarg.h>
#include "common.h"
#include "debug.h"


#if defined(__WINDOWS__)
    #include <windows.h>
    #define DEBUG_OUTPUT(_str_) OutputDebugString(_str_)
#else
    #define DEBUG_OUTPUT(_str_) FPRINTF(stdout, _str_)
#endif

static int    debug_level  = LEVEL_DEBUG;
static char_t last_char    = TEXT('\n');

static int
print_message(__INPUT int      level,
              __INPUT char_t  *file,
              __INPUT int      line,
              __INPUT char_t  *format,
              __INPUT va_list  arg_ptr)
{
    if (format == NULL) {
        return -1;
    }

    if (level <= debug_level) {
        char_t text[MAX_STRING_LENGTH];
        int    num_of_chars = 0;

        if (last_char == TEXT('\n')) {
            switch (level) {
                case LEVEL_ERROR  : num_of_chars = SNPRINTF(text, ARRAY_SIZE(text), TEXT(" [ ERROR ] ")); break;
                case LEVEL_WARN   : num_of_chars = SNPRINTF(text, ARRAY_SIZE(text), TEXT("  [ WARN ] ")); break;
                case LEVEL_INFO   : num_of_chars = SNPRINTF(text, ARRAY_SIZE(text), TEXT("  [ INFO ] ")); break;
                case LEVEL_DEBUG  : num_of_chars = SNPRINTF(text, ARRAY_SIZE(text), TEXT(" [ DEBUG ] ")); break;
                case LEVEL_DETAIL : num_of_chars = SNPRINTF(text, ARRAY_SIZE(text), TEXT("[ DETAIL ] ")); break;
                default : return -1;
            }

            if (debug_level >= LEVEL_DEBUG) {
                num_of_chars = SNPRINTF(&text[STRLEN(text)], ARRAY_SIZE(text), TEXT("(%s:%d) "), file, line);
            }

            if (num_of_chars < 0) {
                return -1;
            }
        }

        num_of_chars = VSNPRINTF(&text[STRLEN(text)], (ARRAY_SIZE(text) - STRLEN(text)), format, arg_ptr);
        if (num_of_chars < 0) {
            return -1;
        }

        last_char = text[STRLEN(text) - 1];
        DEBUG_OUTPUT(text);
    }

    return 0;
}

int
print_error(__INPUT char_t *file,
            __INPUT int     line,
            __INPUT char_t *format, ...)
{
    int     status = 0;
    va_list arg_ptr;

    va_start(arg_ptr, format);
    status = print_message(LEVEL_ERROR, file, line, format, arg_ptr);
    va_end(arg_ptr);
    
    return status;
}

int
print_warn(__INPUT char_t *file,
           __INPUT int     line,
           __INPUT char_t *format, ...)
{
    int     status = 0;
    va_list arg_ptr;

    va_start(arg_ptr, format);
    status = print_message(LEVEL_WARN, file, line, format, arg_ptr);
    va_end(arg_ptr);
    
    return status;
}

int
print_info(__INPUT char_t *file,
           __INPUT int     line,
           __INPUT char_t *format, ...)
{
    int     status = 0;
    va_list arg_ptr;

    va_start(arg_ptr, format);
    status = print_message(LEVEL_INFO, file, line, format, arg_ptr);
    va_end(arg_ptr);
    
    return status;
}

int
print_debug(__INPUT char_t *file,
            __INPUT int     line,
            __INPUT char_t *format, ...)
{
    int     status = 0;
    va_list arg_ptr;

    va_start(arg_ptr, format);
    status = print_message(LEVEL_DEBUG, file, line, format, arg_ptr);
    va_end(arg_ptr);
    
    return status;
}

int
print_detail(__INPUT char_t *file,
             __INPUT int     line,
             __INPUT char_t *format, ...)
{
    int     status = 0;
    va_list arg_ptr;

    va_start(arg_ptr, format);
    status = print_message(LEVEL_DETAIL, file, line, format, arg_ptr);
    va_end(arg_ptr);
    
    return status;
}

int
set_debug_level(__INPUT int level)
{
    if ((level < LEVEL_ERROR) || (level > LEVEL_DETAIL)) {
        return -1;
    }

    debug_level = level;
    return 0;
}

int
get_debug_level(__OUTPUT int *level)
{
    *level = debug_level;
    return 0;
}


