void* __dso_handle = NULL;

namespace __gnu_cxx
{
    void
    __attribute__((noreturn))
    __verbose_terminate_handler();

    void
    __verbose_terminate_handler() {
        while (1);
    }
}

extern "C" {
    void
    __attribute__((noreturn))
    __cxa_pure_virtual();

    void
    __cxa_pure_virtual() {
        while (1);
    }

    void _sbrk(void) {
    }

    int __aeabi_atexit(
        void *object,
        void (*destructor)(void *),
        void *dso_handle)
    {
        return 0;
    }

    void
    __attribute__((noreturn))
    __assert_func (
        const char __attribute__((unused)) *file,
        int __attribute__((unused))  line,
        const char __attribute__((unused))  *func,
        const char __attribute__((unused))  *failedexpr)
    {
        while (1);
        /* NOTREACHED */
    }
}


