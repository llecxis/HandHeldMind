#common utilities for all module

def trap_exc_during_debug(*args):
    # when app raises uncaught exception, print info
    print(args)