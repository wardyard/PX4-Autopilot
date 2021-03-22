px4_add_module(
    MODULE examples__inject_customUORBmsg
    MAIN inject_myUORB
    SRCS
        inject_FCvaluesmsg.cpp
    DEPENDS
)
