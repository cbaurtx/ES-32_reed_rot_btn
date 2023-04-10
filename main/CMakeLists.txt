set(COMPONENT_SRCS "keypad_example_main.c")

set(COMPONENT_ADD_INCLUDEDIRS "../components/reed_rot_btn/include")
set(COMPONENT_COMPILE_OPTIONS "-Wno-error=format -Wno-format")
register_component()
