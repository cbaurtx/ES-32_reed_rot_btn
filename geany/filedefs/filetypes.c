[build-menu]
FT_00_LB=Memory map
FT_00_CM=idf.py -C ~/Development/Source/ESP32/keypad_rotdecode/  size-components
FT_00_WD=
FT_01_LB=_Build
FT_01_CM=idf.py -C ~/Development/Source/ESP32/keypad_rotdecode/ build
FT_01_WD=
FT_02_LB=_Lint
FT_02_CM=cppcheck --language=c --enable=warning,style --template=gcc "%f"
FT_02_WD=
