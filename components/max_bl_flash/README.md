https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/linker-script-generation.html#linker-fragment-files

The linker script template currently used is esp_system/ld/esp32/sections.ld.in; the generated output script sections.ld is put under its build directory.

https://sourceware.org/binutils/docs-2.16/ld/File-Commands.html#File-Commands
https://stackoverflow.com/questions/46052275/gnu-linker-get-objects-through-input
https://mcuoneclipse.com/2022/06/17/include-bin-binary-files-in-a-gnu-linker-file/


## ldgen

https://github.com/espressif/esp-idf/tree/master/tools/ldgen


app.msbl = 
MAX32664C_HSP2_WHRM_AEC_SCD_WSPO2_Z_30.13.30.msbl

## embed

https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html#embedding-binary-data
