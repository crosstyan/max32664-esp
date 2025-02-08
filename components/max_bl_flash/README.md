# Flashing MAX32664 firmware

current `app.msbl` is `MAX32664C_HSP2_WHRM_AEC_SCD_WSPO2_C_30.13.31.msbl`. 

if you are using `MAXM86161`, you need `32.x.x` version of `app.msbl`.
Otherwise, you need `30.x.x` version of `app.msbl` for `MAX86141` AFE.

if you are using `LIS2DS12` accelerometer, you need to use `MAX32664C_OS58_I2C_1PD_WHRM_AEC_SCD_WSPO2_C_32.9.34.msbl`
according to [Compatibility Matrix](MAX32664C_MAXM86161_Release_Notes_and_Compatibility_Matrix.pdf)

- `MAX32664C_OS58_I2C_1PD_WHRM_AEC_SCD_WSPO2_C_32.9.23.msbl`
- `MAX32664C_OS58_I2C_1PD_WHRM_AEC_SCD_WSPO2_C_32.9.34.msbl`
- `MAX32664C_HSP2_WHRM_AEC_SCD_WSPO2_C_30.13.31.msbl`

## See also

- https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html#embedding-binary-data
- https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/linker-script-generation.html#linker-fragment-files

> The linker script template currently used is esp_system/ld/esp32/sections.ld.in; the generated output script sections.ld is put under its build directory.

- https://sourceware.org/binutils/docs-2.16/ld/File-Commands.html#File-Commands
- https://stackoverflow.com/questions/46052275/gnu-linker-get-objects-through-input
- https://mcuoneclipse.com/2022/06/17/include-bin-binary-files-in-a-gnu-linker-file/
- https://github.com/espressif/esp-idf/tree/master/tools/ldgen