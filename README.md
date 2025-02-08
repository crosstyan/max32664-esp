# Heart Rate Band

This is a project for using the MAX32664 as a heart rate monitor with a ESP32H2 MCU.

See [MAX32664 User Guide](docs/max32664-user-guide-txt.md) if you need details on the MAX32664.

Require [`esp-idf v5.3.2`](https://github.com/espressif/esp-idf/tree/v5.3.2) to build the project, don't use `Arduino`.

## Code Style

I prefer to use modern C++ (C++23). Using of `std::span`, `std::optional`,
`tl::expected` is encouraged.

Using of lambda is encouraged, unless external linkage is required.

## Clangd

See also [`.clangd`](.clangd) for the configuration file. You might need to apply following settings to your Visual Studio Code.

```jsonc
{
    "clangd.arguments": [
        "--compile-commands-dir=${workspaceFolder}/build",
        "--query-driver=**/*"
    ],
}
```
