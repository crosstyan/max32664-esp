# HR band

Require [`esp-idf v5.3.2`](https://github.com/espressif/esp-idf/tree/v5.3.2)

## Clangd

See also [`.clangd`](.clangd) for the configuration file. You might need to modify the your Visual Studio Code settings.

```jsonc
{
    // Your Visual Studio Code settings
    "clangd.arguments": [
        "--compile-commands-dir=${workspaceFolder}/build",
        "--query-driver=**/*"
    ],
}
```

