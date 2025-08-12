# Thinker Robot Engine Control

Engine control code

## Usage


1. Configure the build process: `cmake -S . -B build/`.
1. Build the app: `cmake --build build`.
1. use `./deploy-shouler.sh` to deploy shoulder part

## Debug vs Release
```shell
cmake -S . -B build -D CMAKE_BUILD_TYPE=Debug
```

For a release build, which among various optimisations omits UART debugging code, call:

```shell
cmake -S . -B build -D CMAKE_BUILD_TYPE=Release
```

Follow both of these commands with the usual

```shell
cmake --build build
```