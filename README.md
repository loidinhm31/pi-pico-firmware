# Setup Picotool
```html
https://github.com/raspberrypi/picotool
```

```html
https://probe.rs/docs/getting-started/installation/
```

## Install lib-udev
```shell
sudo apt install libudev-dev
```

```shell
cargo install elf2uf2-rs
```

```shell
cargo build --release
```

```shell
elf2uf2-rs target/thumbv6m-none-eabi/release/app
```
OR
```shell
elf2uf2-rs -d target/thumbv6m-none-eabi/release/app
```


# MQTT
```shell
sudo apt install gcc-arm-none-eabi
```
```shell
mosquitto_pub -h 127.0.0.1 -t "arm_bot/commands" -m "b090,s045,e080,g080" -u "armbot" -P "18s=799G"
```

```shell
mosquitto_sub -h 127.0.0.1 -t "arm_bot/commands" -u "armbot" -P "18s=799G"
```

