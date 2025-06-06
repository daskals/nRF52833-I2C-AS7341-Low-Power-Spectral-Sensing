# nRF52833-I2C-AS7341-Low-Power-Spectral-Sensing

Low-power firmware for Nordic Semiconductor's nRF52833 DK to interface with the AS7341 multi-channel spectral sensor over I2C. Designed for outdoor applications such as PAR (Photosynthetically Active Radiation) monitoring and spectral analysis.

## 🛠 Hardware

- **MCU**: nRF52833 DK  
- **Sensor**: AS7341 (AMS)  
- **Interface**: I2C (3.3V logic)  
- **Use Case**: Outdoor spectral measurement / PAR sensing  

## ⚙️ Sensor Configuration

- **Gain**: `AS7341_GAIN_1X`
- **ATIME**: 35  
- **ASTEP**: 999  
- **Total Integration Time**: 100 ms  
- **Channels**: F1–F8, Clear, NIR  

## 🔋 Low Power Strategy

1. Initialize I2C bus  
2. Wake up AS7341 sensor  
3. Configure measurement registers  
4. Acquire spectral data  
5. Shutdown or disconnect I2C pins (high-Z)  

## 📂 Directory Structure

```
firmware/
├── as7341.c / as7341.h          # Sensor driver
├── as7341_defines.h             # Register definitions
├── i2c_interface.c / .h         # TWI abstraction using nRF SDK
├── main.c                       # Measurement logic and logging
├── sdk_config.h                 # SDK configuration
docs/
└── AS7341_datasheet.pdf         # Optional datasheet
```

## 📊 Example Output

| Channel | Value |
|---------|-------|
| F1      | 122   |
| F2      | 147   |
| ...     | ...   |
| NIR     | 135   |

## 📚 References

- [AS7341 Datasheet – AMS](https://ams.com/as7341)
- [nRF52833 DK – Nordic Semiconductor](https://www.nordicsemi.com/Products/nRF52833)
- Based on: [`nRF52833-I2C-SPI-PCAP04-Low-Power-Capacitive-Sensing`](https://github.com/daskals/nRF52833-I2C-SPI-PCAP04-Low-Power-Capacitive-Sensing)

## 📜 License

MIT License
