# TubesAI-GestureReceiver
Untuk keperluan Tugas Besar mata kuliah Artificial Intelligence, Sistem Internet of Things, dan Sistem Tertanam

## Tujuan
- ESP32 scan BLE, connect ke XIAO berdasarkan SERVICE_UUID.
- Subscribe notify pada CHARACTERISTIC_UUID.
- Saat menerima 1 byte:
    - code==1 -> Relay ON, Fan OFF
    - code==2 -> Fan ON, Relay OFF
    - else -> semua OFF

## Hardware
- ESP32 DevKit (WROOM)
- Relay module ke RELAY_PIN
- Fan 12V
- DC Lamp 12V

## Made by 
- Sebastian Cahyaputra 101022300014 
- Muhammad Miqdam Fuadi 101022300106 
- Muhamad Afdhu 101022330171
### Telkom University






