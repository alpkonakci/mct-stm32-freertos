# MCT FreeRTOS STM32 Firmware

[Türkçe](#türkçe) | [English](#english)

## Türkçe

Bu depo, ortak 500 kbit/s CAN hattında çalışan üç STM32F103C8T6 düğümünün FreeRTOS tabanlı firmware projelerini içerir:

- `Motor_Control`: DRV8876 motor sürücüsü, akım koruması ve motor telemetrisi
- `Sensor_Control`: DS18B20 sıcaklık, piezo titreşim ve sensör telemetrisi
- `Bridge_Control`: Raspberry Pi UART ile CAN ağı arasında çift yönlü köprü, alarm LED'leri ve buzzer

Üç düğüm aynı CAN hattına paralel bağlanır. Bridge üzerinde ikinci bir CAN çevrebirimi veya ikinci CAN pin çifti kullanılmaz.

## Sistem Mimarisi

```text
Raspberry Pi / USB-TTL
        |
   UART 115200 8N1
        |
Bridge STM32 + CAN transceiver
        |
 CANH / CANL, 500 kbit/s
        +---------------- Sensor STM32 + transceiver
        |
        +---------------- Motor STM32 + transceiver
```

CAN hattının iki fiziksel ucunda 120 ohm sonlandırma bulunmalıdır. Güç kapalıyken CANH-CANL arasında yaklaşık 60 ohm ölçülmesi beklenir. Bütün düğümlerin GND referansı ortak olmalıdır.

## Dizinler

Her STM32CubeIDE projesinde esas uygulama dosyası `Core/Src/main.c` dosyasıdır.

```text
mct-stm32-freertos/
|-- Motor_Control/
|-- Bridge_Control/
|-- Sensor_Control/
`-- README.md
```

## Ortak Saat ve CAN Ayarları

| Ayar | Değer |
|---|---:|
| MCU | STM32F103C8T6 |
| HSE | 8 MHz |
| SYSCLK | 72 MHz |
| APB1 / PCLK1 | 36 MHz |
| CAN modu | Normal |
| Prescaler | 4 |
| BS1 | 13 TQ |
| BS2 | 4 TQ |
| SJW | 1 TQ |
| CAN hızı | 500 kbit/s |

Hesap: `36 MHz / (4 * (1 + 13 + 4)) = 500 kbit/s`.

CAN pinleri:

- `PA11`: CAN1_RX
- `PA12`: CAN1_TX
- CAN remap kapalıdır.
- `USB low priority or CAN RX0` kesmesi öncelik 5 ile açıktır.
- `USB high priority or CAN TX` kesmesi öncelik 6 ile açıktır.

FreeRTOS kullanan kesmeler, `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5` sınırına uygun olarak 5 veya daha düşük mantıksal öncelikte çalışır.

## CAN Kimlikleri

| ID | Yön | Kaynak / Hedef | İçerik |
|---:|---|---|---|
| `0x101` | RX | Motor | Motor komutu |
| `0x181` | TX | Motor | ACK / NAK |
| `0x201` | TX | Motor | Temel telemetri |
| `0x203` | TX | Motor | Hız / PWM telemetrisi |
| `0x204` | TX | Motor | Çalışma süresi / sayaçlar |
| `0x211` | TX | Motor | Durum |
| `0x221` | TX | Motor | Hata olayı |
| `0x102` | RX | Sensör | Sensör komutu |
| `0x182` | TX | Sensör | ACK / NAK |
| `0x202` | TX | Sensör | Sıcaklık / titreşim telemetrisi |
| `0x212` | TX | Sensör | Durum |
| `0x222` | TX | Sensör | Hata olayı |

Kimlikler klasik firmware ile aynıdır. RTOS'a geçiş UART-CAN protokolünü değiştirmez.

## Motor Control

### Bağlantılar

| STM32 pini | İşlev |
|---|---|
| `PB1 / TIM3_CH4` | DRV8876 EN/IN1 PWM |
| `PB10` | DRV8876 PH/IN2 yön |
| `PB11` | DRV8876 nSLEEP |
| `PA6` | DRV8876 nFAULT, giriş pull-up |
| `PA3 / ADC1_IN3` | DRV8876 IPROPI akım ölçümü |
| `PA11/PA12` | CAN RX/TX |

DRV8876, PMODE düşük olacak şekilde PH/EN modunda kullanılmaktadır. EN girişindeki PWM hızı, PH seviyesi yönü belirler. `TIM3_CH4` ayarları: prescaler 0, period 3599, PWM mode 1, active high.

### RTOS görevleri

| Görev | Öncelik | Stack | Sorumluluk |
|---|---:|---:|---|
| `MotorCtrl` | 5 | 320 word | Komutlar, 5 ms güvenlik döngüsü, PWM ve fault state |
| `CanTx` | 4 | 256 word | CAN TX kuyruğu, timeout ve yeniden başlatma |
| `Telemetry` | 2 | 192 word | 500 ms periyotlu telemetri |

Motor açılışta ileri yönde yüzde 80 hedef hız ile başlar. `U` ve `N` komutları hedef PWM'i 20 puan artırır veya azaltır. Sınırlar yüzde 0 ve yüzde 100'dür.

### Motor komutları

| Komut | İşlem |
|---|---|
| `A` | İleri başlat; opsiyonel byte 1 hız yüzdesi |
| `B` | Geri başlat; opsiyonel byte 1 hız yüzdesi |
| `X` | Frenleyerek durdur |
| `D` | Acil durdur / remote fault kilitle |
| `R` | Hataları temizle, motoru durur halde bırak, hedefi yüzde 80 yap |
| `P,<0..100>` | Mutlak hız yüzdesi |
| `U` veya `+` | Hızı 20 puan artır |
| `N` veya `-` | Hızı 20 puan azalt |
| `I` / `S` | Durum ve telemetri iste |

`A` veya `B`, yalnızca remote emergency fault durumunu temizleyip motoru yeniden başlatabilir. Aşırı akım, nFAULT, sıcaklık veya titreşim hataları önce `R` ile temizlenmelidir.

### Motor güvenlik sınırları

- Akım `500 mA` veya üzerindeyse ve durum `1000 ms` sürerse motor durur.
- Sensör telemetrisi `40.00 C` üzerindeyse motor durur.
- Titreşim yüzde `80` veya üzerindeyse motor durur.
- `PA6 nFAULT` düşük seviyesi 50 ms debounce sonrasında motoru durdurur.

Motor üzerinde enkoder bulunmadığı için gerçek RPM ölçülmez. `0x203` çerçevesi PWM/hız yüzdesini taşır. Yüzdeden hesaplanan RPM yalnızca tahmindir; gerçek RPM için enkoder veya Hall sensörü gerekir.

`0x203` payload:

| Byte | Değer |
|---:|---|
| 0 | Uygulanan hız yüzdesi |
| 1 | Hedef hız yüzdesi |
| 2 | PWM duty yüzdesi |
| 3 | Yön: 0 durdu, 1 ileri, 2 geri |
| 4-5 | TIM3 compare, little-endian |
| 6 | Motor state |
| 7 | Rezerve |

## Sensor Control

### Bağlantılar

| STM32 pini | İşlev |
|---|---|
| `PA1` | DS18B20 1-Wire, open-drain |
| `PA4 / ADC1_IN4` | Piezo titreşim girişi |
| `PA11/PA12` | CAN RX/TX |

DS18B20 DATA ile 3.3 V arasına harici `4.7 kohm` pull-up bağlanmalıdır. PA1 CubeMX'te `GPIO_Output`, open-drain, başlangıç high ve dahili pull olmadan ayarlanır.

### RTOS görevleri

| Görev | Öncelik | Stack | Sorumluluk |
|---|---:|---:|---|
| `SensorCtrl` | 5 | 320 word | Komutlar, piezo örnekleme ve sensör state |
| `CanTx` | 4 | 256 word | CAN TX kuyruğu ve yeniden deneme |
| `Temperature` | 3 | 320 word | Bloklamayan DS18B20 dönüşüm akışı |
| `Telemetry` | 2 | 160 word | 500 ms telemetri olayı |

Piezo 1 ms aralıkla örneklenir. DS18B20 dönüşümü için 800 ms beklenir ve scratchpad CRC kontrol edilir. En son geçerli sıcaklık doğrudan telemetriye aktarılır; güvenlik kararını geciktiren hareketli ortalama kullanılmaz. Arka arkaya 10 okuma hatası sensör fault üretir.

`0x202` payload:

| Byte | Değer |
|---:|---|
| 0-1 | Sıcaklık x100, signed int16 little-endian |
| 2 | Titreşim yüzdesi |
| 3 | Sistem state: 0 normal, 1 warning, 2 error |
| 4 | Genel sensör sağlığı: 1 normal, 0 hata |
| 5 | Stream açık bayrağı |
| 6 | Test modu bayrağı |
| 7 | Fault kodu |

Sensör komutları: `A` stream aç, `X` stream kapat, `D` remote fault, `R` reset, `T` test, `F` flush ACK, `I/S` durum isteği.

## Bridge Control

### Bağlantılar

| STM32 pini | İşlev |
|---|---|
| `PA9` | USART1_TX |
| `PA10` | USART1_RX |
| `PA11/PA12` | CAN RX/TX |
| `PA8` | Aktif buzzer çıkışı |
| `PB12` | Yeşil LED |
| `PB14` | Kırmızı LED |
| `PC13` | Aktivite LED'i |

UART ayarı `115200, 8N1`, flow control kapalıdır.

### RTOS görevleri

| Görev | Öncelik | Stack | Sorumluluk |
|---|---:|---:|---|
| `BridgeCtrl` | 5 | 384 word | CAN/UART olayları ve alarm state |
| `CanTx` | 4 | 256 word | CAN gönderim kuyruğu |
| `UartParse` | 3 | 384 word | UART stream buffer ve frame parser |
| `UartTx` | 2 | 256 word | Sıralı UART çıkışı |
| `Heartbeat` | 1 | 160 word | 1 saniyelik heartbeat |

UART komut biçimi:

```text
<TX,101,1,41>      Motor ileri başlat
<TX,101,1,42>      Motor geri başlat
<TX,101,1,55>      Motor hız artır (U)
<TX,101,1,4E>      Motor hız azalt (N)
<TX,101,1,58>      Motor durdur (X)
<TX,101,1,44>      Acil durdur (D)
<TX,101,1,52>      Motor fault reset (R), motor durur
<TX,102,1,41>      Sensör stream aç
<PING>
<STAT>
```

CAN'dan alınan veri UART'a şu biçimde çıkar:

```text
<CAN,202,8,C409120001010000>
<HB,1000,1,rx_count,tx_count,error,tec,rec,lec>
```

`D` komutu CAN-UART veri akışını durdurur; heartbeat devam eder. `R`, `A` veya `B` komutu veri akışını yeniden açar ve bridge alarmını temizler. `R` motoru çalıştırmaz. `A/B` motoru çalıştırır.

Bridge alarmı:

- Acil durdur komutu, motor fault, sensör fault veya yüksek titreşim alarmı kırmızı LED ve buzzer'ı açar.
- Alarm aktifken yeşil LED söner.
- Sağlıklı motor status çerçevesi yalnızca motor-fault kaynaklı alarmı temizleyebilir; sensör ve acil durdur alarmlarını yanlışlıkla temizlemez.

## FreeRTOS ve CubeMX

Tüm projelerde:

- FreeRTOS arayüzü `CMSIS_V1` seçilidir; uygulama native FreeRTOS API kullanır.
- `configMAX_PRIORITIES = 6`
- Tick rate `1000 Hz`
- Preemption açıktır.
- Bellek tahsisi statiktir.
- Stack overflow kontrolü `Option2` olarak açıktır.
- Malloc failed hook açıktır.
- HAL timebase `TIM2`, interrupt priority 15'tir.

Uygulama görevleri ve RTOS nesneleri `main.c` içinde statik oluşturulur. Bu nedenle CubeMX'in ürettiği aşağıdaki dosyalar Debug ve Release yapılandırmalarında build dışındadır:

- `Core/Src/freertos.c`
- `Core/Src/stm32f1xx_it.c`
- `Core/Src/stm32f1xx_hal_msp.c`
- `Core/Src/stm32f1xx_hal_timebase_tim.c`

Bu dört `.c` dosyasındaki işlevlerin karşılıkları özel `main.c` içinde bulunduğu için dışlamalar kaldırılırsa duplicate symbol/linker hataları oluşabilir. CubeMX tarafından üretilen işlevler yeniden kullanılacaksa exclusion listesi de yeni mimariye göre değiştirilmelidir.

`Core/Inc/stm32f1xx_hal_conf.h` bağımsız olarak derlenen bir kaynak dosyası değildir; HAL kaynakları tarafından `#include` edilir. Bu nedenle bazı CubeIDE yapılandırmalarında dosya Project Explorer'da "Exclude from Build" olarak işaretli görünse bile proje derlenebilir. Önemli olan dosyanın `Core/Inc` içinde bulunması, include path üzerinden erişilebilmesi ve gerekli `HAL_*_MODULE_ENABLED` tanımlarını içermesidir. Dosyayı fiziksel olarak silmek, yeniden adlandırmak veya include path'ten çıkarmak HAL derlemesini bozar.

CubeMX yeniden kod ürettiğinde `.cproject` exclusion ayarlarını ve `Core/Src/main.c` içeriğini tekrar kontrol edin.

## Derleme

1. İlgili `.project` veya `.cproject` projesini STM32CubeIDE ile import edin.
2. `Project > Clean` ardından `Build Project` çalıştırın.
3. Build sonunda doğru projeye ait `.elf`, `.hex` veya `.bin` çıktısını kullanın.

## Firmware Yükleme

ST-Link zorunlu değildir. Firmware aşağıdaki iki yöntemden biriyle yüklenebilir.

### Yöntem 1: ST-Link

1. `SWDIO`, `SWCLK`, `GND` ve gerekirse `3.3V` bağlantılarını yapın.
2. STM32CubeIDE veya STM32CubeProgrammer ile doğru `.elf`, `.hex` ya da `.bin` dosyasını seçin.
3. Programlama sonrasında kartı resetleyin.

### Yöntem 2: USB-TTL ve STM32CubeProgrammer

USB-TTL dönüştürücü 3.3 V TTL lojik seviyesinde çalışmalıdır.

| USB-TTL | STM32F103C8T6 |
|---|---|
| TX | `PA10 / USART1_RX` |
| RX | `PA9 / USART1_TX` |
| GND | GND |

1. Kartın enerjisini kesin.
2. `BOOT0` pinini `1` konumuna alın. Blue Pill üzerinde `BOOT1/PB2` normal olarak `0` kalır.
3. USB-TTL TX'i PA10'a, RX'i PA9'a ve GND'yi ortak GND'ye bağlayın.
4. Kartı yeniden enerjilendirin veya resetleyin; STM32 sistem bootloader'ı açılır.
5. STM32CubeProgrammer'da bağlantı tipini `UART` seçip doğru COM portuna bağlanın.
6. `.hex` dosyasını doğrudan seçin. Ham `.bin` dosyası için başlangıç adresini `0x08000000` olarak ayarlayın.
7. `Download` ile firmware'i yazın ve doğrulamayı tamamlayın.
8. Enerjiyi kesin, `BOOT0` pinini tekrar `0` konumuna alın ve kartı resetleyin.

Bridge kartındaki PA9/PA10 çalışma sırasında Raspberry Pi veya USB-TTL ile kullanıldığı için programlama sırasında aynı hatta iki farklı TX çıkışı bağlanmamalıdır. Gerekirse Pi UART bağlantısını geçici olarak ayırın.

Her MCU'ya kendi proje firmware'i yüklenmelidir. Bridge firmware'ini motor veya sensör kartına yüklemeyin.

## Hızlı Doğrulama

1. Sadece bridge açıkken UART'ta her saniye heartbeat görünmelidir.
2. Motor bağlandığında `0x201`, `0x203`, `0x204` çerçeveleri yaklaşık 500 ms aralıkla görünmelidir.
3. Sensör bağlandığında `0x202` yaklaşık 500 ms aralıkla görünmelidir.
4. `<TX,101,1,55>` sonrası `0x181` ACK ve güncel `0x203` beklenir.
5. `<TX,101,1,4E>` sonrası hız yüzdesi düşmelidir.
6. `<TX,101,1,44>` sonrası motor durmalı, buzzer ve kırmızı LED açılmalı, yalnız heartbeat kalmalıdır.
7. `<TX,101,1,52>` sonrası alarm kapanmalı fakat motor başlamamalıdır.
8. `<TX,101,1,41>` sonrası motor başlamalı ve CAN-UART akışı devam etmelidir.

## Sorun Giderme

- Heartbeat var, CAN yok: transceiver enable/standby pini, ortak GND, CANH/CANL, sonlandırma ve 500 kbit/s ayarını kontrol edin.
- CAN `REC` artıyor ve `LEC=5`: bit timing, ters CANH/CANL, gürültü veya transceiver seviyelerini kontrol edin.
- Komut gidiyor, ACK yok: hedef ID'nin motor için `0x101`, sensör için `0x102` olduğundan emin olun.
- DS18B20 okunmuyor: PA1 ile 3.3 V arasında harici 4.7 kohm pull-up ve ortak GND kullanın.
- Motor akımı sürekli sıfır: IPROPI'nin PA3'e ve RIPROPI direncinin GND'ye bağlı olduğunu kontrol edin.
- Motor hemen fault oluyor: nFAULT bağlantısının PA6'da ve pull-up giriş olarak yapılandırıldığını doğrulayın.

## Kaynak Sözleşmesi

CAN ID'leri ve payload düzenleri bridge, motor, sensör ve Raspberry Pi gateway arasında ortak sözleşmedir. Bir çerçeve değiştirilecekse bütün tüketiciler birlikte güncellenmeli ve UART çıktısı tekrar doğrulanmalıdır.

---

## English

This repository contains FreeRTOS-based firmware projects for three STM32F103C8T6 nodes operating on one shared 500 kbit/s CAN bus:

- `Motor_Control`: DRV8876 motor control, current protection, and motor telemetry
- `Sensor_Control`: DS18B20 temperature, piezo vibration, and sensor telemetry
- `Bridge_Control`: bidirectional Raspberry Pi UART-to-CAN bridge, alarm LEDs, and buzzer

All nodes are connected in parallel to the same CANH/CANL pair. The bridge does not require a second CAN peripheral or a second CAN pin pair.

### System Architecture

```text
Raspberry Pi / USB-TTL
        |
   UART 115200 8N1
        |
Bridge STM32 + CAN transceiver
        |
 CANH / CANL, 500 kbit/s
        +---------------- Sensor STM32 + transceiver
        |
        +---------------- Motor STM32 + transceiver
```

Place one 120-ohm termination resistor at each physical end of the CAN bus. With power removed, approximately 60 ohms should be measured between CANH and CANL. All nodes must share a common ground reference.

### Project Layout

The main application implementation for each STM32CubeIDE project is in `Core/Src/main.c`.

```text
mct-stm32-freertos/
|-- Motor_Control/
|-- Bridge_Control/
|-- Sensor_Control/
`-- README.md
```

### Common Clock and CAN Configuration

| Setting | Value |
|---|---:|
| MCU | STM32F103C8T6 |
| HSE | 8 MHz |
| SYSCLK | 72 MHz |
| APB1 / PCLK1 | 36 MHz |
| CAN mode | Normal |
| Prescaler | 4 |
| BS1 | 13 TQ |
| BS2 | 4 TQ |
| SJW | 1 TQ |
| CAN bitrate | 500 kbit/s |

Bitrate calculation: `36 MHz / (4 * (1 + 13 + 4)) = 500 kbit/s`.

Common CAN pins and interrupts:

- `PA11`: CAN1_RX
- `PA12`: CAN1_TX
- CAN remapping is disabled.
- `USB low priority or CAN RX0` is enabled at interrupt priority 5.
- `USB high priority or CAN TX` is enabled at interrupt priority 6.
- The HAL time base uses TIM2 at interrupt priority 15.

Interrupts that call FreeRTOS APIs use priorities compatible with `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`.

### CAN Identifier Contract

| ID | Direction | Node | Purpose |
|---:|---|---|---|
| `0x101` | RX | Motor | Motor command |
| `0x181` | TX | Motor | ACK / NAK |
| `0x201` | TX | Motor | Basic telemetry |
| `0x203` | TX | Motor | Speed / PWM telemetry |
| `0x204` | TX | Motor | Runtime and counters |
| `0x211` | TX | Motor | Status |
| `0x221` | TX | Motor | Fault event |
| `0x102` | RX | Sensor | Sensor command |
| `0x182` | TX | Sensor | ACK / NAK |
| `0x202` | TX | Sensor | Temperature / vibration telemetry |
| `0x212` | TX | Sensor | Status |
| `0x222` | TX | Sensor | Fault event |

The RTOS conversion does not change the existing UART-CAN protocol or CAN identifiers.

### Motor Control

#### Connections

| STM32 pin | Function |
|---|---|
| `PB1 / TIM3_CH4` | DRV8876 EN/IN1 PWM |
| `PB10` | DRV8876 PH/IN2 direction |
| `PB11` | DRV8876 nSLEEP |
| `PA6` | DRV8876 nFAULT, pull-up input |
| `PA3 / ADC1_IN3` | DRV8876 IPROPI current measurement |
| `PA11/PA12` | CAN RX/TX |

The DRV8876 is used in PH/EN mode with PMODE held low. PWM on EN controls the duty cycle and PH selects direction. TIM3_CH4 uses prescaler 0, period 3599, PWM mode 1, active-high polarity.

#### Tasks

| Task | Priority | Stack | Responsibility |
|---|---:|---:|---|
| `MotorCtrl` | 5 | 320 words | Commands, 5 ms safety loop, PWM, and fault state |
| `CanTx` | 4 | 256 words | CAN TX queue, timeout, and bus recovery |
| `Telemetry` | 2 | 192 words | 500 ms periodic telemetry |

The motor starts forward at an 80 percent target duty cycle after reset. `U` and `N` change the target by 20 percentage points, clamped to 0-100 percent.

| Command | Action |
|---|---|
| `A` | Start forward; optional byte 1 sets speed percent |
| `B` | Start reverse; optional byte 1 sets speed percent |
| `X` | Brake and stop |
| `D` | Emergency stop and latch remote fault |
| `R` | Clear faults, remain stopped, restore 80 percent target |
| `P,<0..100>` | Set absolute speed percentage |
| `U` or `+` | Increase speed by 20 points |
| `N` or `-` | Decrease speed by 20 points |
| `I` / `S` | Request status and telemetry |

Safety behavior:

- Stop when current remains at or above 500 mA for 1000 ms.
- Stop when sensor telemetry reports a temperature above 40.00 C.
- Stop when vibration reaches or exceeds 80 percent.
- Stop when PA6 nFAULT remains low after the 50 ms debounce interval.

There is no encoder in the current hardware, so the firmware does not measure actual RPM. Frame `0x203` reports PWM and speed percentages. RPM calculated from duty cycle is only an estimate; measured RPM requires an encoder or Hall sensor.

`0x203` payload:

| Byte | Value |
|---:|---|
| 0 | Applied speed percentage |
| 1 | Target speed percentage |
| 2 | PWM duty percentage |
| 3 | Direction: 0 stopped, 1 forward, 2 reverse |
| 4-5 | TIM3 compare value, little-endian |
| 6 | Motor state |
| 7 | Reserved |

### Sensor Control

#### Connections

| STM32 pin | Function |
|---|---|
| `PA1` | DS18B20 1-Wire, open-drain |
| `PA4 / ADC1_IN4` | Piezo vibration input |
| `PA11/PA12` | CAN RX/TX |

Connect an external 4.7 kohm pull-up resistor between DS18B20 DATA and 3.3 V. Configure PA1 as open-drain output, initial high, with no internal pull resistor.

#### Tasks

| Task | Priority | Stack | Responsibility |
|---|---:|---:|---|
| `SensorCtrl` | 5 | 320 words | Commands, piezo sampling, and sensor state |
| `CanTx` | 4 | 256 words | CAN TX queue and retries |
| `Temperature` | 3 | 320 words | Non-blocking DS18B20 conversion sequence |
| `Telemetry` | 2 | 160 words | 500 ms telemetry events |

The piezo input is sampled every 1 ms. DS18B20 conversion time is 800 ms and scratchpad CRC is validated. The latest valid temperature is published directly so that the motor's 40 C safety decision is not delayed by a moving average. Ten consecutive read failures latch a temperature sensor fault.

`0x202` payload:

| Byte | Value |
|---:|---|
| 0-1 | Temperature x100, signed int16 little-endian |
| 2 | Vibration percentage |
| 3 | System state: 0 normal, 1 warning, 2 error |
| 4 | Overall health: 1 normal, 0 fault |
| 5 | Stream-enabled flag |
| 6 | Test-mode flag |
| 7 | Fault code |

Sensor commands are `A` stream on, `X` stream off, `D` remote fault, `R` reset, `T` test, `F` flush ACK, and `I/S` status request.

### Bridge Control

#### Connections

| STM32 pin | Function |
|---|---|
| `PA9` | USART1_TX |
| `PA10` | USART1_RX |
| `PA11/PA12` | CAN RX/TX |
| `PA8` | Active buzzer output |
| `PB12` | Green LED |
| `PB14` | Red LED |
| `PC13` | Activity LED |

UART is configured as 115200 baud, 8 data bits, no parity, one stop bit, and no hardware flow control.

#### Tasks

| Task | Priority | Stack | Responsibility |
|---|---:|---:|---|
| `BridgeCtrl` | 5 | 384 words | CAN/UART events and alarm state |
| `CanTx` | 4 | 256 words | CAN transmission queue |
| `UartParse` | 3 | 384 words | UART stream buffer and frame parser |
| `UartTx` | 2 | 256 words | Serialized UART output |
| `Heartbeat` | 1 | 160 words | One-second heartbeat |

UART examples:

```text
<TX,101,1,41>      Start motor forward
<TX,101,1,42>      Start motor reverse
<TX,101,1,55>      Increase motor speed (U)
<TX,101,1,4E>      Decrease motor speed (N)
<TX,101,1,58>      Stop motor (X)
<TX,101,1,44>      Emergency stop (D)
<TX,101,1,52>      Reset motor fault (R), remain stopped
<TX,102,1,41>      Enable sensor stream
<PING>
<STAT>
```

CAN-to-UART output examples:

```text
<CAN,202,8,C409120001010000>
<HB,1000,1,rx_count,tx_count,error,tec,rec,lec>
```

Command `D` disables CAN-to-UART frame output while heartbeat remains active. Commands `R`, `A`, and `B` restore frame output and clear the bridge alarm. `R` does not start the motor; `A/B` start it.

Emergency stop, motor fault, sensor fault, or high vibration turns on the red LED and active buzzer and turns off the green LED. A healthy motor status frame can clear only a motor-fault alarm; it cannot incorrectly clear an emergency-stop or sensor alarm.

### FreeRTOS and CubeMX Notes

All three projects use:

- FreeRTOS `CMSIS_V1` interface with native FreeRTOS APIs in the application
- `configMAX_PRIORITIES = 6`
- 1000 Hz tick rate and preemption enabled
- Static task and object allocation
- Stack overflow check `Option2`
- Malloc-failed hook enabled
- TIM2 HAL time base at interrupt priority 15

Tasks, queues, stream buffers, interrupt handlers, MSP initialization, and the HAL time base are implemented in the custom `main.c`. Therefore the following CubeMX-generated source files are excluded from both Debug and Release builds:

- `Core/Src/freertos.c`
- `Core/Src/stm32f1xx_it.c`
- `Core/Src/stm32f1xx_hal_msp.c`
- `Core/Src/stm32f1xx_hal_timebase_tim.c`

Removing these four exclusions while keeping the matching custom implementations can produce duplicate-symbol linker errors. If the architecture is changed to use CubeMX-generated implementations, update the exclusion list accordingly.

`Core/Inc/stm32f1xx_hal_conf.h` is a header included by HAL sources, not an independently compiled translation unit. Some CubeIDE configurations may display it as "Exclude from Build" without breaking compilation. What matters is that the file still exists under `Core/Inc`, remains reachable through the include path, and contains the required `HAL_*_MODULE_ENABLED` definitions. Physically deleting, renaming, or removing it from the include path will break the HAL build.

After CubeMX code regeneration, recheck `.cproject` exclusions and the custom `Core/Src/main.c`.

### Build

1. Import the relevant `.project` or `.cproject` into STM32CubeIDE.
2. Run `Project > Clean`, then `Build Project`.
3. Use the `.elf`, `.hex`, or `.bin` generated for the correct node.

### Firmware Programming

ST-Link is optional. Either method below can be used.

#### Method 1: ST-Link

1. Connect SWDIO, SWCLK, GND, and optionally 3.3 V.
2. Select the correct `.elf`, `.hex`, or `.bin` in STM32CubeIDE or STM32CubeProgrammer.
3. Program and reset the target.

#### Method 2: USB-TTL with STM32CubeProgrammer

Use a USB-TTL adapter with 3.3 V logic levels.

| USB-TTL | STM32F103C8T6 |
|---|---|
| TX | `PA10 / USART1_RX` |
| RX | `PA9 / USART1_TX` |
| GND | GND |

1. Remove power from the target.
2. Set `BOOT0` to `1`. On a Blue Pill, `BOOT1/PB2` normally remains `0`.
3. Cross-connect USB-TTL TX to PA10 and RX to PA9, and connect common GND.
4. Power or reset the board to enter the STM32 system-memory bootloader.
5. Select `UART` and the correct COM port in STM32CubeProgrammer, then connect.
6. Select a `.hex` file directly. For a raw `.bin`, set the start address to `0x08000000`.
7. Run `Download` and complete verification.
8. Remove power, return `BOOT0` to `0`, and reset the board to run the application.

PA9/PA10 on the bridge are also used for the normal Raspberry Pi UART link. Do not drive the same UART line from two TX outputs while programming; temporarily disconnect the Pi UART if necessary.

Each MCU must receive its matching firmware. Do not flash bridge firmware onto the motor or sensor board.

### Quick Verification

1. With only the bridge running, a heartbeat should appear on UART every second.
2. The motor should produce `0x201`, `0x203`, and `0x204` frames approximately every 500 ms.
3. The sensor should produce `0x202` approximately every 500 ms.
4. `<TX,101,1,55>` should produce a `0x181` ACK and an updated `0x203` frame.
5. `<TX,101,1,4E>` should reduce the reported speed percentage.
6. `<TX,101,1,44>` should stop the motor, enable the red LED and buzzer, and leave heartbeat output active.
7. `<TX,101,1,52>` should clear the alarm but leave the motor stopped.
8. `<TX,101,1,41>` should start the motor and restore CAN-to-UART output.

### Troubleshooting

- Heartbeat works but no CAN frames: check transceiver enable/standby, common ground, CANH/CANL, termination, and 500 kbit/s timing.
- CAN REC increases with `LEC=5`: check bitrate, swapped CANH/CANL, noise, transceiver state, and wiring.
- Command is transmitted but no ACK: use ID `0x101` for motor commands and `0x102` for sensor commands.
- DS18B20 is not detected: install the external 4.7 kohm pull-up from PA1/DATA to 3.3 V and verify common ground.
- Motor current remains zero: verify IPROPI to PA3 and the RIPROPI resistor to ground.
- Motor immediately faults: verify that nFAULT is connected to PA6 and configured as a pull-up input.

### Protocol Contract

CAN identifiers and payload layouts form a shared contract between bridge, motor, sensor, and Raspberry Pi gateway software. When a frame layout changes, update every producer and consumer together and revalidate UART output.
