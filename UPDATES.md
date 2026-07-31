# Update Log

Bu dosya, projede yapılan hata düzeltmelerini ve eksik parça tamamlamalarını
tarih sırasıyla listeler. Her madde: dosya, sorun, ve yapılan düzeltme.

Doğrulama: `pio run -e nucleo_h743zi` ve `pio run -e nucleo_h753zi` derleniyor,
`test/build_and_run_tests.bat` içindeki 55 host-side unit testin tamamı geçiyor.

---

## 2026-08-01

### RCC (saat/register) offset hataları — sistemin hiç açılmamasına yol açacak seviyede

Register offsetleri, resmi STMicroelectronics CMSIS başlığı
(`stm32h753xx.h`, `RCC_TypeDef`) referans alınarak doğrulandı. `CR`,
`D1CFGR`/`D2CFGR`/`D3CFGR` ve `AHB4ENR` dışındaki neredeyse tüm RCC
register adresleri, gerçek silikonda başka bir register'a ya da rezerve
alana denk geliyordu.

- **`src/drivers/system_clock.cpp`**
  - `RCC_CFGR` `0x08` → `0x10`, `RCC_PLLCKSELR` `0x0C` → `0x28`,
    `RCC_PLLCFGR` `0x10` → `0x2C`, `RCC_PLL1DIVR` `0x14` → `0x30`,
    `RCC_BDCR` `0xA0` → `0x70` (bu adres ayrıca `uart_driver.cpp`/`pwm_driver.cpp`
    içindeki `APB2ENR` ile çakışıyordu).
  - `PWR_D3CR` adresi `0x58024808` → `0x58024818` (gerçek `PWR_TypeDef.D3CR`
    ofseti `0x18`, `0x08` değil).
  - `configurePll()`: `PLLSRC` alanı `PLLCKSELR` bit[1:0]'da olması
    gerekirken yanlışlıkla `PLLCFGR` bit0'a yazılıyordu; `DIVM1` bit[9:4]
    yerine bit[3:0]'a yazılıyordu; `DIVP1EN`/`DIVQ1EN`/`DIVR1EN` (bit
    16/17/18) hiç set edilmiyordu — yani PLL kilitlense bile P/Q/R çıkışı
    hiç üretilmiyordu; `PLL1DIVR` R alanı bit24 yerine bit25'e yazılıyordu.
  - `init()`: `CFGR.SWS` (clock-switch-status) bit[5:3]'te iken kod
    bit[3:2]'yi okuyordu — saat kaynağı PLL1'e geçse bile bu kontrol asla
    doğrulanamıyor, her boot'ta ~1.000.000 iterasyonluk gereksiz bekleme
    zaman aşımına uğruyordu.
  - `configureLse()`: `LSERDY` biti bit1 iken kod bit2'yi kontrol ediyordu.
  - Bus prescaler alanları (`D1CFGR`/`D2CFGR`/`D3CFGR`) yanlış bit
    pozisyonlarındaydı; ayrıca `D1CPRE=/2` yazılıyordu ki bu, fonksiyonun
    sonunda iddia edilen 400 MHz CPU hedefiyle çelişiyordu (D1CPRE=/2 ile
    CPU 200 MHz'de kalırdı). `D1CPRE=/1` olacak şekilde düzeltildi.
  - `PWR_D3CR` VOS alanı bit[15:14] yerine bit[1:0]'da işleniyordu; hazır
    olma kontrolü seçim bitlerini değil `VOSRDY` (bit13) bayrağını
    okumalıydı — düzeltildi.

- **`src/drivers/uart_driver.cpp`**: `RCC_APB1LENR` `0x60`→`0xE8`,
  `RCC_APB2ENR` `0xA0`→`0xF0` (adresler yanlıştı; bit pozisyonları zaten
  doğruydu).

- **`src/drivers/pwm_driver.cpp`**: aynı `APB1LENR`/`APB2ENR` adres
  düzeltmesi.

- **`src/drivers/i2c_driver.cpp`**: `APB1LENR` adresi düzeltildi;
  `APB4ENR` adresi `0xE4` (gerçekte `APB3ENR`) → `0xF4` düzeltildi;
  I2C1/2/3 enable biti hesaplanırken `20 + instance - 1` formülü tüm
  instance'ları bir bit kaydırıyordu (I2C1 gerçek bit21 yerine bit20'ye
  yazılıyordu) → `20 + instance` olarak düzeltildi; I2C4 enable biti
  bit4 yerine gerçek konumu olan bit7'ye taşındı.

### `src/drivers/adc_driver.cpp` — ADC hiç çalışmıyordu

- `ADC123_COMMON` adresi `0x40022100` olarak tanımlanmıştı; bu adres
  aslında `ADC2_BASE`'in kendisi — VBAT/VSENSE seçimi ADC2'nin kendi
  `IER` register'ına yazılıyordu. Gerçek ortak blok `ADC1_BASE + 0x300`
  (`0x40022300`) olacak şekilde düzeltildi.
- ADC1/2/3 saat enable bitleri `RCC_AHB4ENR` bit5/bit6 olarak
  tanımlanmıştı — bu bitler gerçekte GPIOF/GPIOG saat enable'ları
  (`gpio_driver.cpp`'teki `GpioPort::PortF/PortG` ile birebir çakışıyor).
  Gerçek konum: ADC1/2 için `RCC_AHB1ENR` bit5, ADC3 için `RCC_AHB4ENR`
  bit24. Düzeltildi; sonuç olarak ADC hiçbir zaman saat almıyordu ve
  `init()` içindeki `ADRDY` bekleme döngüsü sonsuza kadar takılı kalırdı.

### `src/drivers/pwm_driver.cpp` — motor kanalı 2 ve 4 hiç PWM üretmiyordu

`CCMR1`/`CCMR2` register'larının sadece alt baytı (kanal 1 / kanal 3)
yapılandırılıyor, üst bayt (kanal 2 / kanal 4) reset değeri olan
"Frozen" modda kalıyordu. `setAllOutputs()` dört motoru da sürdüğü için
2. ve 4. motor asla PWM sinyali almıyordu. Her iki register'ın hem alt
hem üst baytı aynı bit düzeniyle yazılacak şekilde düzeltildi.

### `src/components/pid_controller.cpp` — türev teriminin işareti ters

`output = pTerm + iTerm - dTerm` yazılmıştı; standart PID formülünde
`output = pTerm + iTerm + dTerm` olmalı. Ters işaret, türev teriminin
sönümleme yerine osilasyonu güçlendirmesine yol açıyordu (uçuş
kontrolcüsünde roll/pitch/yaw PID'leri bunu kullanıyor). Düzeltildi.

### `src/components/error_manager.cpp` — `clearError` çift çağrıda taşıyordu

`clearError()`, kaydın `active` olup olmadığına bakmadan
`errorCount_--` yapıyordu; art arda iki kez çağrıldığında (ya da zaten
pasif bir hata için çağrıldığında) `uint32_t errorCount_` alt taşmaya
uğrayıp ~4 milyara sarıyordu — bu durumda `hasError()`/`isSystemHealthy()`
kalıcı olarak arıza raporluyordu. Ayrıca kodda taşmayı önlemeye çalışan
`if (errorCount_ < 0)` kontrolü, `errorCount_` işaretsiz olduğu için asla
tetiklenemezdi (dead code). `record->active` kontrolü eklendi.

### `src/drivers/imu_sensor.cpp` — sahte IMU'da paylaşımlı static durum

`MockImuSensor::read()` içinde fonksiyon-lokal `static float angle`
kullanılıyordu; birden fazla `MockImuSensor` örneği oluşturulursa hepsi
aynı açıyı paylaşırdı. `angle_` artık sınıfın bir üyesi
(`include/drivers/imu_sensor.hpp`).

### `src/drivers/stm32_pwm_timer.cpp` + `pwm_motor_driver.cpp` — placeholder, hiç PWM üretmiyordu

`Stm32PwmTimer::init()`/`setDutyCycle()` tamamen boş stub'dı (yorum:
"Placeholder for real STM32 timer configuration"). `Stm32PwmMotorDriver`
bunu kullandığından, gerçek donanımda motorlar hiçbir zaman sinyal
almıyordu. `Stm32PwmTimer` artık zaten doğru olan register-seviyeli
`PwmDriver`'ı sarmalıyor. Ayrıca `Stm32PwmMotorDriver::setOutputs()` her
çağrıda yeni bir `Stm32PwmTimer` oluşturup `init()` çağırıyordu (donanımı
kontrol döngüsü hızında yeniden başlatıyordu); artık `timer_` bir üye
olarak tek seferlik `init()` ile kullanılıyor.

### `test/build_and_run_tests.bat` — test paketi hiç derlenmiyordu

`SOURCES` listesinde `test_framework.cpp` ve `mock_system_clock.cpp`
eksikti; script `test::testCount` vb. sembollerde linker hatasıyla
çöküyordu. İkisi de listeye eklendi. Şu an 55/55 test geçiyor.

---

## Bu oturumdan önce zaten uygulanmış ve doğrulanmış düzeltmeler

Aşağıdakiler bu incelemeden önce repoda commit edilmemiş halde
mevcuttu; bu oturumda gözden geçirilip doğru olduğu teyit edildi
(ayrıca değiştirilmedi):

- `include/drivers/i2c_driver.hpp`: I2C1/2/3 taban adresleri düzeltildi
  (`0x40012000`/`13000`/`14000` → gerçek `0x40005400`/`5800`/`5C00`).
- `src/components/command_receiver.cpp`: CRC doğrulaması başarısız olan
  paketin yine de komut olarak çalıştırılması engellendi; CRC byte sırası
  (little-endian) düzeltildi; `reinterpret_cast` ile strict-aliasing ihlali
  yerine `memcpy` kullanıldı.
- `src/components/flight_controller.cpp`: `altVelocity` fonksiyon-lokal
  `static`'ten sınıf üyesine taşındı (çoklu örnekte paylaşılan durum
  hatası).
- `src/drivers/mpu6050_driver.cpp`: `constexpr` üyenin adresi alınarak
  odr-use ihlali yapılması yerine yerel kopya kullanıldı.
- `src/drivers/system_clock.cpp`: HSE/PLL başlatma başarısız olduğunda
  bile SysTick'in yapılandırılması sağlandı (aksi halde failsafe
  zamanlayıcısı hiç çalışmazdı); `HSI_VALUE` sabiti eklendi.
- `src/app/drone_app.cpp`: eksik `drivers::` namespace niteleyicileri ve
  `MotorMixer` çıktı alanı isimleri (`m1..m4` → `frontLeft` vb.) düzeltildi.
- `src/system_stm32h7xx.c`: gerçek `SystemClock_Config` ile çakışan
  placeholder tanım kaldırıldı (çoklu tanım / link hatası riski).
