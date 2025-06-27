# FreeRTOS-Test-Example-2

# Amaç
Bu proje, STM32 platformu üzerinde CMSIS-RTOS v2 kullanılarak geliştirilmiş basit bir gerçek zamanlı işletim sistemi (RTOS) uygulamasıdır. Amacı, task oluşturma, öncelik belirleme, zamanlama kontrolü ve temel çevre birimleri (ADC, UART, GPIO) kullanımı gibi temel RTOS kavramlarını test etmektir.

Projede eşzamanlı çalışan üç görev bulunmaktadır:

IR Görevi: Analog IR sensöründen veri okur.

DHT11 Görevi: Sıcaklık ve nem sensörü olan DHT11’den sıcaklık verisi okur.

Sistem Görevi: Gelen verileri yorumlayarak LED kontrolü ve UART üzerinden mesaj gönderimi yapar.

# RTOS Görev Yapısı

| Görev Adı     | Öncelik      | İşlev                                                                 | Periyot  |
|---------------|--------------|-----------------------------------------------------------------------|----------|
| IR_Task       | Above Normal | ADC üzerinden IR sensör verisini okur ve global değişkende saklar     | 100 ms   |
| DHT11_Task    | Below Normal | DHT11 üzerinden sıcaklık ve nem verilerini okur                       | 2000 ms  |
| System_Task   | Normal       | LED’i kontrol eder ve sıcaklık verisine göre UART mesajı gönderir     | 50 ms    |
    

# Notlar
Bu proje, öğrenme ve deneme amaçlı yazılmıştır. Gerçek uygulamalar için watchdog, error handling ve bellek optimizasyonu gibi ek güvenlik önlemleri gereklidir.


