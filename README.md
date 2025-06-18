
# Afet Durumlarında İHA'ların Etkin Kullanımı - Simülasyon Projesi

Bu proje, afet bölgelerinde insansız hava araçlarının (İHA) daha etkili kullanılabilmesi için çeşitli sürü algoritmalarını karşılaştırmalı olarak analiz eden bir simülasyon sistemini içermektedir.

## Proje Amacı

Afet bölgelerinde hızlı ve etkili tarama yapılabilmesi amacıyla farklı optimizasyon algoritmalarının (PSO, ACO, SMA, GWO, WOA, CO) karşılaştırılması ve simülasyon ortamında performanslarının değerlendirilmesi hedeflenmiştir.

## Kullanılan Teknolojiler

- Python 3.x
- ROS 2 Humble
- PyQt5 - Simülasyon kontrol arayüzü
- RViz2 - Görsel simülasyon çıktısı
- Colcon - ROS 2 paket derleyicisi
- Gazebo (opsiyonel) - Gelişmiş simülasyon ortamı


## Simülasyonu Başlatma

### GUI Üzerinden Çalıştırmak İçin
Kod içinde gereken dosya yolları değiştirilmelidir!
```bash
cd path
python3 ~/path/sim_gui_final.py
```



> Not: RViz görselleştirmesi otomatik olarak başlatılabilir veya `rviz2` komutu ile manuel açılabilir.

## Kayıtlar & Sonuçlar

Her algoritma çalıştırıldığında kapsama oranı ve çalışma süresi `/tmp/coverage_result.txt` veya `data/output.csv` dosyasına yazılır.

| Algoritma | Kapsama (%) | Süre (sn) |
|-----------|--------------|-----------|
| PSO       | 91.3         | 0.9       |
| ACO       | 88.5         | 1.2       |
| SMA       | 67.2         | 0.6       |
| GWO       | 65.0         | 0.6       |

## Notlar

- Formasyon tipleri (Delta, V, Grid, Line) başlangıç dizilimini etkiler.
- Her algoritma birden fazla iterasyonda çalıştırılabilir.
- Senaryo üretimi için GUI üzerinden parametre kombinasyonları verilebilir (örn: 3,5,7 İHA sayısı, 1,2 algoritma kodları vb.)

## Katkı Sağlayanlar

- Ubeydullah Ak
- Umut Engin Berkkan

## Lisans

Bu proje Marmara Üniversitesi Bitirme Projesi kapsamında hazırlanmıştır. Akademik kullanım için uygundur.
