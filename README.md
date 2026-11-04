# TurtleBot3 Otonom Sürüş, SLAM Haritalama ve Sınır Keşfi (ROS 2 Humble)

Bu proje, **ROS 2 Humble** ve **Gazebo** simülasyon ortamında TurtleBot3 Waffle robotu kullanılarak geliştirilmiş kapsamlı bir mobil robotik otonom sürüş, SLAM haritalama ve otonom sınır keşfi (Frontier Exploration) sistemidir.

---

## 📌 Temel Özellikler

1. **3 Farklı Özgün Test Dünyası (8m x 8m Arena):**
   * **Senaryo 1 (Kolay):** Geniş koridorlar, merkezi silindir ve temel engeller.
   * **Senaryo 2 (Yoğun / Slalom):** 18 adet dar aralıklı (0.75m - 0.85m) statik engel içeren slalom parkuru.
   * **Senaryo 3 (5 Odalı Ev Planı):** Salon, mutfak, koridor, yatak odaları ve banyo içeren gerçekçi iç mekan kat planı.

2. **Geliştirilmiş 2D LiDAR Modeli (8 Metre, 10 Hz):**
   * Varsayılan 3.5m'lik sensör menzili 8 metreye, frekansı 10 Hz'e yükseltilerek SLAM haritalamadaki açısal kayma (drift) ve çift duvar hataları kalıcı olarak engellenmiştir.

3. **Otonom Hedefe Sürüş Hattı (`navigator`):**
   * Nav2 ve DWB yerel planlayıcısı ile hedefe doğrudan otonom sürüş.
   * Dinamik metrik kaydı: Süre, kat edilen toplam yol, ortalama hız ve en yakın engel mesafesi.

4. **Otonom Sınır Keşfi (Frontier Exploration - `kesif`):**
   * BFS (Breadth-First Search) tabanlı sınır tespit algoritması ile haritanın bilinmeyen bölgelerini insan müdahalesiz %100 otonom keşfeder.
   * Keşif tamamlandığında haritayı otomatik olarak `.pgm` ve `.yaml` formatında diske kaydeder.

5. **Akıllı Manuel Kontrol (`teleop`):**
   * W, A, S, D tuşlarıyla klavye kontrolü.
   * **Ön Çarpışma Önleyici Fren:** Robotun önünde 25 cm'den yakın engel varken ileri sürüşü otomatik engeller.
   * **Acil Durdurma (`x`):** Manuel tuş bırakıldığında robot anında durur. `x` tuşuna basıldığında arka plandaki otonom Nav2 planlaması anında iptal edilir ve robot kilitlenir (`r` ile devam ettirilebilir).

6. **Canlı Robot Telemetrisi (`subscriber`):**
   * Robotun anlık konumunu $(X, Y)$, yönünü, çizgisel/açısal hızını ve en yakın engelin yön/mesafesini canlı konsol arayüzünde görüntüler.

7. **Otomatik Sonuç Kaydı:**
   * Tüm deney sonuçları `deney_sonuclari.txt` dosyasına otomatik eklenir.

---

## 🛠️ Gereksinimler ve Kurulum

* **İşletim Sistemi:** Ubuntu 22.04 LTS
* **ROS Dağıtımı:** ROS 2 Humble
* **Gerekli Paketler:**
  ```bash
  sudo apt update
  sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    terminator python3-yaml
  ```

### Çalışma Alanını Derleme

```bash
cd ~/Masaüstü/ros2_deneme
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Çalıştırma

Tüm sistemi 4 pencereli terminal düzeni ile yönetebilirsiniz:

```
┌──────────────────────────────────────┬──────────────────────────────────────┐
│  1. [ROS 2 Publisher] (Sol Üst)      │  2. [ROS 2 Subscriber] (Sağ Üst)     │
│  Manuel Klavye Kontrolü (teleop)     │  Canlı Telemetri & Engel Monitörü    │
├──────────────────────────────────────┼──────────────────────────────────────┤
│  3. [gazebo] (Sol Alt)               │  4. [Ros2 Komutları] (Sağ Alt)        │
│  Gazebo + SLAM + Nav2 + RViz2        │  Otonom Sürüş / Keşif Komutları      │
└──────────────────────────────────────┴──────────────────────────────────────┘
```

### 1. Adım: Simülasyonu Başlatın (Sol Alt)
```bash
# Senaryo 1 için:
ros2 launch otonom_surus turtlebot3_control.launch.py senaryo:=1

# Senaryo 2 için:
ros2 launch otonom_surus turtlebot3_control.launch.py senaryo:=2

# Senaryo 3 için:
ros2 launch otonom_surus turtlebot3_control.launch.py senaryo:=3
```

### 2. Adım: Canlı Telemetriyi Başlatın (Sağ Üst)
```bash
ros2 run otonom_surus subscriber
```

### 3. Adım: Sürüş Modunu Seçin

* **A) Otonom Hedefe Sürüş (Sağ Alt):**
  ```bash
  ros2 run otonom_surus navigator --ros-args -p senaryo:=1
  ```

* **B) Tüm Haritayı Otonom Keşfetme (Sağ Alt):**
  ```bash
  ros2 run otonom_surus kesif --ros-args -p senaryo:=1
  ```

* **C) Manuel Klavye Sürüşü (Sol Üst):**
  ```bash
  ros2 run otonom_surus teleop
  ```

---

## 📂 Dizin Yapısı

```
ros2_deneme/
├── src/
│   └── otonom_surus/
│       ├── config/
│       │   ├── nav2_params.yaml       # DWB ve Costmap parametreleri
│       │   └── scenarios.yaml         # Senaryo başlangıç ve hedef koordinatları
│       ├── launch/
│       │   └── turtlebot3_control.launch.py  # Ana başlatma dosyası
│       ├── maps/                      # Otonom keşifle çıkarılan .pgm/.yaml haritaları
│       ├── models/                    # Arena duvarları, hedef işareti ve waffle modeli
│       ├── otonom_surus/
│       │   ├── otonom_navigator.py    # Hedefe otonom gidiş ve metrik takibi
│       │   ├── otonom_kesif.py        # Otonom sınır keşfi (Frontier exploration)
│       │   ├── subscriber.py          # Canlı telemetri düğümü
│       │   └── teleop.py              # Akıllı klavye kontrolü ve acil durdurma
│       ├── rviz/
│       │   └── otonom_surus.rviz      # Hazır RViz2 yapılandırması
│       └── worlds/
│           ├── senaryo1_kolay.world   # 1. Test dünyası
│           ├── senaryo2_yogun.world   # 2. Test dünyası
│           └── senaryo3_ev.world      # 3. Test dünyası
├── deney_sonuclari.txt                # Otomatik kaydedilen test raporları
└── README.md
```

---

## 📜 Lisans & Akademik Beyan
Bu proje, Robotik Sistemler Dönem Projesi kapsamında eğitim ve araştırma amacıyla geliştirilmiştir.
