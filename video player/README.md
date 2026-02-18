(english)

# 📺 ESP32-2432S028 MJPEG + Audio Player

## 📌 Project Description

This project demonstrates that the ESP32-2432S028 (CYD 2.8") is capable of playing **MJPEG video and audio simultaneously** with stable synchronization.

Synchronization is verified using real-time **A/V drift measurement**, consistently showing values between **0–1**, with no progressive accumulation over time.

The main goals of this project are:
- Stability
- Precise synchronization
- No crashes
- No freezes
- No progressive drift

---

# 🏗 System Architecture

## 🔹 Video Format
- Format: MJPEG
- Resolution: 320x240
- Frame rate: 15 FPS
- Each frame is an independent JPEG image

## 🔹 Audio Format
- PCM audio
- Output via DAC
- DMA-based playback for stability

## 🔹 System Workflow

1. Read MJPEG frame from SD card
2. Decode JPEG using CPU
3. Send frame to LCD
4. Audio runs in parallel via DMA
5. Synchronization maintained using a shared time reference

---

# 📊 Performance & Testing

## 🎯 Frame Rate

- 15 FPS selected as the stability sweet spot
- Time per frame ≈ 66.6 ms
- JPEG decode + LCD transfer remain within safe timing budget

## 🎵 Audio-Video Synchronization

Test results show:

- A/V drift consistently at 0 or 1
- Drift never exceeds 1
- No cumulative drift over time
- Stable from start to end of playback

## 🔒 System Stability

During testing:

- No crashes
- No resets
- No freezes
- No increasing A/V drift

The system remains deterministic under sustained load.

---

# ❓ FAQ

### Q1: Can ESP32-2432S028 really play video and audio simultaneously?
Yes.

This project proves stable MJPEG + audio playback with measurable synchronization.

---

### Q2: Why use MJPEG instead of MP4 or H.264?
ESP32 does not have a hardware video decoder.

MJPEG is chosen because:
- Each frame is independent
- No inter-frame compression
- More predictable CPU load
- Suitable for software decoding

Formats like MP4/H.264 are too computationally heavy without hardware acceleration.

---

### Q3: Why lock the system at 15 FPS?
15 FPS is the most stable operating point.

Increasing FPS results in:
- Video lagging behind
- Audio running ahead
- Frame drops
- Potential instability

15 FPS provides balance between performance and synchronization.

---

### Q4: What does A/V drift 0–1 mean?
A/V drift is the timing difference between audio playback and video presentation.

General interpretation:
- < 20 ms → unnoticeable
- < 5 ms → very good
- ≤ 1 → highly precise

A consistent 0–1 drift indicates accurate time alignment.

---

### Q5: Is the drift accumulative over time?
No.

Observed behavior:
- Drift never exceeds 1
- Drift does not grow over time
- No progressive desynchronization

This confirms proper timing control.

---

### Q6: Is the internal DAC used?
Yes.

Audio is output via DAC with DMA support to maintain stable playback.

---

### Q7: Is this production-ready?
No.

This is a **technical proof of capability**, not a commercial media player.

The objective is to demonstrate realistic hardware limits when properly engineered.

---

### Q8: Why do many ESP32 projects fail to combine audio and video?
Common reasons:

- FPS set too high
- No shared time reference
- Blocking video decode interfering with audio
- Poor buffer management

This project focuses on timing control and CPU load balancing.

---

### Q9: Can this be ported to other ESP32 boards?
Conceptually, yes.

Performance depends on:
- CPU speed
- Available RAM
- Display driver efficiency
- Audio configuration

Boards like ESP32-S3 with PSRAM may support higher FPS.

---

# 🧠 Technical Conclusion

ESP32-2432S028 can reliably play MJPEG 320x240 @15FPS with simultaneous audio, stable and measurable.

Key success factors:
- Realistic FPS selection
- Proper time management
- Non-blocking audio playback
- Respecting hardware limitations

This project demonstrates that precise synchronization is achievable on constrained hardware with correct system design.

(indonesian)

# 📺 ESP32-2432S028 MJPEG + Audio Player

## 📌 Deskripsi Proyek

Proyek ini membuktikan bahwa board ESP32-2432S028 (CYD 2.8") mampu memutar **video MJPEG dan audio secara bersamaan** dengan sinkronisasi stabil.

Sinkronisasi diuji menggunakan pengukuran **A/V drift secara real-time**, dengan hasil konsisten di **0–1** dan tidak mengalami akumulasi seiring waktu.

Target utama proyek ini adalah:
- Stabilitas
- Sinkronisasi yang presisi
- Tanpa crash
- Tanpa freeze
- Tanpa drift progresif

---

# 🏗 Arsitektur Sistem

## 🔹 Format Video
- Format: MJPEG
- Resolusi: 320x240
- Frame rate: 15 FPS
- Setiap frame adalah JPEG independen

## 🔹 Format Audio
- Audio PCM
- Output via DAC
- Menggunakan DMA untuk kestabilan

## 🔹 Alur Kerja Sistem

1. Baca frame MJPEG dari SD Card
2. Decode JPEG menggunakan CPU
3. Kirim frame ke LCD
4. Audio berjalan paralel melalui DMA
5. Sistem menjaga sinkronisasi menggunakan time reference

---

# 📊 Performa & Pengujian

## 🎯 Frame Rate

- 15 FPS dipilih sebagai titik stabil
- Waktu per frame ≈ 66.6 ms
- Decode + transfer LCD masih dalam batas aman

## 🎵 Sinkronisasi Audio-Video

Hasil pengujian menunjukkan:

- A/V drift konsisten di 0 atau 1
- Tidak pernah melewati 1
- Tidak terjadi akumulasi drift
- Stabil dari awal hingga akhir video

## 🔒 Stabilitas Sistem

Selama pengujian:

- Tidak ada crash
- Tidak ada reset
- Tidak ada freeze
- Tidak ada peningkatan drift seiring waktu

---

# ❓ FAQ

### Q1: Apakah ESP32-2432S028 benar-benar bisa memutar video dan audio bersamaan?
Ya.

Proyek ini membuktikan pemutaran MJPEG + audio berjalan stabil dengan sinkronisasi terukur.

---

### Q2: Mengapa menggunakan MJPEG, bukan MP4 atau H.264?
ESP32 tidak memiliki hardware video decoder.

MJPEG dipilih karena:
- Setiap frame independen
- Tidak ada inter-frame compression
- Beban CPU lebih realistis
- Lebih cocok untuk decoding berbasis CPU

Format seperti MP4/H.264 terlalu berat untuk ESP32 tanpa akselerasi hardware.

---

### Q3: Mengapa dikunci di 15 FPS?
15 FPS adalah titik stabil terbaik.

Jika FPS dinaikkan:
- Video mulai tertinggal
- Audio lebih cepat dari video
- Risiko drop frame
- Risiko ketidakstabilan meningkat

15 FPS memberikan keseimbangan antara performa dan sinkronisasi.

---

### Q4: Apa arti A/V drift 0–1?
A/V drift adalah selisih waktu antara audio dan video.

Interpretasi umum:
- < 20 ms → tidak terasa
- < 5 ms → sangat baik
- ≤ 1 → sangat presisi

Drift 0–1 yang konsisten menunjukkan sistem sinkron dengan baik.

---

### Q5: Apakah drift bertambah seiring waktu?
Tidak.

Drift:
- Tidak pernah melewati 1
- Tidak bertambah
- Tidak akumulatif

Ini menunjukkan sistem memiliki kontrol waktu yang stabil.

---

### Q6: Apakah menggunakan DAC internal?
Ya.

Audio menggunakan DAC dengan bantuan DMA untuk menjaga kestabilan playback.

---

### Q7: Apakah sistem ini production-ready?
Tidak.

Proyek ini adalah **proof of capability teknis**, bukan media player komersial.

Tujuannya adalah menunjukkan batas realistis ESP32 dengan pendekatan yang benar.

---

### Q8: Mengapa banyak proyek ESP32 gagal menggabungkan audio dan video?
Biasanya karena:

- FPS terlalu tinggi
- Tidak ada time reference yang jelas
- Decode video mengganggu audio
- Manajemen buffer tidak optimal

Proyek ini berfokus pada kontrol waktu dan pembagian beban CPU.

---

### Q9: Bisa dipindahkan ke board ESP32 lain?
Secara konsep, bisa.

Namun performa tergantung pada:
- Kecepatan CPU
- RAM yang tersedia
- Driver display
- Konfigurasi audio

Board seperti ESP32-S3 dengan PSRAM berpotensi mendukung FPS lebih tinggi.

---

# 🧠 Kesimpulan Teknis

ESP32-2432S028 mampu memutar MJPEG 320x240 @15FPS dengan audio secara simultan, stabil, dan terukur.

Kunci keberhasilan:
- FPS realistis
- Manajemen waktu yang tepat
- Audio berjalan non-blocking
- Tidak memaksakan performa di luar batas hardware

Proyek ini membuktikan bahwa sinkronisasi presisi dapat dicapai di hardware terbatas dengan pendekatan sistem yang benar.