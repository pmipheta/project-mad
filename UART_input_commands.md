# ฟังก์ชัน input ผ่าน USART3 (uant แทนปุ่มจริง)

สรุปสิ่งที่เพิ่มใน `Core/Src/main.c` — ระบบ input ที่ทำตาม simulator (Python) แต่รับคำสั่งทาง **USART3**
เพราะยังไม่ได้ต่อปุ่มจริง (ปุ่ม PB0–PB3 ในโค้ดเดิมยังอยู่ และทำหน้าที่เหมือนคีย์ UART ทุกอย่าง)

---

## 1. การต่อสาย / พอร์ต

| | |
|---|---|
| ค่าที่ตั้งไว้ | **USART3 · 115200 · 8N1 · ไม่มี flow control** |
| ขา | ผมย้าย USART3 ไปขา **PD8/PD9** (Virtual COM ของ ST-LINK) → ใช้**สาย USB เส้นที่ใช้แฟลชได้เลย** |
| พอร์ตบน Windows | `COM4` (STMicroelectronics STLink Virtual COM Port) |
| โปรแกรมฝั่ง PC | Tera Term / PuTTY / Arduino Serial Monitor — เลือก COM4, 115200 |

> โปรเจกต์ตั้ง USART3 ไว้ที่ PB10/PB11 (ไม่ใช่ VCP) การย้ายขาอยู่ใน `UartInputInit()`
> แบบ **ไม่แตะ `usart.c`** (ปลอดภัยเวลา CubeMX generate ใหม่)
> ถ้าจะกลับไปใช้ PB10/PB11 + USB-TTL adapter → คอมเมนต์บล็อก `HAL_GPIO_DeInit(GPIOB, ...)` / `HAL_GPIO_Init(GPIOD, ...)` ในฟังก์ชันนั้นทิ้ง แล้วต่อ TX(adapter)→PB11, RX(adapter)→PB10, GND↔GND
> ถ้าอยากให้ถาวรใน CubeMX: `USART3 → Mode: Asynchronous, Pins: PD8/PD9`

## 2. คีย์ (พิมพ์แล้วทำงานทันที ไม่ต้องกด Enter)

| คีย์ | เหมือนปุ่ม | ทำอะไร |
|---|---|---|
| `s` | SET | เลือก setting ถัดไป (FILTER → TRIG → V/DIV → T/DIV) |
| `u` | UP | เพิ่มค่าของ setting ที่เลือก |
| `d` | DOWN | ลดค่าของ setting ที่เลือก |
| `h` | HOLD | freeze / ปลด freeze (ตอน HOLD จะมีเส้น cursor C1/C2) |
| `?` | — | พิมพ์สถานะทั้งหมด (เหมือน panel ของ simulator) |
| `g` | Generate Data | สุ่มข้อมูลใหม่ |

## 3. คำสั่งแบบมีค่า (พิมพ์แล้ว Enter)

```
w SIN | w SQ | w TRI | w SAW | w 0..3     เปลี่ยนชนิดคลื่น (ไม่สนตัวพิมพ์) / w เฉย ๆ = วนถัดไป
f 1000          ความถี่ 100–5000 Hz            f          = +100 Hz
a 0.50          แอมพลิจูด 0.10–1.50 V          a          = +0.05 V
o 1.65          DC offset 0.50–2.50 V          o          = +0.05 V
n 0.05          noise (V, SD) 0–0.30 V         n          = +0.005 V
v + | v -       เลือก V/DIV (0.1 0.2 0.5 1.0 2.0 V)
t + | t -       เลือก T/DIV (0.1 0.2 0.5 1.0 2.0 ms)
c1 40 | c2 160  ตำแหน่ง cursor (จำนวน sample)
cal 0 | cal 1   ปิด/เปิด AC coupling (calibrate offset)
```

## 4. setting ทำอะไรกับสัญญาณ (เหมือน simulator)

| setting | ค่า | ผล |
|---|---|---|
| **FILTER** | 1,3,5…21 | moving average filter ต่อจาก AC coupling — เห็นในหน้าจอเป็น "Det" |
| **TRIG** | RISE / FALL | เปลี่ยนเป็นขอบตก (แบบ simulator ซึ่งไม่มี AUTO/NORM/SNGL แล้ว) |
| **V/DIV** | 0.1–2.0 V | ย่อ/ขยายแนวตั้ง |
| **T/DIV** | 0.1–2.0 ms | ความกว้างเวลา 1 จอ = T/div × 10 ช่อง ⇒ Fs = 240 / (T/div×10) → 120k…12k Sa/s |
| **C1/C2** (ตอน HOLD) | 0…239 | เลื่อน cursor ทีละ 5 sample, แถวล่างโชว์ U1·U2·ΔV·Δt |

Generator ไล่ตามเวลาจริงต่อเนื่อง + roll mode (ข้อมูลใหม่เข้าขวา เส้นไถลซ้าย ~8 sample/เฟรม) เหมือนที่ทำไว้ก่อนหน้า

## 5. ค่าที่พิมพ์ออกหน้าจอ serial (หลังสั่ง '?')

```
== OSCILLOSCOPE =============================
 Setting: FILT:5
  > FILT:5
    TRIG:RISE
    VDIV:1.0
    TDIV:1.0
 HOLD:OFF  CAL:AC  Det:SIN
== FUNCTION GENERATOR (input) ================
 Wave:SINE  Freq:1000 Hz
 Amp:0.500 V  DC:1.650 V  Noise:0.050 V
 V/div:3 (1.0 V)  T/div:3 (1.0 ms)  Fs:24 kSa/s
== MEASUREMENTS ==============================
 Vmax:0.513 V   Vmin:-0.506 V
 Vpp: 1.020 V   RMS: 0.332 V
 DC:  0.000 V   Freq: 1000 Hz
 Calib offset:2047.445 cts   Dev:0.554 cts
----------------------------------------------
 keys: s=SET u=UP d=DOWN h=HOLD g=gen ?=status
```

## 6. ยืนยันแล้วบนบอร์ดจริง

สั่งจาก PC → อ่านค่าตัวแปรใน RAM ผ่าน SWD (hotplug) เทียบ:

| สั่งไป | ค่าที่อ่านได้จริง |
|---|---|
| `w SAW` / `f 500` / `a 1.00` / `o 1.60` / `n 0.05` | `gen_wave=3` · `gen_freq=500.0` · `gen_amp=1.0` · `gen_dc=1.60` · `gen_noise=0.05` |
| `t -` (T/DIV 0.5 ms) | `osc_tdiv_i=2` · `osc_fs_eff=48000` |
| `d` ×2 (FILTER) | `osc_fwin=1` |
| `h` `s` `u` `h` (C2 +5) | `osc_c1=40` · `osc_c2=165` · `hold_active=0` |
| `cal 1` | `osc_calib=1` · `osc_offset_ct=2061.9` → +0.0112 V |
| คลื่น SAW | `osc_det="TRI/SAW"` |

Δt ที่คำนวณ: (165−40)/24000 = 5.208 ms — ตรงกับที่หน้าจอโชว์

## 7. ป้าย T/div บนจอ กับค่าที่ใช้จริง (แก้แล้ว)

เดิมมี **สองตัวแปรสำหรับเรื่องเดียวกัน** ทำให้จอโกหกตัวเอง:

| ตัวแปร | ใช้ทำอะไร | ค่าเริ่มต้น |
|---|---|---|
| `osc_tdiv_i` | คำนวณหน้าต่างเวลา / sample rate จริง | 3 → 1.0 ms |
| `timediv_idx` | พิมพ์ป้าย `T:x ms/d` บนจอ | 1 → 0.2 ms |

ค่าเริ่มต้นไม่ตรงกัน → จอโชว์ **T:0.2ms/d** แต่ข้อมูลจริงกินเวลา **10 ms**
เวลาตั้งความถี่ เช่น 600 Hz จะเห็น 6 รอบในจอ = ถ้าอ่านตามป้ายจะเหมือน ~3 kHz ⇒ **"ความถี่จอกับที่ตั้งไม่ตรงกัน"**

แก้โดยตัด `timediv_idx` / `timediv_labels[]` ออก แล้วให้ป้ายพิมพ์จาก `osc_tdivs_ms[osc_tdiv_i]` (แหล่งเดียว)

ตรวจสอบหลังแก้ (อ่านบัฟเฟอร์ที่วาดจริง `osc_v[]` ผ่าน SWD แล้วนับรอบข้ามศูนย์):

| | |
|---|---|
| `osc_tdiv_i` | 3 → ป้ายบนจอ **T:1.0ms/d** |
| หน้าต่างเวลา | 1.0 ms × 10 ช่อง = **10 ms** |
| `osc_fs_eff` | 24000 Sa/s (= 240 / 10 ms ✓) |
| นับรอบในข้อมูลที่วาด | **10 รอบ / 10 ms** → 999.867 Hz (ตั้ง 1000 Hz, ต่าง 0.01%) |
| firmware วัดเอง | 1000.6 Hz ✓ |

## 8. โหมดกราฟนิ่ง (จัดแนว trigger) / โหมดไถล — คีย์ `m`

| | |
|---|---|
| **โหมดนิ่ง (ค่าเริ่มต้น)** | จัดแนวด้วย trigger — ใช้จุดตัดศูนย์ของสัญญาณที่สอบเทียบแล้ว (AC coupling) เป็นระดับ trigger ⇒ **auto level = กลางสัญญาณ** และใช้ขอบตามที่ตั้งในเมนู `TRIG` (RISE/FALL) |
| **โหมดไถล (roll)** | ไม่จัดแนว ข้อมูลใหม่เข้าขวา เส้นไถลซ้าย (เหมือน real-time monitor) |

สลับได้ตอนรัน — **กด `m`** หรือพิมพ์ `m a` (นิ่ง) / `m r` (ไถล) / `m 1` / `m 0`

**วิธีพิมพ์ (สำคัญ):** `m` และ `u` มีทั้งแบบ "คีย์เดี่ยว" และแบบ "มีค่า" ⇒ firmware จะ **รอให้หยุดพิมพ์ 250 ms** แล้วจึงทำเป็นคีย์เดี่ยว (ไม่ต้องกด Enter) — ถ้าจะใช้แบบมีค่าให้พิมพ์ต่อทันทีแล้วกด Enter: `m a` · `m r` · `ma` · `mr` · `m 1` · `m 0` · `u 2`
(คีย์ `s` `d` `h` `g` `?` ไม่มีรูปแบบมีค่า จึงทำทันทีเมื่อกด — ไม่ต้อง Enter)

บนจอ LCD จะบอกโหมดที่บรรทัดล่าง: `AUTO:R` / `AUTO:F` / `ROLL` · ในหน้ารายงาน serial จะเห็น `Trig:AUTO-RISE`

จุดที่ต่างจากเดิม: `FindTrigger()` เดิมหาจาก `raw_samples[]` เทียบ `ADC_MID` (1.65 V ตรึง) และเป็นขอบขึ้นอย่างเดียว
⇒ ถ้าตั้ง DC offset ให้สัญญาณไม่อยู่กลาง 1.65 V จุด trigger จะไม่ใช่กลางสัญญาณ ส่วนเวอร์ชันใหม่หาจากข้อมูลที่สอบเทียบแล้ว (แกว่งรอบ 0) และรองรับ FALL

### ตรวจสอบบนบอร์ดจริง (SWD)
| ทดสอบ | ผล |
|---|---|
| TRIG=RISE | `dbg_trig_off=22` · `v[21]=-0.1244 → v[22]=+0.0072` = ผ่าน 0 **ขึ้น** ✓ และเท่ากับจุดตัดศูนย์แรกที่คำนวณเองจากบัฟเฟอร์ ✓ |
| TRIG=FALL (เขียน `osc_trig_fall=1` ใน RAM ผ่าน SWD = เทียบเท่ากด `d` ที่เมนู TRIG) | `dbg_trig_off=3` · `v[2]=+0.0245 → v[3]=-0.0895` = ผ่าน 0 **ลง** ✓ เท่ากับที่คำนวณเอง ✓ |
| คืนค่า RISE | `dbg_trig_off=7` · ผ่าน 0 ขึ้น ✓ |
| ความนิ่งของภาพ (อ่านบัฟเฟอร์ที่วาด 2 เฟรม ห่างกัน 0.5 วิ) | สหสัมพันธ์แบบ **จัดแนว = 0.960** (รูปอยู่ที่เดิม) เทียบกับแบบไม่จัดแนว = **−0.500** (รูปเลื่อนไป 1/3 คาบ) |

หมายเหตุ: จุด trigger จะ snap ลงบนตัวอย่าง (sample) ที่ใกล้ที่สุด ⇒ เหลือ jitter ต่ำกว่า 1 ตัวอย่าง (≈1.3 px บนจอ 320 px) ถ้าต้องการนิ่งกว่านี้อีกให้ interpolate จุดตัดศูนย์ (บอกได้)

## 9. Auto scale — คีย์ `z` (จัด V/DIV + T/DIV ให้รูปเต็มจอ)

กด **`z`** ครั้งเดียว (ทำงานทันที ไม่ต้อง Enter) ระบบจะ:
1. สแกน T/DIV 0.1 → 2.0 ms เพื่อหาว่า time base ไหน **วัดแอมพลิจูดได้ครบที่สุด** (ถี่สูงจะถูกกรองที่ base ช้า / ถี่ต่ำจะเหลือไม่ถึงคาบที่ base เร็ว)
2. เลือก **T/DIV** ให้เห็นประมาณ 2.5 คาบบนจอ (โหมด function generator ใช้ความถี่ที่ตั้งไว้เป็นหลัก)
3. เลือก **V/DIV** จาก Vpp ที่วัดได้ (ให้น้อยสุดแต่รูปไม่ล้นจอ) — `OSC_V_FILL_DIVS = 5.4` ช่อง
   (ใช้ 3-4 เฟรม/ขั้น กันค่าชั่วขณะหลังสลับเวลา — ใช้เวลารวม ~2-3 วิ)

| ตั้งไว้ | ผลที่ auto scale เลือก | หมายเหตุ |
|---|---|---|
| SINE 5 kHz (T/div 2.0 ms อยู่) | T/DIV 0.1 ms (5 คาบ) · V/DIV 0.5 | เคส "ความถี่สูงจนแน่น" ที่ต้องการ |
| SINE 1 kHz | T/DIV 0.2 ms (2 คาบ) · V/DIV 0.5 | |
| SINE 100 Hz | T/DIV 2.0 ms (2 คาบ) · V/DIV 0.5 | ถี่ต่ำ (เดิมเลือกผิดเพราะหน้าต่างสั้นกว่าคาบ) |
| SINE 1 kHz amp 0.1 | V/DIV 0.1 | สัญญาณเล็ก |
| SINE 2 kHz amp 1.5 | V/DIV 1.0 | สัญญาณใหญ่ ไม่ล้นจอ |

ข้อจำกัด: รายการ V/DIV หยาบ (0.1/0.2/0.5/1/2) + ค่าที่วัดมักเกินจริง 5-8% ⇒ รูปอาจได้ 2-5 ช่อง
(ไม่ล้นจอ) ปรับต่อได้ด้วย `s` เลือก V/DIV แล้ว `u`/`d`

## 10. Indicator บนจอ LCD ว่ากำลังตั้งค่าอะไร

- **แถบบรรทัดล่างสุดของจอ**: `SET FILT=5  UP/DOWN +-2` · `SET TRIG=RISE  UP/DOWN TOGGLE` · `SET VDIV=1.0V  UP/DOWN STEP` · `SET TDIV=1.0ms  UP/DOWN STEP` (HOLD: `SET C1=40 …`) — ตัวดำบนแถบฟ้า วาดใหม่ทุกเฟรม
- **มุมขวาบน**: ช่อง setting เป็นตัวดำบนแถบส้ม = "ช่องที่ถูกเลือกอยู่"
- **ป้าย `V:..` / `T:..`** เปลี่ยนเป็นสี MAGENTA เมื่อเป็นตัวที่ UP/DOWN กำลังแก้
- ตัวเลข fps (debug) ย้ายไป x=200 ให้พ้นแถบ

## 11. ความทนทานของ input (แก้แล้ว)

- `m` และ `u` มีทั้งแบบคีย์เดี่ยวและแบบมีค่า ⇒ รอหยุดพิมพ์ **250 ms** (`UART_IDLE_MS`) จึงทำเป็นคีย์เดี่ยว; พิมพ์มีค่าได้ทั้ง `m a`/`ma`
- บรรทัดค้างที่ไม่มี `\r` จะถูกทิ้งหลังเงียบ **4 วิ** (`UART_DROP_MS`) — ไม่งั้น `uart_len` ไม่กลับเป็น 0 แล้วคีย์เดี่ยวทุกตัวจะหยุดทำงานถาวร
- `HAL_UART_Transmit` timeout 15 ms (เดิม 300 ms) — ถ้าโปรแกรมฝั่ง PC ไม่ยอมอ่านพอร์ต ลูปหลัก/จอจะค้างได้ถึง ~12 วิ
- ถ้าพิมพ์ค้างแล้วอยากให้ทำงานใหม่: เคาะ Enter 1 ครั้ง (หรือรอ 4 วิ) แล้วพิมพ์ใหม่

## 12. อินพุตจริงจากเครื่องกำเนิดสัญญาณ (ADC) — merge จาก MAD/project_2

สวิตช์เดียวใน `main.c`: `#define ADC_INPUT_ENABLE 1`
- **1 = ใช้ข้อมูลจริงจาก ADC** (ค่าเริ่มต้นตอนนี้) · **0 = กลับไปใช้ simulator เดิม** (GenFrame/SimSamplingStream — โค้ดเดิมอยู่ครบ ไม่ได้ลบ)

| หัวข้อ | ค่า |
|---|---|
| ขาที่ต่อสัญญาณ | **PA0 = ADC1_IN0** (ต้องเลือกพอต/ขานี้ใน CubeIDE pinout เพื่อยืนยัน) |
| ย่านอินพุต | **0 – 3.3 V เท่านั้น** (12-bit) — ห้ามติดลบ; ถ้าเครื่องกำเนิดจ่าย ±V ต้อง AC coupling + bias 1.65 V หรือ divider |
| อัตราสุ่มจริงที่วัดได้ | **791,766 Sa/s** (ADC clock = PCLK2/4 = 12 MHz, sampling 3 cycles) |
| วิธีได้ time base | `step = rate/fs_want` แล้ว **resample แบบเศษ** (เฉลี่ยหน้าต่าง w จุด) → ไม่ต้อง reprogram timer |
| แกนเวลา | `osc_fs_eff = rate/step` → มาจาก "อัตราสุ่มจริงที่วัด" ไม่ใช่ค่าที่ตั้ง (แหล่งความจริงเดียว) |

ตรวจบนบอร์ดจริง (SWD) — ทุกช่วงตรงกับที่ตั้งไว้ 0.00%:

| T/DIV | step (จุดดิบ/จุด) | fs_eff | เวลาจับ/เฟรม |
|---|---|---|---|
| 0.1 ms | 3.30 | 240,000 Sa/s | 1.02 ms |
| 0.2 ms | 6.60 | 120,000 Sa/s | 2.02 ms |
| 0.5 ms | 16.50 | 48,000 Sa/s | 5.00 ms |
| 1.0 ms | 32.99 | 24,000 Sa/s | 9.97 ms |
| 2.0 ms | 65.98 | 12,000 Sa/s | 19.91 ms |

อ่านค่าตรวจสอบผ่าน SWD: `dbg_adc_ok` · `dbg_adc_rate` · `dbg_adc_step` · `dbg_adc_n` · `dbg_adc_us` · `dbg_adc_min/max` · `dbg_adc_frames`

พร้อมใช้: `s`(SET) `u/d` `h` `m` `z`(auto scale) ใช้ได้กับสัญญาณจริงเหมือนเดิม · auto scale จะ**วัดความถี่จริง** (ไม่ใช้ `gen_freq` แล้วในโหมดนี้)

## 13. แก้บั๊ก: ความถี่ต่ำกว่า ~1000 Hz แล้วกราฟพัง (auto scale + trigger)

**อาการ**: ตั้งความถี่ต่ำ (เช่น 100-500 Hz) แล้วกด `z` → กราฟขาด/กระโดด และ auto scale เลือก T/DIV ผิด

**สาเหตุ (3 ชั้น)**
1. **การวัดความถี่ที่หน้าต่างสั้นกว่าหนึ่งคาบใช้ไม่ได้** — ที่ T/DIV เร็วสุด หน้าต่าง 240 จุดสั้นกว่าคาบของสัญญาณความถี่ต่ำ → เหลือเพียงเศษคลื่น + noise → อ่านได้ขยะ (บางครั้ง 0 หรือหลาย kHz)
2. **ตัวทดสอบ "มีคาบหรือยัง"** (`OscWindowHasCycle` เทียบผลต่าง 5 จุด) ถูก noise หลอก → ไม่ยอมช้าลง time base → ตัดสินที่ base ผิด
3. **โค้ดเดิมมีค่า fallback `f = 1000.0f`** (ค่าเริ่มต้นที่ค้างมาจากเดิม) → พอวัดความถี่ไม่ได้ ก็ "สมมติ" ว่าสัญญาณ 1000 Hz → เลือก T/DIV สำหรับ 1000 Hz → หน้าต่างสั้นกว่าคาบจริง → `FindTrigger()` **หาขอบไม่เจอ** → โค้ดเดิมใช้ `trig_hold` ค่าเก่า → จุดเริ่มวาดไม่ตรงกับข้อมูลใหม่ → **กราฟกระโดด/ขาดทุกเฟรม**

**แก้แล้ว (คีย์ `z` เดิม ไม่ต้องเปลี่ยนวิธีใช้)**
- auto scale **สแกน T/DIV ทุกค่า** เก็บ Vpp ของแต่ละ base (อ่าน 2 ครั้ง เอาค่าที่ต่ำกว่า = ทิ้งค่าชั่วขณะหลังสลับเวลา) → เลือก base ที่ **Vpp ครบที่สุด** เป็น base ที่วัดความถี่ (ที่ความถี่ต่ำ base นั้นคือ base ที่ช้าพอ) → ถ้ายังใช้ไม่ได้ ใช้ **median** ของค่าที่วัดได้ทุก base → **ไม่มีค่า 1000 คงที่แล้ว**
- ถ้า `FindTrigger()` หาขอบไม่ได้ (หน้าต่างสั้นกว่าคาบ) → **วาดแบบไถล (offset 0)** แทนการใช้ค่าเก่า → รูปไม่กระโดด

**ตรวจกับสัญญาณจริง (เครื่องกำเนิดตั้ง ~200 Hz ต่อที่ PA0)**

| base (T/DIV) | 0.1 ms | 0.2 ms | 0.5 ms | 1.0 ms | 2.0 ms |
|---|---|---|---|---|---|
| Vpp ที่วัด | 1.259 | 1.093 | **2.690** | 2.683 | 2.665 |
| f ที่วัด | 0 (ใช้ไม่ได้) | 0 | 214 | 200 | 188 |

→ เลือก base 0.5 ms (Vpp ครบสุด) → f ที่ใช้ตัดสิน **214 Hz** (วัดจริง ไม่ใช่ 1000) → **T/DIV 1.0 ms (1.84 คาบ) · V/DIV 1.0 V (2.7 ช่อง) · trig_off = 21** (เจอขอบ trigger จริง → กราฟนิ่ง ไม่พัง)

ตรวจในโหมด simulator (บังคับให้ "วัดเอง" ไม่ใช้ค่าที่ตั้ง) ผ่านทุกช่วง: 100 Hz→2.0 ms (2.0 คาบ) · 200 Hz→1.0 ms · 500 Hz→0.5 ms · 1000 Hz→0.1 ms · 5000 Hz ✓

## 14. แก้บั๊ก: กราฟกระตุกครึ่งหลัง (จุดวนกลับบัฟเฟอร์) — แก้แล้ว

**อาการ**: ครึ่งซ้ายของจอเรียบเป็น sine ปกติ แต่ครึ่งหลังเริ่มกระตุก/รูปเพี้ยน ไม่เป็น sine

**สาเหตุ**: `MapToScreen(t)` วาดจาก index `t` (จุดที่ `FindTrigger()` เจอ) ไปจนสุดบัฟเฟอร์ 240 จุด แล้ว **วนกลับไปเริ่มที่ index 0** — แต่ปลายบัฟเฟอร์กับหัวบัฟเฟอร์ **ไม่ต่อเนื่องกันทางเวลา** (คนละช่วงเวลา) ⇒ ครึ่งหลังของจอจึงเป็นข้อมูลอีกช่วงหนึ่ง → คลื่นขาด/รูปแปลก และเพราะ `t` เปลี่ยนทุกเฟรม (ค้นขอบได้ที่ 0-180) จุดที่วนกลับจึงเลื่อนไปมา = **ดูเหมือนกระตุก**

บั๊กนี้มีอยู่เดิม (อยู่ในลิสต์ "trigger wrap → รอยหัก") แต่ในโหมด simulator มองไม่ชัด เพราะข้อมูลถูกสร้างใหม่ให้เฟสต่อเนื่องกันทุกเฟรม — พอต่อ ADC จริง สองปลายบัฟเฟอร์ไม่สัมพันธ์กันทางเฟส จึงเห็นชัด

**วิธีแก้ (แบบเดียวกับ scope จริง = pretrigger)**
1. `AdcCaptureFrame()` จับข้อมูลดิบเผื่อ **120 จุดก่อน trigger** (`ADC_PRE_POINTS`, บัฟเฟอร์ขยายเป็น 25,000 จุด)
2. หาจุด trigger ในข้อมูลดิบ (ระดับ = ค่ากลางบัฟเฟอร์) แล้ว **resample 240 จุดเริ่มที่จุด trigger** → หน้าต่างที่วาดต่อเนื่องทั้งจอ ไม่ต้องวนกลับ
3. เหลื่อมไปข้างหน้า ~2 จุด (ไม่ให้เริ่มตรงจุดตัดศูนย์พอดี — ไม่งั้นตัววัดความถี่จะนับขอบแรกพลาดเป็นครั้งคราว → f กระพริบเป็น 0)
4. โหมด ADC: ตั้ง `trig_offset = 0` (จัดแนวตั้งแต่ตอนจับแล้ว) — ไม่ให้ `FindTrigger()` หาใหม่แล้วได้คนละจุด

**ตรวจกับสัญญาณจริง (เครื่องกำเนิด ~185 Hz)**

| | ก่อนแก้ | หลังแก้ (6 รอบ) |
|---|---|---|
| รอยหักครึ่งซ้าย | 0.068 V (3%) | 3% |
| รอยหักครึ่งขวา | **0.26-1.14 V (10-44%)** ✗ | **3%** ✓ |
| trig_off (จุดวนกลับ) | 5-131 (กลางจอ) ✗ | 0 (วนกลับนอกจอ) ✓ |
| f ที่อ่านได้ | 184 Hz (บางครั้ง 0) ✗ | 184 Hz คงที่ 6/6 ✓ |

พร้อมใช้: `s u d h m z` เหมือนเดิม · ถ้าอยากให้รูปคมแนะนำ FILT 1-5 (ตอนนี้ตั้งไว้ 5 แล้ว)

## 15. แก้บั๊ก: ค่าความถี่ที่แสดงไม่ตรง (50–200 Hz อ่านได้ 2 เท่า/0) — แก้แล้ว

**สาเหตุ**: ตัววัดความถี่ใน `OscMeasureFrame()` นับ **ทุกครั้ง** ที่ค่าข้ามศูนย์ (ไม่มี hysteresis)
ที่ความถี่สูงมีหลายสิบขอบต่อหน้าต่าง ความผิดพลาด 1-2 ขอบแทบไม่กระทบ (ช่วง 500 Hz–20 kHz จึงยังถูก)
แต่พอหน้าต่างมีแค่ 1-2 คาบ noise รอบ ๆ ศูนย์ทำให้ได้ขอบเกิน → span (ขอบแรก→ขอบสุดท้าย) สั้นกว่าจริง = ครึ่งคาบ → **อ่านได้ 2 เท่า** (100 Hz → 203 Hz) หรือได้ขอบไม่ครบ 2 → **อ่านได้ 0**

**แก้แล้ว**: แยกเป็น 2 รอบ — รอบ 1 หา min/max/RMS/peak, รอบ 2 นับขอบ**แบบมี hysteresis** (หลังนับขอบหนึ่ง ต้องรอให้สัญญาณแกว่งพ้น ±20% ของ Vpp ก่อน จึงนับขอบถัดไป) ตำแหน่งขอบยัง interpolate ตามจริง

**ตรวจแล้ว (โหมด sim ฉีดความถี่ตรง, ทุกคู่ความถี่ × T/DIV)**

| ตั้งไว้ | T/DIV | คาบบนจอ | ก่อนแก้ | หลังแก้ |
|---|---|---|---|---|
| 100 Hz | 2.0 ms | 2.0 | 2 เท่า ✗ | **100 Hz (−0.2%)** ✓ |
| 100 Hz | 1.0 ms | 1.0 | 203 Hz ✗ | 0 (ไม่พอ 2 คาบ → ไม่รายงานค่ามั่ว) |
| 200 Hz | 2.0/1.0 ms | 4.0/2.0 | เพี้ยน ✗ | **201 / 199 Hz** ✓ |
| 500 Hz | 2.0/1.0 ms | 10/5 | — | **500 / 497 Hz** ✓ |
| 1000 Hz | 2.0/1.0/0.2 ms | 20/10/2 | — | **999 / 995 / 1006 Hz** ✓ |
| ≤50 Hz (หน้าต่าง < 1 คาบ) | — | ≤1 | มั่ว ✗ | 0 (ยังต้องแก้ข้อ 3: โหมดสุ่มช้า) |

**เพิ่มด้วย (p43)**: ถ้า Vpp < 0.02 V (ยังไม่ต่อสาย/สัญญาณเล็กมาก) จะรายงาน f = 0 แทนค่า noise (เดิมโชว์ 1089 Hz ทั้งที่ไม่มีสัญญาณ)

## 16. ข้อควรรู้






- **การตรวจจับชนิดคลื่น** ใช้ crest factor ตาม simulator (`rms/peak`: >0.85=SQR, >0.65=SIN, อื่น ๆ TRI/SAW)
  ⇒ ถ้ามี noise เยอะ ค่า peak จะสูงเกิน → คลื่น SQUARE อาจถูกอ่านเป็น SIN (simulator ก็เป็นแบบนี้)
  ถ้าอยากให้แม่นขึ้นบอกได้ (เช่นดู histogram หรือกัน noise ก่อนวัด)
- **HOLD** จะ freeze ข้อมูลไว้ (ไม่ generate ใหม่) แต่ยังปรับ V/DIV, FILTER, C1/C2 ดูของเดิมได้
- แถวที่ 2 ของจอ LCD: ตอน HOLD จะเปลี่ยนไปโชว์ U1/U2/ΔV/Δt แทนค่าวัด
- Vmax/Vmin ในโค้ดที่แก้ไว้อยู่ที่ y=20 ซึ่งต่ำกว่าเส้น separator (HEADER_H=19) → รูปคลื่นอาจทับข้อความ แนะนำเลื่อนกลับ y=10 หรือเพิ่ม HEADER_H
- บั๊กเดิมที่เหลือ (ยังไม่แก้ตามที่สั่งเก็บไว้): CENTER_Y=120, trigger wrap, ปุ่ม PB0–PB3 ไม่ได้ config ใน gpio.c (แต่ UART ใช้ได้แล้วจึงไม่กระทบ)

## 17. v54 — ปุ่ม FREEZE / AUTO / UP-LEFT / DOWN-RIGHT และคีย์ใหม่

ทุกคีย์ทำงานทันที ไม่ต้องกด Enter (ยกเว้น `u` ที่รอ ~250 ms เพราะมีรูปแบบ `u 2`). ปุ่มจริงและคีย์เรียกฟังก์ชันเดียวกัน (`ActFreeze`, `ActAuto`, `ActUp`, `ActDown`, `ActToggleMoveAxis`, `ActToggleFastSlow`, `ActAutoMode`). ไม่กดอะไร = พฤติกรรมเหมือน v53 ทุกอย่าง (auto FAST/SLOW, hold, trigger, Vpp/F).

| คีย์ | ปุ่ม | ทำอะไร |
|---|---|---|
| `h` | FREEZE (ขา HOLD) | freeze / ปลด freeze (ปลด = offset ย้ายภาพกลับเป็น 0) |
| `z` | AUTO (ขา SET, กดสั้น) | auto scale แบบออสซิลโลสโคปจริง: ปลด freeze + ล้าง offset + เลือก V/DIV, T/DIV |
| `u` / `d` | UP-LEFT / DOWN-RIGHT | **ไม่ freeze**: ปรับ setting ที่เลือกด้วย `s` (เหมือนเดิม). **freeze อยู่**: ย้ายภาพ (ดู `x`). ปุ่มจริง: กดค้าง = ก้าวแรกทันที, ซ้ำทุก 60 ms หลัง 300 ms, ก้าวละ 3 เท่าหลัง 1.5 s |
| `x` | — | สลับ `MOVE:Y` ↔ `MOVE:X` (แสดง `MOVE:Y/X` บนหัวจอตอน freeze) |
| `0` | — | จัดภาพกลับกึ่งกลาง (offset Y=0, X=0) |
| `F` (ตัวใหญ่) | PC13 กดสั้น | บังคับ FAST ↔ SLOW (ถ้าอัตโนมัติอยู่ = สลับเป็นตรงข้ามของโหมดตอนนี้) ปิด auto time base. หัวจอแสดง `FAST*` / `SLOW*` (ดอกจัน = บังคับ). ตอน freeze ไม่ทำงาน |
| `A` (ตัวใหญ่) | PC13 กดค้าง 1 s | กลับไปสลับ FAST/SLOW อัตโนมัติ (`AUT`) — ค่าเริ่มต้นหลังเปิดเครื่อง |
| `s` | — | (SET เดิม) เลือก setting ถัดไป — ปุ่ม SET ตอนนี้เป็นปุ่ม AUTO แล้ว ดังนั้น `s` เหลือเฉพาะทาง UART |

`f` / `a` ตัวเล็กยังเป็น freq / amp ของ generator เหมือนเดิม (`F` / `A` ตัวใหญ่ = ฟังก์ชันใหม่, ตัวอักษรถูกแยกด้วยตัวพิมพ์).

**MOVE (เฉพาะตอน freeze)**
- Y: `u` = ภาพขึ้น, `d` = ลง, ก้าวละ 0.1 div, จำกัด ±4 div. เป็น offset ตอนวาดเท่านั้น — ตัวเลข Vpp/F ไม่เปลี่ยน
- X: `u` = ซ้าย (เวลาก่อนหน้า), `d` = ขวา, ก้าวละ 1/40 จอ (6 จุด). จำกัดอยู่ในข้อมูลที่จับไว้จริง: FAST = จุดที่จับไว้ก่อน/หลังหน้าต่าง (ปกติโหมด ROLL มีเฉพาะด้านขวา ~40–200 จุด), SLOW = ย้อนเข้าไปใน history 8 s (สำเนาตอน freeze). ถ้าไม่มีข้อมูลเกินหน้าจอ X ไม่ขยับ
- ตอน freeze `u`/`d` ไม่ปรับ C1/C2/V-DIV อีกแล้ว (ใช้ `c1 <n>` / `c2 <n>` ตั้ง cursor ตรง ๆ ได้)

**AUTO scale (`z` / ปุ่ม AUTO)**: V/DIV = ขั้นเล็กสุดที่ Vpp ไม่เกิน 4.0 div (จอเห็น ~5.3 div); T/DIV = เลือกด้วยกฎเดียวกับ auto time base (FAST ≥ 3.2 คาบ; SLOW ใช้ฐานเดียวกับที่ auto เลือก) เพื่อไม่ให้ auto time base ย้ายซ้ำ; Vpp < 0.05 V = ไม่มีสัญญาณ → 1 V/div + 1 ms/div; trigger อยู่ที่กลางสัญญาณอยู่แล้ว; AUTO ล้างโหมดบังคับ (`F`) กลับเป็นอัตโนมัติ.

`?` แสดงบรรทัด `Mode:FAST*  MOVE:Y (frozen only)  offY:.. offX:..` เพิ่ม.

## v56 — real-oscilloscope picture
- Power-on: **1 ms/div FAST, 1 V/div**, continuous auto time base **OFF**. T/DIV and V/DIV stay where you set them; nothing re-zooms by itself.
- `z` / AUTO button = **one-shot** autoscale: picks T/DIV (about 2–5 cycles on screen), V/DIV and FAST/SLOW once, then stays fixed. It no longer switches the continuous auto time base on.
- `A` (or PC13 long press) = continuous automatic FAST/SLOW + T/DIV again (OFF at power-on). Changing T/DIV or pressing `F` turns it off.
- `F` = toggle FAST/SLOW by hand (picks a sensible T/DIV for that mode). T/DIV ≥ 10 ms/div runs the SLOW engine, faster ones FAST.
- Picture default is **trigger-locked** (`m a`); `m r` = roll, `m` toggles. Without a trigger edge (or below ~45 Hz in FAST) the free-running capture is shown.
- A signal that does not fit the chosen T/DIV is shown as it really is (flat line / dense band); press `z`. Numbers (Vpp/Vmax/Vmin/RMS/F) are measured independently of the picture. FAST cannot read F below ~40 Hz, so F shows 0 until you press `z` / `F` (SLOW).

## v57 — zoom on the frozen picture (`+` / `-`)
| Key | Does (only while frozen, `h`) |
|---|---|
| `+` | zoom in: time AND volts together (v59 combined zoom; the MOVE axis does not matter) |
| `-` | zoom out |

- Immediate keys (no Enter). `v +`, `v -`, `t +`, `t -` are unchanged (a line that starts with a letter is a line command; only a leading `+`/`-` is a zoom key).
- **Combined zoom (v59):** one zoom step 0..6 drives both axes. Time x1, x2, x4, x8, x16, x32, x64 around the screen centre; volts x1, x2, x4, x8, x16, x32 around the centre line (volts stop at x32, so the 6th `+` zooms only time: x64/x32; `-` mirrors it). `x` (MOVE:Y / MOVE:X) now ONLY chooses what `u`/`d` pan (vertical offset vs horizontal). `u`/`d` pan; at xN the view is 1/N of the screen and the pan range grows within the stored data (finer steps: 1/40 of the view).
- FAST: real raw samples of the frozen capture are used (a copy kept for the pan/zoom); below one raw sample per pixel, or without stored samples, points are joined by straight lines. SLOW: the frozen 1 ms history copy, same rule.
- Header while zoomed: `Z:x4` (both axes equal) or `Z:64/32` (time/volt, only at the top step), and the T / V fields show the effective T/DIV (T/DIV ÷ N) or V/DIV (V/DIV ÷ N); cursor `dt` is divided by the time zoom.
- Not frozen: `+`/`-` do nothing. Unfreeze (`h`), `0` and AUTO (`z`) reset the zoom to x1. Vpp / F are unchanged (whole frozen capture). `?` prints `Zoom: X xN  Y xN`. Unfreeze, `0` and AUTO reset the shared step to 0.

## v61 — 8 physical buttons (reference; UART users can ignore)
All active LOW (pull-up, press = connect pin to GND), debounce 200 ms. Every button calls the same Act* function as the UART key shown.

| Pin | Button | Action | UART equivalent |
|---|---|---|---|
| PB0 | INCREASE (+) | OscAdjust(+1) on the selected setting; hold-repeat (300 ms, then 60 ms, 3x after 1.5 s) | `u` when not frozen |
| PB1 | DECREASE (−) | OscAdjust(−1), same hold-repeat | `d` when not frozen |
| PB2 | AUTO | one-shot auto scale | `z` |
| PB3 | FAST/SLOW | short: toggle forced FAST↔SLOW; hold ≥ 1 s: automatic again | `F` / `A` |
| PA3 | V/DIV ↔ T/DIV toggle | toggles the selection between V/DIV and T/DIV; while frozen T/DIV does not exist, so it (re)selects V/DIV | `v` / `t` |
| PC0 | FILTER ↔ TRIG toggle | toggles the selection between FILTER and TRIG; no-op while frozen (neither exists in the hold menu) | `s` cycling |
| PC3 | STABLE lock / cursor | short press (release < 1.5 s): toggle stable/lock (AUTO↔ROLL, same as UART bare `m`; picture stops sliding, capture keeps running live, does NOT freeze). Hold ≥ 1.5 s: toggle C1 ↔ C2 cursor selection (only while frozen; otherwise nothing, and the release toggles nothing) | `m` |
| PF3 | FREEZE | true freeze/unfreeze toggle of `hold_active` on press (`ActFreeze`, the same function as `h`); stops capture so zoom/pan/cursors can be used. Does not touch the stable lock (PC3 short / `m`) | `h` |

Notes: physical + / − **never** move/pan/zoom, even while frozen (they always do OscAdjust); move/zoom/freeze/cursors stay UART-only (`u`/`d`/`+`/`-`/`h`). Select and TRIGGER buttons work frozen or not. PC13 blue button keeps its FAST/SLOW behaviour.
