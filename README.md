# FootTrack — 축구/풋살 실시간 동선 트래커

ESP32-S3 + GPS + IMU를 몸에 달고 뛰면, 폰 브라우저에서 **내가 지나간 길이 지도 위에 실시간으로 그려지고**
이동거리·속도구간·스프린트 횟수 같은 FIFA EPTS 스타일 지표가 함께 표시된다.

| 부품 | 역할 | 인터페이스 |
|---|---|---|
| ESP32-S3-WROOM-1 N16R8 | 수집·지표 계산·웹 서버 | — |
| ATGM336H | GPS/BDS 위치·Doppler 속도, 10Hz | UART1 115200 |
| ICM-42688-P | 6축 IMU 100Hz → PlayerLoad | SPI 8MHz |

## 배선

```
ATGM336H          ESP32-S3
  VCC  ──────────  3V3
  GND  ──────────  GND
  TXD  ──────────  GPIO18 (PIN_GPS_RX)
  RXD  ──────────  GPIO17 (PIN_GPS_TX)

ICM-42688-P 모듈  ESP32-S3        (보라색 브레이크아웃 실크 기준)
  VCC  ──────────  3V3             모듈 3V3 핀이 3.0V 아래로 나오면 5V로 옮길 것
  GND  ──────────  GND
  AD0/MISO ──────  GPIO13
  SDA/MOSI ──────  GPIO11
  SCL/SCLK ──────  GPIO12
  CS   ──────────  GPIO10          LOW 활성, HIGH면 I2C 모드
  INT1, 3V3, NC, INT2 ── 미연결
```

핀은 `include/pins.h`에서만 바꾸면 된다. GPIO35/36/37은 OPI PSRAM 전용이라 쓰지 말 것.
GPS는 부팅 시 GPIO18/17 × 9600/115200 네 조합을 자동 탐색하므로 TX/RX가 뒤바뀌어도 잡힌다.
신호가 없으면 15초마다 재탐색하고, 잡히던 스트림이 8초 이상 끊겨도 재탐색한다. 시리얼 로그:

```
[GPS] 감지: GPIO17에서 NMEA 10문장 @9600 bps  ※ pins.h와 TX/RX 반대 — 자동 적용
[GPS] 신호 없음 — GPIO18/17 양쪽, 9600/115200 모두 조용함. 모듈 VCC/GND/TXD 확인
```
그림으로 보려면 `docs/pinmap.html`을 브라우저로 연다.

## 빌드 · 업로드

```powershell
# 1) 폰 핫스팟 정보 (없으면 AP 모드로만 동작)
Copy-Item include\wifi_secrets.h.example include\wifi_secrets.h   # 열어서 SSID/PW 기입

# 2) 빌드 → 업로드 → 시리얼 모니터
pio run
pio run -t upload
pio device monitor
```

시리얼(115200)에 접속 주소가 찍힌다.

```
[WiFi] 연결됨  →  http://172.20.10.3/  (또는 http://foottrack.local/)
```

## 폰에서 보는 방법 (두 가지)

| 방식 | 설정 | 주소 | 지도 타일 |
|---|---|---|---|
| **A. 폰 핫스팟** (권장) | 폰 핫스팟 켜기 + `wifi_secrets.h`에 SSID/PW | 시리얼에 찍힌 IP, iPhone은 `foottrack.local` | O (OSM 온라인) |
| **B. ESP32 AP** | 폰 WiFi에서 `FootTrack` / `foottrack1234` 접속 | `http://192.168.4.1` | X → 미터 격자 캔버스로 자동 폴백 |

A 방식에서 IP를 모르겠으면: iPhone 핫스팟은 보통 `172.20.10.x`, Android는 핫스팟 설정의 "연결된 기기"에서 확인.
Android는 mDNS(`.local`)를 브라우저에서 못 푸는 경우가 많으니 IP로 접속한다.

## 화면

- **지도**: 파란 선 = 동선, 빨간 점 = 현재 위치. 지도를 드래그하면 "따라가기"가 꺼진다.
- **타일**: 이동거리 / 현재속도 / 최고속도 / 경과시간
- **속도구간 바**: 걷기(<7) · 조깅(7~14) · 러닝(14~20) · 고속(20~25) · 스프린트(>25 km/h) 거리 비율
- **상태줄**: FIX 여부, 위성 수, HDOP, 저장 점 수, PL(PlayerLoad, IMU 있을 때)
- **리셋**: 세션·동선 초기화. **GPX**: 표준 GPX 파일 다운로드(Google Earth, Strava 등에서 열림)

## 지표 계산 방식

- **거리** = Doppler 속도(RMC) × Δt 적분. GPS 좌표 차분은 정지 시 지터가 거리로 쌓이므로 쓰지 않는다.
  0.5 m/s(1.8 km/h) 미만은 정지로 간주, 2초 넘는 fix 공백은 적분 제외.
- **유효 fix** 조건: 위성 ≥ 4, HDOP ≤ 6, 위치 age < 1.5s. 조건 미달이면 지표에 반영하지 않는다.
- **스프린트** = 25 km/h 초과를 1초 이상 유지 → 1회.
- **PlayerLoad** = Σ √(Δax² + Δay² + Δaz²) / 100 (Catapult 정의, 100Hz, 단위 g).
- **동선 점** = 이동 중이고 마지막 저장점에서 1m 이상 벗어났을 때만 저장(정지 블롭 방지).
  버퍼(PSRAM 6만 점)가 차면 절반으로 솎아내고 간격을 2배로 늘려 계속 기록한다.

임계값은 전부 `include/config.h`.

## BLE로 보기 (Android Chrome, Web Bluetooth)

WiFi 없이 BLE만으로 같은 화면을 볼 수 있다. 페이지 원본은 `docs/app/index.html` 하나이고,
빌드 때 `tools/embed_page.py`가 펌웨어에도 내장하므로 WiFi 경로와 BLE 경로가 항상 같은 화면이다.

1. **페이지를 https로 연다.** Web Bluetooth는 https(또는 localhost)에서만 동작한다.
   배포된 주소: **https://changhoon-yoon.github.io/football-gps-tracker/app/** (GitHub Pages, `/docs` 폴더).
   홈 화면에 추가해 두면 앱처럼 쓴다. 저장소: https://github.com/changhoon-yoon/football-gps-tracker
   - 임시 테스트: Android Chrome `chrome://flags/#unsafely-treat-insecure-origin-as-secure`에
     `http://192.168.0.72`(ESP IP)를 넣고 Enabled → ESP가 서빙하는 http 페이지에서도 BLE 버튼이 동작한다.
2. 페이지의 **BLE 연결** 버튼 → 기기 선택창에서 `FootTrack` 선택.
3. 연결되면 저장된 동선이 먼저 재생되고, 이후 FIX 10Hz / STATS 1Hz로 갱신된다. 리셋·GPX도 BLE로 동작한다.
4. 배터리로 뛸 때는 `config.h`의 `ENABLE_WIFI`를 0으로 → WiFi 꺼지고 소비전류가 크게 준다.

GATT 규격(UUID, 프레임 바이트 배치)은 `src/ble_link.h` 상단 주석. 서비스 `f007ba11-0000-4c45-8000-000000000000`,
FIX `…0001`(20B notify), STATS `…0002`(20B notify), HIST `…0003`(8B/점 notify, 끝 마커 lat=0x7FFFFFFF), CTRL `…0004`(write: 1=리셋, 2=이력).

제약: iOS Safari는 Web Bluetooth 미지원(Bluefy 앱 필요). 화면이 꺼지면 연결이 끊길 수 있어 페이지가 Wake Lock으로 화면을 켜 둔다.
WiFi와 BLE를 같이 켜면 WiFi 모뎀 슬립이 강제되어 SSE 지연이 수십 ms 늘어난다.

## HTTP 엔드포인트

| 경로 | 설명 |
|---|---|
| `GET /` | 웹 앱 |
| `GET /events` | SSE. `fix` 이벤트로 10Hz(무fix 시 1Hz) JSON 푸시 |
| `GET /track` | 저장된 동선 전체 `{"pts":[[lat,lon,t_s,spd_cms],…],"n":N}` |
| `GET /track.gpx` | GPX 1.1 다운로드 |
| `GET /status` | 마지막 SSE 페이로드 (디버깅) |
| `POST /reset` | 세션 초기화 |

SSE 페이로드 예:
```json
{"fix":1,"gps":1,"sat":9,"hdop":1.1,"lat":37.1234567,"lon":127.1234567,"spd":12.4,"crs":231,
 "dist":1532.0,"max":24.8,"el":612,"z":[400,700,380,52,0],"spr":0,"pl":312.5,"imu":1,"pt":1,"n":890}
```

## 알려진 한계 · 다음 단계

- **실내 풋살은 GPS가 안 잡힌다.** 이 구성은 실외 전용. 실내는 IMU 지표(PL, 가감속)만 가능.
- GPS 안테나가 하늘을 봐야 하므로 착용 위치는 견갑골 사이(조끼)가 좋다.
- 다음: 경기장 코너 4점 캘리브레이션 → 피치 좌표계 히트맵, 가감속 횟수(IMU 융합), LittleFS 세션 저장.
