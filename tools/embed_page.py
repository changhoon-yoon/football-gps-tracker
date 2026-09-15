# PlatformIO pre-build 스크립트: docs/app/index.html → src/web_page.h (PROGMEM 원시 문자열)
# 페이지 원본은 docs/app/index.html 하나뿐이다. GitHub Pages(BLE)와 ESP32 내장(WiFi)이 같은 파일을 쓴다.
Import("env")
import os

root = env["PROJECT_DIR"]
src = os.path.join(root, "docs", "app", "index.html")
dst = os.path.join(root, "src", "web_page.h")

with open(src, encoding="utf-8") as f:
    html = f.read()
if ')HTML"' in html:
    raise SystemExit("[embed_page] index.html 안에 raw string 종결자 )HTML\" 가 있어 임베드할 수 없습니다")

out = (
    "// 자동 생성 파일 — 직접 편집 금지. 원본: docs/app/index.html (tools/embed_page.py)\n"
    "#pragma once\n#include <Arduino.h>\n\n"
    'static const char INDEX_HTML[] PROGMEM = R"HTML(' + html + ')HTML";\n'
)
old = open(dst, encoding="utf-8").read() if os.path.exists(dst) else None
if old != out:
    with open(dst, "w", encoding="utf-8", newline="\n") as f:
        f.write(out)
    print("[embed_page] src/web_page.h 갱신 (%d bytes)" % len(html))
