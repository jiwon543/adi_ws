#!/usr/bin/env python3
"""
Moondream2 최신 revision 호환성 검증 스크립트
─────────────────────────────────────────────
노트북에서 실행:
  python3 verify_moondream_new.py

검증 항목:
  1. 모델 로드 (2025-01-09 revision)
  2. encode_image → 재사용 가능 여부
  3. query() API
  4. detect() API + bbox 좌표 형식
  5. 추론 시간 측정
"""

import time
import sys

# ── 1. 패키지 버전 확인 ───────────────────────────────────────────────────
print("=" * 60)
print("[1] 환경 확인")
print("=" * 60)

import torch
import transformers
print(f"  Python       : {sys.version.split()[0]}")
print(f"  torch        : {torch.__version__}")
print(f"  transformers : {transformers.__version__}")
print(f"  CUDA         : {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"  GPU          : {torch.cuda.get_device_name(0)}")
    print(f"  VRAM         : {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")

# ── 2. 모델 로드 ─────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("[2] 모델 로드 (revision=2025-01-09)")
print("=" * 60)

from transformers import AutoModelForCausalLM
from PIL import Image

REVISION = "2025-01-09"   # 먼저 안정적인 revision으로 테스트

t0 = time.time()
try:
    model = AutoModelForCausalLM.from_pretrained(
        "vikhyatk/moondream2",
        revision=REVISION,
        trust_remote_code=True,
        torch_dtype=torch.float16,
        device_map={"": "cuda"} if torch.cuda.is_available() else None,
    )
    model.eval()
    print(f"  ✅ 모델 로드 성공 ({time.time()-t0:.1f}s)")
except Exception as e:
    print(f"  ❌ 모델 로드 실패: {e}")
    sys.exit(1)

# ── 3. 더미 이미지 생성 ──────────────────────────────────────────────────
dummy = Image.new("RGB", (640, 480), color=(128, 128, 128))

# ── 4. encode_image 테스트 ───────────────────────────────────────────────
print("\n" + "=" * 60)
print("[3] encode_image 테스트")
print("=" * 60)

try:
    t0 = time.time()
    enc = model.encode_image(dummy)
    print(f"  ✅ encode_image 성공 ({(time.time()-t0)*1000:.0f}ms)")
    print(f"     type: {type(enc)}")
except Exception as e:
    print(f"  ❌ encode_image 실패: {e}")
    enc = None

# ── 5. query() 테스트 ────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("[4] query() 테스트")
print("=" * 60)

# 5a. PIL Image 직접 전달
try:
    t0 = time.time()
    result = model.query(dummy, "What do you see?")
    print(f"  ✅ query(image) 성공 ({(time.time()-t0)*1000:.0f}ms)")
    print(f"     answer: {result['answer']}")
except Exception as e:
    print(f"  ❌ query(image) 실패: {e}")

# 5b. encode_image 재사용
if enc is not None:
    try:
        t0 = time.time()
        result = model.query(enc, "What do you see?")
        print(f"  ✅ query(encoded) 성공 ({(time.time()-t0)*1000:.0f}ms)")
        print(f"     answer: {result['answer']}")
    except Exception as e:
        print(f"  ❌ query(encoded) 실패: {e}")

# 5c. settings (max_tokens) 테스트
try:
    t0 = time.time()
    result = model.query(
        dummy,
        "What do you see?",
        settings={"max_tokens": 20},
    )
    print(f"  ✅ query(settings) 성공 ({(time.time()-t0)*1000:.0f}ms)")
    print(f"     answer: {result['answer']}")
except Exception as e:
    print(f"  ⚠️  query(settings) 실패 (구버전?): {e}")
    # settings 미지원이면 max_tokens 없이 시도
    pass

# ── 6. detect() 테스트 ───────────────────────────────────────────────────
print("\n" + "=" * 60)
print("[5] detect() 테스트")
print("=" * 60)

try:
    t0 = time.time()
    result = model.detect(dummy, "cone")
    print(f"  ✅ detect('cone') 성공 ({(time.time()-t0)*1000:.0f}ms)")
    print(f"     objects: {result['objects']}")
    if result["objects"]:
        obj = result["objects"][0]
        print(f"     첫 bbox: x_min={obj['x_min']:.3f} y_min={obj['y_min']:.3f} "
              f"x_max={obj['x_max']:.3f} y_max={obj['y_max']:.3f}")
except Exception as e:
    print(f"  ❌ detect() 실패: {e}")

# 6b. encode_image 재사용으로 detect
if enc is not None:
    try:
        t0 = time.time()
        result = model.detect(enc, "cone")
        print(f"  ✅ detect(encoded) 성공 ({(time.time()-t0)*1000:.0f}ms)")
    except Exception as e:
        print(f"  ❌ detect(encoded) 실패: {e}")

# ── 7. 멀티 detect 속도 테스트 ───────────────────────────────────────────
print("\n" + "=" * 60)
print("[6] 멀티 detect 속도 (encode 재사용)")
print("=" * 60)

targets = ["traffic cone", "crosswalk", "sign"]
if enc is not None:
    for target in targets:
        try:
            t0 = time.time()
            result = model.detect(enc, target)
            elapsed = (time.time() - t0) * 1000
            n = len(result["objects"])
            print(f"  detect('{target:15s}') → {n} objects | {elapsed:.0f}ms")
        except Exception as e:
            print(f"  detect('{target}') 실패: {e}")

# ── 8. caption() 테스트 (보너스) ─────────────────────────────────────────
print("\n" + "=" * 60)
print("[7] caption() 테스트")
print("=" * 60)

try:
    t0 = time.time()
    result = model.caption(dummy, length="short")
    print(f"  ✅ caption(short) 성공 ({(time.time()-t0)*1000:.0f}ms)")
    print(f"     caption: {result['caption']}")
except Exception as e:
    print(f"  ❌ caption() 실패: {e}")

# ── 결과 요약 ────────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("검증 완료. 위 결과를 Claude에게 공유해주세요.")
print("=" * 60)