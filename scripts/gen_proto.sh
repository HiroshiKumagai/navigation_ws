#!/usr/bin/env bash
# proto/ の API 定義から Python の gRPC コードを生成する（出力: mission_server/gen/）。
# proto を変更したら実行する。生成物は git 管理しない。
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="$WS_DIR/.venv/bin/python"
OUT="$WS_DIR/mission_server/gen"

rm -rf "$OUT"
mkdir -p "$OUT"
"$PYTHON" -m grpc_tools.protoc \
    -I "$WS_DIR/proto" \
    --python_out="$OUT" \
    --pyi_out="$OUT" \
    --grpc_python_out="$OUT" \
    $(cd "$WS_DIR/proto" && find . -name "*.proto" | sed 's|^\./||')

# 生成物を Python パッケージとして import できるようにする（locomotion.v1.locomotion_pb2）
find "$OUT" -type d -exec touch {}/__init__.py \;
rm -f "$OUT/__init__.py"
echo "生成しました: $OUT"
