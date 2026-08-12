#!/usr/bin/env bash
# 문서 규약 검사. 커밋 전 또는 세션 종료 시.
set -u; fail=0

echo "== 1. meta 헤더 없는 문서"
for f in $(find md -name '*.md' -not -path 'md/archive/*'); do
  head -2 "$f" | grep -q '<!-- meta' || { echo "  $f"; fail=1; }
done

echo "== 2. 문서 지도 미등재"
for f in $(find md -name '*.md' -not -name README.md -not -path 'md/journal/*'); do
  grep -q "$(basename "$f")" md/README.md || { echo "  $f"; fail=1; }
done

echo "== 3. 숫자 의존 참조"
grep -rnE '§[A-Z0-9]+[^ 다]|\(위 [0-9]+번\)' md/ --include='*.md' \
  | grep -v 'archive/' | grep -v 'errors-log' && fail=1

echo "== 4. owns 중복 선언"
grep -rh '^owns:' md/ --include='*.md' | sort | uniq -d && fail=1

echo "== 5. 7일 이상 안 고친 live 문서"
for f in $(grep -rl 'status:  *live' md/ --include='*.md'); do
  d=$(grep -m1 '^updated:' "$f" | grep -oE '[0-9]{4}-[0-9]{2}-[0-9]{2}')
  [ -n "$d" ] && [ "$(( ($(date +%s) - $(date -d "$d" +%s)) / 86400 ))" -gt 7 ] \
    && echo "  $f ($d)"
done

exit $fail
