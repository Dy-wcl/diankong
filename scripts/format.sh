#!/usr/bin/env bash
# 一键格式化本仓库自有代码(使用根目录 .clang-format)。
# 范围:app arithmetic bsp calculate component task tool
# 不动 Core/、Drivers/、Middlewares/(CubeMX/HAL 生成代码)和 modules/(submodule)。
#
# 需要 clang-format 23.1.1(与 CI 一致):pip install clang-format==23.1.1
set -euo pipefail
cd "$(dirname "$0")/.."

command -v clang-format >/dev/null 2>&1 || {
  echo "错误: 未找到 clang-format,请先安装: pip install clang-format==23.1.1" >&2
  exit 1
}
echo "使用: $(clang-format --version)"

# 以 NUL 分隔收集文件到数组,文件名含空格等字符也不会被错误拆分
FILES=()
while IFS= read -r -d '' f; do
  if [[ $f =~ \.(c|h|cc|cpp|hpp)$ ]]; then FILES+=("$f"); fi
done < <(git ls-files -z app arithmetic bsp calculate component task tool)

if [ "${#FILES[@]}" -eq 0 ]; then
  echo "没有找到需要格式化的源文件"
  exit 0
fi

clang-format --style=file -i "${FILES[@]}"
echo "已格式化 ${#FILES[@]} 个文件,可用 git diff 查看改动"
