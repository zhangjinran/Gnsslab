#!/usr/bin/env python3

from pathlib import Path
import re
from collections import defaultdict, deque


ROOT = Path(".")
SRC_DIRS = [
    ROOT / "examples",
    ROOT / "lib",
]


# 只分析项目自己的 include
include_pattern = re.compile(
    r'#include\s+"([^"]+)"'
)


def read_includes(file):
    try:
        text = file.read_text(
            encoding="utf-8",
            errors="ignore"
        )
    except:
        return []

    return include_pattern.findall(text)


def resolve_include(name, current):
    candidates = [
        current.parent / name,
        ROOT / name,
        ROOT / "lib" / name,
        ROOT / "examples" / name,
    ]

    for c in candidates:
        if c.exists():
            return c.resolve()

    return None


def scan(start):

    visited = set()
    queue = deque([start])

    result = []

    while queue:

        f = queue.popleft()

        if f in visited:
            continue

        visited.add(f)
        result.append(f)

        for inc in read_includes(f):

            dep = resolve_include(
                inc,
                f
            )

            if dep and dep not in visited:
                queue.append(dep)

    return result


def main():

    examples = sorted(
        (ROOT/"examples").glob("*.cpp")
    )

    for example in examples:

        deps = scan(
            example.resolve()
        )

        print("\n")
        print("="*80)
        print(example)

        for d in sorted(deps):
            print(
                "   ",
                d.relative_to(ROOT.resolve())
            )


if __name__ == "__main__":
    main()
