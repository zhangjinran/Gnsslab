from pathlib import Path
import re
from collections import defaultdict, deque


# ==========================
# Project root
# ==========================

ROOT = Path(".").resolve()


# ==========================
# Scan source files
# ==========================

SEARCH_DIRS = [
    ROOT / "lib",
    ROOT / "examples",
]


source_files = {}

for d in SEARCH_DIRS:

    if not d.exists():
        continue

    for f in d.rglob("*"):

        if f.suffix in [
            ".cpp",
            ".h",
            ".hpp"
        ]:
            source_files[f.name] = f.resolve()


print(
    f"Found {len(source_files)} source files"
)


# ==========================
# Parse include
# ==========================

include_pattern = re.compile(
    r'#include\s+"([^"]+)"'
)


def get_includes(file):

    deps = []

    try:

        text = file.read_text(
            encoding="utf-8",
            errors="ignore"
        )

    except Exception:

        return deps


    includes = include_pattern.findall(text)


    for inc in includes:

        if inc in source_files:

            deps.append(
                source_files[inc]
            )


    return deps



# ==========================
# Build dependency graph
# ==========================

graph = defaultdict(list)


for name, file in source_files.items():

    graph[file] = get_includes(file)



# ==========================
# Recursive dependency
# ==========================

def resolve_dependencies(start):

    visited = set()

    queue = deque()

    queue.append(start)


    while queue:

        current = queue.popleft()


        if current in visited:

            continue


        visited.add(current)


        for nxt in graph[current]:

            if nxt not in visited:

                queue.append(nxt)


    return visited



# ==========================
# All examples
# ==========================

examples = sorted(
    (ROOT / "examples").glob("*.cpp")
)


print(
    f"Found {len(examples)} examples"
)



all_dependencies = set()



with open(
    "dependency_all_examples_report.txt",
    "w",
    encoding="utf-8"
) as report:


    for example in examples:


        deps = resolve_dependencies(
            example.resolve()
        )


        all_dependencies.update(
            deps
        )


        report.write("\n")
        report.write("=" * 80)
        report.write("\n")

        report.write(
            str(
                example.relative_to(ROOT)
            )
        )

        report.write("\n")

        report.write("=" * 80)
        report.write("\n")


        for d in sorted(deps):

            report.write(
                str(
                    d.relative_to(ROOT)
                )
            )

            report.write("\n")



# ==========================
# Export unique headers
# ==========================


with open(
    "all_required_headers.txt",
    "w",
    encoding="utf-8"
) as f:


    for d in sorted(all_dependencies):

        if d.suffix in [
            ".h",
            ".hpp"
        ]:

            f.write(
                str(
                    d.relative_to(ROOT)
                )
            )

            f.write("\n")



print("\nDONE")
print(
    "Report:"
    " dependency_all_examples_report.txt"
)

print(
    "Headers:"
    " all_required_headers.txt"
)

print(
    "Required headers:",
    len(all_dependencies)
) 
