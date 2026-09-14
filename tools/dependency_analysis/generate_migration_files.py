from pathlib import Path


ROOT = Path(".")


header_file = Path(
    "all_required_headers.txt"
)


keep = set()


for line in header_file.read_text().splitlines():

    line = line.strip()

    if not line:
        continue


    path = ROOT / line

    if not path.exists():
        continue


    keep.add(
        path.relative_to(ROOT)
    )


    # 添加同名 cpp
    cpp = path.with_suffix(".cpp")

    if cpp.exists():

        keep.add(
            cpp.relative_to(ROOT)
        )


    # 添加同名 hpp
    hpp = path.with_suffix(".hpp")

    if hpp.exists():

        keep.add(
            hpp.relative_to(ROOT)
        )



with open(
    "migration_keep_files.txt",
    "w",
    encoding="utf-8"
) as f:

    for item in sorted(keep):

        f.write(
            str(item)
        )

        f.write("\n")


print(
    "Total files:",
    len(keep)
)

print(
    "Output: migration_keep_files.txt"
)
