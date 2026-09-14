from pathlib import Path


root = Path(".")


report = Path(
    "dependency_final_report.txt"
)


keep=set()


for line in report.read_text().splitlines():

    line=line.strip()

    if line.startswith("lib/"):

        keep.add(line)

        p=root/line

        stem=p.stem

        parent=p.parent


        # 自动寻找对应 cpp/h/hpp
        for ext in [
            ".cpp",
            ".h",
            ".hpp"
        ]:

            f=parent/(stem+ext)

            if f.exists():

                keep.add(
                    str(
                        f.relative_to(root)
                    )
                )



with open(
    "keep_files.txt",
    "w"
) as f:

    for x in sorted(keep):
        f.write(x+"\n")

