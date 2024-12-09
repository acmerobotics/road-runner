"""
Script to help build the docs that
* Converts named snippets from the docs Gradle project into Hugo shortcodes
* Processes papers with pandoc and puts them in the docs
* Generates API docs
"""

import argparse
import re
from pathlib import Path
import shutil
import subprocess


def unindent(n, s):
    lines = s.split("\n")
    return "\n".join([line[n:] for line in lines])


def main_samples(args):
    samples = {}
    decl_pattern = re.compile(r"([^\n]*)// sample: (.*)\n")

    for ext, lang in (("java", "java"), ("kt", "kotlin")):
        for path in (Path(__file__).parent / "app").glob(f"**/*.{ext}"):
            with path.open("r") as f:
                contents = f.read()

            match = decl_pattern.search(contents)
            while match is not None:
                n = len(match.group(1))
                name = match.group(2)
                if name in samples:
                    raise Exception(f"Duplicate sample name: {name}")

                end = contents.index("// end sample\n", match.end())
                samples[name] = f"```{lang}\n{unindent(n, contents[match.end():end])}\n```\n"
                match = decl_pattern.search(contents, end)

    base_dir = (Path(__file__).parent / "site" / "content").resolve()

    shutil.rmtree(base_dir / "docs", ignore_errors=True)

    paths = (base_dir / "raw-docs").glob("**/*.md")
    pattern = re.compile(r"<!-- sample: (.*) -->\n")
    for path in paths:
        print(f"Processing {path}")
        with path.open("r") as f:
            contents = f.read()

        dest_path = base_dir / "docs" / path.relative_to(base_dir / "raw-docs")
        dest_path.parent.mkdir(parents=True, exist_ok=True)
        parts = []
        match = pattern.search(contents)
        last_match_end = 0
        while match is not None:
            name = match.group(1)

            parts.append(contents[last_match_end:match.start()])
            parts.append(samples[name])

            last_match_end = match.end()
            match = pattern.search(contents, match.end())

        parts.append(contents[last_match_end:])

        with dest_path.open("w") as f:
            f.write("".join(parts))

    for ext in ("mp4", "png", "jpg"):
        for path in (base_dir / "raw-docs").glob(f"**/*.{ext}"):
            dest_path = base_dir / "docs" / path.relative_to(base_dir / "raw-docs")
            dest_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, dest_path)


def main_papers(args):
    papers_dir = (Path(__file__).parent / "site" / "static" / "papers").resolve()
    papers_dir.mkdir(parents=True, exist_ok=True)
    for path in (Path(__file__).parent.parent / "doc" / "pdf").glob("*.tex"):
        print(f"Processing {path}")
        subprocess.run(["pandoc", str(path), "-f", "latex", "-t", "html", "-s", "-o",
            str(papers_dir / path.with_suffix(".html").name), "--mathjax"], check=True)


# 1. Run `./gradlew dokkaHtml dokkaJavadoc` in ~/ftc/road-runner
# 2. Copy `~/ftc/road-runner/{core,actions}/build/dokka/html` to `site/public/v1-0-0-beta2/{core,actions}/kdoc`.
# 3. Copy `~/ftc/road-runner/{core,actions}/build/dokka/javadoc` to `site/public/v1-0-0-beta2/{core,actions}/javadoc`.
def main_api(args):
    assert args.version[0] == "v"

    url_version = args.version.replace(".", "-")

    rr_dir = Path(__file__).parent.parent / "road-runner"
    site_dir = Path(__file__).parent / "site"

    subprocess.run(["./gradlew", "dokkaHtml", "dokkaJavadoc"], cwd=rr_dir, check=True)

    for module in ("core", "actions"):
        shutil.copytree(rr_dir / module / "build" / "dokka" / "html", site_dir / "public" / "docs" / url_version / module / "kdoc")
        shutil.copytree(rr_dir / module / "build" / "dokka" / "javadoc", site_dir / "public" / "docs" / url_version / module / "javadoc")


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(required=True, dest="subparser_name")
    subparsers.add_parser("samples")
    subparsers.add_parser("papers")
    parser_api = subparsers.add_parser("api")
    parser_api.add_argument("--version", "-v", required=True)

    args = parser.parse_args()
    dict(
        samples=main_samples,
        papers=main_papers,
        api=main_api,
    )[args.subparser_name](args)


if __name__ == "__main__":
    main()
