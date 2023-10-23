import os
import re
import sys

import typer
from rich import print
from rich.console import Console

err_console = Console(stderr=True)
app = typer.Typer()
state = {"verbose": False}

warn_string = "[bold orange]Warning![/bold orange]"
err_string = "[bold red]Error![/bold red]"

docs_folder_prefix = "docs/src/content/docs/blocks/"
blocks_folder_prefix = "blocks/"

block_template = """\
from flojoy import flojoy, DataContainer


@flojoy
def {block_name}(
    default: DataContainer,
) -> DataContainer:
    pass
"""


class DocsTemplateBuilder:
    def __init__(self, block_name, block_folder_path):
        self.github_base = "https://github.com/flojoy-ai/blocks/blob/main/blocks/{block_folder_path}/{block_name}.py"

        self.block_name = block_name
        self.block_folder_path = block_folder_path
        self.github_link = self.github_base.format(
            block_name=block_name, block_folder_path=block_folder_path
        )

        self.template = """---
layout: "@/layouts/block-docs-layout.astro"
title: {block_name}
slug: {slug}
---

{{/* DO NOT EDIT THIS FILE! IT IS BEING AUTO GENERATED */}}
{{/* PLEASE REFER TO THE CONTRIBUTION GUIDE ON THE DOCS PAGE */}}
""".format(
            block_name=block_name,
            slug="blocks/" + block_folder_path.replace("_", "-").lower(),
        )

    def add_python_docs_display(self):
        self.template += """
import docstring from "@blocks/{block_folder_path}/docstring.json";
import PythonDocsDisplay from "@/components/python-docs-display.astro";

<PythonDocsDisplay docstring={{docstring}} />
""".format(
            block_folder_path=self.block_folder_path,
        )
        return self

    def add_python_code(self, python_code):
        self.template += """
<details>
<summary>Python Code</summary>

```python
{python_code}
```

[Find this Flojoy Block on GitHub]({github_link})

</details>
""".format(
            python_code=python_code, github_link=self.github_link
        )
        return self

    def add_example_app(self):
        self.template += """
## Example

import GetHelpWidget from "@/components/get-help-widget.astro";

<GetHelpWidget />

import app from "@blocks/{block_folder_path}/app.json";
import AppDisplay from "@/components/app-display.tsx";

<AppDisplay app={{app}} blockName="{block_name}" client:visible />

import Example from "@blocks/{block_folder_path}/example.md";

<Example />
""".format(
            block_folder_path=self.block_folder_path,
            block_name=self.block_name,
        )
        return self

    def build(self):
        self.template += """
{{/* DO NOT EDIT THIS FILE! IT IS BEING AUTO GENERATED */}}
{{/* PLEASE REFER TO THE CONTRIBUTION GUIDE ON THE DOCS PAGE */}}
""".format()
        return self.template


@app.command()
def sync():
    """
    This sync command will only operate on the blocks folder as well as the
    blocks folder in the docs folder.
    """
    total_synced_pages = 0

    # We would like to NOT modify the following files
    keep_files = ["overview.md"]
    auto_gen_categories = ["NUMPY", "SCIPY"]

    print(f"Cleaning the blocks section except all the {keep_files} files.")
    for root, _, files in os.walk(docs_folder_prefix, topdown=False):
        for file in files:
            if file in keep_files:
                continue
            file_path = os.path.join(root, file)
            os.remove(file_path)

    print("Populating the blocks section...")
    for root, _, files in os.walk(blocks_folder_prefix):
        folder_name = os.path.basename(root)

        for file in files:
            file_name = os.path.splitext(file)[0]

            if file_name == folder_name:
                # In this case we found the Python file that matches the folder

                if state["verbose"]:
                    print(f"Processing {file_name}")

                # example: VISUALIZERS/DATA_STRUCTURE/ARRAY_VIEW
                block_folder_path = root.split("blocks", 1)[1].strip("/")

                block_category = block_folder_path.split("/")[0]

                # Read the Python file
                with open(os.path.join(root, file), "r") as f:
                    python_code = f.read()

                if not os.path.exists(os.path.join(root, "example.md")):
                    if block_category in auto_gen_categories:
                        pass
                    else:
                        print(f"{err_string} No example.md found for {file_name}")
                        sys.exit(1)

                if not os.path.exists(os.path.join(root, "app.json")):
                    if block_category in auto_gen_categories:
                        pass
                    else:
                        print(f"{err_string} No app.json found for {file_name}")
                        sys.exit(1)

                # Create the markdown template file in docs

                target_md_file = docs_folder_prefix + os.path.join(
                    block_folder_path + ".mdx"
                )

                os.makedirs(os.path.dirname(target_md_file), exist_ok=True)

                with open(target_md_file, "w") as f:
                    # Write the content of the markdown file
                    if block_category in auto_gen_categories:
                        result = (
                            DocsTemplateBuilder(
                                block_name=file_name,
                                block_folder_path=block_folder_path,
                            )
                            .add_python_docs_display()
                            .add_python_code(python_code)
                            .build()
                        )
                    else:
                        result = (
                            DocsTemplateBuilder(
                                block_name=file_name,
                                block_folder_path=block_folder_path,
                            )
                            .add_python_docs_display()
                            .add_python_code(python_code)
                            .add_example_app()
                            .build()
                        )
                    f.write(result)

                total_synced_pages += 1

    # Remove all empty folders
    print("Almost done! Doing some housekeeping...")
    for dirpath, dirnames, filenames in os.walk(docs_folder_prefix):
        if (
            not filenames and not dirnames
        ):  # Check if the directory has no files or subdirectories
            os.rmdir(dirpath)  # Remove the directory

    print(f"Successfully synced {total_synced_pages} pages!")


@app.command()
def add(block: str):
    # TODO: Update the add command once everything else is done

    # first we verify if the block name is valid
    parts = block.split(".")
    block_name = parts[-1]
    pattern = r"^(?!^\d)\w+$"
    for part in parts:
        if part == "":
            err_console.print(
                f"{err_string} you cannot have empty part in your block name!"
            )
            sys.exit(1)

        match = re.match(pattern, part)
        if not match:
            err_console.print(
                f"{err_string} {part} is not a valid block name. It can only include alphanumeric characters and underscores. It also cannot start with a number."
            )
            sys.exit(1)

    # if it is valid, we can start scaffolding the boilerplate

    # lastly we finish with the python block code
    blocks_target_path = os.path.join(blocks_folder_prefix, block.replace(".", "/"))
    os.makedirs(blocks_target_path, exist_ok=True)
    with open(os.path.join(blocks_target_path, f"{block_name}.py"), "w+") as f:
        f.write(block_template.format(block_name=block_name))

    print(f"Done! Your Flojoy Block is ready at '{blocks_target_path}'")


@app.callback()
def main(verbose: bool = False):
    if verbose:
        print("Verbose mode is on!")
        state["verbose"] = True


if __name__ == "__main__":
    # this is to make sure we are running the cli in the right directory
    required_folders = [docs_folder_prefix, blocks_folder_prefix]
    if not all([os.path.isdir(folder) for folder in required_folders]):
        err_console.print(
            f"{err_string} fjblock.py must be run at a directory where the following folders are present: {required_folders}"
        )
        sys.exit(1)

    app()
