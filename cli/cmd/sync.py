import json
import os
import sys

import frontmatter
from rich import print

from cli.constants import BLOCKS_DOCS_FOLDER, BLOCKS_SOURCE_FOLDER, ERR_STRING
from cli.state import state
from cli.utils.block_docs import BlockDocsBuilder
from cli.utils.generate_docstring_json import generate_docstring_json
from cli.utils.overview_docs import BlockInfo, CategoryOverviewDocsBuilder, CategoryTree


def remove_empty_folders(top_directory):
    for root, dirs, _ in os.walk(top_directory, topdown=False):
        for dir in dirs:
            folder_path = os.path.join(root, dir)
            if not os.listdir(folder_path):
                os.rmdir(folder_path)


def sync():
    """
    This sync command will only operate on the blocks folder as well as the
    blocks folder in the docs folder.
    """
    total_synced_pages = 0

    # We would like to NOT modify the following files
    keep_files = ["overview.mdx"]
    auto_gen_categories = ["NUMPY", "SCIPY"]

    # Remove all empty folders
    print("House keeping before syncing...")
    for root, _, files in os.walk("."):
        for file in files:
            if file == ".DS_Store":
                file_path = os.path.join(root, file)
                os.remove(file_path)

    remove_empty_folders(BLOCKS_DOCS_FOLDER)
    remove_empty_folders(BLOCKS_SOURCE_FOLDER)

    print("Generating docstring.json for all the blocks...")
    success = generate_docstring_json()
    if not success:
        print(f"{ERR_STRING} Please fix all the docstring errors before syncing.")
        sys.exit(1)

    print(f"Cleaning the blocks section except all the {keep_files} files.")
    for root, _, files in os.walk(BLOCKS_DOCS_FOLDER, topdown=False):
        for file in files:
            if file in keep_files:
                continue
            file_path = os.path.join(root, file)
            os.remove(file_path)

    category_tree: CategoryTree = {}

    print("Populating the blocks section...")
    for root, dirs, files in os.walk(BLOCKS_SOURCE_FOLDER):
        folder_name = os.path.basename(root)
        dirs.sort()

        for file in files:
            file_name = os.path.splitext(file)[0]

            if file_name != folder_name:
                continue

            # In this case we found the Python file that matches the folder

            if state["verbose"]:
                print(f"Processing {file_name}")

            # example: VISUALIZERS/DATA_STRUCTURE/ARRAY_VIEW
            current_block_folder_path = root.split("blocks", 1)[1].strip("/")

            current_block_category = current_block_folder_path.split("/")[0]

            if not os.path.exists(os.path.join(root, "example.md")):
                if current_block_category not in auto_gen_categories:
                    print(f"{ERR_STRING} No example.md found for {file_name}")
                    sys.exit(1)

            if not os.path.exists(os.path.join(root, "app.json")):
                if current_block_category not in auto_gen_categories:
                    print(f"{ERR_STRING} No app.json found for {file_name}")
                    sys.exit(1)

            if not os.path.exists(os.path.join(root, "docstring.json")):
                print(f"{ERR_STRING} No docstring.json found for {file_name}")
                sys.exit(1)

            with open(os.path.join(root, "docstring.json"), "r") as f:
                description = json.load(f)["short_description"]

            # Keep track of the file tree structure in order to generate
            # overview pages for all of the top level categories
            _tree_insert_block(
                category_tree,
                current_block_folder_path,
                BlockInfo(
                    link=current_block_folder_path.split("/", 1)[-1]
                    .replace("_", "-")
                    .lower(),
                    name=file_name,
                    description=description,
                ),
            )

            # Create the markdown template file in docs
            target_md_file = BLOCKS_DOCS_FOLDER + os.path.join(
                current_block_folder_path + ".mdx"
            )

            os.makedirs(os.path.dirname(target_md_file), exist_ok=True)

            with open(target_md_file, "w") as f:
                # Write the content of the markdown file
                result = (
                    BlockDocsBuilder(
                        block_name=file_name,
                        block_folder_path=current_block_folder_path,
                        description=description,
                    )
                    .add_python_docs_display()
                    .add_python_code()
                )

                if current_block_category not in auto_gen_categories:
                    result = result.add_example_app()

                f.write(result.build())

            total_synced_pages += 1

    print("Generating overview pages...")
    required_frontmatters = ["title", "description"]

    found_err = False
    for top_level_category in os.listdir(BLOCKS_SOURCE_FOLDER):
        summary_path = os.path.join(
            BLOCKS_SOURCE_FOLDER, top_level_category, "summary.md"
        )

        overview_title = ""
        overview_desc = ""

        if os.path.exists(summary_path):
            summary = frontmatter.load(summary_path)
            missing = False
            for fm in required_frontmatters:
                if fm not in summary:
                    print(
                        f"{ERR_STRING} frontmatter '{fm}' is missing in summary.md for {top_level_category}"
                    )
                    missing = True
            if missing:
                continue
            overview_title = summary["title"]
            overview_desc = summary["description"]

        else:
            print(
                f"{ERR_STRING} summary.md not found for top level category {top_level_category}!"
            )
            found_err = True

        print(f"Generating overview for {top_level_category}...")

        overview_page_path = os.path.join(
            BLOCKS_DOCS_FOLDER, top_level_category, "overview.mdx"
        )
        with open(overview_page_path, "w+") as f:
            try:
                f.write(
                    CategoryOverviewDocsBuilder(
                        overview_title, top_level_category, overview_desc
                    )
                    .add_content(category_tree[top_level_category])
                    .build()
                )
            except ValueError as e:
                print(str(e))
                found_err = True

    if found_err:
        print(f"{ERR_STRING} Found errors when generating overview pages.")
        sys.exit(1)

    print(f"Successfully synced {total_synced_pages} block pages!")


def _split_path(path: str) -> list[str]:
    parts = []
    while True:
        path, part = os.path.split(path)
        if part:
            parts.insert(0, part)
        else:
            break

    return parts


def _tree_insert_block(tree: dict, directory: str, block: BlockInfo):
    path_parts = _split_path(directory)
    leaf_ancestor = path_parts[-2]
    cur_level = tree

    for part in path_parts[:-2]:
        if part not in cur_level:
            cur_level[part] = {}
        cur_level = cur_level[part]

    if leaf_ancestor not in cur_level:
        cur_level[leaf_ancestor] = []

    cur_level[leaf_ancestor].append(block)
