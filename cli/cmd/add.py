import os
import re
import sys

from rich import print

from cli.constants import block_template, blocks_folder_prefix, err_string
from cli.logging import err_console


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