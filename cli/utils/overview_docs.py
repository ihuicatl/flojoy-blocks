import dataclasses

CATEGORY_TEMPLATE = """ \
{header_level} {title}
{content}
"""

OVERVIEW_TEMPLATE_BASE = """\
---
title: {title}
description: "{description}"
slug: {slug}
sidebar:
  order: 0
---

{{/* DO NOT EDIT THIS FILE! IT IS BEING AUTO GENERATED */}}
{{/* PLEASE REFER TO THE CONTRIBUTION GUIDE ON THE DOCS PAGE */}}

import BlockCategory from "@/components/block-category.astro";

"""

TOP_LEVEL_DEPTH = 3


@dataclasses.dataclass
class BlockInfo:
    link: str
    name: str
    description: str


CategoryTree = list[BlockInfo] | dict[str, "CategoryTree"]


def make_category_content(
    name: str, contents: CategoryTree, depth: int = TOP_LEVEL_DEPTH
) -> str:
    match contents:
        # leaf (bottom level category)
        case list():
            blocks = [dataclasses.asdict(b) for b in contents]
            content = f"<BlockCategory blocks={{{blocks}}} />"
        # inner node (recurse on children)
        case dict():
            content = "\n".join(
                make_category_content(key, val, depth + 1)
                for key, val in contents.items()
            )
    # don't show the title of the top level category
    if depth == TOP_LEVEL_DEPTH:
        return content

    return CATEGORY_TEMPLATE.format(
        header_level="#" * depth, title=name, content=content
    )


class CategoryOverviewDocsBuilder:
    def __init__(self, title: str, category_name: str, description: str):
        self.category_name = category_name
        self.template = OVERVIEW_TEMPLATE_BASE.format(
            title=f"{title} Overview",
            description=description.replace("\n", " "),
            slug="blocks/" + category_name.replace("_", "-").lower(),
        )

    def add_content(self, content: CategoryTree):
        self.template += make_category_content(self.category_name, content)
        return self

    def build(self):
        return self.template
