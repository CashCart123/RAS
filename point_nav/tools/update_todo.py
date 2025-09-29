#!/usr/bin/env python3
"""Maintain TODO.md split into # TODO/# DONE with stable numbering."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re
from typing import Iterable, List

_TASK_PATTERN = re.compile(
    r"^\s*(?:[-*+]|\d+\.)\s*\[(?P<state>[ xX])\]\s*(?P<body>.*?)(?:\s*<!--.*-->)?\s*$"
)


@dataclass
class Task:
    text: str
    done: bool
    order: int


def read_tasks(lines: Iterable[str]) -> List[Task]:
    tasks: List[Task] = []
    for index, line in enumerate(lines):
        match = _TASK_PATTERN.match(line.rstrip())
        if not match:
            continue
        text = match.group("body").strip()
        done = match.group("state").lower() == "x"
        tasks.append(Task(text=text, done=done, order=index))
    return tasks


def render_section(title: str, tasks: List[Task], checked: bool) -> List[str]:
    lines = [title]
    if tasks:
        box = "x" if checked else " "
        for index, task in enumerate(tasks, start=1):
            lines.append(f"{index}. [{box}] {task.text}")
    else:
        lines.append("_None_")
    return lines


def render(tasks: List[Task]) -> str:
    todo_tasks = [task for task in tasks if not task.done]
    done_tasks = [task for task in tasks if task.done]

    todo_tasks.sort(key=lambda task: task.order)
    done_tasks.sort(key=lambda task: task.order)

    lines: List[str] = []
    lines.extend(render_section("# TODO", todo_tasks, checked=False))
    lines.append("")
    lines.extend(render_section("# DONE", done_tasks, checked=True))
    lines.append("")
    return "\n".join(lines)


def load_tasks(todo_path: Path) -> List[Task]:
    lines = todo_path.read_text(encoding="utf-8").splitlines()
    return read_tasks(lines)


def write_output(todo_path: Path, content: str) -> None:
    todo_path.write_text(content, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser("Regenerate point_nav/TODO.md in canonical form.")
    default_path = Path(__file__).resolve().parents[1] / "TODO.md"
    parser.add_argument("--todo-path", type=Path, default=default_path, help="Path to the TODO markdown file.")
    parser.add_argument("--dry-run", action="store_true", help="Print the updated checklist instead of writing it.")
    args = parser.parse_args()

    todo_path = args.todo_path
    if not todo_path.exists():
        parser.error(f"TODO file not found: {todo_path}")

    tasks = load_tasks(todo_path)
    output = render(tasks)

    if args.dry_run:
        print(output)
    else:
        write_output(todo_path, output)


if __name__ == "__main__":
    main()
