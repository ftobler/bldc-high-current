#!/usr/bin/env python3
"""
Simple C/C++ linter (single-file).

Usage:
    python c_cpp_linter.py <folder> [more folders...]

Scans for typical C/C++ extensions and prints violations to stdout.

Rules are registered using @rule("name") decorator. Add rules by adding functions
that accept (path, lines, text) and return a list of violation dicts:
    {"line": int, "col": int (optional), "message": str, "rule": str}

Implemented rules:
 - indentation_rule: no tabs, leading spaces must be multiples of 4
 - inline_comment_rule: inline comments must have exactly two spaces before '//' and one space after '//'
 - todo_rule: disallow TODO occurrences
 - two_blank_lines_between_functions_rule: best-effort check for >=2 blank lines between top-level functions
"""

from typing import List, Callable, Any
import argparse
import os
import re
import sys

# ---------- Rule registration decorator ----------
RULES: List[Callable[[str, List[str], str], List[dict[str, Any]]]] = []


def rule(name: str):
    def decorator(func):
        func._rule_name = name
        RULES.append(func)
        return func
    return decorator


# ---------- Helpers ----------
EXTS = {".c", ".cpp", ".cc", ".cxx", ".h", ".hpp", ".hh", ".ipp"}


def find_source_files(paths: List[str]) -> List[str]:
    files = []
    for p in paths:
        if os.path.isfile(p):
            if os.path.splitext(p)[1].lower() in EXTS:
                files.append(os.path.abspath(p))
        else:
            for root, _, filenames in os.walk(p):
                for fn in filenames:
                    if os.path.splitext(fn)[1].lower() in EXTS:
                        files.append(os.path.join(root, fn))
    return sorted(files)


def load_file(path: str):
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        text = f.read()
    lines = text.splitlines()
    return lines, text


def print_violation(path: str, v: dict[str, Any]):
    line = v.get("line", "?")
    col = v.get("col")
    loc = f"{path}:{line}" + (f":{col}" if col is not None else "")
    level = v.get("level", "error")  # default to error if not given
    print(f"{loc}: {level}: {v.get('rule')}: {v.get('message')}")


# ---------- Basic utilities for parsing / cleaning ----------
STRING_RE = re.compile(
    r'(\"(\\.|[^"\\])*\"|\'(\\.|[^\'\\])*\'|R\"(?:\([^)]*\))?.*?\"?)',
    re.DOTALL,
)
BLOCK_COMMENT_RE = re.compile(r"/\*.*?\*/", re.DOTALL)
LINE_COMMENT_RE = re.compile(r"//.*?$", re.MULTILINE)


def remove_strings_and_comments(text: str) -> str:
    """
    Remove strings and comments from text to make brace-matching and simple parsing safer.
    This is a heuristic — not a full parser.
    """
    # remove block comments first
    t = BLOCK_COMMENT_RE.sub("", text)
    # remove line comments
    t = LINE_COMMENT_RE.sub("", t)
    # remove strings
    t = STRING_RE.sub(lambda m: '""' if m.group(0).startswith('"') else "''", t)
    return t


def line_col_from_pos(text: str, pos: int):
    """
    Convert pos index in text to (line, col) 1-based.
    """
    before = text[:pos]
    line = before.count("\n") + 1
    last_n = before.rfind("\n")
    col = pos - last_n
    return line, col


# ---------- Rules ----------

@rule("indentation")
def indentation_rule(path: str, lines: List[str], text: str):
    """
    Configurable indentation rule.

    Configure at the top of the function:
      ALLOWED_INDENT = "spaces"  # enforce spaces only (leading spaces must be multiple of 4)
      ALLOWED_INDENT = "tabs"    # enforce tabs only (no leading spaces allowed)
      ALLOWED_INDENT = "both"    # allow either (no leading-space / tab checks)

    Skips:
      - lines inside block comments (/* ... */) including doxygen /** ... */
      - single-line block comments (/* ... */ on one line)
      - full-line // comments

    Reports:
      - tab characters when not allowed
      - leading spaces not multiple of 4 when spaces-only
      - leading spaces present when tabs-only
    """
    # ---------- configuration ----------
    # options: "spaces", "tabs", "both"
    ALLOWED_INDENT = "both"

    viols = []
    in_block = False
    for i, ln in enumerate(lines, start=1):
        s = ln

        # detect block comment start/end positions on the line (if any)
        start_idx = s.find("/*")
        end_idx = s.find("*/")

        # If not currently in a block and we see a block start that is not closed on the same line,
        # enter block mode and skip this line.
        if not in_block and start_idx != -1:
            # if end exists and end comes after start -> single-line block comment, skip but don't enter block
            if end_idx == -1 or end_idx < start_idx:
                # multi-line block starting (no end on same line) -> enter block
                if end_idx == -1:
                    in_block = True
                # skip the line regardless (either single-line block or line that starts a block)
                continue
            else:
                # start and end both on same line (/* ... */) - skip line but do not set in_block
                continue

        # skip lines starting with * (inside /* .. */)
        if s.strip().startswith("*"):
            continue  # it is a block comment type of thing

        # If we're inside a block comment, skip lines until we find the closing */
        if in_block:
            if end_idx != -1:
                # block ends on this line
                in_block = False
            # skip indentation check for block-comment lines
            continue

        # Skip full-line '//' comments
        if s.lstrip().startswith("//"):
            continue

        # At this point, s is considered a code or inline-comment line. We apply indentation checks.
        # Extract leading whitespace
        m = re.match(r"^([ \t]*)(.*)$", s)
        leading = m.group(1) if m else ""
        # rest = m.group(2) if m else s

        # Helper: count leading spaces (only) and detect tabs presence
        n_leading_spaces = leading.count(" ")
        # has_leading_tabs = "\t" in leading
        has_any_tab = "\t" in s

        # Enforce based on ALLOWED_INDENT
        if ALLOWED_INDENT == "spaces":
            # Tabs are not allowed anywhere (leading or embedded)
            if has_any_tab:
                viols.append(
                    {"line": i, "rule": "indentation", "message": "tab character found (tabs are not allowed when ALLOWED_INDENT='spaces')"}
                )
            # If there are leading spaces, they must be multiple of 4
            if n_leading_spaces > 0:
                if n_leading_spaces % 4 != 0:
                    viols.append(
                        {
                            "line": i,
                            "rule": "indentation",
                            "message": f"leading spaces ({n_leading_spaces}) not a multiple of 4 (ALLOWED_INDENT='spaces')",
                        }
                    )

        elif ALLOWED_INDENT == "tabs":
            # Disallow leading spaces (indentation must be tabs)
            if n_leading_spaces > 0:
                viols.append(
                    {
                        "line": i,
                        "rule": "indentation",
                        "message": f"leading spaces ({n_leading_spaces}) found but ALLOWED_INDENT='tabs' (leading indentation should be tabs)",
                    }
                )
            # (Optional) you may want to flag tabs elsewhere in the line; currently we allow tabs in the rest of the line.
            # If you want to disallow spaces anywhere, add a check for " " in s.

        else:  # ALLOWED_INDENT == "both"
            # No checks on leading whitespace style. If you still want to flag mixed leading whitespace (e.g. " \t"),
            # add a check here. For now, we do nothing.
            pass

    return viols


@rule("inline-comment-format")
def inline_comment_rule(path: str, lines: List[str], text: str):
    """
    For inline comments using //:
      - require exactly two spaces before the //
      - require exactly one space after the // before the comment text
    Full-line comments (line starting with optional whitespace then //) are ignored.
    """
    viols = []
    for i, ln in enumerate(lines, start=1):
        stripped = ln.lstrip()
        # ignore full-line comment
        if stripped.startswith("//"):
            continue
        if stripped.startswith("*"):  # inside /* ... */
            continue
        idx = ln.find("//")
        if idx == -1:
            continue
        # ensure '//' is not inside a string literal naive check: count quotes before idx
        # (a very small heuristic; if an odd number of quotes before idx, skip)
        if ln[:idx].count('"') % 2 == 1 or ln[:idx].count("'") % 2 == 1:
            # probably inside a literal; ignore this occurrence
            continue

        # count consecutive spaces immediately before //
        k = idx - 1
        count_spaces = 0
        while k >= 0 and ln[k] == " ":
            count_spaces += 1
            k -= 1

        if count_spaces != 2:
            # check if previous or next line has '//' at the same column
            aligned = False
            prev_idx = None
            next_idx = None
            if i > 1:
                prev_line = lines[i - 2]
                prev_idx = prev_line.find("//")
            if i < len(lines):
                next_line = lines[i]
                next_idx = next_line.find("//")
            if prev_idx == idx or next_idx == idx:
                aligned = True

            if not aligned:
                viols.append(
                    {
                        "line": i,
                        "col": idx + 1,
                        "rule": "inline-comment-format",
                        "message": f"inline '//' should be preceded by exactly two spaces (found {count_spaces})",
                    }
                )
            continue

        # check there's exactly one space after //
        after_idx = idx + 2
        if after_idx >= len(ln):
            # comment with nothing after it — still flag (no space + no text)
            viols.append(
                {
                    "line": i,
                    "col": idx + 1,
                    "rule": "inline-comment-format",
                    "message": "inline '//' must be followed by a single space and text",
                }
            )
            continue
        # there must be one space, and then some non-space character
        if ln[after_idx] != " ":
            viols.append(
                {
                    "line": i,
                    "col": idx + 1,
                    "rule": "inline-comment-format",
                    "message": "inline '//' must be followed by exactly one space",
                }
            )
            continue
        # ensure at least one non-space after that single space
        if after_idx + 1 >= len(ln) or ln[after_idx + 1] == " ":
            viols.append(
                {
                    "line": i,
                    "col": idx + 1,
                    "rule": "inline-comment-format",
                    "message": "inline '//' must be followed by a single space and then non-space text",
                }
            )
    return viols


@rule("no-todo")
def todo_rule(path: str, lines: List[str], text: str):
    """
    Disallow TODO occurrences (case-insensitive) anywhere in the file.
    """
    viols = []
    pat = re.compile(r"\bTODO\b", re.IGNORECASE)
    for i, ln in enumerate(lines, start=1):
        if pat.search(ln):
            viols.append(
                {
                    "line": i,
                    "rule": "no-todo",
                    "level": "warning",
                    "message": "TODO found (TODOs are not allowed in code)"
                }
            )
    return viols


@rule("two-blank-lines-between-functions")
def two_blank_lines_between_functions_rule(path: str, lines: List[str], text: str):
    """
    Enforce: when a line contains only a single closing brace '}' (optionally surrounded by whitespace),
    then:
      - the next two lines (if present) MUST be blank (contain only whitespace),
      - and if a third line exists it MUST be non-blank (contain something).
    Special cases for end-of-file:
      - If the '}' is the last line in the file -> ok.
      - If there's only 1 or 2 lines after the '}', those must be blank (no further non-blank line is required).
    This is a simple, line-based check (not a full parser).
    """
    viols: List[dict[str, Any]] = []

    def is_blank_ln(idx: int) -> bool:
        """Return True if line index (0-based) exists and is blank (only whitespace)."""
        if 0 <= idx < len(lines):
            return lines[idx].strip() == ""
        return False  # non-existent line treated as 'not present' by callers

    # iterate lines (0-based index easier for lookups)
    for idx, ln in enumerate(lines):
        if ln != "}":
            continue

        # compute indices for the next three physical lines
        n = len(lines)
        next1_idx = idx + 1
        next2_idx = idx + 2
        next3_idx = idx + 3

        # Case: '}' is the last line -> allowed
        if next1_idx >= n:
            continue

        # if next line is from a #define -> allowed
        if lines[next1_idx].startswith("#"):
            continue

        # Next line must be blank
        if not is_blank_ln(next1_idx):
            viols.append(
                {
                    "line": next1_idx + 1,
                    "rule": "two-blank-lines-between-functions",
                    "message": "expected blank line immediately after '}' (1/2).",
                }
            )
            # still continue to check further to report all problems
        # If next2 does not exist (EOF after single line), allow provided next1 was blank (or already reported)
        if next2_idx >= n:
            # nothing more to require (EOF reached); continue to next '}' occurrence
            continue

        # if next line is the start of a doxygen comment -> allowed
        if lines[next2_idx] == "/**":
            continue

        # Next second line must be blank
        if not is_blank_ln(next2_idx):
            viols.append(
                {
                    "line": next2_idx + 1,
                    "rule": "two-blank-lines-between-functions",
                    "message": "expected second blank line after '}' (2/2).",
                }
            )
            # still continue to check next3 if present

        # If there's no third line (EOF after two blank lines), that's acceptable
        if next3_idx >= n:
            continue

        # If third line exists it must be non-blank
        if is_blank_ln(next3_idx):
            viols.append(
                {
                    "line": next3_idx + 1,
                    "rule": "two-blank-lines-between-functions",
                    "message": "expected code or non-blank line after two blank lines following '}'.",
                }
            )

    return viols


@rule("brace-placement-style")
def brace_placement_style_rule(path: str, lines: List[str], text: str):
    """
    Rule: Check consistency of brace placement style in the file.

    For every line containing a '{' at the end:
      - If the stripped line is exactly '{', classify it as "newline" style.
      - Otherwise classify it as "inline" style.

    After scanning the file:
      - If the majority are newline style, the file is considered newline style.
      - If the majority are inline style, the file is considered inline style.
      - Ties: default to inline style.
    All occurrences that do not match the chosen style are flagged as violations.
    """
    newline_positions = []
    inline_positions = []

    for i, ln in enumerate(lines, start=1):
        stripped = ln.strip()

        # dont count extern "C"
        if stripped == 'extern "C" {':
            continue

        # don't count comment areas
        if stripped.startswith("//"):
            continue

        if stripped.endswith("{"):
            if stripped == "{":
                newline_positions.append(i)
            else:
                inline_positions.append(i)

    if not newline_positions and not inline_positions:
        return []  # no braces at line end -> nothing to check

    # decide style
    if len(newline_positions) > len(inline_positions):
        chosen = "newline"
    else:
        chosen = "inline"

    viols = []
    if chosen == "newline":
        for line in inline_positions:
            viols.append(
                {
                    "line": line,
                    "rule": "brace-placement-style",
                    "message": "brace should be placed on its own line (newline style expected)",
                }
            )
    else:  # chosen inline
        for line in newline_positions:
            viols.append(
                {
                    "line": line,
                    "rule": "brace-placement-style",
                    "message": "brace should be placed inline with preceding code (inline style expected)",
                }
            )

    return viols


@rule("inline-brace-space")
def inline_brace_space_rule(path: str, lines: List[str], text: str):
    """
    Tiny rule: If a line contains an inline '{' (i.e., not just '{' alone),
    enforce that the '{' is preceded by a space, so the line must end with ' {'.
    """
    viols = []
    for i, ln in enumerate(lines, start=1):
        stripped = ln.strip()
        if not stripped.endswith("{"):
            continue
        if stripped == "{":
            continue  # standalone brace -> skip

        # check that it ends with ' {' except on comments, there it is allowed
        if not ln.rstrip().endswith(" {") and not ln.strip().startswith("//"):
            viols.append(
                {
                    "line": i,
                    "rule": "inline-brace-space",
                    "message": "inline '{' must be preceded by a space (' {').",
                }
            )
    return viols


# ---------- Main checking ----------
def check_file(path: str):
    try:
        lines, text = load_file(path)
    except Exception as e:
        print(f"{path}: error reading file: {e}", file=sys.stderr)
        return

    all_violations: list[dict[str, Any]] = []
    for r in RULES:
        try:
            v = r(path, lines, text)
            if v:
                # attach rule name if not set
                for item in v:
                    if "rule" not in item:
                        item["rule"] = getattr(r, "_rule_name", r.__name__)
                all_violations.extend(v)
        except Exception as e:
            print(f"{path}: rule {getattr(r, '_rule_name', r.__name__)} crashed: {e}", file=sys.stderr)

    for viol in all_violations:
        print_violation(path, viol)


def main(argv):
    ap = argparse.ArgumentParser(description="Simple C/C++ linter (single file).")
    ap.add_argument("paths", nargs="+", help="Files or folders to scan")
    args = ap.parse_args(argv)

    files = find_source_files(args.paths)
    if not files:
        print("No source files found.", file=sys.stderr)
        return 2

    for f in files:
        check_file(f)

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
