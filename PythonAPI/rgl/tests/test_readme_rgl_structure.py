#!/usr/bin/env python3
"""README_RGL.md (English) and README_RGL.ja.md (its Japanese translation) keep the same structure.

Compared outside fenced code blocks: the headings (level, in order) and their count, the number of fenced code blocks,
and the number of rows of each table. A '#' inside a code block is not a heading. Code blocks are not translated, so
their contents must be identical. Every in-page link (#anchor) must name a heading of its own file, and each file links
to the other near its top. Runs with plain pytest; no CARLA server.
"""
import pathlib
import re

import pytest

REPO = pathlib.Path(__file__).resolve().parents[3]
EN = REPO / "README_RGL.md"
JA = REPO / "README_RGL.ja.md"
TOP_LINES = 6   # the cross link sits within the first lines of each file

_QUOTE = re.compile(r"^[ ]{0,3}>[ ]?")
_FENCE = re.compile(r"^[ \t]*(`{3,}|~{3,})(.*)$")
_ATX = re.compile(r"^[ ]{0,3}(#{1,6})(?:[ \t]+(.*?))?(?:[ \t]+#+)?[ \t]*$")
_SETEXT = re.compile(r"^[ ]{0,3}(=+|-+)[ \t]*$")
_LINK = re.compile(r"\]\(#([^)\s]+)\)")
_INLINE_LINK = re.compile(r"\[([^\]]*)\]\([^)]*\)")


def _unquote(line):
    """The line without its block-quote markers ('> ')."""
    while True:
        m = _QUOTE.match(line)
        if not m:
            return line
        line = line[m.end():]


def slug(title):
    """GitHub's heading anchor: inline code and link markup dropped, lower case, punctuation removed, blanks -> '-'."""
    text = _INLINE_LINK.sub(r"\1", title).replace("`", "").lower()
    text = re.sub(r"[^\w\- ]", "", text)
    return text.replace(" ", "-")


def parse(text):
    """The structure of one Markdown file.

    Returns a dict: headings = [(level, title)], blocks = [(info string, body)], tables = [row count per table],
    links = [in-page anchors]. Raises ValueError on a code block that is never closed.
    """
    headings, blocks, tables, links = [], [], [], []
    fence = None      # [char, length, info, body lines] while inside a fenced block
    rows = 0          # rows of the table being read
    prev = ""         # the previous line when it can be a setext heading's text, else ""
    for raw in text.splitlines():
        line = _unquote(raw)
        if fence is not None:
            m = _FENCE.match(line)
            if m and m.group(1)[0] == fence[0] and len(m.group(1)) >= fence[1] and not m.group(2).strip():
                blocks.append((fence[2], "\n".join(fence[3])))
                fence = None
            else:
                fence[3].append(line)
            continue
        is_row = line.strip().startswith("|")
        if rows and not is_row:
            tables.append(rows)
            rows = 0
        m = _FENCE.match(line)
        if m:
            fence = [m.group(1)[0], len(m.group(1)), m.group(2).strip(), []]
            prev = ""
            continue
        links.extend(_LINK.findall(line))
        if is_row:
            rows += 1
            prev = ""
            continue
        m = _ATX.match(line)
        if m:
            headings.append((len(m.group(1)), (m.group(2) or "").strip()))
            prev = ""
            continue
        m = _SETEXT.match(line)
        if m and prev:
            headings.append((1 if m.group(1)[0] == "=" else 2, prev))
            prev = ""
            continue
        prev = line.strip()
    if fence is not None:
        raise ValueError(f"code block opened with {fence[0] * fence[1]!r} is never closed")
    if rows:
        tables.append(rows)
    return {"headings": headings, "blocks": blocks, "tables": tables, "links": links}


def _read(path):
    if not path.is_file():
        pytest.fail(f"{path.name} is missing")
    return path.read_text(encoding="utf-8")


@pytest.fixture(scope="module", name="en")
def english_readme():
    """Parse the English README for the structure comparisons."""
    return parse(_read(EN))


@pytest.fixture(scope="module", name="ja")
def japanese_readme():
    """Parse the Japanese README for the structure comparisons."""
    return parse(_read(JA))


def test_parser_rules():
    """Verify parser rules."""
    text = "\n".join([
        "Title", "=====", "",
        "## Setup", "",
        "```sh", "# not a heading", "```", "",
        "> ```bash", "> # not a heading either", "> ```", "",
        "| a | b |", "|---|---|", "| 1 | 2 |", "",
        "Sub", "---", "",
        "See [Setup](#setup).",
    ])
    s = parse(text)
    assert s["headings"] == [(1, "Title"), (2, "Setup"), (2, "Sub")]
    assert s["blocks"] == [("sh", "# not a heading"), ("bash", "# not a heading either")]
    assert s["tables"] == [3]
    assert s["links"] == ["setup"]
    with pytest.raises(ValueError):
        parse("```\nnever closed\n")


def test_slug_matches_github():
    """Verify slug matches github."""
    assert slug("Skeletal meshes (vehicle bodies, walkers)") == "skeletal-meshes-vehicle-bodies-walkers"
    assert slug("UDP Raw Packet extension (optional)") == "udp-raw-packet-extension-optional"
    assert slug("Step 1 — `prepare`") == "step-1--prepare"


def test_english_structure_is_not_trivial(en):
    # A parser that saw nothing would make the comparisons below pass vacuously.
    """Verify english structure is not trivial."""
    assert len(en["headings"]) >= 20 and len(en["blocks"]) >= 10 and len(en["tables"]) >= 5
    assert (2, "World Partition maps") in en["headings"]


def test_headings_match(en, ja):
    """Verify headings match."""
    levels_en = [lvl for lvl, _ in en["headings"]]
    levels_ja = [lvl for lvl, _ in ja["headings"]]
    for i, (a, b) in enumerate(zip(en["headings"], ja["headings"])):
        # Preserve percent-format numeric coercion and invalid-input errors.
        # pylint: disable-next=consider-using-f-string
        assert a[0] == b[0], "heading %d: level %d %r (en) vs level %d %r (ja)" % (i + 1, a[0], a[1], b[0], b[1])
    assert len(levels_en) == len(levels_ja), f"headings: {len(levels_en)} (en) vs {len(levels_ja)} (ja)"


def test_code_blocks_match(en, ja):
    """Verify code blocks match."""
    assert len(en["blocks"]) == len(ja["blocks"]), \
        f"code blocks: {len(en['blocks'])} (en) vs {len(ja['blocks'])} (ja)"
    for i, (a, b) in enumerate(zip(en["blocks"], ja["blocks"])):
        assert a == b, f"code block {i + 1} differs (code, commands and log lines are not translated)"


def test_table_rows_match(en, ja):
    """Verify table rows match."""
    assert en["tables"] == ja["tables"], f"rows per table: {en['tables']} (en) vs {ja['tables']} (ja)"


@pytest.mark.parametrize("path", [EN, JA], ids=["en", "ja"])
def test_in_page_links_resolve(path):
    """Verify in page links resolve."""
    s = parse(_read(path))
    anchors, seen = set(), {}
    for _, title in s["headings"]:
        base = slug(title)
        n = seen.get(base, 0)
        anchors.add(base if n == 0 else f"{base}-{n}")
        seen[base] = n + 1
    missing = [a for a in s["links"] if a not in anchors]
    assert not missing, f"{path.name}: links to no heading: {missing}"


@pytest.mark.parametrize("path,other", [(EN, JA.name), (JA, EN.name)], ids=["en", "ja"])
def test_cross_link_at_top(path, other):
    """Verify cross link at top."""
    top = _read(path).splitlines()[:TOP_LINES]
    # Preserve percent-format numeric coercion and invalid-input errors.
    # pylint: disable-next=consider-using-f-string
    assert any(f"]({other})" in line for line in top), "%s: no link to %s in its first %d lines" % (
        path.name, other, TOP_LINES)
