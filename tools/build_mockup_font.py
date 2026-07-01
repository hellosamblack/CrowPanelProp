#!/usr/bin/env python3
"""Embed Eurostile-Bold.ttf into docs/design/communicator-mockup.html as a base64 data URI.

Makes the mockup fully self-contained (no external font file). Idempotent: re-running
replaces whatever base64 is currently embedded, so you can edit the HTML freely and
re-embed after a font change.

    python tools/build_mockup_font.py
"""
import base64, os, re

HERE = os.path.dirname(os.path.abspath(__file__))
TTF = os.path.join(HERE, "..", "resources", "Eurostile-Bold.ttf")
HTML = os.path.join(HERE, "..", "docs", "design", "communicator-mockup.html")

b64 = base64.b64encode(open(TTF, "rb").read()).decode("ascii")
src = open(HTML, encoding="utf-8").read()
new, n = re.subn(r"(base64,)(?:__EUROSTILE_B64__|[A-Za-z0-9+/=]+)(\) format\(\"truetype\"\))",
                 r"\g<1>" + b64 + r"\g<2>", src, count=1)
if n != 1:
    raise SystemExit("could not find the @font-face base64 slot in the HTML")
open(HTML, "w", encoding="utf-8", newline="\n").write(new)
print("embedded Eurostile-Bold (%d base64 chars) into %s" % (len(b64), os.path.relpath(HTML)))
