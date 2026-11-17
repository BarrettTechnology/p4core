"""p4core.guide: build a Barrett app guide (PDF) from Markdown.

    python -m p4core.guide build docs/guide/guide.md [-o OUT.pdf] [--html]
    python -m p4core.guide shots docs/guide/guide.md [--only NAME,...] [--sim PATH/puck_sim]
    python -m p4core.guide all   docs/guide/guide.md [--sim PATH/puck_sim]      shots, then build
    python -m p4core.guide stale docs/guide/guide.md     exit 1 if the screenshots predate the UI code
"""
import argparse
import sys

from . import build, shots


def main(argv=None):
    ap = argparse.ArgumentParser(prog='python -m p4core.guide', description=__doc__.split('\n')[0])
    ap.add_argument('cmd', choices=['build', 'shots', 'all', 'stale'])
    ap.add_argument('guide')
    ap.add_argument('-o', '--out', default=None, help='PDF path (default: front matter "output", else guide.pdf)')
    ap.add_argument('--html', action='store_true', help='also keep the HTML next to the PDF')
    ap.add_argument('--only', default=None, help='shots: comma list of shot names')
    ap.add_argument('--sim', default=None, help='shots: start this puck_sim for the capture')
    a = ap.parse_args(argv)
    if a.cmd == 'stale':                 # exit 1 + a note when the screenshots predate the UI code
        import os
        meta, _, _ = build.read_guide(a.guide)
        note = build.stale_shots(meta, os.path.dirname(os.path.abspath(a.guide)))
        if note:
            print(note)
        return 1 if note else 0
    if a.cmd in ('shots', 'all'):
        print('screenshots ->', shots.take(a.guide, a.only.split(',') if a.only else None, a.sim))
    if a.cmd in ('build', 'all'):
        print('guide ->', build.build_pdf(a.guide, a.out, a.html))


if __name__ == '__main__':
    sys.exit(main())
