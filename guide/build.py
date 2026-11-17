"""guide.md -> guide.html -> guide.pdf (headless Chrome / Chromium / Edge).

The source format (one Markdown file per app guide) is documented in README.md next to this
file. Screenshots come from <guide dir>/shots/<name>.png + .json, made by `python -m p4core.guide shots`.
"""
import html
import json
import pathlib
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, 'assets')
MAX_WIN_PX = 640          # CSS px: 8.5 in page - margins - callout room


def uri(path):
    """file:// URI of a local path (Windows drive letters and spaces included)."""
    return pathlib.Path(path).resolve().as_uri()


# ----------------------------------------------------------------------------- parsing
def read_guide(path):
    """(meta dict, pages: list of block lists, shots: {name: [steps]})."""
    text = open(path, encoding='utf-8').read().replace('\r\n', '\n')
    meta = {}
    if text.startswith('---\n'):
        head, text = text[4:].split('\n---\n', 1)
        for line in head.splitlines():
            if ':' in line and not line.lstrip().startswith('#'):
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
    shots = {}

    def take_shot(m):
        shots[m.group(1)] = m.group(2).strip('\n').splitlines()
        return ''
    text = re.sub(r'```shot\s+(\S+)\n(.*?)```', take_shot, text, flags=re.S)
    pages = [parse_blocks(p) for p in re.split(r'^<!--\s*page\s*-->\s*$', text, flags=re.M)]
    return meta, [p for p in pages if p], shots


ITEM = re.compile(r'^(\d+)\.\s+(.*)$')
TARGET = re.compile(r'\s*\[@([^\]]+?)(?::(left|right|above|below|row))?\]\s*$')


def parse_blocks(text):
    blocks, lines, i = [], text.split('\n'), 0
    while i < len(lines):
        ln = lines[i].rstrip()
        s = ln.strip()
        i += 1
        if re.match(r'<!--\s*row\s*-->$', s):
            blocks.append({'t': 'rowstart'})
            continue
        if re.match(r'<!--\s*/row\s*-->$', s):
            blocks.append({'t': 'rowend'})
            continue
        if not s or s.startswith('<!--'):
            continue
        if s.startswith('### '):
            blocks.append({'t': 'card', 'title': s[4:], 'lines': []})
            while i < len(lines) and not lines[i].strip().startswith('#') and not lines[i].strip().startswith('<!--'):
                if lines[i].strip():
                    blocks[-1]['lines'].append(lines[i].strip())
                i += 1
        elif s.startswith('## '):
            blocks.append({'t': 'pill', 'text': s[3:]})
        elif s.startswith('# '):
            blocks.append({'t': 'panel', 'text': s[2:]})
        elif s.startswith('!['):
            m = re.match(r'!\[(.*?)\]\((shot|menu|img):([^)]+)\)', s)
            if m:
                blocks.append({'t': m.group(2), 'alt': m.group(1), 'ref': m.group(3)})
        elif s.startswith('->'):
            blocks.append({'t': 'next', 'text': s[2:].strip()})
        elif ITEM.match(s):
            m = ITEM.match(s)
            body, target, side = m.group(2), None, None
            t = TARGET.search(body)
            if t:
                target, side = t.group(1).strip(), t.group(2)
                body = body[:t.start()]
            notes = []
            while i < len(lines) and lines[i].startswith((' ', '\t')) and lines[i].strip():
                notes.append(lines[i].strip())
                i += 1
            blocks.append({'t': 'item', 'n': int(m.group(1)), 'body': body, 'notes': notes,
                           'target': target, 'side': side})
        else:
            para = [s]
            while i < len(lines) and lines[i].strip() and not re.match(r'\s*(#|!\[|->|\d+\.\s|<!--)', lines[i]):
                para.append(lines[i].strip())
                i += 1
            blocks.append({'t': 'p', 'text': ' '.join(para)})
    return blocks


def inline(s):
    s = html.escape(s, quote=False)
    s = re.sub(r'\*\*(.+?)\*\*', r'<b>\1</b>', s)
    s = re.sub(r'(?<!\w)_(.+?)_(?!\w)', r'<i>\1</i>', s)
    s = re.sub(r'`(.+?)`', r'<code>\1</code>', s)
    return s


# ----------------------------------------------------------------------------- rendering
def _shot(shots_dir, name):
    j = os.path.join(shots_dir, name + '.json')
    if not os.path.isfile(j):
        raise SystemExit('screenshot "{}" missing ({}): run `python -m p4core.guide shots` first'.format(name, j))
    d = json.load(open(j))
    d['png'] = os.path.join(shots_dir, name + '.png')
    return d


def window_html(d, items, scale, crop=None):
    """The app window: title bar, menu bar, the captured client area, status bar, and one
    callout per item whose target is a widget (f.<name>) or a menu (menu:<label>).
    crop=(x, y, w, h): only that part of the client area, larger, without the window frame."""
    if crop:
        return crop_html(d, scale, crop)
    w, h = d['size']
    scale = min(scale, MAX_WIN_PX / float(w))   # a wide window is scaled to the page width
    out = ['<div class="win" style="width:calc({:.1f}px * var(--shot))">'.format(w * scale)]
    out.append('<div class="win-title">{}</div>'.format(html.escape(d['title'])))
    if d.get('menus'):
        out.append('<div class="win-menu">' + ''.join(
            '<span data-anchor="menu:{0}">{0}</span>'.format(html.escape(m['label'])) for m in d['menus']) + '</div>')
    out.append('<div class="win-client" style="width:calc({:.1f}px * var(--shot));height:calc({:.1f}px * var(--shot))">'.format(w * scale, h * scale))
    out.append('<img src="{}" style="width:100%;height:100%">'.format(uri(d['png'])))
    for name, (x, y, rw, rh) in d['widgets'].items():
        if y >= 0 and y < h:
            out.append('<i class="hot" data-anchor="{}" style="left:{:.2f}%;top:{:.2f}%;width:{:.2f}%;height:{:.2f}%"></i>'
                       .format(html.escape(name), 100 * x / w, 100 * y / h, 100 * rw / w, 100 * rh / h))
    out.append('</div>')
    st = [s for s in d.get('status') or [] if s]
    if d.get('status') is not None and len(d['status']):
        out.append('<div class="win-status">{}</div>'.format(html.escape('   '.join(st)) or '&nbsp;'))
    out.append('</div>')
    return ''.join(out)


def crop_html(d, scale, crop):
    cx, cy, cw, ch = crop
    W, H = d['size']
    scale = min(scale, MAX_WIN_PX / float(cw))
    out = ['<div class="win crop" style="width:calc({0:.1f}px * var(--shot))">'.format(cw * scale),
           '<div class="win-client" style="width:calc({:.1f}px * var(--shot));height:calc({:.1f}px * var(--shot))">'
           .format(cw * scale, ch * scale),
           '<img src="{}" style="position:absolute;left:{:.3f}%;top:{:.3f}%;width:{:.3f}%;height:{:.3f}%">'.format(
               uri(d['png']), -100.0 * cx / cw, -100.0 * cy / ch, 100.0 * W / cw, 100.0 * H / ch)]
    for name, (x, y, rw, rh) in d['widgets'].items():
        if x >= cx and y >= cy and x + rw <= cx + cw + 1 and y + rh <= cy + ch + 1:
            out.append('<i class="hot" data-anchor="{}" style="left:{:.2f}%;top:{:.2f}%;width:{:.2f}%;height:{:.2f}%"></i>'
                       .format(html.escape(name), 100 * (x - cx) / cw, 100 * (y - cy) / ch, 100 * rw / cw, 100 * rh / ch))
    out.append('</div></div>')
    return ''.join(out)


def crop_anchors(d, crop):
    cx, cy, cw, ch = crop
    return {k for k, (x, y, rw, rh) in d['widgets'].items()
            if x >= cx and y >= cy and x + rw <= cx + cw + 1 and y + rh <= cy + ch + 1}


def img_html(gdir, ref, items):
    """A photo figure: `img:main.png | extra.png extra2.png` (files in <guide>/images/): the main
    image, then a row of the others. Callouts: [@pt:x,y] = x %, y % on the main image,
    [@img:extra.png] = one of the row's images. Options after the files: width=4.2in."""
    m = re.search(r'\bwidth=([\d.]+(?:in|px|%))', ref)
    ref = re.sub(r'\bwidth=\S+', '', ref)
    main, _, rest = ref.partition('|')
    main, extras = main.strip(), rest.split()
    for f in [main] + extras:
        if not os.path.isfile(os.path.join(gdir, 'images', f)):
            PROBLEMS.append('image {} not found in {}'.format(f, os.path.join(gdir, 'images')))
            return '', set()
    out = ['<div class="imgfig" style="width:{}">'.format(m.group(1) if m else '100%'),
           '<div class="imgbox"><img src="{}">'.format(uri(os.path.join(gdir, 'images', main)))]
    anchors = set()
    for it in items:
        p = re.match(r'pt:\s*([\d.]+)\s*,\s*([\d.]+)$', it['anchor'])
        if p and 0 <= float(p.group(1)) <= 100 and 0 <= float(p.group(2)) <= 100:
            out.append('<i class="hot" data-anchor="{}" style="left:{}%;top:{}%;width:0;height:0"></i>'.format(
                html.escape(it['anchor']), p.group(1), p.group(2)))
            anchors.add(it['anchor'])
    out.append('</div>')
    if extras:
        out.append('<div class="imgrow">' + ''.join(
            '<div class="imgcell" data-anchor="img:{0}"><img src="{1}"></div>'.format(
                html.escape(f), uri(os.path.join(gdir, 'images', f))) for f in extras) + '</div>')
        anchors |= {'img:' + f for f in extras}
    out.append('</div>')
    return ''.join(out), anchors


def menu_html(d, label):
    m = next((m for m in d.get('menus', []) if m['label'] == label), None)
    if m is None:
        raise SystemExit('menu "{}" not in screenshot {}'.format(label, d['name']))

    def rows(items, depth=0):
        r = []
        for it in items:
            if it.get('sep'):
                r.append('<div class="mi sep"></div>')
                continue
            r.append('<div class="mi d{}" data-anchor="item:{}"><span>{}</span><em>{}</em></div>'.format(
                depth, html.escape(it['label']), html.escape(it['label']),
                html.escape(it.get('accel') or '') if not it.get('items') else ICON_SUB))
            if it.get('items') and depth == 0:
                r.extend(rows(it['items'], 1))
        return r
    return '<div class="menu"><div class="menu-head">{}</div>{}</div>'.format(html.escape(label), ''.join(rows(m['items'])))


def item_html(b):
    title, rest = b['body'], ''
    m = re.match(r'\*\*(.+?)\*\*\s*[-–—]?\s*(.*)', b['body'])
    if m:
        title, rest = m.group(1), m.group(2)
    notes = ''.join('<div class="note">{}</div>'.format(inline(n)) for n in b['notes'])
    return ('<div class="item"><div class="num">{}</div><div><div class="it"><b>{}</b>{}</div>{}</div></div>'
            .format(b['n'], inline(title), ' - ' + inline(rest) if rest else '', notes))


# drawn icons, so nothing depends on the fonts installed on the building machine
_SVG = '<svg class="ic" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">{}</svg>'
ICON_PHONE = _SVG.format('<path d="M5 3h4l2 5-2.5 1.5a11 11 0 0 0 6 6L16 13l5 2v4a2 2 0 0 1-2 2A16 16 0 0 1 3 5a2 2 0 0 1 2-2z"/>')
ICON_WEB = _SVG.format('<circle cx="12" cy="12" r="9"/><path d="M3 12h18M12 3c2.5 3 2.5 15 0 18M12 3c-2.5 3-2.5 15 0 18"/>')
ICON_MAIL = _SVG.format('<rect x="3" y="5" width="18" height="14" rx="2"/><path d="m3 7 9 6 9-6"/>')
ICON_DOWN = _SVG.format('<path d="M12 4v15M6 13l6 6 6-6"/>')
ICON_SUB = '<svg class="ic sub" viewBox="0 0 24 24"><path d="M9 6l7 6-7 6z" fill="currentColor"/></svg>'

ICON_WARN = ('<svg viewBox="0 0 24 24"><path d="M12 3 2 21h20L12 3z" fill="none" stroke="currentColor" '
             'stroke-width="1.8" stroke-linejoin="round"/><path d="M12 10v5M12 17.5v.5" stroke="currentColor" '
             'stroke-width="1.8" stroke-linecap="round"/></svg>')
ICON_TIP = ('<svg viewBox="0 0 24 24"><path d="M9 18h6M10 21h4M12 3a6 6 0 0 0-3.5 10.9c.6.5 1 1.2 1 2.1h5c0-.9.4-1.6 '
            '1-2.1A6 6 0 0 0 12 3z" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linejoin="round"/>'
            '<path d="M12 0v1.5M3.5 3.5l1 1M20.5 3.5l-1 1M0 11h1.5M22.5 11H24" stroke="currentColor" stroke-width="1.5"/></svg>')


def card_html(b):
    body = []
    for ln in b['lines']:
        if ln.startswith('! '):
            body.append('<div class="cl warn">{}<p>{}</p></div>'.format(ICON_WARN, inline(ln[2:])))
        elif ln.startswith('+ '):
            body.append('<div class="cl tip">{}<p>{}</p></div>'.format(ICON_TIP, inline(ln[2:])))
        else:
            body.append('<p>{}</p>'.format(inline(ln)))
    return '<div class="card"><h3>{}</h3>{}</div>'.format(inline(b['title']), ''.join(body))


def page_html(meta, blocks, n, total, shots_dir):
    gdir = os.path.dirname(shots_dir)
    figs, body, items_for_fig, panel_open, items_open = [], [], [], False, False
    scale = float(meta.get('shot_scale', 1.0))

    def close_items():
        nonlocal items_open
        if items_open:
            body.append('</div>')
            items_open = False

    def close_panel():
        nonlocal panel_open
        close_items()
        if panel_open:
            body.append('</div>')
            panel_open = False
        nonlocal row_open
        if row_open:
            body.append('</div>')
            row_open = False

    current_fig = None
    cards = []
    row_open = False
    explicit_row = False
    for b in blocks:
        t = b['t']
        if t != 'card' and cards:
            body.append('<div class="cards">' + ''.join(cards) + '</div>')
            cards = []
        if t == 'rowstart':
            close_panel()
            body.append('<div class="row explicit">')
            explicit_row = True
            continue
        if t == 'rowend':
            close_panel()
            if explicit_row:
                body.append('</div>')
                explicit_row = False
            continue
        if t == 'img':
            close_panel()
            current_fig = {'img': b['ref'], 'items': [], 'what': 'image ' + b['ref'].split('|')[0].strip()}
            figs.append(current_fig)
            body.append('<div class="fig photofig" data-fig="{}">{{FIG{}}}</div>'.format(len(figs) - 1, len(figs) - 1))
            continue
        if t == 'shot':
            close_panel()
            ref, _, opt = b['ref'].partition(' ')
            m = re.search(r'crop=(\d+),(\d+),(\d+),(\d+)', opt)
            crop = tuple(int(v) for v in m.groups()) if m else None
            d = _shot(shots_dir, ref)
            w_, h_ = d['size']
            anchors = crop_anchors(d, crop) if crop else (
                {k for k, (x, y, rw, rh) in d['widgets'].items() if 0 <= y < h_}
                | {'menu:' + m['label'] for m in d.get('menus', [])})
            current_fig = {'html': window_html(d, [], scale, crop), 'items': [], 'd': d,
                           'what': 'screenshot ' + ref + (' (cropped)' if crop else ''), 'anchors': anchors}
            figs.append(current_fig)
            body.append('<div class="fig" data-fig="{}">{{FIG{}}}</div>'.format(len(figs) - 1, len(figs) - 1))
        elif t == 'menu':
            close_items()
            label, shot = (b['ref'].split('@') + [None])[:2]
            d = _shot(shots_dir, shot) if shot else (current_fig or {}).get('d')
            if d is None:
                raise SystemExit('menu:{} needs a screenshot (menu:Label@shot)'.format(label))
            menu = next((m for m in d.get('menus', []) if m['label'] == label), {'items': []})
            current_fig = {'html': menu_html(d, label), 'items': [], 'd': d, 'what': 'the {} menu'.format(label),
                           'anchors': {'item:' + it['label'] for it in menu['items'] if not it.get('sep')}
                           | {'item:' + sub['label'] for it in menu['items'] for sub in it.get('items', [])}}
            figs.append(current_fig)
            close_panel()
            body.append('<div class="row">')
            row_open = True
            body.append('<div class="fig menufig" data-fig="{}">{{FIG{}}}</div>'.format(len(figs) - 1, len(figs) - 1))
        elif t == 'panel':
            if not row_open:
                close_panel()
            body.append('<div class="panel"><div class="panel-title">{}</div>'.format(inline(b['text'])))
            panel_open = True
        elif t == 'pill':
            close_items()
            body.append('<div class="pill">{}</div>'.format(inline(b['text'])))
        elif t == 'item':
            if not items_open:
                body.append('<div class="items">')
                items_open = True
            body.append(item_html(b))
            if b['target'] and current_fig is not None:
                current_fig['items'].append({'n': b['n'], 'anchor': b['target'], 'side': b['side']})
        elif t == 'card':
            close_panel()
            cards.append(card_html(b))
        elif t == 'next':
            close_items()
            body.append('<div class="next">{} <span>{}</span></div>'.format(inline(b['text']), ICON_DOWN))
        elif t == 'p':
            close_items()
            body.append('<p class="para">{}</p>'.format(inline(b['text'])))
    if cards:
        body.append('<div class="cards">' + ''.join(cards) + '</div>')
    close_panel()
    for fg in figs:                       # photo figures: their html and anchors need the items
        if 'img' in fg:
            fg['html'], fg['anchors'] = img_html(gdir, fg['img'], fg['items'])
    if explicit_row:
        body.append('</div>')
    for fg in figs:                       # every callout must land on its figure
        have = fg.get('anchors', set())
        for it in fg['items']:
            if it['anchor'] not in have:
                PROBLEMS.append('page {}: item {} points at [@{}], which is not on {}'.format(
                    n, it['n'], it['anchor'], fg.get('what', 'the figure')))
    out = ''.join(body)
    for k, fg in enumerate(figs):
        out = out.replace('{{FIG{}}}'.format(k), fg['html'] + '<script type="application/json" class="callouts">{}</script>'
                          .format(json.dumps(fg['items'])))
    contact = ''.join('<span>{}{}</span>'.format(ic, html.escape(meta[k])) for k, ic in (
        ('phone', ICON_PHONE), ('web', ICON_WEB), ('email', ICON_MAIL)) if meta.get(k))
    return '''<section class="page">
<header><img class="logo" src="{logo}"><h1>{title}</h1><p>{intro}</p><div class="pageno">Page {n} of {total}</div></header>
<main>{body}</main>
<footer><div class="contact"><b>Contact Us</b>{contact}</div><div class="docno">Document {doc} Rev {rev}{ver}</div></footer>
</section>'''.format(logo=uri(os.path.join(ASSETS, meta.get('logo', 'logo-drive-systems.png'))),
                     title=inline(meta.get('title', '')), intro=inline(meta.get('intro', '')), n=n, total=total,
                     body=out, contact=contact, doc=html.escape(meta.get('document', '')),
                     rev=html.escape(meta.get('revision', '')),
                     ver='  -  {} {}'.format(html.escape(meta.get('app', '')), html.escape(meta['app_version']))
                     if meta.get('app_version') else '')


PROBLEMS = []


def stale_shots(meta, gdir):
    """A note when the committed screenshots are older than the app's UI code (git commit times of
    shots/ against the `ui_sources:` paths, space-separated, relative to guide.md), else None."""
    srcs = [os.path.normpath(os.path.join(gdir, p)) for p in meta.get('ui_sources', '').split()]
    if not srcs:
        return None

    def last(paths):
        r = subprocess.run(['git', 'log', '-1', '--format=%ct', '--'] + paths, cwd=gdir,
                           capture_output=True, text=True)
        return int(r.stdout.strip() or 0) if r.returncode == 0 else 0
    ui, shots = last(srcs), last([os.path.join(gdir, 'shots')])
    if ui and (not shots or shots < ui):
        dirty = subprocess.run(['git', 'status', '--porcelain', '--', os.path.join(gdir, 'shots')], cwd=gdir,
                               capture_output=True, text=True).stdout.strip()
        if not dirty:                   # new screenshots not committed yet count as up to date
            return 'the screenshots were committed before the last change to the app UI ({})'.format(
                ', '.join(os.path.relpath(p, gdir) for p in srcs))
    return None


def page_text(blocks):
    """The words a page shows (titles, items, notes, cards), for the spelling check."""
    out = []
    for b in blocks:
        out += [b.get('text', ''), b.get('title', ''), b.get('body', '')] + b.get('notes', []) + b.get('lines', [])
    return re.sub(r'[*_`]', ' ', '\n'.join(t for t in out if t))


def spelling(meta, pages, gdir):
    """Unknown words, via aspell (en_US) with the shared word list (words.txt here) and the
    guide's own (docs/guide/words.txt). [] and a note when aspell is not installed."""
    exe = shutil.which('aspell')
    if exe is None:
        print('note: aspell not installed: spelling not checked')
        return []
    known = set()
    for f in (os.path.join(HERE, 'words.txt'), os.path.join(gdir, 'words.txt')):
        if os.path.isfile(f):
            known |= {w.strip().lower() for w in open(f, encoding='utf-8') if w.strip() and not w.startswith('#')}
    bad = []
    for n, blocks in enumerate(pages, 1):
        text = page_text(blocks) + ('\n' + meta.get('title', '') + '\n' + meta.get('intro', '') if n == 1 else '')
        r = subprocess.run([exe, 'list', '--lang=en_US', '--ignore-case'], input=text, capture_output=True, text=True)
        for w in sorted(set(r.stdout.split())):
            if w.lower() not in known:
                bad.append('page {}: "{}"'.format(n, w))
    return bad


def _app_version(meta, gdir):
    """app_version: 1.4.2, or `from ../../puckutility/version.py` (its __version__)."""
    v = meta.get('app_version', '')
    if v.startswith('from '):
        path = os.path.normpath(os.path.join(gdir, v[5:].strip()))
        m = re.search(r'__version__\s*=\s*[\'"]([^\'"]+)', open(path, encoding='utf-8').read())
        if not m:
            raise SystemExit('no __version__ in ' + path)
        meta['app_version'] = m.group(1)


def build_html(guide_path, html_path=None):
    meta, pages, _ = read_guide(guide_path)
    gdir = os.path.dirname(os.path.abspath(guide_path))
    _app_version(meta, gdir)
    del PROBLEMS[:]
    if meta.get('spelling', 'on') != 'off':
        wl = os.path.join(os.path.relpath(gdir), 'words.txt')
        PROBLEMS.extend('spelling, {} (a real word? add it to {})'.format(b, wl)
                        for b in spelling(meta, pages, gdir))
    shots_dir = os.path.join(gdir, 'shots')
    css = open(os.path.join(HERE, 'guide.css')).read().replace('{ASSETS}', uri(ASSETS))
    js = open(os.path.join(HERE, 'callouts.js')).read()
    doc = '<!doctype html><html><head><meta charset="utf-8"><title>{}</title><style>{}</style></head><body>{}<script>{}</script></body></html>'.format(
        html.escape(meta.get('title', 'Guide')), css,
        ''.join(page_html(meta, p, i + 1, len(pages), shots_dir) for i, p in enumerate(pages)), js)
    html_path = html_path or os.path.splitext(guide_path)[0] + '.html'
    with open(html_path, 'w', encoding='utf-8') as f:
        f.write(doc)
    if PROBLEMS:
        raise SystemExit('guide not built:\n  ' + '\n  '.join(PROBLEMS))
    return meta, html_path


def find_chrome():
    for c in (os.environ.get('P4GUIDE_CHROME'), 'google-chrome', 'chromium', 'chromium-browser', 'msedge',
              r'C:\Program Files\Google\Chrome\Application\chrome.exe',
              r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe',
              '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'):
        if c and (shutil.which(c) or os.path.isfile(c)):
            return shutil.which(c) or c
    raise SystemExit('no Chrome / Chromium / Edge found (set P4GUIDE_CHROME to its path)')


def build_pdf(guide_path, pdf_path=None, keep_html=False):
    tmp = tempfile.mkdtemp(prefix='p4guide-')
    try:
        return _build_pdf(guide_path, pdf_path, keep_html, tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)       # html, Chrome profile, PDF draft: always removed


def _build_pdf(guide_path, pdf_path, keep_html, tmp):
    meta, html_path = build_html(guide_path, os.path.join(tmp, 'guide.html'))
    pdf_path = os.path.abspath(pdf_path or meta.get('output') and os.path.join(
        os.path.dirname(os.path.abspath(guide_path)), meta['output']) or os.path.splitext(guide_path)[0] + '.pdf')
    prof = os.path.join(tmp, 'chrome-profile')
    tmp_pdf = os.path.join(os.path.dirname(html_path), 'guide.pdf')
    base = [find_chrome(), '--headless=new', '--disable-gpu', '--no-sandbox', '--no-first-run',
            '--user-data-dir=' + prof, '--allow-file-access-from-files',
            '--run-all-compositor-stages-before-draw', '--virtual-time-budget=8000']
    dom = subprocess.run(base + ['--dump-dom', uri(html_path)], capture_output=True, text=True, timeout=180).stdout
    m = re.search(r'data-problems="([^"]*)"', dom)
    if m is None:
        shutil.rmtree(prof, ignore_errors=True)
        raise SystemExit('the layout check did not run (Chrome: {} bytes of DOM)'.format(len(dom)))
    probs = [p for p in html.unescape(m.group(1)).split('|') if p]
    if probs:
        shutil.rmtree(prof, ignore_errors=True)
        raise SystemExit('guide not built (layout):\n  ' + '\n  '.join(probs))
    cmd = [find_chrome(), '--headless=new', '--disable-gpu', '--no-sandbox', '--no-first-run',
           '--user-data-dir=' + prof, '--allow-file-access-from-files', '--no-pdf-header-footer',
           '--run-all-compositor-stages-before-draw', '--virtual-time-budget=8000',
           '--print-to-pdf=' + tmp_pdf, uri(html_path)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    shutil.rmtree(prof, ignore_errors=True)
    if not os.path.isfile(tmp_pdf):
        raise SystemExit('Chrome did not write the PDF:\n' + r.stderr[-2000:])
    shutil.move(tmp_pdf, pdf_path)
    if keep_html:
        shutil.copy(html_path, os.path.splitext(pdf_path)[0] + '.html')
    shutil.rmtree(os.path.dirname(html_path), ignore_errors=True)
    return pdf_path
