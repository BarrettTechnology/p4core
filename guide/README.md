# p4core.guide: the app guides (PDF) from Markdown

Each app keeps its guide in `docs/guide/guide.md` (text), `docs/guide/shots/` (screenshots, generated)
and, for the screenshots, `docs/guide/sim/` (the configuration loaded into the puck simulator).
`scripts/build-guide.sh` in each app wraps the commands below.

## Needs

- Python 3 (standard library only for the build).
- Chrome, Chromium or Edge (renders the PDF); `P4GUIDE_CHROME` points at one elsewhere.
- aspell + aspell-en for the spelling check (`sudo apt install aspell aspell-en`); without it the build
  says so and skips the check. Barrett terms are in `words.txt` here; a guide adds its own in
  `docs/guide/words.txt`.
- Screenshots only: Linux, the app's venv (wx, canopen: already in its requirements), vcan0 and the
  puck simulator (`sim/` in the firmware repo).

`scripts/build-guide.sh` checks all of this first and prints what is missing.

## Uprev a guide

Text only (any machine with Chrome, Chromium or Edge):

    edit docs/guide/guide.md          # wording, version, revision
    python -m p4core.guide build docs/guide/guide.md

New screenshots too (Linux, the app's venv, the puck simulator on vcan0):

    sudo bash <stm32>/sim/setup_vcan.sh                 # once per boot
    python -m p4core.guide all docs/guide/guide.md --sim <stm32>/sim/puck_sim

`all` = `shots` then `build`. `shots` starts the simulator (fresh, empty NV), runs the `setup:`
command (loads the guide's configuration), starts the real app on vcan0, drives it through each
```` ```shot ```` block and saves `shots/<name>.png` + `.json` (the widget positions the callouts use,
the menus, the status bar). The PDF goes where `output:` says: the file the installers ship.
Commit guide.md, shots/ and the PDF together.

## guide.md

Front matter (`key: value`, `#` = comment):

| key | |
|---|---|
| title, intro | header text |
| document, revision | "Document B7372 Rev AB" in the footer |
| app, app_version | footer, after the document number; `app_version: from ../../app/version.py` reads the app's __version__ |
| logo | an image in p4core/guide/assets |
| output | the PDF, relative to guide.md |
| phone, web, email | the Contact Us bar |
| app_main, app_args, app_root | how `shots` starts the app (`module:function`, its arguments, the repo root) |
| app_python | the app's Python, relative to guide.md (else $P4GUIDE_APP_PYTHON, else this one) |
| startup_s | seconds after the window opens before the first shot |
| setup | shell commands before the shots, `;`-separated; `{python}` = the app's Python, `{guide}` = this folder |

Body:

| source | page |
|---|---|
| `<!-- page -->` | new page |
| ```` ```shot NAME ```` ... ```` ``` ```` | how to take screenshot NAME: Python lines run in the app with `f` = the main window (`f.scan_pucks(None)`, `f.text_version.SetValue('5.0.5')`), and `wait 2.5` |
| `![caption](shot:NAME)` | the app window: title bar, menu bar, the screenshot, status bar |
| `![caption](shot:NAME crop=x,y,w,h)` | only that part of the screenshot (client pixels, see shots/NAME.json), larger |
| `![caption](menu:Menu@NAME)` | that menu as a card, read from screenshot NAME; the panel after it sits beside it |
| `# TITLE` | an orange panel |
| `## text` | the panel's blue subtitle |
| `1. **Name** - what it does. [@target]` | a numbered item, and its callout on the figure above it |
| indented line under an item | the item's note (small text) |
| `-> Next part` | the "continue" pointer |
| `### Title` + lines | a troubleshooting card: `! ` problem (warning icon), `+ ` fix (bulb icon), plain = text |
| `**bold**`, `_italic_` | inline |

Callout targets: a window attribute (`[@choice_port]`, `[@can_bar.btn_scan]`), a control's on-screen
text (`[@text:Scan Pucks]`), a menu-bar title (`[@menu:File]`) or, on a menu card, an entry
(`[@item:Save Parameters]`). Add `:left` `:right` `:above` `:below`, or `:row` (point at a
labelled row from outside the window). The build stops, and leaves the old PDF in place, when a target
is not on its figure, a callout lands off the page, or a page does not fit even after shrinking the
panel text, then the screenshots.

Fonts: Poppins, Montserrat, Open Sans (SIL Open Font License, texts in assets/fonts).
