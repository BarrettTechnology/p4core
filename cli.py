"""cli.py — the command-line half of every P4 app.

An app builds its parser with make_parser(), registers the shared
subcommands with add_common_commands() and its own with
``subparsers.add_parser``, and hands everything to main():

    parser, sub = cli.make_parser('p4gui', 'P4 puck bench tool')
    cli.add_common_commands(sub)
    tune = sub.add_parser('tune', help=...)
    tune.set_defaults(func=my_tune, needs_node=True)
    sys.exit(cli.main(parser, argv))

A command function is ``func(ctx) -> int | None``; ctx has ``args``,
``reporter``, ``session`` (connected, unless the command set
``needs_bus=False``) and ``node_id`` (resolved: --id, or the only puck on the
bus).  main() maps exceptions to exit codes, so a function only returns
nonzero for failures it reports itself.

Exit codes: 0 ok, 1 operation failed, 2 usage, 3 no bus / no node,
130 interrupted.
"""

import argparse
import json
import logging
import os
import sys

from canopen.sdo.exceptions import SdoError

from . import calibrate as _calibrate
from . import config_csv, ops, paths
from .reporter import Aborted, ConsoleReporter
from .session import Session

EXIT_OK = 0
EXIT_FAILED = 1
EXIT_USAGE = 2
EXIT_NO_DEVICE = 3
EXIT_INTERRUPTED = 130


def default_port():
    return os.environ.get('P4_CAN', 'can0')


def add_bus_options(parser, suppress=False):
    """--can/--id/--fd/--yes/--verbose/--quiet/--json, on *parser*.

    *suppress* leaves unset options out of the namespace entirely, which is
    how a subcommand repeats them (``app info --id 5`` as well as
    ``app --id 5 info``) without its defaults clobbering the top level's.
    """
    def d(value):
        return argparse.SUPPRESS if suppress else value

    g = parser.add_argument_group('bus')
    g.add_argument('--can', default=d(default_port()), metavar='PORT',
                   help='CAN interface (default: $P4_CAN or can0)')
    g.add_argument('--id', type=int, dest='node_id', metavar='N',
                   default=d(None),
                   help='node ID (default: the only puck on the bus)')
    fd = g.add_mutually_exclusive_group()
    fd.add_argument('--fd', dest='fd', action='store_true', default=d(None),
                    help='require CAN FD')
    fd.add_argument('--classic', dest='fd', action='store_false',
                    default=d(None), help='classic CAN only')
    o = parser.add_argument_group('output')
    o.add_argument('-y', '--yes', action='store_true', default=d(False),
                   help='answer yes to every question')
    o.add_argument('-v', '--verbose', action='store_true', default=d(False))
    o.add_argument('-q', '--quiet', action='store_true', default=d(False),
                   help='only results, warnings and errors')
    o.add_argument('--json', action='store_true', default=d(False),
                   help='print results as JSON')
    return parser


def _inherit_bus_options(sub):
    """Make every parser *sub* creates accept the bus options too."""
    parent = add_bus_options(argparse.ArgumentParser(add_help=False),
                             suppress=True)
    add_parser = sub.add_parser

    def add_parser_with_bus(name, **kwargs):
        kwargs['parents'] = list(kwargs.get('parents', ())) + [parent]
        return add_parser(name, **kwargs)

    sub.add_parser = add_parser_with_bus
    return sub


def add_group(sub, name, help):
    """A command with actions of its own (``app NAME ACTION ...``).

    Returns the actions' subparsers, which accept the bus options like every
    command.  ``app NAME`` alone prints the group's help and exits 2.
    """
    group = sub.add_parser(name, help=help, description=help)
    actions = _inherit_bus_options(
        group.add_subparsers(dest=name.replace('-', '_') + '_command',
                             metavar='ACTION'))
    group.set_defaults(func=lambda ctx: (group.print_help(), EXIT_USAGE)[1],
                       needs_bus=False)
    return actions


def add_action(actions, name, func, help, needs_node=False, needs_bus=True):
    """One action on a group; returns its parser for the arguments."""
    p = actions.add_parser(name, help=help, description=help)
    p.set_defaults(func=func, needs_node=needs_node, needs_bus=needs_bus)
    return p


def make_parser(prog, description, version=None):
    """(parser, subparsers) with the bus options on the top-level parser and
    on every subcommand."""
    parser = argparse.ArgumentParser(prog=prog, description=description)
    if version:
        parser.add_argument('--version', action='version',
                            version='%(prog)s ' + version)
    add_bus_options(parser)
    sub = parser.add_subparsers(dest='command', metavar='COMMAND')
    return parser, _inherit_bus_options(sub)


class Context:
    def __init__(self, args, reporter, session=None, node_id=None):
        self.args = args
        self.reporter = reporter
        self.session = session
        self.node_id = node_id

    def emit(self, result, lines=None):
        """Print a result: JSON with --json, else *lines* (or str(result))."""
        if self.args.json:
            print(json.dumps(result, indent=2, sort_keys=True, default=str))
        elif lines is not None:
            for line in lines:
                print(line)
        elif result is not None:
            print(result)


class NoDevice(RuntimeError):
    pass


def resolve_node(session, node_id, reporter):
    """*node_id* if given (and on the bus), else the only puck found."""
    found = session.found_ids or session.scan()
    if node_id is not None:
        if node_id not in found:
            raise NoDevice('node {} not found on {} (found {})'.format(
                node_id, session.port, found or 'none'))
        return node_id
    if not found:
        raise NoDevice('no pucks found on {}'.format(session.port))
    if len(found) > 1:
        raise NoDevice('{} pucks on the bus ({}): choose one with --id'
                       .format(len(found), found))
    reporter.note('Using node {}'.format(found[0]))
    return found[0]


def main(parser, argv=None, session_factory=Session, failures=(), aborts=()):
    """Parse, connect, run the command, and return an exit code.

    *failures* and *aborts* are an app's own exception types that mean
    'the operation failed' (exit 1) and 'the operation was stopped' (130).
    """
    args = parser.parse_args(argv)
    if not getattr(args, 'func', None):
        parser.print_help()
        return EXIT_USAGE
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING,
                        format='%(levelname)s %(name)s: %(message)s')
    reporter = ConsoleReporter(
        stream=sys.stderr if args.json else sys.stdout,
        assume_yes=args.yes, quiet=args.quiet)
    ctx = Context(args, reporter)
    session = None
    try:
        if getattr(args, 'needs_bus', True):
            session = session_factory()
            try:
                session.connect(args.can, fd=args.fd)
            except ConnectionError as exc:
                raise NoDevice('cannot open {}: {}'.format(args.can, exc))
            ctx.session = session
            if getattr(args, 'needs_node', False):
                ctx.node_id = resolve_node(session, args.node_id, reporter)
        code = args.func(ctx)
        return EXIT_OK if code is None else code
    except KeyboardInterrupt:
        reporter.warn('interrupted')
        return EXIT_INTERRUPTED
    except (Aborted,) + tuple(aborts) as exc:
        reporter.warn('stopped: {}'.format(exc) if str(exc) else 'stopped')
        return EXIT_INTERRUPTED
    except NoDevice as exc:
        reporter.warn(str(exc))
        return EXIT_NO_DEVICE
    except SdoError as exc:
        # An abort names the refusal (e.g. 0x08000022: wrong drive state);
        # a traceback would bury it.
        reporter.warn('SDO: {}'.format(exc))
        return EXIT_FAILED
    except (ops.OperationError, config_csv.ConfigError,
            _calibrate.CalibrationError, FileNotFoundError) + tuple(failures) as exc:
        reporter.warn(str(exc))
        return EXIT_FAILED
    finally:
        if session is not None:
            session.disconnect()


# ---------------------------------------------------------------------------
# Shared subcommands
# ---------------------------------------------------------------------------
def _cmd_scan(ctx):
    nodes = ops.scan(ctx.session, ctx.reporter)
    ctx.emit(nodes, [' '.join(str(n) for n in nodes)] if nodes else [])
    return EXIT_OK if nodes else EXIT_NO_DEVICE


def targets(ctx):
    """--all: every puck; else the resolved node."""
    if getattr(ctx.args, 'all', False):
        found = ctx.session.found_ids or ctx.session.scan()
        if not found:
            raise NoDevice('no pucks found on {}'.format(ctx.session.port))
        return found
    return [resolve_node(ctx.session, ctx.args.node_id, ctx.reporter)]


def _cmd_info(ctx):
    results = []
    for node_id in targets(ctx):
        results.append(ops.info(ctx.session, node_id,
                                flashloader=ctx.args.flashloader,
                                reporter=ctx.reporter))
    lines = [line for r in results for line in ops.format_info(r)]
    ctx.emit(results if ctx.args.all else results[0], lines)


def _cmd_flash(ctx):
    for node_id in targets(ctx):
        ops.flash(ctx.session, node_id, ctx.args.firmware, ctx.reporter)


def _cmd_config_load(ctx):
    failed = False
    for node_id in targets(ctx):
        result = ops.load_config(ctx.session, node_id, ctx.args.csv,
                                 ctx.reporter, save=not ctx.args.no_save,
                                 reboot=not ctx.args.no_reboot,
                                 force=ctx.args.force)
        failed |= not result.ok
    return EXIT_FAILED if failed else EXIT_OK


def _cmd_config_export(ctx):
    template = ctx.args.template
    if not os.path.exists(template):
        template = paths.template(template)
    ops.export_config(ctx.session, ctx.node_id, template, ctx.args.output,
                      ctx.reporter)


def _cmd_config_check(ctx):
    rows = config_csv.read(ctx.args.csv)
    errors, warnings = config_csv.validate(rows, ctx.args.node_id or 1)
    for line in config_csv.describe(warnings):
        ctx.reporter.note('warning, ' + line)
    for line in config_csv.describe(errors):
        ctx.reporter.warn(line)
    ctx.emit(dict(errors=len(errors), warnings=len(warnings),
                  version=config_csv.file_version(rows)),
             ['{}: {} error(s), {} warning(s)'.format(
                 ctx.args.csv, len(errors), len(warnings))])
    return EXIT_FAILED if errors else EXIT_OK


def _cmd_config_templates(ctx):
    names = paths.templates()
    ctx.emit(names, names)


def _cmd_set_id(ctx):
    ok = ops.set_id(ctx.session, ctx.node_id, ctx.args.new_id, ctx.reporter)
    return EXIT_OK if ok else EXIT_FAILED


def _cmd_reset(ctx):
    for node_id in targets(ctx):
        ops.reset(ctx.session, node_id, ctx.reporter)


def _cmd_calibrate(ctx):
    steps = ctx.args.steps or None
    unknown = set(steps or ()) - set(CALIBRATION_STEPS)
    if unknown:
        ctx.reporter.warn('unknown step(s) {}; choose from {}'.format(
            ', '.join(sorted(unknown)), ', '.join(CALIBRATION_STEPS)))
        return EXIT_USAGE
    if not ctx.reporter.confirm(
            'Calibration turns the motor (one revolution each way for the '
            'encoder zero). Is the motor free to turn?', default=True):
        return EXIT_FAILED
    results = {}
    for node_id in targets(ctx):
        results[node_id] = ops.calibrate(ctx.session, node_id, steps,
                                         ctx.reporter)
    if ctx.args.json:
        ctx.emit({str(k): v for k, v in results.items()})


CALIBRATION_STEPS = [name for name, _ in _calibrate.STEPS]


def add_common_commands(sub, only=None):
    """Register scan/info/flash/config/set-id/reset/calibrate on *sub*.

    *only* limits it to the named commands.  Returns {name: subparser} so an
    app can add options to one.
    """
    made = {}

    def add(name, func, help, needs_node=False, needs_bus=True, all_flag=False):
        if only is not None and name not in only:
            return None
        p = sub.add_parser(name, help=help, description=help)
        p.set_defaults(func=func, needs_node=needs_node, needs_bus=needs_bus)
        if all_flag:
            p.add_argument('--all', action='store_true',
                           help='every puck on the bus')
        made[name] = p
        return p

    add('scan', _cmd_scan, 'list the node IDs on the bus')

    p = add('info', _cmd_info, 'firmware, model and live readings',
            all_flag=True)
    if p:
        p.add_argument('--flashloader', action='store_true',
                       help='also read the flashloader version (reboots the puck)')

    p = add('flash', _cmd_flash, 'program firmware (.bin or .ebin) over CAN',
            all_flag=True)
    if p:
        p.add_argument('firmware', help='firmware image')

    if only is None or 'config' in only:
        config = sub.add_parser('config', help='configuration CSVs',
                                description='configuration CSVs')
        csub = _inherit_bus_options(
            config.add_subparsers(dest='config_command', metavar='ACTION'))
        config.set_defaults(func=lambda ctx: (config.print_help(), EXIT_USAGE)[1],
                            needs_bus=False)
        made['config'] = config

        p = csub.add_parser('load', help='download a CSV, save and reboot')
        p.set_defaults(func=_cmd_config_load, needs_bus=True)
        p.add_argument('csv')
        p.add_argument('--all', action='store_true', help='every puck on the bus')
        p.add_argument('--no-save', action='store_true',
                       help='RAM only: skip Save.All (and the reboot)')
        p.add_argument('--no-reboot', action='store_true')
        p.add_argument('--force', action='store_true',
                       help='skip invalid lines instead of refusing the file')

        p = csub.add_parser('export', help="write a template filled with the puck's values")
        p.set_defaults(func=_cmd_config_export, needs_bus=True, needs_node=True)
        p.add_argument('template', help='template CSV, or the name of a shipped one '
                       '(see: config templates)')
        p.add_argument('output')

        p = csub.add_parser('check', help='validate a CSV without a puck')
        p.set_defaults(func=_cmd_config_check, needs_bus=False)
        p.add_argument('csv')

        p = csub.add_parser('templates', help='list the shipped templates')
        p.set_defaults(func=_cmd_config_templates, needs_bus=False)

    p = add('set-id', _cmd_set_id, "change a puck's node ID (and its PDO COB-IDs)",
            needs_node=True)
    if p:
        p.add_argument('new_id', type=int)

    add('reset', _cmd_reset, 'NMT-reset a puck and wait for it', all_flag=True)

    p = add('calibrate', _cmd_calibrate,
            'current-sense bias and gain, encoder zero (the motor turns)',
            all_flag=True)
    if p:
        p.add_argument('steps', nargs='*', metavar='STEP',
                       help='any of {} (default: all, in that order)'.format(
                           ', '.join(CALIBRATION_STEPS)))
    return made
