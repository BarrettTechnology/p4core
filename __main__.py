"""python -m p4core: the shared commands (cli.add_common_commands) on their own."""

import sys

from . import __version__, cli


def main(argv=None):
    parser, sub = cli.make_parser(
        'p4core', 'p4core v{}: scan, flash, configure and wipe P4 pucks.'.format(
            __version__), version='v' + __version__)
    cli.add_common_commands(sub)
    return cli.main(parser, argv)


if __name__ == '__main__':
    sys.exit(main())
