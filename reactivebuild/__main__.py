"""CLI entry point: print a ReactiveBuild config.

Usage:
    python -m reactivebuild                # default tower preset
    python -m reactivebuild tower          # named preset
    python -m reactivebuild chain
    python -m reactivebuild cantilever
    python -m reactivebuild bridge
"""

import sys

from . import config


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    name = argv[0] if argv else "tower"
    try:
        params = config.preset(name)
    except (ValueError, KeyError):
        valid = ", ".join(t.value for t in config.ExperimentType)
        print("unknown preset {!r}; choose one of: {}".format(name, valid),
              file=sys.stderr)
        return 2
    print(params.describe())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
