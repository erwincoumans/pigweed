# Copyright 2020 The Pigweed Authors
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
"""Argument parsing code for presubmit checks."""

import argparse
import logging
from pathlib import Path
import re
import shutil
from typing import Callable, Optional, Sequence

from pw_presubmit import tools

_LOG = logging.getLogger(__name__)


def add_path_arguments(parser) -> None:
    """Adds common presubmit check options to an argument parser."""

    parser.add_argument(
        'paths',
        metavar='pathspec',
        nargs='*',
        type=Path,
        help=('Paths to which to restrict the checks. These are interpreted '
              'as Git pathspecs. If --base is provided, only paths changed '
              'since that commit are checked.'))
    parser.add_argument(
        '-b',
        '--base',
        metavar='COMMIT',
        help=('Git revision against which to diff for changed files. '
              'If none is provided, the entire repository is used.'))
    parser.add_argument(
        '-e',
        '--exclude',
        metavar='REGULAR_EXPRESSION',
        default=[],
        action='append',
        type=re.compile,
        help='Exclude paths matching any of these regular expressions.')


def _add_programs_arguments(exclusive: argparse.ArgumentParser,
                            programs: tools.Programs, default: str):
    def presubmit_program(arg: str) -> tools.Program:
        if arg not in programs:
            raise argparse.ArgumentTypeError(
                f'{arg} is not the name of a presubmit program')

        return programs[arg]

    exclusive.add_argument('-p',
                           '--program',
                           choices=programs.values(),
                           type=presubmit_program,
                           default=default,
                           help='Which presubmit program to run')

    all_steps = programs.all_steps()

    # The step argument appends steps to a program. No "step" argument is
    # created on the resulting argparse.Namespace.
    class AddToCustomProgram(argparse.Action):
        def __call__(self, parser, namespace, values, unused_option=None):
            if not isinstance(namespace.program, list):
                namespace.program = []

            if values not in all_steps:
                raise parser.error(
                    f'argument --step: {values} is not the name of a '
                    'presubmit check\n\n'
                    f'Valid values for --step:\n{{{",".join(all_steps)}}}')

            namespace.program.append(all_steps[values])

    exclusive.add_argument(
        '--step',
        action=AddToCustomProgram,
        default=argparse.SUPPRESS,  # Don't create a "step" argument.
        help='Provide explicit steps instead of running a predefined program.',
    )


def add_arguments(parser: argparse.ArgumentParser,
                  programs: Optional[tools.Programs] = None,
                  default: str = '') -> None:
    """Adds common presubmit check options to an argument parser."""

    add_path_arguments(parser)
    parser.add_argument(
        '--repository',
        default=Path.cwd(),
        type=Path,
        help=(
            'Change to this directory before resolving paths or running the '
            'presubmit. Presubmit checks must be run from a Git repository.'))
    parser.add_argument('-k',
                        '--keep-going',
                        action='store_true',
                        help='Continue instead of aborting when errors occur.')
    parser.add_argument(
        '--output-directory',
        type=Path,
        help='Output directory (default: <repo root>/.presubmit)',
    )

    exclusive = parser.add_mutually_exclusive_group()
    exclusive.add_argument(
        '--clear',
        '--clean',
        action='store_true',
        help='Delete the presubmit output directory and exit.',
    )

    if programs:
        if not default:
            raise ValueError('A default must be provided with programs')

        _add_programs_arguments(parser, programs, default)


def run(
    program: Sequence[Callable],
    clear: bool,
    repository: Path,
    output_directory: Path,
    **other_args,
) -> int:
    """Processes all arguments from add_arguments and runs the presubmit."""

    if not output_directory:
        output_directory = tools.git_repo_path('.presubmit', repo=repository)

    _LOG.debug('Using environment at %s', output_directory)

    if clear:
        _LOG.info('Clearing presubmit output directory')

        if output_directory.exists():
            shutil.rmtree(output_directory)
            _LOG.info('Deleted %s', output_directory)

        return 0

    if tools.run_presubmit(program,
                           repo_path=repository,
                           output_directory=output_directory,
                           **other_args):
        return 0

    return 1