"""Generate or install the boot service for this checkout (no implicit start)."""
import argparse
import getpass
import os
from pathlib import Path
import pwd
import subprocess
import tempfile


def unit_quote(value):
    # systemd specifiers and ExecStart environment expansion are not shell quoting.
    if any(c in value for c in '\n\r\x00$'):
        raise ValueError('Paths/user must not contain newlines, NUL or dollar signs')
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--user', default=os.environ.get('SUDO_USER') or getpass.getuser())
    parser.add_argument('--env-file', type=Path, default=Path('/etc/default/garby-bridge'))
    parser.add_argument('--output', type=Path, help='Write a reviewable unit only; no root required')
    args = parser.parse_args()
    account = pwd.getpwnam(args.user)
    if account.pw_uid == 0:
        parser.error('Choose the ordinary robot account with --user; do not run the bridge as root')
    directory = Path(__file__).resolve().parent
    environment = args.env_file.resolve()
    source = (directory / 'garby-bridge.service.example').read_text()
    source = source.replace('User=garby', 'User=' + args.user)
    source = source.replace('WorkingDirectory=/home/garby/GARBY/RasPi',
                            'WorkingDirectory=' + str(directory).replace('%', '%%'))
    source = source.replace('/home/garby/GARBY/RasPi/run_bridge.sh',
                            unit_quote(str(directory / 'run_bridge.sh')))
    source = source.replace('/home/garby/GARBY/RasPi/run_headless.sh',
                            unit_quote(str(directory / 'run_headless.sh')))
    source = source.replace('EnvironmentFile=/etc/default/garby-bridge',
                            'EnvironmentFile=' + str(environment).replace('%', '%%'))
    if args.output:
        args.output.write_text(source)
        print(f'Generated {args.output}; startup has not been enabled.')
        return
    if os.geteuid() != 0:
        parser.error('Installation needs sudo; use --output to generate without installing')
    if not environment.is_file():
        parser.error(f'Configure {environment} from garby-bridge.env.example first')
    with tempfile.TemporaryDirectory(prefix='garby-unit-') as temporary:
        candidate = Path(temporary) / 'garby-bridge.service'
        candidate.write_text(source)
        subprocess.run(['systemd-analyze', 'verify', str(candidate)], check=True)
        subprocess.run(['install', '-m', '644', str(candidate),
                        '/etc/systemd/system/garby-bridge.service'], check=True)
    subprocess.run(['systemctl', 'daemon-reload'], check=True)
    subprocess.run(['systemctl', 'enable', 'garby-bridge.service'], check=True)
    print('Enabled for boot. Start now with: sudo systemctl start garby-bridge')
    print('Check: journalctl -u garby-bridge -b -f')


if __name__ == '__main__':
    main()
