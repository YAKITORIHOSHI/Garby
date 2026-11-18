#!/usr/bin/env python3
"""Probe Firebase without starting the robot or touching return commands.

Default: discover local credentials for an authenticated heartbeat read;
without credentials, check the anonymous endpoint (never print its value).
--credentials overrides discovery; --anonymous skips authentication entirely.
--write-test adds a disposable
UUID diagnostics record and removes only that record. Exit 2 means incomplete.
"""
import argparse
import json
import os
from pathlib import Path
import sys
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import urlopen
import uuid

from firebase_support import resolve_firebase_credentials

DEFAULT_DATABASE_URL = "https://garby-thesis-default-rtdb.asia-southeast1.firebasedatabase.app"


def anonymous_probe(url):
    try:
        with urlopen(url.rstrip('/') + '/RASPI/STATES/lastSeen.json', timeout=10) as response:
            json.loads(response.read(4096))
        print('REACHABLE: HTTPS read succeeded; data omitted. Admin authentication remains unverified.')
    except HTTPError as error:
        error.close()  # it still holds the HTTP response
        if error.code in (401, 403):
            print(f'REACHABLE: Firebase returned HTTP {error.code}; this anonymous read is denied.')
        else:
            print(f'FAIL: Firebase returned HTTP {error.code}.')
            return 1
    except (OSError, URLError, ValueError):
        print('FAIL: HTTPS connection or JSON response failed.')
        return 1
    print('UNVERIFIED: authenticated read/write; supply --credentials with a service-account file.')
    return 2


def authenticated_probe(url, credential_path, write_test=False):
    if not Path(credential_path).is_file():
        print('UNVERIFIED: service-account file not found.')
        return 2
    try:
        import firebase_admin
        from firebase_admin import credentials, db
    except ImportError:
        print('UNVERIFIED: firebase-admin missing in this Python; install requirements-firebase.txt.')
        return 2
    app = None
    scratch = None
    outcome = 1
    try:
        app = firebase_admin.initialize_app(credentials.Certificate(credential_path),
            {'databaseURL': url, 'httpTimeout': 10}, name='garby-check-' + uuid.uuid4().hex)
        db.reference('/RASPI/STATES/lastSeen', app=app).get()
        print('PASS: authenticated read of heartbeat field (value omitted).')
        outcome = 0
        if write_test:
            marker = uuid.uuid4().hex
            scratch = db.reference('/_diagnostics/garby-check/' + marker, app=app)
            payload = {'probe': marker, 'purpose': 'connectivity-test'}
            scratch.set(payload)
            if scratch.get() != payload:
                raise RuntimeError('Diagnostic read-back mismatch')
            print('PASS: isolated diagnostic write/read-back.')
        else:
            print('UNVERIFIED: write permission; --write-test enables an isolated round-trip.')
    except Exception as error:
        # SDK errors can contain URLs/token details; report only the exception type.
        print(f'FAIL: Firebase operation failed ({type(error).__name__}); credentials/data omitted.')
        outcome = 1
    finally:
        if scratch is not None:
            try:
                scratch.delete()
                if scratch.get() is not None:
                    raise RuntimeError('Diagnostic cleanup not confirmed')
                print('PASS: diagnostic record removed and absence verified.')
            except Exception:
                print('FAIL: cleanup unverified; inspect /_diagnostics/garby-check/' + marker)
                outcome = 1
        if app is not None:
            firebase_admin.delete_app(app)
    return outcome


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--database-url', default=os.environ.get('GARBY_FIREBASE_DATABASE_URL', DEFAULT_DATABASE_URL))
    parser.add_argument('--credentials', help='Service-account file; relative paths use the runtime directory')
    parser.add_argument('--anonymous', action='store_true', help='Probe only the unauthenticated endpoint')
    parser.add_argument('--write-test', action='store_true')
    args = parser.parse_args()
    parsed = urlsplit(args.database_url)
    if parsed.scheme != 'https' or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment or parsed.path not in ('', '/'):
        parser.error('Use an HTTPS database root URL without credentials, query or path')
    if args.anonymous:
        if args.write_test or args.credentials:
            parser.error('--anonymous cannot be combined with --credentials or --write-test')
        return anonymous_probe(args.database_url)
    try:
        credential_path = resolve_firebase_credentials(args.credentials)
    except FileNotFoundError as error:
        if not args.credentials and not os.environ.get('GARBY_FIREBASE_CREDENTIALS') and not args.write_test:
            return anonymous_probe(args.database_url)
        print(f'UNVERIFIED: {error}')
        return 2
    except (OSError, ValueError) as error:
        print(f'UNVERIFIED: {error}')
        return 2
    return authenticated_probe(args.database_url, credential_path, args.write_test)


if __name__ == '__main__':
    sys.exit(main())
