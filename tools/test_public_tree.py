"""Privacy checks do not need any actual personal identifiers."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import check_public_tree


class PublicTreeTests(unittest.TestCase):
    def audit(self, files, tokens=None):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for name, data in files.items():
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(data)
            argv = ['check_public_tree.py']
            if tokens is not None:
                patterns = root/'ignored-patterns.json'
                patterns.write_text(json.dumps(tokens))
                argv += ['--private-patterns', str(patterns)]
            listing = ('\0'.join(files)+'\0').encode()
            output = io.StringIO()
            with patch.object(check_public_tree, 'ROOT', root), patch.object(sys, 'argv', argv), \
                 patch.object(check_public_tree.subprocess, 'check_output', return_value=listing), \
                 contextlib.redirect_stdout(output):
                code = check_public_tree.main()
            return code, output.getvalue()

    def test_private_configuration_and_generated_files_fail(self):
        code, output = self.audit({'.local/account.local.json':'{}', 'build-test/CMakeCache.txt':'cache'})
        self.assertTrue(code)
        self.assertIn('private/generated directory tracked', output)

    def test_tokens_are_redacted_in_diagnostics(self):
        code, output = self.audit({'README.md':'internal-only-sentinel'}, ['internal-only-sentinel'])
        self.assertTrue(code)
        self.assertNotIn('internal-only-sentinel', output)

    def test_portable_public_documentation_passes(self):
        code, output = self.audit({'README.md':'Run from /path/to/project using tools/local_run.py'})
        self.assertFalse(code)
        self.assertIn('PASS', output)


if __name__ == '__main__':
    unittest.main()
