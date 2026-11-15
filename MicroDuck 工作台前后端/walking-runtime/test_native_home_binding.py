import hashlib
from pathlib import Path
import tempfile
import unittest

from run_teleop import verify_native_runtime


class NativeHomeBindingTests(unittest.TestCase):
    def test_original_contract_has_no_custom_binary_requirement(self):
        verify_native_runtime({}, '/missing/original-runtime')

    def test_modified_native_home_requires_exact_binary(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'robotd'
            path.write_bytes(b'compiled native home')
            contract={'native_home_override':{'binary_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}}
            verify_native_runtime(contract,path)
            path.write_bytes(b'old home binary')
            with self.assertRaisesRegex(RuntimeError,'differs'):
                verify_native_runtime(contract,path)
            with self.assertRaisesRegex(RuntimeError,'differs'):
                verify_native_runtime(contract,path.with_name('missing'))

    def test_incomplete_native_home_binding_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'incomplete'):
            verify_native_runtime({'native_home_override':{}},'/missing/runtime')


if __name__=='__main__':unittest.main()
