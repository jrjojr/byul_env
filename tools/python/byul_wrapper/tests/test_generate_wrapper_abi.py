from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "generate_wrapper_abi.py"
SPEC = spec_from_file_location("generate_wrapper_abi", SCRIPT)
GENERATOR = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(GENERATOR)


class GenerateWrapperAbiTest(unittest.TestCase):
    def test_header_to_cdef_removes_preprocessor_and_export_macro(self):
        source = """\
#ifndef SAMPLE_H
#define SAMPLE_H
// comment
typedef struct s_sample {
    int value;
} sample_t;
BYUL_API void sample_set(sample_t* sample, int value);
#endif
"""
        with tempfile.TemporaryDirectory() as directory:
            header = Path(directory) / "sample.h"
            header.write_text(source, encoding="utf-8")
            result = GENERATOR.header_to_cdef(header)

        self.assertIn("typedef struct s_sample", result)
        self.assertIn("void sample_set(sample_t* sample, int value);", result)
        self.assertNotIn("BYUL_API", result)
        self.assertNotIn("#define", result)
        self.assertNotIn("comment", result)

    def test_replace_cdef_preserves_python_wrapper_code(self):
        source = 'from ffi_core import ffi\nffi.cdef("""old();\n""")\nclass Wrapper:\n    pass\n'
        with tempfile.TemporaryDirectory() as directory:
            module = Path(directory) / "sample.py"
            module.write_text(source, encoding="utf-8")
            result = GENERATOR.replace_cdef(module, "void generated(void);")

        self.assertIn("void generated(void);", result)
        self.assertNotIn("old();", result)
        self.assertIn("class Wrapper:\n    pass", result)


if __name__ == "__main__":
    unittest.main()
