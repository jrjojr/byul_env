from pathlib import Path
import tempfile
import unittest

from byul_wrapper_generator.header_parser import audit_header, parse_header


VALID_HEADER = r"""
/**
 * @brief Creates a sample object.
 *
 * @param[in] value Initial value.
 * @return Newly allocated object.
 * @retval NULL Allocation failed.
 *
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error null-return
 */
BYUL_API sample_t* sample_create(int value);

/**
 * @brief Reads the sample value.
 *
 * @param[in] sample Object to inspect.
 * @return Stored value.
 *
 * @byul.nullable sample false
 * @byul.error none
 */
BYUL_API int sample_get_value(const sample_t* sample);
"""


class HeaderParserTest(unittest.TestCase):
    def _write_header(self, source: str) -> tuple[tempfile.TemporaryDirectory, Path]:
        directory = tempfile.TemporaryDirectory()
        path = Path(directory.name) / "sample.h"
        path.write_text(source, encoding="utf-8")
        return directory, path

    def test_parses_multiline_declaration_and_structured_metadata(self):
        directory, path = self._write_header(VALID_HEADER)
        try:
            declarations = parse_header(path)
        finally:
            directory.cleanup()

        self.assertEqual([item.name for item in declarations], ["sample_create", "sample_get_value"])
        self.assertEqual(declarations[0].byul_tags["lifetime"], ("return caller-owned",))
        self.assertEqual(declarations[1].parameters[0].name, "sample")
        self.assertTrue(declarations[1].parameters[0].pointer)
        self.assertTrue(declarations[1].parameters[0].const)

    def test_valid_contract_has_no_lint_errors(self):
        directory, path = self._write_header(VALID_HEADER)
        try:
            report = audit_header(path)
        finally:
            directory.cleanup()

        self.assertEqual(report.issues, ())

    def test_deprecation_annotation_preserves_adjacent_doxygen(self):
        source = r"""
/**
 * @brief Updates a sample through a compatibility adapter.
 * @param[in] value New value.
 */
BYUL_DEPRECATED("Use sample_update_checked; removal is planned for ABI 2.")
BYUL_API void sample_update(int value);
"""
        directory, path = self._write_header(source)
        try:
            report = audit_header(path)
        finally:
            directory.cleanup()

        self.assertEqual(["sample_update"], [
            item.name for item in report.declarations
        ])
        self.assertEqual(report.issues, ())

    def test_reports_missing_pointer_contract_and_unknown_tag(self):
        source = r"""
/**
 * @brief Returns a child object.
 * @param[in] sample Parent object.
 * @return Child object.
 * @byul.unknown value
 */
BYUL_API child_t* sample_get_child(sample_t* sample);
"""
        directory, path = self._write_header(source)
        try:
            report = audit_header(path)
        finally:
            directory.cleanup()

        codes = {issue.code for issue in report.issues}
        self.assertIn("missing-nullable", codes)
        self.assertIn("missing-return-nullable", codes)
        self.assertIn("missing-return-lifetime", codes)
        self.assertIn("unknown-byul-tag", codes)

    def test_reports_documentation_for_unknown_parameter(self):
        source = r"""
/**
 * @brief Updates a sample.
 * @param[in] wrong Unknown parameter.
 */
BYUL_API void sample_update(int value);
"""
        directory, path = self._write_header(source)
        try:
            report = audit_header(path)
        finally:
            directory.cleanup()

        codes = {issue.code for issue in report.issues}
        self.assertIn("missing-param", codes)
        self.assertIn("unknown-param", codes)

    def test_ignores_commented_out_export_declarations(self):
        source = r"""
// BYUL_API bool sample_old_api(int value);
/* BYUL_API bool sample_removed_api(int value); */
/**
 * @brief Updates a sample.
 * @param[in] value New value.
 */
BYUL_API void sample_update(int value);
"""
        directory, path = self._write_header(source)
        try:
            declarations = parse_header(path)
        finally:
            directory.cleanup()
        self.assertEqual(["sample_update"], [item.name for item in declarations])


if __name__ == "__main__":
    unittest.main()
