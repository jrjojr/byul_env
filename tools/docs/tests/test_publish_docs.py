from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
import sys
from unittest import mock


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import publish_docs as publisher  # noqa: E402


class PublisherValidationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        (self.root / "tools" / "docs").mkdir(parents=True)
        (self.root / "docs" / "org" / "en").mkdir(parents=True)
        (self.root / "docs" / "en").mkdir(parents=True)
        self.manifest = self.root / "tools" / "docs" / "document-map.org"
        self.source = self.root / "docs" / "org" / "en" / "example.org"
        self.output = self.root / "docs" / "en" / "example.md"
        self.write_manifest()
        self.write_source()

    def write_manifest(self, extra_rows: str = "") -> None:
        self.manifest.write_text(
            "#+TITLE: Test map\n"
            "#+DOCUMENT_MAP_VERSION: 1\n\n"
            "* Map\n\n"
            "#+NAME: byul-document-map-v1\n"
            "| key | source | output | slug | kind | language | access |\n"
            "|-----+--------+--------+------+-----+----------+--------|\n"
            "| example | docs/org/en/example.org | docs/en/example.md | example | configuration | en | public |\n"
            f"{extra_rows}",
            encoding="utf-8",
            newline="\n",
        )

    def write_source(self, suffix: str = "") -> None:
        self.source.write_text(
            "#+TITLE: Example\n"
            "#+LANGUAGE: en\n"
            "#+DOCUMENT_KIND: configuration\n"
            "#+AUDIENCE: developer\n"
            "#+ACCESS: public\n"
            "#+SECTION: configuration\n"
            "#+SLUG: example\n"
            "#+SOURCE_FORMAT_VERSION: 1\n"
            "#+GENERATED_TARGETS: docs/en/example.md\n\n"
            "* Purpose\n\n"
            f"Example body.{suffix}\n",
            encoding="utf-8",
            newline="\n",
        )

    def load_one(self):
        documents = publisher.load_manifest(self.root, self.manifest)
        self.assertEqual(1, len(documents))

    def test_repository_root_does_not_depend_on_embedded_rules_tree(self) -> None:
        (self.root / "byul").mkdir()
        (self.root / "byul" / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.20)\n", encoding="utf-8"
        )
        fake_script = self.root / "tools" / "docs" / "publish_docs.py"
        with mock.patch.object(publisher, "__file__", str(fake_script)):
            self.assertEqual(self.root, publisher.repository_root())
        document = documents[0]
        source_hash = publisher.validate_source(self.root, document)
        return document, source_hash

    def write_valid_output(self, document, source_hash: str) -> str:
        body = "# Example\n\nGenerated body.\n"
        marker = publisher.build_marker(document, source_hash, publisher.text_digest(body))
        self.output.write_text(marker + body, encoding="utf-8", newline="\n")
        return body

    def test_valid_manifest_source_and_output(self) -> None:
        document, source_hash = self.load_one()
        self.write_valid_output(document, source_hash)
        state = publisher.inspect_output(self.root, document, source_hash)
        self.assertEqual("valid", state.kind)

    def test_duplicate_output_is_rejected_case_insensitively(self) -> None:
        self.write_manifest(
            "| second | docs/org/en/second.org | docs/en/EXAMPLE.md | second | configuration | en | public |\n"
        )
        with self.assertRaisesRegex(publisher.PublishError, "duplicate output"):
            publisher.load_manifest(self.root, self.manifest)

    def test_missing_marker_is_unmanaged(self) -> None:
        document, source_hash = self.load_one()
        self.output.write_text("# Hand-written output\n", encoding="utf-8", newline="\n")
        state = publisher.inspect_output(self.root, document, source_hash)
        self.assertEqual("unmanaged", state.kind)

    def test_body_edit_is_detected(self) -> None:
        document, source_hash = self.load_one()
        body = self.write_valid_output(document, source_hash)
        self.output.write_text(
            self.output.read_text(encoding="utf-8").replace(body, body + "manual edit\n"),
            encoding="utf-8",
            newline="\n",
        )
        state = publisher.inspect_output(self.root, document, source_hash)
        self.assertEqual("modified-output", state.kind)

    def test_source_edit_makes_output_stale(self) -> None:
        document, source_hash = self.load_one()
        self.write_valid_output(document, source_hash)
        self.write_source(" Changed.")
        changed_hash = publisher.validate_source(self.root, document)
        state = publisher.inspect_output(self.root, document, changed_hash)
        self.assertEqual("stale-source", state.kind)

    def test_commit_refuses_output_changed_after_validation(self) -> None:
        document, _ = self.load_one()
        self.output.write_bytes(b"changed after validation\n")
        with self.assertRaisesRegex(publisher.PublishError, "changed after validation"):
            publisher.commit_outputs(
                self.root,
                [(document, b"new generated output\n", b"previous validated output\n")],
            )
        self.assertEqual(b"changed after validation\n", self.output.read_bytes())

    def test_unmapped_org_source_is_rejected(self) -> None:
        (self.root / "docs" / "org" / "en" / "orphan.org").write_text(
            "#+TITLE: Orphan\n", encoding="utf-8", newline="\n"
        )
        documents = publisher.load_manifest(self.root, self.manifest)
        with self.assertRaisesRegex(publisher.PublishError, "unmapped Org source"):
            publisher.validate_document_inventory(self.root, documents)

    def test_unmapped_generated_output_is_rejected(self) -> None:
        document, source_hash = self.load_one()
        body = "# Orphan output\n"
        marker = publisher.build_marker(document, source_hash, publisher.text_digest(body))
        (self.root / "docs" / "en" / "orphan.md").write_text(
            marker + body, encoding="utf-8", newline="\n"
        )
        documents = publisher.load_manifest(self.root, self.manifest)
        with self.assertRaisesRegex(publisher.PublishError, "unmapped generated Markdown"):
            publisher.validate_document_inventory(self.root, documents)

    def test_missing_relative_file_link_is_rejected(self) -> None:
        self.write_source("\n\n[[file:missing.org][Missing]]")
        document = publisher.load_manifest(self.root, self.manifest)[0]
        with self.assertRaisesRegex(publisher.PublishError, "missing file: link target"):
            publisher.validate_source(self.root, document)

    def test_file_link_inside_source_block_is_not_resolved(self) -> None:
        self.write_source(
            "\n\n#+begin_src org\n[[file:example-that-must-not-exist.org]]\n#+end_src"
        )
        document = publisher.load_manifest(self.root, self.manifest)[0]
        publisher.validate_source(self.root, document)

    def test_unbalanced_source_block_is_rejected(self) -> None:
        self.write_source("\n\n#+begin_src text\nnot closed")
        document = publisher.load_manifest(self.root, self.manifest)[0]
        with self.assertRaisesRegex(publisher.PublishError, r"no matching #\+end_src"):
            publisher.validate_source(self.root, document)


if __name__ == "__main__":
    unittest.main()
