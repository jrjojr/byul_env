;;; publish.el --- Deterministic BYUL Org to Markdown export -*- lexical-binding: t; -*-

;; This file is loaded by publish_docs.py with Emacs -Q.  It deliberately does
;; not load a user's init file and never evaluates Babel source blocks.

(require 'org)
(require 'ox-md)
(require 'subr-x)

(defun byul-docs--required-environment (name)
  "Return required environment variable NAME or signal an error."
  (let ((value (getenv name)))
    (unless (and value (not (string-empty-p value)))
      (error "Required environment variable is missing: %s" name))
    value))

(defun byul-docs-export-from-environment ()
  "Export one Org source to a staging Markdown path.

BYUL_DOCS_SOURCE, BYUL_DOCS_OUTPUT and BYUL_DOCS_STAGING_ROOT are supplied by
the Python publisher.  The final generated marker and atomic replacement are
owned by that publisher, not this Emacs helper."
  (let* ((source (expand-file-name
                  (byul-docs--required-environment "BYUL_DOCS_SOURCE")))
         (output (expand-file-name
                  (byul-docs--required-environment "BYUL_DOCS_OUTPUT")))
         (staging-root
          (file-name-as-directory
           (expand-file-name
            (byul-docs--required-environment "BYUL_DOCS_STAGING_ROOT"))))
         (output-directory (file-name-directory output))
         (buffer nil))
    (unless (and (file-regular-p source) (file-readable-p source))
      (error "Org source is not a readable regular file: %s" source))
    (unless (string-equal (downcase (or (file-name-extension source) ""))
                          "org")
      (error "Org source must use the .org extension: %s" source))
    (unless (string-equal (downcase (or (file-name-extension output) ""))
                          "md")
      (error "Markdown staging output must use the .md extension: %s" output))
    (unless (file-in-directory-p output staging-root)
      (error "Output escapes the staging root: %s" output))
    (make-directory output-directory t)
    (unwind-protect
        (let ((enable-local-variables nil)
              (enable-local-eval nil)
              (org-export-use-babel nil)
              (org-confirm-babel-evaluate t)
              (org-export-with-broken-links nil)
              (org-export-with-author nil)
              (org-export-with-creator nil)
              (org-export-with-section-numbers nil)
              (org-export-time-stamp-file nil)
              (org-export-show-temporary-export-buffer nil)
              (make-backup-files nil)
              (auto-save-default nil))
          (setq buffer (find-file-noselect source t))
          (with-current-buffer buffer
            (org-export-to-file
             'md output nil nil nil nil
             '(:with-author nil
               :with-creator nil
               :section-numbers nil
               :time-stamp-file nil))))
      (when (buffer-live-p buffer)
        (kill-buffer buffer)))
    (unless (and (file-regular-p output)
                 (> (file-attribute-size (file-attributes output)) 0))
      (error "Emacs did not create a non-empty Markdown file: %s" output))
    (princ (format "BYUL_DOC_EXPORT emacs=%s org=%s\n"
                   emacs-version (org-version)))))

(provide 'byul-docs-publish)
;;; publish.el ends here
