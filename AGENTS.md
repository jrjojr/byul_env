# AGENTS.md instructions

이 저장소는 BYUL 프로젝트다. 작업을 시작할 때는 `external/rules` Git
submodule의 독립 `rules_env`가 소유하는 다음 Emacs Org 중앙 규칙을 순서대로 읽고
따른다.

```text
external/rules/common/init-rules.org
external/rules/common/project-bootstrap-rules.org
external/rules/projects/byul/byul-init-rules.org
```

이 파일은 도구가 고정 이름으로 읽는 최소 bootstrap이다. 규칙 본문을 이 저장소의
Markdown에 복사하지 않으며, 중앙 경로가 없으면 오래된 local 문서를 fallback 규칙으로
사용하지 않는다.
검토한 중앙 Git revision과 checkout 계약은 `rules-ref.org`가 기록한다.
