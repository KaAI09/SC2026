# KaAI SC2026

SEA:ME Hackathon 2026 — TOPST D3-G 기반 자율주행 스케일카.

## 문서

| 문서 | 내용 |
|---|---|
| **[ROLLBACK.md](ROLLBACK.md)** | ⭐ **전체 명령 시퀀스 (git → 빌드 → 테스트) · 되돌리는 법 · 남은 작업(TODO)** |
| [PERCEPTION.md](PERCEPTION.md) | 인지 파이프라인 · 알려진 한계 · **채택/기각된 접근과 그 측정치** |
| [Task command.md](Task%20command.md) | 런치별 운영 명령 (calibrate / record / perceive / drive) |
| [offline/PIPELINE.md](offline/PIPELINE.md) | 오프라인 도구 (panel_replay · control_predict · control_select) |
| [offline/CONTROL_DESIGN.md](offline/CONTROL_DESIGN.md) | 제어기 C1~C5 설계 · open-loop 평가의 한계 |
| [REFACTORING.md](REFACTORING.md) | (이력) 0709 이전 리팩토링 기록 |
| [CLAUDE.md](CLAUDE.md) | Claude Code 작업 규칙 |
| `D-Racer-Kit/docs/`, `Notice/` | 공식 하드웨어·규정 문서 (**1차 기술 참조 — 수정 금지**) |

## 현재 상태

**기준선**: `2d329ee` = 2026-07-11 완주 (3세션 1,505프레임, 트랙 이탈 0, 인지 30.0Hz, LOST 0%).

그 위에 안전·강건성·인지 커밋 10개. **오프라인 회귀(68/68)만 통과했고 D3-G 실차 재주행은
미완이다.** 무엇을 어떤 순서로 검증하고, 나빠지면 어떻게 되돌리는지는 **[ROLLBACK.md](ROLLBACK.md)**.
