# Vision Mode Command Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Key1 and Key2 select the MaixCAM geometric or playing-card publisher through a CRC-protected STM32-to-camera UART command.

**Architecture:** The existing binary frame envelope gains a control type with a one-byte mode payload. STM32 encodes and retries this command from the Vision UART task; MaixCAM incrementally parses commands and gates its existing publisher so only the selected result type is built and sent.

**Tech Stack:** STM32 HAL/C11, FreeRTOS, MaixPy Python, pytest, CTest.

## Global Constraints

- Keep USART1 and `/dev/ttyS0` at 115200-8-N-1.
- Reuse the existing header, version, sequence, length, CRC16-CCITT-FALSE and terminator.
- Do not restore the old ACK/commit handshake.
- A mode change must discard frozen data from the previous mode.
- Card feature extraction must not run in geometric mode.

---

### Task 1: Define and verify the mode command wire format

**Files:**
- Modify: `App/lib/vision_protocol/vision_protocol.h`
- Modify: `App/lib/vision_protocol/vision_protocol.c`
- Modify: `tests/vision_protocol/vision_protocol_test.c`
- Modify in MaixCAM: `serial_protocol.py`
- Create in MaixCAM: `tests/test_mode_command.py`

**Interfaces:**
- Produces `VisionProtocol_EncodeModeCommand(strategy, seq, buffer, capacity)`.
- Produces `ModeCommandParser.feed(data) -> list[int]`.

- [x] Add failing C tests for the exact 13-byte control frame and invalid inputs.
- [x] Add failing Python tests for fragmented/noisy input, CRC corruption and unknown modes.
- [x] Implement the C encoder and Python incremental parser.
- [x] Run the focused C and Python protocol tests until green.

### Task 2: Gate MaixCAM publishing by the selected mode

**Files:**
- Modify in MaixCAM: `vision_uart.py`
- Modify in MaixCAM: `main.py`
- Modify in MaixCAM: `tests/test_vision_uart_card_publish.py`

**Interfaces:**
- Produces `VisionUartPublisher.poll_mode_command()`.
- Exposes `VisionUartPublisher.mode` as `None`, geometric, or card-pattern.

- [x] Replace the interleaving expectation with failing mode-gating tests.
- [x] Poll commands before the measurement publish branch in every main-loop iteration.
- [x] Send only geometry in geometric mode and only card chunks in card mode.
- [x] Reset frozen results on a real mode change; keep duplicate commands idempotent.
- [x] Run all MaixCAM tests.

### Task 3: Send and retry the selected mode from STM32

**Files:**
- Modify: `App/lib/vision_uart/vision_uart.c`
- Modify: `App/lib/vision_uart/vision_uart.h`
- Modify: `App/lib/vision_protocol/README.md`

**Interfaces:**
- Consumes `DecisionTaskRequest.strategy` and `VisionProtocol_EncodeModeCommand()`.
- Sends immediately after receive startup, then every 100ms up to five attempts until desired data arrives.

- [x] Add mode command counters to `VisionUartOutput` for debugger visibility.
- [x] Send the command after a mission arms and stop retrying at the first matching result type.
- [x] Preserve mismatched-frame ignore behavior while a switch is taking effect.
- [x] Update the protocol documentation from receive-only to bidirectional control.

### Task 4: Full verification

**Files:**
- Modify: `progress.md`
- Modify: this plan, marking completed steps.

- [x] Run MaixCAM pytest and Python syntax compilation.
- [x] Run STM32 protocol, decision and simulation tests.
- [x] Build the Debug firmware.
- [x] Run `git diff --check` in both repositories and record hardware-only UART timing validation.
