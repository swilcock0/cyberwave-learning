# Workflow Automation

This folder contains a low-level Cyberwave workflow runner that discovers nodes, patches model inputs, triggers execution, and prints model output.

## Main Script

- `Workflow.py`

## What It Does

For workflow `VLA_CheckForHazards`, the script performs this sequence:

1. List workflows and select active target workflow.
2. List workflow nodes and detect:
	- manual trigger node
	- call-model node
3. Read existing call-model node metadata and merge new input mappings.
4. Inject prompt values (`prompt` and `text`).
5. Capture a fresh edge camera frame and inject it as `image_bytes` (base64 data URI), while clearing `image_url`.
6. Trigger the workflow using:

	```json
	{"inputs": {"node_uuid": "<trigger-node-uuid>"}}
	```

7. Poll execution status via workflow-scoped endpoint.
8. Extract and print model output (prioritizing `model_result`).

## Prerequisites

1. Install dependency:

	```bash
	pip install cyberwave
	```

2. Provide a `config.py` visible to Python import path with:

	```python
	TWIN_UUID = "..."
	ENVIRONMENT_UUID = "..."
	```

3. Ensure the workflow exists and is active in Cyberwave:

	- Name: `VLA_CheckForHazards`
	- Contains at least one enabled manual trigger node

## Run

From repository root:

```bash
python Intelligence/Workflows/Workflow.py
```

## Expected Output

- Workflow and node discovery printout
- Trigger run UUID and final execution status
- Call-model node status
- Final model sentence (if available in `output_data`)

## Customization

- Prompt text: edit `PROMPT` in `Workflow.py`.
- Workflow name filter: edit `VLA_CheckForHazards` string in the workflow selection block.
- Poll timeout and interval: update `wait_for_run(..., timeout=..., poll_interval=...)`.

## Troubleshooting

- 404 on execution polling: verify workflow-scoped endpoint is used.
- Trigger errors about missing node UUID: ensure `inputs.node_uuid` is sent.
- Call-model configuration disappears after patch: confirm metadata merge is preserved before PUT.
- No model text printed: inspect printed raw `output_data` and confirm your node emits `model_result` or another supported text key.
