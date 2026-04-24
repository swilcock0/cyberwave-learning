"""
Workflow Automation Example

List, trigger, monitor, and cancel workflows from the SDK.

Strategy (informed by production ROS 2 usage):
  1. List all workflow nodes to discover UUIDs.
  2. PATCH the Call Model node's input_mappings to bake in the text value.
  3. POST /trigger with {"inputs": {"node_uuid": <trigger_uuid>}} — the body
     format the backend actually expects.
  4. Poll execution status via /executions/{uuid}.

Requirements:
    pip install cyberwave
"""

from cyberwave import Cyberwave
import time
import base64
from config import TWIN_UUID, ENVIRONMENT_UUID

cw = Cyberwave()
cw.affect("live")

# ── Prompt to send to the VLA model ──────────────────────────────────
PROMPT = "Explain this scene in one sentence."
TWIN_MODEL = "waveshare/ugv-beast"


def list_workflow_nodes(client, workflow_uuid: str):
    """Fetch workflow nodes through the low-level API."""
    api = client.api.api_client
    param = api.param_serialize(
        method="GET",
        resource_path="/api/v1/workflows/{uuid}/nodes",
        path_params={"uuid": workflow_uuid},
        auth_settings=["CustomTokenAuthentication"],
    )
    response = api.call_api(*param)
    response.read()
    return api.response_deserialize(
        response_data=response,
        response_types_map={"200": "List[WorkflowNodeSchema]"},
    ).data


def find_trigger_mission_node(nodes):
    """Return the first enabled trigger:mission node from a node list."""
    for node in nodes:
        node_type = str(getattr(node, "node_type", "")).lower()
        trigger_type = str(getattr(node, "trigger_type", "")).lower()
        is_disabled = bool(getattr(node, "is_disabled", False))
        if node_type == "trigger" and trigger_type == "manual" and not is_disabled:
            return node
    return None


def find_call_model_node(nodes):
    """Return the first non-trigger, non-disabled node that looks like a model call."""
    for node in nodes:
        node_type = str(getattr(node, "node_type", "")).lower()
        is_disabled = bool(getattr(node, "is_disabled", False))
        if is_disabled:
            continue
        # Call model nodes typically have type containing "model", "action", "vla", or "ai"
        if any(kw in node_type for kw in ("model", "action", "vla", "ai", "call")):
            return node
    return None


def get_node(client, workflow_uuid: str, node_uuid: str):
    """GET a single workflow node."""
    api = client.api.api_client
    param = api.param_serialize(
        method="GET",
        resource_path="/api/v1/workflows/{uuid}/nodes/{node_uuid}",
        path_params={"uuid": workflow_uuid, "node_uuid": node_uuid},
        auth_settings=["CustomTokenAuthentication"],
    )
    response = api.call_api(*param)
    response.read()
    return api.response_deserialize(
        response_data=response,
        response_types_map={"200": "WorkflowNodeSchema"},
    ).data


def patch_node_inputs(client, workflow_uuid: str, node_uuid: str, input_mappings: dict):
    """PUT /api/v1/workflows/{uuid}/nodes/{node_uuid} to update a node's input_mappings.

    Reads the existing node metadata first so other fields (e.g. selected_model_uuid)
    are preserved in the PUT body.

    input_mappings format: {"field_name": {"mode": "value", "value": <val>}}
    """
    # Read existing metadata to avoid overwriting fields like selected_model_uuid
    existing = get_node(client, workflow_uuid, node_uuid)
    existing_metadata = dict(getattr(existing, "metadata", None) or {})
    existing_input_mappings = dict(existing_metadata.get("input_mappings", {}))
    existing_input_mappings.update(input_mappings)
    existing_metadata["input_mappings"] = existing_input_mappings

    api = client.api.api_client
    body = {"metadata": existing_metadata}
    param = api.param_serialize(
        method="PUT",
        resource_path="/api/v1/workflows/{uuid}/nodes/{node_uuid}",
        path_params={"uuid": workflow_uuid, "node_uuid": node_uuid},
        body=body,
        auth_settings=["CustomTokenAuthentication"],
    )
    response = api.call_api(*param)
    response.read()
    return api.response_deserialize(
        response_data=response,
        response_types_map={"200": "WorkflowNodeSchema"},
    ).data


def get_edge_photo_bytes(client, twin_uuid: str, environment_uuid: str | None = None):
    """Capture a fresh JPEG from the edge device (does not rely on Redis latest-frame)."""
    twin_kwargs = {"twin_id": twin_uuid}
    if environment_uuid:
        twin_kwargs["environment_id"] = environment_uuid
    twin = client.twin(TWIN_MODEL, **twin_kwargs)
    return twin.camera.edge_photo(format="bytes", timeout=5.0)


def trigger_workflow(client, workflow_uuid: str, trigger_node_uuid: str, extra_inputs: dict | None = None):
    """POST /trigger with the body shape the backend expects:
    {"inputs": {"node_uuid": <trigger_uuid>, ...extra_inputs}}
    Returns a WorkflowRunSchema.
    """
    api = client.api.api_client
    body = {
        "inputs": {
            "node_uuid": trigger_node_uuid,
            **(extra_inputs or {}),
        },
    }
    param = api.param_serialize(
        method="POST",
        resource_path="/api/v1/workflows/{uuid}/trigger",
        path_params={"uuid": workflow_uuid},
        body=body,
        auth_settings=["CustomTokenAuthentication"],
    )
    response = api.call_api(*param)
    response.read()
    return api.response_deserialize(
        response_data=response,
        response_types_map={"200": "WorkflowRunSchema"},
    ).data


def get_execution(client, workflow_uuid: str, execution_uuid: str):
    """Fetch execution by UUID using the workflow-scoped endpoint."""
    api = client.api.api_client
    param = api.param_serialize(
        method="GET",
        resource_path="/api/v1/workflows/{uuid}/executions/{execution_uuid}",
        path_params={"uuid": workflow_uuid, "execution_uuid": execution_uuid},
        auth_settings=["CustomTokenAuthentication"],
    )
    response = api.call_api(*param)
    response.read()
    return api.response_deserialize(
        response_data=response,
        response_types_map={"200": "WorkflowExecutionSchema"},
    ).data


def wait_for_run(client, workflow_uuid: str, run_uuid: str, timeout: float = 120.0, poll_interval: float = 3.0):
    """Poll execution status until terminal or timeout.
    WorkflowRunSchema has .execution_uuid linking to WorkflowExecutionSchema.
    Falls back to polling the run's own status field if execution_uuid absent.
    """
    terminal = {"success", "error", "canceled", "failed", "completed"}
    deadline = time.monotonic() + timeout

    while True:
        try:
            execution = get_execution(client, workflow_uuid, run_uuid)
            status = str(getattr(execution, "status", "")).lower()
            if status in terminal:
                return execution
        except Exception as exc:
            print(f"  [poll] {exc}")
            execution = None

        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"Run {run_uuid} did not complete within {timeout}s")
        time.sleep(min(poll_interval, remaining))


def _extract_text_from_output_item(item):
    """Extract model text from heterogeneous output payloads.

    Prioritize model-generated keys over generic/echo keys like "Result".
    """
    if isinstance(item, str):
        return item

    if isinstance(item, dict):
        # Prefer explicit model result keys first.
        preferred_keys = [
            "model_result",
            "result",
            "response",
            "output",
            "content",
            "answer",
            "text",
            "Result",
        ]
        for key in preferred_keys:
            value = item.get(key)
            if isinstance(value, str) and value.strip():
                return value

        # Recurse into nested values if top-level keys were not useful.
        for value in item.values():
            nested = _extract_text_from_output_item(value)
            if nested:
                return nested

    if isinstance(item, list):
        for value in item:
            nested = _extract_text_from_output_item(value)
            if nested:
                return nested

    return None


# ── List available workflows ──────────────────────────────────────────

workflows = cw.workflows.list()
for wf in workflows:
    print(f"{wf.name} ({wf.uuid}) — {wf.status}")

if not workflows:
    print("No workflows found. Create one in the Cyberwave dashboard first.")
    exit()

# ── Find active VLA_CheckForHazards ──────────────────────────────────

workflow = next(
    (wf for wf in workflows if wf.is_active and wf.name == "VLA_CheckForHazards"),
    None,
)
if workflow is None:
    print("Active workflow 'VLA_CheckForHazards' not found.")
    exit()

print(f"\nFound workflow: '{workflow.name}' ({workflow.uuid})")

# ── Discover nodes ────────────────────────────────────────────────────

nodes = list_workflow_nodes(cw, workflow.uuid)
print(f"\nNodes ({len(nodes)}):")
for n in nodes:
    print(
        f"  [{n.node_type}:{getattr(n, 'trigger_type', '')}]"
        f"  {n.name}  ({n.uuid})"
        f"  disabled={getattr(n, 'is_disabled', False)}"
    )

trigger_node = find_trigger_mission_node(nodes)
if trigger_node is None:
    print("No enabled trigger:mission node found.")
    exit()

print(f"\nTrigger node : {trigger_node.uuid} | {trigger_node.name}")

# ── Patch the Call Model node's text input ────────────────────────────

model_node = find_call_model_node(nodes)
if model_node:
    print(f"Model node   : {model_node.uuid} | {model_node.name} ({model_node.node_type})")
    print(f"Patching text/image_bytes inputs ...")
    # Some workflows use `prompt`, others use `text`; set both to guarantee prompt delivery.
    model_inputs = {
        "prompt": {"mode": "value", "value": PROMPT},
        "text": {"mode": "value", "value": PROMPT},
    }

    # Bypass twin->workflow Redis frame dependency: capture edge photo and inject image_bytes.
    image_bytes_value = None
    try:
        frame_bytes = get_edge_photo_bytes(cw, TWIN_UUID, ENVIRONMENT_UUID)
        if frame_bytes:
            # Metadata is JSON-serialized, so send image bytes as ASCII base64.
            # Use a data URI prefix to help backend decoders infer MIME type.
            image_bytes_value = "data:image/jpeg;base64," + base64.b64encode(frame_bytes).decode("ascii")
            print(f"Captured frame bytes: {len(frame_bytes)} bytes")
    except Exception as exc:
        print(f"Edge photo unavailable, continuing with text only: {exc}")

    if image_bytes_value:
        model_inputs["image_bytes"] = {"mode": "value", "value": image_bytes_value}

    # Explicitly clear URL input so call_model uses image_bytes.
    model_inputs["image_url"] = {"mode": "value", "value": None}

    patch_node_inputs(
        cw,
        workflow.uuid,
        str(model_node.uuid),
        model_inputs,
    )
    print("Patch applied.")
else:
    print("No call-model node found — text will be passed via trigger inputs instead.")

# ── Trigger the workflow ──────────────────────────────────────────────

print(f"\nTriggering '{workflow.name}' ...")
run = trigger_workflow(
    cw,
    workflow.uuid,
    str(trigger_node.uuid),
    extra_inputs={"prompt": PROMPT, "text": PROMPT} if not model_node else None,
)

run_uuid = str(getattr(run, "uuid", "") or getattr(run, "execution_uuid", ""))
run_status = str(getattr(run, "status", ""))
print(f"Run started  : {run_uuid}  (status: {run_status})")

if not run_uuid:
    print("No run UUID returned — cannot poll.")
    cw.disconnect()
    exit()

# ── Wait for the run to finish ────────────────────────────────────────

execution = wait_for_run(cw, workflow.uuid, run_uuid, timeout=120, poll_interval=3)

if execution:
    print(f"\nFinal status : {execution.status}")
    started_at = getattr(execution, "started_at", None)
    finished_at = getattr(execution, "finished_at", None)
    if started_at and finished_at:
        print(f"Started      : {started_at}")
        print(f"Finished     : {finished_at}")
    error_message = getattr(execution, "error_message", None)
    if error_message:
        print(f"Error        : {error_message}")

    node_execs = getattr(execution, "node_executions", None) or []
    # Find the Call Model node execution and extract its text output
    model_node_exec = next(
        (nx for nx in node_execs if str(getattr(nx, "node_uuid", "")) == str(model_node.uuid)),
        None,
    ) if model_node else None

    if model_node_exec:
        print(f"\nCall Model status : {model_node_exec.status}")
        output_data = getattr(model_node_exec, "output_data", []) or []
        # output_data can be nested/unstructured; prefer model_result when present
        text_result = None
        for item in output_data:
            text_result = _extract_text_from_output_item(item)
            if text_result:
                break
        if text_result:
            print(f"\nModel output:\n{text_result}")
        elif output_data:
            print(f"\nModel output_data (raw):\n{output_data}")
        else:
            print("No output data from Call Model node.")
    else:
        # Debug: show all node executions so we can find the right one
        print(f"\nCall Model node exec not found. All node execs:")
        for nx in node_execs:
            print(f"  node_uuid={getattr(nx, 'node_uuid', '?')}  status={getattr(nx, 'status', '?')}")
            print(f"    output_data={getattr(nx, 'output_data', [])}")

# ── Browse past runs ──────────────────────────────────────────────────

past_runs = workflow.runs(status="success")
print(f"\n{len(past_runs)} successful past runs for '{workflow.name}'")

cw.disconnect()
