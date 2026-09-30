"""Chat backend for the iw_hub console, using your Claude Pro subscription via the Claude Code CLI.

browser <-> FastAPI <-> `claude -p` (your Pro login) <-> robot_mcp.py <-> ROS 2 services

Also serves ../frontend, so the console and its Assistant tab share one origin.
Personal use only. Run from a shell with ROS 2 + this workspace sourced:
  .venv/bin/uvicorn server:app --port 8000   then open http://localhost:8000
"""

import asyncio
import json
import os
import shutil
import uuid
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import RedirectResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

HERE = Path(__file__).parent.resolve()
FRONTEND_DIR = HERE.parent / "frontend"

# "sonnet" uses less of your Pro usage limit than "opus". Override with ROBOT_CHAT_MODEL.
MODEL = os.environ.get("ROBOT_CHAT_MODEL", "sonnet")
SYSTEM_PROMPT = (
    "You are the assistant for iw_hub, an autonomous mobile robot (diff-drive AMR with a lift) "
    "running in NVIDIA Isaac Sim with ROS 2 and Nav2. Chat normally with the user. "
    "When the user asks the robot to move, turn, report where it is, or run a pick-and-place "
    "mission, use the robot tools. 'front'/'ahead' means forward. Distances are in meters "
    "unless the user says otherwise; convert other units. Straight moves have no obstacle "
    "avoidance, so if a request is ambiguous, ask before moving. "
    "After a tool runs, briefly tell the user what the robot did."
)

MCP_CONFIG = json.dumps(
    {"mcpServers": {"robot": {"command": str(HERE / ".venv/bin/python"), "args": [str(HERE / "robot_mcp.py")]}}}
)
ALLOWED_TOOLS = [
    "mcp__robot__move_robot",
    "mcp__robot__get_robot_state",
    "mcp__robot__list_waypoints",
    "mcp__robot__run_mission",
]

CLAUDE_BIN = shutil.which("claude") or str(Path.home() / ".local/bin/claude")

# Never pass API credentials to the CLI - it would bill the API instead of using Pro.
# CLAUDECODE is dropped so this also works when uvicorn is started from a Claude Code terminal.
CLI_ENV = {
    k: v for k, v in os.environ.items() if k not in ("ANTHROPIC_API_KEY", "ANTHROPIC_AUTH_TOKEN", "CLAUDECODE")
}

app = FastAPI()


class ChatRequest(BaseModel):
    session_id: str | None = None
    message: str


def sse(event: dict) -> str:
    return f"data: {json.dumps(event)}\n\n"


def is_uuid(value: str | None) -> bool:
    try:
        uuid.UUID(value or "")
        return True
    except ValueError:
        return False


def tool_result_text(content) -> str:
    if isinstance(content, str):
        return content
    return "".join(part.get("text", "") for part in content or [] if isinstance(part, dict))


async def chat_stream(session_id: str | None, user_text: str):
    args = [
        CLAUDE_BIN, "-p",
        "--output-format", "stream-json", "--verbose", "--include-partial-messages",
        "--model", MODEL,
        "--system-prompt", SYSTEM_PROMPT,
        "--tools", "",  # no built-in tools (no Bash, file edits, web, ...)
        "--strict-mcp-config", "--mcp-config", MCP_CONFIG,
        "--allowedTools", *ALLOWED_TOOLS,
    ]
    if is_uuid(session_id):
        args += ["--resume", session_id]

    proc = await asyncio.create_subprocess_exec(
        *args,
        cwd=HERE,
        env=CLI_ENV,
        stdin=asyncio.subprocess.PIPE,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
        limit=16 * 1024 * 1024,  # stream-json lines can be large
    )
    # Send the message on stdin so text starting with "-" is never parsed as a CLI flag.
    proc.stdin.write(user_text.encode())
    proc.stdin.close()

    got_result = False
    try:
        async for line in proc.stdout:
            try:
                ev = json.loads(line)
            except json.JSONDecodeError:
                continue
            kind = ev.get("type")

            if kind == "system" and ev.get("subtype") == "init":
                yield sse({"type": "session", "session_id": ev["session_id"]})

            elif kind == "stream_event" and ev.get("parent_tool_use_id") is None:
                inner = ev.get("event", {})
                delta = inner.get("delta", {})
                if inner.get("type") == "content_block_delta" and delta.get("type") == "text_delta":
                    yield sse({"type": "text", "text": delta["text"]})

            elif kind == "assistant":
                for block in ev["message"].get("content", []):
                    if block.get("type") == "tool_use":
                        name = block["name"].removeprefix("mcp__robot__")
                        yield sse({"type": "tool_call", "name": name, "input": block.get("input")})

            elif kind == "user":
                content = ev["message"].get("content")
                for block in content if isinstance(content, list) else []:
                    if block.get("type") == "tool_result":
                        yield sse({
                            "type": "tool_result",
                            "output": tool_result_text(block.get("content")),
                            "is_error": bool(block.get("is_error")),
                        })

            elif kind == "result":
                got_result = True
                if ev.get("is_error") or ev.get("subtype") != "success":
                    detail = ev.get("result") or "; ".join(ev.get("errors", [])) or ev.get("subtype")
                    yield sse({"type": "error", "message": f"Claude Code error: {detail}"})

        await proc.wait()
        if not got_result:
            stderr = (await proc.stderr.read()).decode(errors="replace").strip()
            yield sse({"type": "error", "message": f"claude exited with code {proc.returncode}: {stderr[-500:]}"})
    finally:
        if proc.returncode is None:  # browser disconnected mid-reply
            proc.kill()
            await proc.wait()

    yield sse({"type": "done"})


@app.post("/api/chat")
async def chat(req: ChatRequest):
    return StreamingResponse(chat_stream(req.session_id, req.message), media_type="text/event-stream")


@app.post("/api/reset")
async def reset(req: ChatRequest):
    # The page forgets its session id; the next message starts a new Claude Code session.
    return {"ok": True}


@app.get("/")
async def index():
    return RedirectResponse("/iw_hub_console.html")


# Registered last so it never shadows the /api routes.
app.mount("/", StaticFiles(directory=FRONTEND_DIR), name="frontend")
