"""Web render backend: aiohttp WebSocket server + Three.js viewer (no GPU, no Isaac)."""
from .server import make_app, serve, cell_config, snapshot  # noqa: F401
