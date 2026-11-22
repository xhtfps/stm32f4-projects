import sys
from pathlib import Path
import unittest

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


SERVER = Path(__file__).resolve().parents[1] / "mcp_server.py"


class McpServerTests(unittest.IsolatedAsyncioTestCase):
    async def test_official_client_can_initialize_and_list_tools(self) -> None:
        server = StdioServerParameters(
            command=sys.executable,
            args=[str(SERVER)],
        )
        async with stdio_client(server) as (read, write):
            async with ClientSession(read, write) as session:
                initialized = await session.initialize()
                self.assertIn(initialized.protocolVersion, {
                    "2025-11-25", "2025-06-18", "2025-03-26", "2024-11-05",
                })
                tools = await session.list_tools()
                names = {tool.name for tool in tools.tools}
                self.assertTrue({"capture_and_analyze", "run_shape", "set_output"} <= names)
                run_tool = next(tool for tool in tools.tools if tool.name == "run_shape")
                self.assertEqual("object", run_tool.inputSchema["type"])
                self.assertIn("target_shape", run_tool.inputSchema["properties"])


if __name__ == "__main__":
    unittest.main()
