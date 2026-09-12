r"""mcp_pdf_client.py —— 直接驱动 pdf-reader MCP server 的轻量客户端（DSH 环境专用）

DSS/DSH harness 没有 MCP 客户端，此脚本通过 stdio JSON-RPC 直接与
.mcp.json 里配置的 pdf-reader server 握手并调用其工具。

用法:
  python mcp_pdf_client.py <pdf路径> <tool1> [tool2 ...]
  tool 可选: get_pdf_info, read_pdf_text, analyze_pdf_structure,
             extract_pdf_images, read_pdf_with_ocr
示例:
  python mcp_pdf_client.py 某个.pdf get_pdf_info read_pdf_text
"""
import json, subprocess, sys, threading, queue, os

SERVER_PY = r"D:\HERO\hero2026\rt-thread_-hero\common\utils\MCP\PDF\mcp_pdf_reader\src\server.py"
MCP_PY    = r"C:\Users\LR\miniconda3\envs\mcp_pdf\python.exe"

class McpClient:
    def __init__(self):
        self.proc = subprocess.Popen([MCP_PY, SERVER_PY],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL, text=True, encoding="utf-8")
        self.q = queue.Queue()
        threading.Thread(target=self._reader, daemon=True).start()
        self._id = 0

    def _reader(self):
        for line in self.proc.stdout:
            self.q.put(line.strip())

    def request(self, method, params=None, timeout=120):
        self._id += 1
        rid = self._id
        msg = {"jsonrpc": "2.0", "id": rid, "method": method}
        if params is not None:
            msg["params"] = params
        self.proc.stdin.write(json.dumps(msg) + "\n")
        self.proc.stdin.flush()
        while True:
            line = self.q.get(timeout=timeout)
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                continue
            if data.get("id") == rid:
                if "error" in data:
                    raise RuntimeError(data["error"])
                return data.get("result")

    def notify(self, method):
        self.proc.stdin.write(json.dumps({"jsonrpc": "2.0", "method": method}) + "\n")
        self.proc.stdin.flush()

    def call_tool(self, name, args):
        return self.request("tools/call", {"name": name, "arguments": args})

def main():
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    pdf, tools = sys.argv[1], sys.argv[2:]
    c = McpClient()
    c.request("initialize", {"protocolVersion": "2024-11-05", "capabilities": {},
                             "clientInfo": {"name": "dsh-driver", "version": "1.0"}})
    c.notify("notifications/initialized")

    for t in tools:
        args = {"file_path": os.path.abspath(pdf)}
        if t == "read_pdf_with_ocr":
            args["ocr_language"] = "chi_sim+eng"
        print(f"\n===== {t} =====", flush=True)
        try:
            res = c.call_tool(t, args)
            for item in (res or {}).get("content", []):
                if item.get("type") == "text":
                    print(item["text"])
        except Exception as e:
            print(f"[ERROR] {t}: {e}")

if __name__ == "__main__":
    main()
