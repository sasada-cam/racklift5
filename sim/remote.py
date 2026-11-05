"""Task buttons for the MuJoCo viewer: a small local web page + a terminal prompt.

The MuJoCo viewer cannot hold custom buttons, so tasks are started from
  * http://127.0.0.1:<port>  - one button per task, shows the current task and step (opens automatically), or
  * the terminal             - type a task number and press Enter.
Both only put numbers into a queue; the viewer loop takes them out with poll().
Uses the standard library only.
"""
import html, json, queue, sys, threading, webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

PAGE = """<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RackLift5 tasks</title><style>
body{font:15px system-ui,sans-serif;margin:16px;max-width:420px;background:#f4f5f7;color:#222}
h1{font-size:18px;margin:0 0 12px} button{display:block;width:100%;margin:6px 0;padding:12px;font-size:15px;text-align:left;
border:1px solid #bbb;border-radius:8px;background:#fff;cursor:pointer} button:hover{background:#e8f0ff}
button:active{background:#cfe0ff} #st{margin-top:14px;padding:10px;border-radius:8px;background:#fff;border:1px solid #ddd;white-space:pre-line}
#log{margin-top:8px;padding:10px;border-radius:8px;background:#fbfbf3;border:1px solid #ddd;white-space:pre-line;font-size:13px;color:#444}
form{display:flex;gap:6px;margin-top:10px} input{flex:1;padding:10px;font-size:15px} form button{width:auto;margin:0}
</style></head><body><h1>RackLift5 v3 - tasks</h1>__BUTTONS__
<form onsubmit="go(this.n.value);this.n.value='';return false"><input name="n" placeholder="task number" inputmode="numeric"><button>Enter</button></form>
<div id="st">...</div>
<div id="log"></div>
<script>
function go(n){fetch('/run?n='+encodeURIComponent(n)).then(r=>r.json()).then(s);}
function s(j){document.getElementById('st').textContent=j.task+'\\n'+j.step+
  ((j.queue&&j.queue.length)?'\\n\\nnext: '+j.queue.join('\\n      '):'');
  document.getElementById('log').textContent=(j.log||[]).join('\\n');}
setInterval(()=>fetch('/status').then(r=>r.json()).then(s).catch(()=>{document.getElementById('st').textContent='viewer closed';}),500);
</script></body></html>"""


class Remote:
    def __init__(self, tasks, port=8765, browser=True, terminal=True):
        self.q = queue.Queue()
        self.status = dict(task="idle", step="", queued=0)
        self.tasks = tasks                                   # {number: name}
        buttons = "".join(f'<button onclick="go({n})"><b>{n}</b> &nbsp; {html.escape(name)}</button>' for n, name in tasks.items())
        page = PAGE.replace("__BUTTONS__", buttons).encode()
        remote = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, body, ctype):
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                u = urlparse(self.path)
                if u.path == "/run":
                    remote.push(parse_qs(u.query).get("n", [""])[0])
                if u.path in ("/run", "/status"):
                    return self._send(json.dumps(remote.status).encode(), "application/json")
                self._send(page, "text/html; charset=utf-8")

        self.url = None
        for p in range(port, port + 10):                    # next free port if 8765 is taken
            try:
                self.srv = ThreadingHTTPServer(("127.0.0.1", p), H)
                self.url = f"http://127.0.0.1:{p}"
                break
            except OSError:
                continue
        if self.url:
            threading.Thread(target=self.srv.serve_forever, daemon=True).start()
            if browser:
                webbrowser.open(self.url)
        if terminal and sys.stdin and sys.stdin.isatty():
            threading.Thread(target=self._stdin, daemon=True).start()

    def push(self, s):
        try:
            n = int(str(s).strip())
        except ValueError:
            return
        if n in self.tasks:
            self.q.put(n)

    def _stdin(self):
        for line in sys.stdin:
            self.push(line)

    def poll(self):
        out = []
        while not self.q.empty():
            out.append(self.q.get_nowait())
        return out

    def close(self):
        if self.url:
            self.srv.shutdown()
