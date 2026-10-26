import asyncio
from aiohttp import web
import threading
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
# import os
from pathlib import Path


class QboWebServer:
    def __init__(self, supervisor):
        self.supervisor = supervisor

        self.static_path = Path(get_package_share_directory('qbo_interface')) / 'web'
        if not self.static_path.exists():
            raise FileNotFoundError(f"[WEB ERROR] Web directory not found: {self.static_path}")

        self.app = web.Application()
        self.app.add_routes([
            web.get('/', self.handle_index),
            web.get('/ws', self.websocket_handler),
            web.static('/static', self.static_path)
        ])
        self.runner = web.AppRunner(self.app)

    async def handle_index(self, request):
        html_path = self.static_path / "index.html"
        if not html_path.exists():
            return web.Response(status=404, text="Error: index.html not found.")
        return web.FileResponse(path=html_path)

        # return web.Response(text="""
        # <html><head><title>Qbo Diagnostics</title></head>
        # <body style="font-family:sans-serif">
        # <h2>Diagnostics Viewer</h2>
        # <div id="graph" style="width: 100%; height: 500px; border: 1px solid #ccc;"></div>
        # <div id="status"></div>
        # <script src="https://unpkg.com/cytoscape@3.19.1/dist/cytoscape.min.js"></script>
        # <script>
        # const ws = new WebSocket("ws://" + location.host + "/ws");

        # let cy = cytoscape({
        #     container: document.getElementById('graph'),
        #     style: [
        #         { selector: 'node', style: { 'label': 'data(id)', 'text-valign': 'center', 'color': 'white', 'background-color': '#666' } },
        #         { selector: 'edge', style: { 'label': 'data(label)', 'curve-style': 'bezier', 'target-arrow-shape': 'triangle', 'width': 2, 'line-color': '#ccc', 'target-arrow-color': '#ccc' } },
        #         { selector: 'node.active', style: { 'background-color': 'green', 'border-color': 'black', 'border-width': 3 } }
        #     ],
        #     layout: { name: 'breadthfirst' }
        # });

        # ws.onmessage = event => {
        #     const data = JSON.parse(event.data);
        #     let html = "";

        #     // Display diagnostics
        #     for (const [k,v] of Object.entries(data.errors)) {
        #         html += `<div style='background:red;color:white;padding:5px;margin:5px;border-radius:8px;'>❌ ${k} : ${v[0]}</div>`;
        #     }
        #     for (const [k,v] of Object.entries(data.warnings)) {
        #         html += `<div style='background:orange;color:black;padding:5px;margin:5px;border-radius:8px;'>⚠️ ${k} : ${v[0]}</div>`;
        #     }
        #     document.getElementById("status").innerHTML = html;

        #     // Build graph
        #     cy.elements().remove();
        #     const states = data.smach.states || [];
        #     const transitions = data.smach.transitions || [];
        #     const active = data.smach.active;

        #     for (const s of states) {
        #         cy.add({ data: { id: s }, classes: s === active ? 'active' : '' });
        #     }
        #     for (const t of transitions) {
        #         cy.add({ data: { id: `${t.from}->${t.to}`, source: t.from, target: t.to, label: t.label } });
        #     }
        #     cy.layout({ name: 'breadthfirst' }).run();
        # };
        # </script>
        # </body></html>
        # """, content_type='text/html')


    async def websocket_handler(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        try:
            while True:
                await asyncio.sleep(1)
                data = {
                    "smach": self.supervisor.get_smach_graph(),
                    "warnings": self.supervisor.get_warn_dict(),
                    "errors": self.supervisor.get_error_dict()
                }
                await ws.send_json(data)
        except (asyncio.CancelledError, ConnectionResetError, web.WebSocketError, aiohttp.ClientConnectionError) as e:
            self.supervisor.get_logger().warn(f"[WebSocket] Connection closed: {e}")
        finally:
            await ws.close()

    async def start(self):
        await self.runner.setup()
        site = web.TCPSite(self.runner, '0.0.0.0', 8080)
        await site.start()


def run_web_server(server: QboWebServer):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(server.start())
    loop.run_forever()



if __name__ == '__main__':
    print("This script is not meant to be run standalone.")
