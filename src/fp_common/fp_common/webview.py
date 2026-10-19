import json
import threading
import time

import cv2
import numpy as np
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

_HTML = """<!doctype html><html><head><meta charset=utf-8><title>__TITLE__</title>
<style>body{background:#222;color:#ddd;font-family:monospace;margin:10px}
img{max-width:100%;border:1px solid #555;cursor:crosshair}
.slider{margin:4px 0} .slider label{display:inline-block;width:70px}
.slider input{width:420px;vertical-align:middle}
button{margin:2px;padding:6px 14px;background:#333;color:#eee;border:1px solid #666;cursor:pointer}
button:hover{background:#444}
#status{white-space:pre;margin:8px 0;color:#8f8}</style></head><body>
<h3>__TITLE__</h3><img id=iv src=/stream>
<div id=sliders></div><div id=buttons></div><div id=status></div>
<script>
const SL=__SLIDERS__; let dragging=null;
const sldiv=document.getElementById('sliders');
for(const k in SL){const s=SL[k];const d=document.createElement('div');d.className='slider';
 d.innerHTML=`<label>${k}</label><input type=range min=${s.min} max=${s.max} step=${s.step} value=${s.init} id=r_${k}> <span id=v_${k}>${s.init}</span>`;
 sldiv.appendChild(d);const r=d.querySelector('input');
 r.addEventListener('mousedown',()=>dragging=k); r.addEventListener('mouseup',()=>dragging=null);
 r.addEventListener('input',()=>{document.getElementById('v_'+k).textContent=r.value; fetch(`/set?k=${k}&v=${r.value}`);});}
const bt=document.getElementById('buttons');
for(const b of __BUTTONS__){const el=document.createElement('button');el.textContent=b;el.onclick=()=>fetch('/btn?k='+b);bt.appendChild(el);}
const iv=document.getElementById('iv');
iv.addEventListener('click',e=>{const r=iv.getBoundingClientRect();
 const x=(e.clientX-r.left)*iv.naturalWidth/r.width, y=(e.clientY-r.top)*iv.naturalHeight/r.height;
 fetch(`/click?x=${x}&y=${y}`);});
async function poll(){try{const s=await (await fetch('/status')).json();
 document.getElementById('status').textContent=s.status;
 if(!dragging) for(const k in SL){const r=document.getElementById('r_'+k);
   if(Math.abs(parseFloat(r.value)-s.params[k])>SL[k].step) r.value=s.params[k];
   document.getElementById('v_'+k).textContent=(+s.params[k]).toFixed(2);}
 }catch(e){} setTimeout(poll,300);}
poll();
</script></body></html>"""

class WebPreview:
    def __init__(self, port, title, sliders=None, buttons=None,
                 on_set=None, on_button=None, on_click=None):
        self.sliders = sliders or {}
        self.buttons = buttons or []
        self.on_set = on_set
        self.on_button = on_button
        self.on_click = on_click
        self.params = {k: v['init'] for k, v in self.sliders.items()}
        self.status = ''
        self._img = None
        self._lock = threading.Lock()
        self._stop = False
        pv = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, ctype, body=b''):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                if body:
                    self.wfile.write(body)

            def do_GET(self):
                u = urlparse(self.path)
                q = parse_qs(u.query)
                if u.path == '/':
                    html = (_HTML.replace('__TITLE__', pv.title)
                            .replace('__SLIDERS__', json.dumps(pv.sliders))
                            .replace('__BUTTONS__', json.dumps(pv.buttons)))
                    self._send(200, 'text/html; charset=utf-8', html.encode())
                elif u.path == '/stream':
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.end_headers()
                    try:
                        while not pv._stop:
                            with pv._lock:
                                img = pv._img
                            if img is not None:
                                ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 82])
                                if ok:
                                    jdata = buf.tobytes()
                                    part = (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                            + str(len(jdata)).encode() + b'\r\n\r\n' + jdata + b'\r\n')
                                    self.wfile.write(part)
                            time.sleep(0.06)
                    except Exception:
                        pass
                elif u.path == '/status':
                    body = json.dumps({'status': pv.status, 'params': pv.params}).encode()
                    self._send(200, 'application/json', body)
                elif u.path == '/set':
                    k, v = q['k'][0], float(q['v'][0])
                    pv.params[k] = v
                    if pv.on_set:
                        pv.on_set(k, v)
                    self._send(200, 'text/plain', b'ok')
                elif u.path == '/btn':
                    k = q['k'][0]
                    if pv.on_button:
                        pv.on_button(k)
                    self._send(200, 'text/plain', b'ok')
                elif u.path == '/click':
                    if pv.on_click:
                        pv.on_click(float(q['x'][0]), float(q['y'][0]))
                    self._send(200, 'text/plain', b'ok')
                else:
                    self._send(404, 'text/plain', b'')

        self.title = title
        self.server = ThreadingHTTPServer(('0.0.0.0', port), H)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def set_image(self, bgr):
        with self._lock:
            self._img = np.ascontiguousarray(bgr)

    def set_params(self, d):
        self.params.update(d)

    @property
    def stopped(self):
        return self._stop

    def request_stop(self):
        self._stop = True

    def shutdown(self):
        self._stop = True
        self.server.shutdown()
        self.server.server_close()
