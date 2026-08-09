"""Wi-Fi and record-enabled WebRTC web page helpers."""

from http.server import BaseHTTPRequestHandler, HTTPServer
import threading

from maix import err, network


WEB_PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
  <title>MaixCAM WebRTC</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; background: #15171a; color: #fff; }
    main { width: min(920px, 100%); margin: auto; padding: 16px; }
    h1 { margin: 4px 0 14px; font-size: 22px; }
    .video-wrap { position: relative; overflow: hidden; border-radius: 14px; background: #000; }
    video { display: block; width: 100%; max-height: 70vh; background: #000; }
    .badge { position: absolute; left: 10px; bottom: 10px; padding: 5px 9px;
             border-radius: 8px; background: #000a; font-size: 13px; }
    .controls { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-top: 14px; }
    button { min-height: 48px; border: 0; border-radius: 12px; font-size: 17px;
             font-weight: 700; color: white; }
    button:disabled { opacity: .42; }
    #start { background: #16894c; }
    #stop { background: #c93636; }
    .status { display: flex; justify-content: space-between; gap: 12px; margin-top: 12px;
              padding: 10px 12px; border-radius: 10px; background: #ffffff0d; }
    #download { display: none; margin-top: 12px; padding: 12px; border-radius: 10px;
                text-align: center; background: #1769aa; color: #fff; text-decoration: none; }
    .hint { color: #b9bec7; font-size: 13px; line-height: 1.5; }
  </style>
</head>
<body>
<main>
  <h1>MaixCAM 实时画面</h1>
  <div class="video-wrap">
    <video id="video" autoplay playsinline muted></video>
    <div class="badge"><span id="resolution">等待视频</span></div>
  </div>
  <div class="controls">
    <button id="start" disabled>开始录制</button>
    <button id="stop" disabled>停止并保存</button>
  </div>
  <div class="status">
    <span id="state">正在连接……</span>
    <span id="timer">00:00</span>
  </div>
  <a id="download">下载录制视频</a>
  <p class="hint">视频由当前浏览器录制并保存在手机或电脑中，不写入 MaixCAM。</p>
</main>
<script>
  const video = document.getElementById('video');
  const startButton = document.getElementById('start');
  const stopButton = document.getElementById('stop');
  const stateText = document.getElementById('state');
  const timerText = document.getElementById('timer');
  const resolutionText = document.getElementById('resolution');
  const downloadLink = document.getElementById('download');

  let peerConnection = null;
  let mediaRecorder = null;
  let chunks = [];
  let timerHandle = null;
  let recordStartedAt = 0;
  let lastDownloadUrl = null;

  const clientId = Array.from(crypto.getRandomValues(new Uint8Array(10)))
    .map(value => (value % 36).toString(36)).join('');
  const wsUrl = `ws://${location.hostname}:__SIGNALING_PORT__/${clientId}`;
  const websocket = new WebSocket(wsUrl);

  websocket.onopen = () => {
    stateText.textContent = '信令已连接，等待视频';
    websocket.send(JSON.stringify({id: 'server', type: 'request'}));
  };

  websocket.onmessage = async event => {
    if (typeof event.data !== 'string') return;
    const message = JSON.parse(event.data);
    if (message.type === 'offer') await handleOffer(message);
  };

  websocket.onerror = () => { stateText.textContent = '连接失败，请刷新页面'; };
  websocket.onclose = () => {
    stateText.textContent = '连接已断开';
    startButton.disabled = true;
    if (mediaRecorder && mediaRecorder.state === 'recording') mediaRecorder.stop();
  };

  function createPeerConnection() {
    const pc = new RTCPeerConnection({
      bundlePolicy: 'max-bundle',
      // Phone and MaixCAM are on the same local AP; public STUN is unnecessary.
      iceServers: []
    });
    pc.ontrack = event => {
      video.srcObject = event.streams[0];
      video.play().catch(() => {});
      startButton.disabled = false;
      stateText.textContent = '视频已连接';
    };
    pc.onconnectionstatechange = () => {
      if (['failed', 'disconnected', 'closed'].includes(pc.connectionState)) {
        stateText.textContent = `WebRTC ${pc.connectionState}`;
        startButton.disabled = true;
      }
    };
    return pc;
  }

  async function waitForIce() {
    if (peerConnection.iceGatheringState === 'complete') return;
    await new Promise(resolve => {
      const listener = () => {
        if (peerConnection.iceGatheringState === 'complete') {
          peerConnection.removeEventListener('icegatheringstatechange', listener);
          resolve();
        }
      };
      peerConnection.addEventListener('icegatheringstatechange', listener);
    });
  }

  async function handleOffer(offer) {
    if (peerConnection) peerConnection.close();
    peerConnection = createPeerConnection();
    await peerConnection.setRemoteDescription(offer);
    await peerConnection.setLocalDescription(await peerConnection.createAnswer());
    await waitForIce();
    websocket.send(JSON.stringify({
      id: 'server',
      type: peerConnection.localDescription.type,
      sdp: peerConnection.localDescription.sdp
    }));
  }

  video.addEventListener('loadedmetadata', () => {
    resolutionText.textContent = `${video.videoWidth} × ${video.videoHeight}`;
  });

  function chooseMimeType() {
    const types = [
      'video/webm;codecs=vp8,opus',
      'video/webm;codecs=vp8',
      'video/webm',
      'video/mp4'
    ];
    return types.find(type => MediaRecorder.isTypeSupported(type)) || '';
  }

  function updateTimer() {
    const seconds = Math.floor((Date.now() - recordStartedAt) / 1000);
    timerText.textContent = `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
  }

  startButton.onclick = () => {
    if (!video.srcObject || typeof MediaRecorder === 'undefined') {
      stateText.textContent = '当前浏览器不支持视频录制';
      return;
    }
    chunks = [];
    downloadLink.style.display = 'none';
    if (lastDownloadUrl) URL.revokeObjectURL(lastDownloadUrl);
    const mimeType = chooseMimeType();
    mediaRecorder = new MediaRecorder(video.srcObject, mimeType ? {mimeType} : undefined);
    mediaRecorder.ondataavailable = event => {
      if (event.data && event.data.size) chunks.push(event.data);
    };
    mediaRecorder.onerror = event => {
      stateText.textContent = `录制失败：${event.error?.message || '未知错误'}`;
    };
    mediaRecorder.onstop = () => {
      clearInterval(timerHandle);
      const finalType = mediaRecorder.mimeType || mimeType || 'video/webm';
      const blob = new Blob(chunks, {type: finalType});
      lastDownloadUrl = URL.createObjectURL(blob);
      const extension = finalType.includes('mp4') ? 'mp4' : 'webm';
      const stamp = new Date().toISOString().replace(/[:.]/g, '-');
      downloadLink.href = lastDownloadUrl;
      downloadLink.download = `maixcam-${stamp}.${extension}`;
      downloadLink.textContent = `下载录制视频（${(blob.size / 1048576).toFixed(1)} MB）`;
      downloadLink.style.display = 'block';
      stateText.textContent = '录制完成，请点击下载';
      startButton.disabled = false;
      stopButton.disabled = true;
    };
    mediaRecorder.start(1000);
    recordStartedAt = Date.now();
    updateTimer();
    timerHandle = setInterval(updateTimer, 500);
    stateText.textContent = '正在录制';
    startButton.disabled = true;
    stopButton.disabled = false;
  };

  stopButton.onclick = () => {
    if (mediaRecorder && mediaRecorder.state === 'recording') mediaRecorder.stop();
  };
</script>
</body>
</html>
"""


class ReusableHTTPServer(HTTPServer):
    allow_reuse_address = True


class WebPageHandler(BaseHTTPRequestHandler):
    page_body = b""

    def do_GET(self):
        if self.path not in ("/", "/index.html"):
            self.send_error(404)
            return
        body = self.page_body
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, _format, *_args):
        pass


def start_access_point(
    ssid,
    password,
    ip="192.168.66.1",
    netmask="255.255.255.0",
    mode="g",
    channel=6,
):
    """Start a deterministic 2.4 GHz AP for direct phone connection."""
    wifi = network.wifi.Wifi()
    print("Starting WiFi AP: {}".format(ssid))
    result = wifi.start_ap(
        ssid,
        password,
        mode=mode,
        channel=channel,
        ip=ip,
        netmask=netmask,
        hidden=False,
    )
    err.check_raise(result, "Failed to start WiFi AP")
    if not wifi.is_ap_mode():
        raise RuntimeError("WiFi start_ap returned success, but AP mode is off")
    print("WiFi AP ready: {} http://{}".format(ssid, ip))
    return wifi, ip


def connect_wifi(ssid, password, timeout_s=60):
    """Connect as a station without calling unsupported disconnect APIs."""
    wifi = network.wifi.Wifi()
    current_ip = wifi.get_ip()
    current_ssid = ""
    try:
        current_ssid = wifi.get_ssid()
    except Exception:
        pass

    if current_ip and current_ssid == ssid:
        print("WiFi already connected: {} {}".format(ssid, current_ip))
        return wifi, current_ip

    print("Connecting to WiFi: {}".format(ssid))
    result = wifi.connect(
        ssid,
        password,
        wait=True,
        timeout=timeout_s,
    )
    err.check_raise(result, "Failed to connect to WiFi")
    ip = wifi.get_ip()
    if not ip:
        raise RuntimeError("WiFi connected, but no IP address was assigned")
    print("WiFi connected: {}".format(ip))
    return wifi, ip


def start_web_page(web_port=8000, signaling_port=8001):
    """Start the record-enabled browser page in a daemon thread."""
    WebPageHandler.page_body = WEB_PAGE.replace(
        "__SIGNALING_PORT__", str(signaling_port)
    ).encode("utf-8")
    httpd = ReusableHTTPServer(("0.0.0.0", web_port), WebPageHandler)
    thread = threading.Thread(
        target=httpd.serve_forever,
        name="webrtc-web",
        daemon=True,
    )
    thread.start()
    return httpd
