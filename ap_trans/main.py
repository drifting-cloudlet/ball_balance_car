from maix import network, err, camera, http, app, display, image


CONFIG = {
    "ap_ssid": "MaixCAM-Pro",
    "ap_password": "18458394456",
    "ap_channel": 6,
    "ap_ip": "192.168.66.1",
    "ap_netmask": "255.255.255.0",
    # 这台设备软件JPEG编码吃CPU，分辨率一超过640x480就明显卡顿，所以定得比摄像头上限(2560x1440)低很多
    # 320x240是历史上唯一真机确认流畅的分辨率（400x300/480x360均未验证过或已确认偏卡），先退回这个已知可行值
    "width": 320,
    "height": 240,
    # to_jpeg()有效范围(50,100]，51是允许的最低值
    "jpeg_quality": 51,
    "screen_url_scale": 1.5,
}


HTML_PAGE = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
  <title>MaixCAM Pro Stream</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; background: #15171a; color: #fff; }
    main { width: min(920px, 100%); margin: auto; padding: 16px; }
    h1 { margin: 4px 0 14px; font-size: 22px; }
    .video-wrap { position: relative; overflow: hidden; border-radius: 14px; background: #000; }
    .video-wrap img { display: block; width: 100%; max-height: 70vh; background: #000; }
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
  <h1>MaixCAM Pro 实时画面</h1>
  <div class="video-wrap">
    <img id="view" src="/stream" alt="Stream">
    <div class="badge"><span id="resolution">连接中</span></div>
  </div>
  <div class="controls">
    <button id="start" disabled>开始录制</button>
    <button id="stop" disabled>停止并保存</button>
  </div>
  <div class="status">
    <span id="state">正在连接画面……</span>
    <span id="timer">00:00</span>
  </div>
  <a id="download">下载录制视频</a>
  <p class="hint">画面是 HTTP/JPEG 推流（不是 WebRTC）。录制原理是把每一帧画到隐藏 canvas 上再用 MediaRecorder 录制，帧率取决于推流帧率，比原生视频录制略有损耗。视频保存在手机/电脑本地，不写入 MaixCAM 的 TF 卡。</p>
  <canvas id="canvas" style="display:none"></canvas>
</main>
<script>
  const img = document.getElementById('view');
  const canvas = document.getElementById('canvas');
  const ctx = canvas.getContext('2d');
  const startButton = document.getElementById('start');
  const stopButton = document.getElementById('stop');
  const stateText = document.getElementById('state');
  const timerText = document.getElementById('timer');
  const resolutionText = document.getElementById('resolution');
  const downloadLink = document.getElementById('download');

  let mediaRecorder = null;
  let chunks = [];
  let timerHandle = null;
  let drawHandle = null;
  let recordStartedAt = 0;
  let lastDownloadUrl = null;
  let streamReady = false;

  function markReady() {
    if (streamReady) return;
    streamReady = true;
    canvas.width = img.naturalWidth || 400;
    canvas.height = img.naturalHeight || 300;
    resolutionText.textContent = `${canvas.width} × ${canvas.height}`;
    stateText.textContent = '画面已连接';
    if (typeof MediaRecorder !== 'undefined' && typeof canvas.captureStream === 'function') {
      startButton.disabled = false;
    } else {
      stateText.textContent = '画面已连接，但当前浏览器不支持录制';
    }
  }

  img.addEventListener('error', () => {
    stateText.textContent = '画面连接失败，请刷新页面';
  });

  // 多路 JPEG（multipart）流在不同浏览器里 load 事件触发不一致，改用轮询 naturalWidth 判断首帧是否到达。
  (function waitForFrame() {
    if (streamReady) return;
    if (img.naturalWidth > 0 && img.naturalHeight > 0) {
      markReady();
    } else {
      setTimeout(waitForFrame, 200);
    }
  })();

  function drawLoop() {
    if (canvas.width && canvas.height) {
      ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
    }
    drawHandle = requestAnimationFrame(drawLoop);
  }

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
    if (!streamReady || typeof MediaRecorder === 'undefined' || typeof canvas.captureStream !== 'function') {
      stateText.textContent = '当前浏览器不支持视频录制';
      return;
    }

    chunks = [];
    downloadLink.style.display = 'none';
    if (lastDownloadUrl) URL.revokeObjectURL(lastDownloadUrl);

    drawLoop();
    const captureStream = canvas.captureStream(30);
    const mimeType = chooseMimeType();
    mediaRecorder = new MediaRecorder(captureStream, mimeType ? {mimeType} : undefined);
    mediaRecorder.ondataavailable = event => {
      if (event.data && event.data.size) chunks.push(event.data);
    };
    mediaRecorder.onerror = event => {
      stateText.textContent = `录制失败：${event.error?.message || '未知错误'}`;
    };
    mediaRecorder.onstop = () => {
      clearInterval(timerHandle);
      cancelAnimationFrame(drawHandle);
      const finalType = mediaRecorder.mimeType || mimeType || 'video/webm';
      const blob = new Blob(chunks, {type: finalType});
      lastDownloadUrl = URL.createObjectURL(blob);
      const extension = finalType.includes('mp4') ? 'mp4' : 'webm';
      const stamp = new Date().toISOString().replace(/[:.]/g, '-');
      downloadLink.href = lastDownloadUrl;
      downloadLink.download = `maixcam-pro-${stamp}.${extension}`;
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
</html>"""


def start_wifi_ap(config):
    wifi = network.wifi.Wifi()
    result = wifi.start_ap(
        ssid=config["ap_ssid"],
        password=config["ap_password"],
        mode="g",
        channel=config["ap_channel"],
        ip=config["ap_ip"],
        netmask=config["ap_netmask"],
        hidden=False,
    )
    err.check_raise(result, "Failed to start WiFi AP")
    return wifi, config["ap_ip"]


def show_connection_info(config, url):
    # 独立于摄像头画面的Image对象，只显示在设备屏幕上，不会混进推流内容里
    disp = display.Display()
    info_img = image.Image(disp.width(), disp.height())
    info_img.clear()
    scale = config["screen_url_scale"]
    info_img.draw_string(10, 10, "WiFi: " + config["ap_ssid"], image.COLOR_WHITE, scale=scale)
    info_img.draw_string(10, 50, "Pass: " + config["ap_password"], image.COLOR_WHITE, scale=scale)
    info_img.draw_string(10, 90, url, image.COLOR_WHITE, scale=scale)
    disp.show(info_img)
    return disp


wifi, ip = start_wifi_ap(CONFIG)

cam = camera.Camera(CONFIG["width"], CONFIG["height"])
stream = http.JpegStreamer()
stream.set_html(HTML_PAGE)
stream.start()

page_url = "http://{}:{}".format(ip, stream.port())
print("Open:", page_url)

disp = show_connection_info(CONFIG, page_url)

while not app.need_exit():
    img = cam.read()
    jpg = img.to_jpeg(CONFIG["jpeg_quality"])
    stream.write(jpg)
