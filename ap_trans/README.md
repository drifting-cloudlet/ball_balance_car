# MaixCAM Pro 视频推流（HTTP/JPEG + 浏览器录制）

最初按 `F:\code\maxicam\r\main.py`（MaixCAM2 工程，WebRTC 方案）等功能移植，**真机验证后发现 WebRTC 在这台 MaixCAM Pro 上不可用，已改用 HTTP/JPEG 推流**，做法对齐 `F:\code\maxicam\trans\main.py`（MaixCAM2 的 HTTP/JPEG 推流实现）。

## 变更记录（保留过程，方便追溯）

1. **第一版（WebRTC）**：逐行核对 MaixCDK 源码，`maixcam`(Pro同芯片) 和 `maixcam2` 两个 port 都有 `maix_webrtc_*.cpp` 实现，头文件签名一致，据此判断可以直接移植。这是**静态源码比对**的结论（R1/R2），当时没有真机证据。
2. **真机报错**：在真实 MaixCAM Pro 上运行，第一行 `from maix import ... webrtc ...` 就报错：
   ```
   ImportError: cannot import name 'webrtc' from 'maix'
   ```
   说明这台设备当前安装的 MaixPy **没有编译进 webrtc 这个 Python 子模块**，哪怕 C++ 源码仓库里有对应 port 实现——源码存在不等于这台设备的固件里编译/装了它，静态比对的结论在这里被真机证据推翻了。
3. **用探测脚本 `probe_stream_modules.py` 拿到确切证据**（`F:\code\maxicam\maxicam1pro\probe_stream_modules.py`，可重复运行）：
   ```
   device_id: maixcam_pro
   os_version: maixcam-pro-2025-03-19-maixpy-v4.10.3
   maixpy_version: 4.10.3
   runtime_version: 1.22.0
   module_webrtc: missing（无法导入）
   module_rtsp: present
   module_rtmp: present
   module_http_jpeg_streamer: present
   ```
4. **改用 HTTP/JPEG**：`rtsp`/`rtmp` 虽然也在，但浏览器不能直接播放（要装 VLC 之类播放器或额外网关），`trans/README.md` 之前已经就这三种方案分析过一次、选了 HTTP/JPEG，这次真机证据又把 webrtc 也排除了，剩下 HTTP/JPEG 是唯一"浏览器直接能看、不用装任何东西"的选项，和 `trans/main.py` 的做法完全一致（`http.JpegStreamer` + `set_html/start/write`），已确认这台设备上 `http.JpegStreamer` 可以正常导入。
5. **查清 webrtc 缺失原因**：MaixPy 源码里 webrtc 框架 2025-12-08 才合入（`maix_webrtc.hpp` 及两个 port 文件的 `@update` 注释都写着这个日期），官方文档 2025-12-11 发布 v1.0.0，并在 `docs/doc/zh/projects/README.md:80` 明确写"WebRTC 推流"支持 `MaixCAM`、`MaixCAM Pro`、`MaixCAM2` 三个平台，Pro 的 C++ 实现（`port/maixcam/maix_webrtc_maixcam.cpp`，743行）比 MaixCAM2 那份（490行）还完整。这台设备固件是 `maixcam-pro-2025-03-19`，比合入时间早 9 个月——**结论是固件太旧，不是平台不支持**，升级固件大概率能恢复 webrtc。
6. **决定不升级固件**：升级能换回原生 WebRTC 录制，但刷机要花时间、有让设备其他已装内容/配置出问题的风险，与你确认后决定先不刷机，改为**给现有 HTTP/JPEG 方案用 canvas workaround 补上录制功能**（见下节），需要更完整的 WebRTC 版录制体验时，可以再回头考虑升级固件。
7. **设备屏幕显示网址 + 分辨率/画质调整**：设备自己的屏幕（640x480）现在会用 `display.Display()` 单独画一张纯文字的图（黑底白字，`image.Image(disp.width(), disp.height(), bg=image.COLOR_BLACK)` + `draw_string`），只显示一次连接地址，和摄像头画面完全是两个独立的 `Image` 对象——摄像头那份只经过 `cam.read() -> img.to_jpeg() -> stream.write()`，没有叠加任何文字，所以推流出去的画面是干净的。这个写法是照抄 `trans/main.py` 已经在 MaixCAM2 真机验证过的模式（差异：那边用 `psutil` 枚举多网卡 IP，这里已经从 `wifi.get_ip()` 拿到唯一确定的地址，不需要再枚举）。另外把分辨率从 640x480 提到 1280x720，`to_jpeg()` 的 `quality` 参数从默认 95 先降到 60 —— 这个参数和取值范围是在 `MaixCDK\components\vision\include\maix_image.hpp:423` 查到的：`to_jpeg(int quality = 95, ...)`，官方注释写明"For MaixCAM supported range is (50, 100], if <= 50 will be fixed to 51"。
8. **反馈：640x480+默认quality实测只有约15fps，太低**。你明确表态"我要的是提升分辨率"——分辨率保留 1280x720（约 3 倍像素），`jpeg_quality` 从 60 继续压到文档给出的下限 51（低于51会被固定成51，已经是这个参数能让出的最大空间）。但要说清楚一个物理限制：`to_jpeg()` 的编码耗时主要跟像素数量（也就是分辨率）挂钩，quality 主要影响压缩后的体积（进而影响 `stream.write()` 通过 WiFi 传输的耗时），对编码本身速度的影响有限——所以 1280x720 即使 quality 压到底，fps 大概率还是会比 640x480 时更低，这是分辨率提升必然要付出的代价，不是没调好参数。如果实测 fps 掉到不可接受，下一步只能是分辨率往回收一点（比如 960x540 折中），quality 这一侧已经没有余量了。
9. **真机报错，同一类"固件较旧、参数比文档少"的问题**：`image.Image(disp.width(), disp.height(), bg=image.COLOR_BLACK)` 报 `TypeError: incompatible constructor arguments`，设备实际报出的可用签名是 `Image(width, height, format=FMT_RGB888)`，没有 `bg` 这个参数——`bg` 是比这台设备固件更新的版本才加的（`trans/main.py` 在 MaixCAM2 上能用，因为那边固件更新）。改成不传 `bg`，构造后显式调用 `info_img.clear()`（`maix_image.hpp:679`"Sets all pixels in the image to zero"）来清成黑色背景，这个方法更基础，不依赖后加的参数。另外从这次日志观察到：`disp.width()/height()` 实际返回的是 480x640（竖屏方向的缓冲区，不是我之前查到的"2.4寸640x480"横屏描述），代码里已经是动态取 `disp.width()/disp.height()`，不影响功能，只是提醒一下实际方向和预期可能不同。
10. **反馈：1280x720 延迟太高，要求降分辨率**。至此两个数据点都有了：640x480（quality默认95左右）约15fps，嫌低；1280x720（quality已压到51下限）延迟明显更高，嫌高。说明 640x480 这个基线本身已经接近这台设备软件JPEG编码的上限，简单的"折中"未必能同时满足"更清晰"和"更流畅"。改成 800x600：保持4:3（和 640x480 同比例，不再是 1280x720 那次顺带引入的16:9），像素只比 640x480 多56%（1280x720 是3倍），比 960x540 更保守，多留一点命中"可接受"区间的机会——但基于已有的两个数据点，**800x600也有可能仍然不够快**，如果还是不满意，下一步大概率要退回640x480甚至更低，画质这边已经没有下调空间了。
11. **反馈：800x600依然卡，要求继续降分辨率；串口不需要每帧打印**。三个数据点连起来看（640x480约15fps、1280x720更卡、800x600依然卡），说明这台设备软件JPEG编码撑不住 640x480 往上的任何分辨率，不是"再试一个折中值"能解决的，这次直接退到 **320x240**——这不是新猜的数，是 `trans/main.py` 里 MaixCAM2 那份同样用 `to_jpeg()+JpegStreamer` 方案、已经在用的分辨率，复用已验证过的值。控制台 `time/fps` 打印当时改成了按 `log_interval_ms`（1000ms）节流，而不是彻底去掉。
12. **反馈：320x240流畅可用，但要求彻底去掉fps/延迟打印，分辨率再小幅提一点**。上一版的"节流"没理解到位，用户要的是完全不打印，不是降低频率——已把主循环里的 `t`/`dt`/`last_log_ms`/`print(time/fps...)` 全部删掉，`time` 这个 import 也因此没用了一起删掉。分辨率在 320x240 流畅的基础上小幅提到 **400x300**（4:3同比例，像素只多56%，远小于之前640x480那次4倍的跨度），具体是否依然流畅还是要看真机体感。
13. **反馈：串口打印内容还是太多，要求精简**。去掉了 `connect_wifi()` 里的 `Connecting to WiFi:` / `WiFi connected` / `MaixCAM Pro IP:` 三行，以及 `Use the green and red buttons...` 提示行，只保留最后一行 `Open: <url>`（唯一真正需要操作的信息）。**需要说明一下边界**：真机日志里那一大堆 `[SAMPLE_COMM_SNS_ParseIni]`、ISP/VI/VO 初始化、`wpa_supplicant`/`udhcpc` 之类的输出，是底层 C 驱动/系统工具自己打的，不是 Python 脚本里的 `print()`，这部分我这边没有能操作的接口，如果这些也算"太多"，需要的话我再去查有没有能关掉这些底层日志的配置项，目前还没找。
14. **优化代码结构，删除多余注释**：`CONFIG` 里分辨率/画质那几条注释之前写成了"调试日记"（640x480多少fps、1280x720/800x600试了不行……这些历史过程），这类"引用当前调试过程"的注释会在参数以后再变的时候过时、读着还费解，已经精简成只描述**当前仍然成立的结论**（"这台设备软件JPEG编码吃CPU，分辨率一超过640x480就明显卡顿"），完整的调试过程还是保留在这份 README 的"变更记录"里，不会丢。其余几处注释（JS里multipart流load事件不可靠、display图像和推流图像是两个独立对象）判断是仍然非显而易见的"为什么"，保留。
15. **改为 MaixCAM Pro 自建热点**：不再连接手机热点或外部路由器，改用 `network.wifi.Wifi.start_ap()` 创建 `MaixCAM-Pro` 热点，设备地址固定为 `192.168.66.1`。Pad 直接连接该热点后访问设备屏幕显示的网址即可，比赛现场不需要互联网或第三方网络设备。

## 和 WebRTC 版本相比，功能上的区别

- 网络改为 MaixCAM Pro 自建 2.4GHz WiFi 热点，Pad 直接连接设备，不依赖手机热点、外部路由器或互联网。
- **录制方式不同，但功能保留了**：WebRTC 原本能直接把 `<video>` 的原生 `MediaStream` 交给 `MediaRecorder` 录；HTTP/JPEG 只是一张不断刷新的 `<img>`，没有现成的 `MediaStream`。现在的做法是：把 `<img>` 隐藏画到一个 `<canvas>` 上（`requestAnimationFrame` 循环 + `drawImage`），再用 `canvas.captureStream(20)` 转出一个 `MediaStream` 交给 `MediaRecorder`。效果上和原版一样有"开始录制/停止并保存/下载"，但多了一层转换：帧率固定按 20fps 采样、比原生 WebRTC 录制多一点延迟和画质损耗，且依赖浏览器支持 `HTMLCanvasElement.captureStream`（较新版本 Chrome/Firefox/Edge 都支持，旧浏览器可能不支持，页面里加了检测，不支持会提示而不是静默失败）。
- 首帧就绪的判断也不同：WebRTC 版靠 `pc.ontrack`；这版因为 multipart JPEG 流在各浏览器里 `<img>` 的 `load` 事件触发时机不一致，改成轮询 `img.naturalWidth` 判断首帧是否到达，更稳。
- 分辨率从 640x480 提到 1280x720（MaixCAM Pro 摄像头上限 2560x1440，够用），配合把 JPEG 质量从默认 95 压到文档允许的下限 51 换取一点编码/传输速度，但分辨率提升本身必然拉低 fps，quality 只能部分对冲，具体够不够流畅要看真机 fps 打印。
- 新增：设备自己的屏幕只显示连接网址（黑底白字，只画一次），推流出去的画面不受影响、不带任何文字叠加。

## 尚待确认

- `MaixCAM开发任务清单.md` 里 MaixCAM Pro 的定位是"接收端"（拉流显示、本地录制、回放），这个文件目前做的还是"自己开摄像头往外推流"（跟 MaixCAM2 同角色），跟任务清单的定位不完全一致，前几版 README 就提过这点，还没有得到你的确认。

## 运行方式

1. 先跑 `probe_stream_modules.py` 确认设备上有哪些推流模块可用（如果固件没变过，可以跳过）。
2. 用 MaixVision 打开 `main.py` 直接运行（单文件，无同级 import），设备会创建 `MaixCAM-Pro` 热点，固定地址为 `192.168.66.1`。
3. 在 Pad 的 WiFi 设置中连接设备屏幕显示的热点；如果系统提示"无互联网"，选择继续保持连接。
4. 用较新版 Chrome 打开设备屏幕显示的 `http://192.168.66.1:<端口>` 地址即可查看画面。
5. 画面连接后"开始录制"按钮会变亮，点一下开始录制、再点"停止并保存"会出现下载链接（文件名 `maixcam-pro-<时间戳>.webm` 或 `.mp4`，取决于浏览器支持的编码）。
6. 控制台不再打印每帧的 `time`/`fps`（应用户要求已删除），流畅与否靠实际看画面判断。

## 待验证（都还没有真机跑过，麻烦你实跑后反馈）

- 400x300 + quality 51 是否依然流畅（在 320x240 已确认流畅的基础上小幅提了一点分辨率）。
- `start_ap()` 在当前设备固件上能否正常创建热点，Pad 能否获得局域网地址并访问 `192.168.66.1`。
- 设备屏幕上的网址文字是否正常显示、字号（`screen_url_scale=1.5`）合不合适、有没有被截断。
- canvas 录制功能：点"开始录制/停止并保存"，确认能正常下载、视频内容和时长对得上。
- 串口打印是否降到了可接受的频率（现在约1秒一条，而不是每帧一条）。
